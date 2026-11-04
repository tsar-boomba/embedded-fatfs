//! Reading and seeking through the cluster map, on fragmented and contiguous files across FAT types
//! and cluster sizes, compared against the data written.

use std::cell::Cell;
use std::rc::Rc;

use embedded_fatfs::{ChronoTimeProvider, FatType, FileSystem, FormatVolumeOptions, FsOptions, LossyOemCpConverter};
use embedded_io_async::{ErrorType, Read, Seek, SeekFrom, Write};

/// An in-memory disk that counts reads.
struct MemDisk {
    data: Vec<u8>,
    pos: usize,
    reads: Rc<Cell<usize>>,
}

impl ErrorType for MemDisk {
    type Error = std::io::Error;
}

impl Read for MemDisk {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.reads.set(self.reads.get() + 1);
        let n = buf.len().min(self.data.len() - self.pos);
        buf[..n].copy_from_slice(&self.data[self.pos..self.pos + n]);
        self.pos += n;
        Ok(n)
    }
}

impl Write for MemDisk {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let n = buf.len().min(self.data.len() - self.pos);
        self.data[self.pos..self.pos + n].copy_from_slice(&buf[..n]);
        self.pos += n;
        Ok(n)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl Seek for MemDisk {
    async fn seek(&mut self, pos: SeekFrom) -> Result<u64, Self::Error> {
        self.pos = match pos {
            SeekFrom::Start(p) => p as usize,
            SeekFrom::End(p) => (self.data.len() as i64 + p) as usize,
            SeekFrom::Current(p) => (self.pos as i64 + p) as usize,
        };
        Ok(self.pos as u64)
    }
}

type Fs = FileSystem<MemDisk, ChronoTimeProvider, LossyOemCpConverter>;

/// Deterministic test data that differs at every offset
fn pattern(len: usize, seed: u32) -> Vec<u8> {
    let mut x = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
    (0..len)
        .map(|_| {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x as u8
        })
        .collect()
}

/// Formats a volume and writes `data` to "song" in chunks of `chunk_clusters` clusters, writing a
/// cluster of another file between chunks to fragment it (no fragmentation if `chunk_clusters` is
/// 0). Returns the filesystem and the disk read counter.
async fn make_fs(fat: FatType, cluster: u32, data: &[u8], chunk_clusters: usize) -> (Fs, Rc<Cell<usize>>) {
    let sectors: u32 = match fat {
        FatType::Fat32 => 70_000 * (cluster / 512) + 8192,
        _ => 20_000 * (cluster / 512).max(1),
    };
    let reads = Rc::new(Cell::new(0));
    let mut disk = MemDisk {
        data: vec![0; sectors as usize * 512],
        pos: 0,
        reads: reads.clone(),
    };
    embedded_fatfs::format_volume(
        &mut disk,
        FormatVolumeOptions::new()
            .fat_type(fat)
            .bytes_per_cluster(cluster)
            .total_sectors(sectors),
    )
    .await
    .unwrap();
    disk.pos = 0;
    let fs = FileSystem::new(disk, FsOptions::new().update_accessed_date(false))
        .await
        .unwrap();
    {
        let root = fs.root_dir();
        let mut song = root.create_file("song").await.unwrap();
        if chunk_clusters == 0 {
            song.write_all(data).await.unwrap();
        } else {
            let mut other = root.create_file("other").await.unwrap();
            let filler = vec![0xAA; cluster as usize];
            for chunk in data.chunks(chunk_clusters * cluster as usize) {
                song.write_all(chunk).await.unwrap();
                song.flush().await.unwrap();
                other.write_all(&filler).await.unwrap();
                other.flush().await.unwrap();
            }
        }
        song.flush().await.unwrap();
    }
    (fs, reads)
}

async fn read_all<R: Read>(r: &mut R, buf_len: usize) -> Vec<u8> {
    let mut out = Vec::new();
    let mut buf = vec![0; buf_len];
    loop {
        let n = r.read(&mut buf).await.unwrap();
        if n == 0 {
            return out;
        }
        out.extend_from_slice(&buf[..n]);
    }
}

async fn check_reads_and_seeks(fat: FatType, cluster: u32, chunk_clusters: usize) {
    let len = 300 * cluster as usize + 123;
    let data = pattern(len, cluster ^ chunk_clusters as u32);
    let (fs, _) = make_fs(fat, cluster, &data, chunk_clusters).await;
    let root = fs.root_dir();
    let what = format!("{fat:?}, {cluster} B clusters, chunks of {chunk_clusters}");

    // Sequential reads with buffers smaller than, equal to, and spanning clusters
    for buf_len in [100, 512, cluster as usize, 3 * cluster as usize + 7, 64 * 1024] {
        let mut file = root.open_file("song").await.unwrap();
        assert!(
            read_all(&mut file, buf_len).await == data,
            "{what}: sequential read, buffer {buf_len}"
        );
    }

    // Seeks in every direction, including to cluster boundaries and the ends
    let cs = cluster as u64;
    let mut file = root.open_file("song").await.unwrap();
    let targets = [
        SeekFrom::Start(5 * cs + 10),
        SeekFrom::Start(250 * cs),
        SeekFrom::Start(cs),
        SeekFrom::Start(cs - 1),
        SeekFrom::Current(40 * cs as i64),
        SeekFrom::Current(-(20 * cs as i64) - 3),
        SeekFrom::End(-100),
        SeekFrom::Start(0),
        SeekFrom::Start(299 * cs),
        SeekFrom::End(0),
        SeekFrom::Start(123 * cs + 456 % cs),
    ];
    for target in targets {
        let pos = file.seek(target).await.unwrap() as usize;
        let mut buf = vec![0; 2 * cluster as usize + 5];
        let mut got = Vec::new();
        while got.len() < buf.len() {
            let want = buf.len() - got.len();
            let n = file.read(&mut buf[..want]).await.unwrap();
            if n == 0 {
                break;
            }
            got.extend_from_slice(&buf[..n]);
        }
        let end = (pos + buf.len()).min(len);
        assert!(got == data[pos..end], "{what}: read after seek to {target:?} ({pos})");
        // Position agrees with what was read
        assert_eq!(file.stream_position().await.unwrap() as usize, end, "{what}: position");
    }

    // A clone keeps working from the same position
    file.seek(SeekFrom::Start(7 * cs)).await.unwrap();
    let mut clone = file.clone();
    assert!(
        read_all(&mut clone, 1000).await == data[7 * cs as usize..],
        "{what}: clone"
    );
}

#[tokio::test]
async fn reads_and_seeks_match_the_data() {
    for (fat, cluster) in [
        (FatType::Fat12, 512),
        (FatType::Fat16, 512),
        (FatType::Fat16, 2048),
        (FatType::Fat32, 512),
        (FatType::Fat32, 4096),
    ] {
        // Contiguous, every cluster in its own run, and a few clusters per run
        for chunk_clusters in [0, 1, 3] {
            check_reads_and_seeks(fat, cluster, chunk_clusters).await;
        }
    }
}

#[tokio::test]
async fn more_runs_than_the_map_holds() {
    // Every cluster is its own run, so the map fills up and the rest of the chain is followed
    // without it
    let cluster = 512;
    let data = pattern(1000 * cluster, 99);
    let (fs, _) = make_fs(FatType::Fat32, cluster as u32, &data, 1).await;
    let root = fs.root_dir();
    let mut file = root.open_file("song").await.unwrap();
    assert!(read_all(&mut file, 3000).await == data);
    for pos in [900 * cluster + 3, 100 * cluster, 999 * cluster, 400 * cluster + 1, 0] {
        file.seek(SeekFrom::Start(pos as u64)).await.unwrap();
        let mut buf = vec![0; 1500];
        let n = file.read(&mut buf).await.unwrap();
        assert!(n > 0);
        assert!(buf[..n] == data[pos..pos + n], "seek to {pos}");
    }
}

#[tokio::test]
async fn contiguous_file_reads_span_clusters() {
    // With 512 B clusters, a 64 KiB read of a contiguous file should be one disk read, not 128
    let cluster = 512;
    let data = pattern(400 * cluster, 5);
    let (fs, reads) = make_fs(FatType::Fat32, cluster as u32, &data, 0).await;
    let root = fs.root_dir();
    let mut file = root.open_file("song").await.unwrap();
    let mut buf = vec![0; 64 * 1024];
    // The first read maps the chain
    file.read(&mut buf).await.unwrap();
    let before = reads.get();
    let n = file.read(&mut buf).await.unwrap();
    assert_eq!(n, buf.len());
    assert!(buf[..] == data[64 * 1024..128 * 1024]);
    assert_eq!(reads.get() - before, 1, "disk reads for one 64 KiB file read");

    // Seeking back to mapped clusters needs no FAT reads: just the data read
    file.seek(SeekFrom::Start(10 * cluster as u64)).await.unwrap();
    let before = reads.get();
    file.read(&mut buf[..cluster]).await.unwrap();
    assert_eq!(reads.get() - before, 1, "disk reads for a read after seeking back");
}

#[tokio::test]
async fn truncate_and_rewrite_after_mapping() {
    let cluster = 512;
    let data = pattern(50 * cluster, 7);
    let (fs, _) = make_fs(FatType::Fat16, cluster as u32, &data, 2).await;
    let root = fs.root_dir();
    let mut file = root.open_file("song").await.unwrap();
    // Map the whole chain
    assert!(read_all(&mut file, 4096).await == data);

    // Cut it in the middle of a cluster, then write new data past the old end. The freed
    // clusters get reused by the other file first, so the new chain differs from the old one.
    let cut = 20 * cluster + 100;
    file.seek(SeekFrom::Start(cut as u64)).await.unwrap();
    file.truncate().await.unwrap();
    {
        let mut other = root.open_file("other").await.unwrap();
        other.seek(SeekFrom::End(0)).await.unwrap();
        other.write_all(&vec![0x55; 10 * cluster]).await.unwrap();
        other.flush().await.unwrap();
    }
    let tail = pattern(40 * cluster, 8);
    file.write_all(&tail).await.unwrap();
    file.flush().await.unwrap();

    let mut expected = data[..cut].to_vec();
    expected.extend_from_slice(&tail);
    file.seek(SeekFrom::Start(0)).await.unwrap();
    assert!(read_all(&mut file, 1000).await == expected, "same handle");
    let mut reopened = root.open_file("song").await.unwrap();
    assert!(read_all(&mut reopened, 1000).await == expected, "reopened");

    // Truncating to nothing and writing again
    file.seek(SeekFrom::Start(0)).await.unwrap();
    file.truncate().await.unwrap();
    file.write_all(&tail[..3 * cluster]).await.unwrap();
    file.flush().await.unwrap();
    file.seek(SeekFrom::Start(0)).await.unwrap();
    assert!(
        read_all(&mut file, 700).await == tail[..3 * cluster],
        "after truncating to 0"
    );
}

#[tokio::test]
async fn fat_reads_are_cached() {
    // Every cluster is its own run and there are more runs than the map holds, so reading follows
    // the FAT for each cluster. The FAT entries are adjacent, so a cached FAT block serves many of
    // them: there should be about one disk read per cluster (its data), not two.
    let cluster = 512;
    let clusters = 1000;
    let data = pattern(clusters * cluster, 11);
    let (fs, reads) = make_fs(FatType::Fat32, cluster as u32, &data, 1).await;
    let root = fs.root_dir();
    let mut file = root.open_file("song").await.unwrap();
    let before = reads.get();
    assert!(read_all(&mut file, cluster).await == data);
    let disk_reads = reads.get() - before;
    assert!(
        disk_reads < clusters + clusters / 10,
        "{disk_reads} disk reads for {clusters} clusters"
    );
}
