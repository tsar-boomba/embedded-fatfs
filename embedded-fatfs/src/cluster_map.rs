//! A cache of where a file's clusters are on disk.
//!
//! Finding the disk cluster that holds a given part of a file means following the file's cluster
//! chain through the FAT, one entry per cluster, from the start of the file. [`ClusterMap`]
//! remembers every part of the chain that has been followed, as runs of consecutive clusters, so
//! that each part only has to be followed once:
//!
//! - Seeking anywhere in the mapped part of a file needs no FAT reads at all. A file written in
//!   one go to a volume with free space usually has its whole chain in one run.
//! - Reads can continue past cluster boundaries within a run, so a single read of the underlying
//!   device can cover many clusters even when the clusters are small.
//!
//! Only a prefix of the chain is mapped: clusters are recorded in order from the first one. The
//! map holds at most [`MAX_RUNS`] runs, so a heavily fragmented file can't use an unbounded
//! amount of memory. Past that, only the last cluster found is remembered, so following the chain
//! costs at most what it did without a map: one FAT entry per cluster crossed.
//!
//! Without the `alloc` feature only that last cluster is remembered.

/// Most runs a single map holds (12 bytes each).
#[cfg(feature = "alloc")]
pub(crate) const MAX_RUNS: usize = 256;

/// Consecutive clusters of a file that are also consecutive on disk.
#[cfg(feature = "alloc")]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Run {
    /// Index of the first cluster in the file
    file_cluster: u32,
    /// Disk cluster holding `file_cluster`
    disk_cluster: u32,
    /// Number of clusters
    len: u32,
}

#[cfg(feature = "alloc")]
impl Run {
    fn file_end(&self) -> u32 {
        self.file_cluster + self.len
    }
}

/// The mapped prefix of a file's cluster chain. See the [module docs](self).
#[derive(Clone, Debug, Default)]
pub(crate) struct ClusterMap {
    /// Sorted, non-overlapping, and covering file clusters `0..known()` with no gaps
    #[cfg(feature = "alloc")]
    runs: alloc::vec::Vec<Run>,
    /// The last cluster found past the mapped prefix, as (file cluster index, disk cluster), so
    /// the chain can be followed on from there instead of being followed again. Used when the map
    /// is full, or always without the `alloc` feature.
    beyond: Option<(u32, u32)>,
}

impl ClusterMap {
    /// Number of file clusters mapped, starting from the first.
    pub(crate) fn known(&self) -> u32 {
        self.prefix_known()
    }

    /// The disk cluster holding file cluster `index`, and how many clusters from there on are
    /// known to be consecutive on disk (at least 1).
    pub(crate) fn get(&self, index: u32) -> Option<(u32, u32)> {
        self.prefix_get(index)
            .or_else(|| self.beyond.filter(|b| b.0 == index).map(|b| (b.1, 1)))
    }

    /// The known cluster closest before or at file cluster `index`, as (file cluster index, disk
    /// cluster), to start following the chain from.
    pub(crate) fn nearest(&self, index: u32) -> Option<(u32, u32)> {
        if let Some((cluster, _)) = self.prefix_get(index) {
            return Some((index, cluster));
        }
        let mut best = self.prefix_last().filter(|l| l.0 <= index);
        if let Some(b) = self.beyond {
            if b.0 <= index && best.map_or(true, |l| b.0 > l.0) {
                best = Some(b);
            }
        }
        best
    }

    /// Records that file cluster `index` is at `disk_cluster`. It is added to the map if it is the
    /// next unmapped cluster and the map has room for it, and otherwise remembered as the last
    /// cluster found past the map.
    pub(crate) fn record(&mut self, index: u32, disk_cluster: u32) {
        if !self.prefix_record(index, disk_cluster) && index >= self.known() {
            self.beyond = Some((index, disk_cluster));
        }
    }

    /// Forgets everything from file cluster `len` on.
    pub(crate) fn truncate(&mut self, len: u32) {
        self.prefix_truncate(len);
        if self.beyond.is_some_and(|b| b.0 >= len) {
            self.beyond = None;
        }
    }
}

#[cfg(feature = "alloc")]
impl ClusterMap {
    fn prefix_known(&self) -> u32 {
        self.runs.last().map_or(0, Run::file_end)
    }

    fn prefix_get(&self, index: u32) -> Option<(u32, u32)> {
        let i = self.runs.partition_point(|r| r.file_end() <= index);
        let run = self.runs.get(i)?;
        let skip = index.checked_sub(run.file_cluster)?;
        Some((run.disk_cluster + skip, run.len - skip))
    }

    fn prefix_last(&self) -> Option<(u32, u32)> {
        self.runs.last().map(|r| (r.file_end() - 1, r.disk_cluster + r.len - 1))
    }

    /// Adds a cluster to the prefix if it is the next one and there is room. Returns whether the
    /// prefix now holds it.
    fn prefix_record(&mut self, index: u32, disk_cluster: u32) -> bool {
        let known = self.prefix_known();
        if index < known {
            return true;
        }
        if index > known {
            return false;
        }
        if let Some(last) = self.runs.last_mut() {
            if last.disk_cluster.checked_add(last.len) == Some(disk_cluster) {
                last.len += 1;
                return true;
            }
        }
        if self.runs.len() < MAX_RUNS {
            self.runs.push(Run {
                file_cluster: index,
                disk_cluster,
                len: 1,
            });
            return true;
        }
        false
    }

    fn prefix_truncate(&mut self, len: u32) {
        let keep = self.runs.partition_point(|r| r.file_cluster < len);
        self.runs.truncate(keep);
        if let Some(last) = self.runs.last_mut() {
            last.len = last.len.min(len - last.file_cluster);
        }
    }

    /// Number of runs, for tests.
    #[cfg(test)]
    fn runs(&self) -> usize {
        self.runs.len()
    }
}

#[cfg(not(feature = "alloc"))]
impl ClusterMap {
    fn prefix_known(&self) -> u32 {
        0
    }

    fn prefix_get(&self, _index: u32) -> Option<(u32, u32)> {
        None
    }

    fn prefix_last(&self) -> Option<(u32, u32)> {
        None
    }

    fn prefix_record(&mut self, _index: u32, _disk_cluster: u32) -> bool {
        false
    }

    fn prefix_truncate(&mut self, _len: u32) {}
}

#[cfg(all(test, feature = "alloc"))]
mod tests {
    use super::*;

    fn map_of(chain: &[u32]) -> ClusterMap {
        let mut map = ClusterMap::default();
        for (i, &c) in chain.iter().enumerate() {
            map.record(i as u32, c);
        }
        map
    }

    #[test]
    fn consecutive_clusters_share_a_run() {
        let map = map_of(&[10, 11, 12, 13]);
        assert_eq!(map.runs(), 1);
        assert_eq!(map.known(), 4);
        assert_eq!(map.get(0), Some((10, 4)));
        assert_eq!(map.get(2), Some((12, 2)));
        assert_eq!(map.get(4), None);
        assert_eq!(map.nearest(u32::MAX), Some((3, 13)));
    }

    #[test]
    fn fragmented_chain() {
        // Runs 10..12, 50, 20..22
        let map = map_of(&[10, 11, 50, 20, 21]);
        assert_eq!(map.runs(), 3);
        assert_eq!(map.get(0), Some((10, 2)));
        assert_eq!(map.get(1), Some((11, 1)));
        assert_eq!(map.get(2), Some((50, 1)));
        assert_eq!(map.get(3), Some((20, 2)));
        assert_eq!(map.get(4), Some((21, 1)));
        assert_eq!(map.get(5), None);
    }

    #[test]
    fn only_the_next_cluster_is_recorded() {
        let mut map = map_of(&[10, 11]);
        // Already known, and a gap
        map.record(0, 99);
        map.record(5, 99);
        assert_eq!(map.known(), 2);
        assert_eq!(map.get(0), Some((10, 2)));
        map.record(2, 12);
        assert_eq!(map.get(0), Some((10, 3)));
    }

    #[test]
    fn full_map_still_extends_its_last_run() {
        let chain: alloc::vec::Vec<u32> = (0..MAX_RUNS as u32).map(|i| i * 2).collect();
        let mut map = map_of(&chain);
        assert_eq!(map.runs(), MAX_RUNS);
        let last = chain[MAX_RUNS - 1];
        // Continues the last run
        map.record(MAX_RUNS as u32, last + 1);
        assert_eq!(map.known(), MAX_RUNS as u32 + 1);
        // Needs a new run: remembered as the last cluster found
        map.record(MAX_RUNS as u32 + 1, 5000);
        assert_eq!(map.known(), MAX_RUNS as u32 + 1);
        assert_eq!(map.runs(), MAX_RUNS);
        assert_eq!(map.get(MAX_RUNS as u32 + 1), Some((5000, 1)));
        assert_eq!(map.nearest(MAX_RUNS as u32 + 10), Some((MAX_RUNS as u32 + 1, 5000)));
        // Before it, the end of the prefix is nearest
        assert_eq!(map.nearest(MAX_RUNS as u32), Some((MAX_RUNS as u32, last + 1)));
        map.record(MAX_RUNS as u32 + 2, 6000);
        assert_eq!(map.get(MAX_RUNS as u32 + 1), None);
        assert_eq!(map.get(MAX_RUNS as u32 + 2), Some((6000, 1)));
        map.truncate(MAX_RUNS as u32 + 2);
        assert_eq!(map.get(MAX_RUNS as u32 + 2), None);
    }

    #[test]
    fn nearest() {
        let map = map_of(&[10, 11, 50]);
        assert_eq!(map.nearest(1), Some((1, 11)));
        assert_eq!(map.nearest(2), Some((2, 50)));
        assert_eq!(map.nearest(9), Some((2, 50)));
        assert_eq!(ClusterMap::default().nearest(0), None);
    }

    #[test]
    fn truncate() {
        let mut map = map_of(&[10, 11, 12, 50, 51]);
        map.truncate(4);
        assert_eq!(map.known(), 4);
        assert_eq!(map.get(3), Some((50, 1)));
        map.truncate(2);
        assert_eq!(map.known(), 2);
        assert_eq!(map.get(0), Some((10, 2)));
        assert_eq!(map.runs(), 1);
        map.truncate(0);
        assert_eq!(map.known(), 0);
        assert_eq!(map.get(0), None);
        // Mapping again from scratch
        map.record(0, 7);
        assert_eq!(map.get(0), Some((7, 1)));
    }
}
