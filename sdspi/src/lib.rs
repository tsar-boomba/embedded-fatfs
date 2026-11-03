//! A crate for interfacing with SD cards over SPI.

#![no_std]

use aligned::Aligned;
use core::fmt::Debug;
use core::future::Future;
use core::marker::PhantomData;
use embassy_futures::select::{select, Either};
use sdio_host::sd::{CardCapacity, CID, CSD, OCR, SD};
use sdio_host::{common_cmd::*, sd_cmd::*};

// MUST be the first module listed
mod fmt;

/// Status for card in the ready state
pub const R1_READY_STATE: u8 = 0x00;
/// Status for card in the idle state
pub const R1_IDLE_STATE: u8 = 0x01;
/// Status bit for illegal command
pub const R1_ILLEGAL_COMMAND: u8 = 0x04;
/// Start data token for read or write single block*/
pub const DATA_START_BLOCK: u8 = 0xFE;
/// Stop token for write multiple blocks*/
pub const STOP_TRAN_TOKEN: u8 = 0xFD;
/// Start data token for write multiple blocks*/
pub const WRITE_MULTIPLE_TOKEN: u8 = 0xFC;
/// Mask for data response tokens after a write block operation
pub const DATA_RES_MASK: u8 = 0x1F;
/// Write data accepted token
pub const DATA_RES_ACCEPTED: u8 = 0x05;

/// How many bytes to clock per SPI transfer while waiting for a data start token. Polling one
/// byte per transfer spends far more time on transfer setup than on the bus. Bytes that arrive
/// after the token aren't lost: they're the start of the data block.
const TOKEN_POLL_LEN: usize = 32;

#[derive(Clone, Copy, Debug, Default)]
/// SD Card
pub struct Card {
    /// The type of this card
    pub card_type: CardCapacity,
    /// Operation Conditions Register
    pub ocr: OCR<SD>,
    /// Relative Card Address
    pub rca: u32,
    /// Card ID
    pub cid: CID<SD>,
    /// Card Specific Data
    pub csd: CSD<SD>,
}

impl Card {
    /// Size in bytes
    pub fn size(&self) -> u64 {
        // SDHC / SDXC / SDUC
        self.csd.block_count() * 512
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    ChipSelect,
    SpiError,
    Timeout,
    UnsupportedCard,
    Cmd58Error,
    Cmd59Error,
    RegisterError(u8),
    CrcMismatch(u16, u16),
    NotInitialized,
    WriteError,
}

/// Must be called between powerup and [SdSpi::init] to ensure the sdcard is properly initialized.
pub async fn sd_init<SPI, CS, BE>(spi: &mut SPI, cs: &mut CS) -> Result<(), Error>
where
    SPI: embedded_hal_async::spi::SpiBus<Error = BE>,
    CS: embedded_hal::digital::OutputPin,
{
    // Supply minimum of 74 clock cycles without CS asserted.
    cs.set_high().map_err(|_| Error::ChipSelect)?;
    spi.write(&[0xFF; 10]).await.map_err(|_| Error::SpiError)?;

    Ok(())
}

pub struct SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs,
    ALIGN: aligned::Alignment,
{
    spi: SPI,
    delay: D,
    card: Option<Card>,
    _align: PhantomData<ALIGN>,
}

impl<SPI, D, ALIGN> SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    pub fn new(spi: SPI, delay: D) -> Self {
        Self {
            spi,
            delay,
            card: None,
            _align: PhantomData,
        }
    }

    /// To comply with the SD card spec, [sd_init] must be called between powerup and calling this function.
    pub async fn init(&mut self) -> Result<(), Error> {
        let r = async {
            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(idle()).await?;
                    if r == R1_IDLE_STATE {
                        return Ok(());
                    }
                }
            })
            .await??;

            // "The SPI interface is initialized in the CRC OFF mode in default"
            // -- SD Part 1 Physical Layer Specification v9.00, Section 7.2.2 Bus Transfer Protection
            if self.cmd(cmd::<R1>(0x3B, 1)).await? != R1_IDLE_STATE {
                return Err(Error::Cmd59Error);
            }

            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(send_if_cond(0x1, 0xAA)).await?;
                    if r == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE) {
                        return Err(Error::UnsupportedCard);
                    }
                    let mut buffer = [0xFFu8; 4];
                    self.spi
                        .transfer_in_place(&mut buffer[..])
                        .await
                        .map_err(|_| Error::SpiError)?;
                    if buffer[3] == 0xAA {
                        return Ok(());
                    }
                }
            })
            .await??;

            trace!("Valid card detected!");

            // If we get here we're at least a v2 card
            let mut card = Card::default();

            // send ACMD41
            with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.acmd(sd_send_op_cond(true, false, true, 0x20)).await?;
                    if r == R1_READY_STATE {
                        return Ok(());
                    }
                }
            })
            .await??;

            trace!("send_ocr");
            card.ocr = with_timeout(self.delay.clone(), 1000, async {
                loop {
                    let r = self.cmd(cmd::<R3>(0x3A, 0)).await?;
                    if r != R1_READY_STATE {
                        return Err(Error::Cmd58Error);
                    }
                    let mut buffer = [0xFFu8; 4];
                    self.spi
                        .transfer_in_place(&mut buffer[..])
                        .await
                        .map_err(|_| Error::SpiError)?;
                    let ocr: OCR<SD> = u32::from_be_bytes(buffer).into();
                    if !ocr.is_busy() {
                        return Ok(ocr);
                    }
                }
            })
            .await??;

            trace!("send_csd");
            let r = self.cmd(send_csd(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut csd = [0xFFu8; 16];
            self.read_data(&mut csd).await?;
            card.csd = u128::from_be_bytes(csd).into();

            trace!("all_send_cid");
            let r = self.cmd(send_cid(card.rca as u16)).await?;
            if r != R1_READY_STATE {
                return Err(Error::RegisterError(r));
            }
            let mut cid = [0xFFu8; 16];
            self.read_data(&mut cid).await?;
            card.cid = u128::from_be_bytes(cid).into();

            // trace!("Card initialized: {:?}", Debug2Format(&card));
            debug!("Found card with size: {}bytes", card.size());

            self.card = Some(card);

            Ok(())
        }
        .await;

        r
    }

    pub async fn read<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        let r = async {
            if data.len() == 1 {
                self.cmd(read_single_block(block_address)).await?;
                self.read_data(&mut data[0][..]).await?;
            } else {
                self.cmd(read_multiple_blocks(block_address)).await?;
                for block in data {
                    self.read_data(&mut block[..]).await?;
                }
                self.cmd(stop_transmission()).await?;
            }
            Ok(())
        }
        .await;

        r?;

        Ok(())
    }

    pub async fn write<const SIZE: usize>(
        &mut self,
        block_address: u32,
        data: &[Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Error> {
        let r = async {
            if data.len() == 1 {
                self.cmd(write_single_block(block_address)).await?;
                self.write_data(DATA_START_BLOCK, &data[0][..]).await?;
                self.wait_idle().await?;
                // check status, in SD SPI mode, the status is two bytes
                if self.cmd(sd_status()).await? != 0 {
                    return Err(Error::WriteError);
                }
                if self.read_byte().await? != 0 {
                    return Err(Error::WriteError);
                }
            } else {
                // Try sending ACMD23 _before_ write.
                // This will pre-erase blocks to improve write performance.
                // We ignore the return value, because whether its accepted
                // or not doesn't matter we will still proceed with the write
                self.acmd(cmd::<R1>(0x17, data.len() as u32)).await?;
                self.wait_idle().await?;

                self.cmd(write_multiple_blocks(block_address)).await?;
                for block in data {
                    self.wait_idle().await?;
                    self.write_data(WRITE_MULTIPLE_TOKEN, &block[..]).await?;
                }
                // stop the write
                self.wait_idle().await?;
                self.spi
                    .write(&[STOP_TRAN_TOKEN])
                    .await
                    .map_err(|_| Error::SpiError)?;
            }
            Ok(())
        }
        .await;

        r?;

        Ok(())
    }

    pub async fn size(&mut self) -> Result<u64, Error> {
        Ok(self.card.ok_or(Error::NotInitialized)?.size())
    }

    async fn read_data(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        // Never clock past this block's CRC. In a multi-block read the next block's token can
        // follow it immediately, and anything read here would be lost.
        let poll_len = TOKEN_POLL_LEN.min(1 + buffer.len() + 2);
        let mut poll = [0xFFu8; TOKEN_POLL_LEN];

        let data_start = with_timeout(self.delay.clone(), 1000, async {
            loop {
                let chunk = &mut poll[..poll_len];
                chunk.fill(0xFF);
                self.spi
                    .transfer_in_place(chunk)
                    .await
                    .map_err(|_| Error::SpiError)?;

                if let Some(i) = chunk.iter().position(|&b| b != 0xFF) {
                    if chunk[i] != DATA_START_BLOCK {
                        return Err(Error::RegisterError(chunk[i]));
                    }
                    return Ok(i + 1);
                }
            }
        })
        .await??;

        // Bytes clocked in after the token are the start of the data, then the CRC
        let early = &poll[data_start..poll_len];
        let n_data = early.len().min(buffer.len());
        let n_crc = early.len() - n_data;
        buffer[..n_data].copy_from_slice(&early[..n_data]);
        let mut crc_bytes = [0xFF; 2];
        crc_bytes[..n_crc].copy_from_slice(&early[n_data..]);

        let rest = &mut buffer[n_data..];
        if !rest.is_empty() {
            rest.fill(0xFF);
            self.spi
                .transfer_in_place(rest)
                .await
                .map_err(|_| Error::SpiError)?;
        }
        if n_crc < crc_bytes.len() {
            self.spi
                .transfer_in_place(&mut crc_bytes[n_crc..])
                .await
                .map_err(|_| Error::SpiError)?;
        }
        let crc = u16::from_be_bytes(crc_bytes);
        let calc_crc = crc16(buffer);
        if crc != calc_crc {
            return Err(Error::CrcMismatch(crc, calc_crc));
        }

        Ok(())
    }

    async fn write_data(&mut self, token: u8, buffer: &[u8]) -> Result<(), Error> {
        self.spi
            .write(&[token])
            .await
            .map_err(|_| Error::SpiError)?;
        self.spi.write(buffer).await.map_err(|_| Error::SpiError)?;
        let crc_bytes = crc16(buffer).to_be_bytes();
        self.spi
            .write(&crc_bytes)
            .await
            .map_err(|_| Error::SpiError)?;

        let status = self.read_byte().await?;
        if (status & DATA_RES_MASK) != DATA_RES_ACCEPTED {
            return Err(Error::WriteError);
        }

        Ok(())
    }

    pub fn spi(&mut self) -> &mut SPI {
        &mut self.spi
    }

    async fn cmd<R: Resp>(&mut self, cmd: Cmd<R>) -> Result<u8, Error> {
        if cmd.cmd != idle().cmd {
            self.wait_idle().await?;
        }

        let mut buf = [
            0x40 | cmd.cmd,
            (cmd.arg >> 24) as u8,
            (cmd.arg >> 16) as u8,
            (cmd.arg >> 8) as u8,
            cmd.arg as u8,
            0,
        ];
        buf[5] = crc7(&buf[0..5]);

        self.spi.write(&buf).await.map_err(|_| Error::SpiError)?;

        // skip stuff byte for stop read
        if cmd.cmd == stop_transmission().cmd {
            self.spi
                .transfer_in_place(&mut [0xFF])
                .await
                .map_err(|_| Error::SpiError)?;
        }

        let byte = with_timeout(self.delay.clone(), 1000, async {
            loop {
                let byte = self.read_byte().await?;
                if byte & 0x80 == 0 {
                    return Ok(byte);
                }
            }
        })
        .await??;

        Ok(byte)
    }

    async fn acmd<R: Resp>(&mut self, cmd: Cmd<R>) -> Result<u8, Error> {
        self.cmd(app_cmd(self.card.map(|c| c.rca).unwrap_or(0) as u16))
            .await?;
        self.cmd(cmd).await
    }

    async fn wait_idle(&mut self) -> Result<(), Error> {
        with_timeout(self.delay.clone(), 5000, async {
            while self.read_byte().await? != 0xFF {}
            Ok(())
        })
        .await?
    }

    async fn read_byte(&mut self) -> Result<u8, Error> {
        let mut buf = [0xFFu8; 1];
        self.spi
            .transfer_in_place(&mut buf[..])
            .await
            .map_err(|_| Error::SpiError)?;

        Ok(buf[0])
    }
}

impl<SPI, D, ALIGN, const SIZE: usize> block_device_driver::BlockDevice<SIZE>
    for SdSpi<SPI, D, ALIGN>
where
    SPI: embedded_hal_async::spi::SpiDevice,
    D: embedded_hal_async::delay::DelayNs + Clone,
    ALIGN: aligned::Alignment,
{
    type Error = Error;
    type Align = ALIGN;

    async fn read(
        &mut self,
        block_address: u32,
        data: &mut [Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Self::Error> {
        self.read(block_address, data).await
    }

    async fn write(
        &mut self,
        block_address: u32,
        data: &[Aligned<ALIGN, [u8; SIZE]>],
    ) -> Result<(), Self::Error> {
        self.write(block_address, data).await
    }

    async fn size(&mut self) -> Result<u64, Self::Error> {
        self.size().await
    }
}

async fn with_timeout<D: embedded_hal_async::delay::DelayNs, F: Future>(
    mut delay: D,
    timeout: u32,
    fut: F,
) -> Result<F::Output, Error> {
    match select(fut, delay.delay_ms(timeout)).await {
        Either::First(r) => Ok(r),
        Either::Second(_) => Err(Error::Timeout),
    }
}

/// Perform the 7-bit CRC used on the SD card
fn crc7(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for mut d in data.iter().cloned() {
        for _bit in 0..8 {
            crc <<= 1;
            if ((d & 0x80) ^ (crc & 0x80)) != 0 {
                crc ^= 0x09;
            }
            d <<= 1;
        }
    }
    (crc << 1) | 1
}

/// Lookup table for [`crc16`]: the CRC of each possible high byte.
///
/// A `static` so it exists once and can be placed in RAM with the `crc-table-in-ram` feature.
/// `.data.*` input sections are copied from flash to RAM at boot by cortex-m-rt style linker
/// scripts, so lookups never go through the flash cache.
#[cfg_attr(
    feature = "crc-table-in-ram",
    unsafe(link_section = ".data.sdspi_crc16_table")
)]
static CRC16_TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

/// The CRC-16-CCITT (XModem: polynomial 0x1021, initial value 0) used for data blocks.
///
/// Table-driven, one lookup per byte. This runs on every block read and written, so it matters.
fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &byte in data {
        crc = (crc << 8) ^ CRC16_TABLE[usize::from((crc >> 8) as u8 ^ byte)];
    }
    crc
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use core::convert::Infallible;
    use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};
    use std::{collections::VecDeque, vec, vec::Vec};

    /// The previous bitwise implementation, kept as a reference.
    fn crc16_bitwise(data: &[u8]) -> u16 {
        let mut crc = 0u16;
        for &byte in data {
            crc = ((crc >> 8) & 0xFF) | (crc << 8);
            crc ^= u16::from(byte);
            crc ^= (crc & 0xFF) >> 4;
            crc ^= crc << 12;
            crc ^= (crc & 0xFF) << 5;
        }
        crc
    }

    #[test]
    fn crc16_matches_bitwise_and_known_value() {
        // From the SD spec: a block of 512 0xFF bytes has CRC 0x7FA1
        assert_eq!(crc16(&[0xFF; 512]), 0x7FA1);
        let data: Vec<u8> = (0..4096u32).map(|i| (i * 31 + i / 7) as u8).collect();
        for len in [0, 1, 2, 16, 511, 512, 4096] {
            assert_eq!(crc16(&data[..len]), crc16_bitwise(&data[..len]));
        }
    }

    /// Plays back a fixed byte stream for every byte clocked in, 0xFF once it runs out.
    struct MockSpi {
        stream: VecDeque<u8>,
        transfers: usize,
    }

    impl ErrorType for MockSpi {
        type Error = Infallible;
    }

    impl SpiDevice for MockSpi {
        async fn transaction(
            &mut self,
            operations: &mut [Operation<'_, u8>],
        ) -> Result<(), Infallible> {
            self.transfers += 1;
            for op in operations {
                let len = match op {
                    Operation::Read(buf) | Operation::TransferInPlace(buf) => {
                        for b in buf.iter_mut() {
                            *b = self.stream.pop_front().unwrap_or(0xFF);
                        }
                        continue;
                    }
                    Operation::Write(buf) => buf.len(),
                    Operation::Transfer(read, write) => read.len().max(write.len()),
                    Operation::DelayNs(_) => 0,
                };
                for _ in 0..len {
                    self.stream.pop_front();
                }
            }
            Ok(())
        }
    }

    /// A delay that never finishes, so timeouts never fire
    #[derive(Clone)]
    struct NoDelay;

    impl embedded_hal_async::delay::DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {
            core::future::pending().await
        }
    }

    fn block(gap: usize, data: &[u8]) -> Vec<u8> {
        let mut out = vec![0xFF; gap];
        out.push(DATA_START_BLOCK);
        out.extend_from_slice(data);
        out.extend_from_slice(&crc16(data).to_be_bytes());
        out
    }

    fn sd(stream: Vec<u8>) -> SdSpi<MockSpi, NoDelay, aligned::A1> {
        SdSpi::new(
            MockSpi {
                stream: stream.into(),
                transfers: 0,
            },
            NoDelay,
        )
    }

    #[test]
    fn read_data_after_long_wait() {
        let data: Vec<u8> = (0..512u32).map(|i| (i * 13) as u8).collect();
        let mut sd = sd(block(100, &data));
        let mut buf = [0u8; 512];
        embassy_futures::block_on(sd.read_data(&mut buf)).unwrap();
        assert_eq!(buf[..], data[..]);
        // 101 bytes up to and including the token at 32 per poll is 4 polls, the last of
        // which holds the first 27 data bytes. Then the rest of the data, then the CRC.
        assert_eq!(sd.spi.transfers, 4 + 1 + 1);
    }

    #[test]
    fn read_data_back_to_back_blocks() {
        // Multi-block read: the next token follows the CRC with no gap at all. Each block must
        // stop exactly at its own CRC.
        let a: Vec<u8> = (0..512u32).map(|i| i as u8).collect();
        let b: Vec<u8> = (0..512u32).map(|i| (i * 3 + 1) as u8).collect();
        let mut stream = block(0, &a);
        stream.extend(block(0, &b));
        let mut sd = sd(stream);

        let mut buf = [0u8; 512];
        embassy_futures::block_on(sd.read_data(&mut buf)).unwrap();
        assert_eq!(buf[..], a[..]);
        embassy_futures::block_on(sd.read_data(&mut buf)).unwrap();
        assert_eq!(buf[..], b[..]);
    }

    #[test]
    fn read_data_short_register_read() {
        // CSD/CID reads are 16 bytes. Here the whole block, CRC included, arrives in the poll.
        let csd: Vec<u8> = (0..16u8).collect();
        let mut stream = block(0, &csd);
        stream.extend([0x12, 0x34]);
        let mut sd = sd(stream);

        let mut buf = [0u8; 16];
        embassy_futures::block_on(sd.read_data(&mut buf)).unwrap();
        assert_eq!(buf[..], csd[..]);
        assert_eq!(sd.spi.transfers, 1);
        // Nothing past the CRC was consumed
        assert_eq!(sd.spi.stream, [0x12, 0x34]);
    }

    #[test]
    fn read_data_rejects_error_token_and_bad_crc() {
        let mut sd1 = sd(vec![0xFF, 0xFF, 0x08]);
        let mut buf = [0u8; 512];
        assert_eq!(
            embassy_futures::block_on(sd1.read_data(&mut buf)),
            Err(Error::RegisterError(0x08))
        );

        let mut stream = block(3, &[0xAB; 512]);
        let last = stream.len() - 1;
        stream[last] ^= 1;
        let mut sd2 = sd(stream);
        assert!(matches!(
            embassy_futures::block_on(sd2.read_data(&mut buf)),
            Err(Error::CrcMismatch(..))
        ));
    }
}
