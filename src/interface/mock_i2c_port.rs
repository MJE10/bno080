extern crate std;

use super::PACKET_HEADER_LENGTH;

use core::ops::Shr;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{
    ErrorKind, ErrorType, Operation, SevenBitAddress,
};
use std::collections::VecDeque;

#[allow(dead_code)]
struct FakeDelay {}

impl DelayNs for FakeDelay {
    async fn delay_ns(&mut self, _ns: u32) {
        // no-op
    }
}

const MAX_FAKE_PACKET_SIZE: usize = 512;

//divides up packets into segments
pub struct FakePacket {
    pub addr: u8,
    pub len: usize,
    pub buf: [u8; MAX_FAKE_PACKET_SIZE],
}

impl FakePacket {
    pub fn new_from_slice(slice: &[u8]) -> Self {
        let src_len = slice.len();
        let mut inst = Self {
            addr: 0,
            len: src_len,
            buf: [0; MAX_FAKE_PACKET_SIZE],
        };
        inst.buf[..src_len].copy_from_slice(&slice);
        inst
    }
}

pub struct FakeI2cPort {
    pub available_packets: VecDeque<FakePacket>,
    pub sent_packets: VecDeque<FakePacket>,
}

impl FakeI2cPort {
    pub fn new() -> Self {
        FakeI2cPort {
            available_packets: VecDeque::with_capacity(3),
            sent_packets: VecDeque::with_capacity(3),
        }
    }

    /// Enqueue a packet to be received later
    pub fn add_available_packet(&mut self, bytes: &[u8]) {
        let pack = FakePacket::new_from_slice(bytes);
        self.available_packets.push_back(pack);
    }
}

#[derive(Debug)]
pub struct FakeI2cError;

impl embedded_hal_async::i2c::Error for FakeI2cError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

impl ErrorType for FakeI2cPort {
    type Error = FakeI2cError;
}

impl embedded_hal_async::i2c::I2c for FakeI2cPort {
    async fn read(
        &mut self,
        addr: SevenBitAddress,
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        let next_pack =
            self.available_packets.pop_front().unwrap_or(FakePacket {
                addr: addr,
                len: 0,
                buf: [0; MAX_FAKE_PACKET_SIZE],
            });

        let src_len = next_pack.len;
        if src_len == 0 {
            return Ok(());
        }

        let dest_len = buffer.len();

        if src_len > dest_len {
            //only read as much as the reader has room for,
            //then push the remainder back onto the queue as a remainder packet
            let read_len = dest_len;
            buffer[..read_len].copy_from_slice(&next_pack.buf[..read_len]);
            let remainder_len = src_len - read_len;
            let mut remainder_packet = FakePacket {
                addr: addr,
                len: remainder_len + 4,
                buf: [0; MAX_FAKE_PACKET_SIZE],
            };
            remainder_packet.buf
                [PACKET_HEADER_LENGTH..PACKET_HEADER_LENGTH + remainder_len]
                .copy_from_slice(
                    &next_pack.buf[read_len..read_len + remainder_len],
                );
            remainder_packet.buf[0] = ((remainder_len + 4) & 0xFF) as u8;
            remainder_packet.buf[1] =
                ((((remainder_len + 4) & 0xFF00) as u16).shr(8) as u8) | 0x80; //set continuation flag
            self.available_packets.push_front(remainder_packet);
        } else if src_len == dest_len {
            let read_len = src_len;
            buffer[..read_len].copy_from_slice(&next_pack.buf[..read_len]);
        } else {
            // src_len < dest_len
            panic!("src_len {} dest_len {}", src_len, dest_len);
        }

        Ok(())
    }

    async fn write(
        &mut self,
        _address: SevenBitAddress,
        bytes: &[u8],
    ) -> Result<(), Self::Error> {
        let sent_pack = FakePacket::new_from_slice(bytes);
        self.sent_packets.push_back(sent_pack);
        Ok(())
    }

    async fn write_read(
        &mut self,
        address: SevenBitAddress,
        send_buf: &[u8],
        recv_buf: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write(address, send_buf).await?;
        self.read(address, recv_buf).await?;
        Ok(())
    }

    async fn transaction(
        &mut self,
        _address: SevenBitAddress,
        _operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        todo!()
    }
}
