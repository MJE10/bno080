/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use crate::interface::{SensorInterface, PACKET_HEADER_LENGTH};

use crate::constants::*;
use core::ops::Shr;
#[cfg(feature = "defmt")]
use defmt::println;
use embedded_hal_async::delay::DelayNs;

const PACKET_SEND_BUF_LEN: usize = 256;
const PACKET_RECV_BUF_LEN: usize = 1024;

#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WrapperError<E> {
    ///Communications error
    CommError(E),
    /// Invalid chip ID was read
    InvalidChipId(u8),
    /// Unsupported sensor firmware version
    InvalidFWVersion(u8),
    /// We expected some data but didn't receive any
    NoDataAvailable,
}

pub struct BNO080<SI> {
    pub(crate) sensor_interface: SI,
    /// each communication channel with the device has its own sequence number
    sequence_numbers: [u8; NUM_CHANNELS],
    /// buffer for building and sending packet to the sensor hub
    packet_send_buf: [u8; PACKET_SEND_BUF_LEN],
    /// buffer for building packets received from the sensor hub
    packet_recv_buf: [u8; PACKET_RECV_BUF_LEN],

    last_packet_len_received: usize,
    /// has the device been successfully reset
    device_reset: bool,
    /// has the product ID been verified
    prod_id_verified: bool,

    init_received: bool,

    /// have we received the full advertisement
    advert_received: bool,

    /// have we received an error list
    error_list_received: bool,
    last_error_received: u8,

    last_chan_received: u8,
    last_exec_chan_rid: u8,
    last_command_chan_rid: u8,

    pub accel: ([f32; 3], u8),
    pub lin_accel: ([f32; 3], u8),
    pub gyro: ([f32; 3], u8),
    pub mag: ([f32; 3], u8),
    pub rotation_vector: ([f32; 4], f32, u8),
    pub gravity: ([f32; 3], u8),

    pub calibration_status: u8, // Byte R0 of ME Calibration Response
}

impl<SI> BNO080<SI> {
    pub fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            sensor_interface,
            sequence_numbers: [0; NUM_CHANNELS],
            packet_send_buf: [0; PACKET_SEND_BUF_LEN],
            packet_recv_buf: [0; PACKET_RECV_BUF_LEN],
            last_packet_len_received: 0,
            device_reset: false,
            prod_id_verified: false,
            init_received: false,
            advert_received: false,
            error_list_received: false,
            last_error_received: 0,
            last_chan_received: 0,
            last_exec_chan_rid: 0,
            last_command_chan_rid: 0,

            accel: ([0.0; 3], 0),
            lin_accel: ([0.0; 3], 0),
            gyro: ([0.0; 3], 0),
            mag: ([0.0; 3], 0),
            rotation_vector: ([0.0; 4], 0.0, 0),
            gravity: ([0.0; 3], 0),
            calibration_status: 0,
        }
    }

    /// Returns previously consumed serial sensor instance.
    pub fn free(self) -> SI {
        self.sensor_interface
    }
}

impl<SI, SE> BNO080<SI>
where
    SI: SensorInterface<SensorError = SE>,
{
    /// Consume all available messages on the port without processing them
    pub async fn eat_all_messages(&mut self, delay: &mut impl DelayNs) {
        #[cfg(feature = "defmt")]
        println!("eat_n");
        loop {
            let msg_count = self.eat_one_message(delay).await;
            if msg_count == 0 {
                break;
            }
            //give some time to other parts of the system
            delay.delay_ms(1).await;
        }
    }

    /// Handle any messages with a timeout
    pub async fn handle_all_messages(
        &mut self,
        delay: &mut impl DelayNs,
        timeout_ms: u8,
    ) -> u32 {
        let mut total_handled: u32 = 0;
        loop {
            let handled_count =
                self.handle_one_message(delay, timeout_ms).await;
            if handled_count == 0 {
                break;
            } else {
                total_handled += handled_count;
                //give some time to other parts of the system
                delay.delay_ms(1).await;
            }
        }
        total_handled
    }

    /// return the number of messages handled
    pub async fn handle_one_message(
        &mut self,
        delay: &mut impl DelayNs,
        max_ms: u8,
    ) -> u32 {
        let mut msg_count = 0;

        let res = self.receive_packet_with_timeout(delay, max_ms).await;
        if res.is_ok() {
            let received_len = res.unwrap_or(0);
            if received_len > 0 {
                msg_count += 1;
                self.handle_received_packet(received_len);
            }
        } else {
            #[cfg(feature = "defmt")]
            println!("handle1 err");
        }

        msg_count
    }

    /// Receive and ignore one message,
    /// returning the size of the packet received or zero
    /// if there was no packet to read.
    pub async fn eat_one_message(&mut self, delay: &mut impl DelayNs) -> usize {
        let res = self.receive_packet_with_timeout(delay, 150).await;
        if let Ok(received_len) = res {
            #[cfg(feature = "defmt")]
            println!("e1 {}", received_len);
            received_len
        } else {
            #[cfg(feature = "defmt")]
            println!("e1 err");
            0
        }
    }

    fn handle_advertise_response(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];
        let mut cursor: usize = 1; //skip response type

        #[cfg(feature = "defmt")]
        println!("AdvRsp: {}", payload_len);

        while cursor < payload_len {
            let _tag: u8 = payload[cursor];
            cursor += 1;
            let len: u8 = payload[cursor];
            cursor += 1;
            //let val: u8 = payload + cursor;
            cursor += len as usize;
        }

        self.advert_received = true;
    }

    fn read_u8_at_cursor(msg: &[u8], cursor: &mut usize) -> u8 {
        let val = msg[*cursor];
        *cursor += 1;
        val
    }

    fn read_i16_at_cursor(msg: &[u8], cursor: &mut usize) -> i16 {
        let val = (msg[*cursor] as i16) | ((msg[*cursor + 1] as i16) << 8);
        *cursor += 2;
        val
    }

    fn try_read_i16_at_cursor(msg: &[u8], cursor: &mut usize) -> Option<i16> {
        let remaining = msg.len() - *cursor;
        if remaining >= 2 {
            let val = (msg[*cursor] as i16) | ((msg[*cursor + 1] as i16) << 8);
            *cursor += 2;
            Some(val)
        } else {
            None
        }
    }

    /// Read data values from a single input report
    fn handle_one_input_report(
        outer_cursor: usize,
        msg: &[u8],
    ) -> (usize, u8, u8, i16, i16, i16, i16, i16) {
        let mut cursor = outer_cursor;

        let feature_report_id = Self::read_u8_at_cursor(msg, &mut cursor);
        let _rep_seq_num = Self::read_u8_at_cursor(msg, &mut cursor);
        let rep_status = Self::read_u8_at_cursor(msg, &mut cursor);
        let _delay = Self::read_u8_at_cursor(msg, &mut cursor);

        let data1: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data2: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data3: i16 = Self::read_i16_at_cursor(msg, &mut cursor);
        let data4: i16 =
            Self::try_read_i16_at_cursor(msg, &mut cursor).unwrap_or(0);
        let data5: i16 =
            Self::try_read_i16_at_cursor(msg, &mut cursor).unwrap_or(0);

        (
            cursor,
            feature_report_id,
            rep_status,
            data1,
            data2,
            data3,
            data4,
            data5,
        )
    }

    /// Handle parsing of an input report packet,
    /// which may include multiple input reports
    fn handle_sensor_reports(&mut self, received_len: usize) {
        // Sensor input packets have the form:
        // [u8; 5]  timestamp in microseconds for the packet?
        // a sequence of n reports, each with four byte header
        // u8 report ID
        // u8 sequence number of report

        // let mut report_count = 0;
        let mut outer_cursor: usize = PACKET_HEADER_LENGTH + 5; //skip header, timestamp
                                                                //TODO need to skip more above for a payload-level timestamp??
        if received_len < outer_cursor {
            #[cfg(feature = "defmt")]
            println!("bad lens: {} < {}", received_len, outer_cursor);
            return;
        }

        let payload_len = received_len - outer_cursor;
        // if payload_len < 14 {
        //     #[cfg(feature = "defmt")]
        //     println!(
        //         "bad report: {:?}",
        //         &self.packet_recv_buf[..PACKET_HEADER_LENGTH]
        //     );
        //
        //     return;
        // }

        // there may be multiple reports per payload
        while outer_cursor < payload_len {
            //let start_cursor = outer_cursor;
            let (
                inner_cursor,
                report_id,
                status,
                data1,
                data2,
                data3,
                data4,
                data5,
            ) = Self::handle_one_input_report(
                outer_cursor,
                &self.packet_recv_buf[..received_len],
            );
            outer_cursor = inner_cursor;
            // report_count += 1;
            let q_triple = |q_point: u8| -> ([f32; 3], u8) {
                (
                    [data1, data2, data3].map(|x| q_to_float(x, q_point)),
                    status,
                )
            };
            match report_id {
                SENSOR_REPORTID_ACCELEROMETER => {
                    self.accel = q_triple(ACCELEROMETER_Q1)
                }
                SENSOR_REPORTID_LINEAR_ACCELERATION => {
                    self.lin_accel = q_triple(LINEAR_ACCELEROMETER_Q1)
                }
                SENSOR_REPORTID_GYROSCOPE => self.gyro = q_triple(GYRO_Q1),
                SENSOR_REPORTID_MAGNETIC_FIELD => {
                    self.mag = q_triple(MAGNETOMETER_Q1)
                }
                SENSOR_REPORTID_ROTATION_VECTOR => {
                    self.rotation_vector = (
                        [data1, data2, data3, data4]
                            .map(|x| q_to_float(x, ROTATION_VECTOR_Q1)),
                        q_to_float(data5, ROTATION_VECTOR_ACCURACY_Q1),
                        status,
                    )
                }
                SENSOR_REPORTID_GRAVITY => self.gravity = q_triple(GRAVITY_Q1),
                _ => {
                    // debug_println!("uhr: {:X}", report_id);
                    // debug_println!("uhr: 0x{:X} {:?}  ", report_id, &self.packet_recv_buf[start_cursor..start_cursor+5]);
                }
            }
        }

        //debug_println!("report_count: {}",report_count);
    }

    /// Handle one or more errors sent in response to a command
    fn handle_cmd_resp_error_list(&mut self, received_len: usize) {
        let payload_len = received_len - PACKET_HEADER_LENGTH;
        let payload = &self.packet_recv_buf[PACKET_HEADER_LENGTH..received_len];

        self.error_list_received = true;
        for cursor in 1..payload_len {
            let err: u8 = payload[cursor];
            self.last_error_received = err;
            #[cfg(feature = "defmt")]
            println!("lerr: {:x}", err);
        }
    }

    pub fn handle_received_packet(&mut self, received_len: usize) {
        let msg = &self.packet_recv_buf[..received_len];
        let chan_num = msg[2];
        //let _seq_num =  msg[3];
        let report_id: u8 = if received_len > PACKET_HEADER_LENGTH {
            msg[4]
        } else {
            0
        };

        self.last_chan_received = chan_num;
        match chan_num {
            CHANNEL_COMMAND => match report_id {
                CMD_RESP_ADVERTISEMENT => {
                    self.handle_advertise_response(received_len);
                }
                CMD_RESP_ERROR_LIST => {
                    self.handle_cmd_resp_error_list(received_len);
                }
                _ => {
                    self.last_command_chan_rid = report_id;
                    #[cfg(feature = "defmt")]
                    println!("unh cmd: {}", report_id);
                }
            },
            CHANNEL_EXECUTABLE => match report_id {
                EXECUTABLE_DEVICE_RESP_RESET_COMPLETE => {
                    self.device_reset = true;
                    #[cfg(feature = "defmt")]
                    println!("resp_reset {}", 1);
                }
                _ => {
                    self.last_exec_chan_rid = report_id;
                    #[cfg(feature = "defmt")]
                    println!("unh exe: {:x}", report_id);
                }
            },
            CHANNEL_CONTROL => {
                match report_id {
                    SHTP_COMMAND_RESPONSE => {
                        // 0xF1 / 241
                        let cmd_resp = msg[6];
                        if cmd_resp == SH2_STARTUP_INIT_UNSOLICITED {
                            self.init_received = true;
                        } else if cmd_resp == SH2_INIT_SYSTEM {
                            self.init_received = true;
                        }
                        #[cfg(feature = "defmt")]
                        println!("CMD_RESP: 0x{:X}", cmd_resp);
                    }
                    SHTP_REPORT_PRODUCT_ID_RESPONSE => {
                        #[cfg(feature = "defmt")]
                        {
                            //let reset_cause = msg[4 + 1];
                            let sw_vers_major = msg[4 + 2];
                            let sw_vers_minor = msg[4 + 3];
                            println!(
                                "PID_RESP {}.{}",
                                sw_vers_major, sw_vers_minor
                            );
                        }

                        self.prod_id_verified = true;
                    }
                    SHTP_REPORT_GET_FEATURE_RESPONSE => {
                        // 0xFC
                        #[cfg(feature = "defmt")]
                        println!("feat resp: {}", msg[5]);
                    }
                    _ => {
                        // #[cfg(feature = "defmt")]
                        // println!(
                        //     "unh hbc: 0x{:X} {:x?}",
                        //     report_id,
                        //     &msg[..PACKET_HEADER_LENGTH]
                        // );
                    }
                }
            }
            CHANNEL_REPORTS => {
                self.handle_sensor_reports(received_len);
            }
            _ => {
                self.last_chan_received = chan_num;
                #[cfg(feature = "defmt")]
                println!("unh chan 0x{:X}", chan_num);
            }
        }
    }

    /// The BNO080 starts up with all sensors disabled,
    /// waiting for the application to configure it.
    pub async fn init(
        &mut self,
        delay_source: &mut impl DelayNs,
    ) -> Result<(), WrapperError<SE>> {
        #[cfg(feature = "defmt")]
        println!("wrapper init");

        //Section 5.1.1.1 : On system startup, the SHTP control application will send
        // its full advertisement response, unsolicited, to the host.
        delay_source.delay_ms(1).await;
        self.sensor_interface
            .setup(delay_source)
            .await
            .map_err(WrapperError::CommError)?;

        if self.sensor_interface.requires_soft_reset() {
            delay_source.delay_ms(1).await;
            self.soft_reset().await?;
            delay_source.delay_ms(150).await;
            self.eat_all_messages(delay_source).await;
            delay_source.delay_ms(50).await;
            self.eat_all_messages(delay_source).await;
        } else {
            // we only expect two messages after reset:
            // eat the advertisement response
            self.eat_one_message(delay_source).await;
            // eat the unsolicited initialization response
            self.eat_one_message(delay_source).await;
        }

        self.verify_product_id(delay_source).await?;
        //self.eat_all_messages(delay_source);

        Ok(())
    }

    /// Enable a particular report
    pub async fn enable_report(
        &mut self,
        report_id: u8,
        millis_between_reports: u16,
    ) -> Result<(), WrapperError<SE>> {
        #[cfg(feature = "defmt")]
        println!("enable_report 0x{:X}", report_id);

        let micros_between_reports: u32 =
            (millis_between_reports as u32) * 1000;
        let cmd_body: [u8; 17] = [
            SHTP_REPORT_SET_FEATURE_COMMAND,
            report_id,
            0,                                        //feature flags
            0,                                        //LSB change sensitivity
            0,                                        //MSB change sensitivity
            (micros_between_reports & 0xFFu32) as u8, // LSB report interval, microseconds
            (micros_between_reports.shr(8) & 0xFFu32) as u8,
            (micros_between_reports.shr(16) & 0xFFu32) as u8,
            (micros_between_reports.shr(24) & 0xFFu32) as u8, // MSB report interval
            0, // LSB Batch Interval
            0,
            0,
            0, // MSB Batch interval
            0, // LSB sensor-specific config
            0,
            0,
            0, // MSB sensor-specific config
        ];

        //we simply blast out this configuration command and assume it'll succeed
        self.send_packet(CHANNEL_CONTROL, &cmd_body).await?;
        // any error or success in configuration will arrive some time later

        Ok(())
    }

    /// Prepare a packet for sending, in our send buffer
    fn prep_send_packet(&mut self, channel: u8, body_data: &[u8]) -> usize {
        let body_len = body_data.len();

        let packet_length = body_len + PACKET_HEADER_LENGTH;
        let packet_header = [
            (packet_length & 0xFF) as u8, //LSB
            packet_length.shr(8) as u8,   //MSB
            channel,
            self.sequence_numbers[channel as usize],
        ];
        self.sequence_numbers[channel as usize] += 1;

        self.packet_send_buf[..PACKET_HEADER_LENGTH]
            .copy_from_slice(packet_header.as_ref());
        self.packet_send_buf[PACKET_HEADER_LENGTH..packet_length]
            .copy_from_slice(body_data);

        packet_length
    }

    /// Send packet from our packet send buf
    async fn send_packet(
        &mut self,
        channel: u8,
        body_data: &[u8],
    ) -> Result<usize, WrapperError<SE>> {
        let packet_length = self.prep_send_packet(channel, body_data);
        self.sensor_interface
            .write_packet(&self.packet_send_buf[..packet_length])
            .await
            .map_err(WrapperError::CommError)?;
        Ok(packet_length)
    }

    /// Read one packet into the receive buffer
    pub(crate) async fn receive_packet_with_timeout(
        &mut self,
        delay: &mut impl DelayNs,
        max_ms: u8,
    ) -> Result<usize, WrapperError<SE>> {
        // #[cfg(feature = "defmt")]
        // println!("r_p");

        self.packet_recv_buf[0] = 0;
        self.packet_recv_buf[1] = 0;
        let packet_len = self
            .sensor_interface
            .read_with_timeout(&mut self.packet_recv_buf, delay, max_ms)
            .await
            .map_err(WrapperError::CommError)?;

        self.last_packet_len_received = packet_len;
        // #[cfg(feature = "defmt")]
        // println!("recv {}", packet_len);

        Ok(packet_len)
    }

    /// Verify that the sensor returns an expected chip ID
    async fn verify_product_id(
        &mut self,
        delay: &mut impl DelayNs,
    ) -> Result<(), WrapperError<SE>> {
        #[cfg(feature = "defmt")]
        println!("request PID...");
        let cmd_body: [u8; 2] = [
            SHTP_REPORT_PRODUCT_ID_REQUEST, //request product ID
            0,                              //reserved
        ];

        // for some reason, reading PID right sending request does not work with i2c
        if self.sensor_interface.requires_soft_reset() {
            self.send_packet(CHANNEL_CONTROL, cmd_body.as_ref()).await?;
        } else {
            let response_size = self
                .send_and_receive_packet(CHANNEL_CONTROL, cmd_body.as_ref())
                .await?;
            if response_size > 0 {
                self.handle_received_packet(response_size);
            }
        };

        // process all incoming messages until we get a product id (or no more data)
        while !self.prod_id_verified {
            #[cfg(feature = "defmt")]
            println!("read PID");
            let msg_count = self.handle_one_message(delay, 150u8).await;
            if msg_count < 1 {
                break;
            }
        }

        if !self.prod_id_verified {
            return Err(WrapperError::InvalidChipId(0));
        }
        Ok(())
    }

    /// Tell the sensor to reset.
    /// Normally applications should not need to call this directly,
    /// as it is called during `init`.
    pub async fn soft_reset(&mut self) -> Result<(), WrapperError<SE>> {
        // #[cfg(feature = "defmt")]
        // println!("soft_reset");
        let data: [u8; 1] = [EXECUTABLE_DEVICE_CMD_RESET];
        // send command packet and ignore received packets
        let received_len = self
            .send_and_receive_packet(CHANNEL_EXECUTABLE, data.as_ref())
            .await?;
        if received_len > 0 {
            self.handle_received_packet(received_len);
        }

        Ok(())
    }

    /// Send a packet and receive the response
    async fn send_and_receive_packet(
        &mut self,
        channel: u8,
        body_data: &[u8],
    ) -> Result<usize, WrapperError<SE>> {
        let send_packet_length = self.prep_send_packet(channel, body_data);
        // #[cfg(feature = "defmt")]
        // println!("srcv {} ...", send_packet_length);

        let recv_packet_length = self
            .sensor_interface
            .send_and_receive_packet(
                &self.packet_send_buf[..send_packet_length].as_ref(),
                &mut self.packet_recv_buf,
            )
            .await
            .map_err(WrapperError::CommError)?;

        #[cfg(feature = "defmt")]
        println!("srcv {} {}", send_packet_length, recv_packet_length);

        Ok(recv_packet_length)
    }
}

fn q_to_float(fixed_point_value: i16, q_point: u8) -> f32 {
    let scale: f32 = 1.0 / ((1 << q_point) as f32);
    (fixed_point_value as f32) * scale
}

/// Command Channel requests / responses

// Commands
//const CMD_GET_ADVERTISEMENT: u8 = 0;
//const CMD_SEND_ERROR_LIST: u8 = 1;

/// Responses
const CMD_RESP_ADVERTISEMENT: u8 = 0;
const CMD_RESP_ERROR_LIST: u8 = 1;

// some mysterious responses we sometimes get:
// 0x78, 0x7C

/// Response to CMD_RESET
const EXECUTABLE_DEVICE_RESP_RESET_COMPLETE: u8 = 1;

/// Commands and subcommands
const SH2_INIT_UNSOLICITED: u8 = 0x80;
const SH2_INIT_SYSTEM: u8 = 1;
const SH2_STARTUP_INIT_UNSOLICITED: u8 =
    COMMAND_INITIALIZE | SH2_INIT_UNSOLICITED;
