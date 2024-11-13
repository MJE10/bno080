/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]
#![allow(async_fn_in_trait)]
extern crate embedded_hal_async;

#[allow(dead_code)]
pub mod constants;
pub mod interface;
pub mod wrapper;

/// Errors in this crate
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),

    /// The sensor is not responding
    SensorUnresponsive,
}
