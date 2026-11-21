//! A platform agnostic driver to interface with the L3GD20 (gyroscope)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/1.0
//!
//! # Examples
//!
//! You should find at least one example in the [f3] crate.
//!
//! [f3]: https://docs.rs/f3/0.6

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

/// Minimal time in nanoseconds between chip select assertion and clock edge.
pub const MINIMUM_CS_SETUP_TIME_NS: u32 = 5;

/// Expected WHO_AM_I register value for the L3GD20 sensor.
pub const WHO_AM_I_L3GD20: u8 = 0xD4;
/// Expected WHO_AM_I register value for the L3GD20H sensor.
pub const WHO_AM_I_L3GD20H: u8 = 0xD7;

/// I2C module.
pub mod i2c;
/// SPI module.
pub mod spi;

/// Trait to represent a value that can be sent to sensor
trait BitValue {
    /// The width of the bitfield in bits
    fn width() -> u8;
    /// The bit 'mask' of the value
    fn mask() -> u8 {
        (1 << Self::width()) - 1
    }
    /// The number of bits to shift the mask by
    fn shift() -> u8;
    /// Convert the type to a byte value to be sent to sensor
    ///
    /// # Note
    /// This value should not be bit shifted.
    fn value(&self) -> u8;
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Register {
    WHO_AM_I = 0x0F,
    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,
    REFERENCE = 0x25,
    OUT_TEMP = 0x26,
    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    FIFO_CTRL_REG = 0x2E,
    FIFO_SRC_REG = 0x2F,
    INT1_CFG = 0x30,
    INT1_SRC = 0x31,
    INT1_TSH_XH = 0x32,
    INT1_TSH_XL = 0x33,
    INT1_TSH_YH = 0x34,
    INT1_TSH_YL = 0x35,
    INT1_TSH_ZH = 0x36,
    INT1_TSH_ZL = 0x37,
    INT1_DURATION = 0x38,
}

/// Output Data Rate
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Odr {
    /// 95 Hz data rate
    Hz95 = 0x00,
    /// 190 Hz data rate
    Hz190 = 0x01,
    /// 380 Hz data rate
    Hz380 = 0x02,
    /// 760 Hz data rate
    Hz760 = 0x03,
}

impl BitValue for Odr {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        6
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Odr {
    fn from_u8(from: u8) -> Self {
        // Extract ODR value, converting to enum (ROI: 0b1100_0000)
        match (from >> Odr::shift()) & Odr::mask() {
            x if x == Odr::Hz95 as u8 => Odr::Hz95,
            x if x == Odr::Hz190 as u8 => Odr::Hz190,
            x if x == Odr::Hz380 as u8 => Odr::Hz380,
            x if x == Odr::Hz760 as u8 => Odr::Hz760,
            _ => unreachable!(),
        }
    }
}

/// Full scale selection
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Scale {
    /// 250 Degrees Per Second
    Dps250 = 0x00,
    /// 500 Degrees Per Second
    Dps500 = 0x01,
    /// 2000 Degrees Per Second
    Dps2000 = 0x03,
}

impl BitValue for Scale {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        4
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Scale {
    fn from_u8(from: u8) -> Self {
        // Extract scale value from register, ensure that we mask with
        // `0b0000_0011` to extract `FS1-FS2` part of register
        match (from >> Scale::shift()) & Scale::mask() {
            x if x == Scale::Dps250 as u8 => Scale::Dps250,
            x if x == Scale::Dps500 as u8 => Scale::Dps500,
            x if x == Scale::Dps2000 as u8 => Scale::Dps2000,
            // Special case for Dps2000
            0x02 => Scale::Dps2000,
            _ => unreachable!(),
        }
    }
}

/// Bandwidth of sensor
///
/// The bandwidth of the sensor is equal to the cut-off for the low-pass
/// filter. The cut-off depends on the `Odr` of the sensor, for specific
/// information consult the data sheet.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Bandwidth {
    /// Lowest possible cut-off for any `Odr` configuration
    Low = 0x00,
    /// Medium cut-off, can be the same as `High` for some `Odr` configurations
    Medium = 0x01,
    /// High cut-off
    High = 0x02,
    /// Maximum cut-off for any `Odr` configuration
    Maximum = 0x03,
}

impl BitValue for Bandwidth {
    fn width() -> u8 {
        2
    }
    fn shift() -> u8 {
        4
    }
    fn value(&self) -> u8 {
        *self as u8
    }
}

impl Bandwidth {
    fn from_u8(from: u8) -> Self {
        // Shift and mask bandwidth of register, (ROI: 0b0011_0000)
        match (from >> Bandwidth::shift()) & Bandwidth::mask() {
            x if x == Bandwidth::Low as u8 => Bandwidth::Low,
            x if x == Bandwidth::Medium as u8 => Bandwidth::Medium,
            x if x == Bandwidth::High as u8 => Bandwidth::High,
            x if x == Bandwidth::Maximum as u8 => Bandwidth::Maximum,
            _ => unreachable!(),
        }
    }
}

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

impl Scale {
    /// Convert a measurement to degrees
    pub fn degrees(&self, val: i16) -> f32 {
        match *self {
            Scale::Dps250 => val as f32 * 0.00875,
            Scale::Dps500 => val as f32 * 0.0175,
            Scale::Dps2000 => val as f32 * 0.07,
        }
    }

    /// Convert a measurement to radians
    pub fn radians(&self, val: i16) -> f32 {
        // TODO: Use `to_radians` or other built in method
        // NOTE: `to_radians` is only exported in `std` (07.02.18)
        self.degrees(val) * (core::f32::consts::PI / 180.0)
    }
}

/// XYZ triple
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// Several measurements
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurements {
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Raw temperature sensor measurement
    pub temp_raw: i8,
}

impl Measurements {
    /// Convert the raw temperature value to degrees celcius
    pub fn temp_celcius(&self) -> i16 {
        25 - self.temp_raw as i16
    }
}

/// Sensor status
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Overrun (data has overwritten previously unread data)
    /// has occurred on at least one axis
    pub overrun: bool,
    /// Overrun occurred on Z-axis
    pub z_overrun: bool,
    /// Overrun occurred on Y-axis
    pub y_overrun: bool,
    /// Overrun occurred on X-axis
    pub x_overrun: bool,
    /// New data is available for either X, Y, Z - axis
    pub new_data: bool,
    /// New data is available on Z-axis
    pub z_new: bool,
    /// New data is available on Y-axis
    pub y_new: bool,
    /// New data is available on X-axis
    pub x_new: bool,
}

impl Status {
    fn from_u8(from: u8) -> Self {
        Status {
            overrun: (from & (1 << 7)) != 0,
            z_overrun: (from & (1 << 6)) != 0,
            y_overrun: (from & (1 << 5)) != 0,
            x_overrun: (from & (1 << 4)) != 0,
            new_data: (from & (1 << 3)) != 0,
            z_new: (from & (1 << 2)) != 0,
            y_new: (from & (1 << 1)) != 0,
            x_new: (from & 1) != 0,
        }
    }
}
