//! A platform agnostic driver to interface with the L3GD20 (gyroscope)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1
//!
//! # Examples
//!
//! You should find at least one example in the [f3] crate.
//!
//! [f3]: https://docs.rs/f3/~0.5

#![deny(missing_docs)]
#![deny(warnings)]
#![feature(unsize)]
#![no_std]

extern crate embedded_hal as hal;

use core::marker::Unsize;
use core::mem;

use hal::blocking::spi::{Transfer, Write};
use hal::spi::{Mode, Phase, Polarity};
use hal::digital::OutputPin;

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

/// L3GD20 driver
pub struct L3gd20<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS, E> L3gd20<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    /// Creates a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: SPI, cs: CS) -> Result<Self, E> {
        let mut l3gd20 = L3gd20 { spi, cs };

        // power up and enable all the axes
        l3gd20.write_register(Register::CTRL_REG1, 0b00_00_1_111)?;

        Ok(l3gd20)
    }

    /// Temperature measurement + gyroscope measurements
    pub fn all(&mut self) -> Result<Measurements, E> {
        let bytes: [u8; 9] = self.read_registers(Register::OUT_TEMP)?;

        Ok(Measurements {
            gyro: I16x3 {
                x: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
                y: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
                z: (bytes[7] as u16 + ((bytes[8] as u16) << 8)) as i16,
            },
            temp: bytes[1] as i8,
        })
    }

    /// Gyroscope measurements
    pub fn gyro(&mut self) -> Result<I16x3, E> {
        let bytes: [u8; 7] = self.read_registers(Register::OUT_X_L)?;

        Ok(I16x3 {
            x: (bytes[1] as u16 + ((bytes[2] as u16) << 8)) as i16,
            y: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
            z: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
        })
    }

    /// Temperature sensor measurement
    pub fn temp(&mut self) -> Result<i8, E> {
        Ok(self.read_register(Register::OUT_TEMP)? as i8)
    }

    /// Reads the WHO_AM_I register; should return `0xD4`
    pub fn who_am_i(&mut self) -> Result<u8, E> {
        self.read_register(Register::WHO_AM_I)
    }

    /// Read `STATUS_REG` of sensor
    pub fn status(&mut self) -> Result<Status, E> {
        let sts = self.read_register(Register::STATUS_REG)?;
        Ok(
            Status{
                overrun:   (sts & 1 << 7) != 0,
                z_overrun: (sts & 1 << 6) != 0,
                y_overrun: (sts & 1 << 5) != 0,
                x_overrun: (sts & 1 << 4) != 0,
                new_data:  (sts & 1 << 3) != 0,
                z_new:     (sts & 1 << 2) != 0,
                y_new:     (sts & 1 << 1) != 0,
                x_new:     (sts & 1 << 0) != 0,
            })
    }

    /// Get the current Output Data Rate
    pub fn odr(&mut self) -> Result<ODR, E> {
        // Read control register
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        // Extract ODR value, converting to enum (ROI: 0b1100_0000)
        let odr = match (reg1 >> 6) & 0x03 {
            x if x == ODR::Hz95  as u8 => ODR::Hz95,
            x if x == ODR::Hz190 as u8 => ODR::Hz190,
            x if x == ODR::Hz380 as u8 => ODR::Hz380,
            x if x == ODR::Hz760 as u8 => ODR::Hz760,
            _ => unreachable!(),
        };
        Ok(odr)
    }

    /// Set the Output Data Rate
    pub fn set_odr(&mut self, odr: ODR) -> Result<&mut Self, E> {
        // New configuration
        let bits = (odr as u8) << 6;
        // Mask to only affect ODR configuration
        let mask = 0b1100_0000;
        // Apply change
        self.change_config(Register::CTRL_REG1, mask, bits)
    }

    /// Get current Bandwidth
    pub fn bandwidth(&mut self) -> Result<Bandwidth, E> {
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        // Shift and mask bandwidth of register, (ROI: 0b0011_0000)
        let bw = match (reg1 >> 4) & 0x03 {
            x if x == Bandwidth::Low     as u8 => Bandwidth::Low,
            x if x == Bandwidth::Medium  as u8 => Bandwidth::Medium,
            x if x == Bandwidth::High    as u8 => Bandwidth::High,
            x if x == Bandwidth::Maximum as u8 => Bandwidth::Maximum,
            _ => unreachable!(),
        };
        Ok(bw)
    }

    /// Set low-pass cut-off frequency (i.e. bandwidth)
    ///
    /// See `Bandwidth` for further explanation
    pub fn set_bandwidth(&mut self, bw: Bandwidth) -> Result<&mut Self, E> {
        let bits = (bw as u8) << 4;
        let mask = 0b0011_0000;
        self.change_config(Register::CTRL_REG1, mask, bits)
    }

    /// Get the current Full Scale Selection
    ///
    /// This is the sensitivity of the sensor, see `Scale` for more information
    pub fn scale(&mut self) -> Result<Scale, E> {
        let scl = self.read_register(Register::CTRL_REG4)?;
        // Extract scale value from register, ensure that we mask with
        // `0b0000_0011` to extract `FS1-FS2` part of register
        let scale = match (scl >> 2) & 0x03 {
            x if x == Scale::Dps250  as u8 => Scale::Dps250,
            x if x == Scale::Dps500  as u8 => Scale::Dps500,
            x if x == Scale::Dps2000 as u8 => Scale::Dps2000,
            // Special case for Dps2000
            0x02 => Scale::Dps2000,
            _ => unreachable!(),
        };
        Ok(scale)
    }

    /// Set the Full Scale Selection
    ///
    /// This sets the sensitivity of the sensor, see `Scale` for more
    /// information
    pub fn set_scale(&mut self, scale: Scale) -> Result<&mut Self, E> {
        let bits = (scale as u8) << 2;
        let mask = 0b0000_1100;
        self.change_config(Register::CTRL_REG4, mask, bits)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        self.cs.set_low();

        let mut buffer = [reg.addr() | SINGLE | READ, 0];
        self.spi.transfer(&mut buffer)?;

        self.cs.set_high();

        Ok(buffer[1])
    }

    fn read_registers<B>(&mut self, reg: Register) -> Result<B, E>
    where
        B: Unsize<[u8]>,
    {
        self.cs.set_low();

        let mut buffer: B = unsafe { mem::uninitialized() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.addr() | MULTI | READ;
            self.spi.transfer(slice)?;
        }

        self.cs.set_high();

        Ok(buffer)
    }

    fn write_register(
        &mut self,
        reg: Register,
        byte: u8,
    ) -> Result<(), E> {
        self.cs.set_low();

        let buffer = [reg.addr() | SINGLE | WRITE, byte];
        self.spi.write(&buffer)?;

        self.cs.set_high();

        Ok(())
    }

    /// Change configuration in register
    ///
    /// Helper function to update a particular part of a register without
    /// affecting other parts of the register that might contain desired
    /// configuration. This allows the `L3gd20` struct to be used like
    /// a builder interface when configuring specific parameters.
    fn change_config(&mut self, reg: Register, mask: u8, new_value: u8) -> Result<&mut Self, E> {
        // Read current value of register
        let current = self.read_register(reg)?;
        // Use supplied mask so we don't affect more than necessary
        let masked  = current & !mask;
        // Use `or` to apply the new value without affecting other parts
        let new_reg = masked | new_value;
        self.write_register(reg, new_reg)?;
        Ok(self)
    }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
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
pub enum ODR {
    /// 95 Hz data rate
    Hz95  = 0x00,
    /// 190 Hz data rate
    Hz190 = 0x01,
    /// 380 Hz data rate
    Hz380 = 0x02,
    /// 760 Hz data rate
    Hz760 = 0x03,
}

/// Full scale selection
#[derive(Debug, Clone, Copy)]
pub enum Scale {
    /// 250 Degrees Per Second
    Dps250  = 0x00,
    /// 500 Degrees Per Second
    Dps500  = 0x01,
    /// 2000 Degrees Per Second
    Dps2000 = 0x03,
}

/// Bandwidth of sensor
///
/// The bandwidth of the sensor is equal to the cut-off for the low-pass
/// filter. The cut-off depends on the `ODR` of the sensor, for specific
/// information consult the data sheet.
#[derive(Debug, Clone, Copy)]
pub enum Bandwidth {
    /// Lowest possible cut-off for any `ODR` configuration
    Low     = 0x00,
    /// Medium cut-off, can be the same as `High` for some `ODR` configurations
    Medium  = 0x01,
    /// High cut-off
    High    = 0x02,
    /// Maximum cut-off for any `ODR` configuration
    Maximum = 0x03,
}

const READ: u8 = 1 << 7;
const WRITE: u8 = 0 << 7;
const MULTI: u8 = 1 << 6;
const SINGLE: u8 = 0 << 6;

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

impl Scale {
    /// Convert a measurement to degrees
    pub fn degrees(&self, val: i16) -> f32 {
        match *self {
            Scale::Dps250  => val as f32 * 0.00875,
            Scale::Dps500  => val as f32 * 0.0175,
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
#[derive(Debug)]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// Several measurements
#[derive(Debug)]
pub struct Measurements {
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Temperature sensor measurement
    pub temp: i8,
}

/// Sensor status
#[derive(Debug, Clone, Copy)]
pub struct Status {
    /// Overrun (data has overwritten previously unread data)
    /// has occurred on at least one axis
    overrun: bool,
    /// Overrun occurred on Z-axis
    z_overrun: bool,
    /// Overrun occurred on Y-axis
    y_overrun: bool,
    /// Overrun occurred on X-axis
    x_overrun: bool,
    /// New data is available for either X, Y, Z - axis
    new_data: bool,
    /// New data is available on Z-axis
    z_new: bool,
    /// New data is available on Y-axis
    y_new: bool,
    /// New data is available on X-axis
    x_new: bool,
}
