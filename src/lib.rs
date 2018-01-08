//! A WIP, no_std, generic driver for the L3GD20 (gyroscope)

#![deny(missing_docs)]
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

const READ: u8 = 1 << 7;
const WRITE: u8 = 0 << 7;
const MULTI: u8 = 1 << 6;
const SINGLE: u8 = 0 << 6;

impl Register {
    fn addr(self) -> u8 {
        self as u8
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
