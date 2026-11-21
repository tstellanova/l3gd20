use embedded_hal::spi::Mode;
use embedded_hal::spi::SpiDevice;

use crate::*;

/// SPI mode
pub const MODE: Mode = embedded_hal::spi::MODE_3;

const READ: u8 = 1 << 7;
const WRITE: u8 = 0 << 7;
const MULTI: u8 = 1 << 6;
const SINGLE: u8 = 0 << 6;

/// L3GD20 driver
pub struct L3gd20<Spi> {
    spi: Spi,
}

impl<Spi: SpiDevice> L3gd20<Spi> {
    /// Creates a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: Spi) -> Result<Self, Spi::Error> {
        let mut l3gd20 = L3gd20 { spi };

        // power up and enable all the axes
        l3gd20.write_register(Register::CTRL_REG1, 0b0000_1111)?;

        Ok(l3gd20)
    }

    /// Temperature measurement + gyroscope measurements
    pub fn all(&mut self) -> Result<Measurements, Spi::Error> {
        let mut bytes = [0u8; 9];
        self.read_many(Register::OUT_TEMP, &mut bytes)?;

        Ok(Measurements {
            gyro: I16x3 {
                x: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
                y: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
                z: (bytes[7] as u16 + ((bytes[8] as u16) << 8)) as i16,
            },
            temp_raw: bytes[1] as i8,
        })
    }

    /// Gyroscope measurements
    pub fn gyro(&mut self) -> Result<I16x3, Spi::Error> {
        let mut bytes = [0u8; 7];
        self.read_many(Register::OUT_X_L, &mut bytes)?;

        Ok(I16x3 {
            x: (bytes[1] as u16 + ((bytes[2] as u16) << 8)) as i16,
            y: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
            z: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
        })
    }

    /// Raw temperature sensor measurement
    pub fn temp_raw(&mut self) -> Result<i8, Spi::Error> {
        Ok(self.read_register(Register::OUT_TEMP)? as i8)
    }

    /// Actual temperature derived by subtracting the raw measurement to the baseline value of 25 C
    pub fn temp_celcius(&mut self) -> Result<i16, Spi::Error> {
        Ok(25 - self.temp_raw()? as i16)
    }

    /// Reads the WHO_AM_I register; should return `0xD4`
    pub fn who_am_i(&mut self) -> Result<u8, Spi::Error> {
        self.read_register(Register::WHO_AM_I)
    }

    /// Read `STATUS_REG` of sensor
    pub fn status(&mut self) -> Result<Status, Spi::Error> {
        let sts = self.read_register(Register::STATUS_REG)?;
        Ok(Status::from_u8(sts))
    }

    /// Get the current Output Data Rate
    pub fn odr(&mut self) -> Result<Odr, Spi::Error> {
        // Read control register
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        Ok(Odr::from_u8(reg1))
    }

    /// Set the Output Data Rate
    pub fn set_odr(&mut self, odr: Odr) -> Result<&mut Self, Spi::Error> {
        self.change_config(Register::CTRL_REG1, odr)
    }

    /// Get current Bandwidth
    pub fn bandwidth(&mut self) -> Result<Bandwidth, Spi::Error> {
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        Ok(Bandwidth::from_u8(reg1))
    }

    /// Set low-pass cut-off frequency (i.e. bandwidth)
    ///
    /// See `Bandwidth` for further explanation
    pub fn set_bandwidth(&mut self, bw: Bandwidth) -> Result<&mut Self, Spi::Error> {
        self.change_config(Register::CTRL_REG1, bw)
    }

    /// Get the current Full Scale Selection
    ///
    /// This is the sensitivity of the sensor, see `Scale` for more information
    pub fn scale(&mut self) -> Result<Scale, Spi::Error> {
        let scl = self.read_register(Register::CTRL_REG4)?;
        Ok(Scale::from_u8(scl))
    }

    /// Set the Full Scale Selection
    ///
    /// This sets the sensitivity of the sensor, see `Scale` for more
    /// information
    pub fn set_scale(&mut self, scale: Scale) -> Result<&mut Self, Spi::Error> {
        self.change_config(Register::CTRL_REG4, scale)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, Spi::Error> {
        let mut buffer = [reg.addr() | SINGLE | READ, 0];
        self.spi.transfer_in_place(&mut buffer)?;

        Ok(buffer[1])
    }

    /// Read multiple bytes starting from the `start_reg` register.
    /// This function will attempt to fill the provided buffer.
    fn read_many(&mut self, start_reg: Register, buffer: &mut [u8]) -> Result<(), Spi::Error> {
        buffer[0] = start_reg.addr() | MULTI | READ;
        self.spi.transfer_in_place(buffer)?;

        Ok(())
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), Spi::Error> {
        let buffer = [reg.addr() | SINGLE | WRITE, byte];
        self.spi.write(&buffer)?;

        Ok(())
    }

    /// Change configuration in register
    ///
    /// Helper function to update a particular part of a register without
    /// affecting other parts of the register that might contain desired
    /// configuration. This allows the `L3gd20` struct to be used like
    /// a builder interface when configuring specific parameters.
    fn change_config<B: BitValue>(
        &mut self,
        reg: Register,
        bits: B,
    ) -> Result<&mut Self, Spi::Error> {
        // Create bit mask from width and shift of value
        let mask = B::mask() << B::shift();
        // Extract the value as u8
        let bits = (bits.value() << B::shift()) & mask;
        // Read current value of register
        let current = self.read_register(reg)?;
        // Use supplied mask so we don't affect more than necessary
        let masked = current & !mask;
        // Use `or` to apply the new value without affecting other parts
        let new_reg = masked | bits;
        self.write_register(reg, new_reg)?;
        Ok(self)
    }
}
