#![no_std]

extern crate byteorder;
extern crate embedded_hal as hal;

use byteorder::{BigEndian, ByteOrder};
use hal::blocking::i2c;

pub const INA219_ADDR: u8 = 0x41;

enum Register {
    // Configuration = 0x00,
    ShuntVoltage = 0x01,
    BusVoltage = 0x02,
    Power = 0x03,
    Current = 0x04,
    Calibration = 0x05,
}

pub struct INA219NonOwned<I2C> {
    address: u8,
    current_lsb: i16,
    power_lsb: i16,
    _marker: core::marker::PhantomData<I2C>,
}

impl<I2C, E> INA219NonOwned<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
{
    /// Maximum expected current in A, must be in (0; 3.2].
    /// r_shunt in Ohm
    pub fn new(
        i2c: &mut I2C,
        address: u8,
        maximum_expected_current: f32,
        r_shunt: f32,
    ) -> Result<INA219NonOwned<I2C>, E> {
        // see https://www.ti.com/lit/ds/symlink/ina219.pdf page 12
        let current_lsb = maximum_expected_current / 2_u32.pow(15) as f32;
        let callibration_register = 0.04096 / (current_lsb as f32 * r_shunt);
        let callibration_register_value = callibration_register as u16;

        let power_lsb = 20_f32 * current_lsb;

        let ina219 = INA219NonOwned {
            address,
            current_lsb: (current_lsb * 1000000_f32) as i16,
            power_lsb: (power_lsb * 1000000_f32) as i16,
            _marker: core::marker::PhantomData,
        };

        ina219.calibrate(i2c, callibration_register_value)?;

        Ok(ina219)
    }

    pub fn calibrate(&self, i2c: &mut I2C, value: u16) -> Result<(), E> {
        i2c.write(
            self.address,
            &[Register::Calibration as u8, (value >> 8) as u8, value as u8],
        )?;
        Ok(())
    }

    /// Shunt voltage in 10 µV
    pub fn shunt_voltage(&mut self, i2c: &mut I2C) -> Result<i16, E> {
        let value = self.read(i2c, Register::ShuntVoltage)?;
        Ok(value as i16)
    }

    /// Voltage in mV
    pub fn voltage(&mut self, i2c: &mut I2C) -> Result<u16, E> {
        let value = self.read(i2c, Register::BusVoltage)?;
        Ok((value >> 3) * 4)
    }

    /// Power in µW
    pub fn power(&mut self, i2c: &mut I2C) -> Result<i16, E> {
        let value = self.read(i2c, Register::Power)?;
        Ok(value as i16 * self.power_lsb)
    }

    /// Current in µA
    pub fn current(&mut self, i2c: &mut I2C) -> Result<i16, E> {
        let value = self.read(i2c, Register::Current)?;
        Ok(value as i16 * self.current_lsb)
    }

    fn read(&mut self, i2c: &mut I2C, register: Register) -> Result<u16, E> {
        let mut buf: [u8; 2] = [0x00; 2];
        i2c.write(self.address, &[register as u8])?;
        i2c.read(self.address, &mut buf)?;
        Ok(BigEndian::read_u16(&buf))
    }
}

pub struct INA219<I2C>(I2C, INA219NonOwned<I2C>);

impl<I2C, E> INA219<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
{
    /// Maximum expected current in A, must be in (0; 3.2].
    /// r_shunt in Ohm
    pub fn new(
        mut i2c: I2C,
        address: u8,
        maximum_expected_current: f32,
        r_shunt: f32,
    ) -> Result<INA219<I2C>, E> {
        let ina219 = INA219NonOwned::new(&mut i2c, address, maximum_expected_current, r_shunt)?;
        Ok(INA219(i2c, ina219))
    }

    pub fn calibrate(&mut self, value: u16) -> Result<(), E> {
        self.1.calibrate(&mut self.0, value)
    }

    /// Shunt voltage in 10 µV
    pub fn shunt_voltage(&mut self) -> Result<i16, E> {
        self.1.shunt_voltage(&mut self.0)
    }

    /// Voltage in mV
    pub fn voltage(&mut self) -> Result<u16, E> {
        self.1.voltage(&mut self.0)
    }

    /// Power in µW
    pub fn power(&mut self) -> Result<i16, E> {
        self.1.power(&mut self.0)
    }

    /// Current in µA
    pub fn current(&mut self) -> Result<i16, E> {
        self.1.current(&mut self.0)
    }
}
