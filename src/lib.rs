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
    _marker: core::marker::PhantomData<I2C>,
}

impl<I2C, E> INA219NonOwned<I2C>
where
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E>,
{
    pub fn new(address: u8) -> INA219NonOwned<I2C> {
        INA219NonOwned {
            address,
            _marker: core::marker::PhantomData,
        }
    }

    pub fn calibrate(&self, i2c: &mut I2C, value: u16) -> Result<(), E> {
        i2c.write(
            self.address,
            &[Register::Calibration as u8, (value >> 8) as u8, value as u8],
        )?;
        Ok(())
    }

    pub fn shunt_voltage(&mut self, i2c: &mut I2C) -> Result<i16, E> {
        let value = self.read(i2c, Register::ShuntVoltage)?;
        Ok(value as i16)
    }

    pub fn voltage(&mut self, i2c: &mut I2C) -> Result<u16, E> {
        let value = self.read(i2c, Register::BusVoltage)?;
        Ok((value >> 3) * 4)
    }

    pub fn power(&mut self, i2c: &mut I2C) -> Result<i16, E> {
        let value = self.read(i2c, Register::Power)?;
        Ok(value as i16)
    }

    pub fn current(&mut self, i2c: &mut I2C) -> Result<i16, E> {
        let value = self.read(i2c, Register::Current)?;
        Ok(value as i16)
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
    pub fn new(i2c: I2C, address: u8) -> INA219<I2C> {
        INA219(i2c, INA219NonOwned::new(address))
    }

    pub fn calibrate(&mut self, value: u16) -> Result<(), E> {
        self.1.calibrate(&mut self.0, value)
    }

    pub fn shunt_voltage(&mut self) -> Result<i16, E> {
        self.1.shunt_voltage(&mut self.0)
    }

    pub fn voltage(&mut self) -> Result<u16, E> {
        self.1.voltage(&mut self.0)
    }

    pub fn power(&mut self) -> Result<i16, E> {
        self.1.power(&mut self.0)
    }

    pub fn current(&mut self) -> Result<i16, E> {
        self.1.current(&mut self.0)
    }
}
