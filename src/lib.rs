#![no_std]

use embedded_hal::blocking::i2c::{Write, WriteRead};
use num_enum::{FromPrimitive, IntoPrimitive};

#[repr(u8)]
enum Reg {
    ChipId = 0x00,
    Error = 0x02,
    Status = 0x03,
}

const BMA423_DEFAULT_ADDRESS: u8 = 0x19;

#[derive(Debug)]
pub enum Bma423Error {
    I2cError,
    Uninitialized,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive, FromPrimitive)]
pub enum ChipId {
    #[default]
    Unknown = 0x00,
    Bma423 = 0x01, // TODO find the real value
}

pub struct Bma423<I2C> {
    address: u8,
    i2c: I2C,
    state: State,
}

pub enum State {
    Uninitialized,
    Initialized(ChipId),
}

impl<I2C, E> Bma423<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            address: BMA423_DEFAULT_ADDRESS,
            i2c,
            state: State::Uninitialized,
        }
    }

    pub fn new_with_address(address: u8, i2c: I2C) -> Self {
        Self {
            address,
            i2c,
            state: State::Uninitialized,
        }
    }

    fn probe_chip(&mut self) -> Result<u8, E> {
        let mut data: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.address, &[Reg::ChipId as u8], &mut data)?;
        Ok(data[0])
    }

    pub fn init(&mut self) -> Result<(), E> {
        let chip_id = self.probe_chip()?;
        self.state = State::Initialized(chip_id.into());
        Ok(())
    }

    pub fn get_chip_id(&mut self) -> Result<ChipId, Bma423Error> {
        match self.state {
            State::Uninitialized => Err(Bma423Error::Uninitialized),
            State::Initialized(chip_id) => Ok(chip_id),
        }
    }
    pub fn get_status(&mut self) -> Result<u8, E> {
        let mut data: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.address, &[Reg::Status as u8], &mut data)?;
        Ok(data[0])
    }
}
