#![no_std]
#[cfg(feature = "accel")]
use accelerometer::{vector::F32x3, Accelerometer};

use embedded_hal::blocking::{
    delay::DelayUs,
    i2c::{Write, WriteRead},
};

use log::*;

use bitmask_enum::bitmask;
use num_enum::{FromPrimitive, IntoPrimitive};

mod config;

#[allow(dead_code)]
#[repr(u8)]
#[derive(Debug, Clone, Copy, IntoPrimitive)]
enum Reg {
    ChipId = 0x00,
    Error = 0x02,
    Status = 0x03,

    AccXLSB = 0x12,
    AccXMSB = 0x13,
    AccYLSB = 0x14,
    AccYMSB = 0x15,
    AccZLSB = 0x16,
    AccZMSB = 0x17,

    AccelConfig = 0x40,
    AccelRange = 0x41,

    Interrupt1IOCtl = 0x53,
    Interrupt2IOCtl = 0x54,
    InterruptConfig = 0x55,
    Interrupt1Mapping = 0x56,
    Interrupt2Mapping = 0x57,
    InterruptMapping = 0x58,
    StartInitialization = 0x59,
    Bma4Reserved5BAddr = 0x5b,
    Bma4Reserved5CAddr = 0x5c,
    FeatureConfig = 0x5e,
    InternalError = 0x5f,
    NvmConfig = 0x6a,
    SerialIfConfig = 0x6b,
    AccelSelfTest = 0x6d,
    NvmBackendConfig = 0x70,
    OffsetX = 0x71,
    OffsetY = 0x72,
    OffsetZ = 0x73,
    PowerConfiguration = 0x7c,
    PowerControl = 0x7d,
    Command = 0x7e,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
enum Command {
    NvmProg = 0xa0,
    FifoFlush = 0xb0,
    SoftReset = 0xb6,
}

#[bitmask(u8)]
#[derive(Copy, Clone, Debug)]
pub enum PowerControlFlag {
    Accelerometer = Self(0b0000_0100),
    Auxiliary = Self(0b0000_0001),
}

#[bitmask(u8)]
#[derive(Copy, Clone, Debug)]
pub enum PowerConfigurationFlag {
    AdvancedPowerSave = Self(0b0000_0001),
    FifoSelfWakeUp = Self(0b0000_0010),
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum AccelConfigOdr {
    Odr0p78 = 0x01,
    Odr1p5 = 0x02,
    Odr3p1 = 0x03,
    Odr6p25 = 0x04,
    Odr12p5 = 0x05,
    Odr25 = 0x06,
    Odr50 = 0x07,
    Odr100 = 0x08,
    Odr200 = 0x09,
    Odr400 = 0x0a,
    Odr800 = 0x0b,
    Odr1k6 = 0x0c,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum AccelConfigBandwidth {
    Osr4Avg1 = 0x00,
    Osr2Avg2 = 0x10,
    NormAvg4 = 0x20,
    CicAvg8 = 0x30,
    ResAvg16 = 0x40,
    ResAvg32 = 0x50,
    ResAvg64 = 0x60,
    ResAvg128 = 0x70,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, IntoPrimitive)]
pub enum AccelConfigPerfMode {
    CicAvg = 0x00,
    Continuous = 0x80,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum AccelRange {
    Range2g = 0x00,
    Range4g = 0x01,
    Range8g = 0x02,
    Range16g = 0x03,
}

const BMA423_DEFAULT_ADDRESS: u8 = 0x19;
const BMA423_READ_WRITE_LEN: usize = 0x08;
const GRAVITY_EARTH: f32 = 9.80665;

#[derive(Debug)]
pub enum Bma423Error {
    I2cError,
    ConfigError,
    Uninitialized,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive, FromPrimitive)]
pub enum ChipId {
    #[default]
    Unknown = 0x00,
    Bma423 = 0x13,
}

pub struct Bma423<I2C> {
    address: u8,
    i2c: I2C,
    state: State,
    config: Option<Config>,
}

#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    bandwidth: AccelConfigBandwidth,
    range: AccelRange,
    performance_mode: AccelConfigPerfMode,
    sample_rate: AccelConfigOdr,
}

pub enum State {
    Uninitialized,
    Initialized(ChipId),
}

impl<I2C> Bma423<I2C>
where
    I2C: Write + WriteRead,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            address: BMA423_DEFAULT_ADDRESS,
            i2c,
            state: State::Uninitialized,
            config: None,
        }
    }

    pub fn new_with_address(address: u8, i2c: I2C) -> Self {
        Self {
            address,
            i2c,
            state: State::Uninitialized,
            config: None,
        }
    }

    fn probe_chip(&mut self) -> Result<u8, <I2C as WriteRead>::Error> {
        let mut data: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.address, &[Reg::ChipId.into()], &mut data)?;
        Ok(data[0])
    }

    pub fn set_power_control(
        &mut self,
        value: PowerControlFlag,
    ) -> Result<(), <I2C as Write>::Error> {
        self.i2c
            .write(self.address, &[Reg::PowerControl.into(), value.into()])
    }

    pub fn set_power_config(
        &mut self,
        value: PowerConfigurationFlag,
    ) -> Result<(), <I2C as Write>::Error> {
        self.i2c.write(
            self.address,
            &[Reg::PowerConfiguration.into(), value.into()],
        )
    }

    pub fn get_chip_id(&mut self) -> Result<ChipId, Bma423Error> {
        match self.state {
            State::Uninitialized => Err(Bma423Error::Uninitialized),
            State::Initialized(chip_id) => Ok(chip_id),
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayUs<u32>) -> Result<(), Bma423Error> {
        /*
        self.i2c
            .write(
                self.address,
                &[Reg::Command.into(), Command::SoftReset.into()],
            )
            .map_err(|_e| Bma423Error::I2cError)?;
        */

        let chip_id = self.probe_chip().map_err(|_e| Bma423Error::I2cError)?;
        self.state = State::Initialized(chip_id.into());

        self.set_power_config(PowerConfigurationFlag::none())
            .map_err(|_e| Bma423Error::I2cError)?;

        delay.delay_us(500);

        self.i2c
            .write(self.address, &[Reg::StartInitialization.into(), 0u8])
            .map_err(|_e| Bma423Error::I2cError)?;

        self.stream_write(Reg::FeatureConfig, &config::BMA423_CONFIG_FILE)
            .map_err(|_e| Bma423Error::I2cError)?;

        self.i2c
            .write(self.address, &[Reg::StartInitialization.into(), 1u8])
            .map_err(|_e| Bma423Error::I2cError)?;

        self.set_power_control(PowerControlFlag::Accelerometer)
            .map_err(|_e| Bma423Error::I2cError)?;

        Ok(())
    }

    fn stream_write(&mut self, reg: Reg, data: &[u8]) -> Result<(), <I2C as Write>::Error> {
        let inc: usize = BMA423_READ_WRITE_LEN;
        let mut index: usize = 0;
        loop {
            if index >= data.len() {
                break;
            }

            let mut buf: [u8; 9] = [0; 9];
            buf[0] = reg.into();
            buf[1..9].copy_from_slice(&data[index..index + inc]);

            let asic_msb: u8 = ((index / 2) >> 4) as u8;
            let asic_lsb: u8 = ((index / 2) & 0x0F) as u8;

            self.i2c
                .write(self.address, &[Reg::Bma4Reserved5BAddr.into(), asic_lsb])?;
            self.i2c
                .write(self.address, &[Reg::Bma4Reserved5CAddr.into(), asic_msb])?;

            self.i2c.write(self.address, &buf)?;

            index += inc;
        }
        Ok(())
    }

    pub fn set_accel_config(
        &mut self,
        odr: AccelConfigOdr,
        bw: AccelConfigBandwidth,
        rm: AccelConfigPerfMode,
        g_range: AccelRange,
    ) -> Result<(), Bma423Error> {
        if rm == AccelConfigPerfMode::Continuous {
            if (bw as u8) > (AccelConfigBandwidth::NormAvg4 as u8) {
                return Err(Bma423Error::ConfigError);
            }
        } else if rm == AccelConfigPerfMode::CicAvg {
            if (bw as u8) > (AccelConfigBandwidth::ResAvg128 as u8) {
                return Err(Bma423Error::ConfigError);
            }
        } else {
            return Err(Bma423Error::ConfigError);
        }

        let accel_config: u8 = odr as u8 | bw as u8 | rm as u8;
        let accel_range: u8 = g_range as u8;
        self.i2c
            .write(self.address, &[Reg::AccelConfig.into(), accel_config])
            .map_err(|_e| Bma423Error::I2cError)?;
        self.i2c
            .write(self.address, &[Reg::AccelRange.into(), accel_range])
            .map_err(|_e| Bma423Error::I2cError)?;

        self.config = Some(Config {
            sample_rate: odr,
            bandwidth: bw,
            performance_mode: rm,
            range: g_range,
        });
        Ok(())
    }

    pub fn get_status(&mut self) -> Result<u8, <I2C as WriteRead>::Error> {
        let mut data: [u8; 1] = [0; 1];
        self.i2c
            .write_read(self.address, &[Reg::Status.into()], &mut data)?;
        Ok(data[0])
    }

    pub fn get_x_y_z(&mut self) -> Result<(f32, f32, f32), <I2C as WriteRead>::Error> {
        info!("Reading x, y, z");
        let mut data: [u8; 6] = [0; 6];
        self.i2c
            .write_read(self.address, &[Reg::AccXLSB.into()], &mut data)?;
        info!("Computing coords");
        let x: i16 = ((((data[1] as i16) << 8) as i16) | (data[0] as i16)) / 0x10;
        let y: i16 = ((((data[3] as i16) << 8) as i16) | (data[2] as i16)) / 0x10;
        let z: i16 = ((((data[5] as i16) << 8) as i16) | (data[4] as i16)) / 0x10;

        info!("Returning result");
        Ok((
            lsb_to_ms2(x, 2.0, 12),
            lsb_to_ms2(y, 2.0, 12),
            lsb_to_ms2(z, 2.0, 12),
        ))
    }
}

#[cfg(feature = "accel")]
impl<I2C> Accelerometer for Bma423<I2C>
where
    I2C: Write + WriteRead,
{
    type Error = Bma423Error;
    fn accel_norm(&mut self) -> Result<F32x3, accelerometer::Error<Bma423Error>> {
        use accelerometer::error::{Error, ErrorKind};
        let (x, y, z) = self
            .get_x_y_z()
            .map_err(|_e| Error::new_with_cause(ErrorKind::Bus, Bma423Error::I2cError))?;
        Ok(F32x3::new(x, y, z))
    }

    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<Bma423Error>> {
        use accelerometer::error::{Error, ErrorKind};
        match self.config {
            Some(config) => match config.sample_rate {
                AccelConfigOdr::Odr0p78 => Ok(25.0 / 32.0),
                AccelConfigOdr::Odr1p5 => Ok(25.0 / 16.0),
                AccelConfigOdr::Odr3p1 => Ok(25.0 / 8.0),
                AccelConfigOdr::Odr6p25 => Ok(25.0 / 4.0),
                AccelConfigOdr::Odr12p5 => Ok(25.0 / 2.0),
                AccelConfigOdr::Odr25 => Ok(25.0),
                AccelConfigOdr::Odr50 => Ok(50.0),
                AccelConfigOdr::Odr100 => Ok(100.0),
                AccelConfigOdr::Odr200 => Ok(200.0),
                AccelConfigOdr::Odr400 => Ok(400.0),
                AccelConfigOdr::Odr800 => Ok(800.0),
                AccelConfigOdr::Odr1k6 => Ok(1600.0),
            },
            None => Err(Error::new_with_cause(
                ErrorKind::Device,
                Bma423Error::ConfigError,
            )),
        }
    }
}

#[inline(always)]
fn lsb_to_ms2(val: i16, g_range: f32, bit_width: u8) -> f32 {
    let half_scale: f32 = (1 << bit_width) as f32;

    GRAVITY_EARTH * val as f32 * g_range / half_scale
}
