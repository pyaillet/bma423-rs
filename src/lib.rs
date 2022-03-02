#![no_std]

#[cfg(feature = "accel")]
use accelerometer::{vector::F32x3, Accelerometer};

use embedded_hal::blocking::{
    delay::DelayUs,
    i2c::{Write, WriteRead},
};

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

    FeatureInterruptStatus = 0x1c,
    HardwareInterruptStatus = 0x1d,

    AccelConfig = 0x40,
    AccelRange = 0x41,

    Interrupt1IOCtl = 0x53,
    Interrupt2IOCtl = 0x54,
    InterruptConfig = 0x55,
    FeatureInterrupt1Mapping = 0x56,
    FeatureInterrupt2Mapping = 0x57,
    HardwareInterruptMapping = 0x58,
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

#[bitmask(u8)]
#[derive(Copy, Clone, Debug)]
pub enum FeatureInterruptStatus {
    /* Taken from the datasheet */
    /*
    StepCounterOut = Self(0b0000_0010),
    ActivityTypeOut = Self(0b0000_0100),
    WristTiltOut = Self(0b0000_1000),
    WakeUpOut = Self(0b0010_0000),
    AnyNoMotionOut = Self(0b0100_0000),
    ErrorIntOut = Self(0b1000_0000),
    */
    /* Taken from examples */
    SingleTap = Self(0b0000_0001),
    StepCounter = Self(0b0000_0010),
    Activity = Self(0b0000_0100),
    WristWear = Self(0b0000_1000),
    DoubleTap = Self(0b0001_0000),
    AnyMotion = Self(0b0010_0000),
    NoMotion = Self(0b0100_0000),
    Error = Self(0b1000_0000),
}

#[bitmask(u8)]
#[derive(Copy, Clone, Debug)]
pub enum HardwareInterruptStatus {
    FifoFull = Self(0x01),
    FifoWatermark = Self(0x02),
    DataReady = Self(0x04),
    AuxiliaryDataReady = Self(0x20),
    AcceleratorDataReady = Self(0x80),
}

#[derive(Copy, Clone, Debug)]
pub struct InterruptStatus {
    pub feature: FeatureInterruptStatus,
    pub hardware: HardwareInterruptStatus,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptLine {
    Line1 = 0x01,
    Line2 = 0x02,
}

#[bitmask(u8)]
#[derive(Copy, Clone, Debug)]
pub enum Features {
    StepCounter = Self(0b0000_0001),
    StepActivity = Self(0b0000_0010),
    WristWear = Self(0b0000_0100),
    SingleTap = Self(0b0000_1000),
    DoubleTap = Self(0b0001_0000),
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum FeatureOffset {
    AnyMotion = 0x00,
    NoMotion = 0x04,
    StepCounterParam = 0x08,
    StepCounter = 0x3A,
    SingleTap = 0x3C,
    DoubleTap = 0x3E,
    WristWear = 0x40,
    ConfigId = 0x42,
    AxesRemap = 0x44,
}

#[bitmask(u8)]
#[derive(Copy, Clone, Debug)]
pub enum FeatureEnableMask {
    StepCounter = Self(0b0001_0000),
    StepActivity = Self(0b0010_0000),
    WristWear = Self(0b0000_0001),
    SingleTap = Self(0b0000_0001),
    DoubleTap = Self(0b0000_0001),
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive, FromPrimitive)]
enum Activity {
    Stationary = 0x00,
    Walking = 0x01,
    Running = 0x02,
    #[default]
    Invalid = 0x03,
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

const DEFAULT_ADDRESS: u8 = 0x19;
const READ_WRITE_LEN: usize = 0x08;
const FEATURE_SIZE: usize = 0x70;
const GRAVITY_EARTH: f32 = 9.80665;

#[derive(Clone, Copy, Debug)]
pub enum Error<E> {
    BusError(E),
    I2cError,
    ConfigError,
    Uninitialized,
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
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

impl<E, I2C> Bma423<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            address: DEFAULT_ADDRESS,
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

    fn write(&mut self, data: &[u8]) -> Result<(), Error<E>> {
        self.i2c.write(self.address, data)?;
        Ok(())
    }

    fn write_read(&mut self, reg: Reg, data: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c.write_read(self.address, &[reg.into()], data)?;
        Ok(())
    }

    fn probe_chip(&mut self) -> Result<u8, Error<E>> {
        let mut data: [u8; 1] = [0; 1];
        self.write_read(Reg::ChipId, &mut data)?;
        Ok(data[0])
    }

    pub fn set_power_control(&mut self, value: PowerControlFlag) -> Result<(), Error<E>> {
        self.write(&[Reg::PowerControl.into(), value.into()])
    }

    pub fn set_power_config(&mut self, value: PowerConfigurationFlag) -> Result<(), Error<E>> {
        self.write(&[Reg::PowerConfiguration.into(), value.into()])
    }

    pub fn get_chip_id(&mut self) -> Result<ChipId, Error<E>> {
        match self.state {
            State::Uninitialized => Err(Error::Uninitialized),
            State::Initialized(chip_id) => Ok(chip_id),
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayUs<u32>) -> Result<(), Error<E>> {
        let chip_id = self.probe_chip()?;
        self.state = State::Initialized(chip_id.into());

        self.set_power_config(PowerConfigurationFlag::none())?;

        delay.delay_us(500);

        self.write(&[Reg::StartInitialization.into(), 0u8])?;

        self.stream_write(Reg::FeatureConfig, &config::BMA423_CONFIG_FILE)?;

        self.write(&[Reg::StartInitialization.into(), 1u8])?;

        self.set_power_control(PowerControlFlag::Accelerometer)?;

        Ok(())
    }

    fn stream_write(&mut self, reg: Reg, data: &[u8]) -> Result<(), Error<E>> {
        let inc: usize = READ_WRITE_LEN;
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

            self.write(&[Reg::Bma4Reserved5BAddr.into(), asic_lsb])?;
            self.write(&[Reg::Bma4Reserved5CAddr.into(), asic_msb])?;

            self.write(&buf)?;

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
    ) -> Result<(), Error<E>> {
        if rm == AccelConfigPerfMode::Continuous {
            if (bw as u8) > (AccelConfigBandwidth::NormAvg4 as u8) {
                return Err(Error::ConfigError);
            }
        } else if rm == AccelConfigPerfMode::CicAvg {
            if (bw as u8) > (AccelConfigBandwidth::ResAvg128 as u8) {
                return Err(Error::ConfigError);
            }
        } else {
            return Err(Error::ConfigError);
        }

        let accel_config: u8 = odr as u8 | bw as u8 | rm as u8;
        let accel_range: u8 = g_range as u8;
        self.write(&[Reg::AccelConfig.into(), accel_config])?;
        self.write(&[Reg::AccelRange.into(), accel_range])?;

        self.config = Some(Config {
            sample_rate: odr,
            bandwidth: bw,
            performance_mode: rm,
            range: g_range,
        });
        Ok(())
    }

    pub fn enable_feature(&mut self, features: Features) -> Result<(), Error<E>> {
        let mut feature_config: [u8; FEATURE_SIZE + 1] = [0; FEATURE_SIZE + 1];
        self.write_read(Reg::FeatureConfig, &mut feature_config[1..FEATURE_SIZE + 1])?;

        if features.contains(Features::StepCounter) {
            let idx = FeatureOffset::StepCounter as usize + 1;
            feature_config[idx] |= u8::from(FeatureEnableMask::StepCounter);
        }
        if features.contains(Features::StepActivity) {
            let idx = FeatureOffset::StepCounter as usize + 1;
            feature_config[idx] |= u8::from(FeatureEnableMask::StepActivity);
        }

        if features.contains(Features::WristWear) {
            let idx = FeatureOffset::WristWear as usize;
            feature_config[idx] |= u8::from(FeatureEnableMask::WristWear);
        }

        if features.contains(Features::SingleTap) {
            let idx = FeatureOffset::SingleTap as usize;
            feature_config[idx] |= u8::from(FeatureEnableMask::SingleTap);
        }
        if features.contains(Features::DoubleTap) {
            let idx = FeatureOffset::DoubleTap as usize;
            feature_config[idx] |= u8::from(FeatureEnableMask::DoubleTap);
        }

        feature_config[0] = Reg::FeatureConfig.into();
        self.write(&feature_config)?;

        Ok(())
    }

    pub fn disable_feature(&mut self, features: Features) -> Result<(), Error<E>> {
        let mut feature_config: [u8; FEATURE_SIZE + 1] = [0; FEATURE_SIZE + 1];
        self.write_read(Reg::FeatureConfig, &mut feature_config[1..FEATURE_SIZE + 1])?;

        if features.contains(Features::StepCounter) {
            let idx = FeatureOffset::StepCounter as usize + 1;
            feature_config[idx] &= u8::from(FeatureEnableMask::StepCounter.not());
        }
        if features.contains(Features::StepActivity) {
            let idx = FeatureOffset::StepCounter as usize + 1;
            feature_config[idx] &= u8::from(FeatureEnableMask::StepActivity.not());
        }

        if features.contains(Features::WristWear) {
            let idx = FeatureOffset::WristWear as usize;
            feature_config[idx] &= u8::from(FeatureEnableMask::WristWear.not());
        }

        if features.contains(Features::SingleTap) {
            let idx = FeatureOffset::SingleTap as usize;
            feature_config[idx] &= u8::from(FeatureEnableMask::SingleTap.not());
        }
        if features.contains(Features::DoubleTap) {
            let idx = FeatureOffset::DoubleTap as usize;
            feature_config[idx] &= u8::from(FeatureEnableMask::DoubleTap.not());
        }

        feature_config[0] = Reg::FeatureConfig.into();
        self.write(&feature_config)?;

        Ok(())
    }

    pub fn map_feature_interrupt(
        &mut self,
        line: InterruptLine,
        interrupts: FeatureInterruptStatus,
        enable: bool,
    ) -> Result<(), Error<E>> {
        let mut data: [u8; 2] = [0; 2];
        let addr = match line {
            InterruptLine::Line1 => Reg::FeatureInterrupt1Mapping,
            InterruptLine::Line2 => Reg::FeatureInterrupt2Mapping,
        };
        self.write_read(Reg::FeatureInterrupt1Mapping, &mut data)?;

        if enable {
            data[line as usize] |= u8::from(interrupts);
        } else {
            data[line as usize] &= u8::from(interrupts.not());
        }

        self.write(&[addr.into(), data[line as usize]])?;

        Ok(())
    }

    pub fn map_hardware_interrupt(
        &mut self,
        interrupts: HardwareInterruptStatus,
        enable: bool,
    ) -> Result<(), Error<E>> {
        let mut data: [u8; 1] = [0];

        self.write_read(Reg::HardwareInterruptMapping, &mut data)?;

        if enable {
            data[0] |= u8::from(interrupts);
        } else {
            data[0] &= u8::from(interrupts.not());
        }

        self.write(&[Reg::HardwareInterruptMapping.into(), data[0]])?;

        Ok(())
    }

    pub fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error<E>> {
        let mut data: [u8; 2] = [0; 2];
        self.write_read(Reg::FeatureInterruptStatus, &mut data)?;
        Ok(InterruptStatus {
            feature: data[0].into(),
            hardware: data[1].into(),
        })
    }

    pub fn get_status(&mut self) -> Result<u8, Error<E>> {
        let mut data: [u8; 1] = [0; 1];
        self.write_read(Reg::Status, &mut data)?;
        Ok(data[0])
    }

    pub fn get_x_y_z(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let mut data: [u8; 6] = [0; 6];
        self.write_read(Reg::AccXLSB, &mut data)?;

        let x: i16 = ((((data[1] as i16) << 8) as i16) | (data[0] as i16)) / 0x10;
        let y: i16 = ((((data[3] as i16) << 8) as i16) | (data[2] as i16)) / 0x10;
        let z: i16 = ((((data[5] as i16) << 8) as i16) | (data[4] as i16)) / 0x10;

        Ok((
            lsb_to_ms2(x, 2.0, 12),
            lsb_to_ms2(y, 2.0, 12),
            lsb_to_ms2(z, 2.0, 12),
        ))
    }
}

#[cfg(feature = "accel")]
impl<E, I2C> Accelerometer for Bma423<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    type Error = Error<E>;

    fn accel_norm(&mut self) -> Result<F32x3, accelerometer::Error<Error<E>>> {
        let (x, y, z) = self.get_x_y_z()?;
        Ok(F32x3::new(x, y, z))
    }

    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<Error<E>>> {
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
            None => Err(accelerometer::Error::from(Error::ConfigError)),
        }
    }
}

#[inline(always)]
fn lsb_to_ms2(val: i16, g_range: f32, bit_width: u8) -> f32 {
    let half_scale: f32 = (1 << bit_width) as f32 / 2.0;

    GRAVITY_EARTH * val as f32 * g_range / half_scale
}
