//! Driver for the BMA323 accelerometer chip by BOSCH.
//!
//! Refer to the [datasheet](https://www.mouser.com/datasheet/2/783/BSCH_S_A0010021471_1-2525113.pdf)
//! and the [features application note](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mas-an032-00.pdf)
//! for more details about the device and its rich feature set.
//! Be aware that the current version of the datasheet is 2.0,
//! and there are many copies of the incorrect version 1.1
//! floating around online.
#![no_std]

use core::ops::RangeInclusive;

#[cfg(feature = "accel")]
use accelerometer::{vector::F32x3, Accelerometer};
use bitmask_enum::bitmask;
use derive_new::new;
use embedded_hal::{delay::DelayUs, i2c::I2c};
use num_enum::{FromPrimitive, IntoPrimitive};

mod config;
pub mod features;

use features::{EditFeatures, FEATURE_SIZE};

/// Chip registers
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

    InternalStatus = 0x2a,

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
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum AccelRange {
    Range2g = 0x00,
    Range4g = 0x01,
    Range8g = 0x02,
    Range16g = 0x03,
}
impl AccelRange {
    pub fn as_float(&self) -> f32 {
        match self {
            AccelRange::Range2g => 2.0,
            AccelRange::Range4g => 4.0,
            AccelRange::Range8g => 8.0,
            AccelRange::Range16g => 16.0,
        }
    }
}

/// Feature interrupt status
#[bitmask(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum FeatureInterruptStatus {
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

/// Hardware interrupt status
#[bitmask(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
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

/// Interrupt line
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptLine {
    Line1 = 0,
    Line2 = 1,
}

/// Interrupt trigger condition when configured as an input
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptTriggerCondition {
    Level = 0x00,
    Edge = 0x01,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptLevel {
    ActiveLow = 0x00,
    ActiveHigh = 0x02,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptOutputBehavior {
    PushPull = 0x00,
    OpenDrain = 0x04,
}

#[derive(Clone, Debug)]
pub enum InterruptDirection {
    Input(InterruptTriggerCondition),
    Output(InterruptOutputBehavior, InterruptLevel),
}
impl InterruptDirection {
    fn bit_mask(&self) -> u8 {
        match self {
            InterruptDirection::Input(_) => 0x10,
            InterruptDirection::Output(_, _) => 0x08,
        }
    }
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
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum PowerControlFlag {
    Accelerometer = Self(0b0000_0100),
    Auxiliary = Self(0b0000_0001),
}

#[bitmask(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum PowerConfigurationFlag {
    AdvancedPowerSave = Self(0b0000_0001),
    FifoSelfWakeUp = Self(0b0000_0010),
}

const DEFAULT_ADDRESS: u8 = 0x18;
const READ_WRITE_LEN: usize = 0x08;
const GRAVITY_EARTH: f32 = 9.80665;

#[derive(Clone, Copy, Debug)]
pub enum Error<E> {
    BusError(E),
    I2cError,
    ConfigError,
    BadInternal(u8),
    Uninitialized,
    BadArgument,
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

/// Structure representing the Bma423 device
pub struct Bma423<I2C> {
    /// I2C address
    address: u8,
    /// I2C peripheral
    i2c: I2C,
    /// Current state
    state: State,
    /// Current configuration
    config: Config,
}

/// Configuration of the device
#[allow(dead_code)]
#[derive(new, Copy, Clone, Debug)]
pub struct Config {
    pub bandwidth: AccelConfigBandwidth,
    pub range: AccelRange,
    pub performance_mode: AccelConfigPerfMode,
    pub sample_rate: AccelConfigOdr,
}
impl Default for Config {
    /// The default configuration is:
    /// * Filter: Average of 4 samples
    /// * Range: 2g
    /// * Performance mode: Averaging
    /// * Sample rate: 50 Hz
    fn default() -> Self {
        Self {
            bandwidth: AccelConfigBandwidth::NormAvg4,
            range: AccelRange::Range2g,
            performance_mode: AccelConfigPerfMode::CicAvg,
            sample_rate: AccelConfigOdr::Odr50,
        }
    }
}

pub enum State {
    Uninitialized,
    Initialized(ChipId),
}

impl<I2C: I2c> Bma423<I2C> {
    /// Create a new Bma423 device with the default slave address (0x18) and configuration.
    ///
    /// # Arguments
    ///
    /// - `i2c` I2C bus used to communicate with the device
    ///
    /// # Returns
    ///
    /// - [Bma423 driver](Bma423) created
    ///
    #[inline(always)]
    pub fn new(i2c: I2C) -> Self {
        Self::new_with_address(i2c, DEFAULT_ADDRESS)
    }

    /// Create a new Bma423 device with a particular slave address and default
    /// configuration.
    ///
    /// # Arguments
    ///
    /// - `i2c` I2C bus used to communicate with the device
    /// - `address: address of the device
    ///
    /// # Returns
    ///
    /// - [Bma423 driver](Bma423) created
    ///
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self {
            address,
            i2c,
            state: State::Uninitialized,
            config: Config::default(),
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, data)?;
        Ok(())
    }

    fn write_read(&mut self, reg: Reg, data: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.address, &[reg.into()], data)?;
        Ok(())
    }

    /// Reads only a single byte from a register
    fn read_register(&mut self, reg: Reg) -> Result<u8, Error<I2C::Error>> {
        let mut data = [0; 1];
        self.write_read(reg, &mut data)?;
        Ok(data[0])
    }

    fn probe_chip(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut data: [u8; 1] = [0; 1];
        self.write_read(Reg::ChipId, &mut data)?;
        Ok(data[0])
    }

    pub fn set_power_control(&mut self, value: PowerControlFlag) -> Result<(), Error<I2C::Error>> {
        self.write(&[Reg::PowerControl.into(), value.into()])
    }

    pub fn set_power_config(
        &mut self,
        value: PowerConfigurationFlag,
    ) -> Result<(), Error<I2C::Error>> {
        self.write(&[Reg::PowerConfiguration.into(), value.into()])
    }

    pub fn get_chip_id(&mut self) -> Result<ChipId, Error<I2C::Error>> {
        match self.state {
            State::Uninitialized => Err(Error::Uninitialized),
            State::Initialized(chip_id) => Ok(chip_id),
        }
    }

    pub fn init(&mut self, delay: &mut impl DelayUs) -> Result<(), Error<I2C::Error>> {
        let chip_id = self.probe_chip()?;
        self.state = State::Initialized(chip_id.into());

        // First, perform a soft reset followed by an arbitrary delay
        self.write(&[Reg::Command.into(), Command::SoftReset.into()])?;
        delay.delay_us(1000);

        // Disable advanced power saving mode
        self.set_power_config(PowerConfigurationFlag::none())?;

        // Wait a bit
        delay.delay_us(500);

        // Enter config file writing mode
        self.write(&[Reg::StartInitialization.into(), 0])?;

        // Stream write the config file
        self.stream_write(Reg::FeatureConfig, &config::BMA423_CONFIG_FILE)?;

        // Exit config file writing mode
        self.write(&[Reg::StartInitialization.into(), 1u8])?;

        // Wait until the chip is ready and initialized with timeout.
        // This is supposed to take no longer than 140-150 ms.
        let mut time_ms: usize = 200;
        while time_ms > 0 && (self.read_register(Reg::InternalStatus)? & 0x0F) != 0x01 {
            delay.delay_us(1000);
            time_ms -= 1;
        }
        if time_ms == 0 {
            // We timed out, so there's a serious problem
            return Err(Error::BadInternal(self.read_register(Reg::InternalStatus)?));
        }

        // Enable accelerometer power
        self.set_power_control(PowerControlFlag::Accelerometer)?;

        // Set the configuration to the default
        self.set_accel_config(self.config)?;

        Ok(())
    }

    fn stream_write(&mut self, reg: Reg, data: &[u8]) -> Result<(), Error<I2C::Error>> {
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

    pub fn set_accel_config(&mut self, config: Config) -> Result<(), Error<I2C::Error>> {
        if config.performance_mode == AccelConfigPerfMode::Continuous {
            if (config.bandwidth as u8) > (AccelConfigBandwidth::NormAvg4 as u8) {
                return Err(Error::ConfigError);
            }
        } else if config.performance_mode == AccelConfigPerfMode::CicAvg {
            if (config.bandwidth as u8) > (AccelConfigBandwidth::ResAvg128 as u8) {
                return Err(Error::ConfigError);
            }
        } else {
            return Err(Error::ConfigError);
        }

        let accel_config: u8 =
            config.sample_rate as u8 | config.bandwidth as u8 | config.performance_mode as u8;
        let accel_range: u8 = config.range as u8;
        self.write(&[Reg::AccelConfig.into(), accel_config])?;
        self.write(&[Reg::AccelRange.into(), accel_range])?;

        self.config = config;
        Ok(())
    }

    /// Obtains an [`EditFeatures`] that can be used to configure the chip features.
    pub fn edit_features(&mut self) -> Result<EditFeatures<'_, I2C>, Error<I2C::Error>> {
        let mut register = [0; FEATURE_SIZE + 1];
        self.write_read(Reg::FeatureConfig, &mut register[1..FEATURE_SIZE + 1])?;

        Ok(EditFeatures {
            register,
            driver: self,
        })
    }

    /// Configures the electrical behavior of an interrupt pin.
    ///
    /// # Arguments
    ///
    /// - `line` Which interrupt line to configure.
    /// - `direction` Whether to configure the interrupt line as
    /// an input or an output.
    pub fn set_interrupt_config(
        &mut self,
        line: InterruptLine,
        direction: InterruptDirection,
    ) -> Result<(), Error<I2C::Error>> {
        let reg = direction.bit_mask()
            | match direction {
                InterruptDirection::Input(tc) => u8::from(tc),
                InterruptDirection::Output(ob, l) => u8::from(ob) | u8::from(l),
            };
        let addr = match line {
            InterruptLine::Line1 => Reg::Interrupt1IOCtl,
            InterruptLine::Line2 => Reg::Interrupt2IOCtl,
        };
        self.write(&[addr.into(), reg])?;

        Ok(())
    }

    // TODO: These are for verifying that feature and other registers are being set correctly
    /* pub fn get_features_mem(&mut self) -> Result<[u8; FEATURE_SIZE], Error<I2C::Error>> {
        // TODO: Get FEATURES_IN memory
        let mut feature_config: [u8; FEATURE_SIZE] = [0; FEATURE_SIZE];
        self.write_read(Reg::FeatureConfig, &mut feature_config)?;
        Ok(feature_config)
    }

    // TODO delete probably
    pub fn test_regs(&mut self) -> Result<[u8; 4], Error<I2C::Error>> {
        let a = self.read_register(Reg::Interrupt1IOCtl)?;
        let b = self.read_register(Reg::Interrupt2IOCtl)?;
        let c = self.read_register(Reg::FeatureInterrupt1Mapping)?;
        let d = self.read_register(Reg::FeatureInterrupt2Mapping)?;
        Ok([a, b, c, d])
    } */

    pub fn map_feature_interrupt(
        &mut self,
        line: InterruptLine,
        interrupts: FeatureInterruptStatus,
        enable: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let addr = match line {
            InterruptLine::Line1 => Reg::FeatureInterrupt1Mapping,
            InterruptLine::Line2 => Reg::FeatureInterrupt2Mapping,
        };
        let mut reg = self.read_register(addr)?;

        if enable {
            reg |= u8::from(interrupts);
        } else {
            reg &= u8::from(interrupts.not());
        }

        self.write(&[addr.into(), reg])?;

        Ok(())
    }

    pub fn map_hardware_interrupt(
        &mut self,
        interrupts: HardwareInterruptStatus,
        enable: bool,
    ) -> Result<(), Error<I2C::Error>> {
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

    pub fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        let mut data: [u8; 2] = [0; 2];
        self.write_read(Reg::FeatureInterruptStatus, &mut data)?;
        Ok(InterruptStatus {
            feature: data[0].into(),
            hardware: data[1].into(),
        })
    }

    pub fn get_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        let mut data: [u8; 1] = [0; 1];
        self.write_read(Reg::Status, &mut data)?;
        Ok(data[0])
    }

    /// Returns the normalized accelerations in g.
    fn accel_norm_int(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut data: [u8; 6] = [0; 6];
        self.write_read(Reg::AccXLSB, &mut data)?;

        let x: i16 = ((((data[1] as i16) << 8) as i16) | (data[0] as i16)) / 0x10;
        let y: i16 = ((((data[3] as i16) << 8) as i16) | (data[2] as i16)) / 0x10;
        let z: i16 = ((((data[5] as i16) << 8) as i16) | (data[4] as i16)) / 0x10;

        let range = self.config.range.as_float();

        Ok((
            lsb_to_ms2(x, range, 12),
            lsb_to_ms2(y, range, 12),
            lsb_to_ms2(z, range, 12),
        ))
    }

    /// Returns the current x, y, z accelerations in absolute units of m/s^2.
    pub fn accel_abs(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let accel = self.accel_norm_int()?;

        Ok((
            GRAVITY_EARTH * accel.0,
            GRAVITY_EARTH * accel.1,
            GRAVITY_EARTH * accel.2,
        ))
    }
}

#[cfg(feature = "accel")]
impl<I2C: I2c> Accelerometer for Bma423<I2C> {
    type Error = Error<I2C::Error>;

    fn accel_norm(&mut self) -> Result<F32x3, accelerometer::Error<Error<I2C::Error>>> {
        Ok(self.accel_norm_int()?.into())
    }

    fn sample_rate(&mut self) -> Result<f32, accelerometer::Error<Error<I2C::Error>>> {
        match self.config.sample_rate {
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
        }
    }
}

#[inline(always)]
fn lsb_to_ms2(val: i16, g_range: f32, bit_width: u8) -> f32 {
    let half_scale: f32 = (1 << bit_width) as f32 / 2.0;

    val as f32 * g_range / half_scale
}
