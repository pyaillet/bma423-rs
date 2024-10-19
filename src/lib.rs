//! Driver for the BMA323 accelerometer chip by BOSCH.
//!
//! Refer to the [datasheet](https://www.mouser.com/datasheet/2/783/BSCH_S_A0010021471_1-2525113.pdf)
//! and the [features application note](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-mas-an032-00.pdf)
//! for more details about the device and its rich feature set.
//! Be aware that the current version of the datasheet is 2.0,
//! and there are many copies of the incorrect version 1.1
//! floating around online.
#![no_std]
// NOTE: Evidently there is no way to document bitmask enums,
// nor can these warnings be disabled for them, so these
// should be used as needed but commented out for commits.
//#![warn(missing_docs)]
//#![warn(clippy::missing_docs_in_private_items)]

#[cfg(feature = "accel")]
use accelerometer::{vector::F32x3, Accelerometer};
use bitmask_enum::bitmask;
use embedded_hal::{delay::DelayNs, i2c::I2c};
use num_enum::{FromPrimitive, IntoPrimitive};

mod config;
pub mod features;

use features::{EditFeatures, FEATURE_SIZE};

/// Chip registers.
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

/// Accelerometer sampling rates.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum AccelConfigOdr {
    /// 0.78 Hz
    Odr0p78 = 0x01,
    /// 1.5 Hz
    Odr1p5 = 0x02,
    /// 3.1 Hz
    Odr3p1 = 0x03,
    /// 6.25 Hz
    Odr6p25 = 0x04,
    /// 12.5 Hz
    Odr12p5 = 0x05,
    /// 25 Hz
    Odr25 = 0x06,
    /// 50 Hz
    Odr50 = 0x07,
    /// 100 Hz
    Odr100 = 0x08,
    /// 200 Hz
    Odr200 = 0x09,
    /// 400 Hz
    Odr400 = 0x0a,
    /// 800 Hz
    Odr800 = 0x0b,
    /// 1.6 kHz
    Odr1k6 = 0x0c,
}

/// Accelerometer sample bandwidth and averaging.
///
/// Refer the the data sheet for further details.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum AccelConfigBandwidth {
    /// Performance mode: OSR4, Low power mode: No averaging
    Osr4Avg1 = 0x00,
    /// Performance mode: OSR2, Low power mode: Average 2 samples
    Osr2Avg2 = 0x10,
    /// Performance mode: Normal, Low power mode: Average 4 samples
    NormAvg4 = 0x20,
    /// Low power mode: Average 8 samples
    CicAvg8 = 0x30,
    /// Low power mode: Average 16 samples
    ResAvg16 = 0x40,
    /// Low power mode: Average 32 samples
    ResAvg32 = 0x50,
    /// Low power mode: Average 64 samples
    ResAvg64 = 0x60,
    /// Low power mode: Average 128 samples
    ResAvg128 = 0x70,
}

/// Accelerometer filter mode
#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, IntoPrimitive)]
pub enum AccelConfigPerfMode {
    /// Averaging mode
    CicAvg = 0x00,
    /// Continuous filter mode
    Continuous = 0x80,
}

/// Accelerometer acceleration vector range.
///
/// Values measured outside this range will be clipped to
/// the range. Bear in mind that the signed range is quantized
/// into 12 bits so that smaller ranges will have more
/// resolution.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum AccelRange {
    /// Max range of ±2 g
    Range2g = 0x00,
    /// Max range of ±4 g
    Range4g = 0x01,
    /// Max range of ±8 g
    Range8g = 0x02,
    /// Max range of ±16 g
    Range16g = 0x03,
}
impl AccelRange {
    /// Returns the range as a floating point in g.
    pub fn as_float(&self) -> f32 {
        match self {
            AccelRange::Range2g => 2.0,
            AccelRange::Range4g => 4.0,
            AccelRange::Range8g => 8.0,
            AccelRange::Range16g => 16.0,
        }
    }
}

/// Feature interrupt status.
#[bitmask(u8)]
pub enum FeatureInterruptStatus {
    /* Taken from examples */
    SingleTap = 0b0000_0001,
    StepCounter = 0b0000_0010,
    Activity = 0b0000_0100,
    WristWear = 0b0000_1000,
    DoubleTap = 0b0001_0000,
    AnyMotion = 0b0010_0000,
    NoMotion = 0b0100_0000,
    Error = 0b1000_0000,
}

/// Hardware interrupt status.
#[bitmask(u8)]
pub enum HardwareInterruptStatus {
    FifoFull = 0x01,
    FifoWatermark = 0x02,
    DataReady = 0x04,
    AuxiliaryDataReady = 0x20,
    AcceleratorDataReady = 0x80,
}

/// Interrupt status structure.
#[derive(Copy, Clone, Debug)]
pub struct InterruptStatus {
    /// Feature interrupt status
    pub feature: FeatureInterruptStatus,
    /// Hardware interrupt status
    pub hardware: HardwareInterruptStatus,
}

/// Which interrupt line.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptLine {
    /// Interrupt line 1
    Line1 = 0,
    /// Interrupt line 2
    Line2 = 1,
}

/// Interrupt trigger condition when configured as an input.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptTriggerCondition {
    /// Trigger when the level reaches a value
    Level = 0x00,
    /// Trigger only on an edge
    Edge = 0x01,
}

/// Interrupt pin output level.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptLevel {
    /// The interrupt line will be low when active and high otherwise
    ActiveLow = 0x00,
    /// The interrupt line will be high when active and low otherwise
    ActiveHigh = 0x02,
}

/// Interrupt pin output behavior.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
pub enum InterruptOutputBehavior {
    /// Push pull output, i.e. the pin drives the line voltage
    PushPull = 0x00,
    /// Open drain output, i.e. the pin acts as a ground switch,
    /// requiring that it be pulled up externally
    OpenDrain = 0x04,
}

/// Direction of an interrupt pin, i.e. an input or and output.
///
/// The configuration for each are the parameters.
#[derive(Clone, Debug)]
pub enum InterruptDirection {
    /// Configure as an input with a trigger condition.
    Input(InterruptTriggerCondition),
    /// Configure as an output with a behavior and level.
    Output(InterruptOutputBehavior, InterruptLevel),
}
impl InterruptDirection {
    /// Bit mask for the direction.
    fn bit_mask(&self) -> u8 {
        match self {
            InterruptDirection::Input(_) => 0x10,
            InterruptDirection::Output(_, _) => 0x08,
        }
    }
}

/// Activity types.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive, FromPrimitive)]
enum Activity {
    Stationary = 0x00,
    Walking = 0x01,
    Running = 0x02,
    #[default]
    Invalid = 0x03,
}

/// Chip commands.
#[allow(dead_code)]
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive)]
enum Command {
    NvmProg = 0xa0,
    FifoFlush = 0xb0,
    SoftReset = 0xb6,
}

#[bitmask(u8)]
pub enum PowerControlFlag {
    Accelerometer = 0b0000_0100,
    Auxiliary = 0b0000_0001,
}

#[bitmask(u8)]
enum PowerConfigurationFlag {
    AdvancedPowerSave = 0b0000_0001,
    FifoSelfWakeUp = 0b0000_0010,
}

/// Default I2C address of the chip.
const DEFAULT_ADDRESS: u8 = 0x18;
/// Size of burst data for loading the config file in bytes.
const READ_WRITE_LEN: usize = 0x08;
/// Acceleration due to Earth gravity in m/s^2.
const GRAVITY_EARTH: f32 = 9.80665;

/// General accelerometer error.
///
/// The generic `E` is the error type of the I2C driver.
#[derive(Clone, Copy, Debug)]
pub enum Error<E> {
    /// I2C bus error
    BusError(E),
    /// Tried to set an invalid configuration
    ConfigError,
    /// The chip is in an erroneous internal state
    BadInternal(u8),
    /// An invalid argument was passed to a function
    BadArgument,
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

/// Accelerometer chip ID.
#[repr(u8)]
#[derive(Copy, Clone, Debug, IntoPrimitive, FromPrimitive)]
pub enum ChipId {
    /// Unknown or invalid chip ID
    #[default]
    Unknown = 0x00,
    /// The chip ID is correct
    Bma423 = 0x13,
}

/// Uninitialized state.
pub struct Uninitialized;
/// Normal full power state.
pub struct FullPower;
/// Advanced power save state.
pub struct PowerSave;

/// A general initialized state.
pub trait Initialized {}
impl Initialized for FullPower {}
impl Initialized for PowerSave {}

/// Structure representing the BMA423 chip.
///
/// This ensures a correct initialization and a consistent
/// state at every moment using type states, which are zero
/// cost abstractions.
pub struct Bma423<I2C, S> {
    /// I2C address.
    address: u8,
    /// I2C peripheral.
    i2c: I2C,
    /// Current configuration.
    config: Config,
    /// State of the chip.
    #[allow(dead_code)]
    state: S,
}

/// Configuration of the accelerometer measurements.
#[allow(dead_code)]
#[derive(Copy, Clone, Debug)]
pub struct Config {
    /// The bandwidth or averaging mode
    pub bandwidth: AccelConfigBandwidth,
    /// The range of measurable accelerations
    pub range: AccelRange,
    /// Filter mode
    pub performance_mode: AccelConfigPerfMode,
    /// Sampling rate
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
            range: AccelRange::Range4g,
            performance_mode: AccelConfigPerfMode::CicAvg,
            sample_rate: AccelConfigOdr::Odr50,
        }
    }
}

impl<I2C: I2c, S> Bma423<I2C, S> {
    /// Writes bytes to the device over I2C.
    ///
    /// Generally the first byte should be the register address.
    fn write(&mut self, data: &[u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write(self.address, data)?;
        Ok(())
    }

    /// Reads some number of bytes from a chip register over
    /// I2C.
    ///
    /// The number of bytes is determined from the size of the
    /// mutable slice `data`.
    fn read_register_bytes(&mut self, reg: Reg, data: &mut [u8]) -> Result<(), Error<I2C::Error>> {
        self.i2c.write_read(self.address, &[reg.into()], data)?;
        Ok(())
    }

    /// Reads only a single byte from a chip register.
    fn read_register(&mut self, reg: Reg) -> Result<u8, Error<I2C::Error>> {
        let mut data = [0; 1];
        self.read_register_bytes(reg, &mut data)?;
        Ok(data[0])
    }

    /// Sets or clears one or more power configuration flags.
    fn set_power_config(
        &mut self,
        value: PowerConfigurationFlag,
        set: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let mut reg = self.read_register(Reg::PowerConfiguration)?;
        if set {
            reg |= u8::from(value);
        } else {
            reg &= !u8::from(value);
        }
        self.write(&[Reg::PowerConfiguration.into(), reg])
    }
}
impl<I2C: I2c> Bma423<I2C, Uninitialized> {
    /// Writes stream data to a chip register using the
    /// special ASIC registers.
    ///
    /// This should only be used to write the config file.
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

    /// Create a new Bma423 device with the default slave address (0x18) and configuration.
    ///
    /// # Arguments
    ///
    /// - `i2c` I2C bus used to communicate with the device
    /// - `config` Initial accelerometer configuration to use
    ///
    /// # Returns
    ///
    /// - [Bma423 driver](Bma423) created
    ///
    #[inline(always)]
    pub fn new(i2c: I2C, config: Config) -> Self {
        Self::new_with_address(i2c, config, DEFAULT_ADDRESS)
    }

    /// Create a new Bma423 device with a particular slave address and default
    /// configuration.
    ///
    /// # Arguments
    ///
    /// - `i2c` I2C bus used to communicate with the device
    /// - `config` Initial accelerometer configuration to use
    /// - `address: address of the device
    ///
    /// # Returns
    ///
    /// - [Bma423 driver](Bma423) created
    ///
    pub fn new_with_address(i2c: I2C, config: Config, address: u8) -> Self {
        Self {
            address,
            i2c,
            config,
            state: Uninitialized,
        }
    }

    /// Initialize the chip by going through its initialization procedure.
    pub fn init(
        mut self,
        delay: &mut impl DelayNs,
    ) -> Result<Bma423<I2C, FullPower>, Error<I2C::Error>> {
        // First, perform a soft reset followed by an arbitrary delay
        self.write(&[Reg::Command.into(), Command::SoftReset.into()])?;
        delay.delay_ms(1);

        // Disable advanced power saving mode
        self.set_power_config(PowerConfigurationFlag::all_bits(), false)?;

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

        let mut driver = Bma423 {
            address: self.address,
            i2c: self.i2c,
            config: self.config,
            state: FullPower,
        };

        // Enable accelerometer power
        driver.set_power_control(PowerControlFlag::Accelerometer)?;

        // Set the configuration to the default
        driver.set_accel_config(delay, driver.config)?;

        Ok(driver)
    }
}
impl<I2C: I2c> Bma423<I2C, FullPower> {
    /// Obtains an [`EditFeatures`] that can be used to configure the chip
    /// features efficiently.
    pub fn edit_features(&mut self) -> Result<EditFeatures<'_, I2C>, Error<I2C::Error>> {
        let mut register = [0; FEATURE_SIZE + 1];
        self.read_register_bytes(Reg::FeatureConfig, &mut register[1..FEATURE_SIZE + 1])?;

        Ok(EditFeatures {
            register,
            driver: self,
        })
    }

    // TODO: This is a test function useful for verifying that feature
    // registers are set correctly. This should be removed once all
    // features are implemented.
    /* pub fn read_features_mem(&mut self) -> Result<[u8; FEATURE_SIZE], Error<I2C::Error>> {
        let mut feature_config: [u8; FEATURE_SIZE] = [0; FEATURE_SIZE];
        self.read_register_bytes(Reg::FeatureConfig, &mut feature_config)?;
        Ok(feature_config)
    } */

    /// Transitions to advanced power save mode.
    pub fn power_save_mode(self) -> Result<Bma423<I2C, PowerSave>, Error<I2C::Error>> {
        let mut driver = Bma423 {
            address: self.address,
            i2c: self.i2c,
            config: self.config,
            state: PowerSave,
        };
        driver.set_power_config(PowerConfigurationFlag::AdvancedPowerSave, true)?;

        Ok(driver)
    }
}
impl<I2C: I2c> Bma423<I2C, PowerSave> {
    /// Transitions to full power mode.
    pub fn full_power_mode(self) -> Result<Bma423<I2C, FullPower>, Error<I2C::Error>> {
        let mut driver = Bma423 {
            address: self.address,
            i2c: self.i2c,
            config: self.config,
            state: FullPower,
        };
        driver.set_power_config(PowerConfigurationFlag::AdvancedPowerSave, false)?;

        Ok(driver)
    }
}
impl<I2C: I2c, S: Initialized> Bma423<I2C, S> {
    /// Sets which components of the chip have power applied.
    pub fn set_power_control(&mut self, value: PowerControlFlag) -> Result<(), Error<I2C::Error>> {
        self.write(&[Reg::PowerControl.into(), value.into()])
    }

    /// Returns the chip ID enum.
    pub fn read_chip_id(&mut self) -> Result<ChipId, Error<I2C::Error>> {
        Ok(ChipId::from(self.read_register(Reg::ChipId)?))
    }

    /// Configures the accelerometer measurements.
    ///
    /// A delay is required to allow time for the new
    /// configuration to take effect.
    pub fn set_accel_config(
        &mut self,
        delay: &mut impl DelayNs,
        config: Config,
    ) -> Result<(), Error<I2C::Error>> {
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

        // This seems to be adequate even if the data rate is low.
        delay.delay_ms(50);

        Ok(())
    }

    /// Configures the electrical behavior of an interrupt pin.
    ///
    /// # Arguments
    ///
    /// - `line` Which interrupt line to configure.
    /// - `direction` Whether to configure the interrupt line as
    ///   an input or an output.
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

    /// Maps features to one of the chip interrupt pins.
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

    /// Enables hardware interrupts.
    pub fn map_hardware_interrupt(
        &mut self,
        interrupts: HardwareInterruptStatus,
        enable: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let mut reg = self.read_register(Reg::HardwareInterruptMapping)?;

        if enable {
            reg |= u8::from(interrupts);
        } else {
            reg &= u8::from(interrupts.not());
        }

        self.write(&[Reg::HardwareInterruptMapping.into(), reg])?;

        Ok(())
    }

    /// Reads and returns the interrupt status from the chip.
    pub fn read_interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        let mut data: [u8; 2] = [0; 2];
        self.read_register_bytes(Reg::FeatureInterruptStatus, &mut data)?;
        Ok(InterruptStatus {
            feature: data[0].into(),
            hardware: data[1].into(),
        })
    }

    /// Reads and returns the general chip status byte.
    pub fn read_status(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(Reg::Status)
    }

    /// Enables or disables FIFO self wake up.
    pub fn set_fifo_self_wakeup(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        self.set_power_config(PowerConfigurationFlag::FifoSelfWakeUp, enable)
    }

    /// Returns the normalized accelerations in g.
    pub fn accel_norm_int(&mut self) -> Result<(f32, f32, f32), Error<I2C::Error>> {
        let mut data: [u8; 6] = [0; 6];
        self.read_register_bytes(Reg::AccXLSB, &mut data)?;

        let x: i16 = (((data[1] as i16) << 8) | (data[0] as i16)) / 0x10;
        let y: i16 = (((data[3] as i16) << 8) | (data[2] as i16)) / 0x10;
        let z: i16 = (((data[5] as i16) << 8) | (data[4] as i16)) / 0x10;

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
impl<I2C: I2c, S: Initialized> Accelerometer for Bma423<I2C, S> {
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

/// Converts fixed point to a float for a given acceleration range and bit depth.
#[inline(always)]
fn lsb_to_ms2(val: i16, g_range: f32, bit_width: u8) -> f32 {
    let half_scale: f32 = (1 << bit_width) as f32 / 2.0;

    val as f32 * g_range / half_scale
}
