//! Items for configuring the features of the chip.
//!
//! The main item is [`EditFeatures`].

use crate::{Error, FullPower, Reg};
use bitmask_enum::bitmask;
use core::ops::RangeInclusive;

/// Size of the FEATURES_IN register in bytes.
pub(crate) const FEATURE_SIZE: usize = 70;

/// Offsets for components of the FEATURES_IN register.
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
enum FeatureOffset {
    AnyMotion = 0x00,
    NoMotion = 0x04,
    //StepCounterParam = 0x08,
    //StepCounter = 0x3A,
    SingleTap = 0x3C,
    DoubleTap = 0x3E,
    //WristWear = 0x40,
    //ConfigId = 0x42,
    AxesRemap = 0x44,
}

/// Which motion feature
#[derive(Copy, Clone, Debug)]
pub enum MotionFeature {
    /// Any motion detection feature
    AnyMotion,
    /// No motion detection feature
    NoMotion,
}

/// Valid range for motion thresholds.
const MOTION_THRESHOLD_RANGE: RangeInclusive<u16> = 0..=0x7FF;
/// Valid range for motion durations.
const MOTION_DURATION_RANGE: RangeInclusive<u16> = 0..=0x1FFF;

/// Motion feature axes.
///
/// Can be combined like a bitmask.
#[bitmask(u8)]
pub enum MotionAxes {
    AxisX = 0b0010_0000,
    AxisY = 0b0100_0000,
    AxisZ = 0b1000_0000,
}

/// New axis to which to remap an axis.
#[repr(u8)]
#[derive(Copy, Clone, Debug)]
pub enum RemapFromAxis {
    /// Positive x-axis
    AxisX = 0b000,
    /// Positive y-axis
    AxisY = 0b001,
    /// Positive z-axis
    AxisZ = 0b010,
    /// Negative x-axis
    AxisXMinus = 0b100,
    /// Negative y-axis
    AxisYMinus = 0b101,
    /// Negative z-axis
    AxisZMinus = 0b110,
}
impl core::ops::Neg for RemapFromAxis {
    type Output = Self;

    fn neg(self) -> Self::Output {
        match self {
            Self::AxisX => Self::AxisXMinus,
            Self::AxisY => Self::AxisYMinus,
            Self::AxisZ => Self::AxisZMinus,
            Self::AxisXMinus => Self::AxisX,
            Self::AxisYMinus => Self::AxisY,
            Self::AxisZMinus => Self::AxisZ,
        }
    }
}

/// Valid range for tap sensitivities.
const TAP_SENSITIVITY_RANGE: RangeInclusive<u8> = 0..=7;

/// Which tap feature
#[derive(Copy, Clone, Debug)]
pub enum TapFeature {
    /// Single tap detection feature
    SingleTap,
    /// Double tap detection feature
    DoubleTap,
}

/// Allows editing feature configurations in an efficient way.
///
/// This can be obtained from the driver by calling
/// [`edit_features`](crate::Bma423::edit_features).
/// Once the desired features are configured, one must
/// call [`write`](EditFeatures::write) to write the
/// configurations back to the chip so that they take
/// effect.
pub struct EditFeatures<'a, I2C> {
    /// FEATURES_IN register contents.
    pub(crate) register: [u8; FEATURE_SIZE + 1],
    /// Driver object for which features are being edited.
    pub(crate) driver: &'a mut crate::Bma423<I2C, FullPower>,
}
impl<I2C: embedded_hal::i2c::I2c> EditFeatures<'_, I2C> {
    /// Allows editing the features register without the offset.
    fn edit_register(&mut self, f: impl FnOnce(&mut [u8])) {
        f(&mut self.register[1..FEATURE_SIZE + 1]);
    }

    /// Sets the motion detection feature configuration.
    ///
    /// # Arguments
    ///
    /// - `which` Specifies which motion feature to configure.
    /// - `threshold` Slope threshold for motion detection.
    ///    The least significant bit is 0.48828125 mg, and the valid
    ///    range is 0 to 0x7FF (1 g).
    ///    The default value is 0xAA (83 mg).
    /// - `duration` The number of consecutive data points for which
    ///   the threshold condition must be respected.
    ///   In terms of time, the least significant bit is 20 ms, and the
    ///   valid range is 0 to 0x1FFF (163 s).
    ///   The default value is 5 (100 ms).
    /// - `enabled_axes` Axes for which to enable the motion detection.
    pub fn set_motion_config(
        &mut self,
        which: MotionFeature,
        threshold: u16,
        duration: u16,
        enabled_axes: MotionAxes,
    ) -> Result<(), Error<I2C::Error>> {
        // Validate arguments
        if !MOTION_THRESHOLD_RANGE.contains(&threshold)
            || !MOTION_DURATION_RANGE.contains(&duration)
        {
            return Err(Error::BadArgument);
        }

        self.edit_register(|reg| {
            let offset = match which {
                MotionFeature::AnyMotion => FeatureOffset::AnyMotion,
                MotionFeature::NoMotion => FeatureOffset::NoMotion,
            } as usize;

            // Set the threshold
            let threshold = threshold.to_le_bytes();
            reg[offset] = threshold[0];
            reg[offset + 1] = threshold[1];

            // Set the duration
            let duration = duration.to_le_bytes();
            reg[offset + 2] = duration[0];
            reg[offset + 3] = duration[1];

            // Set enabled axes
            reg[offset + 3] = u8::from(enabled_axes) & 0xE0;
        });

        Ok(())
    }

    /// Sets the tap detection feature configuration.
    ///
    /// Unless the axes are remapped (see [`remap_axes`](EditFeatures::remap_axes)), tap detection is in the z-axis.
    ///
    /// # Arguments
    ///
    /// - `which` Specifies which tap feature to configure.
    /// - `sensitivity` A sensitivity level from 0 (most sensitive) to
    ///   7 (least sensitive).
    ///   The default value is 3.
    /// - `enable` Whether to enable or disable tap detection.
    pub fn set_tap_config(
        &mut self,
        which: TapFeature,
        sensitivity: u8,
        enable: bool,
    ) -> Result<(), Error<I2C::Error>> {
        if !TAP_SENSITIVITY_RANGE.contains(&sensitivity) {
            return Err(Error::BadArgument);
        }

        self.edit_register(|reg| {
            let offset = match which {
                TapFeature::SingleTap => FeatureOffset::SingleTap,
                TapFeature::DoubleTap => FeatureOffset::DoubleTap,
            } as usize;

            reg[offset] = sensitivity << 1;
            if enable {
                reg[offset] |= 1;
            }
        });

        Ok(())
    }

    /// Remaps the accelerometer axes.
    ///
    /// Note that remapping multiple axes from the same axis can result in strange behavior.
    ///
    /// # Arguments
    ///
    /// - `x_axis` Specifies which physical axis to map onto the x-axis
    /// - `y_axis` Specifies which physical axis to map onto the y-axis
    /// - `z_axis` Specifies which physical axis to map onto the z-axis
    ///
    /// # Example
    ///
    /// ```compile_fail,no_run
    /// edit_features.remap_axes(RemapFromAxis::AxisY, -RemapFromAxis::AxisZ, -RemapFromAxis::AxisX);
    /// ```
    /// In the above example, action in the physical positive y-axis direction
    /// would register on the positive x-axis.
    /// Similarly action in the physical negative z-axis direction would
    /// register on the positive y-axis, and action in the physical negative
    /// x-axis direction would register on the positive z-axis.
    pub fn remap_axes(
        &mut self,
        x_axis: RemapFromAxis,
        y_axis: RemapFromAxis,
        z_axis: RemapFromAxis,
    ) {
        let val = (x_axis as u16) | (y_axis as u16) << 3 | (z_axis as u16) << 6;

        self.edit_register(|reg| {
            let bytes = val.to_le_bytes();
            reg[FeatureOffset::AxesRemap as usize] = bytes[0];
            reg[FeatureOffset::AxesRemap as usize + 1] = bytes[1];
        })
    }

    /// Writes the edited features back to the chip so that they take effect.
    pub fn write(mut self) -> Result<(), Error<I2C::Error>> {
        self.register[0] = Reg::FeatureConfig.into();
        self.driver.write(&self.register)
    }
}
