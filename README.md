<p align="center">
    <a href="https://github.com/pyaillet/bma423-rs/actions/workflows/ci.yml"><img src="https://github.com/pyaillet/bma423-rs/actions/workflows/ci.yml/badge.svg?branch=main" alt="Build status" /></a>
    <a href="https://crates.io/crates/bma423"><img src="https://img.shields.io/crates/v/bma423.svg" alt="Crates.io"></a>
    <a href="https://docs.rs/bma423"><img src="https://docs.rs/bma423/badge.svg" alt="Docs.rs"></a>
</p>

# BMA423 Rust driver

This is an experimental Rust driver for the BMA423 accelerometer.

What's working:
- Getting x, y, z axis acceleration values
- The motion detection feature
- The tap detection feature
- Configuring the interrupt pins
- Mapping features to the interrupt pins (as outputs)
- Remapping the axes

What's missing:
- The step counter and step detection features
- Activity classification feature
- Wrist wakeup feature
- Auxiliary interface configuration
- FIFO configuration

## Examples

You can find an example usage in this project: [TTGO T-Watch v1 rust example](https://github.com/pyaillet/twatch-idf-rs).

## Contributing

This project is open to contributions of any form, do not hesitate to open an issue or a pull-request
if you have questions or suggestions.
