# lis2duxs12-rs
[![Crates.io][crates-badge]][crates-url]
[![BSD 3-Clause licensed][bsd-badge]][bsd-url]

[crates-badge]: https://img.shields.io/crates/v/lis2duxs12-rs
[crates-url]: https://crates.io/crates/lis2duxs12-rs
[bsd-badge]: https://img.shields.io/crates/l/lis2duxs12-rs
[bsd-url]: https://opensource.org/licenses/BSD-3-Clause

Provides a platform-agnostic, no_std-compatible driver for the ST LIS2DUXS12 sensor, supporting both I2C and SPI communication interfaces.

## Sensor Overview

The LIS2DUXS12 is a smart, digital, 3-axis linear accelerometer whose MEMS and ASIC have been expressly designed to combine the lowest current consumption possible with features such as always-on antialiasing filtering, a finite state machine (FSM) and machine learning core (MLC) with adaptive self-configuration (ASC), and an analog hub / Qvar sensing channel.

The FSM and MLC with ASC deliver outstanding always-on, edge processing capabilities to the LIS2DUXS12, while the analog hub / Qvar sensing channel defines a new degree of system optimization. The LIS2DUXS12 MIPI I3C® slave interface and embedded 128-level FIFO buffer complete a set of features that make this accelerometer a reference in terms of system integration from a standpoint of the bill of materials, processing, or power consumption.

The LIS2DUXS12 has user-selectable full scales of ±2g/±4g/±8g/±16g and is capable of measuring accelerations with output data rates from 1.6 Hz to 800 Hz.

The LIS2DUXS12 has a dedicated internal engine to process motion and acceleration detection including free-fall, wake-up, single/double/triple-tap recognition, activity/inactivity, and 6D/4D orientation.

The LIS2DUXS12 is available in a small thin plastic, land grid array package (LGA) and it is guaranteed to operate over an extended temperature range from -40°C to +85°C.

For more info, please visit the device page at [https://www.st.com/en/mems-and-sensors/lis2duxs12.html](https://www.st.com/en/mems-and-sensors/lis2duxs12.html)

## Installation

Add the driver to your `Cargo.toml` dependencies:

```toml
[dependencies]
lis2duxs12-rs = "0.1.0"
```

Or, add it directly from the terminal:

```sh
cargo add lis2duxs12-rs
```

## Usage

Include the crate and its prelude
```rust
use lis2duxs12_rs as lis2duxs12;
use lis2duxs12::*;
use lis2duxs12::prelude::*;
```

### Create an instance

Create an instance of the driver with the `new_<bus>` associated function, by passing an I2C (`embedded_hal::i2c::I2c`) instance and I2C address, or an SPI (`embedded_hal::spi::SpiDevice`) instance, along with a timing peripheral.

An example with I2C:

```rust
let mut sensor = Lis2duxs12::new_i2c(i2c, I2CAddress::I2cAddH, delay);
```

### Check "Who Am I" Register

This step ensures correct communication with the sensor. It returns a unique ID to verify the sensor's identity.

```rust
let whoami = sensor.device_id_get().unwrap();
if whoami != ID {
    panic!("Invalid sensor ID");
}
```

### Configure

See details in specific examples; the following are common api calls:

```rust
// Restore default configuration
sensor.init_set(Init::Reset).unwrap();

// Wait for reset to complete
while sensor.status_get().unwrap().sw_reset == 1 {}

// Set BDU and IF_INC recommended for driver usage
sensor.init_set(Init::SensorOnlyOn).unwrap();

// Set Output Data Rate
let md = Md {
    fs: Fs::_4g,
    bw: Bw::OdrDiv4,
    odr: Odr::_25hzLp,
};
sensor.mode_set(&md).unwrap();
```

## License

Distributed under the BSD-3 Clause license.

More Information: [http://www.st.com](http://st.com/MEMS).

**Copyright (C) 2025 STMicroelectronics**