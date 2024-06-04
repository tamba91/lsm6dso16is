 This crate provides a platform-agnostic driver for the ST LSM6DSO16IS accelerometer-gyroscope sensor.
 The datasheet and other documentation is available at <https://www.st.com/en/mems-and-sensors/lsm6dso16is.html>.
 This driver was built using the [embedded-hal](https://docs.rs/embedded-hal/1.0.0/embedded_hal/) traits.
 Ensure that the hardware abstraction layer of your microcontroller implements the embedded-hal traits.

 ## Instantiating

 Create an instance of the driver with the `new_i2c` or `new_spi` associated function, by passing i2c and address instances
 or an spi (SpiDevice) instance.
 
 ### I2C:

```rust
 use lsm6dso16is::{Lsm6dso16is, I2CAddress};
-
 let mut sensor = Lsm6dso16is::new_i2c(i2c, I2CAddress::Address0).unwrap();
 ```
 
 i2c instance must implement the I2c trait of embedded-hal. The address is an enum variant of I2CAddress enum.
 There are two addresses available: 0x6A and 0x6B, which correspond to enum variants Address0 and Address1.
 Check the datasheet for I2C address configuration.

 If multiple sensors are used on the same I2C bus, create an instance of i2c that implements bus sharing:
 (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
```rust
 use core::cell::RefCell;
 use embedded_hal_bus::i2c;

 use lsm6dso16is::{Lsm6dso16is, I2CAddress as Lsm6dso16isAddress};
 use stts22h::{Stts22h, I2CAddress as Stts22hAddress}; // the STTS22H is another sensor of ST MEMS family

 let i2c_bus = RefCell::new(i2c);
```
 and then create an instance of i2c for each sensor connected. 
```rust
 let mut lps22df = Lps22df::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Lsm6dso16isAddress::Address0).unwrap();
 let mut stts22h = Stts22h::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Stts22hAddress::Address0).unwrap();
```
 
 In this example sharing is implemented with a RefCell, so it only allows sharing within a single thread.
 If you need to share a bus across several threads, use CriticalSectionDevice instead.

 ### SPI:

 HALs normally implement SpiBus trait. In order to obtain an SpiDevice from an SpiBus embedded-hal-bus crate can be used.
 (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
 ```rust
 use embedded_hal_bus::spi::RefCellDevice;
 use lsm6dso16is::Lsm6dso16is;
 let spi = RefCellDevice::new_no_delay(&spi, cs).unwrap(); // cs is the chip select pin
 let mut sensor = Lsm6dso16is::new_spi(spi).unwrap();
 ```
 
 ## Setting ODR (output data rate) and and FS (full scale) for the accelerometer and the gyroscope:

 ```rust
 use lsm6dso16is::{Lsm6dso16is, XlFullScale, GFullScale};
 
 sensor.set_odr_xl(104.0, false).unwrap(); // accelerometer odr 125Hz, low power mode disabled
 sensor.set_odr_g(12.5, false).unwrap(); // gyroscope odr 12.5Hz, low power mode disabled
 sensor.set_fs_g(GFullScale::Dps125).unwrap(); // gyroscope full scale 125 degree per second
 sensor.set_fs_xl(XlFullScale::G2).unwrap(); // // accelerometer full scale 2g
 ```
 
 Read acceleration and angular rate data when available
 
 ```rust
 loop {
     if sensor.is_xl_data_avail().unwrap() == true {
         let xl_axes = sensor.get_xl_axes().unwrap();
         writeln!(tx, "x: {} mg, y: {} mg, z: {} mg", xl_axes.0, xl_axes.1, xl_axes.2).unwrap(); // acceler. data for x, y, z axes in millig
     }
     if sensor.is_g_data_avail().unwrap() == true {
         let g_axes = sensor.get_g_axes().unwrap();
         writeln!(tx, "x: {} dps, y: {} dps, z: {} dsp", g_axes.0, g_axes.1, g_axes.2).unwrap(); // degree per second data for x, y, z axes
     }
      
 }
 ```
  
 the values are then printed on a generic uart interface
