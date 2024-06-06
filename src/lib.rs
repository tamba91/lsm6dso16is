//! This crate provides a platform-agnostic driver for the ST LSM6DSO16IS accelerometer-gyroscope sensor.
//! The datasheet and other documentation is available at <https://www.st.com/en/mems-and-sensors/lsm6dso16is.html>.
//! This driver was built using the [embedded-hal](https://docs.rs/embedded-hal/1.0.0/embedded_hal/) traits.
//! Ensure that the hardware abstraction layer of your microcontroller implements the embedded-hal traits.
//!
//! ## Instantiating
//!
//! Create an instance of the driver with the `new_i2c` or `new_spi` associated function, by passing i2c and address instances
//! or an spi (SpiDevice) instance.
//! 
//! ### I2C:
//!
//!```rust
//! use lsm6dso16is::{Lsm6dso16is, I2CAddress};
//!
//! let mut sensor = Lsm6dso16is::new_i2c(i2c, I2CAddress::Address0).unwrap();
//! ```
//! 
//! i2c instance must implement the I2c trait of embedded-hal. The address is an enum variant of I2CAddress enum.
//! There are two addresses available: 0x6A and 0x6B, which correspond to enum variants Address0 and Address1.
//! Check the datasheet for I2C address configuration.
//!
//! If multiple sensors are used on the same I2C bus, create an instance of i2c that implements bus sharing:
//! (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
//!```rust
//! use core::cell::RefCell;
//! use embedded_hal_bus::i2c;
//!
//! use lsm6dso16is::{Lsm6dso16is, I2CAddress as Lsm6dso16isAddress};
//! use stts22h::{Stts22h, I2CAddress as Stts22hAddress}; // the STTS22H is another sensor of ST MEMS family
//!
//! let i2c_bus = RefCell::new(i2c);
//!```
//! and then create an instance of i2c for each sensor connected. 
//!```rust
//! let mut lps22df = Lps22df::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Lsm6dso16isAddress::Address0).unwrap();
//! let mut stts22h = Stts22h::new_i2c(i2c::RefCellDevice::new(&i2c_bus), Stts22hAddress::Address0).unwrap();
//!```
//! 
//! In this example sharing is implemented with a RefCell, so it only allows sharing within a single thread.
//! If you need to share a bus across several threads, use CriticalSectionDevice instead.
//!
//! ### SPI:
//!
//! HALs normally implement SpiBus trait. In order to obtain an SpiDevice from an SpiBus embedded-hal-bus crate can be used.
//! (from [`embedded-hal-bus`](https://docs.rs/embedded-hal-bus), or others).
//! ```rust
//! use embedded_hal_bus::spi::RefCellDevice;
//! use lsm6dso16is::Lsm6dso16is;
//! let spi = RefCellDevice::new_no_delay(&spi, cs).unwrap(); // cs is the chip select pin
//! let mut sensor = Lsm6dso16is::new_spi(spi).unwrap();
//! ```
//! 
//! ## Setting ODR (output data rate) and and FS (full scale) for the accelerometer and the gyroscope:
//!
//! ```rust
//! use lsm6dso16is::{Lsm6dso16is, XlFullScale, GFullScale};
//! 
//! sensor.set_odr_xl(104.0, false).unwrap(); // accelerometer odr 125Hz, low power mode disabled
//! sensor.set_odr_g(12.5, false).unwrap(); // gyroscope odr 12.5Hz, low power mode disabled
//! sensor.set_fs_g(GFullScale::Dps125).unwrap(); // gyroscope full scale 125 degree per second
//! sensor.set_fs_xl(XlFullScale::G2).unwrap(); // // accelerometer full scale 2g
//! ```
//! 
//! Read acceleration and angular rate data when available
//! 
//! ```rust
//! loop {
//!     if sensor.is_xl_data_avail().unwrap() == true {
//!         let xl_axes = sensor.get_xl_axes().unwrap();
//!         writeln!(tx, "x: {} mg, y: {} mg, z: {} mg", xl_axes.0, xl_axes.1, xl_axes.2).unwrap(); // acceleration data for x, y, z axes in millig
//!     }
//!     if sensor.is_g_data_avail().unwrap() == true {
//!         let g_axes = sensor.get_g_axes().unwrap();
//!         writeln!(tx, "x: {} dps, y: {} dps, z: {} dsp", g_axes.0, g_axes.1, g_axes.2).unwrap(); // degree per second data for x, y, z axes
//!     }
//!      
//! }
//! ```
//!  
//! the values are then printed on a generic uart interface
//!

#![no_std]
use bitfield::bitfield;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::SpiDevice;
use embedded_hal::delay::DelayNs;
use self::lsm6dso16is_reg::{Ctrl1XlOdr, Ctrl2GOdr};

mod lsm6dso16is_reg;

///
/// The LSM6DSO16IS driver struct.
///
pub struct Lsm6dso16is<B> {
    bus: B,
}

    ///
    /// Constructor method (associated function) for using the I2C bus. This method checks for the presence of the sensor on the bus
    /// and returns a new driver instance if the sensor responds with the correct identifier.
    ///
    /// # Arguments
    ///
    /// * `i2c` - an I2C peripheral instance.
    /// * `address` - an I2C address enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * Self: The sensor driver instance.
    ///     * Error: If a wrong identifier is received (!= 0x22) an Error::WhoAmIError(u8) is returned.
    ///              The error contains the wrong number received.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
impl<P: I2c> Lsm6dso16is<lsm6dso16is_reg::Lsm6dso16isI2C<P>> {
    pub fn new_i2c(i2c: P, address: I2CAddress) -> Result<Self, Error<P::Error>> {
        let bus = lsm6dso16is_reg::Lsm6dso16isI2C::new(i2c, address as SevenBitAddress);
        let mut instance = Self { bus };
        let who = instance.who_am_i_get()?;
        if who != 0x22 {
            return Err(Error::WhoAmIError(who));
        }
        instance.ctrl3_c_set_sw_reset()?;
        while instance.ctrl3_c_get_sw_reset()? != 0 {}
        instance.ctrl3_c_set_bdu(true as u8)?;

        Ok(instance)
    }
}

impl<P: SpiDevice> Lsm6dso16is<lsm6dso16is_reg::Lsm6dso16isSPI<P>> {

    ///
    /// Constructor method (associated function) for using the SPI bus. This method checks for the presence of the sensor on the bus
    /// and returns a new driver instance if the sensor responds with the correct identifier.
    ///
    /// # Arguments
    ///
    /// * `spi` - an SPI peripheral instance.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * Self: The sensor driver instance.
    ///     * Error: If a wrong identifier is received (!= 0x22) an Error::WhoAmIError(u8) is returned.
    ///              The error contains the wrong number received.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn new_spi(spi: P) -> Result<Self, Error<P::Error>> {
        let bus = lsm6dso16is_reg::Lsm6dso16isSPI::new(spi);
        let mut instance = Self { bus };
        let who = instance.who_am_i_get()?;
        if who != 0x22 {
            return Err(Error::WhoAmIError(who));
        }
        instance.ctrl3_c_set_sw_reset()?;
        while instance.ctrl3_c_get_sw_reset()? != 0 {}
        instance.ctrl3_c_set_bdu(true as u8)?;

        Ok(instance)
    }
    
}

///
/// Available I2C addresses for the LSM6DSO16IS sensor. Check the datasheet for I2C address configuration.
///
#[repr(u8)]
pub enum I2CAddress {
    Address0 = 0x6A,
    Address1 = 0x6B,
}

///
/// Driver errors.
///
#[derive(Copy, Clone, Debug)]
pub enum Error<B> {
    /// An error occurred at the bus level. Any methods that access the I2C/SPI bus to interact with the sensor may return this error
    /// if the bus operation fails.
    /// The generic type B represents the specific error generated by the HAL of the microcontroller in use.
    Bus(B),
    /// The `who_am_i` method returned an incorrect sensor identifier (the LSM6DSO16IS identifier is 0x22).
    WhoAmIError(u8),
    /// The attempt to write to a register failed,
    /// resulting in a discrepancy between the intended value and the actual value stored in the register.
    WriteFailure,
    /// An invalid ISPU register value was passed to the `read_ispu_output` method.
    InvalidIspuRegValue,
}

///
/// Accelerometer full scale selection in g.
/// 
#[repr(u8)]
pub enum XlFullScale {
    G2 = 0b00,
    G4 = 0b10,
    G8 = 0b11,
    G16 = 0b01,
}
///
/// Gyroscope full scale selection in degree per second.
/// 
#[repr(u8)]
pub enum GFullScale {
    Dps125 = 0b001,
    Dps250 = 0b000,
    Dps500 = 0b010,
    Dps1000 = 0b100,
    Dps2000 = 0b110,
}

///
/// Signal Type for INT1 pin and INT2 pin. This enum is used by the method `set_signal_mode`to specify the behavior for the INT1 pin and
/// INT2 pin, when the accelerometer or the gyroscope data-ready signal to pin is enabled through the methods `enable_xl_drdy_to_pin`
/// and `enable_g_drdy_to_pin`.
///
#[repr(u8)]
pub enum SignalMode {
    // The pin remains asserted until the new data value is read  with the method any of the `get_xl...` or `get_g...` methods.
    Latched = 0b0,
    /// The pin is asserted for approximately 75 μs (pulse) when a new data value is available. After this time, the pin clears itself.
    Pulsed = 0b1,
}

///
/// This enum represents the INT1 and INT2 pins.
/// 
pub enum Pin {
    Int1,
    Int2,
}

pub struct UcfLineExtT {
    pub op: Op,
    pub address: u8,
    pub data: u8,
}

#[repr(u8)]
pub enum Op {
    MemsUcfOpWrite = 1,
    MemsUcfOpDelay = 2,
}

bitfield! {
    pub struct IspuIntStatus(u32);
    pub ia_ispu_0, _: 0, 0;
    pub ia_ispu_1, _: 1, 1;
    pub ia_ispu_2, _: 2, 2;
    pub ia_ispu_3, _: 3, 3;
    pub ia_ispu_4, _: 4, 4;
    pub ia_ispu_5, _: 5, 5;
    pub ia_ispu_6, _: 6, 6;
    pub ia_ispu_7, _: 7, 7;
    pub ia_ispu_8, _: 8, 8;
    pub ia_ispu_9, _: 9, 9;
    pub ia_ispu_10, _: 10, 10;
    pub ia_ispu_11, _: 11, 11;
    pub ia_ispu_12, _: 12, 12;
    pub ia_ispu_13, _: 13, 13;
    pub ia_ispu_14, _: 14, 14;
    pub ia_ispu_15, _: 15, 15;
    pub ia_ispu_16, _: 16, 16;
    pub ia_ispu_17, _: 17, 17;
    pub ia_ispu_18, _: 18, 18;
    pub ia_ispu_19, _: 19, 19;
    pub ia_ispu_20, _: 20, 20;
    pub ia_ispu_21, _: 21, 21;
    pub ia_ispu_22, _: 22, 22;
    pub ia_ispu_23, _: 23, 23;
    pub ia_ispu_24, _: 24, 24;
    pub ia_ispu_25, _: 25, 25;
    pub ia_ispu_26, _: 26, 26;
    pub ia_ispu_27, _: 27, 27;
    pub ia_ispu_28, _: 28, 28;
    pub ia_ispu_29, _: 29, 29;
}

impl<B: lsm6dso16is_reg::BusOperation> Lsm6dso16is<B> {

    ///
    /// Method that returns the sensor identifier (0x22).
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * u8: The sensor identifier number (0x22).
    ///     * Error: If a wrong identifier is received (!= 0x22) an Error::WhoAmIError(u8) is returned.
    ///              The error contains the wrong number received.
    ///              The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn who_am_i(&mut self) -> Result<u8, Error<B::Error>> {
        let res = self.who_am_i_get()?;

        if res != 0x22 {
            return Err(Error::WhoAmIError(res));
        }

        Ok(res)
    }

    ///
    /// Method that returns the current ODR (output data rate) for the accelerometer.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * f32: The accelerometer current ODR (output data rate). If 0.0 is returned the accelerometer is in power-down mode.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn get_odr_xl(&mut self) -> Result<f32, Error<B::Error>> {
        let odr: Ctrl1XlOdr = self.ctrl1_xl_get_odr_xl()?.into();

        Ok(odr.into())
    }

    ///
    /// Method that sets the ODR (output data rate) for the accelerometer.
    ///     
    /// # Arguments
    ///
    /// * odr: a 32 bit float number representing the desired odr.
    ///        The available ODRs are:
    ///        0.0: sets the accelerometer in power-down mode, 1.6Hz (low power mode only) 12.5Hz (low power and high performance), 
    ///        26.0Hz (low power and high performance), 52.0Hz (low power and high performance), 104.0Hz (low power and high performance),
    ///        208.0Hz (low power and high performance), 416.0Hz (high performance only), 833.0Hz (high performance only), 
    ///        1667.0Hz (high performance only), 3333.0Hz (high performance only) 6667.0Hz (high performance only).
    /// * enable_low_power_mode: bool, if true low power mode is enabled (not all ODRs are available)
    ///                           
    /// # Note
    ///
    /// Passing an ODR value from the list will set the accelerometer to this exact ODR.
    /// If an ODR value outside the list is passed, it will be rounded to the next greater value.
    /// If a value greater than 6667.0 (or 208.0 if enable_low_power_mode is true) is passed, the ODR will be rounded to 6667.0
    /// (or 208.0).
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example 1
    ///   
    /// ```rust
    /// sensor.set_odr_xl(104.0, false).unwrap(); // the odr is set at 104.0Hz, (low power mode disabled)
    /// let current_odr = sensor.get_odr_xl().unwrap(); // current_odr is 104.0Hz
    /// ```
    ///
    /// # Example 2
    ///   
    /// ```rust
    /// sensor.set_odr_xl(50.0, false).unwrap(); // 50.0 is not a value in the list, the odr is set at 52.0 Hz (low power mode disabled)
    /// let current_odr = sensor.get_odr_xl().unwrap(); // current_odr is 52.0
    /// ```
    ///
    /// # Example 3
    ///   
    /// ```rust
    /// sensor.set_odr_xl(0.0, false).unwrap();; // the accelerometer is put in power-down mode
    /// let current_odr = sensor.get_odr_xl().unwrap(); // current_odr is 0.0
    /// ```
    ///
    pub fn set_odr_xl(
        &mut self,
        odr: f32,
        enable_low_power_mode: bool,
    ) -> Result<(), Error<B::Error>> {
        match enable_low_power_mode {
            true => {
                if odr != 0.0 && odr > 208.0 {
                    let odr: Ctrl1XlOdr = 208.0.into();
                    self.ctrl1_xl_set_odr_xl(odr as u8)?;
                } else {
                    let odr: Ctrl1XlOdr = odr.into();
                    self.ctrl1_xl_set_odr_xl(odr as u8)?;
                }
                self.ctrl6_c_set_xl_hm_mode(true as u8)?;
            }
            false => {
                if odr != 0.0 && odr < 12.5 {
                    let odr: Ctrl1XlOdr = 12.5.into();
                    self.ctrl1_xl_set_odr_xl(odr as u8)?;
                } else {
                    let odr: Ctrl1XlOdr = odr.into();
                    self.ctrl1_xl_set_odr_xl(odr as u8)?;
                }
                self.ctrl6_c_set_xl_hm_mode(false as u8)?;
            }
        }

        Ok(())
    }

    ///
    /// This method returns the current accelerometer sensitivity. The sensitivity is used as the multiplier 
    /// when converting a two's complement integer number which represents a raw acceleration
    /// to obtain an acceleration value in mg.
    /// 
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * f32: The accelerometer current sensitivity value, according to the full scale
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn get_sensitivity_xl(&mut self) -> Result<f32, Error<B::Error>> {
        let fs: XlFullScale = self.ctrl1_xl_get_fs_xl()?.into();

        Ok(fs.into())
    }

    ///
    /// This method sets the current accelerometer full scale. 
    /// 
    /// # Arguments
    ///
    /// * fs a XlFullScale enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn set_fs_xl(&mut self, fs: XlFullScale) -> Result<(), Error<B::Error>> {
        self.ctrl1_xl_set_fs_xl(fs as u8)?;

        Ok(())
    }

    ///
    /// Method that returns the current ODR (output data rate) for the gyroscope.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * f32: The gyroscope current ODR (output data rate). If 0.0 is returned the gyroscope is in power-down mode.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn get_odr_g(&mut self) -> Result<f32, Error<B::Error>> {
        let odr: Ctrl2GOdr = self.ctrl2_g_get_odr_g()?.into();

        Ok(odr.into())
    }

    ///
    /// Method that sets the ODR (output data rate) for the gyroscope.
    ///     
    /// # Arguments
    ///
    /// * odr: a 32 bit float number representing the desired odr.
    ///        The available ODRs are:
    ///        0.0: sets the gyroscope in power-down mode, 12.5Hz (low power and high performance), 
    ///        26.0Hz (low power and high performance), 52.0Hz (low power and high performance), 104.0Hz (low power and high performance),
    ///        208.0Hz (low power and high performance), 416.0Hz (high performance only), 833.0Hz (high performance only), 
    ///        1667.0Hz (high performance only), 3333.0Hz (high performance only) 6667.0Hz (high performance only).
    /// * enable_low_power_mode: bool, if true low power mode is enabled (not all ODRs are available)
    ///                           
    /// # Note
    ///
    /// Passing an ODR value from the list will set the gyroscope to this exact ODR.
    /// If an ODR value outside the list is passed, it will be rounded to the next greater value.
    /// If a value greater than 6667.0 (or 208.0 if enable_low_power_mode is true) is passed, the ODR will be rounded to 6667.0
    /// (or 208.0).
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example 1
    ///   
    /// ```rust
    /// sensor.set_odr_g(104.0, false).unwrap(); // the odr is set at 104.0 Hz, (low power mode disabled)
    /// let current_odr = sensor.get_odr_g().unwrap(); // current_odr is 104.0Hz
    /// ```
    ///
    /// # Example 2
    ///   
    /// ```rust
    /// sensor.set_odr_g(50.0, false).unwrap(); // 50.0 is not a value in the list, the odr is set at 52.0 Hz (low power mode disabled)
    /// let current_odr = sensor.get_odr_g().unwrap(); // current_odr is 52.0
    /// ```
    ///
    /// # Example 3
    ///   
    /// ```rust
    /// sensor.set_odr_g(0.0, false).unwrap();; // the accelerometer is put in power-down mode
    /// let current_odr = sensor.get_odr_g().unwrap(); // current_odr is 0.0
    /// ```
    ///
    pub fn set_odr_g(
        &mut self,
        odr: f32,
        enable_low_power_mode: bool,
    ) -> Result<(), Error<B::Error>> {
        self.ctrl3_c_set_bdu(true as u8)?;
        match enable_low_power_mode {
            true => {
                if odr != 0.0 && odr > 208.0 {
                    let odr: Ctrl2GOdr = 208.0.into();
                    self.ctrl2_g_set_odr_g(odr as u8)?;
                } else {
                    let odr: Ctrl2GOdr = odr.into();
                    self.ctrl2_g_set_odr_g(odr as u8)?;
                }
                self.ctrl7_g_set_g_hm_mode(true as u8)?;
            }
            false => {
                let odr: Ctrl2GOdr = odr.into();
                self.ctrl2_g_set_odr_g(odr as u8)?;
                self.ctrl7_g_set_g_hm_mode(false as u8)?;
            }
        }

        Ok(())
    }

    ///
    /// This method returns the current gyroscope sensitivity. The sensitivity is used as the multiplier 
    /// when converting a two's complement integer number which represents a raw angular value
    /// to obtain a value in degree per second.
    /// 
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * f32: The gyroscope current sensitivity value, according to the full scale
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn get_sensitivity_g(&mut self) -> Result<f32, Error<B::Error>> {
        let fs: GFullScale = self.ctrl2_g_get_fs_g()?.into();

        Ok(fs.into())
    }

    ///
    /// This method sets the current gyroscope full scale. 
    /// 
    /// # Arguments
    ///
    /// * fs a GFullScale enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn set_fs_g(&mut self, fs: GFullScale) -> Result<(), Error<B::Error>> {
        self.ctrl2_g_set_fs_g(fs as u8)?;

        Ok(())
    }

    ///
    /// Method that returns true if an unread temperature sample is available.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: if true an unread temperature sample is available, if false no unread temperature sample is available.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn is_temp_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_reg_get_tda()?;

        Ok(val != 0)
    }

    ///
    /// Method that returns the temperature value.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * f32: temperature in °C. The value is expressed as 32-bit floating point.           
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///  
    pub fn get_temp(&mut self) -> Result<f32, Error<B::Error>> {
        let raw_temp = self.out_temp_get()?;

        Ok(raw_temp as f32 / 256.0 + 25.0)
    }

    ///
    /// Method that returns true if an unread accelerometer sample is available.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: if true an unread accelerometer sample is available, if false no unread accelerometer sample is available.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn is_xl_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_reg_get_xlda()?;

        Ok(val != 0)
    }

    ///
    /// Method that returns the raw acceleration value for the x axis.
    /// This method avoids floating point opeation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i16: raw acceleration value for the x axis. The value is expressed as two’s complement 16-bit integer.
    ///            To obtain the real acceleration in mg multiply by the sensitivity obtained from `get_sensitivity_xl` method.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_xl_x_axis_raw(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(self.outx_a_get()?)
    }

    ///
    /// Method that returns the raw acceleration value for the y axis.
    /// This method avoids a floating point operation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i16: raw acceleration value for the y axis. The value is expressed as two’s complement 16-bit integer.
    ///            To obtain the real acceleration in mg multiply by the sensitivity obtained from `get_sensitivity_xl` method.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_xl_y_axis_raw(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(self.outy_a_get()?)
    }

    ///
    /// Method that returns the raw acceleration value for the z axis.
    /// This method avoids a floating point operation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i16: raw acceleration value for the z axis. The value is expressed as two’s complement 16-bit integer.
    ///            To obtain the real acceleration in mg multiply by the sensitivity obtained from `get_sensitivity_xl` method.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_xl_z_axis_raw(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(self.outz_a_get()?)
    }

    ///
    /// Method that returns the raw accelerometer value for each axis.
    /// This method avoids a floating point operation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * (i16, i16, i16): Raw accelerometer axes value (x axis, y axis, z axis). 
    ///                        The values are expressed as two’s complement 16-bit integer.
    ///                        To obtain the real accelerometer values multiply each member of the tuple by the sensitivity 
    ///                        obtained from `get_sensitivity_xl` method
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    /// 
    pub fn get_xl_axes_raw(&mut self) -> Result<(i16, i16, i16), Error<B::Error>> {
        Ok(self.outxyz_a_get()?)
    }

    ///
    /// Method that returns the accelerometer value for each axis.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * (f32, f32, f32): accelerometer axes value (x axis, y axis, z axis). 
    ///                        The values are expressed as f32 floating point number.
    ///                        
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_xl_axes(&mut self) -> Result<(f32, f32, f32), Error<B::Error>> {
        let xl_raw_values: (i16, i16, i16) = self.outxyz_a_get()?;
        let fs: XlFullScale = self.ctrl1_xl_get_fs_xl()?.into();
        let sensitivity: f32 = fs.into();

        Ok((
            xl_raw_values.0 as f32 * sensitivity,
            xl_raw_values.1 as f32 * sensitivity,
            xl_raw_values.2 as f32 * sensitivity,
        ))
    }

    ///
    /// Method that returns true if an unread gyroscope sample is available.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: if true an unread gyroscope sample is available, if false no unread gyroscope sample is available.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn is_g_data_avail(&mut self) -> Result<bool, Error<B::Error>> {
        let val = self.status_reg_get_gda()?;

        Ok(val != 0)
    }

    ///
    /// Method that returns the raw degree per second value for the x axis.
    /// This method avoids a floating point operation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i16: raw degree per second value for the x axis. The value is expressed as two’s complement 16-bit integer.
    ///            To obtain the real degree per second in dps multiply by the sensitivity obtained from `get_sensitivity_g` method.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_g_x_axis_raw(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(self.outx_g_get()?)
    }

    ///
    /// Method that returns the raw degree per second value for the y axis.
    /// This method avoids a floating point operation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i16: raw degree per second value for the y axis. The value is expressed as two’s complement 16-bit integer.
    ///            To obtain the real degree per second in dps multiply by the sensitivity obtained from `get_sensitivity_g` method.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_g_y_axis_raw(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(self.outy_g_get()?)
    }

    ///
    /// Method that returns the raw degree per second value for the z axis.
    /// This method avoids a floating point operation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * i16: raw degree per second value for the z axis. The value is expressed as two’s complement 16-bit integer.
    ///            To obtain the real degree per second in dps multiply by the sensitivity obtained from `get_sensitivity_g` method.
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_g_z_axis_raw(&mut self) -> Result<i16, Error<B::Error>> {
        Ok(self.outz_g_get()?)
    }

    ///
    /// Method that returns the raw gyroscope value for each axis.
    /// This method avoids a floating point operation.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * (i16, i16, i16): Raw gyroscope axes value (x axis, y axis, z axis). 
    ///                        The values are expressed as two’s complement 16-bit integer.
    ///                        To obtain the real gyroscope values multiply each member of the tuple by the sensitivity 
    ///                        obtained from `get_sensitivity_g` method
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_g_axes_raw(&mut self) -> Result<(i16, i16, i16), Error<B::Error>> {
        Ok(self.outxyz_g_get()?)
    }

    ///
    /// Method that returns the gyroscope value for each axis.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * (f32, f32, f32): gyroscope axes value (x axis, y axis, z axis). 
    ///                        The values are expressed as f32 floating point number.
    ///                        
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn get_g_axes(&mut self) -> Result<(f32, f32, f32), Error<B::Error>> {
        let g_raw_values: (i16, i16, i16) = self.outxyz_g_get()?;
        let fs: GFullScale = self.ctrl2_g_get_fs_g()?.into();
        let sensitivity: f32 = fs.into();

        Ok((
            g_raw_values.0 as f32 * sensitivity,
            g_raw_values.1 as f32 * sensitivity,
            g_raw_values.2 as f32 * sensitivity,
        ))
    }

    ///
    /// This method sets the signal mode for the INT1/2 pin. See SignalMode enum documentation.
    ///      
    /// # Arguments
    ///
    /// * signal_mode: a SignalMode enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn set_signal_mode(&mut self, signal_mode: SignalMode) -> Result<(), Error<B::Error>> {
        self.drdy_pulsed_reg_set_drdy_pulsed_reg(signal_mode as u8)?;

        Ok(())
    }

    ///
    /// Method that enables the accelerometer data ready signal on the INT1/2 pin.
    ///     
    /// # Arguments
    ///
    /// * pin: a Pin enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: If true an unread accelerometer sample is already available when the data ready signal to INT1/2 pin is enabled.
    ///             If false no unread data sample is available when the data ready signal to INT1/2 is enabled.    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example 
    /// 
    /// This example demonstrates how the data ready signal for the accelerometer can be driven to INT1 pin (INT2 is the same).
    /// When a new sample from the accelerometer is available the pin is asserted.
    /// 
    /// ```rust
    /// sensor.set_signal_mode(SignalMode::Pulsed).unwrap(); // the pin remains asserted for 75 μs (pulse). After this time, the pin clears itself
    /// sensor.enable_xl_drdy_to_int(Pin::Int1).unwrap(); // when a new accelerometer sample is available INT1 pin is asserted
    /// sensor.set_odr_xl(1.6, true).unwrap(); // the odr is set at 1.6Hz, (low power mode enabled)
    /// ```
    /// 
    pub fn enable_xl_drdy_to_pin(&mut self, pin: Pin) -> Result<bool, Error<B::Error>> {
        let val = self.outz_h_a_get()?;
        match pin {    
            Pin::Int1 => self.int1_ctrl_set_int1_drdy_xl(true as u8)?,       
            Pin::Int2 => self.int2_ctrl_set_int2_drdy_xl(true as u8)?,  
        }

        Ok(val != 0)
    }

    ///
    /// Method that enables the gyroscope data ready signal on the INT1/2 pin.
    /// 
    /// # Arguments
    ///
    /// * pin: a Pin enum variant.
    ///
    /// # Returns
    ///
    /// * Result
    ///     * bool: If true an unread gyroscope sample is already available when the data ready signal to INT1/2 pin is enabled.
    ///             If false no unread data sample is available when the data ready signal to INT1/2 is enabled.    
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    /// # Example 
    /// 
    /// This example demonstrates how the data ready signal for the gyroscope can be driven to INT1 pin (INT2 is the same).
    /// When a new sample from the gyroscope is available the pin is asserted.
    /// 
    /// ```rust
    /// sensor.set_signal_mode(SignalMode::Pulsed).unwrap(); // the pin remains asserted for 75 μs (pulse). After this time, the pin clears itself
    /// sensor.enable_g_drdy_to_int(Pin::Int1).unwrap(); // when a new accelerometer sample is available INT1 pin is asserted
    /// sensor.set_odr_g(104.0, false).unwrap(); // the odr is set at 104.0Hz, (low power mode disabled)
    /// ```
    pub fn enable_g_drdy_to_pin(&mut self, pin: Pin) -> Result<bool, Error<B::Error>> {
        let val = self.outz_h_g_get()?;
        match pin {            
            Pin::Int1 => self.int1_ctrl_set_int1_drdy_g(true as u8)?,
            Pin::Int2 => self.int2_ctrl_set_int2_drdy_g(true as u8)?,
        }

        Ok(val != 0)
    }

    ///
    /// Method that disables the accelerometer data ready signal on the INT1/2 pin.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()     
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_xl_drdy_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        self.int1_ctrl_set_int1_drdy_xl(false as u8)?;
        self.int2_ctrl_set_int2_drdy_xl(false as u8)?;

        Ok(())
    }

    ///
    /// Method that disables the gyroscope data ready signal on the INT1/2 pin.
    ///     
    /// # Arguments
    ///
    /// * None
    ///
    /// # Returns
    ///
    /// * Result
    ///     * ()     
    ///     * Error: The failure of a bus operation returns Error::Bus(B).
    ///
    pub fn disable_g_drdy_to_pin(&mut self) -> Result<(), Error<B::Error>> {
        self.int1_ctrl_set_int1_drdy_g(false as u8)?;
        self.int2_ctrl_set_int2_drdy_g(false as u8)?;

        Ok(())
    }

    pub fn load_ispu_bytecode<T: DelayNs>(
        &mut self,
        raw: &[UcfLineExtT],
        delay: &mut T,
    ) -> Result<(), Error<B::Error>> {
        self.func_cfg_access_set_sw_reset_ispu(true as u8)?;
        self.func_cfg_access_set_sw_reset_ispu(false as u8)?;
        for i in raw {
            match i.op {
                Op::MemsUcfOpWrite => {
                    self.write_raw_bytes(&[i.address as u8, i.data as u8])?;
                }
                Op::MemsUcfOpDelay => delay.delay_ms(i.data as u32),
            }
        }

        Ok(())
    }

    pub fn read_ispu_output(
        &mut self,
        start_reg: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<B::Error>> {
        if start_reg < 0x10 || start_reg as usize + buf.len() > 0x4F {
            return Err(Error::InvalidIspuRegValue);
        }
        self.func_cfg_access_set_ispu_reg_access(true as u8)?;
        self.write_raw_bytes(&[start_reg])?;
        self.read_raw_bytes(buf)?;
        self.func_cfg_access_set_ispu_reg_access(false as u8)?;

        Ok(())
    }

    pub fn get_ispu_int_status(&mut self) -> Result<IspuIntStatus, Error<B::Error>> {
        let ispu_int_status = IspuIntStatus(self.ispu_int_status_mainpage_get()?);

        Ok(ispu_int_status)
    }
}
