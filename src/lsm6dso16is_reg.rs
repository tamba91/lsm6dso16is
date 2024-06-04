use bitfield::bitfield;
use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_hal::spi::{Operation, SpiDevice};

use super::Error;
use super::{GFullScale, XlFullScale};
pub struct Lsm6dso16isI2C<P> {
    i2c: P,
    address: SevenBitAddress,
}

impl<P: I2c> Lsm6dso16isI2C<P> {
    pub(super) fn new(i2c: P, address: SevenBitAddress) -> Self {
        Self { i2c, address }
    }
}

pub struct Lsm6dso16isSPI<P> {
    spi: P,
}

impl<P: SpiDevice> Lsm6dso16isSPI<P> {
    pub(super) fn new(spi: P) -> Self {
        Self { spi }
    }
}

pub trait BusOperation {
    type Error;

    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error>;
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error>;
    fn write_byte_read_bytes(&mut self, wbuf: &[u8; 1], rbuf: &mut [u8])
        -> Result<(), Self::Error>;
}

impl<P: I2c> BusOperation for Lsm6dso16isI2C<P> {
    type Error = P::Error;

    #[inline]
    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.read(self.address, rbuf)?;

        Ok(())
    }

    #[inline]
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.i2c.write(self.address, wbuf)?;

        Ok(())
    }

    #[inline]
    fn write_byte_read_bytes(&mut self, wbuf: &[u8; 1], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.i2c.write_read(self.address, wbuf, rbuf)?;

        Ok(())
    }
}

impl<P: SpiDevice> BusOperation for Lsm6dso16isSPI<P> {
    type Error = P::Error;

    #[inline]
    fn read_bytes(&mut self, rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [Operation::Read(rbuf)])?;

        Ok(())
    }

    #[inline]
    fn write_bytes(&mut self, wbuf: &[u8]) -> Result<(), Self::Error> {
        self.spi.transaction(&mut [Operation::Write(wbuf)])?;

        Ok(())
    }

    #[inline]
    fn write_byte_read_bytes(&mut self, wbuf: &[u8; 1], rbuf: &mut [u8]) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [Operation::Write(&[wbuf[0] | 0x80]), Operation::Read(rbuf)])?;

        Ok(())
    }
}

impl<B: BusOperation> super::Lsm6dso16is<B> {
    fn read_from_register(&mut self, reg: Reg, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus
            .write_byte_read_bytes(&[reg as u8], buf)
            .map_err(Error::Bus)?;

        Ok(())
    }

    #[inline]
    fn write_to_register(&mut self, reg: Reg, val: u8) -> Result<(), Error<B::Error>> {
        self.bus
            .write_bytes(&[reg as u8, val])
            .map_err(Error::Bus)?;
        let mut arr: [u8; 1] = [0];
        self.read_from_register(reg, &mut arr)?;
        if arr[0] != val {
            return Err(Error::WriteFailure);
        }

        Ok(())
    }

    #[inline]
    fn write_to_register_no_check(&mut self, reg: Reg, val: u8) -> Result<(), Error<B::Error>> {
        self.bus
            .write_bytes(&[reg as u8, val])
            .map_err(Error::Bus)?;

        Ok(())
    }

    pub(super) fn write_raw_bytes(&mut self, buf: &[u8]) -> Result<(), Error<B::Error>> {
        self.bus.write_bytes(buf).map_err(Error::Bus)?;

        Ok(())
    }

    pub(super) fn read_raw_bytes(&mut self, buf: &mut [u8]) -> Result<(), Error<B::Error>> {
        self.bus.read_bytes(buf).map_err(Error::Bus)?;

        Ok(())
    }

    pub(super) fn func_cfg_access_set_ispu_reg_access(
        &mut self,
        ispu_reg_access: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::FuncCfgAccess, &mut arr)?;
        let mut val = FuncCfgAccess(arr[0]);
        val.set_func_cfg_access_ispu_reg_access(ispu_reg_access);
        self.write_to_register(Reg::FuncCfgAccess, val.func_cfg_access())?;

        Ok(())
    }

    pub(super) fn func_cfg_access_set_sw_reset_ispu(&mut self, sw_reset_ispu: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::FuncCfgAccess, &mut arr)?;
        let mut val = FuncCfgAccess(arr[0]);
        val.set_func_cfg_access_sw_reset_ispu(sw_reset_ispu);
        self.write_to_register(Reg::FuncCfgAccess, val.func_cfg_access())?;

        Ok(())
    }

    pub(super) fn who_am_i_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::WhoAmI, &mut arr)?;

        Ok(arr[0])
    }

    pub(super) fn drdy_pulsed_reg_set_drdy_pulsed_reg(
        &mut self,
        drdy_pulsed_reg: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::DrdyPulsedReg, &mut arr)?;
        let mut val = DrdyPulsedregG(arr[0]);
        val.set_drdy_pulsed_reg_drdy_pulsed(drdy_pulsed_reg);
        self.write_to_register(Reg::DrdyPulsedReg, val.drdy_pulsed_reg())?;

        Ok(())
    }

    pub(super) fn int1_ctrl_set_int1_drdy_g(
        &mut self,
        int1_drdy_g: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Int1Ctrl, &mut arr)?;
        let mut val = Int1Ctrl(arr[0]);
        val.set_int1_ctrl_int1_drdy_g(int1_drdy_g);
        self.write_to_register(Reg::Int1Ctrl, val.int1_ctrl())?;

        Ok(())
    }

    pub(super) fn int1_ctrl_set_int1_drdy_xl(
        &mut self,
        int1_drdy_xl: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Int1Ctrl, &mut arr)?;
        let mut val = Int1Ctrl(arr[0]);
        val.set_int1_ctrl_int1_drdy_xl(int1_drdy_xl);
        self.write_to_register(Reg::Int1Ctrl, val.int1_ctrl())?;

        Ok(())
    }

    pub(super) fn int2_ctrl_set_int2_drdy_g(
        &mut self,
        int2_drdy_g: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Int2Ctrl, &mut arr)?;
        let mut val = Int2Ctrl(arr[0]);
        val.set_int2_ctrl_int2_drdy_g(int2_drdy_g as u8);
        self.write_to_register(Reg::Int2Ctrl, val.int2_ctrl())?;

        Ok(())
    }

    pub(super) fn int2_ctrl_set_int2_drdy_xl(
        &mut self,
        int2_drdy_xl: u8,
    ) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Int2Ctrl, &mut arr)?;
        let mut val = Int2Ctrl(arr[0]);
        val.set_int2_ctrl_int2_drdy_xl(int2_drdy_xl);
        self.write_to_register(Reg::Int2Ctrl, val.int2_ctrl())?;

        Ok(())
    }

    pub(super) fn ctrl1_xl_get_odr_xl(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl1Xl, &mut arr)?;
        let val = Ctrl1Xl(arr[0]).ctrl1_xl_odr_xl();

        Ok(val)
    }

    pub(super) fn ctrl1_xl_set_odr_xl(&mut self, odr_xl: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl1Xl, &mut arr)?;
        let mut val = Ctrl1Xl(arr[0]);
        val.set_ctrl1_xl_odr_xl(odr_xl);
        self.write_to_register(Reg::Ctrl1Xl, val.ctrl1_xl())?;

        Ok(())
    }

    pub(super) fn ctrl1_xl_get_fs_xl(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl1Xl, &mut arr)?;
        let val = Ctrl1Xl(arr[0]).ctrl1_xl_fs_xl();

        Ok(val)
    }

    pub(super) fn ctrl1_xl_set_fs_xl(&mut self, fs_xl: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl1Xl, &mut arr)?;
        let mut val = Ctrl1Xl(arr[0]);
        val.set_ctrl1_xl_fs_xl(fs_xl);
        self.write_to_register(Reg::Ctrl1Xl, val.ctrl1_xl())?;

        Ok(())
    }

    pub(super) fn ctrl2_g_get_odr_g(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl2G, &mut arr)?;
        let val = Ctrl2G(arr[0]).ctrl2_g_odr_g();

        Ok(val)
    }

    pub(super) fn ctrl2_g_set_odr_g(&mut self, odr_g: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl2G, &mut arr)?;
        let mut val = Ctrl2G(arr[0]);
        val.set_ctrl2_g_odr_g(odr_g);
        self.write_to_register(Reg::Ctrl2G, val.ctrl2_g())?;

        Ok(())
    }

    pub(super) fn ctrl2_g_get_fs_g(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl2G, &mut arr)?;
        let val = Ctrl2G(arr[0]).ctrl2_g_fs_g_fs_125();

        Ok(val)
    }

    pub(super) fn ctrl2_g_set_fs_g(&mut self, fs_g: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl2G, &mut arr)?;
        let mut val = Ctrl2G(arr[0]);
        val.set_ctrl2_g_fs_g_fs_125(fs_g);
        self.write_to_register(Reg::Ctrl2G, val.ctrl2_g())?;

        Ok(())
    }

    pub(super) fn ctrl3_c_set_bdu(&mut self, bdu: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl3C, &mut arr)?;
        let mut val = Ctrl3C(arr[0]);
        val.set_ctrl3_c_bdu(bdu);
        self.write_to_register(Reg::Ctrl3C, val.ctrl3_c())?;

        Ok(())
    }

    pub(super) fn ctrl3_c_get_sw_reset(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl3C, &mut arr)?;
        let val = Ctrl3C(arr[0]).ctrl3_c_sw_reset();

        Ok(val)
    }

    pub(super) fn ctrl3_c_set_sw_reset(&mut self) -> Result<(), Error<B::Error>> {
        self.write_to_register_no_check(Reg::Ctrl3C, 0x1)?;

        Ok(())
    }

    pub(super) fn ctrl6_c_set_xl_hm_mode(&mut self, xl_hm_mode: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl6C, &mut arr)?;
        let mut val = Ctrl6C(arr[0]);
        val.set_ctrl6_c_xl_hm_mode(xl_hm_mode);
        self.write_to_register(Reg::Ctrl6C, val.ctrl6_c())?;

        Ok(())
    }

    pub(super) fn ctrl7_g_set_g_hm_mode(&mut self, g_hm_mode: u8) -> Result<(), Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::Ctrl7G, &mut arr)?;
        let mut val = Ctrl7G(arr[0]);
        val.set_ctrl7_g_g_hm_mode(g_hm_mode);
        self.write_to_register(Reg::Ctrl6C, val.ctrl7_g())?;

        Ok(())
    }

    pub(super) fn ispu_int_status_mainpage_get(&mut self) -> Result<u32, Error<B::Error>> {
        let mut arr: [u8; 4] = [0; 4]; 
        self.read_from_register(Reg::IspuIntStatus0Mainpage, &mut arr)?;
        let val: u32 = arr[0] as u32 | (arr[1] as u32) << 8 | (arr[2] as u32) << 16 | (arr[3] as u32) << 24;
        
        Ok(val)
    }

    pub(super) fn status_reg_get_tda(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::StatusReg, &mut arr)?;
        let val = StatusReg(arr[0]).status_reg_tda();

        Ok(val)
    }

    pub(super) fn status_reg_get_gda(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::StatusReg, &mut arr)?;
        let val = StatusReg(arr[0]).status_reg_gda();

        Ok(val)
    }

    pub(super) fn status_reg_get_xlda(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0];
        self.read_from_register(Reg::StatusReg, &mut arr)?;
        let val = StatusReg(arr[0]).status_reg_xlda();

        Ok(val)
    }
    pub(super) fn out_temp_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::OutTempL, &mut arr)?;
        let t_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;

        Ok(t_raw as i16)
    }

    pub(super) fn outx_g_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::OutxLG, &mut arr)?;
        let x_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;

        Ok(x_raw as i16)
    }

    pub(super) fn outy_g_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::OutyLG, &mut arr)?;
        let y_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;

        Ok(y_raw as i16)
    }

    pub(super) fn outxyz_g_get(&mut self) -> Result<(i16, i16, i16), Error<B::Error>> {
        let mut arr: [u8; 6] = [0; 6];
        self.read_from_register(Reg::OutxLG, &mut arr)?;
        let x_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;
        let y_raw: u16 = arr[2] as u16 | (arr[3] as u16) << 8;
        let z_raw: u16 = arr[4] as u16 | (arr[5] as u16) << 8;

        Ok((x_raw as i16, y_raw as i16, z_raw as i16))
    }

    pub(super) fn outz_g_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::OutzLG, &mut arr)?;
        let z_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;

        Ok(z_raw as i16)
    }

    pub(super) fn outz_h_g_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0; 1];
        self.read_from_register(Reg::OutzHG, &mut arr)?;

        Ok(arr[0])
    }

    pub(super) fn outx_a_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::OutxLA, &mut arr)?;
        let x_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;

        Ok(x_raw as i16)
    }

    pub(super) fn outy_a_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::OutyLA, &mut arr)?;
        let y_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;

        Ok(y_raw as i16)
    }

    pub(super) fn outz_a_get(&mut self) -> Result<i16, Error<B::Error>> {
        let mut arr: [u8; 2] = [0; 2];
        self.read_from_register(Reg::OutzLA, &mut arr)?;
        let z_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;

        Ok(z_raw as i16)
    }

    pub(super) fn outz_h_a_get(&mut self) -> Result<u8, Error<B::Error>> {
        let mut arr: [u8; 1] = [0; 1];
        self.read_from_register(Reg::OutzHA, &mut arr)?;

        Ok(arr[0])
    }

    pub(super) fn outxyz_a_get(&mut self) -> Result<(i16, i16, i16), Error<B::Error>> {
        let mut arr: [u8; 6] = [0; 6];
        self.read_from_register(Reg::OutxLA, &mut arr)?;
        let x_raw: u16 = arr[0] as u16 | (arr[1] as u16) << 8;
        let y_raw: u16 = arr[2] as u16 | (arr[3] as u16) << 8;
        let z_raw: u16 = arr[4] as u16 | (arr[5] as u16) << 8;

        Ok((x_raw as i16, y_raw as i16, z_raw as i16))
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
enum Reg {
    FuncCfgAccess = 0x01,
    DrdyPulsedReg = 0x0B,
    Int1Ctrl = 0x0D,
    Int2Ctrl = 0x0E,
    WhoAmI = 0x0F,
    Ctrl1Xl = 0x10,
    Ctrl2G = 0x11,
    Ctrl3C = 0x12,
    Ctrl6C = 0x15,
    Ctrl7G = 0x16,
    IspuIntStatus0Mainpage = 0x1A,
    StatusReg = 0x1E,
    OutTempL = 0x20,
    OutxLG = 0x22,
    OutyLG = 0x24,
    OutzLG = 0x26,
    OutzHG = 0x27,
    OutxLA = 0x28,
    OutyLA = 0x2A,
    OutzLA = 0x2C,
    OutzHA = 0x2D,
}

bitfield! {
    struct FuncCfgAccess(u8);
    func_cfg_access, _: 7, 0;
    func_cfg_access_ispu_reg_access, set_func_cfg_access_ispu_reg_access: 7, 7;
    func_cfg_access_shub_reg_access, set_func_cfg_access_shub_reg_access: 6, 6;
    not_used5_2, _: 5, 2;
    func_cfg_access_sw_reset_ispu, set_func_cfg_access_sw_reset_ispu: 1, 1;
    not_used0, _: 0, 0;
}

bitfield! {
    struct DrdyPulsedregG(u8);
    drdy_pulsed_reg, _: 7, 0;
    drdy_pulsed_reg_drdy_pulsed, set_drdy_pulsed_reg_drdy_pulsed: 7, 7;
    not_used6_0, _: 6, 0;
}

bitfield! {
    struct Int1Ctrl(u8);
    int1_ctrl, _: 7, 0;
    not_used7_3, _: 7, 3;
    int1_ctrl_int1_boot, set_int1_ctrl_int1_boot: 2, 2;
    int1_ctrl_int1_drdy_g, set_int1_ctrl_int1_drdy_g: 1, 1;
    int1_ctrl_int1_drdy_xl, set_int1_ctrl_int1_drdy_xl: 0, 0;

}

bitfield! {
    struct Int2Ctrl(u8);
    int2_ctrl, _: 7, 0;
    int2_ctrl_int2_sleep_ispu, set_int2_ctrl_int2_sleep_ispu: 7, 7;
    not_used6_3, _: 6, 3;
    int2_ctrl_int2_drdy_temp, set_int2_ctrl_int2_drdy_temp: 2, 2;
    int2_ctrl_int2_drdy_g, set_int2_ctrl_int2_drdy_g: 1, 1;
    int2_ctrl_int2_drdy_xl, set_int2_ctrl_int2_drdy_xl: 0, 0;
}

bitfield! {
    struct Ctrl1Xl(u8);
    ctrl1_xl, _: 7, 0;
    ctrl1_xl_odr_xl, set_ctrl1_xl_odr_xl: 7, 4;
    ctrl1_xl_fs_xl, set_ctrl1_xl_fs_xl: 3, 2;
    not_used1_0, _: 1, 0;
}

bitfield! {
    struct Ctrl2G(u8);
    ctrl2_g, _: 7, 0;
    ctrl2_g_odr_g, set_ctrl2_g_odr_g: 7, 4;
    ctrl2_g_fs_g_fs_125, set_ctrl2_g_fs_g_fs_125: 3, 1;
    not_used0, _: 0, 0;
}

bitfield! {
    struct Ctrl3C(u8);
    ctrl3_c, _: 7, 0;
    ctrl3_c_boot, set_ctrl3_c_boot: 7, 7;
    ctrl3_c_bdu, set_ctrl3_c_bdu: 6, 6;
    ctrl3_c_h_lactive, set_ctrl3_c_h_lactive: 5, 5;
    ctrl3_c_pp_od, set_ctrl3_c_pp_od: 4, 4;
    ctrl3_c_sim, set_ctrl3_c_sim: 3, 3;
    ctrl3_c_if_inc, set_ctrl3_c_if_inc: 2, 2;
    not_used1, _: 1, 1;
    ctrl3_c_sw_reset, set_ctrl3_c_sw_reset: 0, 0;
}

bitfield! {
    struct Ctrl6C(u8);
    ctrl6_c, _: 7, 0;
    not_used7_5, _: 7, 5;
    ctrl6_c_xl_hm_mode, set_ctrl6_c_xl_hm_mode: 4, 4;
    not_used3_0, _: 3, 0;
}

bitfield! {
    struct Ctrl7G(u8);
    ctrl7_g, _: 7, 0;
    ctrl7_g_g_hm_mode, set_ctrl7_g_g_hm_mode: 7, 7;
    not_used6_0, _: 6, 0;
}

bitfield! {
    struct StatusReg(u8);
    status_reg, _: 7, 0;
    status_reg_timestamp_endcount, _: 7, 7;
    not_used6_3, _: 6, 3;
    status_reg_tda, _: 2, 2;
    status_reg_gda, _: 1, 1;
    status_reg_xlda, _: 0, 0;
}

#[repr(u8)]
pub(super) enum Ctrl1XlOdr {
    PowerDown = 0b0000,
    Hz1_6 = 0b1011,
    Hz12_5 = 0b0001,
    Hz26 = 0b0010,
    Hz52 = 0b0011,
    Hz104 = 0b0100,
    Hz208 = 0b0101,
    Hz416 = 0b0110,
    Hz833 = 0b0111,
    Hz1667 = 0b1000,
    Hz3333 = 0b1001,
    Hz6667 = 0b1010,
}

impl From<f32> for Ctrl1XlOdr {
    fn from(value: f32) -> Self {
        if value == 0.0 {
            Self::PowerDown
        } else if value <= 1.6 {
            Self::Hz1_6
        } else if value <= 12.5 {
            Self::Hz12_5
        } else if value <= 26.0 {
            Self::Hz26
        } else if value <= 52.0 {
            Self::Hz52
        } else if value <= 104.0 {
            Self::Hz104
        } else if value <= 208.0 {
            Self::Hz208
        } else if value <= 416.0 {
            Self::Hz416
        } else if value <= 833.0 {
            Self::Hz833
        } else if value <= 1667.0 {
            Self::Hz1667
        } else if value <= 3333.0 {
            Self::Hz3333
        } else {
            Self::Hz6667
        }
    }
}

impl From<u8> for Ctrl1XlOdr {
    fn from(value: u8) -> Self {
        match value {
            0b0000 => Self::PowerDown,
            0b1011 => Self::Hz1_6,
            0b0001 => Self::Hz12_5,
            0b0010 => Self::Hz26,
            0b0011 => Self::Hz52,
            0b0100 => Self::Hz104,
            0b0101 => Self::Hz208,
            0b0110 => Self::Hz416,
            0b0111 => Self::Hz833,
            0b1000 => Self::Hz1667,
            0b1001 => Self::Hz3333,
            0b1010 => Self::Hz6667,
            _ => Self::PowerDown,
        }
    }
}

impl From<Ctrl1XlOdr> for f32 {
    fn from(value: Ctrl1XlOdr) -> Self {
        match value {
            Ctrl1XlOdr::PowerDown => 0.0,
            Ctrl1XlOdr::Hz1_6 => 1.6,
            Ctrl1XlOdr::Hz12_5 => 12.5,
            Ctrl1XlOdr::Hz26 => 26.0,
            Ctrl1XlOdr::Hz52 => 52.0,
            Ctrl1XlOdr::Hz104 => 104.0,
            Ctrl1XlOdr::Hz208 => 208.0,
            Ctrl1XlOdr::Hz416 => 416.0,
            Ctrl1XlOdr::Hz833 => 833.0,
            Ctrl1XlOdr::Hz1667 => 1667.0,
            Ctrl1XlOdr::Hz3333 => 3333.0,
            Ctrl1XlOdr::Hz6667 => 6667.0,
        }
    }
}

impl From<u8> for XlFullScale {
    fn from(value: u8) -> Self {
        match value {
            0b00 => Self::G2,
            0b10 => Self::G4,
            0b11 => Self::G8,
            0b01 => Self::G16,
            _ => Self::G8,
        }
    }
}

impl From<XlFullScale> for f32 {
    fn from(value: XlFullScale) -> Self {
        match value {
            XlFullScale::G2 => 0.061,
            XlFullScale::G4 => 0.122,
            XlFullScale::G8 => 0.244,
            XlFullScale::G16 => 0.488,
        }
    }
}

pub(super) enum Ctrl2GOdr {
    PowerDown = 0b0000,
    Hz12_5 = 0b0001,
    Hz26 = 0b0010,
    Hz52 = 0b0011,
    Hz104 = 0b0100,
    Hz208 = 0b0101,
    Hz416 = 0b0110,
    Hz833 = 0b0111,
    Hz1667 = 0b1000,
    Hz3333 = 0b1001,
    Hz6667 = 0b1010,
}

impl From<f32> for Ctrl2GOdr {
    fn from(value: f32) -> Self {
        if value == 0.0 {
            Self::PowerDown
        } else if value <= 12.5 {
            Self::Hz12_5
        } else if value <= 26.0 {
            Self::Hz26
        } else if value <= 52.0 {
            Self::Hz52
        } else if value <= 104.0 {
            Self::Hz104
        } else if value <= 208.0 {
            Self::Hz208
        } else if value <= 416.0 {
            Self::Hz416
        } else if value <= 833.0 {
            Self::Hz833
        } else if value <= 1667.0 {
            Self::Hz1667
        } else if value <= 3333.0 {
            Self::Hz3333
        } else {
            Self::Hz6667
        }
    }
}

impl From<u8> for Ctrl2GOdr {
    fn from(value: u8) -> Self {
        match value {
            0b0000 => Self::PowerDown,
            0b0001 => Self::Hz12_5,
            0b0010 => Self::Hz26,
            0b0011 => Self::Hz52,
            0b0100 => Self::Hz104,
            0b0101 => Self::Hz208,
            0b0110 => Self::Hz416,
            0b0111 => Self::Hz833,
            0b1000 => Self::Hz1667,
            0b1001 => Self::Hz3333,
            0b1010 => Self::Hz6667,
            _ => Self::PowerDown,
        }
    }
}

impl From<Ctrl2GOdr> for f32 {
    fn from(value: Ctrl2GOdr) -> Self {
        match value {
            Ctrl2GOdr::PowerDown => 0.0,
            Ctrl2GOdr::Hz12_5 => 12.5,
            Ctrl2GOdr::Hz26 => 26.0,
            Ctrl2GOdr::Hz52 => 52.0,
            Ctrl2GOdr::Hz104 => 104.0,
            Ctrl2GOdr::Hz208 => 208.0,
            Ctrl2GOdr::Hz416 => 416.0,
            Ctrl2GOdr::Hz833 => 833.0,
            Ctrl2GOdr::Hz1667 => 1667.0,
            Ctrl2GOdr::Hz3333 => 3333.0,
            Ctrl2GOdr::Hz6667 => 6667.0,
        }
    }
}

impl From<u8> for GFullScale {
    fn from(value: u8) -> Self {
        match value {
            0b001 => Self::Dps125,
            0b000 => Self::Dps250,
            0b010 => Self::Dps500,
            0b100 => Self::Dps1000,
            0b110 => Self::Dps2000,
            _ => Self::Dps125,
        }
    }
}

impl From<GFullScale> for f32 {
    fn from(value: GFullScale) -> Self {
        match value {
            GFullScale::Dps125 => 4.375,
            GFullScale::Dps250 => 8.75,
            GFullScale::Dps500 => 17.50,
            GFullScale::Dps1000 => 35.0,
            GFullScale::Dps2000 => 70.0,
        }
    }
}
