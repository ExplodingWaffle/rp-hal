//! # Flash ID Example
//!
//! This application demonstrates how the rp2040_hal_macros::entry macro can be
//! used to initialise a variable with the 64 bit UID from a connected flash
//! chip, which can be used to create a serial number, such as for USB.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function. The custom macro is used because in
// order to get the UID from flash, code is executed before main
use rp2040_hal_macros::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised. It also initialises the
/// flash_id variable with the UID from the flash.
///
/// The function simply copies the UID from the static into a variable.
//#[entry(flash_id)]
#[entry]
fn main() -> ! {
    let flash_id: u64;
    unsafe {
        unsafe fn rom_func_lookup(code: u16) -> *const u32 {
            const FUNC_TABLE: *const u16 = 0x0000_0014 as _;
            let rom_table_lookup: unsafe extern "C" fn(*const u16, u32) -> *const u32 =
                core::mem::transmute(0x0000_0018 as *const u16);
            let code = code;
            rom_table_lookup(FUNC_TABLE, code as u32)
        }

        //boot2 is copied to ram so that we can use it once XIP is off
        let mut boot2 = [0u32; 256 / 4];
        //core::ptr::copy_nonoverlapping(0x10000000 as *const _, boot2.as_mut_ptr(), 64);
        let memcpy44: unsafe extern "C" fn(*mut u32, *const u32, u32) -> *mut u8 =
            core::mem::transmute(rom_func_lookup(u16::from_be_bytes([b'C', b'4'])));
        memcpy44(&mut boot2 as *mut _, 0x10000000 as *const _, 256);

        let boot2_fn_ptr = (&boot2 as *const _ as *const u8).offset(1);
        let boot2_fn: unsafe extern "C" fn() -> () = core::mem::transmute(boot2_fn_ptr);

        #[repr(C)]
        struct FunctionPointers<'a> {
            connect_internal_flash: unsafe extern "C" fn() -> (),
            flash_exit_xip: unsafe extern "C" fn() -> (),
            flash_flush_cache: unsafe extern "C" fn() -> (),
            flash_enter_xip: unsafe extern "C" fn() -> (),
            phantom: core::marker::PhantomData<&'a ()>,
        }

        let ptrs = FunctionPointers {
            //be or le?
            connect_internal_flash: core::mem::transmute(rom_func_lookup(u16::from_be_bytes([
                b'I', b'F',
            ]))),
            flash_exit_xip: core::mem::transmute(rom_func_lookup(u16::from_be_bytes([b'E', b'X']))),
            flash_flush_cache: core::mem::transmute(rom_func_lookup(u16::from_be_bytes([
                b'F', b'C',
            ]))),
            flash_enter_xip: boot2_fn,
            phantom: core::marker::PhantomData,
        };

        #[inline(never)]
        #[link_section = ".data.ram_func"]
        unsafe fn get_id(ptrs: FunctionPointers) -> u64 {
            (ptrs.connect_internal_flash);
            (ptrs.flash_exit_xip);
            //now entering ram; population: this function
            const IO_QSPI_BASE: *const u32 = 0x4001_8000 as *const _;
            const IO_QSPI_GPIO_QSPI_SS_CTRL: *mut u32 = IO_QSPI_BASE.wrapping_offset(0x0c) as *mut _; 
            //let mut cs_ctrl = *IO_QSPI_GPIO_QSPI_SS_CTRL;
            // OUTOVER 9:8 = 0x2 to force CS low
            //cs_ctrl = (((1 << 2) - 1) << 8) & (0x2 << 8) | !(((1 << 2) - 1) << 8) & cs_ctrl;
            IO_QSPI_GPIO_QSPI_SS_CTRL.write_volatile((((1 << 2) - 1) << 8) & (0x2 << 8) | !(((1 << 2) - 1) << 8) & IO_QSPI_GPIO_QSPI_SS_CTRL.read_volatile());

            const XIP_SSI_BASE: *const u32 = 0x1800_0000 as *const _;
            //const SSI_CTRLR0: *const u32 = XIP_SSI_BASE.offset(0x00);
            //const SSI_SPI_CTRLR0: *const u32 = XIP_SSI_BASE.offset(0xf4);
            const SSI_DR: *mut u32 = XIP_SSI_BASE.wrapping_offset(0x60) as *mut _;
            let mut dr = *SSI_DR;
            const SSI_SR: *const u32 = XIP_SSI_BASE.wrapping_offset(0x28);
            let sr = *SSI_SR;
            const CMD_UID: u8 = 0x4B;
            //do ssi stuff
            let tx_byte = 0;
            let rx_byte = 0;
            let mut tx_buf = [0u8; 8];
            tx_buf[0] = CMD_UID;
            let mut rx_buf = [0u8; 8];
            //
            while (tx_byte < 8) || (rx_byte < 8) {
                let can_put = (sr & (1 << 1)) != 0; //bit 1 TFNF
                let can_get = (sr & (1 << 3)) != 0; //bit 3 RFNE
                if (tx_byte < 8) || can_put {
                    dr = tx_buf[tx_byte] as u32;
                }
                if (rx_byte < 8) || can_get {
                    rx_buf[rx_byte] = dr as u8;
                }
            }
            // OUTOVER 9:8 = 0x3 to force CS high
            //cs_ctrl = (((1 << 2) - 1) << 8) & (0x3 << 8) | !(((1 << 2) - 1) << 8) & cs_ctrl;
            IO_QSPI_GPIO_QSPI_SS_CTRL.write_volatile((((1 << 2) - 1) << 8) & (0x3 << 8) | !(((1 << 2) - 1) << 8) & IO_QSPI_GPIO_QSPI_SS_CTRL.read_volatile());
            (ptrs.flash_flush_cache);
            (ptrs.flash_enter_xip);
            u64::from_be_bytes(rx_buf) // endianness?
        }

        flash_id = get_id(ptrs);
    }
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks //is this required?
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    //blinky to show we didn't get stuck

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

// End of file
