#![no_std]
#![no_main]

use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use lis2duxs12_rs::{prelude::*, I2CAddress, Lis2duxs12, ID};
use panic_halt as _;
use stm32f4xx_hal::{
    gpio,
    gpio::{Edge, Input},
    i2c::{DutyCycle, I2c, Mode},
    interrupt, pac,
    prelude::*,
    serial::{config::Config, Serial},
};
type IntPin = gpio::PB0<Input>;

static INT_PIN: Mutex<RefCell<Option<IntPin>>> = Mutex::new(RefCell::new(None));
static MEMS_EVENT: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);
    let tim1 = dp.TIM1.delay_us(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioa = dp.GPIOA.split();

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();

    let i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &clocks,
    );

    let tx_pin = gpioa.pa2.into_alternate();

    let mut tx = Serial::tx(
        dp.USART2,
        tx_pin,
        Config::default().baudrate(115_200.bps()),
        &clocks,
    )
    .unwrap();
    let mut int_pin = gpiob.pb0.into_input();
    // Configure Pin for Interrupts
    // 1) Promote SYSCFG structure to HAL to be able to configure interrupts
    let mut syscfg = dp.SYSCFG.constrain();
    // 2) Make an interrupt source
    int_pin.make_interrupt_source(&mut syscfg);
    // 3) Make an interrupt source
    int_pin.trigger_on_edge(&mut dp.EXTI, Edge::Rising);
    // 4) Enable gpio interrupt
    int_pin.enable_interrupt(&mut dp.EXTI); // Enable the external interrupt in the NVIC

    // Enable the external interrupt in the NVIC by passing the interrupt number
    unsafe {
        cortex_m::peripheral::NVIC::unmask(int_pin.interrupt());
    }

    // Now that pin is configured, move pin into global context
    cortex_m::interrupt::free(|cs| {
        INT_PIN.borrow(cs).replace(Some(int_pin));
    });

    delay.delay_ms(5);

    let mut sensor = Lis2duxs12::new_i2c(i2c, I2CAddress::I2cAddH, tim1);

    delay.delay_ms(10);

    // sensor.exit_deep_power_down().unwrap(); // Only SPI

    // Loop if wrong device
    match sensor.device_id_get() {
        Ok(id) => {
            if id != ID {
                loop {}
            }
        }
        Err(e) => writeln!(tx, "Error in reading id: {:?}", e).unwrap(),
    }

    sensor.init_set(Init::Reset).unwrap();

    while sensor.status_get().unwrap().sw_reset == 1 {}

    sensor.init_set(Init::SensorOnlyOn).unwrap();

    let int1_route = PinIntRoute {
        drdy: 1,
        ..Default::default()
    };
    sensor.pin_int1_route_set(&int1_route).unwrap();

    // Set Output Data Rate
    let md = Md {
        fs: Fs::_4g,
        bw: Bw::OdrDiv4,
        odr: Odr::_25hzLp,
    };
    sensor.mode_set(&md).unwrap();

    loop {
        // Wait for interrupt
        let mems_event = cortex_m::interrupt::free(|cs| {
            let flag = *MEMS_EVENT.borrow(cs).borrow();
            if flag {
                MEMS_EVENT.borrow(cs).replace(false);
            }
            flag
        });
        if !mems_event {
            continue;
        }
        if sensor.status_get().unwrap().drdy != 0 {
            // Read acceleration data
            match sensor.xl_data_get(&md) {
                Ok(data_xl) => {
                    writeln!(
                        tx,
                        "Acceleration [mg]: {:.2}\t{:.2}\t{:.2}",
                        data_xl.mg[0], data_xl.mg[1], data_xl.mg[2]
                    )
                    .unwrap();
                }
                Err(e) => writeln!(tx, "Error in reading acceleration data: {:?}", e).unwrap(),
            }

            // Read temperature data
            match sensor.outt_data_get() {
                Ok(data_temp) => {
                    writeln!(tx, "Temp [degC]: {:.2}", data_temp.heat.deg_c).unwrap();
                }
                Err(e) => writeln!(tx, "Error in reading temperature data: {:?}", e).unwrap(),
            }
        }
    }
}

#[interrupt]
fn EXTI0() {
    cortex_m::interrupt::free(|cs| {
        let mut int_pin = INT_PIN.borrow(cs).borrow_mut();
        if int_pin.as_mut().unwrap().check_interrupt() {
            int_pin.as_mut().unwrap().clear_interrupt_pending_bit();
        }
        MEMS_EVENT.borrow(cs).replace(true);
    })
}
