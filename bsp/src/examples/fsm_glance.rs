use defmt::info;
use maybe_async::maybe_async;
use crate::*;

use crate::config::fsm_config::GLANCE;
use st_mems_reg_config_conv::ucf_entry::MemsUcfOp;

#[maybe_async]
pub async fn run<B, D, L, I>(bus: B, mut tx: L, delay: D, mut int_pin: I) -> !
where
    B: BusOperation,
    D: DelayNs + Clone,
    L: embedded_io::Write,
    I: InterruptPin
{
    use lis2duxs12::*;

    info!("Configuring the sensor");
    let mut sensor = Lis2duxs12::from_bus(bus, delay);

    // sensor.exit_deep_power_down().await.unwrap(); // Only SPI

    // Check device ID
    let whoami = sensor.device_id_get().await.unwrap();
    info!("Device ID: {:x}", whoami);
    if whoami != ID {
        writeln!(tx, "Device ID mismatch: {:#02x}", whoami).unwrap();
        loop {}
    }

    // Restore default configuration
    sensor.sw_reset().await.unwrap();

    // Set BDU and IF_INC recommended for driver usage
    sensor.init_set().await.unwrap();

    for ucf_entry in GLANCE {
        match ucf_entry.op {
            MemsUcfOp::Delay => {
                sensor.tim.delay_ms(ucf_entry.data.into()).await;
            }
            MemsUcfOp::Write => {
                sensor
                    .bus
                    .write_to_register(ucf_entry.address as u8, &[ucf_entry.data])
                    .await
                    .unwrap();
            }
            _ => {}
        }
    }

    loop {
        // Wait for interrupt
        int_pin.wait_for_event().await;
        let status = sensor.fsm_status_get().await.unwrap();

        if status.is_fsm1() == 1 {
            let fsm_out = sensor.fsm_out_get().await.unwrap();
            let fsm_out = fsm_out[0]; // the ucf store the fsm in the first index

            if fsm_out == 0x20 {
                // glance event catched

                writeln!(tx, "GLANCE event").unwrap();
            }
            if fsm_out == 0x08 {
                // deglance event catched

                writeln!(tx, "DEGLANCE event").unwrap();
            }
        }
    }}
