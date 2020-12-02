#![no_std]
#![no_main]

#[macro_use]
extern crate log;

use smoltcp as net;
use stm32h7xx_hal::{gpio::Speed, prelude::*, ethernet};

use heapless::{consts, String};

use cortex_m;

use panic_halt as _;
use serde::{Deserialize, Serialize};

use rtic::cyccnt::{Instant, U32Ext};

mod tcp_stack;

use minimq::{
    embedded_nal::{IpAddr, Ipv4Addr},
    MqttClient, QoS,
};
use tcp_stack::NetworkStack;

use core::fmt::Write;

pub struct NetStorage {
    ip_addrs: [net::wire::IpCidr; 1],
    neighbor_cache: [Option<(net::wire::IpAddress, net::iface::Neighbor)>; 8],
}

static mut NET_STORE: NetStorage = NetStorage {
    // Placeholder for the real IP address, which is initialized at runtime.
    ip_addrs: [net::wire::IpCidr::Ipv6(
        net::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
    )],
    neighbor_cache: [None; 8],
};

#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

#[derive(Serialize, Deserialize)]
struct Random {
    random: Option<u32>
}

type NetworkInterface =
    net::iface::EthernetInterface<'static, 'static, 'static, ethernet::EthernetDMA<'static>>;

macro_rules! add_socket {
    ($sockets:ident, $tx_storage:ident, $rx_storage:ident) => {
        let mut $rx_storage = [0; 4096];
        let mut $tx_storage = [0; 4096];

        let tcp_socket = {
            let tx_buffer = net::socket::TcpSocketBuffer::new(&mut $tx_storage[..]);
            let rx_buffer = net::socket::TcpSocketBuffer::new(&mut $rx_storage[..]);

            net::socket::TcpSocket::new(tx_buffer, rx_buffer)
        };

        let _handle = $sockets.add(tcp_socket);
    };
}

#[cfg(not(feature = "semihosting"))]
fn init_log() {}

#[cfg(feature = "semihosting")]
fn init_log() {
    use cortex_m_log::log::{init as init_log, Logger};
    use cortex_m_log::printer::semihosting::{hio::HStdout, InterruptOk};
    use log::LevelFilter;

    static mut LOGGER: Option<Logger<InterruptOk<HStdout>>> = None;

    let logger = Logger {
        inner: InterruptOk::<_>::stdout().unwrap(),
        level: LevelFilter::Info,
    };

    let logger = unsafe { LOGGER.get_or_insert(logger) };

    init_log(logger).unwrap();
}

#[derive(Debug, Copy, Clone)]
pub struct HeartbeatSettings {
    rate: u32,
    status: u8
}

impl HeartbeatSettings {
    fn string_set(&mut self, field: &str, value: &str) -> Result<(),()> {
        match field {
            "rate" => {
                self.rate = value.parse().map_err(|_|{()})?;
                Ok(())
            }
            "status" => {
                self.status = value.parse().map_err(|_|{()})?;
                Ok(())
            }
            _ => Err(())
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        net_interface: NetworkInterface,
        rng: stm32h7xx_hal::rng::Rng,
        usart3_tx: stm32h7xx_hal::serial::Tx<stm32h7xx_hal::stm32::USART3>,
        heartbeat_settings: HeartbeatSettings,
        heartbeat_counter: u8,
    }

    #[init(schedule=[usart3])]
    fn init(mut c: init::Context) -> init::LateResources {
        c.core.DWT.enable_cycle_counter();

        // Enable SRAM3 for the descriptor ring.
        c.device.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        let rcc = c.device.RCC.constrain();
        let pwr = c.device.PWR.constrain();
        let vos = pwr.freeze();

        let ccdr = rcc
            .sysclk(400.mhz())
            .hclk(200.mhz())
            .per_ck(100.mhz())
            .pll2_p_ck(100.mhz())
            .pll2_q_ck(100.mhz())
            .freeze(vos, &c.device.SYSCFG);

        init_log();

        let gpioa = c.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = c.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = c.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = c.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpiog = c.device.GPIOG.split(ccdr.peripheral.GPIOG);

        // Configure ethernet IO
        {
            let _rmii_refclk = gpioa.pa1.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_mdio = gpioa.pa2.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_mdc = gpioc.pc1.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_crs_dv = gpioa.pa7.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_rxd0 = gpioc.pc4.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_rxd1 = gpioc.pc5.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_tx_en = gpiog.pg11.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_txd0 = gpiog.pg13.into_alternate_af11().set_speed(Speed::VeryHigh);
            let _rmii_txd1 = gpiob.pb13.into_alternate_af11().set_speed(Speed::VeryHigh);
        }

        // Configure ethernet
        let net_interface = {
            let mac_addr = net::wire::EthernetAddress([0xAC, 0x6F, 0x7A, 0xDE, 0xD6, 0xC8]);
            let (eth_dma, _eth_mac) = unsafe {
                ethernet::new_unchecked(
                    c.device.ETHERNET_MAC,
                    c.device.ETHERNET_MTL,
                    c.device.ETHERNET_DMA,
                    &mut DES_RING,
                    mac_addr.clone(),
                )
            };

            unsafe { ethernet::enable_interrupt() }

            let store = unsafe { &mut NET_STORE };

            store.ip_addrs[0] = net::wire::IpCidr::new(net::wire::IpAddress::v4(10, 0, 0, 2), 24);

            let neighbor_cache = net::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

            net::iface::EthernetInterfaceBuilder::new(eth_dma)
                .ethernet_addr(mac_addr)
                .neighbor_cache(neighbor_cache)
                .ip_addrs(&mut store.ip_addrs[..])
                .finalize()
        };



        // Initialize random number generator
        let rng = c.device.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);

        // Initialize UART 3
        let usart3 = c.device.USART3.serial(
            (gpiod.pd8.into_alternate_af7(), gpiod.pd9.into_alternate_af7()),
            115200.bps(),
            ccdr.peripheral.USART3,
            &ccdr.clocks
           ).unwrap();
        let (u3tx, _) = usart3.split();

        let heartbeat = HeartbeatSettings {
            rate: 1,
            status: 0,
        };

        c.schedule.usart3(c.start + (400_000_000u32 / heartbeat.rate).cycles());

        c.core.SCB.enable_icache();

        init::LateResources {
            net_interface: net_interface,
            rng: rng,
            usart3_tx: u3tx,
            heartbeat_settings: heartbeat,
            heartbeat_counter: 0,
        }
    }

    #[idle(resources=[net_interface, rng, heartbeat_settings])]
    fn idle(mut c: idle::Context) -> ! {
        let mut time: u32 = 0;
        let mut next_ms = Instant::now();

        // Initialize staging settings to current settings values
        let mut staging_settings = c.resources.heartbeat_settings.lock(|s| {*s});

        next_ms += 400_00.cycles();

        let mut socket_set_entries: [_; 8] = Default::default();
        let mut sockets = net::socket::SocketSet::new(&mut socket_set_entries[..]);
        add_socket!(sockets, rx_storage, tx_storage);

        let net_interface = c.resources.net_interface;
        let tcp_stack = NetworkStack::new(net_interface, sockets);
        let mut heartbeat_settings = c.resources.heartbeat_settings;
        let mut client = MqttClient::<consts::U256, _>::new(
            IpAddr::V4(Ipv4Addr::new(10, 0, 0, 1)),
            "nucleo",
            tcp_stack,
        )
        .unwrap();

	let mut subscribed = false;

        loop {

            if client.is_connected().unwrap() && !subscribed {
                client.subscribe("settings/#", &[]).unwrap();
                client.subscribe("commit", &[]).unwrap();
                info!("subscribed to topic!");
                subscribed = true;
            }
            let tick = Instant::now() > next_ms;

            if tick {
                next_ms += 400_000.cycles();
                time += 1;
            }

            if tick && (time % 1000) == 0 {
                client
                    .publish("nucleo", "Hello, World!".as_bytes(), QoS::AtMostOnce, &[])
                    .unwrap();

                let random = Random {
                    random: c.resources.rng.gen().ok()
                };

                let random: String<consts::U256> =
                    serde_json_core::to_string(&random).unwrap();
                client
                    .publish(
                        "random",
                        &random.into_bytes(),
                        QoS::AtMostOnce,
                        &[],
                    )
                    .unwrap();
            }


            match client
                .poll(|_client, topic, message, _properties| match topic.split('/').nth(0).unwrap() {
                    "settings" => {
                        let message = core::str::from_utf8(message).unwrap();
                        let subtopic = topic.split('/').skip(1).nth(0).unwrap();
                        info!("subtopic: {:#?}", subtopic);
                        let update_result = staging_settings.string_set(subtopic, message);
                        info!("settings update: '{:?}', received: {:?}, result: {:?}", topic, message, update_result);
                        info!("staged settings: {:#?}", staging_settings);

                    }
                    "commit" => {
                        info!("commiting settings");
                        // Commit the settings to the real structure
                        heartbeat_settings.lock(|s| {
                            *s = staging_settings;
                        });
                    }
                    _ => {
                        let message = core::str::from_utf8(message).unwrap();
                        info!("On '{:?}', received: {:?}", topic, message)
                    }
                })
            {
                Ok(_) => {},
                // If we got disconnected from the broker
                Err(minimq::Error::Disconnected) => {
                    info!("MQTT client disconnected")
                },
                Err(e) => {
                    panic!("{:#?}", e);
                }
            };


            // Update the TCP stack.
            let sleep = client.network_stack.update(time);
            if sleep {
                //cortex_m::asm::wfi();
                cortex_m::asm::nop();
            }
        }
    }

    #[task(resources=[usart3_tx, heartbeat_settings, heartbeat_counter], schedule=[usart3])]
    fn usart3(cx: usart3::Context) {
        writeln!(cx.resources.usart3_tx, "{} status: {}",
            cx.resources.heartbeat_counter,
            cx.resources.heartbeat_settings.status);

        *cx.resources.heartbeat_counter = cx.resources.heartbeat_counter.wrapping_add(1);

        let rate = cx.resources.heartbeat_settings.rate;
        cx.schedule.usart3(Instant::now() + (400_000_000u32 / rate).cycles());
    }

    #[task(binds=ETH)]
    fn eth(_: eth::Context) {
        unsafe { ethernet::interrupt_handler() }
    }

    extern "C" {
        fn USART3();
    }
};
