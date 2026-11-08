#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::InputPin;
use embedded_hal::pwm::SetDutyCycle;
use hal::{
    adc::{Adc, AdcPin},
    clocks::init_clocks_and_plls,
    gpio::Pins,
    pwm::Slices,
    sio::Sio,
    watchdog::Watchdog,
    Timer,
};
use panic_halt as _;
use rp235x_hal as hal;
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid},
};
use usbd_audio::{AudioClassBuilder, Format, StreamConfig, TerminalType};
use usbd_hid::hid_class::HIDClass;

const XTAL_HZ: u32 = 12_000_000;
const SAMPLE_HZ: u32 = 16_000;
const SAMPLES_PER_USB_MS: usize = 16;
const AUDIO_PACKET_BYTES: usize = SAMPLES_PER_USB_MS * 2;
const ADC_MIDPOINT: u16 = 2048;

// Vendor-defined one-bit input report. Hosts see a standard HID interface;
// softphone mapping to PTT is performed by the host application.
const PTT_REPORT_DESCRIPTOR: &[u8] = &[
    0x06, 0x00, 0xff, // Usage Page (Vendor-defined 0xFF00)
    0x09, 0x01, // Usage (1)
    0xa1, 0x01, // Collection (Application)
    0x09, 0x02, // Usage (2): PTT state
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x01, // Logical Maximum (1)
    0x75, 0x01, // Report Size (1 bit)
    0x95, 0x01, // Report Count (1)
    0x81, 0x02, // Input (Data, Variable, Absolute)
    0x75, 0x07, // Report Size (7 padding bits)
    0x95, 0x01, // Report Count (1)
    0x81, 0x03, // Input (Constant, Variable, Absolute)
    0xc0, // End Collection
];

// RP2350 boot ROM application image marker.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        XTAL_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Handset PTT pair: one wire to GP15, the other to GND.
    // No voltage from the handset is expected on this contact pair.
    let mut ptt_pin = pins.gpio15.into_pull_up_input();

    // MCP6002-conditioned electret mic output: GP26 / ADC0. The external
    // analog stage must center audio at ~1.65 V and stay within 0..3.3 V.
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut mic_pin = AdcPin::new(pins.gpio26.into_floating_input()).unwrap();

    // GP16 / PWM0A -> reconstruction filter -> PAM8302A input.
    // Receiver connects only across PAM8302A OUT+ and OUT-.
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm0;
    pwm.set_top(255);
    pwm.set_div_int(1);
    pwm.set_div_frac(0);
    pwm.enable();
    let mut speaker_pwm = pwm.channel_a;
    let _speaker_pin = speaker_pwm.output_to(pins.gpio16);

    let timer = Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Mono UAC1 capture (microphone) and playback (speaker), 16-bit/16 kHz.
    let mut audio = AudioClassBuilder::new()
        .input(
            StreamConfig::new_discrete(Format::S16le, 1, &[SAMPLE_HZ], TerminalType::InMicrophone)
                .unwrap(),
        )
        .output(
            StreamConfig::new_discrete(Format::S16le, 1, &[SAMPLE_HZ], TerminalType::OutSpeaker)
                .unwrap(),
        )
        .build(&usb_bus)
        .unwrap();
    let mut ptt_hid = HIDClass::new_ep_in(&usb_bus, PTT_REPORT_DESCRIPTOR, 1);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x5054))
        .strings(&[StringDescriptors::default()
            .manufacturer("Open PTT")
            .product("Pico 2 Handset Adapter")
            .serial_number("PICO2-PTT-01")])
        .unwrap()
        .device_class(0)
        .build();

    let mut ptt = PttDebouncer::new();
    let mut last_sample_us = timer.get_counter().ticks();
    let mut sample_phase = 0u64;
    let mut mic_packet = [0u8; AUDIO_PACKET_BYTES];
    let mut mic_samples = 0usize;
    let mut speaker_packet = [0u8; 64];
    let mut speaker_samples = [0i16; 32];
    let mut speaker_len = 0usize;
    let mut speaker_pos = 0usize;
    let mut last_ptt_report = false;

    loop {
        usb_dev.poll(&mut [&mut audio, &mut ptt_hid]);

        // Read the host's speaker packet when one is ready. USB Audio packet
        // sizes can vary by a sample; process only complete 16-bit samples.
        if let Ok(n) = audio.read(&mut speaker_packet) {
            speaker_len = n / 2;
            speaker_pos = 0;
            for i in 0..speaker_len.min(speaker_samples.len()) {
                speaker_samples[i] =
                    i16::from_le_bytes([speaker_packet[i * 2], speaker_packet[i * 2 + 1]]);
            }
            speaker_len = speaker_len.min(speaker_samples.len());
        }

        let now_us = timer.get_counter().ticks();
        let elapsed_us = now_us.wrapping_sub(last_sample_us);
        last_sample_us = now_us;
        sample_phase += elapsed_us * SAMPLE_HZ as u64;
        while sample_phase >= 1_000_000 {
            sample_phase -= 1_000_000;
            let raw = adc.read(&mut mic_pin).unwrap_or(ADC_MIDPOINT);
            let mic_pcm = audio::adc12_to_pcm16(raw, ADC_MIDPOINT);
            let [lo, hi] = mic_pcm.to_le_bytes();
            mic_packet[mic_samples * 2] = lo;
            mic_packet[mic_samples * 2 + 1] = hi;
            mic_samples += 1;

            let speaker_pcm = if speaker_pos < speaker_len {
                let s = speaker_samples[speaker_pos];
                speaker_pos += 1;
                s
            } else {
                0
            };
            let duty = audio::pcm16_to_pwm8(speaker_pcm) as u16;
            let max = speaker_pwm.max_duty_cycle();
            let scaled = duty * max / 255;
            let _ = speaker_pwm.set_duty_cycle(scaled);

            if mic_samples == SAMPLES_PER_USB_MS {
                let _ = audio.write(&mic_packet);
                mic_samples = 0;
            }
        }

        // PTT contact is active low. Report changes only after debounce.
        let contact_closed = matches!(ptt_pin.is_low(), Ok(true));
        let now_ms = (now_us / 1_000) as u32;
        if let Some(pressed) = ptt.update(contact_closed, now_ms) {
            if pressed != last_ptt_report {
                let report = [u8::from(pressed)];
                if ptt_hid.push_raw_input(&report).is_ok() {
                    last_ptt_report = pressed;
                }
            }
        }
    }
}

pub struct PttDebouncer {
    pressed: bool,
    candidate: bool,
    candidate_since_ms: u32,
}

impl PttDebouncer {
    pub const fn new() -> Self {
        Self {
            pressed: false,
            candidate: false,
            candidate_since_ms: 0,
        }
    }

    pub fn update(&mut self, contact_closed: bool, now_ms: u32) -> Option<bool> {
        if contact_closed != self.candidate {
            self.candidate = contact_closed;
            self.candidate_since_ms = now_ms;
            return None;
        }
        if self.candidate != self.pressed && now_ms.wrapping_sub(self.candidate_since_ms) >= 20 {
            self.pressed = self.candidate;
            return Some(self.pressed);
        }
        None
    }
}

pub mod audio {
    pub fn adc12_to_pcm16(raw: u16, midpoint: u16) -> i16 {
        let centered = raw as i32 - midpoint as i32;
        (centered * 16).clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }

    pub fn pcm16_to_pwm8(sample: i16) -> u8 {
        ((sample as i32 >> 8) + 128).clamp(0, 255) as u8
    }
}
