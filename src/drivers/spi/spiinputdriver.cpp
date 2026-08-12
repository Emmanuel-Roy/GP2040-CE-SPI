#include "drivers/spi/spiinputdriver.h"
#include "storagemanager.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include <cstring>

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

static inline uint16_t scale_axis_8_to_16(uint8_t val) {
    if (val == 128) return 0x8000; // 32768 exact center (>> 8 = 128)
    if (val == 255) return 0xFFFF; // 65535 max (>> 8 = 255)
    if (val == 0)   return 0x0000; // 0 min (>> 8 = 0)
    return static_cast<uint16_t>(val) * 257;
}

uint8_t SpiInputDriver::calculate_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void SpiInputDriver::update_ack_packet() {
    tx_ack_packet.header = 0xA5;
    tx_ack_packet.slave_id = SLAVE_ID;
    tx_ack_packet.status_flags = valid_packet_received ? 0x03 : 0x01;
    tx_ack_packet.player_leds = 1;
    tx_ack_packet.packet_count = valid_packet_counter;
    tx_ack_packet.reserved = 0x00;
    tx_ack_packet.crc8 = calculate_crc8(reinterpret_cast<const uint8_t*>(&tx_ack_packet), 7);
}

// Safely load all 8 bytes of ACK packet into RP2040 hardware SPI TX FIFO without blocking
static void fill_tx_fifo(const ControllerSpiAckPacket &ack) {
    // Drain any leftover bytes in RX FIFO
    while (spi_is_readable(SPI_PORT)) {
        (void)spi_get_hw(SPI_PORT)->dr;
    }

    const uint8_t *ack_bytes = reinterpret_cast<const uint8_t*>(&ack);
    for (size_t i = 0; i < sizeof(ControllerSpiAckPacket); i++) {
        if (spi_is_writable(SPI_PORT)) {
            spi_get_hw(SPI_PORT)->dr = static_cast<uint32_t>(ack_bytes[i]);
        } else {
            break; // TX FIFO full (8 bytes loaded); never block CPU in slave mode
        }
    }
}

void SpiInputDriver::initialize() {
    XInputDriver::initialize();

    // Initialize onboard LED (GP25) for visual SPI & USB hardware diagnostics
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // Disable physical GPIO button debouncer on slave RP2040
    if (Storage::getInstance().GetGamepad()) {
        Storage::getInstance().GetGamepad()->debouncedGpio = 0;
        Storage::getInstance().GetGamepad()->clearState();
    }
    if (Storage::getInstance().GetProcessedGamepad()) {
        Storage::getInstance().GetProcessedGamepad()->debouncedGpio = 0;
        Storage::getInstance().GetProcessedGamepad()->clearState();
    }

    // Initialize SPI0 at 1 MHz in SLAVE mode using SPI CPHA=1 for stable multi-byte transfers
    spi_init(SPI_PORT, 1 * 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

    // Configure all 4 pins (RX=GP16, CSn=GP17, SCK=GP18, TX=GP19) into hardware SPI mode ONCE
    gpio_set_function(PIN_SPI_RX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_CS, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_TX, GPIO_FUNC_SPI);

    gpio_pull_up(PIN_SPI_CS);
    gpio_pull_up(PIN_SPI_SCK);

    rx_count = 0;
    valid_packet_counter = 0;
    memset(rx_buf, 0, sizeof(rx_buf));
    memset(&last_valid_packet, 0, sizeof(last_valid_packet));
    last_valid_packet.header = 0x5A;
    last_valid_packet.target_id = SLAVE_ID;
    last_valid_packet.lx = 128;
    last_valid_packet.ly = 128;
    last_valid_packet.rx = 128;
    last_packet_time = 0;
    valid_packet_received = false;

    // Pre-load initial 8-byte ACK packet into TX FIFO
    update_ack_packet();
    fill_tx_fifo(tx_ack_packet);
}

void SpiInputDriver::reset_to_neutral(Gamepad *gamepad) {
    gamepad->clearState();
    gamepad->state.lx = 0x8000; // 32768 (>> 8 = 128 exact center)
    gamepad->state.ly = 0x8000;
    gamepad->state.rx = 0x8000;
    gamepad->state.ry = 0x8000;
    gamepad->hasAnalogTriggers = false;

    if (Storage::getInstance().GetProcessedGamepad()) {
        memcpy(&Storage::getInstance().GetProcessedGamepad()->state, &gamepad->state, sizeof(GamepadState));
    }
}

void SpiInputDriver::update_gamepad_state(Gamepad *gamepad, const ControllerSpiPacket &packet) {
    // 0. Reset gamepad state completely on EVERY new SPI frame (no button sticking)
    gamepad->clearState();

    uint16_t b = packet.buttons;

    // 1. Explicit D-Pad Mapping (Bits 0..3)
    uint8_t dpad = 0;
    if (b & (1 << 0)) dpad |= GAMEPAD_MASK_UP;
    if (b & (1 << 1)) dpad |= GAMEPAD_MASK_DOWN;
    if (b & (1 << 2)) dpad |= GAMEPAD_MASK_LEFT;
    if (b & (1 << 3)) dpad |= GAMEPAD_MASK_RIGHT;
    gamepad->state.dpad = dpad;

    // 2. Explicit GP2040-CE Button Bitmask Mapping (Bits 4..15)
    uint32_t buttons = 0;
    if (b & (1 << 4))  buttons |= GAMEPAD_MASK_B1;  // B (Switch B)
    if (b & (1 << 5))  buttons |= GAMEPAD_MASK_B2;  // A (Switch A)
    if (b & (1 << 6))  buttons |= GAMEPAD_MASK_B3;  // Y (Switch Y)
    if (b & (1 << 7))  buttons |= GAMEPAD_MASK_B4;  // X (Switch X)

    if (b & (1 << 8))  buttons |= GAMEPAD_MASK_L1;  // L Bumper
    if (b & (1 << 9))  buttons |= GAMEPAD_MASK_R1;  // R Bumper
    if (b & (1 << 10)) buttons |= GAMEPAD_MASK_L2;  // ZL Trigger
    if (b & (1 << 11)) buttons |= GAMEPAD_MASK_R2;  // ZR Trigger

    if (b & (1 << 12)) buttons |= GAMEPAD_MASK_S1;  // Select (-)
    if (b & (1 << 13)) buttons |= GAMEPAD_MASK_S2;  // Start (+)

    if (b & (1 << 14)) buttons |= GAMEPAD_MASK_L3;  // L3 Click
    if (b & (1 << 15)) buttons |= GAMEPAD_MASK_R3;  // R3 Click

    gamepad->state.buttons = buttons;
    gamepad->state.aux = 0;

    // 3. Map 8-bit analog sticks to 16-bit range with 0x8000 (32768) center
    gamepad->state.lx = scale_axis_8_to_16(packet.lx);
    gamepad->state.ly = scale_axis_8_to_16(packet.ly);
    gamepad->state.rx = scale_axis_8_to_16(packet.rx);
    gamepad->state.ry = scale_axis_8_to_16(128); // Neutral RY

    gamepad->state.lt = (b & (1 << 10)) ? 255 : 0;
    gamepad->state.rt = (b & (1 << 11)) ? 255 : 0;

    gamepad->hasLeftAnalogStick = true;
    gamepad->hasRightAnalogStick = true;
    gamepad->hasAnalogTriggers = false;

    // Copy to processedGamepad so GP2040-CE subsystems see identical state
    if (Storage::getInstance().GetProcessedGamepad()) {
        memcpy(&Storage::getInstance().GetProcessedGamepad()->state, &gamepad->state, sizeof(GamepadState));
    }
}

bool SpiInputDriver::process(Gamepad *gamepad) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    static size_t rx_bytes_read = 0;
    static uint8_t current_rx[8] = {0};

    // 1. Drain RX FIFO as Master clocks SCK bytes in
    while (spi_is_readable(SPI_PORT)) {
        uint8_t byte = static_cast<uint8_t>(spi_get_hw(SPI_PORT)->dr);

        if (rx_bytes_read < 8) {
            current_rx[rx_bytes_read++] = byte;
        }

        // When a full 8-byte transfer is completed by Master:
        if (rx_bytes_read == 8) {
            rx_bytes_read = 0; // Reset for next transfer

            if (current_rx[0] == 0x5A && current_rx[1] == SLAVE_ID) {
                uint8_t expected_crc = calculate_crc8(current_rx, 7);
                if (current_rx[7] == expected_crc) {
                    memcpy(&last_valid_packet, current_rx, sizeof(ControllerSpiPacket));
                    last_packet_time = now;
                    valid_packet_received = true;
                    valid_packet_counter++;
                }
            }

            // Immediately update and pre-load the next 8-byte ACK into TX FIFO
            update_ack_packet();
            fill_tx_fifo(tx_ack_packet);
        }
    }

    // 2. Enforce Gamepad State
    bool spi_active = valid_packet_received && (now - last_packet_time <= 200);
    if (spi_active) {
        update_gamepad_state(gamepad, last_valid_packet);
    } else {
        reset_to_neutral(gamepad);
        valid_packet_received = false;
    }

    // 3. Hardware LED Status Diagnostic Indicator (GP25)
    bool usb_mounted = tud_mounted();
    if (!usb_mounted) {
        gpio_put(PICO_DEFAULT_LED_PIN, (now / 100) % 2); // 10 Hz Fast Blink
    } else if (!spi_active) {
        gpio_put(PICO_DEFAULT_LED_PIN, (now / 500) % 2); // 1 Hz Slow Blink
    } else {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);               // Solid ON
    }

    return XInputDriver::process(gamepad);
}
