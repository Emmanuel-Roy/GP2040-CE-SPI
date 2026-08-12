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

// Safely load ACK bytes into SPI TX FIFO without touching or clearing RX FIFO
static void fill_tx_fifo(const ControllerSpiAckPacket &ack) {
    const uint8_t *ack_bytes = reinterpret_cast<const uint8_t*>(&ack);
    for (size_t i = 0; i < sizeof(ControllerSpiAckPacket); i++) {
        if (spi_is_writable(SPI_PORT)) {
            spi_get_hw(SPI_PORT)->dr = static_cast<uint32_t>(ack_bytes[i]);
        } else {
            break; // TX FIFO full (8 entries)
        }
    }
}

void SpiInputDriver::initialize() {
    SwitchDriver::initialize();

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    if (Storage::getInstance().GetGamepad()) {
        Storage::getInstance().GetGamepad()->debouncedGpio = 0;
        Storage::getInstance().GetGamepad()->clearState();
    }
    if (Storage::getInstance().GetProcessedGamepad()) {
        Storage::getInstance().GetProcessedGamepad()->debouncedGpio = 0;
        Storage::getInstance().GetProcessedGamepad()->clearState();
    }

    spi_init(SPI_PORT, 1 * 1000 * 1000);
    spi_set_slave(SPI_PORT, true);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

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

    update_ack_packet();
    fill_tx_fifo(tx_ack_packet);
}

void SpiInputDriver::reset_to_neutral(Gamepad *gamepad) {
    gamepad->clearState();
    gamepad->state.lx = 0x8000;
    gamepad->state.ly = 0x8000;
    gamepad->state.rx = 0x8000;
    gamepad->state.ry = 0x8000;
    gamepad->hasAnalogTriggers = false;

    if (Storage::getInstance().GetProcessedGamepad()) {
        memcpy(&Storage::getInstance().GetProcessedGamepad()->state, &gamepad->state, sizeof(GamepadState));
    }
}

void SpiInputDriver::update_gamepad_state(Gamepad *gamepad, const ControllerSpiPacket &packet) {
    gamepad->clearState();

    uint16_t b = packet.buttons;

    uint8_t dpad = 0;
    if (b & (1 << 0)) dpad |= GAMEPAD_MASK_UP;
    if (b & (1 << 1)) dpad |= GAMEPAD_MASK_DOWN;
    if (b & (1 << 2)) dpad |= GAMEPAD_MASK_LEFT;
    if (b & (1 << 3)) dpad |= GAMEPAD_MASK_RIGHT;
    gamepad->state.dpad = dpad;

    uint32_t buttons = 0;
    if (b & (1 << 4))  buttons |= GAMEPAD_MASK_B1;  // B
    if (b & (1 << 5))  buttons |= GAMEPAD_MASK_B2;  // A
    if (b & (1 << 6))  buttons |= GAMEPAD_MASK_B3;  // Y
    if (b & (1 << 7))  buttons |= GAMEPAD_MASK_B4;  // X

    if (b & (1 << 8))  buttons |= GAMEPAD_MASK_L1;  // L
    if (b & (1 << 9))  buttons |= GAMEPAD_MASK_R1;  // R
    if (b & (1 << 10)) buttons |= GAMEPAD_MASK_L2;  // ZL
    if (b & (1 << 11)) buttons |= GAMEPAD_MASK_R2;  // ZR

    if (b & (1 << 12)) buttons |= GAMEPAD_MASK_S1;  // Select (-)
    if (b & (1 << 13)) buttons |= GAMEPAD_MASK_S2;  // Start (+)

    if (b & (1 << 14)) buttons |= GAMEPAD_MASK_L3;  // L3
    if (b & (1 << 15)) buttons |= GAMEPAD_MASK_R3;  // R3

    gamepad->state.buttons = buttons;
    gamepad->state.aux = 0;

    gamepad->state.lx = scale_axis_8_to_16(packet.lx);
    gamepad->state.ly = scale_axis_8_to_16(packet.ly);
    gamepad->state.rx = scale_axis_8_to_16(packet.rx);
    gamepad->state.ry = scale_axis_8_to_16(128);

    gamepad->state.lt = (b & (1 << 10)) ? 255 : 0;
    gamepad->state.rt = (b & (1 << 11)) ? 255 : 0;

    gamepad->hasLeftAnalogStick = true;
    gamepad->hasRightAnalogStick = true;
    gamepad->hasAnalogTriggers = false;

    if (Storage::getInstance().GetProcessedGamepad()) {
        memcpy(&Storage::getInstance().GetProcessedGamepad()->state, &gamepad->state, sizeof(GamepadState));
    }
}

bool SpiInputDriver::process(Gamepad *gamepad) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    static size_t rx_bytes_read = 0;
    static uint8_t current_rx[8] = {0};

    while (spi_is_readable(SPI_PORT)) {
        uint8_t byte = static_cast<uint8_t>(spi_get_hw(SPI_PORT)->dr);

        if (rx_bytes_read < 8) {
            current_rx[rx_bytes_read++] = byte;
        }

        if (rx_bytes_read == 8) {
            rx_bytes_read = 0;

            if (current_rx[0] == 0x5A && current_rx[1] == SLAVE_ID) {
                uint8_t expected_crc = calculate_crc8(current_rx, 7);
                if (current_rx[7] == expected_crc) {
                    memcpy(&last_valid_packet, current_rx, sizeof(ControllerSpiPacket));
                    last_packet_time = now;
                    valid_packet_received = true;
                    valid_packet_counter++;
                }
            }
        }
    }

    // Keep TX FIFO filled with latest ACK telemetry packet
    update_ack_packet();
    fill_tx_fifo(tx_ack_packet);

    if (now - last_packet_time > 250) {
        reset_to_neutral(gamepad);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    } else {
        update_gamepad_state(gamepad, last_valid_packet);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
    }

    return SwitchDriver::process(gamepad);
}
