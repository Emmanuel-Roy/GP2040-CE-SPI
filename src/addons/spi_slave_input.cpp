#include "addons/spi_slave_input.h"
#include "storagemanager.h"
#include "helper.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

uint8_t SpiSlaveInput::calculate_crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void SpiSlaveInput::setup() {
    // Initialize SPI0 in Slave Mode at 10 MHz
    spi_init(SLAVE_SPI_PORT, 10000000);
    spi_set_slave(SLAVE_SPI_PORT, true);

    // Map GPIO pins for SPI Slave (SCK=18, MOSI=16, CS=17)
    gpio_set_function(SLAVE_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SLAVE_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SLAVE_CS_PIN, GPIO_FUNC_SPI);

    last_packet_time_ms = getMillis();
}

void SpiSlaveInput::apply_neutral_state(Gamepad* gamepad) {
    gamepad->state.buttons = 0;
    gamepad->state.dpad = 0;
    gamepad->state.lx = GAMEPAD_JOYSTICK_MID;
    gamepad->state.ly = GAMEPAD_JOYSTICK_MID;
    gamepad->state.rx = GAMEPAD_JOYSTICK_MID;
    gamepad->state.ry = GAMEPAD_JOYSTICK_MID;
    gamepad->state.lt = 0;
    gamepad->state.rt = 0;
}

void SpiSlaveInput::apply_packet_state(Gamepad* gamepad, const SpiSlavePacket& packet) {
    // Scale 8-bit inputs (0..255) to GP2040-CE 16-bit range (0..65535)
    gamepad->state.lx = static_cast<uint16_t>(packet.lx) * 257;
    gamepad->state.ly = static_cast<uint16_t>(packet.ly) * 257;
    gamepad->state.rx = static_cast<uint16_t>(packet.rx) * 257;
    gamepad->state.ry = static_cast<uint16_t>(packet.ry) * 257;
    gamepad->state.lt = static_cast<uint16_t>(packet.lt) * 257;
    gamepad->state.rt = static_cast<uint16_t>(packet.rt) * 257;

    // Apply button bitmask & D-Pad bits directly
    gamepad->state.buttons = packet.buttons & 0x3FFF; // Lower 14 bits for buttons
    gamepad->state.dpad = (packet.buttons >> 14) & 0x03; // Bits 14 & 15 for D-Pad
}

void SpiSlaveInput::process() {
    Gamepad* gamepad = Storage::getInstance().GetGamepad();
    uint32_t now_ms = getMillis();

    // Check if data is available in SPI hardware FIFO
    if (spi_is_readable(SLAVE_SPI_PORT)) {
        uint8_t buffer[sizeof(SpiSlavePacket)];
        size_t bytes_read = 0;

        while (spi_is_readable(SLAVE_SPI_PORT) && bytes_read < sizeof(SpiSlavePacket)) {
            spi_read_blocking(SLAVE_SPI_PORT, 0, &buffer[bytes_read], 1);
            bytes_read++;
        }

        if (bytes_read == sizeof(SpiSlavePacket)) {
            SpiSlavePacket* packet = reinterpret_cast<SpiSlavePacket*>(buffer);

            // Validate 0x5A header sync byte and CRC-8 checksum
            if (packet->header == SPI_SLAVE_SYNC_BYTE) {
                uint8_t expected_crc = calculate_crc8(buffer, 10);
                if (packet->crc8 == expected_crc) {
                    last_valid_packet = *packet;
                    last_packet_time_ms = now_ms;
                }
            }
        }
    }

    // Apply last valid packet state or reset to neutral if watchdog timed out (>50 ms)
    if (now_ms - last_packet_time_ms <= SPI_WATCHDOG_TIMEOUT_MS) {
        apply_packet_state(gamepad, last_valid_packet);
    } else {
        apply_neutral_state(gamepad);
    }
}
