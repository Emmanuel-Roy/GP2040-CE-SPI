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

static void normalize_packet(uint8_t *buf, size_t len) {
    for (size_t i = 0; i < len - 1; i++) {
        buf[i] = (buf[i] << 1) | (buf[i + 1] >> 7);
    }
    buf[len - 1] <<= 1;
}

void SpiInputDriver::update_ack_packet() {
    tx_ack_packet.header = 0xA5;
    // Echo back the slot this board is being addressed as. The master checks
    // rx_ack.slave_id against the target it selected, so this is what lets a
    // single firmware image work in any of the four slots.
    tx_ack_packet.slave_id = learned_slave_id;
    tx_ack_packet.status_flags = valid_packet_received ? 0x03 : 0x01;
    tx_ack_packet.player_leds = 1;
    tx_ack_packet.packet_count = valid_packet_counter;
    // Rotate this board's hardware unique id through the spare ACK byte, one
    // byte per packet, so the master can learn which physical board answers on
    // each slot. The index is derived from the packet counter rather than sent
    // explicitly - the master already receives that counter, so no extra field
    // is needed. It matches the USB serial the same board reports to the host,
    // which is what ties "slot N" to a specific device on the PC.
    if (!board_id_loaded) {
        pico_unique_board_id_t id;
        pico_get_unique_board_id(&id);
        memcpy(board_id, id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
        board_id_loaded = true;   // explicit flag: a real id may begin with 0x00
    }
    tx_ack_packet.reserved = board_id[valid_packet_counter % PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
    tx_ack_packet.crc8 = calculate_crc8(reinterpret_cast<const uint8_t*>(&tx_ack_packet), 7);
}

// Preload the whole ACK packet into the hardware SPI TX FIFO once it has drained.
void SpiInputDriver::fill_tx_fifo() {
    update_ack_packet();
    // Check if Transmit FIFO is empty (TFE bit in SR register)
    if (spi_get_hw(SPI_PORT)->sr & SPI_SSPSR_TFE_BITS) {
        // Exactly SPI_ACK_VALID_BYTES - never more. The FIFO is 8 entries deep
        // and cannot drain while the master is idle between polls, so a 9th
        // write would just be discarded (taking the CRC with it). Writing only
        // what fits also keeps this loop free of any wait, which matters: this
        // runs every process() call and any stall here delays draining the RX
        // FIFO, which is what starves the input path.
        const uint8_t *ack = reinterpret_cast<const uint8_t*>(&tx_ack_packet);
        for (size_t i = 0; i < SPI_ACK_VALID_BYTES; i++) {
            spi_get_hw(SPI_PORT)->dr = static_cast<uint32_t>(ack[i]);
        }
    }
}

void SpiInputDriver::initialize() {
    SwitchProDriver::initialize();

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

    spi_init(SPI_PORT, 4 * 1000 * 1000);   // must match the master exactly
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
    last_valid_packet.flags = SLAVE_ID;   // target id only; Home/Capture cleared
    last_valid_packet.lx = 128;
    last_valid_packet.ly = 128;
    last_valid_packet.rx = 128;
    last_valid_packet.ry = 128;
    last_packet_time = 0;
    valid_packet_received = false;

    fill_tx_fifo();
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

    // Home and Capture ride in the spare bits of the flags byte
    if (packet.flags & SPI_AUX_MASK_HOME)    buttons |= GAMEPAD_MASK_A1;  // Home
    if (packet.flags & SPI_AUX_MASK_CAPTURE) buttons |= GAMEPAD_MASK_A2;  // Capture

    gamepad->state.buttons = buttons;
    gamepad->state.aux = 0;

    gamepad->state.lx = scale_axis_8_to_16(packet.lx);
    gamepad->state.ly = scale_axis_8_to_16(packet.ly);
    gamepad->state.rx = scale_axis_8_to_16(packet.rx);
    gamepad->state.ry = scale_axis_8_to_16(packet.ry);

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
    static uint8_t current_rx[9] = {0};

    while (spi_is_readable(SPI_PORT)) {
        uint8_t byte = static_cast<uint8_t>(spi_get_hw(SPI_PORT)->dr);

        if (rx_bytes_read == 0) {
            if (byte == 0x5A || byte == 0xB4 || byte == 0x2D) {
                current_rx[0] = byte;
                rx_bytes_read = 1;
            }
        } else {
            current_rx[rx_bytes_read++] = byte;
        }

        if (rx_bytes_read == sizeof(ControllerSpiPacket)) {
            rx_bytes_read = 0;

            bool valid = false;
            ControllerSpiPacket pkt = {};

            if (current_rx[0] == 0x5A) {
                if (calculate_crc8(current_rx, 8) == current_rx[8]) {
                    memcpy(&pkt, current_rx, sizeof(ControllerSpiPacket));
                    valid = true;
                }
            }

            if (!valid) {
                uint8_t norm_rx[sizeof(ControllerSpiPacket)];
                memcpy(norm_rx, current_rx, sizeof(norm_rx));
                normalize_packet(norm_rx, sizeof(norm_rx));
                if (norm_rx[0] == 0x5A) {
                    if (calculate_crc8(norm_rx, 8) == norm_rx[8]) {
                        memcpy(&pkt, norm_rx, sizeof(ControllerSpiPacket));
                        valid = true;
                    }
                }
            }

            if (valid) {
                learned_slave_id = pkt.flags & SPI_TARGET_ID_MASK;
                memcpy(&last_valid_packet, &pkt, sizeof(ControllerSpiPacket));
                last_packet_time = now;
                valid_packet_received = true;
                valid_packet_counter++;
            }
        }
    }

    fill_tx_fifo();

    const bool fresh = (now - last_packet_time <= 250);
    if (!fresh) {
        reset_to_neutral(gamepad);
    } else {
        update_gamepad_state(gamepad, last_valid_packet);
    }

    // The LED reports whether the host has this player enabled, read from the
    // enabled bitmask every packet carries, rather than merely whether bytes
    // arrived. Those are the same thing today - the master only polls enabled
    // slots - but this states the intent, so the light keeps meaning "the host
    // is driving this player" even if the master later polls a slot for some
    // other reason. Still gated on fresh traffic: with the host gone there is
    // no enable state to report and the board is not driving anything.
    const uint8_t enabled = SPI_ENABLED_SLOTS(last_valid_packet.flags);
    const bool host_enabled = (enabled >> learned_slave_id) & 1u;
    gpio_put(PICO_DEFAULT_LED_PIN, (fresh && host_enabled) ? 1 : 0);

    return SwitchProDriver::process(gamepad);
}
