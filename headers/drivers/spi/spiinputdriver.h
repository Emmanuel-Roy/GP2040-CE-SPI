#ifndef SPIINPUTDRIVER_H
#define SPIINPUTDRIVER_H

#include "drivers/xinput/XInputDriver.h"
#include "gamepad.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <cstdint>

#define SPI_PORT    spi0
#define PIN_SPI_RX  16  // MOSI from Master (GP16)
#define PIN_SPI_CS  17  // CSn from Master (GP17)
#define PIN_SPI_SCK 18  // SCK from Master (GP18)
#define PIN_SPI_TX  19  // MISO to Master (GP19)

#define SLAVE_ID    0   // Default Slave ID for this board

#pragma pack(push, 1)
// Unified 8-Byte Master -> Target SPI Packet Structure (Fits 8-entry RP2040 PL022 TX FIFO)
struct ControllerSpiPacket {
    uint8_t  header;    // Byte 0: Always 0x5A
    uint8_t  target_id; // Byte 1: Target ID (0..3)
    uint16_t buttons;   // Bytes 2..3: 16-bit button bitmask
    uint8_t  lx;        // Byte 4: Left Stick X (0..255, center 128)
    uint8_t  ly;        // Byte 5: Left Stick Y (0..255, center 128)
    uint8_t  rx;        // Byte 6: Right Stick X (0..255, center 128)
    uint8_t  crc8;      // Byte 7: Polynomial 0x07 over bytes 0..6
};

// Unified 8-Byte Target -> Master MISO ACK & Telemetry Packet Structure
struct ControllerSpiAckPacket {
    uint8_t  header;        // Byte 0: Always 0xA5 (ACK Header)
    uint8_t  slave_id;      // Byte 1: Target ID (0..3)
    uint8_t  status_flags;  // Byte 2: Bit 0 = Active, Bit 1 = Valid Packet Rx
    uint8_t  player_leds;   // Byte 3: Active Player LEDs (1..4)
    uint16_t packet_count;  // Bytes 4..5: Received Valid Packet Count
    uint8_t  reserved;      // Byte 6: 0x00
    uint8_t  crc8;          // Byte 7: Polynomial 0x07 over bytes 0..6
};
#pragma pack(pop)

class SpiInputDriver : public XInputDriver {
public:
    void initialize() override;
    bool process(Gamepad * gamepad) override;

private:
    uint32_t last_packet_time = 0;
    uint8_t  rx_buf[sizeof(ControllerSpiPacket) * 2] = {0};
    size_t   rx_count = 0;
    bool     valid_packet_received = false;
    uint16_t valid_packet_counter = 0;

    ControllerSpiPacket last_valid_packet = {};
    ControllerSpiAckPacket tx_ack_packet = {};

    uint8_t calculate_crc8(const uint8_t *data, size_t len);
    void reset_to_neutral(Gamepad *gamepad);
    void update_gamepad_state(Gamepad *gamepad, const ControllerSpiPacket &packet);
    void update_ack_packet();
};

#endif // SPIINPUTDRIVER_H
