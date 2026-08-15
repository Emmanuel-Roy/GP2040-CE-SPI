#ifndef SPIINPUTDRIVER_H
#define SPIINPUTDRIVER_H

#include "drivers/switchpro/SwitchProDriver.h"
#include "gamepad.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/unique_id.h"
#include <cstdint>

#define SPI_PORT    spi0
#define PIN_SPI_RX  16  // MOSI from Master (GP16)
#define PIN_SPI_CS  17  // CSn from Master (GP17)
#define PIN_SPI_SCK 18  // SCK from Master (GP18)
#define PIN_SPI_TX  19  // MISO to Master (GP19)

#define SLAVE_ID    0   // Default Slave ID for this board

// Contents of ControllerSpiPacket.flags. The target id only needs 2 bits, so
// the Switch Pro's two remaining buttons ride in the top of the same byte.
//
// Why this packet is 9 bytes and not 8: four 8-bit axes (32 bits) plus 18
// buttons is 50 bits of payload, but 8 bytes leaves only 48 after the header
// and CRC. Fitting 8 bytes would require cutting analog resolution or sending
// some inputs less often than others, and neither is acceptable - every input
// keeps full 8-bit range and every input updates at the same rate.
// MUST stay in sync with Hardware/host-src/include/packet.h
#define SPI_TARGET_ID_MASK   0x03
#define SPI_AUX_MASK_HOME    (1 << 6)
#define SPI_AUX_MASK_CAPTURE (1 << 7)

// Bits 2..5 carry which player slots the host currently has enabled, one bit
// per slot. Every packet carries it, so a board can tell whether the host
// actually means to be driving it - which is what the onboard LED reports.
#define SPI_ENABLED_SHIFT    2
#define SPI_ENABLED_MASK     0x3C
#define SPI_ENABLED_SLOTS(flags) (((flags) & SPI_ENABLED_MASK) >> SPI_ENABLED_SHIFT)

#pragma pack(push, 1)
// Unified 9-Byte Master -> Target SPI Packet Structure.
// Carries the complete Switch Pro Controller state every poll: 18 buttons (16
// here plus Home/Capture in flags) and all four analog axes at full 8-bit
// range. Nothing is multiplexed, so no input updates faster than another.
struct ControllerSpiPacket {
    uint8_t  header;    // Byte 0: Always 0x5A
    uint8_t  flags;     // Byte 1: [1:0] Target ID, [6] Home, [7] Capture
    uint16_t buttons;   // Bytes 2..3: 16-bit button bitmask
    uint8_t  lx;        // Byte 4: Left Stick X (0..255, center 128)
    uint8_t  ly;        // Byte 5: Left Stick Y (0..255, center 128)
    uint8_t  rx;        // Byte 6: Right Stick X (0..255, center 128)
    uint8_t  ry;        // Byte 7: Right Stick Y (0..255, center 128)
    uint8_t  crc8;      // Byte 8: Polynomial 0x07 over bytes 0..7
};

// Unified 9-Byte Target -> Master MISO ACK & Telemetry Packet Structure.
// A synchronous SPI transfer clocks the same number of bytes in both
// directions, so this must be the same size as ControllerSpiPacket.
//
// IMPORTANT: only the first 8 bytes are actually transmitted. This PL022's TX
// FIFO holds exactly 8 entries and fill_tx_fifo() preloads the whole ACK in
// one go while the master is idle, so a 9th byte cannot be queued - it is
// silently discarded. The reply has room to spare, so every meaningful field
// and the CRC lives in bytes 0..7 and byte 8 is never written. (This is why
// the command packet may be 9 bytes but the ACK may not.)
struct ControllerSpiAckPacket {
    uint8_t  header;        // Byte 0: Always 0xA5 (ACK Header)
    uint8_t  slave_id;      // Byte 1: Target ID (0..3)
    uint8_t  status_flags;  // Byte 2: Bit 0 = Active, Bit 1 = Valid Packet Rx
    uint8_t  player_leds;   // Byte 3: Active Player LEDs (1..4)
    uint16_t packet_count;  // Bytes 4..5: Received Valid Packet Count
    uint8_t  reserved;      // Byte 6: 0x00
    uint8_t  crc8;          // Byte 7: Polynomial 0x07 over bytes 0..6
    uint8_t  pad;           // Byte 8: NOT transmitted - never written
};

// Bytes of the ACK that are actually transmitted and covered by its CRC.
#define SPI_ACK_VALID_BYTES 8
#pragma pack(pop)

static_assert(sizeof(ControllerSpiPacket) == sizeof(ControllerSpiAckPacket),
              "command and ACK packets must be the same size - a synchronous SPI "
              "transfer clocks equal byte counts in both directions");

class SpiInputDriver : public SwitchProDriver {
public:
    void initialize() override;
    bool process(Gamepad * gamepad) override;

private:
    uint32_t last_packet_time = 0;
    uint8_t  rx_buf[sizeof(ControllerSpiPacket) * 2] = {0};
    size_t   rx_count = 0;
    bool     valid_packet_received = false;
    uint16_t valid_packet_counter = 0;

    // Learned at runtime from the packets this board actually receives, rather
    // than compiled in - chip-select already decides which slave listens, so
    // any packet arriving here is by definition addressed to this board. That
    // lets one firmware image serve all four slots instead of needing four
    // builds that differ only by SLAVE_ID.
    uint8_t  learned_slave_id = SLAVE_ID;

    // This board's hardware unique id, streamed to the master a byte at a time
    // through the ACK's spare byte. Same value this board reports as its USB
    // serial, which is what lets the host tie a slot to a physical device.
    uint8_t  board_id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES] = {0};
    bool     board_id_loaded = false;

    ControllerSpiPacket last_valid_packet = {};
    ControllerSpiAckPacket tx_ack_packet = {};

    uint8_t calculate_crc8(const uint8_t *data, size_t len);
    void reset_to_neutral(Gamepad *gamepad);
    void update_gamepad_state(Gamepad *gamepad, const ControllerSpiPacket &packet);
    void update_ack_packet();
    void fill_tx_fifo();
};

#endif // SPIINPUTDRIVER_H
