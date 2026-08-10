#ifndef _SPI_SLAVE_INPUT_H_
#define _SPI_SLAVE_INPUT_H_

#include <string>
#include <cstdint>
#include "gpaddon.h"
#include "gamepad.h"
#include "storagemanager.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"

#define SpiSlaveInputName "SpiSlaveInput"

// Slave SPI Pin Definitions (RP2040 / Pico 1)
#ifndef SLAVE_SPI_PORT
#define SLAVE_SPI_PORT spi0
#endif

#ifndef SLAVE_SCK_PIN
#define SLAVE_SCK_PIN 18
#endif

#ifndef SLAVE_MOSI_PIN
#define SLAVE_MOSI_PIN 16
#endif

#ifndef SLAVE_CS_PIN
#define SLAVE_CS_PIN 17
#endif

#define SPI_SLAVE_SYNC_BYTE 0x5A
#define SPI_WATCHDOG_TIMEOUT_MS 50

#pragma pack(push, 1)
struct SpiSlavePacket {
    uint8_t  header;    // Sync byte (0x5A)
    uint16_t buttons;   // Bitmask for GP2040-CE logical buttons
    uint8_t  lx;        // Left Stick X (0..255, center 128)
    uint8_t  ly;        // Left Stick Y (0..255, center 128)
    uint8_t  rx;        // Right Stick X (0..255, center 128)
    uint8_t  ry;        // Right Stick Y (0..255, center 128)
    uint8_t  lt;        // Left Trigger (0..255)
    uint8_t  rt;        // Right Trigger (0..255)
    uint8_t  sequence;  // Frame counter
    uint8_t  crc8;      // CRC-8 checksum over bytes 0..9
};
#pragma pack(pop)

static_assert(sizeof(SpiSlavePacket) == 11, "SpiSlavePacket must be 11 bytes!");

class SpiSlaveInput : public GPAddon {
public:
    virtual bool available() override { return true; }
    virtual void setup() override;
    virtual void process() override;
    virtual void preprocess() override {}
    virtual void postprocess(bool sent) override {}
    virtual void reinit() override {}
    virtual std::string name() override { return SpiSlaveInputName; }

private:
    uint32_t last_packet_time_ms = 0;
    SpiSlavePacket last_valid_packet;

    uint8_t calculate_crc8(const uint8_t* data, size_t len);
    void apply_neutral_state(Gamepad* gamepad);
    void apply_packet_state(Gamepad* gamepad, const SpiSlavePacket& packet);
};

#endif // _SPI_SLAVE_INPUT_H_
