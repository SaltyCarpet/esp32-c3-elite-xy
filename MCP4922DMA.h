#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize MCP4922 DMA SPI device.
// - mosi, sclk, cs, ldac: GPIO pins
// - clock_hz: SPI clock (e.g., 20'000'000)
// - queue_depth: number of transactions in-flight (e.g., 16)
void MCP4922_DMA_init_dual(
    int mosi, int sclk,
    int cs_xy, int z_1,
    int z_2, int ldac,
    int clock_hz,
    int queue_depth_points);

// Send an array of 32-bit words, one word per XY point:
//   word = (cmdA << 16) | cmdB
// Each 16-bit cmd is a full MCP4922 frame (control bits + 12-bit value).
// Non-blocking per transaction; internally pipelines and drains as needed.
void MCP4922_DMA_send_XYZ(const uint16_t* xyz_triplets, int count_points);

// Wait until all queued transactions have completed.
void MCP4922_DMA_wait(void);

// Helper encoders for MCP4922 16-bit command frames.
static inline uint16_t MCP4922_cmdX(uint16_t v)
{
    // A/B=0 (A), BUF=1, GA=1 (1x), SHDN=1, 12-bit data
    return (uint16_t)((v & 0x0FFF) | 0b0111000000000000);
}

static inline uint16_t MCP4922_cmdY(uint16_t v)
{
    // A/B=1 (B), BUF=1, GA=1 (1x), SHDN=1, 12-bit data
    return (uint16_t)((v & 0x0FFF) | 0b1111000000000000);
}

static inline uint16_t MCP4922_cmdZ(uint16_t v)
{
    // A/B=0 (A), BUF=1, GA=1 (1x), SHDN=1, 12-bit data
    return (uint16_t)(v & 0x3);
}

#ifdef __cplusplus
}
#endif
