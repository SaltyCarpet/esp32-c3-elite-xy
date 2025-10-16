#include "esp32-hal-gpio.h"
#include "rom/ets_sys.h"
#include "esp32-hal.h"
#include <sys/_stdint.h>
#include "HardwareSerial.h"
#include "MCP4922DMA.h"

#include <string.h>
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_err.h>
#include <soc/gpio_struct.h>

#ifndef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) do { esp_err_t __err = (x); if (__err != ESP_OK) { Serial.printf("ESP_ERROR at %s:%d code=%d\n", __FILE__, __LINE__, (int)__err); abort(); } } while(0)
#endif

// ---------- Config ----------
#ifndef DEFAULT_SPI_HOST
#define DEFAULT_SPI_HOST SPI2_HOST // On ESP32-C3, SPI2_HOST is the general-purpose SPI
#endif

// Host/bus
static spi_device_handle_t s_dev_xy = nullptr;
static bool s_inited = false;

// LDAC control
static int s_ldac_pin = -1;
// Brightness control
static int s_z_pin_1 = -1;
static int s_z_pin_2 = -1;

// ISR: pulse LDAC only after B (Y) completes, so A and B latch together. 
static void IRAM_ATTR spi_post_cb(spi_transaction_t* trans) {
    // No LDAC here — do it when we get results in task context 
    // if ((uintptr_t)trans->user == TX_TAG_B) { ldac_pulse_fast(); } 
    }

static inline void ldac_pulse_fast(int brightness)
{
    // Latch new DAC values (LDAC low)
    gpio_set_level((gpio_num_t)s_ldac_pin, 0);
    gpio_set_level((gpio_num_t)s_z_pin_1, brightness & 0x1);
    gpio_set_level((gpio_num_t)s_z_pin_2, (brightness >> 1) & 0x1);
    // Ensure minimum LDAC low width for MCP4922 (t_LDAC ≥ ~100 ns)
    // loop with respect to CPU Clock (100ns*0.160GHz)/3CpL =~ 5L < 8:
    //for(int i = 0; i < 2; i++)
    //{__asm__ __volatile__("nop");}
    //Serial.println("pulse");
    //ets_delay_us(1);
    // Release LDAC
    gpio_set_level((gpio_num_t)s_ldac_pin, 1);
}

// Helper to write 16-bit big-endian into tx_data
static inline void pack_be16(uint8_t* dst, uint16_t w)
{
    dst[0] = (uint8_t)(w >> 8);
    dst[1] = (uint8_t)(w & 0xFF);
}

void MCP4922_DMA_init_dual(
    int mosi, int sclk,
    int cs_xy, int z_1,
    int z_2, int ldac,
    int clock_hz,
    int /*queue_depth_points unused now*/)
{
  if (s_inited) return;

  s_ldac_pin = ldac;
  s_z_pin_1  = z_1;
  s_z_pin_2  = z_2;

  // LDAC high idle
  {
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << ldac);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level((gpio_num_t)ldac, 1);
  }
  // Z pins
  if (z_1 >= 0) { pinMode(z_1, OUTPUT); digitalWrite(z_1, LOW); }
  if (z_2 >= 0) { pinMode(z_2, OUTPUT); digitalWrite(z_2, LOW); }

  // SPI bus (no MISO)
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = mosi;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = sclk;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 64; // 2 x 16-bit fits easily

  ESP_ERROR_CHECK(spi_bus_initialize(DEFAULT_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

  // Device (mode 0, MSB first), CS controlled by driver
  spi_device_interface_config_t dev = {};
  dev.clock_speed_hz = clock_hz; // try 2-8 MHz first
  dev.mode = 0;                  // CPOL=0, CPHA=0
  dev.spics_io_num = cs_xy;
  dev.queue_size = 4;
  dev.flags = 0;
  dev.pre_cb = nullptr;
  dev.post_cb = nullptr;

  ESP_ERROR_CHECK(spi_bus_add_device(DEFAULT_SPI_HOST, &dev, &s_dev_xy));
  if (!s_dev_xy) {
    Serial.println("SPI device handle is null!");
    return;
  }

  s_inited = true;
  Serial.println("MCP4922 init (blocking SPI) complete");
}

// Blocking, robust sender: sends each XY pair as two 16-bit words and pulses LDAC with Z
void MCP4922_DMA_send_XYZ(const uint16_t* xyz_triplets, int count_points)
{
  if (!s_inited || !s_dev_xy || count_points <= 0) return;

  for (int i = 0; i < count_points; ++i) {
    const uint16_t xw = xyz_triplets[3*i + 0];
    const uint16_t yw = xyz_triplets[3*i + 1];
    const uint16_t z12 = xyz_triplets[3*i + 2];

    // X
    {
      spi_transaction_t t = {};
      t.length = 16;
      t.flags = SPI_TRANS_USE_TXDATA;
      pack_be16(t.tx_data, xw);
      esp_err_t err = spi_device_transmit(s_dev_xy, &t);
      if (err != ESP_OK) {
        Serial.printf("SPI transmit X error: %d\n", err);
        return;
      }
    }
    // Y
    {
      spi_transaction_t t = {};
      t.length = 16;
      t.flags = SPI_TRANS_USE_TXDATA;
      pack_be16(t.tx_data, yw);
      esp_err_t err = spi_device_transmit(s_dev_xy, &t);
      if (err != ESP_OK) {
        Serial.printf("SPI transmit Y error: %d\n", err);
        return;
      }
    }

    // Latch XY with Z2-bit code
    ldac_pulse_fast(z12);
  }
}

void MCP4922_DMA_wait(void)
{
  // Blocking API completes per call; nothing to drain.
}