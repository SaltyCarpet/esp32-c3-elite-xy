#include "esp32-hal-gpio.h"
#include "rom/ets_sys.h"
#include "esp32-hal.h"
#include <sys/_stdint.h>
#include "HardwareSerial.h"
#include "MCP4922DMA.h"
#include "ErrorHandler.h"
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <hal/spi_ll.h>
#include <soc/gpio_struct.h>
#include <soc/io_mux_reg.h>

#ifndef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) do {                                                          \
    esp_err_t __err = (x);                                                               \
    if (__err != ESP_OK) {                                                               \
        Serial.printf("ESP_ERROR at %s:%d code=%d\n", __FILE__, __LINE__, (int)__err);  \
        setStatus(STATUS_ERROR);                                                         \
        abort();                                                                         \
    }                                                                                    \
} while (0)
#endif

#ifndef DEFAULT_SPI_HOST
#define DEFAULT_SPI_HOST SPI2_HOST
#endif

static spi_device_handle_t s_dev_xy = nullptr;
static spi_dev_t* s_hw = nullptr;
static bool s_inited = false;

static int s_cs_xy_pin = -1;
static int s_ldac_pin = -1;
static int s_z_pin_1 = -1;
static int s_z_pin_2 = -1;

static uint32_t s_cs_mask = 0;
static uint32_t s_ldac_mask = 0;
static uint32_t s_z_mask = 0;
static uint32_t s_z_lut[4] = {};

static constexpr int kPreferredFspiMosiPin = 11;
static constexpr int kPreferredFspiSclkPin = 12;
static constexpr int kPreferredManualCsPin = 10;

static inline void gpio_write_high(uint32_t mask)
{
    GPIO.out_w1ts = mask;
}

static inline void gpio_write_low(uint32_t mask)
{
    GPIO.out_w1tc = mask;
}

static void init_fast_gpio(int cs_xy, int z1, int z2, int ldac)
{
    s_cs_xy_pin = cs_xy;
    s_ldac_pin = ldac;
    s_z_pin_1 = z1;
    s_z_pin_2 = z2;

    s_cs_mask = (1UL << s_cs_xy_pin);
    s_ldac_mask = (1UL << s_ldac_pin);
    s_z_mask = (1UL << s_z_pin_1) | (1UL << s_z_pin_2);

    s_z_lut[0] = 0;
    s_z_lut[1] = (1UL << s_z_pin_1);
    s_z_lut[2] = (1UL << s_z_pin_2);
    s_z_lut[3] = (1UL << s_z_pin_1) | (1UL << s_z_pin_2);

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << s_cs_xy_pin) |
                      (1ULL << s_ldac_pin) |
                      (1ULL << s_z_pin_1) |
                      (1ULL << s_z_pin_2);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_write_high(s_cs_mask | s_ldac_mask);
    gpio_write_low(s_z_mask);
}

static void validate_spi_pin_choices(int mosi, int sclk, int cs_xy)
{
    if (mosi == kPreferredFspiMosiPin && sclk == kPreferredFspiSclkPin) {
        Serial.println("SPI using preferred ESP32-S3 FSPI IOMUX pins GPIO11/GPIO12");
    } else {
        Serial.printf("Warning: SPI MOSI/SCLK on GPIO%d/GPIO%d, preferred ESP32-S3 FSPI pins are GPIO11/GPIO12\n",
                      mosi, sclk);
    }

    if (cs_xy != kPreferredManualCsPin) {
        Serial.printf("Warning: manual DAC CS is on GPIO%d, recommended pin is GPIO10 for this wiring\n", cs_xy);
    }
}

static void configure_fspi_iomux_if_possible(int mosi, int sclk)
{
#if defined(IO_MUX_GPIO11_REG) && defined(IO_MUX_GPIO12_REG) && \
    defined(FUNC_GPIO11_FSPID) && defined(FUNC_GPIO12_FSPICLK)
    if (mosi == kPreferredFspiMosiPin && sclk == kPreferredFspiSclkPin) {
        PIN_FUNC_SELECT(IO_MUX_GPIO11_REG, FUNC_GPIO11_FSPID);
        PIN_FUNC_SELECT(IO_MUX_GPIO12_REG, FUNC_GPIO12_FSPICLK);
    }
#else
    (void)mosi;
    (void)sclk;
#endif
}

static void prime_spi_device(spi_device_handle_t dev)
{
    spi_transaction_t t = {};
    t.length = 16;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = 0x00;
    t.tx_data[1] = 0x00;
    ESP_ERROR_CHECK(spi_device_transmit(dev, &t));
}

static inline void IRAM_ATTR spi_wait_idle()
{
    while (s_hw->cmd.usr) {}
}

static inline void IRAM_ATTR latch_with_z(uint8_t brightness)
{
    const uint32_t set_mask = s_z_lut[brightness & 0x3];
    const uint32_t clear_mask = (s_z_mask & ~set_mask) | s_ldac_mask;

    gpio_write_high(set_mask);
    gpio_write_low(clear_mask);
    gpio_write_high(s_ldac_mask);
}

static inline void IRAM_ATTR spi_send_16(uint16_t word)
{
    spi_wait_idle();

    s_hw->data_buf[0] = HAL_SPI_SWAP_DATA_TX(word, 16);
    s_hw->user.usr_mosi = 1;
    s_hw->ms_dlen.ms_data_bitlen = 15;

    gpio_write_low(s_cs_mask);
    s_hw->cmd.usr = 1;
    while (s_hw->cmd.usr) {}
    gpio_write_high(s_cs_mask);
}

void MCP4922_DMA_init_dual(
    int mosi, int sclk,
    int cs_xy, int z_1,
    int z_2, int ldac,
    int clock_hz,
    int /*queue_depth_points*/)
{
    if (s_inited) return;

    init_fast_gpio(cs_xy, z_1, z_2, ldac);
    validate_spi_pin_choices(mosi, sclk, cs_xy);

    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4;

    ESP_ERROR_CHECK(spi_bus_initialize(DEFAULT_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    configure_fspi_iomux_if_possible(mosi, sclk);

    spi_device_interface_config_t dev = {};
    dev.clock_speed_hz = clock_hz;
    dev.mode = 0;
    dev.spics_io_num = -1;
    dev.queue_size = 1;
    dev.flags = SPI_DEVICE_NO_DUMMY;

    ESP_ERROR_CHECK(spi_bus_add_device(DEFAULT_SPI_HOST, &dev, &s_dev_xy));
    prime_spi_device(s_dev_xy);

    s_hw = SPI_LL_GET_HW(DEFAULT_SPI_HOST);
    if (!s_hw) {
        Serial.println("SPI hardware pointer is null");
        setStatus(STATUS_ERROR);
        abort();
    }

    s_hw->user.usr_mosi = 1;
    s_hw->user.usr_miso = 0;

    s_inited = true;
    Serial.println("MCP4922 init (direct SPI hot path) complete");
}

void MCP4922_DMA_send_XYZ(const uint16_t* xyz_triplets, int count_points)
{
    if (!s_inited || !s_hw || count_points <= 0) return;

    for (int i = 0; i < count_points; ++i) {
        const uint16_t xw = xyz_triplets[3 * i + 0];
        const uint16_t yw = xyz_triplets[3 * i + 1];
        const uint16_t z12 = xyz_triplets[3 * i + 2];

        spi_send_16(xw);
        spi_send_16(yw);
        latch_with_z((uint8_t)(z12 & 0x3));
    }
}

void MCP4922_DMA_wait(void)
{
    if (!s_inited || !s_hw) return;
    spi_wait_idle();
}
