
#include "atm90e32.h"
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <math.h>
#include <esp_timer.h>

static const char *TAG = "ATM90E32";

static void delay_us(uint32_t us) {
    uint64_t start = esp_timer_get_time();
    while (esp_timer_get_time() - start < us) {
        // Busy wait
    }
}

void atm90e32_reset(atm90e32_config_t *config) {
    ESP_LOGI(TAG, "Resetting ATM90E32, RESET pin (GPIO %d) low", config->reset_pin);
    gpio_set_level(config->reset_pin, 0); // Reset low
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2s low
    ESP_LOGI(TAG, "RESET pin (GPIO %d) high", config->reset_pin);
    gpio_set_level(config->reset_pin, 1); // Reset high
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2s high
}
void atm90e32_init(atm90e32_config_t *config) {
    // Configure CS and Reset pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->cs_pin) | (1ULL << config->reset_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(config->cs_pin, 1); // CS high by default
    gpio_set_level(config->reset_pin, 1); // Reset high by default

    ESP_LOGI(TAG, "Initializing ATM90E32");

    // Perform hardware reset
    atm90e32_reset(config);

    // Perform soft reset
    atm90e32_comm_energy_ic(config, WRITE, SoftReset, 0x789A);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Enable register config access
    atm90e32_comm_energy_ic(config, WRITE, CfgRegAccEn, 0x55AA);
    atm90e32_comm_energy_ic(config, WRITE, MeterEn, 0x0000); // Disable metering initially

    // Voltage sag and frequency thresholds
    uint16_t v_sag_th, freq_hi_thresh, freq_lo_thresh;
    if (config->line_freq == 4485 || config->line_freq == 4231) {
        v_sag_th = (90 * 100 * 1.414213562) / (2 * config->u_gain / 32768.0);
        freq_hi_thresh = 61 * 100;
        freq_lo_thresh = 59 * 100;
    } else {
        v_sag_th = (190 * 100 * 1.414213562) / (2 * config->u_gain / 32768.0);
        freq_hi_thresh = 51 * 100;
        freq_lo_thresh = 49 * 100;
    }

    // Write configuration registers
    atm90e32_comm_energy_ic(config, WRITE, SagPeakDetCfg, 0x143F);
    atm90e32_comm_energy_ic(config, WRITE, SagTh, v_sag_th);
    atm90e32_comm_energy_ic(config, WRITE, FreqHiTh, freq_hi_thresh);
    atm90e32_comm_energy_ic(config, WRITE, FreqLoTh, freq_lo_thresh);
    atm90e32_comm_energy_ic(config, WRITE, EMMIntEn0, 0xB76F);
    atm90e32_comm_energy_ic(config, WRITE, EMMIntEn1, 0xDDFD);
    atm90e32_comm_energy_ic(config, WRITE, EMMIntState0, 0x0001);
    atm90e32_comm_energy_ic(config, WRITE, EMMIntState1, 0x0001);
    atm90e32_comm_energy_ic(config, WRITE, ZXConfig, 0xD654);

    // Metering config
    uint32_t plconst = 0x0861c468 / 1; // K_I = 1 for currents up to 250A
    atm90e32_comm_energy_ic(config, WRITE, PLconstH, plconst >> 16);
    atm90e32_comm_energy_ic(config, WRITE, PLconstL, plconst & 0xFFFF);
    atm90e32_comm_energy_ic(config, WRITE, MMode0, config->line_freq);
    atm90e32_comm_energy_ic(config, WRITE, MMode1, config->pga_gain);

    // Startup thresholds
    atm90e32_comm_energy_ic(config, WRITE, PStartTh, 0x1D4C);
    atm90e32_comm_energy_ic(config, WRITE, QStartTh, 0x1D4C);
    atm90e32_comm_energy_ic(config, WRITE, SStartTh, 0x1D4C);
    atm90e32_comm_energy_ic(config, WRITE, PPhaseTh, 0x02EE);
    atm90e32_comm_energy_ic(config, WRITE, QPhaseTh, 0x02EE);
    atm90e32_comm_energy_ic(config, WRITE, SPhaseTh, 0x02EE);

    // Calibration registers
    atm90e32_comm_energy_ic(config, WRITE, PQGainA, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PhiA, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PQGainB, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PhiB, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PQGainC, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PhiC, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PoffsetA, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, QoffsetA, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PoffsetB, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, QoffsetB, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PoffsetC, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, QoffsetC, 0x0000);

    // Harmonic calibration
    atm90e32_comm_energy_ic(config, WRITE, POffsetAF, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, POffsetBF, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, POffsetCF, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PGainAF, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PGainBF, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, PGainCF, 0x0000);

    // Measurement calibration
    atm90e32_comm_energy_ic(config, WRITE, UgainA, config->u_gain);
    atm90e32_comm_energy_ic(config, WRITE, IgainA, config->i_gain_a);
    atm90e32_comm_energy_ic(config, WRITE, UoffsetA, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, IoffsetA, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, UgainB, config->u_gain);
    atm90e32_comm_energy_ic(config, WRITE, IgainB, config->i_gain_b);
    atm90e32_comm_energy_ic(config, WRITE, UoffsetB, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, IoffsetB, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, UgainC, config->u_gain);
    atm90e32_comm_energy_ic(config, WRITE, IgainC, config->i_gain_c);
    atm90e32_comm_energy_ic(config, WRITE, UoffsetC, 0x0000);
    atm90e32_comm_energy_ic(config, WRITE, IoffsetC, 0x0000);

    atm90e32_comm_energy_ic(config, WRITE, CfgRegAccEn, 0x0000); // End configuration
    ESP_LOGI(TAG, "Voltage Gain: %d", atm90e32_comm_energy_ic(config, READ, UgainA, 0xFFFF));

    vTaskDelay(pdMS_TO_TICKS(100));
    atm90e32_comm_energy_ic(config, WRITE, MeterEn, 0x0001); // Enable metering
}

uint16_t atm90e32_comm_energy_ic(atm90e32_config_t *config, uint8_t rw, uint16_t address, uint16_t val) {
    esp_err_t ret;
    spi_transaction_t trans = {0};
    uint8_t tx_data[4] = {0};
    uint8_t rx_data[4] = {0};
    uint16_t output;

    // Prepare address and value
    address |= rw << 15; // Set R/W flag (MSB)
    tx_data[0] = address >> 8; // Address MSB
    tx_data[1] = address & 0xFF; // Address LSB
    tx_data[2] = val >> 8; // Value MSB
    tx_data[3] = val & 0xFF; // Value LSB

    trans.tx_buffer = tx_data;
    trans.rx_buffer = rx_data;
    trans.length = 4 * 8; // 4 bytes
    trans.rxlength = 4 * 8; // Always receive 4 bytes

    // Log SPI transaction
    ESP_LOGI(TAG, "SPI TX [Addr: 0x%04x, RW: %d]: %02x %02x %02x %02x", 
             (address & 0x7FFF), rw, tx_data[0], tx_data[1], tx_data[2], tx_data[3]);

    // Perform SPI transaction
    ESP_LOGI(TAG, "CS (GPIO %d) set low", config->cs_pin);
    gpio_set_level(config->cs_pin, 0); // CS low
    delay_us(200); // Increased delay
    ret = spi_device_polling_transmit(config->spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transaction failed: %s", esp_err_to_name(ret));
    }
    delay_us(200); // Increased delay
    ESP_LOGI(TAG, "CS (GPIO %d) set high", config->cs_pin);
    gpio_set_level(config->cs_pin, 1); // CS high
    delay_us(200); // Increased delay

    if (rw == READ) {
        output = (rx_data[2] << 8) | rx_data[3];
        ESP_LOGI(TAG, "SPI RX: %02x %02x %02x %02x, Output: 0x%04x", 
                 rx_data[0], rx_data[1], rx_data[2], rx_data[3], output);
        return output;
    }
    return val;
}

int32_t atm90e32_read32_register(atm90e32_config_t *config, int16_t regh_addr, int16_t regl_addr) {
    int32_t val, val_h, val_l;
    val_h = atm90e32_comm_energy_ic(config, READ, regh_addr, 0xFFFF);
    val_l = atm90e32_comm_energy_ic(config, READ, regl_addr, 0xFFFF);
    val = atm90e32_comm_energy_ic(config, READ, regh_addr, 0xFFFF);

    val = val_h << 16;
    val |= val_l; // Concatenate to make 32-bit number
    return val;
}

double atm90e32_calculate_vi_offset(atm90e32_config_t *config, uint16_t regh_addr, uint16_t regl_addr) {
    uint32_t val, val_h, val_l;
    uint16_t offset;
    val_h = atm90e32_comm_energy_ic(config, READ, regh_addr, 0xFFFF);
    val_l = atm90e32_comm_energy_ic(config, READ, regl_addr, 0xFFFF);
    val = atm90e32_comm_energy_ic(config, READ, regh_addr, 0xFFFF);

    val = val_h << 16;
    val |= val_l;
    val = val >> 7; // Right shift 7 bits
    val = (~val) + 1; // 2's complement
    offset = val; // Keep lower 16 bits
    return (double)offset;
}

double atm90e32_calculate_power_offset(atm90e32_config_t *config, uint16_t regh_addr, uint16_t regl_addr) {
    uint32_t val, val_h, val_l;
    uint16_t offset;
    val_h = atm90e32_comm_energy_ic(config, READ, regh_addr, 0xFFFF);
    val_l = atm90e32_comm_energy_ic(config, READ, regl_addr, 0xFFFF);
    val = atm90e32_comm_energy_ic(config, READ, regh_addr, 0xFFFF);

    val = val_h << 16;
    val |= val_l;
    val = (~val) + 1; // 2's complement
    offset = val; // Keep lower 16 bits
    return (double)offset;
}

double atm90e32_calibrate_vi(atm90e32_config_t *config, uint16_t reg, uint16_t actual_val) {
    uint16_t gain, val, m, gain_reg;
    // Sample reading
    val = atm90e32_comm_energy_ic(config, READ, reg, 0xFFFF);
    val += atm90e32_comm_energy_ic(config, READ, reg, 0xFFFF);
    val += atm90e32_comm_energy_ic(config, READ, reg, 0xFFFF);
    val += atm90e32_comm_energy_ic(config, READ, reg, 0xFFFF);

    // Determine gain register
    switch (reg) {
        case UrmsA: gain_reg = UgainA; break;
        case UrmsB: gain_reg = UgainB; break;
        case UrmsC: gain_reg = UgainC; break;
        case IrmsA: gain_reg = IgainA; break;
        case IrmsB: gain_reg = IgainB; break;
        case IrmsC: gain_reg = IgainC; break;
        default: return 0.0; // Invalid register
    }

    gain = atm90e32_comm_energy_ic(config, READ, gain_reg, 0xFFFF);
    m = actual_val;
    m = ((m * gain) / val);
    gain = m;

    atm90e32_comm_energy_ic(config, WRITE, gain_reg, gain);
    return (double)gain;
}

// Voltage
double atm90e32_get_line_voltage_a(atm90e32_config_t *config) {
    uint16_t voltage = atm90e32_comm_energy_ic(config, READ, UrmsA, 0xFFFF);
    return (double)voltage / 100.0;
}

double atm90e32_get_line_voltage_b(atm90e32_config_t *config) {
    uint16_t voltage = atm90e32_comm_energy_ic(config, READ, UrmsB, 0xFFFF);
    return (double)voltage / 100.0;
}

double atm90e32_get_line_voltage_c(atm90e32_config_t *config) {
    uint16_t voltage = atm90e32_comm_energy_ic(config, READ, UrmsC, 0xFFFF);
    return (double)voltage / 100.0;
}

// Current
double atm90e32_get_line_current_a(atm90e32_config_t *config) {
    uint16_t current = atm90e32_comm_energy_ic(config, READ, IrmsA, 0xFFFF);
    return (double)current / 1000.0;
}

double atm90e32_get_line_current_b(atm90e32_config_t *config) {
    uint16_t current = atm90e32_comm_energy_ic(config, READ, IrmsB, 0xFFFF);
    return (double)current / 1000.0;
}

double atm90e32_get_line_current_c(atm90e32_config_t *config) {
    uint16_t current = atm90e32_comm_energy_ic(config, READ, IrmsC, 0xFFFF);
    return (double)current / 1000.0;
}

double atm90e32_get_line_current_n(atm90e32_config_t *config) {
    uint16_t current = atm90e32_comm_energy_ic(config, READ, IrmsN, 0xFFFF);
    return (double)current / 1000.0;
}

// Active Power
double atm90e32_get_active_power_a(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, PmeanA, PmeanALSB);
    return (double)val * 0.00032;
}

double atm90e32_get_active_power_b(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, PmeanB, PmeanBLSB);
    return (double)val * 0.00032;
}

double atm90e32_get_active_power_c(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, PmeanC, PmeanCLSB);
    return (double)val * 0.00032;
}

double atm90e32_get_total_active_power(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, PmeanT, PmeanTLSB);
    return (double)val * 0.00032;
}

// Active Fundamental/Harmonic Power
double atm90e32_get_total_active_fund_power(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, PmeanTF, PmeanTFLSB);
    return (double)val * 0.00032;
}

double atm90e32_get_total_active_har_power(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, PmeanTH, PmeanTHLSB);
    return (double)val * 0.00032;
}

// Reactive Power
double atm90e32_get_reactive_power_a(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, QmeanA, QmeanALSB);
    return (double)val * 0.00032;
}

double atm90e32_get_reactive_power_b(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, QmeanB, QmeanBLSB);
    return (double)val * 0.00032;
}

double atm90e32_get_reactive_power_c(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, QmeanC, QmeanCLSB);
    return (double)val * 0.00032;
}

double atm90e32_get_total_reactive_power(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, QmeanT, QmeanTLSB);
    return (double)val * 0.00032;
}

// Apparent Power
double atm90e32_get_apparent_power_a(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, SmeanA, SmeanALSB);
    return (double)val * 0.00032;
}

double atm90e32_get_apparent_power_b(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, SmeanB, SmeanBLSB);
    return (double)val * 0.00032;
}

double atm90e32_get_apparent_power_c(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, SmeanC, SmeanCLSB);
    return (double)val * 0.00032;
}

double atm90e32_get_total_apparent_power(atm90e32_config_t *config) {
    int32_t val = atm90e32_read32_register(config, SmeanT, SAmeanTLSB);
    return (double)val * 0.00032;
}

// Frequency
double atm90e32_get_frequency(atm90e32_config_t *config) {
    uint16_t freq = atm90e32_comm_energy_ic(config, READ, Freq, 0xFFFF);
    return (double)freq / 100.0;
}

// Power Factor
double atm90e32_get_power_factor_a(atm90e32_config_t *config) {
    int16_t pf = (int16_t)atm90e32_comm_energy_ic(config, READ, PFmeanA, 0xFFFF);
    return (double)pf / 1000.0;
}

double atm90e32_get_power_factor_b(atm90e32_config_t *config) {
    int16_t pf = (int16_t)atm90e32_comm_energy_ic(config, READ, PFmeanB, 0xFFFF);
    return (double)pf / 1000.0;
}

double atm90e32_get_power_factor_c(atm90e32_config_t *config) {
    int16_t pf = (int16_t)atm90e32_comm_energy_ic(config, READ, PFmeanC, 0xFFFF);
    return (double)pf / 1000.0;
}

double atm90e32_get_total_power_factor(atm90e32_config_t *config) {
    int16_t pf = (int16_t)atm90e32_comm_energy_ic(config, READ, PFmeanT, 0xFFFF);
    return (double)pf / 1000.0;
}

// Phase Angle
double atm90e32_get_phase_a(atm90e32_config_t *config) {
    uint16_t angle = atm90e32_comm_energy_ic(config, READ, PAngleA, 0xFFFF);
    return (double)angle / 10.0;
}

double atm90e32_get_phase_b(atm90e32_config_t *config) {
    uint16_t angle = atm90e32_comm_energy_ic(config, READ, PAngleB, 0xFFFF);
    return (double)angle / 10.0;
}

double atm90e32_get_phase_c(atm90e32_config_t *config) {
    uint16_t angle = atm90e32_comm_energy_ic(config, READ, PAngleC, 0xFFFF);
    return (double)angle / 10.0;
}

// Temperature
double atm90e32_get_temperature(atm90e32_config_t *config) {
    int16_t temp = (int16_t)atm90e32_comm_energy_ic(config, READ, Temp, 0xFFFF);
    return (double)temp;
}

// Gain Parameters
double atm90e32_get_value_register(atm90e32_config_t *config, uint16_t register_read) {
    return (double)atm90e32_comm_energy_ic(config, READ, register_read, 0xFFFF);
}

uint32_t atm90e32_get_voltage_gain(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, UgainA, 0xFFFF);
}

uint32_t atm90e32_get_voltage_rms_gain(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, UrmsA, 0xFFFF);
}

uint32_t atm90e32_get_voltage_rms_lsb(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, UrmsALSB, 0xFFFF);
}

uint32_t atm90e32_get_current_gain(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, IgainA, 0xFFFF);
}

uint32_t atm90e32_get_current_rms_gain(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, IrmsA, 0xFFFF);
}

uint32_t atm90e32_get_current_rms_lsb(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, IrmsALSB, 0xFFFF);
}

void atm90e32_disable_config(atm90e32_config_t *config) {
    atm90e32_comm_energy_ic(config, WRITE, CfgRegAccEn, 0x55AA);
}

// Energy Consumption
double atm90e32_get_import_energy(atm90e32_config_t *config) {
    uint16_t energy = atm90e32_comm_energy_ic(config, READ, APenergyT, 0xFFFF);
    return (double)energy / 100.0 / 3200.0; // Returns kWh
}

double atm90e32_get_import_reactive_energy(atm90e32_config_t *config) {
    uint16_t energy = atm90e32_comm_energy_ic(config, READ, RPenergyT, 0xFFFF);
    return (double)energy / 100.0 / 3200.0; // Returns kWh
}

double atm90e32_get_import_apparent_energy(atm90e32_config_t *config) {
    uint16_t energy = atm90e32_comm_energy_ic(config, READ, SAenergyT, 0xFFFF);
    return (double)energy / 100.0 / 3200.0; // Returns kWh
}

double atm90e32_get_export_energy(atm90e32_config_t *config) {
    uint16_t energy = atm90e32_comm_energy_ic(config, READ, ANenergyT, 0xFFFF);
    return (double)energy / 100.0 / 3200.0; // Returns kWh
}

double atm90e32_get_export_reactive_energy(atm90e32_config_t *config) {
    uint16_t energy = atm90e32_comm_energy_ic(config, READ, RNenergyT, 0xFFFF);
    return (double)energy / 100.0 / 3200.0; // Returns kWh
}

// System Status
uint16_t atm90e32_get_sys_status0(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, EMMIntState0, 0xFFFF);
}

uint16_t atm90e32_get_sys_status1(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, EMMIntState1, 0xFFFF);
}

uint16_t atm90e32_get_meter_status0(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, EMMState0, 0xFFFF);
}

uint16_t atm90e32_get_meter_status1(atm90e32_config_t *config) {
    return atm90e32_comm_energy_ic(config, READ, EMMState1, 0xFFFF);
}



