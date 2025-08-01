
#ifndef ATM90E32_H
#define ATM90E32_H

#include <stdint.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

// SPI Read/Write flags
#define WRITE 0
#define READ 1

// Status Registers
#define MeterEn 0x00        // Metering Enable
#define ChannelMapI 0x01    // Current Channel Mapping Configuration
#define ChannelMapU 0x02    // Voltage Channel Mapping Configuration
#define SagPeakDetCfg 0x05  // Sag and Peak Detector Period Configuration
#define OVth 0x06           // Over Voltage Threshold
#define ZXConfig 0x07       // Zero-Crossing Config
#define SagTh 0x08          // Voltage Sag Threshold
#define PhaseLossTh 0x09    // Voltage Phase Losing Threshold
#define INWarnTh 0x0A       // Neutral Current (Calculated) Warning Threshold
#define OIth 0x0B           // Over Current Threshold
#define FreqLoTh 0x0C       // Low Threshold for Frequency Detection
#define FreqHiTh 0x0D       // High Threshold for Frequency Detection
#define PMPwrCtrl 0x0E      // Partial Measurement Mode Power Control
#define IRQ0MergeCfg 0x0F   // IRQ0 Merge Configuration

// EMM Status Registers
#define SoftReset 0x70      // Software Reset
#define EMMState0 0x71      // EMM State 0
#define EMMState1 0x72      // EMM State 1
#define EMMIntState0 0x73   // EMM Interrupt Status 0
#define EMMIntState1 0x74   // EMM Interrupt Status 1
#define EMMIntEn0 0x75      // EMM Interrupt Enable 0
#define EMMIntEn1 0x76      // EMM Interrupt Enable 1
#define LastSPIData 0x78    // Last Read/Write SPI Value
#define CRCErrStatus 0x79   // CRC Error Status
#define CRCDigest 0x7A      // CRC Digest
#define CfgRegAccEn 0x7F    // Configure Register Access Enable

// Low Power Mode Registers (Not Used)
#define DetectCtrl 0x10
#define DetectTh1 0x11
#define DetectTh2 0x12
#define DetectTh3 0x13
#define PMOffsetA 0x14
#define PMOffsetB 0x15
#define PMOffsetC 0x16
#define PMPGA 0x17
#define PMIrmsA 0x18
#define PMIrmsB 0x19
#define PMIrmsC 0x1A
#define PMConfig 0x10B
#define PMAvgSamples 0x1C
#define PMIrmsLSB 0x1D

// Configuration Registers
#define PLconstH 0x31       // High Word of PL_Constant
#define PLconstL 0x32       // Low Word of PL_Constant
#define MMode0 0x33         // Metering Mode Config
#define MMode1 0x34         // PGA Gain Configuration for Current Channels
#define PStartTh 0x35       // Startup Power Threshold (P)
#define QStartTh 0x36       // Startup Power Threshold (Q)
#define SStartTh 0x37       // Startup Power Threshold (S)
#define PPhaseTh 0x38       // Startup Power Accum Threshold (P)
#define QPhaseTh 0x39       // Startup Power Accum Threshold (Q)
#define SPhaseTh 0x3A       // Startup Power Accum Threshold (S)

// Calibration Registers
#define PoffsetA 0x41       // A Line Power Offset (P)
#define QoffsetA 0x42       // A Line Power Offset (Q)
#define PoffsetB 0x43       // B Line Power Offset (P)
#define QoffsetB 0x44       // B Line Power Offset (Q)
#define PoffsetC 0x45       // C Line Power Offset (P)
#define QoffsetC 0x46       // C Line Power Offset (Q)
#define PQGainA 0x47        // A Line Calibration Gain
#define PhiA 0x48           // A Line Calibration Angle
#define PQGainB 0x49        // B Line Calibration Gain
#define PhiB 0x4A           // B Line Calibration Angle
#define PQGainC 0x4B        // C Line Calibration Gain
#define PhiC 0x4C           // C Line Calibration Angle

// Fundamental/Harmonic Energy Calibration Registers
#define POffsetAF 0x51      // A Fundamental Power Offset (P)
#define POffsetBF 0x52      // B Fundamental Power Offset (P)
#define POffsetCF 0x53      // C Fundamental Power Offset (P)
#define PGainAF 0x54        // A Fundamental Power Gain (P)
#define PGainBF 0x55        // B Fundamental Power Gain (P)
#define PGainCF 0x56        // C Fundamental Power Gain (P)

// Measurement Calibration Registers
#define UgainA 0x61         // A Voltage RMS Gain
#define IgainA 0x62         // A Current RMS Gain
#define UoffsetA 0x63       // A Voltage Offset
#define IoffsetA 0x64       // A Current Offset
#define UgainB 0x65         // B Voltage RMS Gain
#define IgainB 0x66         // B Current RMS Gain
#define UoffsetB 0x67       // B Voltage Offset
#define IoffsetB 0x68       // B Current Offset
#define UgainC 0x69         // C Voltage RMS Gain
#define IgainC 0x6A         // C Current RMS Gain
#define UoffsetC 0x6B       // C Voltage Offset
#define IoffsetC 0x6C       // C Current Offset
#define IoffsetN 0x6E       // N Current Offset

// Energy Registers
#define APenergyT 0x80      // Total Forward Active
#define APenergyA 0x81      // A Forward Active
#define APenergyB 0x82      // B Forward Active
#define APenergyC 0x83      // C Forward Active
#define ANenergyT 0x84      // Total Reverse Active
#define ANenergyA 0x85      // A Reverse Active
#define ANenergyB 0x86      // B Reverse Active
#define ANenergyC 0x87      // C Reverse Active
#define RPenergyT 0x88      // Total Forward Reactive
#define RPenergyA 0x89      // A Forward Reactive
#define RPenergyB 0x8A      // B Forward Reactive
#define RPenergyC 0x8B      // C Forward Reactive
#define RNenergyT 0x8C      // Total Reverse Reactive
#define RNenergyA 0x8D      // A Reverse Reactive
#define RNenergyB 0x8E      // B Reverse Reactive
#define RNenergyC 0x8F      // C Reverse Reactive
#define SAenergyT 0x90      // Total Apparent Energy
#define SenergyA 0x91       // A Apparent Energy
#define SenergyB 0x92       // B Apparent Energy
#define SenergyC 0x93       // C Apparent Energy

// Fundamental/Harmonic Energy Registers
#define APenergyTF 0xA0     // Total Forward Fundamental Energy
#define APenergyAF 0xA1     // A Forward Fundamental Energy
#define APenergyBF 0xA2     // B Forward Fundamental Energy
#define APenergyCF 0xA3     // C Forward Fundamental Energy
#define ANenergyTF 0xA4     // Total Reverse Fundamental Energy
#define ANenergyAF 0xA5     // A Reverse Fundamental Energy
#define ANenergyBF 0xA6     // B Reverse Fundamental Energy
#define ANenergyCF 0xA7     // C Reverse Fundamental Energy
#define APenergyTH 0xA8     // Total Forward Harmonic Energy
#define APenergyAH 0xA9     // A Forward Harmonic Energy
#define APenergyBH 0xAA     // B Forward Harmonic Energy
#define APenergyCH 0xAB     // C Forward Harmonic Energy
#define ANenergyTH 0xAC     // Total Reverse Harmonic Energy
#define ANenergyAH 0xAD     // A Reverse Harmonic Energy
#define ANenergyBH 0xAE     // B Reverse Harmonic Energy
#define ANenergyCH 0xAF     // C Reverse Harmonic Energy

// Power & Power Factor Registers
#define PmeanT 0xB0         // Total Mean Power (P)
#define PmeanA 0xB1         // A Mean Power (P)
#define PmeanB 0xB2         // B Mean Power (P)
#define PmeanC 0xB3         // C Mean Power (P)
#define QmeanT 0xB4         // Total Mean Power (Q)
#define QmeanA 0xB5         // A Mean Power (Q)
#define QmeanB 0xB6         // B Mean Power (Q)
#define QmeanC 0xB7         // C Mean Power (Q)
#define SmeanT 0xB8         // Total Mean Power (S)
#define SmeanA 0xB9         // A Mean Power (S)
#define SmeanB 0xBA         // B Mean Power (S)
#define SmeanC 0xBB         // C Mean Power (S)
#define PFmeanT 0xBC        // Mean Power Factor
#define PFmeanA 0xBD        // A Power Factor
#define PFmeanB 0xBE        // B Power Factor
#define PFmeanC 0xBF        // C Power Factor

#define PmeanTLSB 0xC0      // Lower Word (Total Active Power)
#define PmeanALSB 0xC1      // Lower Word (A Active Power)
#define PmeanBLSB 0xC2      // Lower Word (B Active Power)
#define PmeanCLSB 0xC3      // Lower Word (C Active Power)
#define QmeanTLSB 0xC4      // Lower Word (Total Reactive Power)
#define QmeanALSB 0xC5      // Lower Word (A Reactive Power)
#define QmeanBLSB 0xC6      // Lower Word (B Reactive Power)
#define QmeanCLSB 0xC7      // Lower Word (C Reactive Power)
#define SAmeanTLSB 0xC8     // Lower Word (Total Apparent Power)
#define SmeanALSB 0xC9      // Lower Word (A Apparent Power)
#define SmeanBLSB 0xCA      // Lower Word (B Apparent Power)
#define SmeanCLSB 0xCB      // Lower Word (C Apparent Power)

// Fundamental/Harmonic Power & V/I RMS Registers
#define PmeanTF 0xD0        // Total Active Fundamental Power
#define PmeanAF 0xD1        // A Active Fundamental Power
#define PmeanBF 0xD2        // B Active Fundamental Power
#define PmeanCF 0xD3        // C Active Fundamental Power
#define PmeanTH 0xD4        // Total Active Harmonic Power
#define PmeanAH 0xD5        // A Active Harmonic Power
#define PmeanBH 0xD6        // B Active Harmonic Power
#define PmeanCH 0xD7        // C Active Harmonic Power
#define UrmsA 0xD9          // A RMS Voltage
#define UrmsB 0xDA          // B RMS Voltage
#define UrmsC 0xDB          // C RMS Voltage
#define IrmsN 0xDC          // Calculated N RMS Current
#define IrmsA 0xDD          // A RMS Current
#define IrmsB 0xDE          // B RMS Current
#define IrmsC 0xDF          // C RMS Current

#define PmeanTFLSB 0xE0     // Lower Word (Total Active Fundamental Power)
#define PmeanAFLSB 0xE1     // Lower Word (A Active Fundamental Power)
#define PmeanBFLSB 0xE2     // Lower Word (B Active Fundamental Power)
#define PmeanCFLSB 0xE3     // Lower Word (C Active Fundamental Power)
#define PmeanTHLSB 0xE4     // Lower Word (Total Active Harmonic Power)
#define PmeanAHLSB 0xE5     // Lower Word (A Active Harmonic Power)
#define PmeanBHLSB 0xE6     // Lower Word (B Active Harmonic Power)
#define PmeanCHLSB 0xE7     // Lower Word (C Active Harmonic Power)
#define UrmsALSB 0xE9       // Lower Word (A RMS Voltage)
#define UrmsBLSB 0xEA       // Lower Word (B RMS Voltage)
#define UrmsCLSB 0xEB       // Lower Word (C RMS Voltage)
#define IrmsALSB 0xED       // Lower Word (A RMS Current)
#define IrmsBLSB 0xEE       // Lower Word (B RMS Current)
#define IrmsCLSB 0xEF       // Lower Word (C RMS Current)

// Peak, Frequency, Angle & Temperature Registers
#define UPeakA 0xF1         // A Voltage Peak
#define UPeakB 0xF2         // B Voltage Peak
#define UPeakC 0xF3         // C Voltage Peak
#define IPeakA 0xF5         // A Current Peak
#define IPeakB 0xF6         // B Current Peak
#define IPeakC 0xF7         // C Current Peak
#define Freq 0xF8           // Frequency
#define PAngleA 0xF9        // A Mean Phase Angle
#define PAngleB 0xFA        // B Mean Phase Angle
#define PAngleC 0xFB        // C Mean Phase Angle
#define Temp 0xFC           // Measured Temperature
#define UangleA 0xFD        // A Voltage Phase Angle
#define UangleB 0xFE        // B Voltage Phase Angle
#define UangleC 0xFF        // C Voltage Phase Angle

// ATM90E32 configuration structure
typedef struct {
    gpio_num_t cs_pin;          // Chip Select pin
    gpio_num_t reset_pin;       // Reset pin
    uint16_t line_freq;         // Line frequency (e.g., 389, 4485)
    uint16_t pga_gain;          // PGA gain (e.g., 42)
    uint16_t u_gain;            // Voltage RMS gain
    uint16_t i_gain_a;          // Current gain for channel A
    uint16_t i_gain_b;          // Current gain for channel B
    uint16_t i_gain_c;          // Current gain for channel C
    spi_device_handle_t spi;    // SPI device handle
} atm90e32_config_t;

// Function prototypes
void atm90e32_init(atm90e32_config_t *config);
void atm90e32_reset(atm90e32_config_t *config);
uint16_t atm90e32_comm_energy_ic(atm90e32_config_t *config, uint8_t rw, uint16_t address, uint16_t val);
int32_t atm90e32_read32_register(atm90e32_config_t *config, int16_t regh_addr, int16_t regl_addr);
double atm90e32_calculate_vi_offset(atm90e32_config_t *config, uint16_t regh_addr, uint16_t regl_addr);
double atm90e32_calculate_power_offset(atm90e32_config_t *config, uint16_t regh_addr, uint16_t regl_addr);
double atm90e32_calibrate_vi(atm90e32_config_t *config, uint16_t reg, uint16_t actual_val);

// Main Electrical Parameters
double atm90e32_get_line_voltage_a(atm90e32_config_t *config);
double atm90e32_get_line_voltage_b(atm90e32_config_t *config);
double atm90e32_get_line_voltage_c(atm90e32_config_t *config);
double atm90e32_get_line_current_a(atm90e32_config_t *config);
double atm90e32_get_line_current_b(atm90e32_config_t *config);
double atm90e32_get_line_current_c(atm90e32_config_t *config);
double atm90e32_get_line_current_n(atm90e32_config_t *config);
double atm90e32_get_active_power_a(atm90e32_config_t *config);
double atm90e32_get_active_power_b(atm90e32_config_t *config);
double atm90e32_get_active_power_c(atm90e32_config_t *config);
double atm90e32_get_total_active_power(atm90e32_config_t *config);
double atm90e32_get_total_active_fund_power(atm90e32_config_t *config);
double atm90e32_get_total_active_har_power(atm90e32_config_t *config);
double atm90e32_get_reactive_power_a(atm90e32_config_t *config);
double atm90e32_get_reactive_power_b(atm90e32_config_t *config);
double atm90e32_get_reactive_power_c(atm90e32_config_t *config);
double atm90e32_get_total_reactive_power(atm90e32_config_t *config);
double atm90e32_get_apparent_power_a(atm90e32_config_t *config);
double atm90e32_get_apparent_power_b(atm90e32_config_t *config);
double atm90e32_get_apparent_power_c(atm90e32_config_t *config);
double atm90e32_get_total_apparent_power(atm90e32_config_t *config);
double atm90e32_get_frequency(atm90e32_config_t *config);
double atm90e32_get_power_factor_a(atm90e32_config_t *config);
double atm90e32_get_power_factor_b(atm90e32_config_t *config);
double atm90e32_get_power_factor_c(atm90e32_config_t *config);
double atm90e32_get_total_power_factor(atm90e32_config_t *config);
double atm90e32_get_phase_a(atm90e32_config_t *config);
double atm90e32_get_phase_b(atm90e32_config_t *config);
double atm90e32_get_phase_c(atm90e32_config_t *config);
double atm90e32_get_temperature(atm90e32_config_t *config);

// Gain Parameters
double atm90e32_get_value_register(atm90e32_config_t *config, uint16_t register_read);
uint32_t atm90e32_get_voltage_gain(atm90e32_config_t *config);
uint32_t atm90e32_get_voltage_rms_gain(atm90e32_config_t *config);
uint32_t atm90e32_get_voltage_rms_lsb(atm90e32_config_t *config);
uint32_t atm90e32_get_current_gain(atm90e32_config_t *config);
uint32_t atm90e32_get_current_rms_gain(atm90e32_config_t *config);
uint32_t atm90e32_get_current_rms_lsb(atm90e32_config_t *config);

// Energy Consumption
double atm90e32_get_import_energy(atm90e32_config_t *config);
double atm90e32_get_import_reactive_energy(atm90e32_config_t *config);
double atm90e32_get_import_apparent_energy(atm90e32_config_t *config);
double atm90e32_get_export_energy(atm90e32_config_t *config);
double atm90e32_get_export_reactive_energy(atm90e32_config_t *config);

// System Status
uint16_t atm90e32_get_sys_status0(atm90e32_config_t *config);
uint16_t atm90e32_get_sys_status1(atm90e32_config_t *config);
uint16_t atm90e32_get_meter_status0(atm90e32_config_t *config);
uint16_t atm90e32_get_meter_status1(atm90e32_config_t *config);

void atm90e32_disable_config(atm90e32_config_t *config);

#endif // ATM90E32_H
