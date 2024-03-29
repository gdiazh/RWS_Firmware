/*
* @file rw_firmware.h
* @author Gustavo Diaz H.
* @date 01 Aug 2020
* @brief Reaction wheel's Firmware using three DRV10987 BLDC drivers
*/

#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function
#include <Arduino.h>
// Device i2c address
#define SAMD21GD_ADDR 0x30
#define DRV10987_ADDR 0x52
#define DRV10987_DEVID_ADDR 0X08
#define DRV10987_CONFIG1_ADDR 0X90
#define DRV10987_CONFIG2_ADDR 0X91
#define DRV10987_CONFIG3_ADDR 0X92
#define DRV10987_CONFIG4_ADDR 0X93
#define DRV10987_CONFIG5_ADDR 0X94
#define DRV10987_CONFIG6_ADDR 0X95
#define DRV10987_CONFIG7_ADDR 0X96
#define DRV10987_EECTRL_ADDR 0x60
#define DRV10987_EEPROM_PROG1_ADDR 0x31
#define DRV10987_EEPROM_PROG2_ADDR 0x32
#define DRV10987_EEPROM_PROG5_ADDR 0x35
#define DRV10987_MOTORSPEED_STATUS_ADDR 0x01
#define DRV10987_MOTORCURRENT_STATUS_ADDR 0x04
#define DRV10987_MOTORSPEED_ADDR 0x30
// I2C Multiplexer
#define MULTIPLEXER_ADDR 0X70
#define MOTOR1_I2C_PORT 2
#define MOTOR2_I2C_PORT 1
#define MOTOR3_I2C_PORT 0
// Firmware commands
#define SAMPLE_SPEED_CODE_MOTOR1 21
#define SAMPLE_CURRENT_CODE_MOTOR1 22
#define SET_SPEED_CODE_MOTOR1 23
#define SAMPLE_SPEED_CODE_MOTOR2 24
#define SAMPLE_CURRENT_CODE_MOTOR2 25
#define SET_SPEED_CODE_MOTOR2 26
#define SAMPLE_SPEED_CODE_MOTOR3 27
#define SAMPLE_CURRENT_CODE_MOTOR3 28
#define SET_SPEED_CODE_MOTOR3 29
#define MOTOR1_ID 1
#define MOTOR2_ID 2
#define MOTOR3_ID 3
// Motor Parameters
#define CONSTANT_RM 0x3C //RPH_CT = 1.8624[Ohm], the closest to 1.9[Ohm] measured
#define CONSTANT_KT 0x19 //Kt = 16.56[mV/Hz], the closest to 16.2504 [mV/Hz] measured
#define FAULTS_CHECK_TIME 10000 //[ms]
#define SPEED_FAULT 10000 //[rpm]
// Pin Assignments
#define RWS_EN_PIN PORT_PA07
#define MULTIPLEXER_RST_PA04 PORT_PA04
#define SDA_PA22_SERCOM30 20 //default I2C SDA
#define SCL_PA23_SERCOM31 21 //default I2C SCL
#define SDA_PA16_SERCOM10 11
#define SCL_PA17_SERCOM11 13
#define SDA_PA08_SERCOM20 4
#define SCL_PA09_SERCOM21 3
#define MOTOR1_DIR_PB23 PORT_PB23
#define MOTOR2_DIR_PB22 PORT_PB22
#define MOTOR3_DIR_PA10 PORT_PA10
#define SPEEDFAULT PORT_PA31
#define LED_BLUE1 PORT_PA15
#define LED_BLUE2 PORT_PA11
#define LED_GREEN PORT_PB03
#define LED_ORANGE PORT_PB02
#ifndef SerialPort
    #define SerialPort SerialUSB
#endif
// Data structures
#define MOTOR_DT_SZ 8
typedef struct{
    float speed;
    float current;
    uint8_t raw_speed[2];
    uint8_t raw_current[2];
}motor_data_t;

#define I2C_CMD_SZ 4
typedef struct{
    uint8_t code;
    uint8_t dir;
    uint16_t speed;
}i2c_command_t;

#define UART_CMD_SZ 6
typedef struct{
    uint16_t code;
    uint16_t speed;
    uint8_t tail;
    uint8_t checksum;
}uart_command_t;

#define UART_PCKT_SZ 22
typedef struct{
    uint16_t raw_current_m1;
    uint16_t raw_speed_m1;
    uint16_t cmd_speed_m1;
    uint16_t raw_current_m2;
    uint16_t raw_speed_m2;
    uint16_t cmd_speed_m2;
    uint16_t raw_current_m3;
    uint16_t raw_speed_m3;
    uint16_t cmd_speed_m3;
    uint8_t tail1;
    uint8_t tail2;
    uint8_t tail3;
    uint8_t checksum;
}uart_packet_t;

class RWFirmware
{
    // Private Members
    // DRV10987 Configuration registers states
    uint8_t eeReadyStatus_[2] = {0, 0};
    uint8_t config1Status_[2] = {0, 0};
    uint8_t config2Status_[2] = {0, 0};
    uint8_t config3Status_[2] = {0, 0};
    uint8_t config4Status_[2] = {0, 0};
    uint8_t config5Status_[2] = {0, 0};
    uint8_t config6Status_[2] = {0, 0};
    uint8_t config7Status_[2] = {0, 0};
    uint8_t devId_[2] = {0, 0};
    // motors data to send through uart interface
    uart_packet_t uart_packet_;
    // I2C (TWI) Interfaces
    // TwoWire i2c_multiplexer;
    TwoWire i2c_csp;
    // faults
    unsigned long last_faults_check_time_;
public:
    TwoWire i2c_multiplexer;
    // Public Members
    // Command handlers
    i2c_command_t i2c_cmd;
    uart_command_t uart_cmd;
    uint8_t i2c_cmd_result = 0;
    uint8_t i2c_rx_frame[I2C_CMD_SZ];
    uint8_t reading_drvs = 0;
    // Motor State handlers
    motor_data_t motor1_data;
    motor_data_t motor2_data;
    motor_data_t motor3_data;
    // Constructor
    RWFirmware(SERCOM* sercom1, SERCOM* sercom2, uint8_t sda_m2, uint8_t scl_m2, uint8_t sda_m3, uint8_t scl_m3):
    i2c_multiplexer(sercom1, sda_m2, scl_m2),
    i2c_csp(sercom2, sda_m3, scl_m3)
    {}
    // methods
    void begin(void);
    void setDefault(uint16_t m1_spd, uint16_t m2_spd, uint16_t m3_spd);
    uint8_t readCommandUart(void);
    void sendDataUart(void);
    void runI2cControllerCommand(void);
    void runUartControllerCommand(void);
    void readStatesDRV(uint8_t motor_id);
    void readStatesDRVs(void);
    void setMotorSpeed(uint8_t motor_id, float speed);
    void drvRead(uint8_t motor_id, uint8_t reg_addr, uint8_t *data);
    void drvWrite(uint8_t motor_id, uint8_t reg_addr, uint8_t data1, uint8_t data2);
    void write2b(uint8_t data1, uint8_t data2);
    uint8_t checksum(uint8_t *packet, uint8_t n);
    void display_motorsdata_uart();
    void set_dir_m1(uint8_t dir);
    void set_dir_m2(uint8_t dir);
    void set_dir_m3(uint8_t dir);
    void checkFaults();
    void raiseSpeedFault();
    void cleareSpeedFault();
    void tcaselect(uint8_t i);
    void tcaDeSelect();
    void tcaReset();
    void blue1_blink(uint8_t dly_ms);
    void green_blink(uint8_t dly_ms);
    void blue2_blink(uint8_t dly_ms);
    void orange_blink(uint8_t dly_ms);
    void blue1_on();
    void green_on();
    void blue2_on();
    void orange_on();
    void blue1_off();
    void green_off();
    void blue2_off();
    void orange_off();
};