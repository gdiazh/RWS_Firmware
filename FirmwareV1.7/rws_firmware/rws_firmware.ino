/*
* @file rw_firmware.ino
* @author Gustavo Diaz H.
* @date 01 Aug 2020
* @brief Reaction wheel's Firmware using three DRV10987 BLDC drivers
*/

#include "rws_firmware.h"
#include "WDTZero.h"

RWFirmware rws(&sercom1, &sercom2, SDA_PA16_SERCOM10, SCL_PA17_SERCOM11, SDA_PA08_SERCOM20, SCL_PA09_SERCOM21);

WDTZero MyWatchDoggy; // Define WDT  

uint8_t speed_status[2] = {0, 0};
uint8_t current_status[2] = {0, 0};
uint8_t motorkt_status[2] = {0, 0};
uint16_t speed_status_value = 0;

void setup()
{
    // Fault handling
    MyWatchDoggy.attachShutdown(myshutdown);
    MyWatchDoggy.setup(WDT_SOFTCYCLE32S);  // initialize WDT-softcounter refesh cycle on 32sec interval
    // RWS setup
    rws.begin();
    Wire.onReceive(readMasterWrite);
    Wire.onRequest(responseToMasterRead);
    rws.setDefault(100, 100, 100);
}

void loop()
{ 
    // rws.checkFaults();
    // rws.blue1_on();
    // delay(500);

    // rws.runUartControllerCommand();
    rws.runI2cControllerCommand();
    // rws.readStatesDRVs();
    // rws.display_motorsdata_uart();
    // rws.sendDataUart();

    // rws.blue1_off();
    MyWatchDoggy.clear();  // refresh wdt - before it loops
    // delay(500);
    // SerialPort.println("running");
}

void readMasterWrite(int HowMany)
{
    if (Wire.available() && rws.reading_drvs == 0)
    {
        memset(rws.i2c_rx_frame, 0, I2C_CMD_SZ);
        rws.i2c_rx_frame[0] = Wire.read();
        rws.i2c_rx_frame[1] = Wire.read();
        rws.i2c_rx_frame[2] = Wire.read();
        rws.i2c_rx_frame[3] = Wire.read();
        memcpy(&rws.i2c_cmd, rws.i2c_rx_frame, sizeof(rws.i2c_cmd));
        rws.i2c_cmd_result = 1;
        // SerialPort.println(rws.i2c_cmd.code);
        // SerialPort.println(rws.i2c_cmd.dir);
        // SerialPort.println(rws.i2c_cmd.speed);
        // Wire.end();
    }
}

void responseToMasterRead()
{
    if (rws.i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR1)
    {
        rws.write2b(rws.motor1_data.raw_speed[0], rws.motor1_data.raw_speed[1]);
    }
    else if (rws.i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR1)
    {
        rws.write2b(rws.motor1_data.raw_current[0], rws.motor1_data.raw_current[1]);
    }
    else if (rws.i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR2)
    {
        rws.write2b(rws.motor2_data.raw_speed[0], rws.motor2_data.raw_speed[1]);
    }
    else if (rws.i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR2)
    {
        rws.write2b(rws.motor2_data.raw_current[0], rws.motor2_data.raw_current[1]);
    }
    else if (rws.i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR3)
    {
        rws.write2b(rws.motor3_data.raw_speed[0], rws.motor3_data.raw_speed[1]);
    }
    else if (rws.i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR3)
    {
        rws.write2b(rws.motor3_data.raw_current[0], rws.motor3_data.raw_current[1]);
    }
}

void myshutdown()
{

// SerialPort.println("\nWe gonna shut down ! ...");
rws.reading_drvs = 0;
rws.tcaReset();
// rws.tcaDeSelect();
// rws.i2c_multiplexer.setClock(100000);
PORT->Group[PORTA].OUTCLR.reg = RWS_EN_PIN;
delay(40);
PORT->Group[PORTA].OUTCLR.reg = RWS_EN_PIN;
delay(40);
}