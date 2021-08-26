/*
* @file rw_firmware.py
* @author Gustavo Diaz H.
* @date 01 Aug 2020
* @brief Reaction wheel's Firmware using three DRV10987 BLDC drivers
*/

#include "rws_firmware.h"

void RWFirmware::begin(void)
{
    // I2C Interfaces
    Wire.begin(SAMD21GD_ADDR);
    motor2_.begin(SAMD21GD_ADDR);
    motor3_.begin(SAMD21GD_ADDR);
    // Assign pins to SERCOM functionality
    pinPeripheral(SDA_PA16_SERCOM10, PIO_SERCOM);
    pinPeripheral(SCL_PA17_SERCOM11, PIO_SERCOM);
    pinPeripheral(SDA_PA08_SERCOM20, PIO_SERCOM_ALT);
    pinPeripheral(SCL_PA09_SERCOM21, PIO_SERCOM_ALT);
    // Wire.setClock(100000);
    motor2_.setClock(100000);
    motor3_.setClock(100000);
    // Faults
    PORT->Group[PORTA].DIRSET.reg = SPEEDFAULT;
    PORT->Group[PORTA].OUTCLR.reg = SPEEDFAULT;
    // Debug 
    pinMode(DEBUG_LED_1, OUTPUT);
    digitalWrite(DEBUG_LED_1, LOW);
    SerialPort.begin(115200);
    SerialPort.println("DRV10987 Firmware Interface");
}

void RWFirmware::setDefault(uint16_t m1_spd, uint16_t m2_spd, uint16_t m3_spd){
    /* Set default values of variables, and motor speed
    *
    * @param m1_spd uint16_t dafault speed motor 1
    * @param m2_spd uint16_t dafault speed motor 2
    * @param m3_spd uint16_t dafault speed motor 3
    */
    // I2C Interface Commands
    i2c_cmd.code = 0;
    i2c_cmd.speed = 0;
    // Motor Data States
    motor1_data.speed = 0;
    motor1_data.current = 0;
    motor1_data.raw_speed[0] = 0;
    motor1_data.raw_speed[1] = 0;
    motor1_data.raw_current[0] = 0;
    motor1_data.raw_current[1] = 0;
    motor2_data.speed = 0;
    motor2_data.current = 0;
    motor2_data.raw_speed[0] = 0;
    motor2_data.raw_speed[1] = 0;
    motor2_data.raw_current[0] = 0;
    motor2_data.raw_current[1] = 0;
    motor3_data.speed = 0;
    motor3_data.current = 0;
    motor3_data.raw_speed[0] = 0;
    motor3_data.raw_speed[1] = 0;
    motor3_data.raw_current[0] = 0;
    motor3_data.raw_current[1] = 0;
    // Uart packet struct
    uart_packet_.raw_current_m1 = 0;
    uart_packet_.raw_speed_m1 = 0;
    uart_packet_.cmd_speed_m1 = 0;
    uart_packet_.raw_current_m2 = 0;
    uart_packet_.raw_speed_m2 = 0;
    uart_packet_.cmd_speed_m2 = 0;
    uart_packet_.raw_current_m3 = 0;
    uart_packet_.raw_speed_m3 = 0;
    uart_packet_.cmd_speed_m3 = 0;
    uart_packet_.tail1 = 255;
    uart_packet_.tail2 = 254;
    uart_packet_.tail3 = 253;
    uart_packet_.checksum = checksum((uint8_t*)&uart_packet_, sizeof(uart_packet_));
    // Set Initial Motor Speeds
    setMotorSpeed(MOTOR1_ID, m1_spd);
    setMotorSpeed(MOTOR2_ID, m2_spd);
    setMotorSpeed(MOTOR3_ID, m3_spd);
    // Faults
    last_faults_check_time_ = millis();
    // Debug Led State
    digitalWrite(DEBUG_LED_1, LOW);
    delay(100); // wait for the motors to reach initial speed
}

uint8_t RWFirmware::readCommandUart(void)
{
    /* Read data frame containing cmds from uart port
    *
    * @update uart_cmd uart_command_t data structure to save cmds
    * @return sum uint8_t code result, 1 if received ok and 0 i fail
    */
    uint8_t uart_rx_frame[UART_CMD_SZ];
    memset(uart_rx_frame, 0, UART_CMD_SZ);
    uint8_t readed_bytes = 0;
    if (SerialPort.available())
        readed_bytes = SerialPort.readBytes((char*)&uart_rx_frame, UART_CMD_SZ);
    if (readed_bytes == UART_CMD_SZ)
    {
        // fill sttruct
        uint8_t cks = checksum(uart_rx_frame, UART_CMD_SZ);
        if (uart_rx_frame[UART_CMD_SZ-1] == cks && uart_rx_frame[UART_CMD_SZ-2] == 255)
        {
            memcpy(&uart_cmd, uart_rx_frame, sizeof(uart_cmd));
            return 1;
        }
    }
    return 0; //fail rx
}

void RWFirmware::sendDataUart(void)
{
    /* Send motors state through uart port
    */
    uart_packet_.raw_current_m1 = (motor1_data.raw_current[0]<<8) | motor1_data.raw_current[1]; //[MSB LSB]
    uart_packet_.raw_speed_m1 = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1]; //[MSB LSB]
    
    uart_packet_.raw_current_m2 = (motor2_data.raw_current[0]<<8) | motor2_data.raw_current[1]; //[MSB LSB]
    uart_packet_.raw_speed_m2 = (motor2_data.raw_speed[0]<<8) | motor2_data.raw_speed[1]; //[MSB LSB]
    
    uart_packet_.raw_current_m3 = (motor3_data.raw_current[0]<<8) | motor3_data.raw_current[1]; //[MSB LSB]
    uart_packet_.raw_speed_m3 = (motor3_data.raw_speed[0]<<8) | motor3_data.raw_speed[1]; //[MSB LSB]
    
    // uart_packet_.time_arduino = millis();
    uart_packet_.tail1 = 255;
    uart_packet_.tail2 = 254;
    uart_packet_.tail3 = 253;
    uart_packet_.checksum = checksum((uint8_t*)&uart_packet_, sizeof(uart_packet_));
    SerialPort.write((uint8_t*)&uart_packet_, sizeof(uart_packet_));
}

void RWFirmware::runI2cControllerCommand(void)
{
    /* Run Master-Controller commands for specified motor, reading or
    *  writing to the corresponding DRV10987 driver
    *  - get_speed
    *  - get_current
    *  - set_speed
    * @update motor1_data motor_data_t speed(float) and current(float)
    * @update motor2_data motor_data_t speed(float) and current(float)
    * @update motor3_data motor_data_t speed(float) and current(float)
    */
    if (i2c_cmd_result) // check if there is a valid cmd from the Master-Controller
    {
        if (i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR1)
        {
            drvRead(MOTOR1_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor1_data.raw_speed);
            motor1_data.speed = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1]; //[RPM]
        }
        else if (i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR1)
        {
            drvRead(MOTOR1_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor1_data.raw_current);
            uint16_t current_aux = (motor1_data.raw_current[0]&0x07)<<8 | motor1_data.raw_current[1];
            motor1_data.current = 1.46484375*(current_aux-1023);       //[mA]
        }
        else if (i2c_cmd.code == SET_SPEED_CODE_MOTOR1)
        {
            setMotorSpeed(MOTOR1_ID, i2c_cmd.speed);
            uart_packet_.cmd_speed_m1 = i2c_cmd.speed;
        }
        else if (i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR2)
        {
            drvRead(MOTOR2_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor2_data.raw_speed);
            motor2_data.speed = (motor2_data.raw_speed[0]<<8) | motor2_data.raw_speed[1]; //[RPM]
        }
        else if (i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR2)
        {
            drvRead(MOTOR2_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor2_data.raw_current);
            uint16_t current_aux = (motor2_data.raw_current[0]&0x07)<<8 | motor2_data.raw_current[1];
            motor2_data.current = 1.46484375*(current_aux-1023); //[mA]
        }
        else if (i2c_cmd.code == SET_SPEED_CODE_MOTOR2)
        {
            setMotorSpeed(MOTOR2_ID, i2c_cmd.speed);
            uart_packet_.cmd_speed_m2 = i2c_cmd.speed;
        }
        else if (i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR3)
        {
            drvRead(MOTOR3_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor3_data.raw_speed);
            motor3_data.speed = (motor3_data.raw_speed[0]<<8) | motor3_data.raw_speed[1]; //[RPM]
        }
        else if (i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR3)
        {
            drvRead(MOTOR3_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor3_data.raw_current);
            uint16_t current_aux = (motor3_data.raw_current[0]&0x07)<<8 | motor3_data.raw_current[1];
            motor3_data.current = 1.46484375*(current_aux-1023); //[mA]
        }
        else if (i2c_cmd.code == SET_SPEED_CODE_MOTOR3)
        {
            setMotorSpeed(MOTOR3_ID, i2c_cmd.speed);
            uart_packet_.cmd_speed_m3 = i2c_cmd.speed;
        }
        i2c_cmd_result = 0;
    }
}

void RWFirmware::runUartControllerCommand(void)
{
    /* Run Uart-Controller commands for specified motor,
    *  writing to the corresponding DRV10987 driver
    *  - set_speed
    * @update uart_packet_ uart_packet_t data structure to save states
    */
    uint8_t uart_cmd_result = readCommandUart();
    if (uart_cmd_result)
    {
        if (uart_cmd.code == SET_SPEED_CODE_MOTOR1)
        {
            setMotorSpeed(MOTOR1_ID, uart_cmd.speed);
            uart_packet_.cmd_speed_m1 = uart_cmd.speed;
        }
        else if (uart_cmd.code == SET_SPEED_CODE_MOTOR2)
        {
            setMotorSpeed(MOTOR2_ID, uart_cmd.speed);
            uart_packet_.cmd_speed_m2 = uart_cmd.speed;
        }        
        else if (uart_cmd.code == SET_SPEED_CODE_MOTOR3)
        {
            setMotorSpeed(MOTOR3_ID, uart_cmd.speed);
            uart_packet_.cmd_speed_m3 = uart_cmd.speed;
        }
        uart_cmd_result = 0;
    }
}

void RWFirmware::readStatesDRV(uint8_t motor_id)
{
    /* Read all motor states from respective DRV10987 drivers
    * through the respective I2C interface and update data structs
    *
    * @update motor1_data motor_data_t speed(float) and current(float)
    * @update motor2_data motor_data_t speed(float) and current(float)
    * @update motor3_data motor_data_t speed(float) and current(float)
    */
    if (motor_id == MOTOR1_ID)
    {
        // Read motor 1 state
        drvRead(MOTOR1_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor1_data.raw_speed);
        motor1_data.speed = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1];     //[RPM]
        // delay(5);   //TODO: check if/why this is necessary
        drvRead(MOTOR1_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor1_data.raw_current);
        uint16_t current_aux1 = (motor1_data.raw_current[0]&0x07)<<8 | motor1_data.raw_current[1];
        motor1_data.current = 1.46484375*(current_aux1-1023);           //[mA]
    }
    if (motor_id == MOTOR2_ID)
    {
        // Read motor 2 state
        drvRead(MOTOR2_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor2_data.raw_speed);
        motor2_data.speed = (motor2_data.raw_speed[0]<<8) | motor2_data.raw_speed[1];     //[RPM]
        // delay(5);
        drvRead(MOTOR2_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor2_data.raw_current);
        uint16_t current_aux2 = (motor2_data.raw_current[0]&0x07)<<8 | motor2_data.raw_current[1];
        motor2_data.current = 1.46484375*(current_aux2-1023);           //[mA]
    }
    if (motor_id == MOTOR3_ID)
    {
        // Read motor 3 state
        drvRead(MOTOR3_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor3_data.raw_speed);
        motor3_data.speed = (motor3_data.raw_speed[0]<<8) | motor3_data.raw_speed[1];     //[RPM]
        // delay(5);
        drvRead(MOTOR3_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor3_data.raw_current);
        uint16_t current_aux3 = (motor3_data.raw_current[0]&0x07)<<8 | motor3_data.raw_current[1];
        motor3_data.current = 1.46484375*(current_aux3-1023);           //[mA]
    }
}

void RWFirmware::readStatesDRVs(void)
{
    /* Read all motor states from respective DRV10987 drivers
    * through the respective I2C interface and update data structs
    *
    * @update motor1_data motor_data_t speed(float) and current(float)
    * @update motor2_data motor_data_t speed(float) and current(float)
    * @update motor3_data motor_data_t speed(float) and current(float)
    */
    // Read motor 1 state
    drvRead(MOTOR1_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor1_data.raw_speed);
    motor1_data.speed = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1];     //[RPM]
    // delay(5);   //TODO: check if/why this is necessary
    drvRead(MOTOR1_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor1_data.raw_current);
    uint16_t current_aux1 = (motor1_data.raw_current[0]&0x07)<<8 | motor1_data.raw_current[1];
    motor1_data.current = 1.46484375*(current_aux1-1023);           //[mA]
    // Read motor 2 state
    drvRead(MOTOR2_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor2_data.raw_speed);
    motor2_data.speed = (motor2_data.raw_speed[0]<<8) | motor2_data.raw_speed[1];     //[RPM]
    // delay(5);
    drvRead(MOTOR2_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor2_data.raw_current);
    uint16_t current_aux2 = (motor2_data.raw_current[0]&0x07)<<8 | motor2_data.raw_current[1];
    motor2_data.current = 1.46484375*(current_aux2-1023);           //[mA]
    // Read motor 3 state
    drvRead(MOTOR3_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor3_data.raw_speed);
    motor3_data.speed = (motor3_data.raw_speed[0]<<8) | motor3_data.raw_speed[1];     //[RPM]
    // delay(5);
    drvRead(MOTOR3_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor3_data.raw_current);
    uint16_t current_aux3 = (motor3_data.raw_current[0]&0x07)<<8 | motor3_data.raw_current[1];
    motor3_data.current = 1.46484375*(current_aux3-1023);           //[mA]
}

void RWFirmware::setMotorSpeed(uint8_t motor_id, float speed)
{
    /* Write speed value to the corresponding register on DRV10987 driver
    * through the respective I2C interface
    *
    * @param motor_id uint8_t motor id number (valid values under <Firmware commands> include)
    * @param speed float speed value should be in the range [0, 511]
    */
    uint16_t speed_aux = (uint16_t) speed;
    if (speed_aux>511) speed_aux = 511;
    uint8_t speed_H = speed_aux >> 8;       //MSB
    uint8_t speed_L = speed_aux & 0xff;     //LSB
    speed_H = speed_H | 1<<7;               // Enable bit for i2c setting mode

    if (motor_id == MOTOR1_ID)
    {
        drvWrite(MOTOR1_ID, DRV10987_MOTORSPEED_ADDR, speed_H, speed_L);
    }
    else if (motor_id == MOTOR2_ID)
    {
        drvWrite(MOTOR2_ID, DRV10987_MOTORSPEED_ADDR, speed_H, speed_L);
    }
    else if (motor_id == MOTOR3_ID)
    {
        drvWrite(MOTOR3_ID, DRV10987_MOTORSPEED_ADDR, speed_H, speed_L);
    }
}

void RWFirmware::drvRead(uint8_t motor_id, uint8_t reg_addr, uint8_t *data)
{
    /* Read 2 bytes of data from given motor id (and therefore respective drv10987 driver)
    * through respective I2C interface
    *
    * @param motor_id uint8_t motor id number (valid values under <Firmware commands> include)
    * @param reg_addr uint8_t drv10987 register address to read the data (valid values under <device i2c address> include)
    * @param data uint8_t[2] array to save the readed data
    */
    if (motor_id == MOTOR1_ID)
    {   // The motor 1 share the default TWI with an external control device
        Wire.setClock(100000);                          // set shared interface as master temporarily
        Wire.beginTransmission(DRV10987_ADDR);
        uint8_t bytesSent = Wire.write(reg_addr);       // send reg addr to be readed
        uint8_t errcode = Wire.endTransmission(false);  // stop transmitting
        Wire.requestFrom(DRV10987_ADDR, 2);               // request data from given addr
        if (Wire.available())
        {   //TODO: slave may send less than requested, check condition
            data[0] = Wire.read();                      // read MSB of data
            data[1] = Wire.read();                      // read LSB of data
        }
        Wire.begin(SAMD21GD_ADDR);                          // leave shared interface as slave
    }
    else if (motor_id == MOTOR2_ID)
    {   // Use SERCOM1 interface
        motor2_.beginTransmission(DRV10987_ADDR);
        uint8_t bytesSent = motor2_.write(reg_addr);     // send reg addr to be readed
        uint8_t errcode = motor2_.endTransmission(false);// stop transmitting
        motor2_.requestFrom(DRV10987_ADDR, 2);             // request data from given addr
        if (motor2_.available())
        {   //TODO: slave may send less than requested, check condition
            data[0] = motor2_.read();                    // read MSB of data
            data[1] = motor2_.read();                    // read LSB of data
        }
    }
    else if (motor_id == MOTOR3_ID)
    {   // Use SERCOM2 interface
        motor3_.beginTransmission(DRV10987_ADDR);
        uint8_t bytesSent = motor3_.write(reg_addr);     // send reg addr to be readed
        uint8_t errcode = motor3_.endTransmission(false);// stop transmitting
        motor3_.requestFrom(DRV10987_ADDR, 2);             // request data from given addr
        if (motor3_.available())
        {   //TODO: slave may send less than requested, check condition
            data[0] = motor3_.read();                    // read MSB of data
            data[1] = motor3_.read();                    // read LSB of data
        }
    }
}

void RWFirmware::drvWrite(uint8_t motor_id, uint8_t reg_addr, uint8_t data1, uint8_t data2)
{
    /* Write 2 bytes of data to given motor id (and therefore respective drv10987 driver)
    * through respective I2C interface
    *
    * @param motor_id uint8_t motor id number (valid values under <Firmware commands> include)
    * @param reg_addr uint8_t drv10987 register address to write the data (valid values under <device i2c address> include)
    * @param data1 uint8_t data MSB
    * @param data2 uint8_t data LSB
    */
    if (motor_id == MOTOR1_ID)
    {   // The motor 1 share the default TWI with an external control device
        Wire.setClock(100000);                // set shared interface as master temporarily
        Wire.beginTransmission(DRV10987_ADDR);
        Wire.write(reg_addr);                 // send reg addr
        Wire.write(data1);                    // send MSB first
        Wire.write(data2);                    // send LSB second
        Wire.endTransmission(true);           // stop transmitting
        Wire.begin(SAMD21GD_ADDR);                // leave shared interface as slave
    }
    else if (motor_id == MOTOR2_ID)
    {   // Use SERCOM1 interface
        motor2_.beginTransmission(DRV10987_ADDR);
        motor2_.write(reg_addr);                 // send reg addr
        motor2_.write(data1);                    // send MSB first
        motor2_.write(data2);                    // send LSB second
        motor2_.endTransmission(true);           // stop transmitting
    }
    else if (motor_id == MOTOR3_ID)
    {   // Use SERCOM2 interface
        motor3_.beginTransmission(DRV10987_ADDR);
        motor3_.write(reg_addr);                 // send reg addr
        motor3_.write(data1);                    // send MSB first
        motor3_.write(data2);                    // send LSB second
        motor3_.endTransmission(true);           // stop transmitting
    }
}

void RWFirmware::write2b(uint8_t data1, uint8_t data2)
{
    /* Write 2 bytes of data as a response to a read from a Master device
    * through the default I2C interface
    *
    * @param data1 uint8_t data MSB
    * @param data2 uint8_t data LSB
    */
    Wire.write(data1);                    // send MSB first
    Wire.write(data2);                    // send LSB second
}

uint8_t RWFirmware::checksum(uint8_t *packet, uint8_t n)
{
    /* Check transmition errors of data structure
    *
    * @param packet uint8_t[UART_PCKT_SZ] data structure to verify
    * @param n uint8_t number of bytes on <packet>
    * @return sum uint8_t calculated checksum value
    */
    // TODO: implement better data authenticity algorith
    uint32_t sum = 0;
    for (int j=0;j<n-1;j++) sum += packet[j];
    return sum & 0x00FF;
}

void RWFirmware::checkFaults()
{
    if ((millis()-last_faults_check_time_)>FAULTS_CHECK_TIME)
    {
        readStatesDRVs();
        SerialPort.print(motor1_data.speed);
        SerialPort.print("\t\t");
        SerialPort.print(motor2_data.speed);
        SerialPort.print("\t\t");
        SerialPort.print(motor3_data.speed);
        SerialPort.print("\t\t");

        SerialPort.print(motor1_data.current);
        SerialPort.print("\t\t");
        SerialPort.print(motor2_data.current);
        SerialPort.print("\t\t");
        SerialPort.println(motor3_data.current);

        if (motor1_data.speed >SPEED_FAULT || motor2_data.speed >SPEED_FAULT || motor3_data.speed >SPEED_FAULT)
        {
            raiseSpeedFault();
            SerialPort.println("[WARNING] SPEED FAULT");
            SerialPort.println("[WARNING] Waiting for driver reset...");
            delay(3000);
            SerialPort.println("[WARNING] CLEARING SPEED FAULT");
            cleareSpeedFault();
            SerialPort.println("[WARNING] Waiting driver power on...");
            delay(100);
            SerialPort.println("[WARNING] Setting Default Speeds...");
            begin();
            setDefault(0,0,0);
        }
        else
        last_faults_check_time_ = millis();
    }
}

void RWFirmware::raiseSpeedFault()
{
    PORT->Group[PORTA].OUTSET.reg = SPEEDFAULT;
}
void RWFirmware::cleareSpeedFault()
{
    PORT->Group[PORTA].OUTCLR.reg = SPEEDFAULT;
}