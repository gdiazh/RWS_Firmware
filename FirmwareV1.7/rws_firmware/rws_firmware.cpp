/*
* @file rw_firmware.py
* @author Gustavo Diaz H.
* @date 01 Aug 2020
* @brief Reaction wheel's Firmware using three DRV10987 BLDC drivers
*/

#include "rws_firmware.h"

void RWFirmware::begin(void)
{
    // Turn on power switch (BTS5030)
    PORT->Group[PORTA].DIRSET.reg = RWS_EN_PIN;
    // PORT->Group[PORTA].OUTCLR.reg = RWS_EN_PIN;
    // delay(500); //wait for regulator to stabilize
    PORT->Group[PORTA].OUTSET.reg = RWS_EN_PIN;
    delay(500); //wait for regulator to stabilize
    // I2C Interfaces
    PORT->Group[PORTA].DIRSET.reg = MULTIPLEXER_RST_PA04;
    PORT->Group[PORTA].OUTSET.reg = MULTIPLEXER_RST_PA04; //multiplexer reset off
    i2c_multiplexer.begin();        //initialize as master
    Wire.begin(SAMD21GD_ADDR);      //initialize as slave
    // i2c_csp.begin(SAMD21GD_ADDR);   //initialize as slave
    i2c_multiplexer.setTimeout(20);     //ms
    // Wire.setTimeout(20);
    // i2c_csp.setTimeout(20);
    // Assign pins to SERCOM functionality
    pinPeripheral(SDA_PA16_SERCOM10, PIO_SERCOM);       //i2c multiplex interface
    pinPeripheral(SCL_PA17_SERCOM11, PIO_SERCOM);   
    pinPeripheral(SDA_PA08_SERCOM20, PIO_SERCOM_ALT);   //i2c csp interface
    pinPeripheral(SCL_PA09_SERCOM21, PIO_SERCOM_ALT);
    i2c_multiplexer.setClock(100000);
    // Wire.setClock(100000);
    // i2c_csp.setClock(100000);
    // Faults
    PORT->Group[PORTA].DIRSET.reg = SPEEDFAULT;
    PORT->Group[PORTA].OUTCLR.reg = SPEEDFAULT;
    // Direction pins
    PORT->Group[PORTB].DIRSET.reg = MOTOR1_DIR_PB23;    //motor1
    PORT->Group[PORTB].DIRSET.reg = MOTOR2_DIR_PB22;    //motor2
    PORT->Group[PORTA].DIRSET.reg = MOTOR3_DIR_PA10;    //motor3
    set_dir_m1(0);
    set_dir_m2(0);
    set_dir_m3(0);
    // Debug 
    PORT->Group[PORTA].DIRSET.reg = PORT_PA15;    //blue1
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA15;
    PORT->Group[PORTB].DIRSET.reg = PORT_PB03;    //green
    PORT->Group[PORTB].OUTCLR.reg = PORT_PB03;
    PORT->Group[PORTA].DIRSET.reg = PORT_PA11;    //blue2
    PORT->Group[PORTA].OUTCLR.reg = PORT_PA11;
    PORT->Group[PORTB].DIRSET.reg = PORT_PB02;    //orange
    PORT->Group[PORTB].OUTCLR.reg = PORT_PB02;
    // SerialPort.begin(115200);
    // SerialPort.println("DRV10987 Firmware Interface");
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
    delay(100);
    setMotorSpeed(MOTOR2_ID, m2_spd);
    delay(100);
    setMotorSpeed(MOTOR3_ID, m3_spd);
    // Faults
    last_faults_check_time_ = millis();
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
            // SerialPort.println("w1");
            drvRead(MOTOR1_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor1_data.raw_speed);
            motor1_data.speed = (motor1_data.raw_speed[0]<<8) | motor1_data.raw_speed[1]; //[RPM]
        }
        else if (i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR1)
        {
            // SerialPort.println("i1");
            drvRead(MOTOR1_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor1_data.raw_current);
            uint16_t current_aux = (motor1_data.raw_current[0]&0x07)<<8 | motor1_data.raw_current[1];
            if (current_aux>=1023)
            {
                motor1_data.current = 1.46484375*(current_aux-1023);       //[mA]
            }
            else
            {
                motor1_data.current = 1.46484375*(current_aux);       //[mA]
            }
        }
        else if (i2c_cmd.code == SET_SPEED_CODE_MOTOR1)
        {
            // SerialPort.println("set w1");
            set_dir_m1(i2c_cmd.dir);
            setMotorSpeed(MOTOR1_ID, i2c_cmd.speed);
            uart_packet_.cmd_speed_m1 = i2c_cmd.speed;
        }
        else if (i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR2)
        {
            // SerialPort.println("w2");
            drvRead(MOTOR2_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor2_data.raw_speed);
            motor2_data.speed = (motor2_data.raw_speed[0]<<8) | motor2_data.raw_speed[1]; //[RPM]
        }
        else if (i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR2)
        {
            // SerialPort.println("i2");
            drvRead(MOTOR2_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor2_data.raw_current);
            uint16_t current_aux = (motor2_data.raw_current[0]&0x07)<<8 | motor2_data.raw_current[1];
            if (current_aux>=1023)
            {
                motor2_data.current = 1.46484375*(current_aux-1023); //[mA]
            }
            else
            {
                motor2_data.current = 1.46484375*(current_aux); //[mA]
            }
        }
        else if (i2c_cmd.code == SET_SPEED_CODE_MOTOR2)
        {
            // SerialPort.println("set w2");
            set_dir_m2(i2c_cmd.dir);
            setMotorSpeed(MOTOR2_ID, i2c_cmd.speed);
            uart_packet_.cmd_speed_m2 = i2c_cmd.speed;
        }
        else if (i2c_cmd.code == SAMPLE_SPEED_CODE_MOTOR3)
        {
            // SerialPort.println("w3");
            drvRead(MOTOR3_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor3_data.raw_speed);
            motor3_data.speed = (motor3_data.raw_speed[0]<<8) | motor3_data.raw_speed[1]; //[RPM]
        }
        else if (i2c_cmd.code == SAMPLE_CURRENT_CODE_MOTOR3)
        {
            // SerialPort.println("i3");
            drvRead(MOTOR3_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor3_data.raw_current);
            uint16_t current_aux = (motor3_data.raw_current[0]&0x07)<<8 | motor3_data.raw_current[1];
            if (current_aux>=1023)
            {
                motor3_data.current = 1.46484375*(current_aux-1023); //[mA]
            }
            else
            {
                motor3_data.current = 1.46484375*(current_aux); //[mA]
            }
        }
        else if (i2c_cmd.code == SET_SPEED_CODE_MOTOR3)
        {
            // SerialPort.println("set w3");
            set_dir_m3(i2c_cmd.dir);
            setMotorSpeed(MOTOR3_ID, i2c_cmd.speed);
            uart_packet_.cmd_speed_m3 = i2c_cmd.speed;
        }
        i2c_cmd_result = 0;
        i2c_cmd.code = 0;
        i2c_cmd.speed = 0;
        // Wire.begin(SAMD21GD_ADDR);      //initialize as slave
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
        if (current_aux1>=1023)
        {
            motor1_data.current = 1.46484375*(current_aux1-1023);           //[mA]
        }
        else
        {
            motor1_data.current = 1.46484375*(current_aux1);           //[mA]
        }
    }
    if (motor_id == MOTOR2_ID)
    {
        // Read motor 2 state
        drvRead(MOTOR2_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor2_data.raw_speed);
        motor2_data.speed = (motor2_data.raw_speed[0]<<8) | motor2_data.raw_speed[1];     //[RPM]
        // delay(5);
        drvRead(MOTOR2_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor2_data.raw_current);
        uint16_t current_aux2 = (motor2_data.raw_current[0]&0x07)<<8 | motor2_data.raw_current[1];
        if (current_aux2>=1023)
        {
            motor2_data.current = 1.46484375*(current_aux2-1023);           //[mA]
        }
        else
        {
            motor2_data.current = 1.46484375*(current_aux2);           //[mA]
        }
    }
    if (motor_id == MOTOR3_ID)
    {
        // Read motor 3 state
        drvRead(MOTOR3_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor3_data.raw_speed);
        motor3_data.speed = (motor3_data.raw_speed[0]<<8) | motor3_data.raw_speed[1];     //[RPM]
        // delay(5);
        drvRead(MOTOR3_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor3_data.raw_current);
        uint16_t current_aux3 = (motor3_data.raw_current[0]&0x07)<<8 | motor3_data.raw_current[1];
        if (current_aux3>=1023)
        {
            motor3_data.current = 1.46484375*(current_aux3-1023);           //[mA]
        }
        else
        {
            motor3_data.current = 1.46484375*(current_aux3);           //[mA]
        }
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
    if (current_aux1>=1023)
    {
        motor1_data.current = 1.46484375*(current_aux1-1023);           //[mA]
    }
    else
    {
        motor1_data.current = 1.46484375*(current_aux1);           //[mA]
    }
    // Read motor 2 state
    drvRead(MOTOR2_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor2_data.raw_speed);
    motor2_data.speed = (motor2_data.raw_speed[0]<<8) | motor2_data.raw_speed[1];     //[RPM]
    // delay(5);
    drvRead(MOTOR2_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor2_data.raw_current);
    uint16_t current_aux2 = (motor2_data.raw_current[0]&0x07)<<8 | motor2_data.raw_current[1];
    if (current_aux2>=1023)
    {
        motor2_data.current = 1.46484375*(current_aux2-1023);           //[mA]
    }
    else
    {
        motor2_data.current = 1.46484375*(current_aux2);           //[mA]
    }
    // Read motor 3 state
    drvRead(MOTOR3_ID, DRV10987_MOTORSPEED_STATUS_ADDR, motor3_data.raw_speed);
    motor3_data.speed = (motor3_data.raw_speed[0]<<8) | motor3_data.raw_speed[1];     //[RPM]
    // delay(5);
    drvRead(MOTOR3_ID, DRV10987_MOTORCURRENT_STATUS_ADDR, motor3_data.raw_current);
    uint16_t current_aux3 = (motor3_data.raw_current[0]&0x07)<<8 | motor3_data.raw_current[1];
    if (current_aux3>=1023)
    {
        motor3_data.current = 1.46484375*(current_aux3-1023);           //[mA]
    }
    else
    {
        motor3_data.current = 1.46484375*(current_aux3);           //[mA]
    }
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
    // noInterrupts();
    reading_drvs = 1;
    tcaReset();
    // i2c_multiplexer.begin();        //initialize as master
    i2c_multiplexer.setClock(100000);
    // Selec corresponding multiplexer channel
    if (motor_id == MOTOR1_ID)
    {
        // SerialPort.println("m1");
        green_blink(5);
        tcaselect(MOTOR1_I2C_PORT);
    }
    else if (motor_id == MOTOR2_ID)
    {
        // SerialPort.println("m2");
        blue2_blink(15);
        tcaselect(MOTOR2_I2C_PORT);
    }
    else if (motor_id == MOTOR3_ID)
    {
        // SerialPort.println("m3");
        orange_blink(20);
        tcaselect(MOTOR3_I2C_PORT);
    }
    i2c_multiplexer.beginTransmission(DRV10987_ADDR);         // transmit to device
    uint8_t bytesSent = i2c_multiplexer.write(reg_addr);      // sends reg addr
    byte errcode = i2c_multiplexer.endTransmission(false);    // stop transmitting
    i2c_multiplexer.requestFrom(DRV10987_ADDR, 2);            // request data from given addr
    if (i2c_multiplexer.available())
    {   //TODO: slave may send less than requested, check condition
        data[0] = i2c_multiplexer.read();                     // read MSB of data
        data[1] = i2c_multiplexer.read();                     // read LSB of data
    }
    // errcode = i2c_multiplexer.endTransmission(false);    // stop transmitting
    // i2c_multiplexer.flush();
    // delay(100);   //TODO: check if/why this is necessary
    reading_drvs = 0;
    // tcaDeSelect();
    // tcaReset();
    // i2c_multiplexer.setClock(100000);
    // interrupts();
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
    reading_drvs = 1;
    tcaReset();
    // i2c_multiplexer.begin();        //initialize as master
    i2c_multiplexer.setClock(100000);
    // Selec corresponding multiplexer channel
    if (motor_id == MOTOR1_ID)
    {
        green_blink(20);
        tcaselect(MOTOR1_I2C_PORT);
    }
    else if (motor_id == MOTOR2_ID)
    {
        blue2_blink(60);
        tcaselect(MOTOR2_I2C_PORT);
    }
    else if (motor_id == MOTOR3_ID)
    {
        orange_blink(120);
        tcaselect(MOTOR3_I2C_PORT);
    }
    i2c_multiplexer.beginTransmission(DRV10987_ADDR);
    i2c_multiplexer.write(reg_addr);                 // sends reg addr
    i2c_multiplexer.write(data1);                    // send MSB first
    i2c_multiplexer.write(data2);                    // send LSB second
    i2c_multiplexer.endTransmission(true);           // stop transmitting
    // delay(100);   //TODO: check if/why this is necessary
    reading_drvs = 0;
    // tcaDeSelect();
    // tcaReset();
    // i2c_multiplexer.setClock(100000);
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
void RWFirmware::display_motorsdata_uart()
{
    SerialPort.print("m1_w: ");
    SerialPort.print(motor1_data.speed);
    SerialPort.print("\t\t");
    SerialPort.print("m2_w: ");
    SerialPort.print(motor2_data.speed);
    SerialPort.print("\t\t");
    SerialPort.print("m3_w: ");
    SerialPort.print(motor3_data.speed);
    SerialPort.print("\t\t");

    SerialPort.print("m1_i: ");
    SerialPort.print(motor1_data.current);
    SerialPort.print("\t\t");
    SerialPort.print("m2_i: ");
    SerialPort.print(motor2_data.current);
    SerialPort.print("\t\t");
    SerialPort.print("m3_i: ");
    SerialPort.println(motor3_data.current);
}

void RWFirmware::checkFaults()
{
    if ((millis()-last_faults_check_time_)>FAULTS_CHECK_TIME)
    {
        readStatesDRVs();
        display_motorsdata_uart();

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

void RWFirmware::tcaselect(uint8_t i) {
  if (i > 7) return;
 
  i2c_multiplexer.beginTransmission(MULTIPLEXER_ADDR);
  i2c_multiplexer.write(1 << i);
  i2c_multiplexer.endTransmission();  
}
void RWFirmware::tcaDeSelect() {
 
  i2c_multiplexer.beginTransmission(MULTIPLEXER_ADDR);
  i2c_multiplexer.write(0);
  i2c_multiplexer.endTransmission();  
}

void RWFirmware::tcaReset() {
  PORT->Group[PORTA].OUTCLR.reg = MULTIPLEXER_RST_PA04;
  delay(10);
  PORT->Group[PORTA].OUTSET.reg = MULTIPLEXER_RST_PA04;
}

void RWFirmware::blue1_blink(uint8_t dly_ms){
  PORT->Group[PORTA].OUTSET.reg = LED_BLUE1;
  delay(dly_ms);
  PORT->Group[PORTA].OUTCLR.reg = LED_BLUE1;
  delay(dly_ms);
}

void RWFirmware::green_blink(uint8_t dly_ms){
  PORT->Group[PORTB].OUTSET.reg = LED_GREEN;
  delay(dly_ms);
  PORT->Group[PORTB].OUTCLR.reg = LED_GREEN;
  delay(dly_ms);
}

void RWFirmware::blue2_blink(uint8_t dly_ms){
  PORT->Group[PORTA].OUTSET.reg = LED_BLUE2;
  delay(dly_ms);
  PORT->Group[PORTA].OUTCLR.reg = LED_BLUE2;
  delay(dly_ms);
}

void RWFirmware::orange_blink(uint8_t dly_ms){
  PORT->Group[PORTB].OUTSET.reg = LED_ORANGE;
  delay(dly_ms);
  PORT->Group[PORTB].OUTCLR.reg = LED_ORANGE;
  delay(dly_ms);
}

void RWFirmware::blue1_on(){
  PORT->Group[PORTA].OUTSET.reg = LED_BLUE1;
}

void RWFirmware::green_on(){
  PORT->Group[PORTB].OUTSET.reg = LED_GREEN;
}

void RWFirmware::blue2_on(){
  PORT->Group[PORTA].OUTSET.reg = LED_BLUE2;
}

void RWFirmware::orange_on(){
  PORT->Group[PORTB].OUTSET.reg = LED_ORANGE;
}

void RWFirmware::blue1_off(){
  PORT->Group[PORTA].OUTCLR.reg = LED_BLUE1;
}
void RWFirmware::green_off(){
  PORT->Group[PORTB].OUTCLR.reg = LED_GREEN;
}
void RWFirmware::blue2_off(){
  PORT->Group[PORTA].OUTCLR.reg = LED_BLUE2;
}
void RWFirmware::orange_off(){
  PORT->Group[PORTB].OUTCLR.reg = LED_ORANGE;
}

void RWFirmware::set_dir_m1(uint8_t dir){
    if (dir == 0)
    {
        PORT->Group[PORTB].OUTCLR.reg = MOTOR1_DIR_PB23;
    }
    else
    {
        PORT->Group[PORTB].OUTSET.reg = MOTOR1_DIR_PB23;
    }
}

void RWFirmware::set_dir_m2(uint8_t dir){
    if (dir == 0)
    {
        PORT->Group[PORTB].OUTCLR.reg = MOTOR2_DIR_PB22;
    }
    else
    {
        PORT->Group[PORTB].OUTSET.reg = MOTOR2_DIR_PB22;
    }
}

void RWFirmware::set_dir_m3(uint8_t dir){
    if (dir == 0)
    {
        PORT->Group[PORTA].OUTCLR.reg = MOTOR3_DIR_PA10;
    }
    else
    {
        PORT->Group[PORTA].OUTSET.reg = MOTOR3_DIR_PA10;
    }
}