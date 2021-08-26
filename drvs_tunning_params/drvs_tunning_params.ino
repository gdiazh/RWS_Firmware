#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

// Device addres
#define BIuC_ADDR 0x11
#define MOTOR1_ADDR 0x52
#define DEVID_ADDR 0X08
#define CONFIG1_ADDR 0X90
#define CONFIG2_ADDR 0X91
#define CONFIG3_ADDR 0X92
#define CONFIG4_ADDR 0X93
#define CONFIG5_ADDR 0X94
#define CONFIG6_ADDR 0X95
#define CONFIG7_ADDR 0X96
#define EECTRL_ADDR 0x60
#define EEPROM_PROG1_ADDR 0x31
#define EEPROM_PROG2_ADDR 0x32
#define EEPROM_PROG5_ADDR 0x35

#define MOTOR1_ID 1
#define MOTOR2_ID 2
#define MOTOR3_ID 3


// Motor Parameters
// #define CONSTANT_RM 0x3C //RPH_CT = 1.8624[Ohm], the closest to 1.9[Ohm] measured
// #define CONSTANT_KT 0x2a //Kt = 16.56[mV/Hz], the closest to 16.2504 [mV/Hz] measured

#define CONSTANT_RM 0x48 //RPH_CT = 1.8624[Ohm], the closest to 1.9[Ohm] measured
#define CONSTANT_KT 0x08 //Kt = 16.56[mV/Hz], the closest to 16.2504 [mV/Hz] measured

// #define CONSTANT_RM 0x3D //RPH_CT = 1.8624[Ohm], the closest to 1.9[Ohm] measured
// #define CONSTANT_KT 0x06 //Kt = 16.56[mV/Hz], the closest to 16.2504 [mV/Hz] measured


#define SerialPort SerialUSB
// #define SerialPort Serial
#define DEBUG_LED 5

// TwoWire motor1(&sercom3, 20, 21);
TwoWire motor2(&sercom1, 11, 13); //SDA[PAD0], SCL[PAD1]
TwoWire motor3(&sercom2, 4, 3);   //SDA, SCL

// variables
uint8_t eeReadyStatus[2] = {0, 0};
uint8_t config1Status[2] = {0, 0};
uint8_t config2Status[2] = {0, 0};
uint8_t config3Status[2] = {0, 0};
uint8_t config4Status[2] = {0, 0};
uint8_t config5Status[2] = {0, 0};
uint8_t config6Status[2] = {0, 0};
uint8_t config7Status[2] = {0, 0};
uint8_t devId[2] = {0, 0};
float rmValue = 0;

void setup() {
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, HIGH);
  SerialPort.println("Read EEPROM");
  SerialPort.begin(115200);
  
  Wire.begin(BIuC_ADDR);
  motor2.begin(BIuC_ADDR);
  motor3.begin(BIuC_ADDR);

  pinPeripheral(11, PIO_SERCOM);    //SDA
  pinPeripheral(13, PIO_SERCOM);    //SCL
  pinPeripheral(4, PIO_SERCOM_ALT); //SDA
  pinPeripheral(3, PIO_SERCOM_ALT); //SCL

  Wire.setClock(200000);
  motor2.setClock(200000);
  motor3.setClock(200000);

  delay(1000);
  digitalWrite(DEBUG_LED, LOW);
  delay(1000);

  // Write config registers
  // digitalWrite(DEBUG_LED, HIGH);
  writeEEPROM(MOTOR3_ID);
  // writeEEPROM(MOTOR2_ID);
  // writeEEPROM(MOTOR3_ID);

  // Read EEPROM
  readEEPROM(MOTOR3_ID);
  // readEEPROM(MOTOR2_ID);
  // readEEPROM(MOTOR3_ID);

  SerialPort.println("Readed EEPROM");  
}


void loop()
{ 
  // digitalWrite(DEBUG_LED, LOW);
  SerialPort.println("Reading registers");

  configByMotor(MOTOR3_ID);
  // configByMotor(MOTOR2_ID);
  // configByMotor(MOTOR3_ID);
  // digitalWrite(DEBUG_LED, HIGH);
  delay(100);
}

void configByMotor(uint8_t motor_id)
{
  drvRead(motor_id, CONFIG1_ADDR, config1Status);
  SerialPort.print("config1Status: ");
  SerialPort.print(config1Status[0], HEX);SerialPort.print(" ");
  SerialPort.println(config1Status[1], HEX);

  drvRead(motor_id, CONFIG2_ADDR, config2Status);
  SerialPort.print("config2Status: ");
  SerialPort.print(config2Status[0], HEX);SerialPort.print(" ");
  SerialPort.println(config2Status[1], HEX);

  drvRead(motor_id, CONFIG3_ADDR, config3Status);
  SerialPort.print("config3Status: ");
  SerialPort.print(config3Status[0], HEX);SerialPort.print(" ");
  SerialPort.println(config3Status[1], HEX);

  drvRead(motor_id, CONFIG4_ADDR, config4Status);
  SerialPort.print("config4Status: ");
  SerialPort.print(config4Status[0], HEX);SerialPort.print(" ");
  SerialPort.println(config4Status[1], HEX);

  drvRead(motor_id, CONFIG5_ADDR, config5Status);
  SerialPort.print("config5Status: ");
  SerialPort.print(config5Status[0], HEX);SerialPort.print(" ");
  SerialPort.println(config5Status[1], HEX);

  drvRead(motor_id, CONFIG6_ADDR, config6Status);
  SerialPort.print("config6Status: ");
  SerialPort.print(config6Status[0], HEX);SerialPort.print(" ");
  SerialPort.println(config6Status[1], HEX);

  drvRead(motor_id, CONFIG7_ADDR, config7Status);
  SerialPort.print("config7Status: ");
  SerialPort.print(config7Status[0], HEX);SerialPort.print(" ");
  SerialPort.println(config7Status[1], HEX);

  /*drvRead(DEVID_ADDR, devId);
  SerialPort.print("Rev ID: ");SerialPort.println(devId[0]);
  SerialPort.print("Dev ID: ");SerialPort.println(devId[1]);
  delay(100);*/
}

void drvWrite(uint8_t motor_id, uint8_t reg_addr, uint8_t data1, uint8_t data2)
{

  if (motor_id == MOTOR1_ID)
  {
    Wire.setClock(200000);
    Wire.beginTransmission(MOTOR1_ADDR);
    Wire.write(reg_addr);                 // sends reg addr
    Wire.write(data1);                    // sends [0:7] data
    Wire.write(data2);                    // sends [8:15] data
    Wire.endTransmission(true);           // stop transmitting
    Wire.begin(BIuC_ADDR);
  }
  else if (motor_id == MOTOR2_ID)
  {
    motor2.beginTransmission(MOTOR1_ADDR);
    motor2.write(reg_addr);                 // sends reg addr
    motor2.write(data1);                    // sends [0:7] data
    motor2.write(data2);                    // sends [8:15] data
    motor2.endTransmission(true);           // stop transmitting
  }
  else if (motor_id == MOTOR3_ID)
  {
    motor3.beginTransmission(MOTOR1_ADDR);
    motor3.write(reg_addr);                 // sends reg addr
    motor3.write(data1);                    // sends [0:7] data
    motor3.write(data2);                    // sends [8:15] data
    motor3.endTransmission(true);           // stop transmitting
  }
}

void drvRead(uint8_t motor_id, uint8_t reg_addr, uint8_t *data)
{
  if (motor_id == MOTOR1_ID)
  {
    Wire.setClock(200000);
    Wire.beginTransmission(MOTOR1_ADDR);         // transmit to device #4
    uint8_t bytesSent = Wire.write(reg_addr);        // sends reg addr
    byte errcode = Wire.endTransmission(false);    // stop transmitting
    Wire.requestFrom(MOTOR1_ADDR, 2);
    if (Wire.available())
    { // slave may send less than requested
      data[0] = Wire.read();
      data[1] = Wire.read();
    }
    Wire.begin(BIuC_ADDR);
  }
  else if (motor_id == MOTOR2_ID)
  {
    motor2.beginTransmission(MOTOR1_ADDR);         // transmit to device #4
    uint8_t bytesSent = motor2.write(reg_addr);        // sends reg addr
    byte errcode = motor2.endTransmission(false);    // stop transmitting
    motor2.requestFrom(MOTOR1_ADDR, 2);
    if (motor2.available())
    { // slave may send less than requested
      data[0] = motor2.read();
      data[1] = motor2.read();
    }
  }
  else if (motor_id == MOTOR3_ID)
  {
    motor3.beginTransmission(MOTOR1_ADDR);         // transmit to device #4
    uint8_t bytesSent = motor3.write(reg_addr);        // sends reg addr
    byte errcode = motor3.endTransmission(false);    // stop transmitting
    motor3.requestFrom(MOTOR1_ADDR, 2);
    if (motor3.available())
    { // slave may send less than requested
      data[0] = motor3.read();
      data[1] = motor3.read();
    }
  }
}

void readEEPROM(uint8_t motor_id)
{
  delay(10);                               //wait 10ms
  drvWrite(motor_id, EECTRL_ADDR, 1<<7, 0x00);       //set MTR_DIS = EECTRL_ADDR[15] = 1 to disables the motor driver
  drvWrite(motor_id, EEPROM_PROG1_ADDR, 0X00, 0X00); //clear the EEPROM access code
  drvWrite(motor_id, EEPROM_PROG1_ADDR, 0XC0, 0XDE); //enable access to EEPROM
  drvRead(motor_id, EEPROM_PROG2_ADDR, eeReadyStatus);
  if (eeReadyStatus[1] == 1)
  { // We can access eeprom
    SerialPort.println("Accessing EEPROM");
    eeReadyStatus[0] = 0;
    eeReadyStatus[1] = 0;
    drvWrite(motor_id, EEPROM_PROG5_ADDR, 0X00, 0x02);
    // device starts reading the EEPROM and storing it in the shadow registers
    while (eeReadyStatus[1] == 0)
    {
      drvRead(motor_id, EEPROM_PROG2_ADDR, eeReadyStatus);
    }
    SerialPort.println("EEPROM Readed");
    // digitalWrite(DEBUG_LED, HIGH);
  }
  else
  {
    SerialPort.println("Fail reading EEPROM");
  }
  drvWrite(motor_id, EECTRL_ADDR, 0x00, 0x00); //set MTR_DIS = EECTRL_ADDR[15] = 0 to re-enable the motor driver
}

void writeEEPROM(uint8_t motor_id)
{
  delay(10);                               //wait 10ms
  digitalWrite(DEBUG_LED, HIGH);
  drvWrite(motor_id, EECTRL_ADDR, 1<<7, 0x00);       //set MTR_DIS = EECTRL_ADDR[15] = 1 to disables the motor driver
  digitalWrite(DEBUG_LED, LOW);
  drvWrite(motor_id, EEPROM_PROG1_ADDR, 0X00, 0X00); //clear the EEPROM access code
  drvWrite(motor_id, EEPROM_PROG1_ADDR, 0XC0, 0XDE); //enable access to EEPROM
  drvRead(motor_id, EEPROM_PROG2_ADDR, eeReadyStatus);
  // while (eeReadyStatus[0] != 1)
  // {
  //   drvRead(motor_id, EEPROM_PROG2_ADDR, eeReadyStatus);
  // }
  if (eeReadyStatus[1] == 1)
  { // We can access eeprom
    SerialPort.println("Accessing EEPROM");
    eeReadyStatus[0] = 0;
    eeReadyStatus[1] = 0;
    
    //***** Config 1 *****
    // 15-14 : 11   -> +-15% dithering
    // 13-12 : 00   -> FG outputs in both open loop and closed loop
    // 11-8  : 1011 -> n=11 FG output is electrical speed / (n + 1)
    // 7     : 0    -> Full-cycle adjus
    // 6-0   : CONSTANT_RM
    // Value : 11001011 | 0 0x03 (default: 0xc0 | 0x00)
    drvWrite(motor_id, CONFIG1_ADDR, 0xc0, CONSTANT_RM);
    // drvWrite(motor_id, CONFIG1_ADDR, 0xcb, CONSTANT_RM);

    //***** Config 2 *****
    // 15    : 0             -> reserved
    // 14-8  : CONSTANT_KT    
    // 7     : 0             -> Volt. advance is maintained at a fixed time relative to the estimated BEMF
    // 6-0   : 0000000       -> commutation advance timing
    // Value : 0 0x2d | 0  (default: 0x00 | 0x49)
    drvWrite(motor_id, CONFIG2_ADDR, CONSTANT_KT, 0x99);
    // drvWrite(motor_id, CONFIG2_ADDR, CONSTANT_KT, 0xce);

    //***** Config 3 *****
    // 15-14 : 00            -> 6Hz ISD
    // 13    : 0             -> 24mA Brake current-level-threshold
    // 12    : 0             -> 20mV hysteresis for BEMF
    // 11    : 0             -> ISD disabled
    // 10    : 0             -> Reverse drive disabled
    // 9:8   : 00            -> 6.3 Hz reverse drive threshold
    // 7:6   : 11            -> 1.6A Open-loop current and 1.2A Align current
    // 5:3   : 001           -> 3Vcc/s
    // 2:0   : 001           -> 2.7s
    //Value : 0 0x2d | 0 (default: 0x00 | 0xc1)
    drvWrite(motor_id, CONFIG3_ADDR, 0x05, 0xc0);
    // drvWrite(motor_id, CONFIG3_ADDR, 0x00, 0xc9);

    //***** Config 4 *****
    // 15    : 0             -> reserved
    // 14    : 0             -> Fast
    // 13-11 : 000           -> 010 = 3.3 Hz/s^2
    // 10-8  : 000           -> 000 = 76 Hz/s
    // 7-3   : 1 1111        -> 01111 = 0.8 Hz/s
    // 2-0   : 110           -> 011 = 2.7 s
    // Value : 0 0x2d | 0 (default: 0x37 | 0x88)
    // drvWrite(motor_id, CONFIG4_ADDR, 0x37, 0x88);
    drvWrite(motor_id, CONFIG4_ADDR, 0x00, 0xba);

    //***** Config 5 *****
    // 15-14 : 00            -> No temperature-based current-limit
    // 13    : 0             -> Stuck in closed loop disable
    // 12    : 0             -> Open loop stuck disable
    // 11    : 0             -> No motor fault disable
    // 10    : 0             -> Abnormal Kt disable 
    // 9     : 0             -> Abnormal speed disable 
    // 8     : 0             -> Lock-detection current limit disable
    // 7-4   : 0000          -> Software current limit disable
    // 3-1   : 111           -> 3.2A HWILimitThr
    // 0     : 1             -> Range2 of current limit for HWILimitThr
    // Value : 0 0x2d | 0 (default: 0x3b | 0xaf)
     // drvWrite(motor_id, CONFIG5_ADDR, 0x3b, 0xaf);
    drvWrite(motor_id, CONFIG5_ADDR, 0x00, 0xf);

    //***** Config 6 *****
    drvWrite(motor_id, CONFIG6_ADDR, 0x38, 0x40);
    
    // ... 3-7 ... TODO
    // drvWrite(motor_id, CONFIG3_ADDR, config3Status[1], config3Status[0]);
    // drvWrite(motor_id, CONFIG4_ADDR, config4Status[1], config4Status[0]);
    // drvWrite(motor_id, CONFIG5_ADDR, config5Status[1], config5Status[0]);
    // drvWrite(motor_id, CONFIG6_ADDR, config6Status[1], config6Status[0]);
    // drvWrite(motor_id, CONFIG7_ADDR, config7Status[1], config7Status[0]);

    drvWrite(motor_id, EEPROM_PROG5_ADDR, 0X00, 0x06);  //updare EEPROM with shadows registers
    // device starts reading the EEPROM and storing it in the shadow registers
    while (eeReadyStatus[1] == 0)
    {
      drvRead(motor_id, EEPROM_PROG2_ADDR, eeReadyStatus);
    }
    SerialPort.println("EEPROM Writed");
    digitalWrite(DEBUG_LED, HIGH);
  }
  else
  {
    SerialPort.println("Fail writing EEPROM");
  }
}