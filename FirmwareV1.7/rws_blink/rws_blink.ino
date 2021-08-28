#define SerialPort SerialUSB
 
void setup() {
  // ----------------------------------- LED Pins -----------------------------------------
  // PORT->Group[PORTA].DIRSET.reg = PORT_PA15;
  // PORT->Group[PORTA].OUTCLR.reg = PORT_PA15;
  pinMode(5, OUTPUT);

  // ----------------------------------- Debug --------------------------------------------

  SerialPort.begin(115200);
}

void loop()
{
  // PORT->Group[PORTA].OUTSET.reg = PORT_PA15;
  digitalWrite(5, HIGH);
  delay(100);
  // PORT->Group[PORTA].OUTCLR.reg = PORT_PA15;
  digitalWrite(5, LOW);
  delay(100);
}