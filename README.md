# DRV10987 RW's Firmware
Arduino/C++ Library Firmware for Reaction Wheels using 3 DRV10987 BLDC motor drivers
and ROS-Python Serial - UART Interface/Controller

## Brief

* [rws_firmware/](#firmware) Contains the Arduino/C++ Library and main program
* [ros_uart_controller/](#controller) Contains the ROS-Python Serial - UART Interface

### Firmware Upload
* Open the rws_firmware.ino file on Arduino IDE
* Edit the `#define SerialPort SerialUSB` line acording to the Serial interface of your board
* Upload

### ROS-Python Serial-UART Interface
* Quick Test
	* Go to `ros_uart_controller/` folfer and run
	```python
	python3 uart_speed_step.py
	```
	* Type `dummy` for file name when the script request it 