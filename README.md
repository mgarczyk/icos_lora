# icos_lora
Repository contains code for LoRa modules and ROS2 nodes to create additional communication link beetewen robot and remote controller.
# LoRa module (Heltec WiFi LoRa 32(v3))
Code is stored in LoRa_module folder, it is documented by comments. To porgram module we need arduino IDE or VSCode Community, arduino IDE is easier to set up.
1) Install Arduino IDE
2) When choosing board, first install additional ESP32 boards. Then choose Heltec WiFi LoRa 32(v3).
3) Then install all dependencies, by library manager:
  lib_deps =
	<br /> heltecautomation/Heltec ESP32 Dev-Boards@^2.0.2
	<br /> jgromes/RadioLib@^6.6.0
	 <br />ropg/HotButton@^0.1.1
	<br />ropg/Heltec_ESP32_LoRa_v3@^0.9.1
	<br /> thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays@^4.6.1
 <br /> Last one is needed only if you want using OLED screen it is useful for debbuging purpose, but probably wouldn't be used in final version installed on robot.
4) Next step is little tricky, but needed. We need to comment or delete lines 23-29 in Arduino\libraries\Heltec_ESP32_LoRa_v3\src\RadioLib_convenience.h if arduino IDE was used. This is some status information for debugging, and we didin't want to send it anywhere. If VSCode community and platformio was used to set up board path is libdeps\Heltec_ESP32_LoRa_v3\src\RadioLib_convenience.h.
5) Then we program MCU with file main.cpp
# ROS2 modules
There are two script write in Python, one for the robot and second for the remote controller. Scripts neeed to be added to ROS2 workspace of the robot (place in relevant folder and added into xml or yaml or python config file). Subscriber take navigation data from robot and send by UART into LoRa module, and publisher recive data from UART, then create navigation frame, and publish it into 'lora/ublox_rover/fix' and 'lora/ublox_moving_base/fix' channels.
# Tests
We provided some manual tests with use of rosbag and data base that we recived. Evrything looks good, but more tests is neeeded.
