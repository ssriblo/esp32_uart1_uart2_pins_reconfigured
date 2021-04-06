# esp32_uart1_uart2_pins_reconfigured
ESP32 code: Round-robin tx of UART1&amp;UART2 to multiple pins; rx UART1 and UART2 on fixed pins.

from https://www.upwork.com/jobs/~01fc48e9ab3ce2538d
Looking for concept validation ESP32 C code (Arduino preferred, IDF ok) that will send strings out on different pins on the ESP32, multiplexing the use of the UART transmit on multiple pins so that the two UART transmitters can send a message out on to any of the GPO pins on the ESP32.

If both UARTs are busy, either return a busy() status or enqueue the message so that it will transmit when a UART is free. Please gracefully handle so that if two messages are scheduled for the same port, the first message completes before the second is sent (either from the same UART or the other UARTs after pin mapping change). UART pin changes should occur only after the respective TX register is empty. When switching UART from one pin to another, the suspended pin should be appropriately pulled so that there is no line break.

Build command:

 idf.py -p /dev/ttyUSB0 flash monitor 2>&1 | tee log1.txt

Turn on logging prints - comment this line at the app_main():
    esp_log_level_set(TAG, ESP_LOG_NONE);     // Disabe local log

Test string at the routing_task() function 
	char *mydata="ABCDEFGHIJ";
String Format to sent - 16 variants at the loop. Loop counter is "cycle16"=[0...15]:
<pin-msb><pin-lsb><blank><cycle16-msb><cycle16-lsb><blank><mydata>

Pin number hardcoded at the array:
    pinMsgList[0...15][1]

