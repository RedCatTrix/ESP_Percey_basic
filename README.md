# ESP_Percey_basic
ESP32/ESP8266 Telegram bot sketch for Percey: automatic toilet flusher

PLEASE NOTE

1) Before compile set values pin_UART_TX and pin_LED accordingly to your platform
2) Install FastBot library


This sketch demonstrates basic communication principles with Percey flusher system via Universal Asynchronous Transmitter Receiver protocol (UART).
The sketch implemented on synchronous communication with Telegram server and based on FastBot library, for more details refer to its GitHub: https://github.com/GyverLibs/FastBot/tree/main

Simple communication diagram:
[Percey flusher] <----UART----> [ESP8266/ESP32] <----Internet----> [Telegram chat]

Platform: ESP8266, ESP32
Latest update: 28.03.2024

Percey flusher guide and description:

(ENG) https://drive.google.com/file/d/1GUO4CxzXS69auFtKdyNXuGJjHMAfbz1F/view

(RUS) https://drive.google.com/file/d/1CV3mvw3wF0RgGhDVKfFtLYkZRFTRWWTS/view


A few advices to harness ESP:

1) ESP operates unstable while heavy HEAP load. Use UTF-8 character text.
2) Avoid unreasonably long text.
3) Avoid FPSTR() macro, try F() instead.
4) FastBot implemented on Telegram bot API, apparantly it has low service priority on Telegram server. 
   Thus, delays or message drop in some case may occur. Especially if requests too frequent.
   The sketch uses simple trick to achieve better communication experience with Server.
   This acts kinda time damper, because Telegram server do not love too frequent requests.
5) Add yield() inside while() loops


For more details message me: cattrix@pm.me
