/*

PLEASE NOTE

1) Before compile set values pin_UART_TX and pin_LED accordingly to your platform
2) Install FastBot library


This sketch demonstrates basic communication principles with Percey flusher system via Universal Asynchronous Transmitter Receiver protocol (UART).
The sketch implemented on synchronous communication with Telegram server and based on FastBot library, for more details refer to its GitHub: https://github.com/GyverLibs/FastBot/tree/main

Simple communication diagram:
[Percey flusher] <----UART----> [ESP8266/ESP32] <----Internet----> [Telegram chat]


Platforms: ESP8266, ESP32
Latest update: 11.03.2024
Author's GitHub: https://github.com/RedCatTrix

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

*/


/* Attached libraries */
#include <FastBot.h>
//#include <EEPROM.h>

/* Compiler setup */
#undef DEBUG_READ

/* Remote devices */
//#define FAN_DEV_ADDRESS 0xYY

/* PINs                                                                     ESP32-C3_mini  ESP8266            */
#define pin_UART_TX   1                                                   //      21          1
#define pin_LED       2                                                   //      8           2

/* macros */
#define activity_LED(n)  digitalWrite(pin_LED, !n)

/* UART setting */
#define UART_BAUD     115200                                              // 115200 dafault baudrate, do not change
#define UART_MAX_BUF  56                                                  // UART message maximum length is 56 bytes


/* WiFi connection data */
#define WiFi_SSID     "networkname"                                       // Your WiFi name
#define WiFi_PASSWORD "password"                                          // Your WiFi password

/*  Telegram bot access data */
#define BOT_TOKEN     "1234567890:token-token-token-token-token-token"    // Your bot token, ask Telegram @BotFather for that
#define MY_CHAT_ID    "123456789"                                         // Your uniq Telegram chat identifier, get this ID via @username_to_id_bot (IDBot) in Telegram

/*  OTA   */
#define OTA_FILE  "otafw.bin"

/*    commands and other      */
#define ON                    0
#define OFF                   0
#define	CMD_IN_HANDSHAKE      254
#define	CMD_OUT_HANDSHAKE	    253
#define	CMD_POWER_FAILURE	    2
#define	CMD_SENSOR_ERROR	    3
#define	CMD_DEBUG_PRINTOUT	  4
#define	CMD_MESSAGE_ACK	      9
#define	CMD_SYSTEM_INIT		    10
#define	CMD_EDC_EVENT		      14
#define	CMD_EDC_EVENT_RUN	    0
#define	CMD_EDC_EVENT_STOP	  1
#define	CMD_EDC_EVENT_RESET	  2
#define	CMD_EDC_EVENT_WAKE	  5
#define	CMD_REFILL_COMPLETE	  20
#define	CMD_AUTO_FLUSH	      21
#define	CMD_SYSTEM_HALTED	    23
#define	CMD_SYS_HLT_SENSOR	  0
#define	CMD_SYS_HLT_FLUSH	    1
#define	CMD_SYS_HLT_REFILL	  2
#define	CMD_SYS_HLT_TEST	    3
#define	CMD_SYS_HLT_MANU	    4
#define	CMD_TEST_BEEP_BEEP	  25
#define	CMD_SYSTEM_STATUS	    27
#define	CMD_MRA_COMPLETE	    28
#define	CMD_NO_WATER_REFILL	  29
#define	CMD_SYSTEM_SLEEP	    30
#define	CMD_LEAKAGE_DETECTED  31
#define	CMD_MRA_RESULT	      32
#define	CMD_MRA_RESULT_CAT	  0
#define	CMD_MRA_RESULT_SIT	  1
#define	CMD_MRA_RESULT_MAN	  2
#define	CMD_MRA_RESULT_UFO	  3
#define	CMD_WAIT_FOR_REFILL	  35
#define	CMD_EVAL_REFILL_TIME	40
#define	CMD_HALF_DELAY_ACK	  41
#define	CMD_SYSTEM_CONFIG	    42
#define	CMD_SYS_ACKNOWLEDGE	  43
#define	CMD_MANUAL_FLUSH	    63
#define	CMD_SYS_WAKE_INT	    94
#define	CMD_AUTO_FLUSH_RST	  157
#define	CMD_SYS_WAKE_EXT	    158
#define	CMD_PULL_VALVE	      159


/*  Static text   */
static const char text_start[] PROGMEM = "WELCOME! Here's command list:\n\n"
                                         "/system - System menu\n"
                                         "/halfdelay <val> - Set half flush delay\n"
                                         "/dbgon - Debug printout ON\n"
                                         "/dbgoff - Debug printout OFF\n"
                                         "/sm <msg> - Send message\n\n"
                                         "/WiFi_restart - ESP soft restart\n"
                                         "/ESP_svc - WiFi info & UART reset\n\n";

static const char text_SYS_caption[] PROGMEM = "SYSTEM MENU";                                                                                   // меню смыва, заголовок
static const char text_SYS_menu[] PROGMEM = "Full flush \t Half flush \t Fix valve \n Status \t Config \t Halt \n Reset \t Default \t Beep Beep";  // меню системы
static const char text_blank[] PROGMEM = " ";
static const char text_on[] PROGMEM = "ON";
static const char text_off[] PROGMEM = "OFF";
static const char text_cancel[] PROGMEM = "Flush cancelled";
static const char text_tank_fill[] PROGMEM = " refill ";
static const char text_cat[] PROGMEM = "CAT: ";
static const char text_sit[] PROGMEM = "SIT: ";
static const char text_man[] PROGMEM = "MAN: ";
static const char text_ufo[] PROGMEM = "UFO: ";
static const char text_yes[] PROGMEM = "YES";
static const char text_no[] PROGMEM = "NO";
static const char text_restart[] PROGMEM = "Restarting";
static const char text_no_power[] PROGMEM = "No power";
static const char text_halted[] PROGMEM = "Halted: ";
static const char text_sensor_error[] PROGMEM = "Sensor error";
static const char text_ok[] PROGMEM = "OK";
static const char text_failed[] PROGMEM = "failed";

/*  Create class object "bot" */
FastBot bot;

/*  UART TX/RX buffers  */
uint8_t buf_rx[UART_MAX_BUF];
uint8_t buf_tx[UART_MAX_BUF];

/*  Variables  */
  String s_buf                = "";       // generic String buffer
    bool uart_link_ok         = false;    // communication link is OK
    bool host_sleep           = true;     // host in sleep state
    bool skip_msg             = false;    // immune from Telegram server "punishment" duo too frequent requests
    bool awaiting_text_input  = false;    // to manage message menu
uint32_t msg_menu_id          = 0;        // message menu ID, used to delete this menu after action
    bool sys_suspended        = false;    // temporarily unable to process incoming message, doing some job
    bool restart_wifi         = false;    // FastBot: specific feature... refer to Fastbot doc for details
uint32_t sleep_time_mark      = 0;        // Telegram worship related variable

/* Method prototypes */
void try_to_awake_host_if_it_sleeping(bool forced = false);           // it tries to awake host if it sleeping
void UART_send_buffer();                                              // this method sends out data from buf_tx to host via Serial interface
void UART_handshake();                                                // check UART link state
void WiFi_connect();                                                  // WiFi connection procedure
void newMsg(FB_msg& msg);                                             // New message handler
void wordToHEXchar(word num, char* char_buf);                         // WORD (integer value) to HEX (char array) converter routine, not mandatory helper
void wash_dishes();                                                   // initializes both UART buffers
void botSendMessage(String& msg);                                     // bot.sendMessage() wrapper

void setup(void) {
  
  pinMode(pin_LED, OUTPUT);
  activity_LED(OFF);

  // Initialize UART hardware
  Serial.begin(UART_BAUD);

  // Connect to WiFi network
  // Try for 20 seconds, restart if failed and try over
  WiFi_connect();

  // Reserve 500 bytes for s_buf stringbuffer. Use heap memory reasonable.
  s_buf.reserve(350);

  // Initialize BOT data and send first message with available command list
  bot.setToken(BOT_TOKEN);
  bot.setChatID(MY_CHAT_ID);
  bot.attach(newMsg);

  // Send the first message to chat
  s_buf = FPSTR(text_start);
  botSendMessage(s_buf);
  // Check communication link state once run
  UART_handshake();
}

void loop() {

  // FastBot: request Telegram server every 3.6 seconds for updates. You can change it within setUpdates() method. There two update methods in Telegram API - getUpdates and Webhook, refer to Telegram API doc for details.
  bot.tick();

  // read UART buffer when available()
  if (Serial.available()) {

    // wait to get whole message before parsing, 30 ms pretty enough for 115200 bps rate
    delay(30);
  
    // total number of bytes bo be receive
    // incoming message length in buf_rx[0] 
    byte c = 0;

    // because there is incoming message, thus the host obviously not sleeping
    host_sleep = false;

    // Hanshake() handler
    if (Serial.peek() == CMD_IN_HANDSHAKE) {
      Serial.read();
      Serial.write(CMD_OUT_HANDSHAKE);  // incoming Handshake() response
      Serial.flush();
      uart_link_ok = true;
      activity_LED(ON);
      return;
    }

    // Read array bytes from UART buffer to local one
    do {
      buf_rx[c++] = Serial.read();
      yield();
    } while (Serial.available() and c < buf_rx[0] + 1);

    // Because we get message on UART, it means host available and not sleep
    uart_link_ok = true;
    activity_LED(ON);

    /*** UART incoming message parse ***/  
    // Ok, next let parse incoming message
    // as described in Percey doc, the first byte buf_rx[0] is a message length, buf_rx[1] is a command, rest bytes are arguments and parameters)

    switch (buf_rx[1]) {
      case CMD_POWER_FAILURE:
        { // No power or supply voltage low
          s_buf = FPSTR(text_no_power);
          botSendMessage(s_buf);
          break;
        }
      case CMD_SENSOR_ERROR:
        { // Sensor unit error (not connected, failed etc.)
          s_buf = FPSTR(text_sensor_error);
          botSendMessage(s_buf);
          break;
        }
#ifdef DEBUG_READ
      case CMD_DEBUG_PRINTOUT:
        {  bot.setTextMode(FB_MARKDOWN);
          uint16_t tmp;
          s_buf = '`';
          tmp = buf_rx[2] << 8 | buf_rx[3];
          s_buf += tmp;
          s_buf += FPSTR(text_blank);
          tmp = buf_rx[4] << 8 | buf_rx[5];
          s_buf += tmp;
          s_buf += FPSTR(text_blank);
          tmp = buf_rx[6] << 8 | buf_rx[7];
          s_buf += tmp;
          s_buf += FPSTR(text_blank);
          tmp = buf_rx[8] << 8 | buf_rx[9];
          s_buf += tmp;
          s_buf += FPSTR(text_blank);
          s_buf += buf_rx[10];
          s_buf += FPSTR(text_blank);
          s_buf += buf_rx[11];
          s_buf += FPSTR(text_blank);
          s_buf += buf_rx[12];
          s_buf += FPSTR(text_blank);
          tmp = buf_rx[13] << 8 | buf_rx[14];
          s_buf += tmp;
          s_buf += FPSTR(text_blank);
          tmp = buf_rx[15] << 8 | buf_rx[16];
          s_buf += tmp;
          s_buf += '`';
          botSendMessage(s_buf);
          bot.setTextMode(FB_TEXT);
          break;
        }
#endif        
      case CMD_MESSAGE_ACK:
        { // Short message delivery ack
          s_buf = F("Message ");
          s_buf += buf_rx[2] ? F("closed") : F("delivered");
          sys_suspended = !buf_rx[2];
          botSendMessage(s_buf);
          break;
        }
      case CMD_SYSTEM_INIT:    
        { // System init data, once on restart
          char c_buf[3];
          c_buf[2] = 0;
          wordToHEXchar(buf_rx[2], c_buf);
          s_buf = c_buf;
          wordToHEXchar(buf_rx[3], c_buf);
          s_buf += c_buf;
          wordToHEXchar(buf_rx[4], c_buf);
          s_buf += c_buf;
          wordToHEXchar(buf_rx[5], c_buf);
          s_buf += c_buf;
          wordToHEXchar(buf_rx[6], c_buf);
          s_buf += c_buf;
          botSendMessage(s_buf);
          break;
        }
      case CMD_EDC_EVENT:    
        { // External device control
          if (buf_rx[2] < 5) {         // Accept codes less than 10, drop others. Command #5 drops too, intentionally
          
            s_buf = F("EDC ");

            switch (buf_rx[2]) {
              case CMD_EDC_EVENT_RUN:
                {
                  s_buf += F("run");
                  break;
                }
              case CMD_EDC_EVENT_STOP:
                {
                  s_buf += F("stop");
                  break;
                }
              case CMD_EDC_EVENT_RESET:
                {
                  s_buf += F("reset");
                  break;
                }
              case CMD_EDC_EVENT_WAKE:
                { // Sync signal sent, occurs each time system awaken, do nothing. Can be used to control something.
                  s_buf += F("dummy");  
                  break;
                }
              default: break;
            }
            botSendMessage(s_buf);
          }
          break;
        }
      /*case 15:
        { // UART – EDC relay
          s_buf = buf_rx[2] ? FPSTR(text_ok) : FPSTR(text_failed);
          botSendMessage(s_buf);
          break;
        }*/
      case CMD_REFILL_COMPLETE:
        { // Tank refill complete
          s_buf = F("Tank refill complete");
          botSendMessage(s_buf);
          break;
        }
      case CMD_AUTO_FLUSH:
        { // Auto flush result
          sys_suspended = false;
          s_buf = F("Autoflush: ");
          s_buf += buf_rx[2] ? F("half") : F("full");
          botSendMessage(s_buf);
          break;
        }
      case CMD_SYSTEM_HALTED:
        { // Halt reasons
          s_buf = FPSTR(text_halted);
          switch (buf_rx[2]) {
            case CMD_SYS_HLT_SENSOR:
              {
                s_buf += F("Sensor");
                break;
              }
            case CMD_SYS_HLT_FLUSH:
              {
                s_buf += F("Flush");
                break;
              }
            case CMD_SYS_HLT_REFILL:
              {
                s_buf += F("Refill");
                break;
              }
            case CMD_SYS_HLT_TEST:
              {
                s_buf += F("Self-test");
                break;
              }
            case CMD_SYS_HLT_MANU:
              {
                s_buf += F("Manually");
                break;
              }
          }
          botSendMessage(s_buf);
          break;
        }
      case CMD_TEST_BEEP_BEEP:
        { // Beep Beep done, clear busy flag
          sys_suspended = false;
          break;
        }
      case CMD_SYSTEM_STATUS:
        {  // Runtime status printout
          String obj_stat_s = F("Status and stats\n\n");
          for (byte h = 0; h < 4; h++) {
            switch (buf_rx[h * 2 + 2]) {
              case 0:
                {
                  obj_stat_s += FPSTR(text_cat);
                  obj_stat_s += (buf_rx[h * 2 + 3]);
                  obj_stat_s += '\n';
                  break;
                }
              case 1:
                {
                  obj_stat_s += FPSTR(text_sit);
                  obj_stat_s += (buf_rx[h * 2 + 3]);
                  obj_stat_s += '\n';
                  break;
                }
              case 2:
                {
                  obj_stat_s += FPSTR(text_man);
                  obj_stat_s += (buf_rx[h * 2 + 3]);
                  obj_stat_s += '\n';
                  break;
                }
              case 3:
                {
                  obj_stat_s += FPSTR(text_ufo);
                  obj_stat_s += (buf_rx[h * 2 + 3]);
                  obj_stat_s += '\n';
                  break;
                }
            }
          }

          String fillTime_s = F("\nTime of");

          fillTime_s += FPSTR(text_tank_fill);
          
          // Convert received bytes to readable time string
          uint32_t fillTime_l = buf_rx[10] << 24 | buf_rx[11] << 16 | buf_rx[12] << 8 | buf_rx[13];

          if (fillTime_l < 900000) {
            byte minutes_ = fillTime_l / 60000;
            byte seconds_ = (fillTime_l % 60000) / 1000;
            fillTime_s += minutes_;
            fillTime_s += F(" m ");
            if (seconds_ < 10) fillTime_s += '0';
            fillTime_s += seconds_;
            fillTime_s += F(" s");
          } else {
            fillTime_s += FPSTR(text_no);
          }

          obj_stat_s += fillTime_s;

          s_buf = F("\n\nWDT:\t");
          s_buf += bitRead(buf_rx[14], 7) ? FPSTR(text_yes) : FPSTR(text_no);
          s_buf += F("\nSelf-test:\t");
          s_buf += bitRead(buf_rx[14], 6) ? FPSTR(text_on) : FPSTR(text_off);
          s_buf += F("\nMAA:\t");
          s_buf += bitRead(buf_rx[14], 5) ? FPSTR(text_on) : FPSTR(text_off);
          s_buf += F("\nMode:\t");
          s_buf += bitRead(buf_rx[14], 4) ? (bitRead(buf_rx[14], 5) ? F("ANY 3") : F("ANY 2")) : F("CAT");
          s_buf += F("\nECO:\t");
          s_buf += bitRead(buf_rx[14], 3) ? FPSTR(text_on) : FPSTR(text_off);
          s_buf += F("\nTank:\t");
          s_buf += bitRead(buf_rx[14], 2) ? F("full") : F("low");
          s_buf += F("\nToilet:\t");
          s_buf += bitRead(buf_rx[14], 1) ? F("Occupied") : F("free");
          s_buf += F("\nMovement:\t");
          s_buf += bitRead(buf_rx[14], 0) ? FPSTR(text_yes) : FPSTR(text_no);

          obj_stat_s += s_buf;

          botSendMessage(obj_stat_s);
          break;
        }
      case CMD_MRA_COMPLETE:
        { // Analysis has been completed, going to flush automatically
          sys_suspended = true;
          //botSendMessage(F("MRA completed"));
          break;
        }
      case CMD_NO_WATER_REFILL:
        { // Low water, waiting for refill
          sys_suspended = true;
          s_buf = F("Waiting for refill");
          botSendMessage(s_buf);
          break;
        }
      case CMD_SYSTEM_SLEEP:
        { // System gonna nap
          host_sleep = true;
          skip_msg = false;
          uint32_t m = millis();

          // Inevitable Telegram worship
          // Before send message to Telegram server, delay for a moment to make some tribute for Telegram server, and do not forget to answer handshake() request...
          while (millis() - m < 1500) {
            yield();
            // incoming handshake() respond routine, this signal 
            if (Serial.read() == CMD_IN_HANDSHAKE) {
              Serial.write(CMD_OUT_HANDSHAKE);
              uart_link_ok = true;
              activity_LED(ON);
              skip_msg = true;
              break;
            }
          }
          if (!skip_msg) {
            if (awaiting_text_input) {
              awaiting_text_input = false;
              bot.deleteMessage(msg_menu_id);
            } else { 
              if (!msg_menu_id) {
                s_buf = F("Zzz");
                botSendMessage(s_buf);
                sleep_time_mark = millis();
              } else {
                msg_menu_id = 0;
                sleep_time_mark = 0;
              }
            }
          }
          activity_LED(OFF);
          break;
        }
      case CMD_LEAKAGE_DETECTED:
        { // Leakage detection
        s_buf = F("Leakage?");
          botSendMessage(s_buf);
          break;
        }
      case CMD_MRA_RESULT:
        { // Recognition result 
          s_buf = F("Recognized ");
          switch (buf_rx[2]) {
            case CMD_MRA_RESULT_CAT:
              {
                s_buf += FPSTR(text_cat);
                break;
              }
            case CMD_MRA_RESULT_SIT:
              {
                s_buf += FPSTR(text_sit);
                break;
              }
            case CMD_MRA_RESULT_MAN:
              {
                s_buf += FPSTR(text_man);
                break;
              }
            case CMD_MRA_RESULT_UFO:
              {
                s_buf += FPSTR(text_ufo);
                break;
              }
          }
          s_buf += buf_rx[3];
          botSendMessage(s_buf);
          break;
        }
      case CMD_WAIT_FOR_REFILL:
        { // Low water, waiting for refill before sleep mode
          sys_suspended = false;
          s_buf = F("Wait for");
          s_buf += FPSTR(text_tank_fill);
          botSendMessage(s_buf);
          break;
        }
      case CMD_EVAL_REFILL_TIME:
        { // Evaluating refill time
          s_buf = F("Evaluating time of");
          s_buf += FPSTR(text_tank_fill);
          botSendMessage(s_buf);
          break;
        }
      case CMD_HALF_DELAY_ACK:
        { // Half flush delay setup ack
          s_buf = F("Half flush delay: ");
          s_buf += (10 * buf_rx[2]);
          s_buf += F(" ms");
          botSendMessage(s_buf);
          break;
        }
      case CMD_SYSTEM_CONFIG:
        { // System config data
          s_buf = F("`TBS50 ");
          s_buf += (buf_rx[2]);
          s_buf += F(" Slw65 ");
          s_buf += buf_rx[3];
          s_buf += F(" Sup10 ");
          s_buf += buf_rx[4];
          s_buf += F(" Tcs75 ");
          s_buf += buf_rx[5];
          s_buf += F("\nTmn12 ");
          s_buf += buf_rx[6];
          s_buf += F(" Tmx65 ");
          s_buf += buf_rx[7];
          s_buf += F(" TPg60 ");
          s_buf += buf_rx[8];
          s_buf += F(" Qf146 ");
          s_buf += buf_rx[9];
          s_buf += F("\nQf34 ");
          s_buf += buf_rx[10];
          s_buf += F(" Qd43 ");
          s_buf += buf_rx[11];
          s_buf += F(" Qd328 ");
          s_buf += buf_rx[12];
          s_buf += F(" Qvr39 ");
          s_buf += buf_rx[13];
          s_buf += F("\nTvr60 ");
          s_buf += buf_rx[14];

          s_buf += F(" SvcM ");
          s_buf += bitRead(buf_rx[15], 7);

          s_buf += F(" Diag ");
          s_buf += bitRead(buf_rx[15], 6);

          s_buf += F(" Dbug ");
          s_buf += bitRead(buf_rx[15], 5);

          s_buf += F("\nPaa0 ");
          s_buf += bitRead(buf_rx[15], 4);

          s_buf += F(" Beep ");
          s_buf += bitRead(buf_rx[15], 3);

          s_buf += F(" Eco ");
          s_buf += bitRead(buf_rx[15], 2);

          s_buf += F(" PiT ");
          s_buf += bitRead(buf_rx[15], 1);

          s_buf += F(" Rsrv ");
          s_buf += bitRead(buf_rx[15], 0);
          s_buf += '`';

          bot.setTextMode(FB_MARKDOWN);
          botSendMessage(s_buf);
          bot.setTextMode(FB_TEXT);
          bot.tickManual();
          break;
        }
      case CMD_SYS_ACKNOWLEDGE:
        { // dbg ctrl ack
          s_buf = buf_rx[2] ? F("ok") : F("err");
          botSendMessage(s_buf);
          break;
        }
      case CMD_MANUAL_FLUSH:
        { // Manual flush
          sys_suspended = false;
          s_buf = F("Manual flush");
          botSendMessage(s_buf);
          break;
        }
      case CMD_SYS_WAKE_INT:
        { // System wake up (movement or leakage detected)
          sys_suspended = false;
          if (!skip_msg and !awaiting_text_input) {
            // Inevitable Telegram server worship
            while (millis() - sleep_time_mark < 1500) {yield();}
            sleep_time_mark = 0;
            //
            s_buf = F("Awoke");
            botSendMessage(s_buf);
          }
          break;
        }
      case CMD_AUTO_FLUSH_RST:
        { // Auto flush manually cancelled
          sys_suspended = false;
          s_buf = FPSTR(text_cancel);
          botSendMessage(s_buf);
          break;
        }
      case CMD_SYS_WAKE_EXT:
        { // system awaken by command, the command message to be sent should be in TX buffer already 
          sys_suspended = false;
          s_buf = F("Awoke, command");
          if (!awaiting_text_input) botSendMessage(s_buf);
          // let the system get ready after sleep, delay should be within 750 - 1000 ms
          delay(800);
          UART_send_buffer();
          break;
        }
      case CMD_PULL_VALVE:
        { // Valve has been fixed
          s_buf = F("Try to fix valve");
          botSendMessage(s_buf);
          break;
        }
    }
    // Clear RX buffer
    memset(buf_rx, 0, 32);
  }

  // if restart_wifi flag, perform ESP restart routine
  if (restart_wifi) {
    bot.tickManual();
    s_buf = F("ESP restarting...");
    botSendMessage(s_buf);
    ESP.restart();
  }
}


/* Bot incoming messages handler routine */

void newMsg(FB_msg& msg) {
  // blink onboard LED on new message receive
  // remember, ESP inverts pin state: LOW means ON, HIGH means OFF
  uint8_t old_LED_stat = digitalRead(pin_LED);
  activity_LED(old_LED_stat);
  delay(25);
  activity_LED(!old_LED_stat);

  // ESP OTA firmware update over Telegram, top convenient thing
  if (msg.OTA && (msg.chatID == MY_CHAT_ID) && msg.fileName == OTA_FILE) {
    bot.tickManual();
    bot.update();
    return;
  }

  // generic String buffer, store incoming message
  String bmsg = msg.text;

  // Display WiFi network para
  if (bmsg == F("/ESP_svc")) {
    s_buf = F("Local IP: ");
    s_buf += WiFi.localIP().toString();
    s_buf += F("\nGW IP: ");
    s_buf += WiFi.gatewayIP().toString();
    s_buf += F("\nSSID: ");
    s_buf += WiFi.SSID();
    s_buf += F("\nRSSI: ");
    s_buf += WiFi.RSSI();
    s_buf += F(" dBm\n");
    s_buf += F("UART: ");
    wash_dishes();
    try_to_awake_host_if_it_sleeping(true);
    delay(200);
    UART_handshake();
    s_buf += uart_link_ok ? F("connected") : F("not connected");
    botSendMessage(s_buf);
    return;
  }

  // Show available commands
  if (bmsg == F("/start")) {
    s_buf = FPSTR(text_start);
    botSendMessage(s_buf);
    return;
  }

  // Begin ESP restart routine
  if (bmsg == F("/WiFi_restart")) {
    restart_wifi = true;
    return;
  }

  // In case the system busy doing operation with blocking
  if (sys_suspended) {
    s_buf = FPSTR(text_start);
    botSendMessage(s_buf);
    return;
  }

  // Host sleeping? Wake it up
  try_to_awake_host_if_it_sleeping();

  // If communication was disrupted, tell about it, and then try to restore
  if (!uart_link_ok) {
    s_buf = F("No answer, reconnecting...");
    botSendMessage(s_buf);
    try_to_awake_host_if_it_sleeping(true);
    delay(200);
    UART_handshake();
    s_buf = F("Now connected");
    if (uart_link_ok) botSendMessage(s_buf);
    return;
  }

  // Force uart_link_ok to false here, if next time it will be false still, that means communication problem
  uart_link_ok = false;

  activity_LED(OFF);

  // Just for code optimization reason. The first item is a message is its length, most commands is a single byte. So, set it once here.
  buf_tx[0] = 1;
  // Clean up previous command
  buf_tx[1] = 0;

  // Control debug data sending to UART interface.
  // This mode is for technical purposes.
  // Should be off in normal operation
  // Debug: enable
  if (bmsg.indexOf(F("/dbgo")) != -1) {
    buf_tx[0] = 3;
    buf_tx[1] = 43;
    buf_tx[2] = 94;
    buf_tx[3] = bmsg.indexOf(F("on")) != -1 ? 1 : 0;
  }

  // Show system menu (basic view). Inline menu as a variant.
  if (bmsg == F("/system")) {
    bot.inlineMenu(FPSTR(text_SYS_caption), FPSTR(text_SYS_menu));
    uart_link_ok = true;
    buf_tx[1] = 1;
  }

  // Handle system menu buttons action
  if (bmsg == FPSTR(text_SYS_caption)) {
    if (msg.data == F("Full flush")) {
      buf_tx[0] = 2;
      buf_tx[1] = 21;
      buf_tx[2] = 0;
    }
    if (msg.data == F("Half flush")) {
      buf_tx[0] = 2;
      buf_tx[1] = 21;
      buf_tx[2] = 1;
    }
    if (msg.data == F("Fix valve")) {
      buf_tx[1] = 159;
    }
    if (msg.data == F("Status")) {
      buf_tx[1] = 27;
    }
    if (msg.data == F("Config")) {
      buf_tx[1] = 42;
    }
    if (msg.data == F("Halt")) {
      buf_tx[0] = 2;
      buf_tx[1] = 23;
      buf_tx[2] = 4;
    }
    if (msg.data == F("Reset")) {
      buf_tx[1] = 24;
      s_buf = F("Host reset...");
      botSendMessage(s_buf);
      uart_link_ok = false;
      activity_LED(OFF);
    }
    if (msg.data == F("Default")) {
      buf_tx[1] = 26;
      s_buf = F("Restore defaults and reset");
      botSendMessage(s_buf);
      activity_LED(OFF);
    }
    if (msg.data.indexOf(F("Beep Beep")) != -1) {
      sys_suspended = true;
      buf_tx[1] = 25;
    }
  }

  // Setup half flush delay. Default value 600 ms.
  // Command pattern to set half delay to 750 ms looks like: "/halfdelay 750"
  // Mind the value to send (and received) should be divided on 10. E.g., to set delay to 750 ms, send 75 to the system.
  // New value should be within 0 < v < 201 (10..2000 ms)
  // Delay less than ~300..400 ms has no sense
  if (bmsg.indexOf(F("/halfdelay")) != -1) {
    buf_tx[0] = 2;
    buf_tx[1] = 41;
    buf_tx[2] = 0;
    if (bmsg.length() > 10) {
      s_buf = bmsg.substring(11);
      uint16_t del_val = s_buf.toInt() / 10;
      buf_tx[2] = (del_val > 0 && del_val < 201) ? del_val : 60;
    }
  }

  // Tiny pager. It will send text message to OLED display with beep until it been closed
  // Command example: "/sm Hello, world! How is it going today?"
  // If text length exceeds 48 characters, it will be trimmed to 48 
  // Tiny pager recognizes roman charset (UTF-8) only, this cannot be modified due to system memory limits

  if (bmsg.indexOf(F("/sm")) != -1 or awaiting_text_input) {
    
    // Cancel message input and return to main loop if menu was evoke previously
    if (bmsg == F("Enter message") and msg.data == F("cancel")) {
      bot.deleteMessage(msg_menu_id);
      awaiting_text_input = false;
      uart_link_ok = true;
      return;
    }

    // Show message menu and allow further text input
    if (bmsg.length() < 5 and !awaiting_text_input) {
      bot.deleteMessage(bot.lastUsrMsg());
      bot.inlineMenu(F("Enter message"), F("cancel"));
      msg_menu_id = bot.lastBotMsg();
      awaiting_text_input = true;
      uart_link_ok = true;
      return;
    }

    // For alternative message input mode, this delete first 4 characters
    if (!awaiting_text_input) bmsg.remove(0, 4);
    
    // Convert text to uint8_t array and then send it to host
    if (bmsg.length() > 48) bmsg.remove(48);
    bmsg.toCharArray((char*)buf_tx + 3, bmsg.length() + 1);
    buf_tx[0] = bmsg.length() + 2;
    buf_tx[1] = bmsg.length() > 3 ? 9 : 0;
    buf_tx[2] = 1; // service byte, always set to 1 

    awaiting_text_input = false;
    if (msg_menu_id) {
      bot.deleteMessage(msg_menu_id);
      msg_menu_id = 0;
    }
  }

  // Drop zero commands
  if (!buf_tx[1]) {
    s_buf = F("Unrecognized: ");
    s_buf += bmsg;
    botSendMessage(s_buf);
    uart_link_ok = true;
    return;
  }

  //Transmit ready data to system
  UART_send_buffer();
}

// BONUS: Int to HEX converter subroutine
void wordToHEXchar(word num, char* char_buf) {
  static const char* HEXVAL PROGMEM = "0123456789ABCDEF";
  byte k = 1;
  word a = num;
  word b = a % 16;
  char_buf[k] = HEXVAL[b];

  while (bool(k)) {
    yield();
    a = a / 16;
    b = a % 16;
    char_buf[--k] = HEXVAL[b];
  }
}

void try_to_awake_host_if_it_sleeping(bool forced) {
  if (host_sleep or !uart_link_ok or forced) {
    Serial.end();
    delay(5);
    pinMode(pin_UART_TX, OUTPUT);
    digitalWrite(pin_UART_TX, LOW);
    delay(5);
    digitalWrite(pin_UART_TX, HIGH);
    delay(5);
    Serial.begin(UART_BAUD);
  }
}

// UART transmission
void UART_send_buffer() {
  if (!host_sleep) {
    Serial.write(buf_tx, buf_tx[0] + 1);
    Serial.flush();
  }
}

void UART_handshake() {
  uart_link_ok = false;
  activity_LED(uart_link_ok);
  Serial.write(CMD_IN_HANDSHAKE);
  uint32_t m = millis();
  while (millis() - m < 300) {
    yield();
    if (Serial.read() == CMD_OUT_HANDSHAKE) {
      uart_link_ok = true;
      break;
    }
  }
  // control ESP onboard LED
  activity_LED(uart_link_ok);
}

void WiFi_connect() {
  WiFi.begin(WiFi_SSID, WiFi_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() > 20000) ESP.restart();
    delay(200);                             //Do not remove this line to avoid exception with infinite boot loop. You can use yield() as well instead delay()
  }
}

// Resetting essential variables 
void wash_dishes() {
  Serial.flush();
  while (Serial.available()) Serial.read();
  memset(buf_rx, 0, 32);
  memset(buf_tx, 0, 32);
  sys_suspended = false;
  host_sleep = false;
}

// bot.sendMessage() wrapper with onboard LED control
void botSendMessage(String& msg) {
//  uint8_t old_LED_stat = !digitalRead(pin_LED);
//  activity_LED(!old_LED_stat);
  bot.sendMessage(msg, MY_CHAT_ID);
//  activity_LED(old_LED_stat);
}