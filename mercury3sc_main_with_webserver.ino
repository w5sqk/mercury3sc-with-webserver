// 6/09/2023 w5sqk
// v1.0 
// Initial merge of webserver examples code by Khoi Hoang (noted below) and mercury3sc code

// initial code working
// ip address via dhcp 
// webdisplay @ http://<ip address> port 80
// displays same amplifier values as K9SUL's serial console 't' command, refreshes every 1 seconds, future effort to move to socket based 
//    and push updates as amplifier updates Nextion dispay, may need to throttle updates, work in progress
//
// Initial code summary:
//    renamed serial console print outputs to match mercury3sc  serial console prints.
//    moved code supporting various arduino cpus to boardDefs module- code otherwise unchanged
//    moved web initializations and print outputs to WebCheck. Added simple one to one print outputs to match serial 't" outputs.
//    data varibles exceed nano 2k memery, moved to mega 2560 for code development
//
//

/****************************************************************************************************************************
  WebServer.ino

  Ethernet_Generic is a library for the W5x00 Ethernet shields trying to merge the good features of
  previous Ethernet libraries

  Built by Khoi Hoang https://github.com/khoih-prog/Ethernet_Generic
 *****************************************************************************************************************************/
/*
   The Arduino board communicates with the shield using the SPI bus. This is on digital pins 11, 12, and 13 on the Uno
   and pins 50, 51, and 52 on the Mega. On both boards, pin 10 is used as SS. On the Mega, the hardware SS pin, 53,
   is not used to select the Ethernet controller chip, but it must be kept as an output or the SPI interface won't work.
*/

/**
 * mercury3sc: Mercury IIIS remote controller
 * Copyright (c) 2023 Kihwal Lee, K9SUL
 * 
 * It acts as man in the middle for the existing serial connection
 * between the internal Arduino Nano and the Nextion LCD.  The USB
 * serial port is used for control and status reporting.
 * 
 * It uses a HW serial on and AltSoftSerial because Teensy 2.0 has
 * only one HW serial port.
 * 
 * HW serial pins: 8(tx), 7(rx) - Serial1 - Nextion
 * Alt SW serial pins: 9(tx), 10(rx) - SW Serial - Merc III Nano
 *
 * PIN_B0 is connected to the gate of a 2N7000 for power on/off control
 * EEPROM address 0 stores the beep setting.
 */

//
// Initial modification to K9SUL's code to handle two char command sequences.
// Changes principally in module updateState & printStatus
// 
// w5sqk 05/27/2023
//

#define M3S_BAUD 57600         // Mercury IIIS's internal baud rate
#define M3S_BUFF_SIZE 64      // Internal receive buffer size
#define M3S_LED 11             // LDE pin
#define M3S_PCTL PIN_B0        // Power on/off io pin
#define M3S_ATTN PIN_B1        // Attenuator relay control (K9SUL custom)
#define M3S_ST_WINDOW 3        // outlier drop window size
#define M3S_ST_VAL    4        // M3S_ST_WINDOW + 1

#include <string.h>
// #include <AltSoftSerial.h>
#include <EEPROM.h>

//i2c to dual uart
#include <DFRobot_IICSerial.h>

DFRobot_IICSerial iicSerial3(Wire, /*subUartChannel =*/SUBUART_CHANNEL_1,/*IA1 = */1,/*IA0 = */1);//Construct UART1
DFRobot_IICSerial iicSerial2(Wire, /*subUartChannel =*/SUBUART_CHANNEL_2, /*IA1 = */1,/*IA0 = */1);//Construct UART2

// #define LCDSerial Serial1      // serial port for communicating with the Nextion LCD
// AltSoftSerial CTLSerial;       // serial port for communicating with the onboad Arduino Nano

#define LCDSerial iicSerial2
#define CTLSerial iicSerial3

const int PIN_B0 = 2;
const int PIN_B1 = 3;
char buff[M3S_BUFF_SIZE];      // receiver buffer
char outb[128];                // send buffer
boolean dir = true;            // comm direction. Read from nextion when true.
boolean beep = false;           // whether to send a beep or not.
boolean debug = false;         // verbose output
const char M3S_TERM = 0xff;
long loop_count;

// Variables to keep track of the amp state. Each state keeps a history of the length
// defined by M3S_ST_WINDOW. If a newly added value is an outlier, it will still be
// recorded, but won't be relayed to the LCD.
//
// vol[M3S_ST_WINDOW] contains the curent head index
// vol[M3S_ST_WINDOW + 1] contains last known good value
int vol[M3S_ST_WINDOW+2], cur[M3S_ST_WINDOW+2], swr[M3S_ST_WINDOW+2];
int ref[M3S_ST_WINDOW+2], pwr[M3S_ST_WINDOW+2], tmp[M3S_ST_WINDOW+2];
boolean transmit = false;

#include "defines.h"

int reqCount = 0;                // number of requests received

EthernetServer server(80);

//#include WString.h:#define F(string_literal) (reinterpret_cast<const __FlashStringHelper *>(PSTR(string_literal)))
// Prints to the USB serial port. Used to dump the captured commands
// Control characters are printed in hex.
void printUSB(char* buff, int len, boolean lcd) {
  if (lcd) {
    Serial.print("> ");
  } else {
    Serial.print("< ");
  }

  for (int i = 0; i < len; i++) {
    char c = buff[i];
    if (c > 31 && c < 128) {
      Serial.print(c);
    } else {
      // For non printable chars.
      Serial.print("[");
      Serial.print((uint8_t)c, HEX);
      Serial.print("]");
    }
  }
  Serial.println(" ");
}

void printHelp() {
  Serial.println("BPF selection: a 160m, b 80m, c 60/40, d 30/20, e 17/15, f 12/10, g 6, h auto");
  Serial.println("ANT selection: 1, 2, 3");
  Serial.println("Reset: r");
  Serial.println("Fan  : j auto, k max");
  Serial.println("Beep : s to toggle");
  Serial.println("Status: t for human-readable format, u for short form");
  Serial.println("Verbose: v to toggle");
  Serial.println("Power on/off: p/q (normally off)");
  Serial.println("Attenuator on/off: y/x (normally on)");
}

// Send a command to the nano controller
void sendCtrlMsg(const char* msg) {
  sprintf(outb,"%s%c%c%c", msg, M3S_TERM, M3S_TERM, M3S_TERM);
  CTLSerial.print(outb);
}

// send a command to the LCD
void sendLcdMsg(const char* msg) {
  sprintf(outb,"%s%c%c%c", msg, M3S_TERM, M3S_TERM, M3S_TERM);
  LCDSerial.print(outb);
}

// Does it end with the terminal sequence, 0xff 0xff 0xff?
// The bit pattern is 0xff, which shouldn't be confused with the value of
// a particular type.  E.g. 0xff in char is -1. 0xff in int is 255.
// Be careful with type casting and comparisons.
boolean term_seq(char* data, int len) {
  // false if the input is too short
  if (len < 3)
    return false;

  // examine the last three bytes.
  if (data[len-1] == M3S_TERM && data[len-2] == M3S_TERM && data[len-3] == M3S_TERM) {
    return true;
  } else {
    return false;
  }
}

// Reset the Nextion LCD after transitioning from TX to RX. This is to clear
// any inconsistent updates during TX.
void resetLcdState() {
  sendLcdMsg("s.val=10");
  sendLcdMsg("c.val=0");
  sendLcdMsg("p.val=0");
  sendLcdMsg("r.val=0");
  swr[M3S_ST_WINDOW + 1] = 10;
  pwr[M3S_ST_WINDOW + 1] = 0;
  cur[M3S_ST_WINDOW + 1] = 0;
  ref[M3S_ST_WINDOW + 1] = 0;
}

// Add a new value to the array.
boolean addVal(int st[], int val) {
  boolean isGoodVal = true;
  
  int cidx = st[M3S_ST_WINDOW]; // last element is used for current head index
  // is it a good value to report?
  for (int i = 0; i < M3S_ST_WINDOW; i++) {
    if (i == cidx)
      continue; // this is the oldest val that will be replaced.
    int diff = (st[i] > val) ? (st[i] - val) : (val - st[i]);
    // It is an outliner if more than +/- 25%
    if (diff > st[i]/4) {
      isGoodVal = false; // this is an outlier
      break;
    }
  }
  // save the value and update the index
  st[cidx] = val;
  st[M3S_ST_WINDOW] = (cidx + 1) % M3S_ST_WINDOW;
  if (isGoodVal) {
    st[M3S_ST_VAL] = val;
  }
  return isGoodVal;
}

void addValNoCheck(int st[], int val) {
  // save the value and update the index
  int cidx = st[M3S_ST_WINDOW];
  st[cidx] = val;
  st[M3S_ST_WINDOW] = (cidx + 1) % M3S_ST_WINDOW;
  st[M3S_ST_VAL] = val;
}

// Parse and update the internal state if needed.
// returns true if the record is to be reported.
boolean updateState(char* buff, int len) {
  // Is it tx/rx mode indicator? oa.picc=1 (rx), oa.picc=2 (tx)
  if (len == 12) {
    if (!strncmp(buff, "oa.picc=1", 9)) {
      transmit = false;
      digitalWrite(M3S_LED, LOW);
      // Make sure display is reset correctly. During transmit, the high traffic
      // can cause random byte drops. If it happens at the end of transmission,
      // the display might be left in inconsistent state.
      sendLcdMsg("tsw 255,1");
      resetLcdState();
    } else if (!strncmp(buff, "oa.picc=2", 9)) {
      transmit = true;
      digitalWrite(M3S_LED, HIGH);
    } else if (!strncmp(buff, "tsw 255,1", 9) && transmit) {
      // "oa.picc=1" is always immediately followed by "tsw 255,1". If we see this
      // and still in transmit mode, it must mean "oa.picc=1" was lost. 
      transmit = false;
      digitalWrite(M3S_LED, LOW);
      sendLcdMsg("oa.picc=1");
      resetLcdState();
    }
  }
  
  // Is it in the form of "x.val="?
  
  // determine if 1 or 2 char cmd
      int dotPos = -1;
      if (buff[1] == '.') {
          dotPos = 1;
      } 
      if (buff[2] == '.') {
          dotPos = 2;
      }
  
  // offset position of buff test based on whether one or two char cmd
  if (buff[dotPos] == '.' && buff[dotPos+1] == 'v' && buff[dotPos+2] == 'a' && buff[dotPos+3] == 'l' && buff[dotPos+4] == '=') {
    if (buff[dotPos+5] == M3S_TERM) { // 0xff terminator
      // no data after "=".
      return false; // discard without updating
    }

    // parse the integer string
    buff[len-3] = '\0'; // temporarily null terminated

    //parse based on whether one or two char cmd
    int val;
    switch (dotPos) {
        case 1:
           val = atoi(buff + 6); 
           break;
        case 2:
           val = atoi(buff + 7); 
           break;
      }
    buff[len-3] = M3S_TERM; // restore 0xff
    switch(buff[0]) {
      case 'v':
        // skip bad voltages.
        // consider only 3 digit reports (> 10.0V) are valid.
        if (val < 100) return false;
        if (transmit)
          return addVal(vol, val);
        addValNoCheck(vol, val);
        break;
      case 'c':
        if (transmit)
          return addVal(cur, val);
        addValNoCheck(cur, val);
        break;
      case 's':
        if (val < 10) return false;
        if (transmit)
          return addVal(swr, val);
        addValNoCheck(swr, val);
        break;
      case 'r':
        if (transmit)
          return addVal(ref, val);
        addValNoCheck(ref, val);
        break;
      case 'p':
        if (transmit)
          return addVal(pwr, val);
        addValNoCheck(pwr, val);
        break;
      case 't':
        if (transmit)
          return addVal(tmp, val);
        addValNoCheck(tmp, val);
        break;
      default:
        break;
     }
     return true;
  }
  return true;
}

void printStatus(boolean human_readable) {
  if (human_readable) {
    Serial.print("Output Power   : ");
    Serial.println(pwr[M3S_ST_VAL]);
    Serial.print("Reflected Power: ");
    Serial.println(ref[M3S_ST_VAL]);
    Serial.print("SWR : ");
    Serial.println(swr[M3S_ST_VAL]);
    Serial.print("Drain Voltage  : ");
    Serial.println(vol[M3S_ST_VAL]);
    Serial.print("Drain Current  :");
    Serial.println(cur[M3S_ST_VAL]);
    Serial.print("Temperature(C) : ");
    Serial.println(tmp[M3S_ST_VAL]);
  } else {
    sprintf(outb, "%d %d %d %d %d %d", pwr[M3S_ST_VAL], ref[M3S_ST_VAL],
        swr[M3S_ST_VAL], vol[M3S_ST_VAL], cur[M3S_ST_VAL], tmp[M3S_ST_VAL]);
    Serial.println(outb);
  }
}

// Update the band display on LCD.
//
// q6.picc to q12.picc are the thin lines under the each band button.
// The active one is set to 2 and 1 turns it off.  This is used in the
// auto switching mode.
//
// band0.val to band6.val are for the band buttons. 1 to select, 0 for off.
// band7.val is for the auto button, which is turned off whenever a band is
// selected by this controller.
void setLcdBand(int b) {
  // clear auto-selected band marker
  for (int i = 6; i <= 12; i++) {
    sprintf(outb, "q%d.picc=1%c%c%c", i, M3S_TERM, M3S_TERM, M3S_TERM);
    LCDSerial.print(outb);
  
  }
  // Select the manual band button
  for (int i = 0; i <= 7; i++) {
    sprintf(outb, "band%d.val=%d%c%c%c", i, (i==b) ? 1:0 ,M3S_TERM, M3S_TERM, M3S_TERM);
    LCDSerial.print(outb);
  }
}



void setup()
{
  SerialDebug.begin(115200); 
  // init webserver
  while (!Serial && millis() < 5000);
  //Serial.println(“Hello World”);
  SerialDebug.print("\nStarting WebServer on ");
  SerialDebug.print(BOARD_NAME);
  SerialDebug.print(" with ");
  SerialDebug.println(SHIELD_TYPE);
  SerialDebug.println(ETHERNET_GENERIC_VERSION);
  boardDefs (); // check for specific arduino cpu 
  //end init webserver
  Wire.setClock(400000); // set i2c bus speed fast
//  Serial.begin(115200); // USB serial output. the speed has no meaning.
  LCDSerial.setTimeout(1); // 1ms timeout
  LCDSerial.begin(M3S_BAUD);

  CTLSerial.setTimeout(1);
  CTLSerial.begin(M3S_BAUD);

  pinMode(M3S_PCTL, OUTPUT);  // amp power control
  pinMode(M3S_ATTN, OUTPUT);
  pinMode(M3S_LED,  OUTPUT);

  digitalWrite(M3S_LED,  LOW); // turn on the led
  digitalWrite(M3S_PCTL, LOW);  // amp off
  digitalWrite(M3S_ATTN, LOW);  // attn on

  if (EEPROM.read(0) == 0x30) {
    beep = false;
  }
  if (EEPROM.read(1) == 0x30) {
    debug = true;
  }

  loop_count = 0;

  // init the state storage.
  for (int i = 0; i < M3S_ST_WINDOW + 2; i++) {
    vol[i] = cur[i] = swr[i] = ref[i] = pwr[i] = tmp[i] = 0;
  }
}

void loop() {
  int c;
  int idx;
  unsigned long t;
  boolean terminated = false;

  // read one command at a time.
  idx = 0;
  t = millis();
  webCheck(); // check for client connection and print output if connected.
  while (1) {
    // dir tells it to read from LCD or the controller. It alternates between
    // the two unless there are more data readily available in the current port.
    // This is happens a lot when transmitting.
    c = (dir) ? LCDSerial.read() : CTLSerial.read();

    if (c != -1) {
      if (idx == 0 && (char)c == M3S_TERM) {
        return;
      }
      buff[idx++] = (char)c;
      if (buff[idx-1] == M3S_TERM) {
        // terminating sequence started. Add two more 0xff.
        buff[idx++] = M3S_TERM;
        buff[idx++] = M3S_TERM;
        // now skip up to two 0xff in the stream.
        c = (dir) ? LCDSerial.peek() : CTLSerial.peek();
        if ((char)c == M3S_TERM) {
          c = (dir) ? LCDSerial.read() : CTLSerial.read();
          c = (dir) ? LCDSerial.peek() : CTLSerial.peek();
          if ((char)c == M3S_TERM) {
            c = (dir) ? LCDSerial.read() : CTLSerial.read();
          }
        }
        terminated = true;
        break;
      }
    }
    // timeout, buffer full, or nothing read.
    if (idx == 0 || (millis() - t) > /*10*/ 70 || idx == M3S_BUFF_SIZE) {
      // Commands are much shorter than the buffer. If the buffer is full, it
      // means there is corruption/drop. In 10ms, about 60 chars can be sent at 57.6kbps.
      // A timeout means the terminating sequence will never come.  It is better to simply
      // drop it.
      break;
    }
  }

  // Relay, process and print the received command
  if (idx > 0 && terminated) {
    if (dir) {
      // We read from the LCD. Write it to the controller.
      int ecode = 0x1a;
      if (buff[0] == (char)ecode)
        return;
      CTLSerial.write(buff, idx);
    } else {
      if (updateState(buff, idx))
        LCDSerial.write(buff, idx);
    }

    if (debug) {
      printUSB(buff, idx, dir);
    }
  }

  // intelligently switch between the sources. If the current source has
  // more data to read, stay with the source.
  loop_count++; // starvation prevention
  if (dir && !LCDSerial.available()) {
    loop_count = 0;
    dir = false;
  } else if (!dir && (!CTLSerial.available() || loop_count > 10)) {
    loop_count = 0;
    dir = true;
  }

  // External command processing.
  // BPF selection: a 160, b 80, c 40, d 20, e 15, f 10, g 6, h auto
  // Ant selection: 1, 2, 3
  // reset: r
  // fan: j auto, k max
  // beep: s to toggle
  // status: t for human-readable format, u for short form
  // Verbose: v to toggle
  // power on/off: p/q (normally off)
  // attn on/off: y/x (normally on)
  //
  // The ant is automatically set after a band switch. If a custom ant port
  // needs to be set, be sure to select an ant after setting the band.
  if (Serial.available()) {
    c = Serial.read();
    if (c == -1) {
      return;
    }

    if (beep && c != 't' && c != 'u' && c != 'v')
      sendCtrlMsg("psound");

    switch(c) {
      // BPF selection
      case 'a':
        sendCtrlMsg("pdia=160");
        setLcdBand(0);
        break;
      case 'b':
        sendCtrlMsg("pdia=80");
        setLcdBand(1);
        break;
      case 'c':
        sendCtrlMsg("pdia=40");
        setLcdBand(2);
        break;
      case 'd':
        sendCtrlMsg("pdia=20");
        setLcdBand(3);
        break;
      case 'e':
        sendCtrlMsg("pdia=15");
        setLcdBand(4);
        break;
      case 'f':
        sendCtrlMsg("pdia=10");
        setLcdBand(5);
        break;
      case 'g':
        sendCtrlMsg("pdia=6");
        setLcdBand(6);
        break;
      case 'h':
        setLcdBand(7);
        sendCtrlMsg("pdia=255");
        break;
      case 'i':
        printHelp();
        break;

      // power on
      case 'p':
        digitalWrite(M3S_PCTL, HIGH);
        break;
      // power off
      case 'q':
        digitalWrite(M3S_PCTL, LOW);
        break;
        
      // reset
      case 'r':
        sendCtrlMsg("preset_main");
        break;

      // toggle beep
      case 's':
        beep = !beep;
        if (beep) {
          EEPROM.write(0, 0x00);
        } else {
          EEPROM.write(0, 0x30);
        }
        break;

      // status in human readable form
      case 't':
        printStatus(true);
        break;
        
      // raw status data
      case 'u':
        printStatus(false);
        break;

      // toggle debug
      case 'v':
        debug = !debug;
        Serial.print("Verbose mode ");
        Serial.println(debug ? "on":"off");
        if (debug) {
          EEPROM.write(1, 0x30);
        } else {
          EEPROM.write(1, 0x00);
        }
        break;

      // attn off
      case 'x':
        digitalWrite(M3S_ATTN, HIGH);
        break;
      // attn on
      case 'y':
        digitalWrite(M3S_ATTN, LOW);
        break;

      // antenna selection
      case '1':
        sendCtrlMsg("ponant1");
        sendLcdMsg("ant1.val=1");
        sendLcdMsg("ant2.val=0");
        sendLcdMsg("ant3.val=0");
        break;
      case '2':
        sendCtrlMsg("ponant2");
        sendLcdMsg("ant1.val=0");
        sendLcdMsg("ant2.val=1");
        sendLcdMsg("ant3.val=0");
        break;
      case '3':
        sendCtrlMsg("ponant3");
        sendLcdMsg("ant1.val=0");
        sendLcdMsg("ant2.val=0");
        sendLcdMsg("ant3.val=1");
        break;

      // fan speed. LCD update is done by the controller.
      case 'j':
        sendCtrlMsg("pfanmin");
        break;
      case 'k':
        sendCtrlMsg("pfanmax");
        break;

      default:
        break;
    }
  }
}

void boardDefs () {
  #if (USING_SPI2)
  #if defined(CUR_PIN_MISO)
    ETG_LOGWARN("Default SPI pinout:");
    ETG_LOGWARN1("MOSI:"), CUR_PIN_MOSI);
    ETG_LOGWARN1("MISO:"), CUR_PIN_MISO);
    ETG_LOGWARN1("SCK:"),  CUR_PIN_SCK);
    ETG_LOGWARN1("SS:"),   CUR_PIN_SS);
    ETG_LOGWARN("=========================");
  #endif
  #else
    ETG_LOGWARN("Default SPI pinout:");
    ETG_LOGWARN1(("MOSI:"), MOSI);
    ETG_LOGWARN1(("MISO:"), MISO);
    ETG_LOGWARN1(("SCK:"),  SCK);
    ETG_LOGWARN1(("SS:"),   SS);
    ETG_LOGWARN("=========================");
  #endif

  #if defined(ESP8266)
    // For ESP8266, change for other boards if necessary
  #ifndef USE_THIS_SS_PIN
  #define USE_THIS_SS_PIN   D2    // For ESP8266
  #endif

    ETG_LOGWARN1("ESP8266 setCsPin:"), USE_THIS_SS_PIN);

    // For ESP8266
    // Pin                D0(GPIO16)    D1(GPIO5)    D2(GPIO4)    D3(GPIO0)    D4(GPIO2)    D8
    // Ethernet_Generic   X                 X            X            X            X        0
    // D2 is safe to used for Ethernet_Generic libs
    //Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (USE_THIS_SS_PIN);

  #elif defined(ESP32)

    // You can use Ethernet.init(pin) to configure the CS pin
    //Ethernet.init(10);  // Most Arduino shields
    //Ethernet.init(5);   // MKR ETH shield
    //Ethernet.init(0);   // Teensy 2.0
    //Ethernet.init(20);  // Teensy++ 2.0
    //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
    //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet

  #ifndef USE_THIS_SS_PIN
  #define USE_THIS_SS_PIN   5   //22    // For ESP32
  #endif

    ETG_LOGWARN1("ESP32 setCsPin:"), USE_THIS_SS_PIN);

    // Must use library patch for Ethernet, EthernetLarge libraries
    // ESP32 => GPIO2,4,5,13,15,21,22 OK with Ethernet, Ethernet2, EthernetLarge
    // ESP32 => GPIO2,4,5,15,21,22 OK with Ethernet3

    //Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (USE_THIS_SS_PIN);

  #elif ETHERNET_USE_RPIPICO

    pinMode(USE_THIS_SS_PIN, OUTPUT);
    digitalWrite(USE_THIS_SS_PIN, HIGH);

    // ETHERNET_USE_RPIPICO, use default SS = 5 or 17
  #ifndef USE_THIS_SS_PIN
  #if defined(ARDUINO_ARCH_MBED)
  #define USE_THIS_SS_PIN   17     // For Arduino Mbed core
  #else
  #define USE_THIS_SS_PIN   17    // For E.Philhower core
  #endif
  #endif

    ETG_LOGWARN1("RPIPICO setCsPin:"), USE_THIS_SS_PIN);

    // Must use library patch for Ethernet, EthernetLarge libraries
    // For RPI Pico using Arduino Mbed RP2040 core
    // SCK: GPIO2,  MOSI: GPIO3, MISO: GPIO4, SS/CS: GPIO5
    // For RPI Pico using E. Philhower RP2040 core
    // SCK: GPIO18,  MOSI: GPIO19, MISO: GPIO16, SS/CS: GPIO17
    // Default pin 5/17 to SS/CS

    //Ethernet.setCsPin (USE_THIS_SS_PIN);
    Ethernet.init (USE_THIS_SS_PIN);

  #else   //defined(ESP8266)
    // unknown board, do nothing, use default SS = 10
  #ifndef USE_THIS_SS_PIN
  #define USE_THIS_SS_PIN   10    // For other boards
  #endif

  #if defined(BOARD_NAME)
    ETG_LOGWARN3(("Board :"), BOARD_NAME, (", setCsPin:"), USE_THIS_SS_PIN);
  #else
    ETG_LOGWARN1(("Unknown board setCsPin:"), USE_THIS_SS_PIN);
  #endif

    // For other boards, to change if necessary
    Ethernet.init (USE_THIS_SS_PIN);

  #endif    // defined(ESP8266)

    // start the ethernet connection and the server:
    // Use DHCP dynamic IP and random mac
    uint16_t index = millis() % NUMBER_OF_MAC;
    // Use Static IP
    //Ethernet.begin(mac[index], ip);
    Ethernet.begin(mac[index]);

    //SPIClass SPI2(HSPI);
    //Ethernet.begin(mac[index], &SPI2);

    // Just info to know how to connect correctly
    // To change for other SPI
  #if defined(CUR_PIN_MISO)
    ETG_LOGWARN("Currently Used SPI pinout:");
    ETG_LOGWARN1("MOSI:"), CUR_PIN_MOSI);
    ETG_LOGWARN1("MISO:"), CUR_PIN_MISO);
    ETG_LOGWARN1("SCK:"),  CUR_PIN_SCK);
    ETG_LOGWARN1("SS:"),   CUR_PIN_SS);
    ETG_LOGWARN("=========================");
  #else
    ETG_LOGWARN("Currently Used SPI pinout:");
    ETG_LOGWARN1(("MOSI:"), MOSI);
    ETG_LOGWARN1(("MISO:"), MISO);
    ETG_LOGWARN1(("SCK:"),  SCK);
    ETG_LOGWARN1(("SS:"),   SS);
    ETG_LOGWARN("=========================");
  #endif

    SerialDebug.print("Using mac index = ");
    SerialDebug.println(index);

    SerialDebug.print("Connected! IP address: ");
    SerialDebug.println(Ethernet.localIP());

    if ( (Ethernet.getChip() == w5500) || (Ethernet.getChip() == w6100) || (Ethernet.getAltChip() == w5100s) )
    {
      if (Ethernet.getChip() == w6100)
        SerialDebug.print("W6100 => ");
      else if (Ethernet.getChip() == w5500)
        SerialDebug.print("W6100 => ");
      else
        SerialDebug.print("W5100S => ");
      
      SerialDebug.print("Speed: ");
      SerialDebug.print(Ethernet.speedReport());
      SerialDebug.print(", Duplex: ");
      SerialDebug.print(Ethernet.duplexReport());
      SerialDebug.print(", Link status: ");
      SerialDebug.println(Ethernet.linkReport());
  }

  // start the web server on port 80
  server.begin();

}

void webCheck () {

   // listen for incoming clients
  EthernetClient client = server.available();

  if (client)
  {
    SerialDebug.println("New client");
    // an http request ends with a blank line
    bool currentLineIsBlank = true;

    while (client.connected())
    {
      if (client.available())
      {
        char c = client.read();
        SerialDebug.write(c);

        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank)
        {
          SerialDebug.println("Sending response");

          // send a standard http response header
          // use \r\n instead of many println statements to speedup data send
          client.print(
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Connection: close\r\n"  // the connection will be closed after completion of the response
            "Refresh: 1\r\n"        // refresh the page automatically every 20 sec
            "\r\n");
          client.print("<!DOCTYPE HTML>\r\n");
          client.print("<html>\r\n");
          client.print("<h2>Hello World from ");
          client.print(BOARD_NAME);
          client.print("!</h2>\r\n");
          client.print("Requests received: ");
          client.print(++reqCount);
          client.print("<br>\r\n");
          /*client.print("Analog input A0: ");
          client.print(analogRead(0));
          client.print("<br>\r\n");
          client.print("</html>\r\n"); */
          /*client.print("Analog input A0: ");
          client.print(analogRead(0)); */
          client.print("<br>\r\n");
          client.print("</html>\r\n"); 
          
          client.print("Output Power: ");
          client.print(pwr[M3S_ST_VAL]);
          client.print("<br>\r\n");
          client.print("Reflected Power: ");
          client.print(ref[M3S_ST_VAL]);
          client.print("<br>\r\n");
          client.print("SWR: ");
          client.print(swr[M3S_ST_VAL]);
          client.print("<br>\r\n");
          client.print("Drain Voltage: ");
          client.print(vol[M3S_ST_VAL]);
          client.print("<br>\r\n");
          client.print("</html>\r\n");
          client.print("Drain Current: ");
          client.print(cur[M3S_ST_VAL]);
          client.print("<br>\r\n");
          client.print("</html>\r\n");
          client.print("Temp (C): ");
          client.print(tmp[M3S_ST_VAL]);
          client.print("<br>\r\n");
          client.print("</html>\r\n");
          break;
        }

        if (c == '\n')
        {
          // you're starting a new line
          currentLineIsBlank = true;
        }
        else if (c != '\r')
        {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }

    // give the web browser time to receive the data
    delay(10);

    // close the connection:
    client.stop();
    SerialDebug.println("Client disconnected");
  }
}


