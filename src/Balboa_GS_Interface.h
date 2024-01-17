#ifndef Balboa_GS_Interface_h
#define Balboa_GS_Interface_h

#include <Arduino.h>


const byte displayDataBufferSize             = 74;     // Size of display data buffer
const byte displayDataBits                  = 71;     // 0-70 bits length of display data within a cycle
const byte buttonDataBits                   = 3;      // 0-3 bits length of button data within a cycle
const byte totalDataBits                    = 75;     // 0-73 total number of pulses within a cycle
const unsigned int durationNewCycle         = 5000;   // How many microsecounds to detect new cycle if no interrupt occurs 
const unsigned long buttonPressTimerMillis  = 500;  // Timer in milliseconds between update temperature button presses 


class BalboaInterface {

  public:
  
  BalboaInterface(byte setClockPin, byte setReadPin, byte setWritePin);
  
  // Interface control
  void begin();                       // Initializes the stream output to Serial by default
  bool loop();                                    // Returns true if valid data is available
  void stop();                                    // Disables the clock hardware interrupt 
  void resetStatus();                             // Resets the state of all status components as changed for sketches to get the current status  
  void updateTemperature(float Temperature);    // Function to set the water temperature  

  // Status tracking
  bool displayBit0;
  float waterTemperature;           // Water temperatur 
  float setTemperature;             // The wanted set temperature   
  String LCD_display;               // The text shown on display 
  bool displayButton;               // Temp up/down button pressed
  bool displayBit29;                // Still unknown functionality, if at all used!
  bool displayBit30;                // Still unknown functionality, if at all used!
  bool displayBit31;                // Standard Mode activated or not
  bool displayBit32;                // Still unknown functionality, if at all used!
  bool displayBit33;                // Still unknown functionality, if at all used!  
  bool displayBit34;                // Still unknown functionality, if at all used!
  bool displayBit35;               // Heater running or not
  bool displayFilter1;                // Pump 1 running or not 
  bool displayFilter2;                // Pump 2 running or not
  bool displayBit38;                // Hot tube lights activated or not
  bool displayHeater; 
  bool displayBit40; 
  bool displayBit41; 
  bool displayBlower;
  bool displayBit43; 
  bool displayFiltration; 
  bool displayBit45; 
  bool displayBit46; 
  bool displayLight; 
  bool displayPump1; 
  bool displayPump2; 
  bool displayBit50; 
  bool displayBit51; 
  bool displayBit52; 
  bool displayBit53; 
  bool displayBit54; 
  bool displayBit55; 
  bool displayBit56; 
  bool displayBit57; 
  bool displayBit58;
  bool displayStandardMode;
  bool displayEcoMode;
  bool displayBit61;
  bool displayBit62;
  bool displayBit63;
  bool displayBit64;
  bool displayBit65;
  bool displayBit66;
  bool displayBit67;
  bool displayBit68;
  bool displayBit69;
  bool displayBit70;
  bool displayBit71;
  static bool displayDataBufferOverflow;
  String displayMode = "Inconnu";
  
  // Write button data to control unit  
  static bool writeDisplayData;               // If something should be written to button data line  
  static bool writeMode;
  static bool writeTempUp;
  static bool writeTempDown;
  static bool writeLight;
  static bool writePump1;
  static bool writePump2;   
  static bool writePump3;
 static bool writeBlower;

  static byte displayDataBuffer[displayDataBufferSize];   // Array of display data measurements 
  static bool displayDataBufferReady;                 // Is buffer available to be decoded

  private:
  
  static void clockPinInterrupt();
  void decodeDisplayData();
  String lockup_LCD_character(int LCD_character);
  int LCD_segment_1;
  int LCD_segment_2;
  int LCD_segment_3;
  int LCD_segment_4;
  String LCD_display_1;
  String LCD_display_2;
  String LCD_display_3;
  String LCD_display_4;  
  //static byte displayDataBuffer[displayDataBufferSize];   // Array of display data measurements 
  static unsigned long clockInterruptTime;
  static int clockBitCounter;                       // Counter of pulses within a cycle
  static byte dataIndex;                  
  static byte clockPin;
  static byte displayPin;
  static byte buttonPin;

  int updateTempDirection;
  int updateTempButtonPresses;
  unsigned long buttonPressTimerPrevMillis; 
};

  
#endif  // Balboa_GS_Interface_hd
