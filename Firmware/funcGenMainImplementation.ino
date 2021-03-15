/*
  SFU Engineering Competition 2020
  Author: Mahmoud Kamaleldin

  Firmware written for Arduino Uno Platform to run a
  2-channel power supply/function generator
*/


#include <ArduinoJson.h>
#include "crc16.h"
#include <SPI.h>
#include <MD_AD9833.h>
#include "Encoder.h"
#include "mcp4xxx.h"
#include <Wire.h>
#include "DFRobot_LCD.h"


using namespace icecave::arduino;
/*
 * Defines
 */
// UART Message Bytes
#define START_BYTE '\1'
#define DATA_END_BYTE '\2'
#define CRC_END_BYTE '\3'

// Commands to be sent by GUI
#define UPDATE_OUTPUT_COMMAND "UPDATE_OUTPUT"
#define GET_STATUS_COMMAND "GET_STATUS"
#define CHANGE_MODE_COMMAND "CHANGE_MODE"

// Serial Port
#define SERIAL_FREQUENCY 115200

// Digital Pins for SPI comm with the AD9833 IC
#define SPI_DATA  11  ///< SPI Data pin number
#define SPI_CLK   13  ///< SPI Clock pin number

// Clock Generating Pin
#define AD_CLK 10  // Crystal Pin for AD9833 ICs
#define AD_CLK_FREQ 4000000 // 4MHz Crystal Frequency

// SPI Chip Select Pins
#define AD1_FSYNC 12  // Chip Select for AD9833 on Channel 1
#define AD2_FSYNC 7   // Chip Select for AD9833 on Channel 2
#define DIGIPOT_CH1 6 // Chip Select for DigiPot on Channel 1
#define DIGIPOT_CH2 5 // Chip Select for DigiPot on Channel 2

// Pins for I2C Comms with LCD
#define I2C_SCL 19
#define I2C_SDA 18

// Digital Pins for Attenuator and Power Supply Output
#define ATTENUATOR_CH1 8
#define ATTENUATOR_CH2 9
#define PS_CH_1 2
#define PS_CH_2 3

// Encoder Pins
#define ENCODER_PIN_A 4
#define ENCODER_PIN_B A1

// Parameter Switch Button (Frq, Phs, Amp)
#define PARAM_BUTTON A2

// Pin for Switch between Channels
#define CHANNEL_SELECT A0

// MIN/MAX Params
#define MAX_FREQ 1000000
#define MIN_FREQ 1

#define MAX_AMP 5.0
#define MIN_AMP -5.0

#define MIN_SHIFT -5.0
#define MAX_SHIFT 5.0

#define MAX_PHASE 360
#define MIN_PHASE 0

// LCD Params
#define LCD_COLOR_BLUE 4

// Datatypes
typedef enum
{
  IDLE,
  DATA_READ,
  CRC_READ,
  PARSE_MESSAGE
}UartState_t;

typedef enum
{
  SOFTWARE_MODE,
  HARDWARE_MODE
}ControlState_t;

typedef enum
{
  DEVICE_CHANNEL_1,
  DEVICE_CHANNEL_2
}DeviceChannel_t;

typedef enum
{
  FUNC_GEN = HIGH,
  POW_SUP = LOW
}ChannelMode_t;

typedef enum
{
  FREQUENCY,
  AMPLITUDE,
  SHIFT,
  PHASE,
  WAVEFORM
}Parameter_t;

enum WAVE_MODE
{
  MODE_OFF,
  MODE_SINE,      ///< Set output to a sine wave at selected frequency
  MODE_SQUARE,   ///< Set output to a square wave at selected frequency
  MODE_TRIANGLE,  ///< Set output to a triangle wave at selected frequency
  MODE_DC_SUPPLY
};

enum ERROR_MESSAGES
{
  NOT_START_BYTE_IN_IDLE,
  CRC_ERROR
};

typedef struct
{
  int8_t waveform;
  uint32_t frequency;
  float amplitude;
  float shift;
  uint16_t phase;
} FuncGenIC_t;

// UART Comms Global Variables
Crc16 crc;
String inputString = "";            // a String to hold incoming data
uint16_t guiCrc = 0;                // an int to hold incoming CRC
//String jsonData = "";               // String to save inputString in when message is processed
bool stringComplete = false;        // whether the string is complete
StaticJsonDocument<400> jsonParser; // Holds JSON object from GUI
UartState_t uart_state = IDLE;      // UART Comms State   

// Device Output Global Variables
ControlState_t control_state = HARDWARE_MODE;         // Device control mode
MD_AD9833 funcGenIC(AD1_FSYNC);                       // Hardware SPI
FuncGenIC_t deviceChannels[2];                        // 2-Channel Struct to keep track of parameters
Encoder hwModeEncoder(ENCODER_PIN_A, ENCODER_PIN_B);  // Encoder object to keep track of encoder position
int previousEncoderPosition  = 0;                     // Previously recorded encoder position
Parameter_t selectedParam = FREQUENCY;                // Param that user is controlling at a time
DFRobot_LCD lcd(16,2);                                // 16 characters and 2 lines LCD Screen over I2C
MCP4XXX* digipot_amp_ch1;                             // DigiPot for Amplitude Control on Channel 1
MCP4XXX* digipot_shift_ch1;                           // DigiPot for Shift Control on Channel 1
MCP4XXX* digipot_amp_ch2;                             // DigiPot for Amplitude Control on Channel 1
MCP4XXX* digipot_shift_ch2;                           // DigiPot for Shift Control on Channel 1

void updateLCDScreen(FuncGenIC_t *funcGenStruct, DeviceChannel_t channel)
{
  String param = "";

  // Evaluate Parameter Selection
  if(selectedParam == FREQUENCY)
  {
    param = "F";
  }
  else
  if(selectedParam == AMPLITUDE)
  {
    param = "A";
  }
  else
  if(selectedParam == PHASE)
  {
    param = "P";
  } 
  else
  if(selectedParam == WAVEFORM)
  {
    param = "W";
  } 
  else
  if(selectedParam == SHIFT)
  {
    param = "S";
  } 


  String freqUnit = "";
  int frequency = 0;

  String mode = "";

  // Evaluate Frequency Unit
  if(funcGenStruct->frequency < 1000)
  {
    freqUnit = "Hz";
    frequency = funcGenStruct->frequency;
  }
  else
  if(funcGenStruct->frequency < 1000000)
  {
    freqUnit = "kHz";
    frequency = funcGenStruct->frequency/1000;
  }
  else
  {
    freqUnit = "MHz";
    frequency = funcGenStruct->frequency/1000000;
  }

  // Evaluate Channel Mode
  if(funcGenStruct->waveform == MODE_SINE)
  {
    mode = "SN";
  }
  else
  if(funcGenStruct->waveform == MODE_SQUARE)
  {
    mode = "SQ";
  }
  else
  if(funcGenStruct->waveform == MODE_TRIANGLE)
  {
    mode = "TR";
  } 
  else
  if(funcGenStruct->waveform == MODE_DC_SUPPLY)
  {
    mode = "DC";
  } 
  else
  if(funcGenStruct->waveform == MODE_OFF)
  {
    mode = "O";
  } 

  const char buffer[16];
  snprintf(&buffer[0], sizeof(buffer), "%+.1fV %sC%d %s", funcGenStruct->amplitude, mode.c_str(), channel, param.c_str());
  lcd.setCursor(0, 0);
  lcd.print(buffer);

  snprintf(&buffer[0], sizeof(buffer), "%+.1fV %d%s", funcGenStruct->shift, frequency, freqUnit.c_str());
  lcd.setCursor(0, 1);
  lcd.print(buffer);
}

void setActiveSpi(uint8_t pin)
{
  if(pin == AD1_FSYNC)
  {
    digitalWrite(AD2_FSYNC, LOW);
    digitalWrite(DIGIPOT_CH1, LOW);
    digitalWrite(DIGIPOT_CH2, LOW);

    digitalWrite(AD1_FSYNC, HIGH);
  }
  else
  if(pin == AD2_FSYNC)  
  {
    digitalWrite(AD1_FSYNC, LOW);
    digitalWrite(DIGIPOT_CH1, LOW);
    digitalWrite(DIGIPOT_CH2, LOW);

    digitalWrite(AD2_FSYNC, HIGH);  
  }
  else
  if(pin == DIGIPOT_CH1)  
  {
    digitalWrite(AD1_FSYNC, LOW);
    digitalWrite(DIGIPOT_CH2, LOW);
    digitalWrite(AD2_FSYNC, LOW);

    digitalWrite(DIGIPOT_CH1, HIGH);  
  }
  else
  if(pin == DIGIPOT_CH2)  
  {
    digitalWrite(AD1_FSYNC, LOW);
    digitalWrite(DIGIPOT_CH1, LOW);
    digitalWrite(AD2_FSYNC, LOW);

    digitalWrite(DIGIPOT_CH2, HIGH); 
  }
  else
  {
    // undefined channel
    return;
  }
}

void setOutputVoltage(float voltage, float shift, DeviceChannel_t channel)
{
  uint8_t voltage_pot = (voltage+5)/10 * 127.0;
  uint8_t shift_pot = (shift+5)/10 * 127.0;

  if(channel == DEVICE_CHANNEL_1)
  {
    digipot_amp_ch1->set(voltage_pot);
    digipot_shift_ch1->set(shift_pot);
  }
  else
  if(channel == DEVICE_CHANNEL_2)  
  {
    digipot_amp_ch2->set(voltage_pot);
    digipot_shift_ch2->set(shift_pot);
  }
  else
  {
    // undefined channel
    return;
  }  
}

void setOutput(FuncGenIC_t *funcGenStruct, DeviceChannel_t channel) 
{
  if(funcGenStruct->waveform == MODE_DC_SUPPLY)
  {
    digitalWrite(ATTENUATOR_CH2, HIGH);
    digitalWrite(PS_CH_2, HIGH);

    setOutputVoltage(funcGenStruct->amplitude, funcGenStruct->shift, channel);
  }
  else
  {
    if(funcGenStruct->waveform == MODE_OFF)
    {
      if(channel == DEVICE_CHANNEL_1)
      {
        digitalWrite(ATTENUATOR_CH1, LOW);
        digitalWrite(PS_CH_1, LOW);
      }
      else
      {
        digitalWrite(ATTENUATOR_CH2, LOW);
        digitalWrite(PS_CH_2, LOW);
      }  
    } 
    else
    {
      digitalWrite(ATTENUATOR_CH2, HIGH);
      digitalWrite(PS_CH_2, LOW);
    }
    (channel == DEVICE_CHANNEL_1) ? setActiveSpi(AD1_FSYNC) : setActiveSpi(AD2_FSYNC);   
    funcGenIC.setPhase(MD_AD9833::CHAN_0, funcGenStruct->phase);
    funcGenIC.setFrequency(MD_AD9833::CHAN_0, funcGenStruct->frequency);
    setOutputVoltage(funcGenStruct->amplitude, funcGenStruct->shift, channel);
    funcGenIC.setMode(funcGenStruct->waveform);        
  }
}

/*
  Send back status to GUI with this JSON format
  {
      "channel 1":{
          "waveform": "Constant", #Constant, Triangle, Square, Sine
          "frequency": int(100), #in Hz
          "amplitude": float(3.155),
          "phase":  int(90), #between 0 and something
      },
      "channel 2":{
          "waveform": "Constant", #Constant, Triangle, Square, Sine
          "frequency": int(100), #in Hz
          "amplitude": float(3.155),
          "phase":  int(90), #between 0 and something
      },        
    }
*/
void sendStatusToGui(String command)
{
  jsonParser.clear();
  // Create JSON with status
  jsonParser["command"] = command;
  jsonParser["payload"]["channel 1"]["waveform"] = deviceChannels[DEVICE_CHANNEL_1].waveform;
  jsonParser["payload"]["channel 1"]["frequency"] = deviceChannels[DEVICE_CHANNEL_1].frequency;
  jsonParser["payload"]["channel 1"]["amplitude"] = deviceChannels[DEVICE_CHANNEL_1].amplitude;
  jsonParser["payload"]["channel 1"]["phase"] = deviceChannels[DEVICE_CHANNEL_1].phase;
  jsonParser["payload"]["channel 1"]["shift"] = deviceChannels[DEVICE_CHANNEL_1].shift;

  jsonParser["payload"]["channel 2"]["waveform"] = deviceChannels[DEVICE_CHANNEL_2].waveform;
  jsonParser["payload"]["channel 2"]["frequency"] = deviceChannels[DEVICE_CHANNEL_2].frequency;
  jsonParser["payload"]["channel 2"]["amplitude"] = deviceChannels[DEVICE_CHANNEL_2].amplitude;
  jsonParser["payload"]["channel 2"]["phase"] = deviceChannels[DEVICE_CHANNEL_2].phase;
  jsonParser["payload"]["channel 2"]["shift"] = deviceChannels[DEVICE_CHANNEL_2].shift;
  //String serializedStatus = "";
  serializeJson(jsonParser, inputString);

  jsonParser.clear();
  // Create CRC
  uint16_t arduinoCrc = crc.XModemCrc(inputString.c_str(), 0, strlen(inputString.c_str())); 
  char hexCRC[4];
  snprintf(hexCRC, "%X", arduinoCrc); //convert number to hex

  Serial.print(START_BYTE);
  Serial.print(inputString);
  Serial.print(DATA_END_BYTE);
  Serial.print(hexCRC);
  Serial.print(CRC_END_BYTE);
  //String msgData = START_BYTE + serializedStatus + DATA_END_BYTE + hexCRC + CRC_END_BYTE;

  //Serial.println(msgData);
  Serial.println('\4');
}

void setup() {
  // Initialize serial port
  Serial.begin(SERIAL_FREQUENCY);
  while (!Serial) continue;

  // Setup Pins
  pinMode(AD1_FSYNC, OUTPUT);
  pinMode(AD2_FSYNC, OUTPUT);
  pinMode(ATTENUATOR_CH1, OUTPUT);
  pinMode(ATTENUATOR_CH2, OUTPUT);
  pinMode(PS_CH_1, OUTPUT);
  pinMode(PS_CH_2, OUTPUT);
  pinMode(PARAM_BUTTON, INPUT);
  pinMode(CHANNEL_SELECT, INPUT); 
  tone(AD_CLK, AD_CLK_FREQ);

  // Setup DigiPot
  digipot_amp_ch1 = new MCP4XXX(DIGIPOT_CH1, MCP4XXX::pot_0);
  digipot_shift_ch1 = new MCP4XXX(DIGIPOT_CH1, MCP4XXX::pot_1);

  digipot_amp_ch2 = new MCP4XXX(DIGIPOT_CH2, MCP4XXX::pot_0);
  digipot_shift_ch2 = new MCP4XXX(DIGIPOT_CH2, MCP4XXX::pot_1);

  // Setup LCD
  lcd.init();
  lcd.setColor(LCD_COLOR_BLUE);

  attachInterrupt(digitalPinToInterrupt(PARAM_BUTTON), handleParamButtonISR, RISING);

  // AD9833 Initialization, set IC to use Freq and Phase Channel 0
  funcGenIC.begin();

  bool isfuncGenICInitialized = false;
  while(!isfuncGenICInitialized)
  {
    funcGenIC.setActiveFrequency(MD_AD9833::CHAN_0);
    funcGenIC.setActivePhase(MD_AD9833::CHAN_0);

    if(funcGenIC.getActiveFrequency() == MD_AD9833::CHAN_0 && funcGenIC.getActivePhase() == MD_AD9833::CHAN_0)
    {
      isfuncGenICInitialized = true;
    }
  }
  MD_AD9833::channel_t chan = funcGenIC.getActiveFrequency();

  inputString.reserve(300);
  //jsonData.reserve(300);
}

void loop() {
  if(control_state == SOFTWARE_MODE)
  {
    if (stringComplete)
    {    
        DeserializationError error = deserializeJson(jsonParser, inputString);
        inputString = ""; // Reset jsonData string
        stringComplete = false;
        // Test if parsing succeeds.
        if (error)
        {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(error.f_str());
          Serial.println('\4');
          return;
        }  

        const char* command = jsonParser["command"];
        Serial.println(command);
        
        if(strcmp(command, UPDATE_OUTPUT_COMMAND) == 0)
        {
          Serial.println('\4');
          uint8_t channel = jsonParser["payload"]["channel"];

          deviceChannels[channel].waveform = jsonParser["payload"]["waveform"];
          deviceChannels[channel].frequency = jsonParser["payload"]["frequency"];
          deviceChannels[channel].amplitude = jsonParser["payload"]["amplitude"];
          deviceChannels[channel].phase = jsonParser["payload"]["phase"]; 
          deviceChannels[channel].shift = jsonParser["payload"]["shift"]; 

          setOutput(&deviceChannels[channel], channel);
          sendStatusToGui(UPDATE_OUTPUT_COMMAND);
        }
        else
        if(strcmp(jsonParser["command"], CHANGE_MODE_COMMAND) == 0)
        {
          //Serial.println("Change Mode Command\4");
          control_state = HARDWARE_MODE;
          sendStatusToGui(CHANGE_MODE_COMMAND);
        }
        else
        if(strcmp(jsonParser["command"], GET_STATUS_COMMAND) == 0)
        {
          //Serial.println("Change Mode Command\4");
          sendStatusToGui(GET_STATUS_COMMAND);
        }
    }
  }
  else
  if(control_state == HARDWARE_MODE)
  {
    if (stringComplete) {    
      Serial.println(inputString);
      DeserializationError error = deserializeJson(jsonParser, inputString);
      inputString = ""; // Reset jsonData string
      stringComplete = false;
      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        Serial.println('\4');
        return;
      }  
      
      if(strcmp(jsonParser["command"], CHANGE_MODE_COMMAND) == 0)
      {
        Serial.println("Change Mode Command\4");
        control_state = SOFTWARE_MODE;
        sendStatusToGui(CHANGE_MODE_COMMAND);
      }
      else
      if(strcmp(jsonParser["command"], GET_STATUS_COMMAND) == 0)
      {
        //Serial.println("Change Mode Command\4");
        sendStatusToGui(GET_STATUS_COMMAND);
      }    
    }
    else
    {
      int newEncoderPosition = hwModeEncoder.read();
      int encoderPosDiff = previousEncoderPosition - newEncoderPosition;
      if (encoderPosDiff != 0)
      {
        previousEncoderPosition = newEncoderPosition;
      }
      else
      {
        return;
      }

      DeviceChannel_t channel = digitalRead(CHANNEL_SELECT);

      if(selectedParam == FREQUENCY)
      {
        int increment = 0;
        if(abs(encoderPosDiff) > 0 && abs(encoderPosDiff) <= 5)
        {
          increment = 1;
        }
        else
        if(abs(encoderPosDiff) > 5 && abs(encoderPosDiff) <= 20)
        {
          increment = 10;
        }
        else
        if(abs(encoderPosDiff) > 20 && abs(encoderPosDiff) <= 100)
        {
          increment = 100;
        }
        else
        if(abs(encoderPosDiff) > 100)
        {
          increment = 1000;
        }

        increment *= (encoderPosDiff/encoderPosDiff);

        deviceChannels[channel].frequency+= increment;
        if(deviceChannels[channel].frequency < MIN_FREQ)
        {
          deviceChannels[channel].frequency = MIN_FREQ;
        }
        else
        if(deviceChannels[channel].frequency > MAX_FREQ)
        {
          deviceChannels[channel].frequency = MAX_FREQ;
        }
      }
      else
      if(selectedParam == AMPLITUDE)
      {
        int increment = 0.1;
        increment *= (encoderPosDiff/encoderPosDiff);

        deviceChannels[channel].amplitude+= increment;
        if(deviceChannels[channel].amplitude < MIN_AMP)
        {
          deviceChannels[channel].amplitude = MIN_AMP;
        }
        else
        if(deviceChannels[channel].amplitude > MAX_AMP)
        {
          deviceChannels[channel].amplitude = MAX_AMP;
        }
      }
      else
      if(selectedParam == PHASE)
      {
        int increment = 1;
        increment *= (encoderPosDiff/encoderPosDiff);

        deviceChannels[channel].amplitude+= increment;
        if(deviceChannels[channel].phase < MIN_PHASE)
        {
          deviceChannels[channel].phase = MIN_PHASE;
        }
        else
        if(deviceChannels[channel].phase > MAX_PHASE)
        {
          deviceChannels[channel].phase = MAX_PHASE;
        }
      }
      else
      if(selectedParam == WAVEFORM)
      {
        int increment = 1;
        increment *= (encoderPosDiff/encoderPosDiff);

        deviceChannels[channel].waveform+= increment;
        if(deviceChannels[channel].waveform < MODE_SINE)
        {
          deviceChannels[channel].waveform = MODE_DC_SUPPLY;
        }
        else
        if(deviceChannels[channel].waveform > MODE_DC_SUPPLY)
        {
          deviceChannels[channel].waveform = MODE_SINE;
        }
      }
      else
      if(selectedParam == SHIFT)
      {
        int increment = 0.1;
        increment *= (encoderPosDiff/encoderPosDiff);

        deviceChannels[channel].shift+= increment;
        if(deviceChannels[channel].shift < MIN_SHIFT)
        {
          deviceChannels[channel].shift = MIN_SHIFT;
        }
        else
        if(deviceChannels[channel].shift > MAX_SHIFT)
        {
          deviceChannels[channel].shift = MAX_SHIFT;
        }
      }

      updateLCDScreen(&deviceChannels[channel], channel); 
      setOutput(&deviceChannels[channel], channel);
    }
  }
}

void handleParamButtonISR()
{
  selectedParam = selectedParam + 1;
  if(selectedParam > 4 || selectedParam < 0)
  {
    selectedParam = FREQUENCY;
  }

  DeviceChannel_t channel = digitalRead(CHANNEL_SELECT);
  updateLCDScreen(&deviceChannels[channel], channel); 
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();  
    /*
     * ----UART State Machine----
     * Parse incoming bytes based on what stage we are in in the parsing operation
     */
    if(uart_state == IDLE)
    {
      if(inChar == START_BYTE)
      {
        inputString = "";
        uart_state = DATA_READ;
        return;    
      }
      else
      {
        Serial.print("ERROR:");
        Serial.print(NOT_START_BYTE_IN_IDLE);
        Serial.println('\0');
        return;
      }
    }
    else
    if(uart_state == DATA_READ)
    {
      if(inChar == START_BYTE)
      {
        inputString = "";
        Serial.println("new message takeover");
      }
      if(inChar == DATA_END_BYTE)
      {
        Serial.println("Data finished transmitting");
        uart_state = CRC_READ;   
      }
      else
      {
        inputString += inChar;   
      }
    }
    else
    if(uart_state == CRC_READ)
    {
      if(inChar == START_BYTE)
      {
        guiCrc = 0;
        inputString = "";
        Serial.println("new message takeover from crc");
        uart_state = DATA_READ;
      }
      if(inChar == CRC_END_BYTE)
      {
        Serial.println("CRC finished transmitting");
        uint16_t arduinoCrc = crc.XModemCrc(inputString.c_str(), 0, strlen(inputString.c_str())); 

        if(arduinoCrc != guiCrc)
        {
          Serial.print("ERROR:");
          Serial.println(CRC_ERROR);
          Serial.println(arduinoCrc);
          Serial.println(guiCrc);
        }
        else
        {
          Serial.println("CRC Good");
        }

        //jsonData = inputString;
        Serial.println("DONE");
        Serial.println(inputString);
        //Serial.println(jsonData);
        guiCrc = 0;
        //inputString = "";
        stringComplete = true;
  
        uart_state = IDLE;
      }
      else
      {
        guiCrc = guiCrc << 8;
        guiCrc += (uint8_t)inChar;
      }      
    }
    else
    {
      // Undefined state, reset to IDLE
      uart_state = IDLE;
    }
  }
}
