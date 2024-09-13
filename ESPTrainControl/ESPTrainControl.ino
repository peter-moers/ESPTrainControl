#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>


static const char VISION_SENSOR_PORT_ID = 0x12;
static const char VISION_SENSOR_MODE_COLOR = 0x00;
static const char VISION_SENSOR_MODE_CTAG = 0x01;
static const char VISION_SENSOR_MODE_REFLECTIVITY = 0x02;
static const char VISION_SENSOR_MODE_RGB = 0x03;

static const int BLACK = 0;
static const int PINK = 1;
static const int PURPLE = 2;
static const int BLUE = 3;
static const int LIGHTBLUE = 4;
static const int CYAN = 5;
static const int GREEN = 6;
static const int YELLOW = 7;
static const int ORANGE = 8;
static const int RED = 9;
static const int WHITE = 10;
static const int NONE = 255;
static const char *COLORS[12] = {"black", "pink", "purple", "blue", "lightblue", "cyan", "green", "yellow", "orange", "red", "white", "none"};

static const char SPEED_SENSOR_PORT_ID = 0x13;
static const char SPEED_SENSOR_MODE_SPEED = 0x00;
static const char SPEED_SENSOR_MODE_COUNT = 0x01;

static const char VOLTAGE_SENSOR_PORT_ID = 0x14;

static const char speed_fw1 = 0x1e;
static const char speed_fw2 = 0x32;
static const char speed_fw3 = 0x64;
static const char speed_stop = 0x00;
static const char speed_bw = 0xce;

// GPIO pin for analog input
const int analogInputPin = 34;

const int pin_horn = 13;
const int pin_stop = 27;
const int pin_fuel = 12;
const int pin_light = 14;


// Speed control parameters
const int minAnalogValue = 0;
const int maxAnalogValue = 4095; // Maximum value for 12-bit ADC
const int minSpeed = 240; // Minimum speed value - use for calibration
const int maxSpeed = -80; // Maximum speed value - use for calibration


BLEClient* pClient;
bool isConnected = false;
BLEAddress serverAddress("18:04:ED:F2:A3:48"); // you'll need to change this!

// UUIDs for the service and characteristics
BLEUUID serviceUUID("00001623-1212-efde-1623-785feabcd123");
BLEUUID charUUID("00001624-1212-efde-1623-785feabcd123");

// Characteristics
BLERemoteCharacteristic* pRemoteCharacteristic;

// Actions to do in the main loop
bool speed_new_value = false;
uint8_t speed = 0x00;

// Inputvalue throtle
int analogValue = 0;
int speed_percent = 40;
int speed_percent_prev = 40;
bool input_speed = false;

// Timeout parameters to avoid interference between control modes (push <-> throttle)
const unsigned long speedChangeTimeout = 2000; // 2 seconds
unsigned long lastSpeedZeroTime = 0;

//Color  variable
int current_color = 10;

// Digital input debounce
unsigned long debounceDelay = 80; 
int buttonState_horn;            // the current reading from the input pin
int lastButtonState_horn = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime_horn = 0;
int buttonState_fuel;            // the current reading from the input pin
int lastButtonState_fuel = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime_fuel = 0;
int buttonState_light;            // the current reading from the input pin
int lastButtonState_light = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime_light = 0;
int buttonState_stop;            // the current reading from the input pin
int lastButtonState_stop = LOW;  // the previous reading from the input pin
unsigned long lastDebounceTime_stop = 0;

void setup() {
  pinMode(pin_horn, INPUT_PULLUP);
  pinMode(pin_fuel, INPUT_PULLUP);
  pinMode(pin_stop, INPUT_PULLUP);
  pinMode(pin_light, INPUT_PULLUP);
  Serial.begin(115200);
  BLEDevice::init("");
}

void loop() {

  if (isConnected) {

    // Read buttons   
    readInputPins();

    // Read throttle
    analogValue = analogRead(analogInputPin);
    //Serial.println(analogValue);
    speed_percent = map(analogValue, minAnalogValue, maxAnalogValue, minSpeed, maxSpeed);
    speed_percent = (speed_percent + speed_percent_prev) / 2;  //basic noise filtering
    if( speed_percent < speed_percent_prev - 4 || speed_percent > speed_percent_prev + 4) { //some more noise filtering
      input_speed = true;
      //Serial.println(speed_percent);
      if(speed_percent>20 && speed_percent<60 && speed!=speed_stop) { //stop
        speed = speed_stop;
        lastSpeedZeroTime = millis();
      }
      if(speed_percent<20) {
        speed = speed_bw;
      }
      if(speed_percent>60 && speed_percent<75) { //slow forward
        speed = speed_fw1;
      }
      if(speed_percent>75 && speed_percent<90) { //medium forward
        speed = speed_fw2;
      }
      if(speed_percent>90 ) { //fast forward
        speed = speed_fw3;
      }
      speed_percent_prev = speed_percent;
      speed_new_value = true;
    }

    // Set speed is new value is available
    if(speed_new_value) {
      speed_new_value = false;
      Serial.println("New speed!");

      if (speed != 0x00 && millis() - lastSpeedZeroTime < speedChangeTimeout && !input_speed) {
          // Block speed change if speed is 0 and timeout has not elapsed
          Serial.println("Blocked speed change due to timeout");
        } else {
          controlSpeed(speed);
          /*if (speed == 0x00) {
            lastSpeedZeroTime = millis(); // Record time when speed is set to 0
          }*/
        }
        input_speed = false; // Release for default behaviour

    }
    
  } else {
    Serial.println("Connection to Train lost....Reconnecting...");
    connectToServer();
    delay(100);
  }

  // Reconnect
  if(pClient->isConnected()) {
    isConnected = true;
  } else {
    delay(1000); // allow train to shut down
    isConnected = false;
  }

}

void connectToServer() {
  Serial.println("Attempting to connect to BLE server...");
  BLEDevice::getScan()->stop();
  pClient = BLEDevice::createClient(); // Create a new BLE client instance

  // Connect to the server with the specified UUID
  pClient->connect(BLEAddress(serverAddress)); // Replace with the BLE server address

  if (pClient->isConnected()) {
    isConnected = true;
    Serial.println("Connected to BLE server!");
  } else {
    Serial.println("Failed to connect to BLE server...");
    return;
  }

 
  // Obtain a reference to the service
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.println("Failed to find service");
    return;
  }
  Serial.println("Found service");

  // Obtain a reference to the characteristics
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.println("Failed to find characteristic");
    return;
  }
  Serial.println("Found characteristic");
  isConnected = true;

  // Activate sensors
  activateSpeedSensor();
  delay(200);

  activateRGBSensor();
  delay(200);

  // Register for BLE notifications
  pRemoteCharacteristic->registerForNotify(onNotify);

  //Prepare BLE
  prepareBLE();

}

void onNotify(BLERemoteCharacteristic* pCharacteristic, uint8_t* pData, size_t length, bool isNotify) {

  // Create a copy of the pData array
  uint8_t dataCopy[length];
  for (int i = 0; i < length; i++) {
    dataCopy[i] = pData[i];
  } 
  /*for (int i = 0; i < length; i++) {
    Serial.print(dataCopy[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  */

  // Start motor on rotation
  if(dataCopy[3] == SPEED_SENSOR_PORT_ID && !input_speed) {
    uint8_t byte1 = dataCopy[4];
    uint8_t byte2 = dataCopy[5];
    int16_t speed_sensor = (byte2 << 8) | byte1;
    //Serial.println(speed_sensor);
    if(speed==0x00 && speed_sensor>10 && !input_speed && !speed_new_value) { // forward
      speed = 0x32;
      speed_new_value = true;
    } else if(speed==0x00 && speed_sensor<-10 && !input_speed && !speed_new_value) { // backward
      speed = 0xce;
      speed_new_value = true;
    } else if(speed_sensor<10 && speed_sensor>-10 && !speed_new_value) {  // stop
      speed = 0x00;
      input_speed = false;
    }
  }

  // Color sensor actions
  if(dataCopy[3] == VISION_SENSOR_PORT_ID) {
    //TODO
  }
}



void controlSpeed(uint8_t set_speed) {
  // Format the speed command and send it
  uint8_t command2[8] = {0x08, 0x00, 0x81, 0x00, 0x01, 0x51, 0x00, set_speed};
  sendCommand(command2, sizeof(command2));
}

void playSound(int soundID) {
  // Format the sound command and send it
  uint8_t command2[8] = {0x08, 0x00, 0x81, 0x01, 0x11, 0x51, 0x01, soundID};
  sendCommand(command2, sizeof(command2));
}

void prepareBLE() {
  uint8_t command[10] = {0x0a, 0x00, 0x41, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01};
  sendCommand(command, sizeof(command));
}

void nextColor() {
  current_color++;
  if(current_color==11) {
    current_color=0;
  }
  setLight(current_color);
}

void setLight(int lightValue){
  uint8_t command2[8] = {0x08, 0x00, 0x81, 0x11, 0x11, 0x51, 0x00,lightValue};
  sendCommand(command2, sizeof(command2));
}

void activateSpeedSensor() {
  Serial.println("Activating speed sensor.");
  uint8_t command[10] = { 0x0a, 0x00, 0x41, SPEED_SENSOR_PORT_ID, SPEED_SENSOR_MODE_SPEED, 0x01, 0x00, 0x00, 0x00, 0x01 };
  sendCommand(command, sizeof(command));
}

void activateRGBSensor() {
  Serial.println("Activating RGB sensor.");
  uint8_t command[10] = { 0x0a, 0x00, 0x41, VISION_SENSOR_PORT_ID, VISION_SENSOR_MODE_COLOR, 0x01, 0x00, 0x00, 0x00, 0x01 };
  sendCommand(command, sizeof(command));
}

void sendCommand(uint8_t* command, size_t length) {
  if (pRemoteCharacteristic != nullptr) {
    pRemoteCharacteristic->writeValue(command, length);
    Serial.print("Sent command: ");
    for (int i = 0; i < length; i++) {
      Serial.print(command[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("Characteristic not available");
  }
}

void readInputPins() {

  int reading = digitalRead(pin_light); // Read the state of the button
  // If the reading has changed since the last time
  if (reading != lastButtonState_light) {
    lastDebounceTime_light = millis(); // reset the debounce timer
  }
  // If the button state has been stable for the debounce delay
  if ((millis() - lastDebounceTime_light) > debounceDelay) {
    // If the button state has changed
    if (reading != buttonState_light) {
      buttonState_light = reading; // Update the button state
      // If the button state is HIGH (pressed)
      if (buttonState_light == LOW) {
        Serial.println("Light pressed!");
        nextColor();
      }
    }
  }
  lastButtonState_light = reading; // Save the reading for the next loop iteration

  reading = digitalRead(pin_horn); // Read the state of the button
  // If the reading has changed since the last time
  if (reading != lastButtonState_horn) {
    lastDebounceTime_horn = millis(); // reset the debounce timer
  }
  // If the button state has been stable for the debounce delay
  if ((millis() - lastDebounceTime_horn) > debounceDelay) {
    // If the button state has changed
    if (reading != buttonState_horn) {
      buttonState_horn = reading; // Update the button state
      // If the button state is HIGH (pressed)
      if (buttonState_horn == LOW) {
        Serial.println("Horn pressed!");
        playSound(0x09);
      }
    }
  }
  lastButtonState_horn = reading; // Save the reading for the next loop iteration

  reading = digitalRead(pin_stop); // Read the state of the button
  // If the reading has changed since the last time
  if (reading != lastButtonState_stop) {
    lastDebounceTime_horn = millis(); // reset the debounce timer
  }
  // If the button state has been stable for the debounce delay
  if ((millis() - lastDebounceTime_stop) > debounceDelay) {
    // If the button state has changed
    if (reading != buttonState_stop) {
      buttonState_stop = reading; // Update the button state
      // If the button state is HIGH (pressed)
      if (buttonState_stop == LOW) {
        Serial.println("Stop pressed!");
        speed = 0x00;
        input_speed = false;
        speed_new_value = true;
        lastSpeedZeroTime = millis();
      }
    }
  }
  lastButtonState_stop = reading; // Save the reading for the next loop iteration

  reading = digitalRead(pin_fuel); // Read the state of the button
  // If the reading has changed since the last time
  if (reading != lastButtonState_fuel) {
    lastDebounceTime_fuel = millis(); // reset the debounce timer
  }
  // If the button state has been stable for the debounce delay
  if ((millis() - lastDebounceTime_fuel) > debounceDelay) {
    // If the button state has changed
    if (reading != buttonState_fuel) {
      buttonState_fuel = reading; // Update the button state
      // If the button state is HIGH (pressed)
      if (buttonState_fuel == LOW) {
        Serial.println("Hier kunde panken!");
        playSound(0x07);
      }
    }
  }
  lastButtonState_fuel = reading; // Save the reading for the next loop iteration

}