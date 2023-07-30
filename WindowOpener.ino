/***************************************************************************
  This sketch automates the opening and closing of our bedroom window. It 
  opens or closes the window based on the time of day or controlled by
  buttons or a remote control. A linear motor is used to move the window.
  Various information, such as the position of the window, is presented on a web
  page. Look on https://github.com/Tsjakka/WindowOpener for more information.
  
  This sketch uses code from, among others, the following websites:
  https://randomnerdtutorials.com/esp8266-web-server/
  https://www.arduino.cc/en/Tutorial/UdpNTPClient
  https://github.com/sfrwmaker/sunMoon
  https://github.com/RobTillaart/INA226

  Written by Tsjakka from the Netherlands.
  BSD license, all text above must be included in any redistribution.
 ***************************************************************************/

#include <math.h>
#include <Time.h>
#include <TimeLib.h>
#include <sunMoon.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BME280.h>
#include <ESP_Mail_Client.h>
#include <ESP_EEPROM.h>
#include <INA226.h>
#include <Wire.h>

// ************************************************************************
// Change the constants below to adjust the software to your situation
// ************************************************************************

// GPIO
const byte OpenPin = 12;                    // Input pin used for opening the window (C button on remote control, D2 input)
const byte ClosePin = 13;                   // Input pin used for closing the window (A button on remote control, D0 input)
const byte StorePin = 14;                   // Input pin used for storing the memory position of the window (D button on remote control, D1 input)
const byte StopPin = 15;                    // Input pin used for stopping the motor (B button on remote control, D3 input)
const byte DirectionPin = 0;                // When high, the motor will open the window, low closes the window
const byte PowerPin = 2;                    // When high turns on power to the motor
const int buttonDelay = 20;                 // Number of milliseconds a button input has to be high before a button press is registered.
// Pins 4 and 5 are used for I2C

// Regional settings. Adjust these to match your location
const float Latitude = 0.0000;              // Specifies the north�south position of your location
const float Longitude = 0.0000;             // Specifies the east-west position of your location
int Timezone = 60;                          // UTC difference in minutes (can be changed through web page)
bool UseDST = true;                         // Indicates whether Daylight Saving Time is observed in your region or not.

// Opening and closing times (can be changed through the web page)
uint8_t HourOpen = 0;                       // The time the window opens
uint8_t MinuteOpen = 30;
uint8_t HourClose = 6;                      // The time the window closes
uint8_t MinuteClose = 0;
uint8_t WeekendHourOpen = 1;                // The time the window opens in the weekend
uint8_t WeekendMinuteOpen = 0;
uint8_t WeekendHourClose = 6;               // The time the window closes in the weekend
uint8_t WeekendMinuteClose = 30;

// For the Wifi connection. Replace with your network credentials
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";
const time_t connectPeriod = 90;            // Max period (in s) for setting up a wifi connection

// NTP stuff
const char* NtpServer = "time.windows.com"; // A reliable NTP server ("time.nist.gov" didn't work so well)
const unsigned int localPort = 2390;        // Local port to listen for UDP packets
const int NtpPacketSize = 48;               // NTP time stamp is in the first 48 bytes of the message
const time_t UpdateTimeTimeout = 300;       // Time (in seconds) we will try updating the clock

// Email credentials. Replace with yours
const char* fromAddress = "REPLACE_WITH_YOUR_EMAIL";      // The address you want alarm messages sent from
const char* toAddress = "REPLACE_WITH_YOUR_EMAIL";        // The email address you want alarm messages sent to
const char* smtpServer = "REPLACE_WITH_YOUR_SERVER";      // The server to use for sending email
const char* emailAccount = "REPLACE_WITH_YOUR_ACCOUNT";   // The email account on the server
const char* emailPassword = "REPLACE_WITH_YOUR_PASSWORD"; // The password for the email account

// Stuff related to controlling the motor
int CurrentLimit = 750;                     // The maximum current that may be drawn. If exceeded the software will stop the motor.
int OpenMillis = 18000;                     // The time needed to open the window
int CloseMillis = 18000;                    // The time needed to fully close the window
int LastOpenMillis = 0;                     // The duration of the latest opening motion
int MemoryMillis = 0;                       // The time it takes to open the window to the memory position
const int DirDelay = 500;                   // Delay to prevent abrupt motor direction changes

// INA226
#define I2C_ADDRESS 0x40

// For reading the temperature
bool Bme280Present = false;                 // Initialize/use the BME280 or not
const bool UseTemperature = false;          // If true, the window will close when the temperature drops below ClosingTemperature
const float ClosingTemperature = 0;         // Close the window at night when the temperature goes below this value
const time_t TempCheckPeriod = 180;         // Number of seconds between checks

// ************************************************************************
// Change the constants above to adjust the software to your situation
// ************************************************************************

enum MotorCommand {
  Stop,
  OpenDir,
  CloseDir,
};

enum UserCommand {
  OpenCmd,
  CloseCmd,
  StoreCmd,
  StopCmd,
  Open1SecCmd,
  Close1SecCmd,
  NoCmd
};

// Stuff related to the state machine controlling the window
enum StateMachineState {
  NotRunning = 0,
  Stopped = 1,
  Closed = 2,
  MovingIntoSensor = 3,
  ClearingSensor = 4,
  Opening = 5,
  Open = 6,
  Closing = 7,
  Moving1Sec = 8
};

// Set web server port number to 80
WiFiServer server(80);
WiFiClient client;
bool clientActive = false;                  // A web client is active
String currentLine = "";                    // A string to hold incoming data from the client
time_t clientConnectedAt;
const time_t webConnectPeriod = 30;         // Max timeout period (in s) for HTTP connections

// Variable to store the HTTP request
String header;

WiFiUDP Udp;                                // A UDP instance for sending and receiving packets over UDP
byte packetBuffer[NtpPacketSize];           // Buffer to hold incoming and outgoing packets
time_t updateTimeStarted;                   // The time when we started updating the clock
bool ntpTimeSet = false;                    // Has the time been set through NTP?
bool emailSent = false;                     // Has an email been sent because setting the time failed?
int lastSecond = -1;                        // The second we last did stuff for NTP
bool settingSunRiseSunSet = false;          // True when setting the sunrise and sunset

// The SMTP session object used for email sending
SMTPSession smtp;

// Callback function to get the email sending status
void smtpCallback(SMTP_Status status);

// Daylight Saving Time (0 or 60 minutes, calculated from current date)
int DST = 0;

// For sunrise and sunset
sunMoon sm;
time_t sunRise = 0;
time_t sunSet = 0;

// Related to main state machine
enum StateMachineState stateMachineState = NotRunning;
enum StateMachineState previousStateMachineState = NotRunning;
bool stateChanged = true;                       // Start with entry code
UserCommand userCommand = NoCmd;                // Command given through a button press or through the web interface
unsigned long startMovingMillis = 0;
bool buttonPressed;
unsigned long buttonPressedAt;
bool buttonHandled;
bool storeButtonPressed;
unsigned long storeButtonPressedAt;

// Power measurement
INA226 ina226(I2C_ADDRESS);
unsigned long prevTime;
unsigned long startMotion;
bool moving = false;
float maxCurrent = 0.0;

// The class that controls the BME280 sensor
Adafruit_BME280 bme;
volatile time_t previousCheckAt = 0;
float temperature;
float humidity;                             // Informational
float pressure;                             // Informational

String debugText = "";                      // A string to hold all debug text

// The setup function that initializes everything
void setup() {
  Serial.begin(115200);

  // Initialize GPIO
  pinMode(OpenPin, INPUT);
  pinMode(ClosePin, INPUT);
  pinMode(StorePin, INPUT);
  pinMode(StopPin, INPUT);
  pinMode(DirectionPin, OUTPUT);
  pinMode(PowerPin, OUTPUT);
  digitalWrite(DirectionPin, LOW);
  digitalWrite(PowerPin, LOW);

  // Initialize Wi-Fi. Force the ESP to reset Wi-Fi and initialize correctly.
  Serial.print("WiFi status = ");
  Serial.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("Wi-Fi status = ");
  Serial.println(WiFi.getMode());
  // End Wi-Fi initialization

  // Connect to Wi-Fi network with SSID and password. Reboot if it continuously fails.
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Initialize Wi-Fi
  WiFi.begin(ssid, password);

  time_t firstCheckAt = now();
  while ((WiFi.waitForConnectResult() != WL_CONNECTED) && (now() < firstCheckAt + connectPeriod)) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    // Print local IP address and start web server
    Serial.println("Wi-Fi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Print the SSID of the network you're attached to
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // Print the received signal strength
    long rssi = WiFi.RSSI();
    Serial.print("Signal strength (RSSI): ");
    Serial.print(rssi);
    Serial.println(" dBm");
  } else {
    WiFi.disconnect();
    Serial.println();
    Serial.println("No Wi-Fi connection established, rebooting");
    delay(5000);
    ESP.restart();
  }

  // Initialize I2C and power measurement device
  Wire.begin();
  if (!ina226.begin()) {
    Serial.println("No connection with INA226, rebooting");
    //delay(5000);
    //ESP.restart();
  } else {
    ina226.setAverage(1); 
    ina226.setMaxCurrentShunt(15, 0.002);
  }

  // When using the temperature, initialize the BME280
  if (Bme280Present) {
    // Check for the BME280 sensor
    printLine("Starting BME280 sensor");
    if (bme.begin(0x76)) {
      // Scenario for weather monitoring
      printLine("Setting BME280 to weather station scenario:");
      printLine("  Forced mode, 1x temperature / 1x humidity / 1x pressure oversampling, filter off");
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1,  // Temperature
        Adafruit_BME280::SAMPLING_X1,  // Pressure
        Adafruit_BME280::SAMPLING_X1,  // Humidity
        Adafruit_BME280::FILTER_OFF);
    } else {
      printLine("Could not find a valid BME280 sensor, please check wiring.");
      Bme280Present = false;
    }
  }

  // The begin() call is required to initialize the EEPROM library
  EEPROM.begin(32);

  // Read from EEPROM
  bool dataPresent = false;
  EEPROM.get(0, dataPresent);
  if (dataPresent) {
    EEPROM.get(1, Timezone);
    EEPROM.get(5, UseDST);
    EEPROM.get(6, HourOpen);
    EEPROM.get(7, MinuteOpen);
    EEPROM.get(8, HourClose);
    EEPROM.get(9, MinuteClose);
    EEPROM.get(10, WeekendHourOpen);
    EEPROM.get(11, WeekendMinuteOpen);
    EEPROM.get(12, WeekendHourClose);
    EEPROM.get(13, WeekendMinuteClose);
    EEPROM.get(14, OpenMillis);
    EEPROM.get(18, CloseMillis);
    EEPROM.get(22, MemoryMillis);
    EEPROM.get(26, CurrentLimit);
  }

  // For NTP
  Udp.begin(localPort);
  printLine("Started listening for UDP packets");
  updateTimeStarted = now();

  // Set the callback function to get email sending results
  smtp.callback(smtpCallback);

  // Start the webserver
  server.begin();
}

// Send an NTP request to the time server at the given address
int sendNtpPacket(const char* host) {
  int result = 0;

  // Set all bytes in the buffer to 0
  memset(packetBuffer, 0, NtpPacketSize);

  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode

  // All needed NTP fields have been given values, now
  // send a packet requesting a timestamp
  result = Udp.beginPacket(host, 123);  // NTP requests are to port 123
  Udp.write(packetBuffer, NtpPacketSize);
  Udp.endPacket();

  return result;
}

time_t getNtpTime() {
  time_t result = 0;

  // Check if a reply is available
  if (Udp.parsePacket()) {
    printLine("Packet received");

    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NtpPacketSize);  // Read the packet into the buffer

    // The timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);

    // Combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    printNoLine("Seconds since Jan 1 1900 = ");
    printLine(String(secsSince1900));

    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;

    // Subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;

    printNoLine("UTC time is: ");
    printDateTime(epoch);
    printLine("");

    result = epoch;
  }

  return result;
}

// Daylight Saving Time starts at 2 a.m. on the last Sunday in March. The clock is then set 1 hour
// ahead to 3 o'clock. So this Sunday lasts only 23 hours; an hour shorter than a normal day.
// Winter time starts at 3 a.m. on the last Sunday in the month of October. The time is
// then reset 1 hour to 2 hours. In practice this means that this day lasts 25 hours.
// This function determines whether DST is active on the date specified by parameter 'time'.
// Note that it does not look at the time to make this determination, so it returns true for the
// last Sunday in March, regardless of the time, and false for the last Sunday in October.
// Note: dayOfWeek(SUN) = 1, dayOfWeek(MON) = 2, etc.
bool dstActive(time_t time) {
  bool result = false;
  int dayParam = day(time);
  int monthParam = month(time);
  int dayOfWeekParam = dayOfWeek(time);

  // Check if today is a DST day
  if ((monthParam == 3 && dayParam >= 25 && dayOfWeekParam <= dayParam - 24) ||
      (monthParam == 10 && (dayParam < 25 || (dayParam >= 25 && dayParam - dayOfWeekParam <= 23))) ||
      (monthParam > 3 && monthParam < 10)) {
    result = true;
  }

  return result;
}

bool isWeekendDay() {
  return (dayOfWeek(now()) == 1 || dayOfWeek(now()) == 7);
}

// Two functions to handle debug messages
void printNoLine(String text) {
  if (debugText.length() > 23800) {
    debugText.remove(0, 1000);
  }
  debugText += text;
  Serial.print(text);
}

void printLine(String text) {
  if (debugText.length() > 23800) {
    debugText.remove(0, 1000);
  }
  debugText += text + "<br>\n";
  Serial.println(text);
}

void printDateTime(time_t date) {
  char buff[20];
  sprintf(buff, "%2d-%02d-%4d %02d:%02d:%02d",
    day(date), month(date), year(date), hour(date), minute(date), second(date));
  printNoLine(buff);
}

void moveWindow(MotorCommand command) {
  // Always stop motor before running it to avoid abrupt changes
  digitalWrite(PowerPin, LOW);
  digitalWrite(DirectionPin, LOW);
  delay(DirDelay);

  // Write new motor command and speed
  if (command == OpenDir) {
    printLine("Opening window...");
    ina226.setModeShuntBusContinuous();
    digitalWrite(DirectionPin, HIGH);
    digitalWrite(PowerPin, HIGH);
  } else if (command == CloseDir) {
    printLine("Closing window...");
    ina226.setModeShuntBusContinuous();
    digitalWrite(PowerPin, HIGH);
  } else if (command == Stop) {
    printLine("Stop...");
    ina226.shutDown();
    // Stopping already done at top of function
  }
}

// Constantly check for the motor going over the maximum current when it is running.
// If something is blocking the window, the current will go above the limit and
// the motor must be stopped.
// The first 60 or so milliseconds after starting the motor the current flowing is
// really high, around 1.1 A. After that it drops to around 400 mA when closing the
// window, 600 mA when opening. When the window is blocked the current goes over CurrentLimit mA.
bool checkCurrentWithinLimits() {
  float current_mA = 0.0;
  float shuntVoltage_mV = 0.0;
  float busVoltage_V = 0.0;
  float power_mW = 0.0;
  unsigned long time = millis();

  current_mA = ina226.getCurrent_mA();

  if (current_mA > maxCurrent) {
    maxCurrent = current_mA;
  }

  // Check the current to test if the motor is running
  if (!moving && current_mA > 100) {
    startMotion = time;
    moving = true;
  }

  // Print the current, etc.
  if (time - prevTime > 500) {
    prevTime = time;

    shuntVoltage_mV = ina226.getShuntVoltage_mV();
    busVoltage_V = ina226.getBusVoltage_mV();
    power_mW = ina226.getPower_mW();

    Serial.print(" Current: ");
    Serial.print(current_mA);
    Serial.print(" mA | Shunt: ");
    Serial.print(shuntVoltage_mV);
    Serial.print(" mV | Bus: ");
    Serial.print(busVoltage_V);
    Serial.print(" mV | Power: ");
    Serial.print(power_mW);
    Serial.print(" mW");
    Serial.println();
  }

  if (moving && time - startMotion > 50) {
    // The current should have dropped by now
    if (current_mA > CurrentLimit) {
      printNoLine(" Current: ");
      printNoLine(String(current_mA));
      printNoLine(" mA | Time: ");
      printNoLine(String(time - startMotion));
      printLine(" ms (overflow)");
      return false;
    }
  }

  // Useful for calibrating the current limit
  if (moving && time - startMotion < 100) {
    Serial.print(" Current: ");
    Serial.print(current_mA);
    Serial.print(" mA | Time: ");
    Serial.print(time - startMotion);
    Serial.println(" ms");
  }

  return true;
}

void stopCheckingCurrentWithinLimits() {
  Serial.print(" Max. current: ");
  Serial.print(String(maxCurrent));
  Serial.println(" mA");

  moving = false;
  maxCurrent = 0.0;
}

void checkButtons() {
  // Out of an abundance of caution, make sure we don't interpret noise on the button input ports as a button press.
  if (digitalRead(OpenPin) == HIGH || digitalRead(ClosePin) == HIGH || digitalRead(StorePin) == HIGH || digitalRead(StopPin) == HIGH) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressedAt = millis();
      buttonHandled = false;
    }
  } else {
    // Button was released, reset
    buttonPressed = false;
  }

  if (buttonPressed && !buttonHandled) {  // && millis() > buttonPressedAt + buttonDelay) {
    if (digitalRead(StopPin) == HIGH) {
      userCommand = StopCmd;
      printLine("Stop button pressed");
    } else if (digitalRead(ClosePin) == HIGH) {
      userCommand = CloseCmd;
      printLine("Close button pressed");
    } else if (digitalRead(OpenPin) == HIGH) {
      userCommand = OpenCmd;
      printLine("Open button pressed");
    } else if (digitalRead(StorePin) == HIGH) {
      userCommand = StoreCmd;
      printLine("Store button pressed");
    }

    buttonHandled = true;
  }
}

void handleWebClient() {
  char buf[20];

  if (!clientActive) {
    client = server.available();  // Listen for incoming clients
    if (client) {                 // If a new client connects
      clientConnectedAt = now();
      printNoLine("New client connection at ");
      printDateTime(clientConnectedAt);
      printLine("");
      clientActive = true;
      currentLine = "";
    }
  } else {
    if (client.connected()) {    // Check if the client is still connected
      if (client.available()) {  // If there's bytes to read from the client,
        char c = client.read();  // read a byte, then
        Serial.write(c);         // print it out the serial monitor
        header += c;
        if (c == '\n') {         // If the byte is a newline character
          // If the current line is blank, you got two newline characters in a row.
          // That's the end of the client HTTP request, so send a response
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Handle user input
            if (header.indexOf("GET /close1sec") >= 0) {
              printLine("Closed for 1 second selected");
              userCommand = Open1SecCmd;
            } else if (header.indexOf("GET /open1sec") >= 0) {
              printLine("Open for 1 second selected");
              userCommand = Close1SecCmd;
            } else if (header.indexOf("GET /open") >= 0) {
              printLine("Open selected");
              userCommand = OpenCmd;
            } else if (header.indexOf("GET /close") >= 0) {
              printLine("Closed selected");
              userCommand = CloseCmd;
            } else if (header.indexOf("GET /stop") >= 0) {
              printLine("Stop selected");
              userCommand = StopCmd;
            } else if (header.indexOf("GET /store") >= 0) {
              printLine("Store position selected");
              userCommand = StoreCmd;
            } else if (header.indexOf("GET /params") >= 0) {
              // Find first parameter
              int begin = header.indexOf('?');
              int end = header.indexOf('&');
              // printNoLine("begin=");
              // printNoLine(String(begin));
              // printNoLine(" end=");
              // printLine(String(end));
              if (begin > -1 && end > begin + 1) {
                String sub = header.substring(begin + 1, end);
                // printNoLine("Substring: ");
                // printLine(sub);

                // Split parameter in name and value. Repeat for all parameters
                bool dataPresent = true;
                bool invalidParam = false;

                UseDST = false;

                do {
                  int equalsPos = sub.indexOf('=');
                  if (equalsPos > -1) {
                    String param = sub.substring(0, equalsPos);
                    String value = sub.substring(equalsPos + 1, sub.length());

                    // Check for and set the variables
                    long temp;
                    if (param.indexOf("Revert") == 0) {
                      dataPresent = false;
                    } else if (param.indexOf("Timezone") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) Timezone = temp;
                    } else if (param.indexOf("UseDST") == 0) {
                      UseDST = true;
                    } else if (param.indexOf("HourOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) HourOpen = temp;
                    } else if (param.indexOf("MinuteOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) MinuteOpen = temp;
                    } else if (param.indexOf("HourClose") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) HourClose = temp;
                    } else if (param.indexOf("MinuteClose") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) MinuteClose = temp;
                    } else if (param.indexOf("WeekendHourOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) WeekendHourOpen = temp;
                    } else if (param.indexOf("WeekendMinuteOpen") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) WeekendMinuteOpen = temp;
                    } else if (param.indexOf("WeekendHourClose") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) WeekendHourClose = temp;
                    } else if (param.indexOf("WeekendMinuteClose") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) WeekendMinuteClose = temp;
                    } else if (param.indexOf("OpenMillis") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) OpenMillis = temp;
                    } else if (param.indexOf("CloseMillis") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) CloseMillis = temp;
                    } else if (param.indexOf("MemoryMillis") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) MemoryMillis = temp;
                    } else if (param.indexOf("CurrentLimit") == 0) {
                      temp = value.toInt();
                      if (temp >= 0) CurrentLimit = temp;
                    } else {
                      invalidParam = true;
                    }
                  } else {
                    invalidParam = true;
                  }

                  // Go to next parameter
                  begin = end + 1;
                  end = header.indexOf('&', begin);
                  if (end < 0 || end > header.indexOf(' ', begin)) {
                    end = header.indexOf(' ', begin);
                    if (end < 0) {
                      end = header.length();
                    }
                  }

                  if (end > begin + 1) {
                    sub = header.substring(begin, end);
                  }
                } while (!invalidParam && end > begin + 1);

                // Put new settings into EEPROM
                EEPROM.put(0, dataPresent);
                EEPROM.put(1, Timezone);
                EEPROM.put(5, UseDST);
                EEPROM.put(6, HourOpen);
                EEPROM.put(7, MinuteOpen);
                EEPROM.put(8, HourClose);
                EEPROM.put(9, MinuteClose);
                EEPROM.put(10, WeekendHourOpen);
                EEPROM.put(11, WeekendMinuteOpen);
                EEPROM.put(12, WeekendHourClose);
                EEPROM.put(13, WeekendMinuteClose);
                EEPROM.put(14, OpenMillis);
                EEPROM.put(18, CloseMillis);
                EEPROM.put(22, MemoryMillis);
                EEPROM.put(26, CurrentLimit);

                // Write the data to EEPROM
                bool ok = EEPROM.commit();
                printLine((ok) ? "Commit OK" : "Commit failed");
              }
            }

            time_t t_now = now();
            printDateTime(t_now);
            printLine("");

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");

            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP8266 Web Server</h1>");
            client.print("<p>Today date/time: ");
            sprintf(buf, "%2d-%02d-%4d %02d:%02d:%02d",
              day(t_now), month(t_now), year(t_now), hour(t_now), minute(t_now), second(t_now));
            client.print(buf);
            client.println("</p>");

            client.print("<p>Today's sunrise and sunset: ");
            sprintf(buf, "%2d-%02d-%4d %02d:%02d:%02d",
              day(sunRise), month(sunRise), year(sunRise), hour(sunRise), minute(sunRise), second(sunRise));
            client.print(buf);
            client.print(", ");
            sprintf(buf, "%2d-%02d-%4d %02d:%02d:%02d",
              day(sunSet), month(sunSet), year(sunSet), hour(sunSet), minute(sunSet), second(sunSet));
            client.print(buf);
            client.println("</p>");

            client.print("<p>Today's opening time: ");
            if (isWeekendDay()) {
              sprintf(buf, "%02d:%02d", WeekendHourOpen, WeekendMinuteOpen);
            } else {
              sprintf(buf, "%02d:%02d", HourOpen, MinuteOpen);
            }
            client.print(buf);
            client.println("</p>");

            client.print("<p>Today's closing time: ");
            if (isWeekendDay()) {
              sprintf(buf, "%02d:%02d", WeekendHourClose, WeekendMinuteClose);
            } else {
              sprintf(buf, "%02d:%02d", HourClose, MinuteClose);
            }
            client.print(buf);
            client.println("</p>");

            if (Bme280Present) {
              client.print("<p>Current temperature, humidity and pressure: ");
              sprintf(buf, "%0.1f*C, ", temperature);
              client.print(buf);
              sprintf(buf, "%0.f%%, ", humidity);
              client.print(buf);
              sprintf(buf, "%0.1f hPa", pressure);
              client.print(buf);
              client.println("</p>");
            }

            //
            client.print("<p>State: ");
            switch (stateMachineState) {
              case NotRunning:
                client.print("NotRunning</p>");
                break;
              case Stopped:
                client.print("Stopped</p>");
                break;
              case Closed:
                client.print("Closed</p>");
                client.println("<p><a href=\"/open\"><button class=\"button\">Open</button></a></p>");
                client.println("<p><a href=\"/close1sec\"><button class=\"button\">Close 1 Second</button></a></p>");
                client.println("<p><a href=\"/open1sec\"><button class=\"button\">Open 1 Second</button></a></p>");
                break;
              case Opening:
                client.print("Opening</p>");
                client.println("<p><a href=\"/stop\"><button class=\"button\">Stop</button></a></p>");
                break;
              case Open:
                client.print("Open</p>");
                client.println("<p><a href=\"/close\"><button class=\"button\">Close</button></a></p>");
                client.println("<p><a href=\"/close1sec\"><button class=\"button\">Close 1 Second</button></a></p>");
                client.println("<p><a href=\"/open1sec\"><button class=\"button\">Open 1 Second</button></a></p>");
                break;
              case Closing:
                client.print("Closing</p>");
                client.println("<p><a href=\"/stop\"><button class=\"button\">Stop</button></a></p>");
                break;
              case Moving1Sec:
                client.print("Moving1Sec</p>");
                break;
              default:
                client.print("Unknown</p>");
                break;
            }
            client.println("<p><a href=\"/\"><button class=\"button\">Refresh</button></a></p><br>");

            // Print the form for uploading settings
            client.println("<form action=\"/params\">");
            client.println("<p>Revert to default values after reset:<input type=\"checkbox\" name=\"Revert\"></p>");
            client.print("<p>Timezone:<input type=\"text\" name=\"Timezone\" value=\"");
            client.print(Timezone);
            client.println("\"></p>");
            client.print("<p>Observe DST:<input type=\"checkbox\" name=\"UseDST\"");
            if (UseDST) client.print(" checked");
            client.println("></p>");
            client.print("<p>HourOpen:<input type=\"text\" name=\"HourOpen\" value=\"");
            client.print(HourOpen);
            client.println("\"></p>");
            client.print("<p>MinuteOpen:<input type=\"text\" name=\"MinuteOpen\" value=\"");
            client.print(MinuteOpen);
            client.println("\"></p>");
            client.print("<p>HourClose:<input type=\"text\" name=\"HourClose\" value=\"");
            client.print(HourClose);
            client.println("\"></p>");
            client.print("<p>MinuteClose:<input type=\"text\" name=\"MinuteClose\" value=\"");
            client.print(MinuteClose);
            client.println("\"></p>");
            client.print("<p>WeekendHourOpen:<input type=\"text\" name=\"WeekendHourOpen\" value=\"");
            client.print(WeekendHourOpen);
            client.println("\"></p>");
            client.print("<p>WeekendMinuteOpen:<input type=\"text\" name=\"WeekendMinuteOpen\" value=\"");
            client.print(WeekendMinuteOpen);
            client.println("\"></p>");
            client.print("<p>WeekendHourClose:<input type=\"text\" name=\"WeekendHourClose\" value=\"");
            client.print(WeekendHourClose);
            client.println("\"></p>");
            client.print("<p>WeekendMinuteClose:<input type=\"text\" name=\"WeekendMinuteClose\" value=\"");
            client.print(WeekendMinuteClose);
            client.println("\"></p>");
            client.print("<p>OpenMillis:<input type=\"text\" name=\"OpenMillis\" value=\"");
            client.print(OpenMillis);
            client.println("\"></p>");
            client.print("<p>CloseMillis:<input type=\"text\" name=\"CloseMillis\" value=\"");
            client.print(CloseMillis);
            client.println("\"></p>");
            client.print("<p>MemoryMillis:<input type=\"text\" name=\"MemoryMillis\" value=\"");
            client.print(MemoryMillis);
            client.println("\"></p>");
            client.print("<p>CurrentLimit:<input type=\"text\" name=\"CurrentLimit\" value=\"");
            client.print(CurrentLimit);
            client.println("\"></p>");
            client.println("<p><input type=\"submit\" value=\"Submit\"></p>");
            client.println("</form>");

            client.print("<p>");
            client.print(debugText);
            client.println("</p>");

            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();

            // Clear the header variable
            header = "";

            // Close the connection
            client.stop();
            printLine("Client disconnected");
            printLine("");

            clientActive = false;
          } else {  // If you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // If you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      } else {
        // The client is connected but no data is available. Time out if this takes too long.
        if (now() > clientConnectedAt + webConnectPeriod) {
          clientActive = false;
          printLine("Client connection timed out");
        }
      }
    } else {
      clientActive = false;
      printLine("Client no longer connected");
    }
  }
}

// Calculate the next sunrise and sunset
void setSunriseSunset() {
  // Initialize sunMoon
  sm.init(Timezone, Latitude, Longitude);
  sunRise = sm.sunRise() + DST * 60;
  sunSet = sm.sunSet() + DST * 60;
  printNoLine("Today's sunrise and sunset: ");
  printDateTime(sunRise);
  printNoLine(", ");
  printDateTime(sunSet);
  printLine("");
  printLine("");
}

void sendEmail(int selectMessage) {
  ESP_Mail_Session session;  // Session config data
  SMTP_Message message;      // Message class

  // Set the session config
  session.server.host_name = smtpServer;
  session.server.port = 587;
  session.login.email = emailAccount;
  session.login.password = emailPassword;
  session.login.user_domain = "";

  // Set message headers
  message.sender.name = "WindowOpener";
  message.sender.email = fromAddress;
  message.subject = "Problem with window opener";
  message.addRecipient("Me", toAddress);

  // Create raw text message
  if (selectMessage == 1) {
    message.text.content = "The clock could not be set.";
  } else if (selectMessage == 2) {
    message.text.content = "Calibration failed.";
  }

  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;

  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;
  message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

  // Connect to server with the session config
  if (!smtp.connect(&session)) {
    return;
  }

  // Send email and close the session
  if (!MailClient.sendMail(&smtp, &message)) {
    printLine("Error sending Email, " + smtp.errorReason());
  }
}

// Callback function to get the email sending status
void smtpCallback(SMTP_Status status) {
  // Print the current status
  printLine(status.info());

  /* Print the sending result */
  if (status.success()) {
    printLine("Mail sent OK");
  }
}

void loop() {
  // Set the time when not set
  if (!ntpTimeSet) {
    // If not set, check for the time, but only for a few minutes.
    // In order not to do these checks every loop, proceed only once a second
    if ((now() < updateTimeStarted + UpdateTimeTimeout) && (lastSecond != second())) {
      // Send a packet every 30 seconds and check for an answer the other times
      if (second() % 30 == 8) {
        printNoLine("Requesting time from NTP server at ");
        printDateTime(now());
        printLine("");

        // Send an NTP packet to a time server
        if (sendNtpPacket(NtpServer) == 0) {
          printNoLine("DNS lookup failed for ");
          printLine(NtpServer);
        }
      } else {
        // Check for received packets
        time_t epoch = getNtpTime();
        if (epoch > 0) {
          printNoLine("Daylight Saving Time ");
          if (UseDST) {
            printNoLine("observed ");
            if (dstActive(epoch)) {
              DST = 60;
              printLine("and active");
            } else {
              DST = 0;
              printLine("but not active");
            }
          } else {
            printLine("not observed");
          }

          printNoLine("Setting system time to UTC + ");
          printNoLine(String(Timezone + DST));
          printLine(" minutes");
          setTime(epoch + (Timezone + DST) * 60);
          ntpTimeSet = true;

          setSunriseSunset();

          // Time was set, start the state machine for the window
          stateMachineState = Stopped;
          stateChanged = (stateMachineState != previousStateMachineState);
          previousStateMachineState = stateMachineState;
        }
      }
      lastSecond = second();
    } else if (now() > updateTimeStarted + 3600) {
      // After an hour, try it again.
      updateTimeStarted = now();
    } else if (now() > updateTimeStarted + UpdateTimeTimeout) {
      // No time packet was received, send a warning email once.
      // Chances are the internet connection is down anyway.
      if (!emailSent) {
        emailSent = true;
        sendEmail(1);
      }
    }
  } else {
    // Make sure the clock is updated once a week on Sunday mornings.
    // This also allows for DST to be set correctly. Note that time can move
    // back or forward by one hour.
    if (dayOfWeek(now()) == 1 && hour() == 1 && minute() == 10 && second() == 0) {
      printNoLine("Invalidating time at ");
      printDateTime(now());
      printLine("");
      ntpTimeSet = false;
      updateTimeStarted = now();
      emailSent = false;
    }
  }

  time_t t_now = now();  // The number of seconds since Jan 1 1970

  // Every day at two thirty a.m. calculate the next sunrise and sunset and the time for closing the door
  if (!settingSunRiseSunSet && hour() == 2 && minute() == 30 && second() == 0) {
    settingSunRiseSunSet = true;
    printNoLine("Calculating sunrise and sunset at ");
    printDateTime(t_now);
    printLine("");

    setSunriseSunset();
  } else if (settingSunRiseSunSet && second() != 0) {
    settingSunRiseSunSet = false;
  }

  // Check if a button on the RC was pressed. Only run in stable state,
  // because a state change can be requested.
  if (!stateChanged) {
    checkButtons();
  }

  // Web server stuff. Only run in stable state, because a state change can be requested.
  if (!stateChanged) {
    handleWebClient();
  }

  // Update the temperature every TempCheckPeriod seconds
  if (Bme280Present && (t_now >= previousCheckAt + TempCheckPeriod)) {
    previousCheckAt = t_now;

    // Print timer interrupt count and time
    // printNoLine("Reading BME280 at ");
    // printDateTime(now());
    // printLine("");

    // Start measurement
    bme.takeForcedMeasurement();

    // Temperature
    temperature = bme.readTemperature() - 2;      // Correction for my BME280
    humidity = bme.readHumidity() - 9.0;          // Correction for my BME280
    pressure = bme.readPressure() / 100.0F;       // Correction for my BME280
    printf("Temperature: %0.1f*C, humidity: %0.f%%, pressure: %0.1f hPa\r\n", temperature, humidity, pressure);
  }

  //******************************************************************
  // The state machine for controlling the window of the chicken coop //
  //******************************************************************

  switch (stateMachineState) {
    //==============================================================
    // State 'Stopped'
    //==============================================================
    case Stopped:

      // Actions on entry
      if (stateChanged) {
        moveWindow(Stop);
        printDateTime(t_now);
        printLine(" State: Stopped");
        printLine("");
      }

      // Transitions
      if (!stateChanged) {
        // As we don't know where we are, we leave the window as it is.
        // We just assume it is in the right place to handle the next timed action.

        // Open the window at the specified time except when the temperature is too low
        if (((!UseTemperature || temperature > ClosingTemperature) &&
             ((!isWeekendDay() && (hour() == HourOpen) && (minute() == MinuteOpen)) ||
              (isWeekendDay() && (hour() == WeekendHourOpen) && (minute() == WeekendMinuteOpen)))) ||
            (userCommand == OpenCmd)) {
          userCommand = NoCmd;
          stateMachineState = Opening;
        }

        // Close the window at the specified time or when the temperature becomes too low
        if ((UseTemperature && temperature <= ClosingTemperature) ||
            (!isWeekendDay() && (hour() == HourClose) && (minute() == MinuteClose)) ||
            (isWeekendDay() && (hour() == WeekendHourClose) && (minute() == WeekendMinuteClose)) ||
            (userCommand == CloseCmd)) {
          userCommand = NoCmd;
          stateMachineState = Closing;
        }
        
        // Manual moves
        if (userCommand == Open1SecCmd || userCommand == Close1SecCmd) {
          stateMachineState = Moving1Sec;
        }
      }
      break;

    //==============================================================
    // State 'Closed'
    //==============================================================
    case Closed:

      // Actions on entry
      if (stateChanged) {
        moveWindow(Stop);
        printDateTime(t_now);
        printLine(" State: Closed");
        printLine("");
      }

      // Continuous actions
      // Check for the Store button
      if (userCommand == StoreCmd) {
        if (!storeButtonPressed) {
          storeButtonPressed = true;
          storeButtonPressedAt = millis();
        }
      } else {
        storeButtonPressed = false;
      }

      // Check if the Store button is being held for more than 2 seconds, if so reset the memory position
      if (storeButtonPressed && millis() > storeButtonPressedAt + 2000) {
        userCommand = NoCmd;
        storeButtonPressed = false;

        MemoryMillis = 0;

        // Write the new memory position to EEPROM
        EEPROM.put(22, MemoryMillis);
        if (EEPROM.commit()) {
          printNoLine("New memory position stored: ");
          printLine(String(MemoryMillis));
        } else {
          printLine("New memory position not stored");
        }
      }

      // Transitions
      if (!stateChanged) {
        // Open the window at the specified time except when the temperature is too low
        if (((!UseTemperature || temperature > ClosingTemperature) && 
            ((!isWeekendDay() && (hour() == HourOpen) && (minute() == MinuteOpen)) ||
             (isWeekendDay() && (hour() == WeekendHourOpen) && (minute() == WeekendMinuteOpen)))) ||
            (userCommand == OpenCmd)) {
          userCommand = NoCmd;
          stateMachineState = Opening;
        } else if (userCommand == CloseCmd) {
          userCommand = NoCmd;
          stateMachineState = Closing;
        } else if (userCommand == Open1SecCmd || userCommand == Close1SecCmd) {
          stateMachineState = Moving1Sec;
        }
      }
      break;

    //==============================================================
    // State 'Opening'
    //==============================================================
    case Opening:

      // Actions on entry
      if (stateChanged) {
        printDateTime(t_now);
        printLine(" State: Opening");

        startMovingMillis = millis();
        moveWindow(OpenDir);
      }

      // Continuous actions

      // Transitions
      if (!stateChanged) {
        if (!checkCurrentWithinLimits()) {
          stopCheckingCurrentWithinLimits();
          userCommand = Close1SecCmd;
          stateMachineState = Moving1Sec;
        } else if ((MemoryMillis > 0 && millis() - startMovingMillis > MemoryMillis) || (millis() - startMovingMillis > OpenMillis) || (userCommand == StopCmd)) {
          stopCheckingCurrentWithinLimits();
          userCommand = NoCmd;
          stateMachineState = Open;
        }
      }
      break;

    //==============================================================
    // State 'Open'
    //==============================================================
    case Open:

      // Actions on entry
      if (stateChanged) {
        LastOpenMillis = millis() - startMovingMillis;
        moveWindow(Stop);
        printDateTime(t_now);
        printLine(" State: Open");
        printLine("");
      }

      // Continuous actions
      // Check for the Store button
      if (userCommand == StoreCmd) {
        if (!storeButtonPressed) {
          storeButtonPressed = true;
          storeButtonPressedAt = millis();
        }
      } else {
        storeButtonPressed = false;
      }

      // Check if the Store button is being held for more than 2 seconds, if so store the current position of the window
      if (storeButtonPressed && millis() > storeButtonPressedAt + 2000) {
        userCommand = NoCmd;
        storeButtonPressed = false;

        MemoryMillis = LastOpenMillis;

        // Write the new memory position to EEPROM
        EEPROM.put(22, MemoryMillis);
        if (EEPROM.commit()) {
          printNoLine("New memory position stored: ");
          printLine(String(MemoryMillis));
        } else {
          printLine("New memory position not stored");
        }
      }

      // Transitions
      if (!stateChanged) {
        // Close the window at the specified time or when the temperature becomes too low
        if ((UseTemperature && (temperature <= ClosingTemperature)) ||
            (!isWeekendDay() && (hour() == HourClose) && (minute() == MinuteClose)) ||
             (isWeekendDay() && (hour() == WeekendHourClose) && (minute() == WeekendMinuteClose)) ||
            (userCommand == CloseCmd)) {
          userCommand = NoCmd;
          stateMachineState = Closing;
        } else if (userCommand == OpenCmd) {
          userCommand = NoCmd;
          stateMachineState = Opening;
        } else if (userCommand == Open1SecCmd || userCommand == Close1SecCmd) {
          stateMachineState = Moving1Sec;
        }
      }
      break;

    //==============================================================
    // State 'Closing'
    //==============================================================
    case Closing:

      // Actions on entry
      if (stateChanged) {
        printDateTime(t_now);
        printLine(" State: Closing");
        startMovingMillis = millis();
        moveWindow(CloseDir);
      }

      // Continuous actions

      // Transitions
      if (!stateChanged) {
        if (!checkCurrentWithinLimits()) {
          stopCheckingCurrentWithinLimits();
          userCommand = NoCmd;
          stateMachineState = Open;
        } else if ((millis() - startMovingMillis > CloseMillis) || (userCommand == StopCmd)) {
          stopCheckingCurrentWithinLimits();
          userCommand = NoCmd;
          stateMachineState = Closed;
        }
      }
      break;

    //==============================================================
    // State 'Moving1Sec'
    //==============================================================
    case Moving1Sec:

      // Actions on entry
      if (stateChanged) {
        printDateTime(t_now);
        printNoLine(" State: ");
        if (userCommand == Close1SecCmd) {
          printLine("Closing 1 second");
          moveWindow(CloseDir);
        } else {
          printLine("Opening 1 second");
          moveWindow(OpenDir);
        }
        userCommand = NoCmd;
        startMovingMillis = millis();
      }

      // Continuous actions

      // Transitions
      if (!stateChanged) {
        if (!checkCurrentWithinLimits() || millis() - startMovingMillis > 1000) {
          stopCheckingCurrentWithinLimits();
          stateMachineState = Stopped;
        }
      }
      break;
  }

  // Update state variables
  stateChanged = (stateMachineState != previousStateMachineState);
  previousStateMachineState = stateMachineState;

  // Wait a bit to conserve power
  if (stateMachineState == Opening || stateMachineState == Closing || stateMachineState == Moving1Sec || clientActive) {
    // Don't wait when moving or serving the web page
  } else {
    delay(10);
  }
}
