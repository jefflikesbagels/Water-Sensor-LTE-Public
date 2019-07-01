/*  
    This is a simple water float switch alert device. When the float switch reaches its lower limit,
    the Arduino will activate the Botletics SIM7000 shield and send a TCP payload to hologram.io,
    which will then route an email to the owner.
    
    Author: Jefflikesbagels (jeff@jefflikesbagels.net)
    Last Updated: 6/22/2019
    License: GNU GPL v3.0
*/

#include "Adafruit_FONA.h"

//#define DEBUG // Enable debug mode
#define SIMCOM_7000 // Botletics SIM7000
#define FONA_PWRKEY 6 // Microcontroller Power
#define FONA_RST 7 // Microcontroller Reset
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX
#define FLOAT_SWITCH 3 // Water float switch
#define GREEN_LED 4 // Water level normal LED
#define RED_LED 5 // Water level too low LED

// this is a large buffer for replies
char replybuffer[255];

// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);

// Use the following line for ESP8266 instead of the line above (comment out the one above)
//SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX, false, 256); // TX, RX, inverted logic, buffer size
SoftwareSerial *fonaSerial = &fonaSS;

// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
#ifdef SIMCOM_7000
  Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
#endif

void setup() {
  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state
  digitalWrite(FONA_PWRKEY, HIGH); // Default state
  pinMode(FONA_PWRKEY, OUTPUT);
  // Set up LED's and float switch IO
  pinMode(FLOAT_SWITCH, INPUT_PULLUP);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  Serial.begin(9600);

  // Turn on the module by pulsing PWRKEY low for a little bit
  powerOn(true);
  moduleSetup();
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  #ifdef DEBUG
  Serial.println(F("Type 1 to simulate high water level, 0 to simulate low water level."));
  #endif
}

void loop() {
  #ifdef DEBUG
  Serial.print(F("FONA> "));
  while (! Serial.available() ) {
    if (fona.available()) {
      Serial.write(fona.read());
    }
  }
  int command = Serial.parseInt();
  Serial.println(command);
  switch (command) {
  #else
  switch (digitalRead(FLOAT_SWITCH)) {
  #endif
    case 0: {
        //Serial.println(F("Water level is LOW"));
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(RED_LED, HIGH);

        // Send TCP payload to server via LTE CAT-M/NB-IoT
        char host[] = "cloudsocket.hologram.io";
        uint32_t port = 9999;
        char devicekey[] = "xxxxxxxx";
        char data[] = "Water_Low";
        char topics[] = "WATER_LOW";
        char TCPpayload[sizeof(devicekey)+sizeof(data)+sizeof(topics)+24];
        sprintf(TCPpayload, "{\"k\":\"%s\",\"d\":\"%s\",\"t\":\"%s\"}", devicekey, data, topics);
        Serial.print(F("TCP Payload: ")); Serial.print(TCPpayload); Serial.println();

        // Connect to GPRS
        fona.enableGPRS(true);

        // Connect to TCP server
        fona.TCPconnect(host, port);

        // Send TCP payload
        fona.TCPsend(TCPpayload,sizeof(TCPpayload));

        // Check if server responded
        uint16_t numBytes = fona.TCPavailable();

        // Store response from server
        char replybuffer[255];
        uint16_t bytesRead = fona.TCPread(replybuffer, 250);

        // Disconnect from TCP server
        fona.TCPclose();

        // Disconnect from GPRS
        fona.enableGPRS(false);

        // Verify response from server
        if (numBytes >= 5 && replybuffer[1] == 0x30) {
          Serial.println(F("Successfully sent TCP payload!"));

          // Wait 24 hours before sending another alert
          unsigned long countdown = 86400000;
          while (countdown > 0) {
            if (digitalRead(FLOAT_SWITCH) == 1) {
              digitalWrite(GREEN_LED, HIGH);
            } else {
              digitalWrite(GREEN_LED, LOW);
            }
            digitalWrite(RED_LED, LOW);        
            delay(500);
            digitalWrite(RED_LED, HIGH);
            delay(500);
            countdown = countdown - 1000;
          }
        } else {
          // Wait 5 seconds and try again
          Serial.print(F("Failed to send TCP payload! Response: 0x"));
          Serial.print(replybuffer[1],HEX); Serial.println();
          delay(5000);
        }
        break;
    }
    case 1: {
        //Serial.println(F("Water level is GOOD"));
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(RED_LED, LOW);
        break;
      }
  }

  #ifdef DEBUG
  while (Serial.available())
    Serial.read();
  while (fona.available()) {
    Serial.write(fona.read());
  }
  #endif
}

void moduleSetup() {
  // Serial.println(F("FONA basic test"));
  // Serial.println(F("Initializing....(May take several seconds)"));

  // Note: The SIM7000A baud rate seems to reset after being power cycled (SIMCom firmware thing)
  // SIM7000 takes about 3s to turn on but SIM7500 takes about 15s
  // Press reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset
  
  fonaSS.begin(115200); // Default SIM7000 shield baud rate

  // Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    // Serial.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1

  // Configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
  //fona.setNetworkSettings(F("your APN"), F("your username"), F("your password"));
  //fona.setNetworkSettings(F("m2m.com.attz")); // For AT&T IoT SIM card
  //fona.setNetworkSettings(F("telstra.internet")); // For Telstra (Australia) SIM card - CAT-M1 (Band 28)
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  fona.setHTTPSRedirect(true);

  /*
  // Other examples of some things you can set:
  fona.setPreferredMode(38); // Use LTE only, not 2G
  fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
  fona.setOperatingBand("CAT-M", 12); // AT&T uses band 12
//  fona.setOperatingBand("CAT-M", 13); // Verizon uses band 13
  fona.enableRTC(true);
  
  fona.enableSleepMode(true);
  fona.set_eDRX(1, 4, "0010");
  fona.enablePSM(true);

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  fona.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
  fona.setNetLED(false); // Disable network status LED
  */

#ifdef DEBUG
  // get firmware version
  fona.getVersion();

  // read the ADC
  uint16_t adc;
  if (! fona.getADCVoltage(&adc)) {
    Serial.println(F("Failed to read ADC"));
  } else {
    Serial.print(F("ADC = ")); Serial.print(adc); Serial.println(F(" mV"));
  }
  
  // read the battery voltage and percentage
  uint16_t vbat;
  if (! fona.getBattVoltage(&vbat)) {
    Serial.println(F("Failed to read Batt"));
  } else {
    Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
  }
#endif
}

// Power on the module
void powerOn(bool state) {
  if (state) {
    digitalWrite(FONA_PWRKEY, LOW);
    delay(100); // For SIM7000
    digitalWrite(FONA_PWRKEY, HIGH);
    delay(4500);
  }
  else {
    digitalWrite(FONA_PWRKEY, LOW);
    delay(1500); // For SIM7000
    digitalWrite(FONA_PWRKEY, HIGH);
    delay(4500);
  }
}
