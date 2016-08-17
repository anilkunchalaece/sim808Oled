/**
  Gps with sim808 module and Oled Disply
*/
#include <Adafruit_SleepyDog.h>
#include "Adafruit_FONA.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"

// standard pins for the shield, adjust as necessary
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

//SSD 1351  OLED Pins via SPI
#define dc   8
#define cs   10
#define rst  9

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF



SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

/************************* WiFi Access Point *********************************/
// Optionally configure a GPRS APN, username, and password.
// You might need to do this to access your network's GPRS/data
// network.  Contact your provider for the exact APN, username,
// and password values.  Username and password are optional and
// can be removed, but APN is required.
#define FONA_APN       ""
#define FONA_USERNAME  ""
#define FONA_PASSWORD  ""

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "anilkunchalaece"
#define AIO_KEY         "e996656b9fe54f9fa298816e1fb9398f"


/************ Global State (you don't need to change this!) ******************/

// Setup the FONA MQTT class by passing in the FONA class and MQTT server and login details.
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// FONAconnect is a helper function that sets up the FONA and connects to
// the GPRS network. See the fonahelper.cpp tab above for the source!
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char LAT_FEED[]     = AIO_USERNAME "/feeds/latfeed";
Adafruit_MQTT_Publish latFeed = Adafruit_MQTT_Publish(&mqtt, LAT_FEED);

const char LON_FEED[]     = AIO_USERNAME "/feeds/lonfeed";
Adafruit_MQTT_Publish lonFeed = Adafruit_MQTT_Publish(&mqtt, LON_FEED);


Adafruit_SSD1351 tft = Adafruit_SSD1351(cs, dc, rst);

/*************************** Sketch Code ************************************/

// How many transmission failures in a row we're willing to be ok with before reset
uint8_t txfailures = 0;
#define MAXTXFAILURES 3

void setup() {

  while (! Serial);

  Serial.begin(115200);
  Serial.println(F("Adafruit FONA 808 & 3G GPS demo"));
  Serial.println(F("Initializing FONA... (May take a few seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  // Initialise the FONA module
  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }

  Serial.println(F("Connected to Cellular!"));

  Watchdog.reset();
  delay(5000);  // wait a few seconds to stabilize connection
  Watchdog.reset();

  // Try to enable GPRS
  Serial.println(F("Enabling GPS..."));
  fona.enableGPS(true);

  tft.begin();
  tft.fillScreen(BLACK);
  testdrawtext("Kunchala Anil From KSRMCE KADAPA...... Innovation Lab", WHITE, 5, 5);
  testdrawtext("WELCOME", WHITE, 5, 50);

  delay(1000);
}

void loop() {
  // Make sure to reset watchdog every loop iteration!
  delay(1000);
  Watchdog.reset();
  tft.fillScreen(BLACK);

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  Watchdog.reset();

  float latitude, longitude, speed_kph, heading, speed_mph, altitude;

  // if you ask for an altitude reading, getGPS will return false if there isn't a 3D fix
  boolean gps_success = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);

  if (gps_success) {

    testdrawtext("Fix : ", WHITE, 5, 5);
    testdrawtext("1", RED, 40, 5);

    Serial.print("GPS lat:");
    Serial.println(latitude, 6);

    testdrawtext("Lat : ", WHITE, 5, 40);
    char bufLat[8];
    dtostrf(latitude, 8, 6, bufLat); //Convert float into char array using dtostrf function (Built-in AVR Function)
    testdrawtext(bufLat, WHITE, 5, 50);

    Serial.print("GPS long:");
    Serial.println(longitude, 6);

    testdrawtext("Lon : ", WHITE, 5, 60);
    char bufLon[8];
    dtostrf(longitude, 8, 6, bufLon); //Convert float into char array using dtostrf function (Built-in AVR Function)
    testdrawtext(bufLon, WHITE, 5, 70);

    Serial.print("GPS speed KPH:");
    Serial.println(speed_kph);
    Serial.print("GPS speed MPH:");
    speed_mph = speed_kph * 0.621371192;
    Serial.println(speed_mph);
    Serial.print("GPS heading:");
    Serial.println(heading);
    Serial.print("GPS altitude:");
    Serial.println(altitude);

    Serial.print(F("\nSending Lat val "));
    Serial.print(bufLat);
    Serial.print("...");
    if (! latFeed.publish(bufLat)) {
      Serial.println(F("Failed"));
      txfailures++;
    } else {
      Serial.println(F("OK!"));
      txfailures = 0;
    }
    Serial.print(F("\nSending Lon val "));
    Serial.print(bufLon);
    Serial.print("...");
    if (! lonFeed.publish(bufLon)) {
      Serial.println(F("Failed"));
      txfailures++;
    } else {
      Serial.println(F("OK!"));
      txfailures = 0;
    }
    Watchdog.reset();

  } else {
    Serial.println("Waiting for FONA GPS 3D fix...");
    testdrawtext("Fix : ", WHITE, 5, 5);
    testdrawtext("0", RED, 50, 5);
  }

  // Fona 3G doesnt have GPRSlocation :/
  if ((fona.type() == FONA3G_A) || (fona.type() == FONA3G_E))
    return;
  // Check for network, then GPRS
  Serial.println(F("Checking for Cell network..."));
  if (fona.getNetworkStatus() == 1) {
    // network & GPRS? Great! Print out the GSM location to compare
    boolean gsmloc_success = fona.getGSMLoc(&latitude, &longitude);

    if (gsmloc_success) {
      Serial.print("GSMLoc lat:");
      Serial.println(latitude, 6);

      Serial.print("GSMLoc long:");
      Serial.println(longitude, 6);
      fona.enableNetworkTimeSync(true);
      fona.enableNTPTimeSync(true, F("pool.ntp.org"));
      char timeB[23];
      if (fona.getTime(timeB, 23)) {

        testdrawtext(timeB, WHITE, 5, 15);
        Serial.println(timeB);
      }

    } else {
      Serial.println("GSM location failed...");
      Serial.println(F("Disabling GPRS"));
      fona.enableGPRS(false);
      Serial.println(F("Enabling GPRS"));
      if (!fona.enableGPRS(true)) {
        Serial.println(F("Failed to turn GPRS on"));
      }
    }
  }
}

void testdrawtext(char *text, uint16_t color, uint8_t set_x, uint8_t set_y) {
  tft.setCursor(set_x, set_y);
  tft.setTextColor(color);
  tft.print(text);

}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
