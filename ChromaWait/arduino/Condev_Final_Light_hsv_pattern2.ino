#include <ArduinoJson.h>

#include <Arduino_JSON.h>
//#include "ArduinoJson.h"

#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"

#include <FastLED.h>

#define LED_PIN     3
#define BRIGHTNESS  50
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB

// Params for width and height
const uint8_t kMatrixWidth  = 16;
const uint8_t kMatrixHeight = 16;

// Param for different pixel layouts
const bool    kMatrixSerpentineLayout = true;

int mycolor[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
#define MAX_DIMENSION ((kMatrixWidth>kMatrixHeight) ? kMatrixWidth : kMatrixHeight)

// The leds
CRGB leds[kMatrixWidth * kMatrixHeight];

// The 16 bit version of our coordinates
static uint16_t x;
static uint16_t y;
static uint16_t z;
uint16_t speed = 20; // speed is set dynamically once we've started up
uint16_t scale = 30; // scale is set dynamically once we've started up

// This is the array that we keep our computed noise values in
uint8_t noise[MAX_DIMENSION][MAX_DIMENSION];

CRGBPalette16 currentPalette( PartyColors_p );
uint8_t       colorLoop = 1;



int deleteH;
int deleteS;
int deleteV;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
JSONVar allInfo;
int patientNum = 0;


// initialize WiFi connection:
WiFiClient wifi;
MqttClient mqttClient(wifi);

// details for MQTT client:
const char broker[] = "voruin.cloud.shiftr.io";
//const char broker[] = "public.cloud.shiftr.io";
int port = 1883;
const char topic1[] = "lights";
const char topic2[] = "doctorCallingH";
const char topic3[] = "doctorCallingS";
const char topic4[] = "doctorCallingV";

char clientID[] = "arduinoLightClient";


void setup() {
  // initialize serial:
  Serial.begin(9600);
  // wait for serial monitor to open:
  //  while (!Serial);

  // initialize WiFi, if not connected:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.println(SECRET_SSID);
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(2000);
  }
  // print IP address once connected:
  Serial.print("Connected. My IP address: ");
  Serial.println(WiFi.localIP());

  // set the credentials for the MQTT client:
  mqttClient.setId(clientID);
  mqttClient.setUsernamePassword(SECRET_MQTT_USER, SECRET_MQTT_PASS);

  // try to connect to the MQTT broker once you're connected to WiFi:
  while (!connectToBroker()) {
    Serial.println("attempting to connect to broker");
    delay(1000);
  }
  Serial.println("connected to broker");


  delay(3000);
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  // Initialize our coordinates to some random values
  x = random16();
  y = random16();
  z = random16();

}

void fillnoise8() {
  // If we're runing at a low "speed", some 8-bit artifacts become visible
  // from frame-to-frame.  In order to reduce this, we can do some fast data-smoothing.
  // The amount of data smoothing we're doing depends on "speed".
  uint8_t dataSmoothing = 0;
  if ( speed < 50) {
    dataSmoothing = 200 - (speed * 4);
  }

  for (int i = 0; i < MAX_DIMENSION; i++) {
    int ioffset = scale * i;
    for (int j = 0; j < MAX_DIMENSION; j++) {
      int joffset = scale * j;

      uint8_t data = inoise8(x + ioffset, y + joffset, z);

      if ( dataSmoothing ) {
        uint8_t olddata = noise[i][j];
        uint8_t newdata = scale8( olddata, dataSmoothing) + scale8( data, 256 - dataSmoothing);
        data = newdata;
      }

      noise[i][j] = data;
    }
  }

  z += speed;

  // apply slow drift to X and Y, just for visual variation.
  x += speed / 8;
  y -= speed / 16;
}

void mapNoiseToLEDsUsingPalette()
{
  static uint8_t ihue = 0;

  for (int i = 0; i < kMatrixWidth; i++) {
    for (int j = 0; j < kMatrixHeight; j++) {
      // We use the value at the (i,j) coordinate in the noise
      // array for our brightness, and the flipped value from (j,i)
      // for our pixel's index into the color palette.

      uint8_t index = noise[j][i];
      uint8_t bri =   noise[i][j];

      // if this palette is a 'loop', add a slowly-changing base value
      if ( colorLoop) {
        index += ihue;
      }

      // brighten up, as the color palette itself often contains the
      // light/dark dynamic range desired
      if ( bri > 127 ) {
        bri = 255;
      } else {
        bri = dim8_raw( bri * 2);
      }

      CRGB color = ColorFromPalette( currentPalette, index, bri);
      leds[XY(i, j)] = color;
    }
  }

  ihue += 1;
}



//// subscribe to a topic
//mqttClient.subscribe(topic);

// topics can be unsubscribed using:
// mqttClient.unsubscribe(topic);

//Serial.print("Waiting for messages on topic: ");
//Serial.println(topic);
//Serial.println();

boolean connectToBroker() {
  // if the MQTT client is not connected:
  if (!mqttClient.connect(broker, port)) {
    // print out the error message:
    Serial.print("MOTT connection failed. Error no: ");
    Serial.println(mqttClient.connectError());
    // return that you're not connected:
    return false;
  }
  // once you're connected, you can proceed:
  mqttClient.subscribe(topic1);
  mqttClient.subscribe(topic2);
  mqttClient.subscribe(topic3);
  mqttClient.subscribe(topic4);
  // return that you're connected:
  return true;
}

void loop() {
//for (int i =0;i< 12;i++) {
//  Serial.println(mycolor[i]);
//  }
  
  
  // if not connected to the broker, try to connect:
  if (!mqttClient.connected()) {
    Serial.println("reconnecting");
    connectToBroker();
  }

  // if a message comes in, read it:
  if (mqttClient.parseMessage() > 0) {
    if (mqttClient.messageTopic() == "lights") {
      Serial.print("Got a message on topic : ");
      Serial.print(mqttClient.messageTopic());
      Serial.println("from waiting room website");

      // read the message:
      while (mqttClient.available()) {

        StaticJsonDocument<256> doc;
        deserializeJson(doc, mqttClient);
        //      String doc = mqttClient.read();
        //      Serial.print((char)mqttClient.read());
        String patientName = doc["name"];
        String doctorName =  doc["doctor"];
        String reservationTime = doc["apptTime"];
        String submitTime = doc["subTime"];
        int h = doc["color"][0];
        int s = doc["color"][1];
        int v = doc["color"][2];

        int reservationHour = reservationTime.substring(0, 2).toInt();
        int reservationMinute = reservationTime.substring(3, 5).toInt();
        int submitHour = submitTime.substring(0, 2).toInt();
        int submitMinute = submitTime.substring(3, 5).toInt();
        int submitSecond = submitTime.substring(6, 8).toInt();
        int waitingTime = reservationHour * 3600 + reservationMinute * 60 - (submitHour * 3600 + submitMinute * 60 + submitSecond);
        int reservationTimeInSeconds = reservationHour * 3600 + reservationMinute * 60;


        Serial.println(h);

        
        if (patientNum > 3) {
          patientNum = 0;
        }

        mycolor[patientNum*3] = h;
        mycolor[patientNum*3+1] = s;
        mycolor[patientNum*3+2] = v;
        patientNum++;

      }

    }

    // if a message comes in, read it:
    else {
      Serial.print("Got a message on topic : ");
      Serial.print(mqttClient.messageTopic());
      Serial.println("from doctor website");
      // read the message:
      while (mqttClient.available()) {
        if (mqttClient.messageTopic() == "doctorCallingH") {
          deleteH = mqttClient.parseInt();
//          Serial.println(deleteH);
        }
        else if (mqttClient.messageTopic() == "doctorCallingS") {

          deleteS = mqttClient.parseInt();
        }
        else if (mqttClient.messageTopic() == "doctorCallingV") {

          deleteV = mqttClient.parseInt();
        }

        for (int t = 0; t < 10; t += 3) {
          if (mycolor[t] == deleteH && mycolor[t + 1] == deleteS && mycolor[t + 2] == deleteV) {
            mycolor[t] = 0;
            mycolor[t + 1] = 0;
            mycolor[t + 2] = 0;
          }
        }
      }

    }
  }
  // Periodically choose a new palette, speed, and scale
  ChangePaletteAndSettingsPeriodically();

  // generate noise data
  fillnoise8();

  // convert the noise data to colors in the LED array
  // using the current palette
  mapNoiseToLEDsUsingPalette();

  FastLED.show();
  // delay(10);

}

#define HOLD_PALETTES_X_TIMES_AS_LONG 1

void ChangePaletteAndSettingsPeriodically()
{
  uint8_t secondHand = ((millis() / 1000) / HOLD_PALETTES_X_TIMES_AS_LONG) % 180;
  static uint8_t lastSecond = 99;
  SetupPurpleAndGreenPalette();    speed =  1; scale = 100; colorLoop = 0;

}




void SetupPurpleAndGreenPalette()
{
  CRGB purple = CHSV(mycolor[0], mycolor[1], mycolor[2]);
  CRGB green  = CHSV( mycolor[3], mycolor[4], mycolor[5]);
  CRGB black  = CHSV( mycolor[6], mycolor[7], mycolor[8]);
  CRGB white  =  CHSV( mycolor[9], mycolor[10], mycolor[11]);
  currentPalette = CRGBPalette16(
                     green,  green,  black,  black,
                     purple, purple, white,  black,
                     white,  green,  black,  black,
                     purple, purple, black,  white );
}

//
// Mark's xy coordinate mapping code.  See the XYMatrix for more information on it.
//
uint16_t XY( uint8_t x, uint8_t y)
{
  uint16_t i;
  if ( kMatrixSerpentineLayout == false) {
    i = (y * kMatrixWidth) + x;
  }
  if ( kMatrixSerpentineLayout == true) {
    if ( y & 0x01) {
      // Odd rows run backwards
      uint8_t reverseX = (kMatrixWidth - 1) - x;
      i = (y * kMatrixWidth) + reverseX;
    } else {
      // Even rows run forwards
      i = (y * kMatrixWidth) + x;
    }
  }
  return i;
}
