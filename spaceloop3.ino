/*
   SpaceLoop 3

   Hardwarepinout

   GPIO  0 - Button

   GPIO  2 - APA102 Data
   GPIO 12 - APA102 Clock

   GPIO  4 - 1wire bus (might be 5)

   GPIO 13 - Buzzer
   GPIO 14 - DHT22

   GPIO  5 - Optional (might be 4)
   GPIO 16 - Optional

*/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <EEPROM.h>
#define FASTLED_ESP8266_RAW_PIN_ORDER
#include "FastLED.h"

FASTLED_USING_NAMESPACE
#include <OneWire.h>
#include <SPI.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <DHT.h>

#define Button 0
#define LedData 2
#define LedClock 12
#define OneWirePin 4
#define Buzzer 13
#define DHT22pin 14

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHT22pin, DHTTYPE);

#define LED_TYPE    APA102
#define COLOR_ORDER BGR
#define NUM_LEDS    64
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

bool spacestate;

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "1883";
char location[11];
char id[11];
char debugtopic[40] = "debug";
char pubtopic[40] = "sensors";
const String swversion = "Spaceloop3 v0.2";

String chipid;
char chipidchar[6];
uint8_t MAC_array[6];
char MAC_char[18];

uint16_t i, j;

unsigned long starttijd;

//flag for saving data
bool shouldSaveConfig = false;

// The extra parameters to be configured (can be either global or just in the setup)
// After connecting, parameter.getValue() will get you the configured value
// id/name placeholder/prompt default length
WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 5);
WiFiManagerParameter custom_location("location", "controller location", location, 10);
WiFiManagerParameter custom_id("id", "controller id", id, 10);
WiFiManagerParameter custom_debug("debugtopic", "Debug Topic", debugtopic, 40);
WiFiManagerParameter custom_pub("publictopic", "Publication Topic", pubtopic, 40);

OneWire  ds(OneWirePin);



void setup() {
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW);

  chipid = ESP.getChipId();
  chipid.toCharArray(chipidchar, sizeof chipidchar);

  WiFi.macAddress(MAC_array);
  sprintf(MAC_char, "%02x:%02x:%02x:%02x:%02x:%02x",
          MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);

  dht.begin();

  FastLED.addLeds<LED_TYPE, LedData, LedClock, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.show();

  Serial.begin(115200);
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          Serial.println(mqtt_server);

          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(location, json["location"]);
          strcpy(id, json["id"]);
          strcpy(debugtopic, json["debugtopic"]);
          strcpy(pubtopic, json["pubtopic"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  pinMode(Button, INPUT);


  WiFiManager wifiManager;
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_location);
  wifiManager.addParameter(&custom_id);
  wifiManager.addParameter(&custom_debug);
  wifiManager.addParameter(&custom_pub);

  //wifiManager.resetSettings();

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  updateParameters();

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.print("MQTT Server: ");
  Serial.println(mqtt_server);
  Serial.println();
  client.setServer(mqtt_server, 1883);
  client.setCallback(onMqttMessage);
}

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm};

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns


void loop() {
  FastLED.show();
  EVERY_N_MILLISECONDS( 20 ) {
    gHue++;  // slowly cycle the "base color" through the rainbow
  }

  if (!client.connected()) {
    Serial.println("Attempting MQTT Reconnect.");
    reconnect();
  }
  
  client.loop();

  if (digitalRead(Button) == LOW ) {
    ondemandPortal();
  }

  if (millis() - starttijd > 60000) {
    while (temperatuur());
    readDHT();
    starttijd = millis();
  }

  FastLED.delay(1000 / FRAMES_PER_SECOND);

  gPatterns[gCurrentPatternNumber]();

}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if ( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS);
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for ( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for ( int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}


//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


void updateParameters() {
  if (shouldSaveConfig) {
    //read updated parameters
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(location, custom_location.getValue());
    strcpy(id, custom_id.getValue());
    strcpy(debugtopic, custom_debug.getValue());
    strcpy(pubtopic, custom_pub.getValue());

    //save the custom parameters to FS

    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["location"] = location;
    json["id"] = id;
    json["debugtopic"] = debugtopic;
    json["pubtopic"] = pubtopic;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
  }
}

void ondemandPortal() {
  WiFiManager wifiManager;
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_location);
  wifiManager.addParameter(&custom_id);
  wifiManager.addParameter(&custom_debug);
  wifiManager.addParameter(&custom_pub);
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  if (!wifiManager.startConfigPortal("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  Serial.println("connected...yeey :)");
  updateParameters();
}

boolean reconnect() {
  if (client.connect(chipidchar)) {
    Serial.println("Reconnected to MQTT");
    // Once connected, publish an announcement...
    mqtt_publish(String(debugtopic) + "/" + location + "/" + chipid, swversion + String(" ") + WiFi.localIP().toString() + String(" ") + String(MAC_char));

    client.subscribe("revspace/button/#");
    client.subscribe("revspace/state");
    client.loop();
  }
  return client.connected();
}

void onMqttMessage(char* topic, byte * payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  char bericht[50] = "";
  if (strcmp(topic, "revspace/state") == 0) {
    for (uint8_t pos = 0; pos < length; pos++) {
      bericht[pos] = payload[pos];
    }

    if (strcmp(bericht, "open") == 0) {
      Serial.println("Revspace is open");
      if (spacestate == LOW) {
        spacestate = HIGH;
        gCurrentPatternNumber = 0;
      }
    } else {
      // If it ain't open, it's closed! (saves a lot of time doing strcmp).
      Serial.println("Revspace is dicht");
      if (spacestate == HIGH) {
        spacestate = LOW;
        gCurrentPatternNumber = 2;
      }
    }
  }

  if (strcmp(topic, "revspace/button/nomz") == 0) {

    
    for (int repeats = 0; repeats < 40; repeats++) {
      client.loop();
      
      for (int odd = 1; odd < NUM_LEDS; odd = odd + 2) {
        leds[odd] = 0xFFA500;
      }
      for (int even = 0; even < NUM_LEDS; even = even + 2) {
        leds[even] = 0x0000FF;
      }
      FastLED.show();
      
      digitalWrite(Buzzer, HIGH);
      delay(350);

      for (int odd = 1; odd < NUM_LEDS; odd = odd + 2) {
        leds[odd] = 0x0000FF;
      }
      for (int even = 0; even < NUM_LEDS; even = even + 2) {
        leds[even] = 0xFFA500;
      }
      FastLED.show();
      digitalWrite(Buzzer, LOW);
      delay(350);
    }
  }

  if (strcmp(topic, "revspace/button/doorbell") == 0) {
    for (int repeats = 0; repeats < 40; repeats++) {
      client.loop();
      
      for (int odd = 1; odd < NUM_LEDS; odd = odd + 2) {
        leds[odd] = 0xFFFF00;
      }
      for (int even = 0; even < NUM_LEDS; even = even + 2) {
        leds[even] = 0x000000;
      }
      FastLED.show();
      digitalWrite(Buzzer, HIGH);
      delay(50);

      for (int odd = 1; odd < NUM_LEDS; odd = odd + 2) {
        leds[odd] = 0x000000;
      }
      for (int even = 0; even < NUM_LEDS; even = even + 2) {
        leds[even] = 0xFFFF00;
      }
      FastLED.show();
      digitalWrite(Buzzer, LOW);
      delay(20);
    }
  }

  if (strcmp(topic, "revspace/button/rollcall") == 0) {
    Serial.println("Rollcall!");
    mqtt_publish(String(debugtopic) + "/" + location + "/" + chipid, swversion + String(" ") + WiFi.localIP().toString() + String(" ") + String(MAC_char));
  }
}

void mqtt_publish (String topic, String message) {
  Serial.println();
  Serial.print("Publishing ");
  Serial.print(message);
  Serial.print(" to ");
  Serial.println(topic);

  char t[100], m[100];
  topic.toCharArray(t, sizeof t);
  message.toCharArray(m, sizeof m);

  client.publish(t, m, /*retain*/ 1);
}

boolean temperatuur() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  String adres1wire = "";
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    return false;
  }

  Serial.print("ROM =");
  for ( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return false;
  }
  Serial.println();

  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return false;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  float celsius;
  celsius = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.println("°C");

  // Byte 2 is zone
  // Byte 3 is id
  String ds18zone    = String(data[2], DEC);
  String ds18id      = String(data[3], DEC);
  String topic   = String(pubtopic) + "/temperature/" + ds18zone + "/" + ds18id;
  String message = String(celsius, 2) + " °C";
  mqtt_publish(topic, message);

  return true;
}

void readDHT() {
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print("DHT22: Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" °C ");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" °C ");
  Serial.println();

  String topic   = String(pubtopic) + "/temperature/" + location + "/" + id;
  String message = String(t, 2) + " °C";
  mqtt_publish(topic, message);

  topic   = String(pubtopic) + "/humidity/" + location + "/" + id;
  message = String(h, 2) + " %";
  mqtt_publish(topic, message);

  topic   = String(pubtopic) + "/heatindex/" + location + "/" + id;
  message = String(hic, 2) + " °C";
  mqtt_publish(topic, message);
}


