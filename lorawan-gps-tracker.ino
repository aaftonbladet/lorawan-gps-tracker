#include <CayenneLPP.h>
#include <Adafruit_GPS.h>
#include <Adafruit_SleepyDog.h>

// Radiolib godmode makes all private members of radiolib objects accessible.
// We use this in transmitMessage() to send uplinks without waiting for a response.
#define RADIOLIB_GODMODE 1
#include <RadioLib.h>

#include "secrets.h"

// How often we want the location to be transmitted
constexpr int SEND_INTERVAL_MS = 120000;

// How long we should wait for a GPS fix before giving up
constexpr int GPS_FIX_TIMEOUT_MS = 60000;

RFM96 radio = new Module(8, 3, 4, 3);
LoRaWANNode node(&radio, &EU868);
Adafruit_GPS GPS(&Serial1);

constexpr int GPS_DISABLE_PIN = A0;
constexpr int BATTERY_VOLTAGE_PIN = A7;
constexpr float REFERENCE_VOLTAGE = 3.3f;

void setGpsEnabled(bool enabled) {
  digitalWrite(GPS_DISABLE_PIN, !enabled);
}

float getBatteryVoltage() {
  return analogRead(BATTERY_VOLTAGE_PIN) * 2.0f * REFERENCE_VOLTAGE / 1024.0f;
}

// Ask the linker for a reference to the tick count. See delay.c in arduino core.
extern "C" volatile uint32_t _ulTickCount;

int deepSleep(int32_t sleepLength) {
  int begin = millis();

  setGpsEnabled(false);
  //GPS.standby();
  radio.sleep();

  int remainingSleep = sleepLength;
  while (remainingSleep > 1)
    remainingSleep -= Watchdog.sleep(remainingSleep);

  setGpsEnabled(true);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.wakeup();

  // Calculate by how much we overshot sleepLength
  // Includes the time spent on GPS.standby(), GPS.wakeup() and radio.sleep(),
  // along with how much Watchdog.sleep overshot the requested sleep length.
  int lostWhileSleeping = millis() - begin - remainingSleep;

  // Adjust the tick count (what millis() uses internally) to include the ticks missed while sleeping.
  // Allows RadioLib to correctly calculate the duty cycle.
  _ulTickCount += sleepLength;

  return lostWhileSleeping;
}

// Critical issue? Stop execution, blink the LED, and print our error message to the serial console.
template<typename... TArgs>
void trap(const char* f, TArgs... args) {
  int it = 0;
  while (true) {
    digitalWrite(LED_BUILTIN, ++it % 2); // let dwVal evaulate to 0/1 every other iteration.
    Serial.printf(f, args...);
    delay(500);
  }
}

void setup() {
  pinMode(GPS_DISABLE_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  GPS.begin(9600);

  //while (!Serial)
  //  yield();

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  Serial.printf("Getting ready ... ");

  if (int status = radio.begin())
    trap("Failed to initialize lora radio, status %d\n", status);

  // Allow multiple tries for over-the-air activation, in case of a spotty connection.
  int otaaAttemptsRemaining = 6;

  while (int status = node.beginOTAA(TTN_JoinEUI, TTN_DevEUI, TTN_NwkKey, TTN_AppKey))
    if (--otaaAttemptsRemaining == 0)
      trap("OTA activation failed, status %d\n", status);

  Serial.printf("Ready! OTAA has %d tries remaining\n", otaaAttemptsRemaining);
  
  // TIP: GPS fix taking too long? Put your device someplace with a clear view of the sky,
  // then run the code below. 
  /*Serial.println("Collecting almanac for 2 cycles (25 minutes).");

  auto begin = millis();
  while (millis() - begin < 25 * 60 * 1000)
    yield();
  
  Serial.println("Done collecting almanac.");*/
}

// Helper function that performs an uplink without waiting for a response.
bool transmitMessage(uint8_t* data, size_t len) {
  int status = node.uplink(data, len, 10, false);

  // Ugly hack to let us send an uplink without having any downlinks in between.
  // Likely not LoRaWAN-compliant.
  node.rxDelayEnd = 0;
  node.rxDelayStart = 0;

  return status == 0;
}

void loop() {
  // Keep track of when this iteration started.
  auto begin = millis();

  // Wait for the GPS to get a confident estimate of the current location
  while (!GPS.read() || !GPS.newNMEAreceived() || !GPS.parse(GPS.lastNMEA()) || !GPS.fix) {
    if (millis() - begin > GPS_FIX_TIMEOUT_MS) {
      Serial.println("GPS took too long for fix");
      break;
    }

    yield();
  }

  // We're done with the GPS for now, turn it off to conserve energy.
  setGpsEnabled(false);

  //Serial.printf("gps: 20%02d-%02d-%02d %02d:%02d:%02d.%03d %f %f %f\n", GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds, GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude);

  CayenneLPP uplinkMessage(64);

  // If we're confident of our location, include it in the uplink message.
  if (GPS.fix)
    uplinkMessage.addGPS(0, GPS.latitudeDegrees, GPS.longitudeDegrees, GPS.altitude);
  
  uplinkMessage.addAnalogInput(1, getBatteryVoltage()); // Current battery voltage
  uplinkMessage.addAnalogInput(2, static_cast<float>(millis() - begin) / 1000.0f); // Time spent waiting for a fix

  // Finally, submit our uplink message.
  if (transmitMessage(uplinkMessage.getBuffer(), uplinkMessage.getSize()))
    Serial.println("Sent uplink!");
  else
    Serial.printf("Failed to send uplink");

  // lostWhileSleeping contains by how much deepSleep() overshot the requested sleep length last iteration.
  // By removing lostWhileSleeping when calculating the new sleep length, we can get closer to having the send interval be exactly SEND_INTERVAL_MS
  static int lostWhileSleeping = 0;

  // Calculate the sleep length. Time spent waiting for fix & sending the uplink is used as credit (millis() - begin), as well as possible overshoot from the last iteration (lostWhileSleeping)
  // Also, constrain it so it can't be negative.
  int sleepLength = std::max(SEND_INTERVAL_MS - (millis() - begin) - lostWhileSleeping, 0);

  Serial.printf("Sleeping for %d\n", sleepLength);

  lostWhileSleeping = deepSleep(sleepLength);
}
