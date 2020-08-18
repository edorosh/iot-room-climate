/**
 * MQTT Room Climate.
 * 
 * Monitors temperature, humidity and air pressure in a room and sends the data
 * to MQTT server. Not energy efficient device. It is supposed to be powered up 
 * by a constant usb power supply.
 * 
 * Author: Evgeny Doros based on https://lastminuteengineers.com/bme280-esp8266-weather-station/
 * Email: eugene.dorosh@gmail.com
 * Github: https://github.com/edorosh
 * License: MIT 
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "Serial.h"

#include "Config.h"
#include "DebugMacro.h"
#include "Version.h"

#define BME_ADDRESS 0x76

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;

float temperature, humidity, pressure, altitude;

// todo: replace timing vars by a Timout library
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMs = 0;    // will store last time the Sensor was updated

void setup()
{
#ifdef DEBUG
  beginSerial();
  DPRINTLN("");
  DPRINT(F("Sketch starting: iot-room-climate "));
  DPRINTLN(FW_VERSION);
  DPRINT(F("Reset reason: "));
  DPRINTLN(ESP.getResetReason());
  DPRINT(F("Core Version: "));
  DPRINTLN(ESP.getCoreVersion());
  DPRINT(F("SDK Version: "));
  DPRINTLN(ESP.getSdkVersion());
  DPRINTLN("");
#endif

  bme.begin(BME_ADDRESS); 

  connectToWiFi();
  connectToMQTT();

  setupOTA();

  // auto Modem-sleep. In Modem-sleep, the system can be woken up automatically. 
  // Users don’t need to configure the interface.
  wifi_set_sleep_type(MODEM_SLEEP_T);
}

void loop()
{
  client.loop();
  ArduinoOTA.handle();

  unsigned long currentMs = millis();
  if (currentMs - previousMs >= READ_INTERVAL_MS)
  {
    // save the last time you updated the DHT values
    previousMs = currentMs;

    readSensorsData();
    publishSensorsData();
  }
}


/** Open Serial port for debugging purposes. */
inline void beginSerial()
{
  initSerial(SERIAL_SPEED_BAUD, SERIAL_TIMEOUT_MS);
  Serial.println(F("Booting..."));
}

inline void connectToWiFi()
{
  WiFi.forceSleepWake();
  yield();

  DPRINTLNF("Enabling STA mode");

  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.persistent(false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  WiFi.hostname(STAHOSTNAME);

  DPRINTFF("Connecting to WiFi network ");
  DPRINTLN(STASSID);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    DPRINTLNF("Connection Failed! Restarting with delay...");
    delay(WIFI_RECONNECT_ON_FAILURE_MS);
    ESP.restart();
  }

  DPRINTFF("Connected! IP address is ");
  DPRINTLN(WiFi.localIP().toString().c_str());
}

inline void connectToMQTT()
{
  client.setServer(MQTTSERVER, MQTTPORT);

  // Loop until we're reconnected
  while (!client.connected())
  {
    DPRINT("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(MQTT_CLIENT_NAME))
    {
      DPRINTLNF(" connected");
    }
    else
    {
      DPRINTFF(" failed, rc=");
      DPRINT(client.state());
      DPRINTLNF(" try again in 5 seconds");

      // Wait some time before retrying
      delay(CONNECT_MQTT_TIMEOUT_MS);
    }
  }
}

inline void setupOTA()
{
  DPRINT("Enabling OTA with hostname ");
  DPRINTLN(STAHOSTNAME);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(STAHOSTNAME);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    DPRINTLN("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    DPRINTLN("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DPRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DPRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      DPRINTLNF("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      DPRINTLNF("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      DPRINTLNF("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      DPRINTLNF("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      DPRINTLNF("End Failed");
    }
  });

  ArduinoOTA.begin();
}

bool readSensorsData()
{
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;

  DPRINT(F("Temp: "));
  DPRINTLN(temperature);

  DPRINT(F("Humidity: "));
  DPRINTLN(humidity);

  DPRINT(F("Pressure: "));
  DPRINTLN(pressure);

  return true;
}

void publishSensorsData()
{
  //todo: get rid of global variable
  if (!client.publish(MQTT_TEMPERATURE_TOPIC, String(temperature).c_str(), true)) {
    DPRINTLNF("Sending temperature to MQTT failed");
  }

  if (!client.publish(MQTT_HUMIDITY_TOPIC, String(humidity).c_str(), true)) {
    DPRINTLNF("Sending humidity to MQTT failed");
  }

  if (!client.publish(MQTT_PRESSURE_TOPIC, String(pressure).c_str(), true)) {
    DPRINTLNF("Sending pressure to MQTT failed");
  }
}