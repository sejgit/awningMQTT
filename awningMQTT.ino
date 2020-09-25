/*
 *  Awning control & Living room temperature (not implemented)
 *
 *  Written for an ESP8266 Sonoff Wifi switch
 *  --fqbn esp8266:8266:generic
 *
 *  init   SeJ 03 09 2019 specifics to my application
 *  update SeJ 03 24 2019 change to relay
 *  update SeJ 04 07 2019 changes to work on ESP relay
 *  update SeJ 06 30 2020 add MQTT capability -- REFER: [[https://gist.github.com/boverby/d391b689ce787f1713d4a409fb43a0a4][ESP8266 MQTT example]]
 *  update SeJ 07 03 2020 modifiy hb to one character, last will, monitor
 *  update SeJ 07 08 2020 move to MQTT only removing REST ISY
 *  update SeJ 09 24 2020 with learnings from FishtankOTA
 */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <Timezone.h>
#include <LittleFS.h>
#include <DHT.h>


/* Passwords & Ports
 * wifi: ssid, password
 * ISY: hash, isy, isyport
 * MQTT mqtt_server, mqtt_serverport
 */
#include <../../../../../../../../../Projects/keys/sej/sej.h>


/*
 * Time
 */
// NTP Servers:
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";

const int timeZone = 0;     // use UTC due to Timezone corr
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

// US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     // Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);
TimeChangeRule *tcr;        // pointer to the time change rule, use to get TZ abbrev

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
const char* defaultTime = "00:00:00";
char stringTime[10];
int oldmin = 99;
time_t local;


/*
 * Web Server
 */
WiFiServer server(80);
String ServerTitle = "Jenkins Living Room Awning";


/*
 *  MQTT
 */
const char* topic = "sej"; // main topic
String clientId = "awning"; // client ID for this unit
char buffer[256];

// MQTT topics
const char* topic_control_relay = "sej/awning/control/relay"; // control topic
const char* message_control_relay[] = {"ON", "---"};

const char* topic_status_relay = "sej/awning/status/relay"; // bounce back
const char* message_status_relay[] = {"ON", "OFF"};

const char* topic_status_position = "sej/awning/status/position"; // awning position
const char* message_status_position[] = {"IN", "OUT"};
boolean awningPosition;

const char* topic_control_reset = "sej/awning/control/reset"; // reset position
const char* message_control_reset[] = {"--", "RESET"};

const char* topic_status_hb = "sej/awning/status/hb"; // hb topic
const char* message_status_hb[] = {"ON", "OFF"};

const char* willTopic = topic; // will topic
byte willQoS = 0;
boolean willRetain = false;
const char* willMessage = ("lost connection " + clientId).c_str();
boolean heartbeat = false; // heartbeat to mqtt

WiFiClient espClient;
PubSubClient mqttClient(espClient);
long mqttLastMsg = 0;
int mqttValue = 0;


// cfg updates
boolean cfgChangeFlag = false;

/*
 * timers
 */
unsigned long currentMillis = 0;
unsigned long hbMillis = 0;
const long hbInterval = 60000; // how often to send hb
unsigned long ledMillis = 0;
const long ledInterval = 3000; // blink led h
bool ledState = false;
// awning
unsigned long relayMillis = 0;
const long relayInterval = 30000; // update relay at least this often
int awningpushtime = 2000;


/*
 * IO
 */
int pbpower = 12; // Pin D2 GPIO 0 12
int pbbutton = 13; // Pin D3 GPIO 4 13
int statusled = 16; // status led
// LED_BUILTIN


/*
 * Declare Subroutines
 */

/*
 * Establish Wi-Fi connection & start web server
 */
boolean initWifi(int tries = 2, int waitTime = 2) {
    int status = WL_IDLE_STATUS;
    WiFi.mode(WIFI_STA);

    while(status != WL_CONNECTED && (tries-- > 0)) {
        status = WiFi.begin(ssid, password);
        int timeout = waitTime;

        while (WiFi.status() != WL_CONNECTED && (timeout-- > 0)) {
            delay(1000);
        }
        if (WiFi.status() == WL_CONNECTED) {
            break;
        }
    }

    if(WiFi.status() != WL_CONNECTED) {
        return false;
    }
    else {
        // Starting the web server
        server.begin();
        return true;
    }
}


/*
 * MQTT client init connection
 */
boolean initMQTT() {
    Serial.print(F("Attempting MQTT connection..."));

    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), clientId.c_str(), password,
                           willTopic, willQoS, willRetain, willMessage)) {
        Serial.println(F("connected"));
        // Once connected, publish an announcement...
        mqttClient.publish(topic, ("connected " + clientId).c_str() , true );
        mqttClient.subscribe(topic_control_relay);
        mqttClient.subscribe(topic_control_reset);
        return true;
    } else {
        return false;
    }
}


/*
 * refresh config parameters to MQTT
 */
boolean mqttRefreshConfig() {
    if(mqttClient.connected()) {
        mqttClient.publish(topic_status_position, message_status_position[awningPosition], true);
        return true;
    } else {
        return false;
    }
}


/*
 * updateLocalTime
 */
boolean updateLocalTime() {
    if(timeStatus() == timeSet) {
        if(oldmin != minute()) {
            local = myTZ.toLocal(now(), &tcr);
            sprintf(stringTime, "%02d:%02d", hour(local), minute(local));
            oldmin = minute();
            return true;
        }
    }
    return false;
}


/*
 * MQTT Callback message
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    char mypayload[length+1];
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
        mypayload[i] = (char)payload[i];
    }
    mypayload[length] = '\0';
    Serial.println();

    // Pulse the RELAY if an ON is received
    if(strcmp(topic, topic_control_relay)==0){
        if(strcmp(mypayload, message_control_relay[0]) == 0) {
            AwningControl();
        }
    }

    // Reset the awningPosition var if RESET received
    if(strcmp(topic, topic_control_reset)==0){
        if(strcmp(mypayload, message_control_reset[1]) == 0) {
            AwningPosition();
        }
    }
}


/*
 * Awning signal control
 */
void AwningControl() {
    Serial.println("request awning control");
    // Awning Control only so often
    if(currentMillis - relayMillis > relayInterval) {
        digitalWrite(pbpower, HIGH);
        digitalWrite(pbbutton, HIGH);
        Serial.println("**awning button pushed**");
        mqttClient.publish(topic_status_relay, message_status_relay[0], false);
        delay(awningpushtime);

        digitalWrite(pbbutton, LOW);
        digitalWrite(pbpower, LOW);
        Serial.println("**awning button released**");
        mqttClient.publish(topic_status_relay, message_status_relay[0], true);
        mqttClient.publish(topic_control_relay, message_control_relay[0], true); // clear control msg
        AwningPosition();
        relayMillis = currentMillis;
    } else {
        Serial.println("awning control too soon");
    }
}


/*
 * Awning position swap
 */
void AwningPosition() {
    Serial.println("request awning position flip");
    awningPosition = !awningPosition;
    cfgChangeFlag = true;
    if(mqttClient.connected()) {
        mqttClient.publish(topic_status_position, message_status_position[awningPosition], true);
        mqttClient.publish(topic_control_reset, message_control_reset[0], true);
    }
}


/*
 * NTP code
 */
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
    IPAddress ntpServerIP; // NTP server's ip address

    while (Udp.parsePacket() > 0) ; // discard any previously received packets
    WiFi.hostByName(ntpServerName, ntpServerIP);
    sendNTPpacket(ntpServerIP);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 1500) {
        int size = Udp.parsePacket();
        if (size >= NTP_PACKET_SIZE) {
            Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
            unsigned long secsSince1900;
            // convert four bytes starting at location 40 to a long integer
            secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
            secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
            secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
            secsSince1900 |= (unsigned long)packetBuffer[43];
            return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
        }
    }
    return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    Udp.beginPacket(address, 123); //NTP requests are to port 123
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
}


/*
 * saveConfig file
 */
void saveConfig() {
    Serial.println(F("Saving config."));
    File f = LittleFS.open("/awning.cnf", "w");
    if(!f){
        Serial.println(F("Write failed."));
    } else {
        f.print(F("awningPosition="));
        f.println(awningPosition);
        f.flush();
        f.close();
        Serial.println(F("Saved values."));
    }
}


/* Web Client
 * Listening for new clients & serve them
 */
void webClient() {
    WiFiClient client = server.available();
    if (client) {
        Serial.println(F("New client"));
        // bolean to locate when the http request ends
        boolean blank_line = true;
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n' && blank_line) {
                    client.println(F("HTTP/1.1 200 OK"));
                    client.println(F("Content-Type: text/html"));
                    client.println(F("Connection: close"));
                    client.println();
                    // web page that displays temperature
                    client.println(F("<!DOCTYPE HTML><html><head></head><body><h1>"));
                    client.println(ServerTitle);
                    client.println(F("</h1><h3>Awning Position: "));
                    client.println(awningPosition, BIN);
                    client.println(F("</h3><h3>"));
                    client.println(stringTime);
                    client.println(F("</h3></body></html>"));
                    break;
                }
                if (c == '\n') {
                    // when starts reading a new line
                    blank_line = true;
                }
                else if (c != '\r') {
                    // when finds a character on the current line
                    blank_line = false;
                }
            }
        }
        // closing the client connection
        delay(1);
        client.stop();
        Serial.println(F("WebClient disconnected."));
    }
}


/*
 * Setup
 */
void setup(){
    Serial.println(F("Boot Start."));

    // Set-up I/O
    Serial.begin(9600);
    delay(10);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH

    // prepare GPIO4
    pinMode(pbpower, OUTPUT);
    digitalWrite(pbpower, LOW);
    delay(1);
    pinMode(pbbutton, OUTPUT);
    digitalWrite(pbbutton, LOW);
    delay(1);

    // get WiFi up and going
	Serial.println("");
	Serial.println(F("Connecting to: "));
    Serial.print(ssid);
    if (!initWifi(5, 10)) {
        Serial.println(F("Initial Connection Failed! Rebooting..."));
        delay(5000);
        ESP.restart();
    }
    Serial.println(F("Connected."));
    Serial.print(F("IP: "));
    Serial.println(WiFi.localIP());

    // OTA set-up

    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    // ArduinoOTA.setHostname("myesp8266");

    // No authentication by default
    // ArduinoOTA.setPassword("admin");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else { // U_FS
                type = "filesystem";
            }

            // NOTE: if updating FS this would be the place to unmount FS using FS.end()
            LittleFS.end();
            Serial.println("Start updating " + type);
        });

    ArduinoOTA.onEnd([]() {
            Serial.println("\nEnd");
        });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        });

    ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                Serial.println(F("Auth Failed"));
            } else if (error == OTA_BEGIN_ERROR) {
                Serial.println(F("Begin Failed"));
            } else if (error == OTA_CONNECT_ERROR) {
                Serial.println(F("Connect Failed"));
            } else if (error == OTA_RECEIVE_ERROR) {
                Serial.println(F("Receive Failed"));
            } else if (error == OTA_END_ERROR) {
                Serial.println(F("End Failed"));
            }
        });

    // OTA
    ArduinoOTA.begin();
    Serial.println(F("OTA Ready."));

    // time
    sprintf(stringTime, "%s", defaultTime);
    Udp.begin(localPort);
    setSyncProvider(getNtpTime);
    setSyncInterval(300);
    if(!updateLocalTime()){
        Serial.println(F("Time update failed"));
        sprintf(stringTime, "%s", defaultTime);
    }
    oldmin = 99;

    // MQTT
    mqttClient.setServer(mqtt_server, mqtt_serverport);
    mqttClient.setCallback(mqttCallback);
    if(initMQTT()) {
        Serial.println(F("MQTT connected."));
    } else {
        Serial.println(F("MQTT failed."));
    }

    // LittleFS
    LittleFSConfig cfg;
    LittleFS.setConfig(cfg);
    LittleFS.begin();
    Serial.println(F("Loading config"));
    File f = LittleFS.open("/awning.cnf", "r");
    if (!f) {
        //File does not exist -- first run or someone called format()
        //Will not create file; run save code to actually do so (no need here since
        //it's not changed)
        Serial.println(F("Failed to open config file"));
        awningPosition = false;
    } else {
        while(f.available()){
            String key = f.readStringUntil('=');
            String value = f.readStringUntil('\n');
            Serial.println(key + F(" = [") + value + ']');
            Serial.println(key.length());
            if (key == F("awningPosition")) {
                awningPosition = value.toInt();
            }
        }
        Serial.println(F("Config file"));
    }
    f.close();

    mqttRefreshConfig();

    // clean-up
    Serial.println(F("Boot complete."));
    delay(1000);
}


/*
 * loop
 */
void loop() {
    ArduinoOTA.handle();

    currentMillis = millis();

    // Wifi status & init if dropped
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println(F("Reconnecting WiFi."));
        if(initWifi()) {
            Serial.println(F("WiFi connected."));
        }
    } else {
        // MQTT status & init if dropped
        if(!mqttClient.connected()) {
            Serial.println(F("Reconnecting MQTT."));
            if(initMQTT()) {
                Serial.println(F("MQTT connected."));
            }
        } else {
            mqttClient.loop();
        }
    }

    // MQTT Heartbeat
    if(currentMillis - hbMillis > hbInterval) {
        hbMillis = currentMillis;
        heartbeat = not(heartbeat);
        if(mqttClient.connected()) {
            mqttClient.publish(topic_status_hb, message_status_hb[heartbeat] , true);
        }
    }

    // update Local Time
    updateLocalTime();

    // flash local led hb if any non-standard condition otherwise off
    if(WiFi.status() != WL_CONNECTED || !mqttClient.connected()) {
        if(currentMillis - ledMillis > ledInterval) {
            ledMillis = currentMillis;
            ledState = not(ledState);
            digitalWrite(LED_BUILTIN, !ledState);
        }
    } else {
        ledState = false;
        digitalWrite(LED_BUILTIN, !ledState);
    }

// Web Client
// Listening for new clients & serve them
    webClient();

// update the config file if required
    if(cfgChangeFlag) {
        saveConfig();
        cfgChangeFlag = false;
    }
}