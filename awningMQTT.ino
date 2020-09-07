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
 */

#include <ESP8266WiFi.h>
#include <OneWire.h>
//#include <DallasTemperature.h>
#include <PubSubClient.h>

/* Passwords & Ports
 * wifi: ssid, password
 * ISY: hash, isy, isyport
 * MQTT mqtt_server, mqtt_serverport
 */
#include <../../../../../../../../../Projects/keys/sej/sej.h>


// Web Server on port 80
WiFiServer server(80);
String ServerTitle = "Jenkins Living Room Awning";

// MQTT
const char* topic = "sej"; // main topic
String clientId = "awning"; // client ID for this unit

const char* topic_temp = "sej/awning/temp"; // temp topic NOTE not implemented

const char* topic_control = "sej/awning/control/trigger"; // control topic
const char* control_message1 = "ON"; // control message1 only as trigger

const char* topic_monitor = "sej/awning/status/trigger"; // bounce back
const char* monitor_message1 = "ON"; // monitor message1
const char* monitor_message2 = "OFF"; // monitor message2

const char* topic_monitor2 = "sej/awning/status/position"; // monitor topic NOTE not implemented
const char* monitor2_message1 = "IN"; // monitor message1 NOTE would require new poly device
const char* monitor2_message2 = "OUT"; // monitor message2 NOTE unless use ON OFF

const char* topic_hb = "sej/awning/status/hb"; // hb topic
const char* hb_message1 = "ON"; // hb message1
const char* hb_message2 = "OFF"; // hb message2

const char* willTopic = topic; // will topic
byte willQoS = 0;
boolean willRetain = false;
const char* willMessage = ("lost connection " + clientId).c_str();
float heartbeat=0; // heartbeat to mqtt

WiFiClient espClient;
PubSubClient mqttClient(espClient);
long mqttLastMsg = 0;
int mqttValue = 0;

// Data wire is plugged into pin D1 on the ESP8266 12-E - GPIO 5
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire oneWire(ONE_WIRE_BUS); //removed for now

// Pass our oneWire reference to Dallas Temperature.
//DallasTemperature DS18B20(&oneWire); // removed for now
char temperatureCString[7];
char temperatureFString[7];
char outstr[7];
float tempC;
float tempF;
float tempOld;

// time
unsigned long currentMillis;
unsigned long tempMillis = 0;
unsigned long resetwifiMillis = 0;
unsigned long tempInterval = 30000; // minimum 10s for DS18B20
unsigned long resetwifiInterval = 60000;
unsigned long mqttMillis = 0;
unsigned long mqttInterval = 30000;
unsigned long awningcontrolMillis = 0;
unsigned long awningcontrolInterval = 30000;

// awning
int awningpushtime = 2000;
int control = true; // allow awning control periodically

int pbpower = 12; // Pin D2 GPIO 0 12
int pbbutton = 13; // Pin D3 GPIO 4 13
int statusled = 16; // status led


/*
 * Setup
 */
void setup(){
	Serial.begin(9600);
	delay(10);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH

    // IC Default 9 bit. If you have troubles consider upping it 12.
    // Ups the delay giving the IC more time to process the temperature measurement
	//DS18B20.begin(); // removed for now

    initWifi();
    mqttClient.setServer(mqtt_server, mqtt_serverport);
    mqttClient.setCallback(mqttCallback);
	getTemperature(); // does nothing
    tempOld = 0;

    control = true;

    // prepare GPIO4
    pinMode(pbpower, OUTPUT);
    digitalWrite(pbpower, LOW);
    delay(1);
    pinMode(pbbutton, OUTPUT);
    digitalWrite(pbbutton, LOW);
    delay(1);
}

/*
 * Main Loop
 */
void loop(){
    currentMillis = millis();

    // Init Wifi if dropped
    if(WiFi.status() != WL_CONNECTED) {
        initWifi();
    }

    // Init MQTT if dropped
    if(mqttClient.connected()) {
        mqttClient.loop();
    } else {
        mqttReconnect();
    }

    // Temperature retrieve
    if(currentMillis - tempMillis > tempInterval) {
        tempMillis = currentMillis;
        getTemperature();
        if (tempOld != tempF){
            // insert publishing when implemented
        }
    }

    // Heartbeat
    if(currentMillis - resetwifiMillis > resetwifiInterval) {
        resetwifiMillis = currentMillis;
        if(heartbeat == 0){
            heartbeat = 1;
            if(mqttClient.connected()) {
                mqttClient.publish(topic_hb, monitor_message1 , false);
            }
        }
        else {
            heartbeat = 0;
            if(mqttClient.connected()) {
                mqttClient.publish(topic_hb, monitor_message2 , false);
            }
        }
    }

    // Awning Control only so often
    if(currentMillis - awningcontrolMillis > awningcontrolInterval) {
        control = true;
    }

    // Web Client

    // Listening for new clients
    WiFiClient client = server.available();
    if (client) {
        Serial.println("New client");
        // bolean to locate when the http request ends
        boolean blank_line = true;
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n' && blank_line) {
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-Type: text/html");
                    client.println("Connection: close");
                    client.println();
                    // your actual web page that displays temperature
                    client.println("<!DOCTYPE HTML><html><head></head><body><h1>");
                    client.println(ServerTitle);
                    client.println("</h1><h3>Temperature in Celsius: ");
                    client.println(temperatureCString);
                    client.println("*C</h3><h3>Temperature in Fahrenheit: ");
                    client.println(temperatureFString);
                    client.println("*F</h3></body></html>");
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
        Serial.println("WebClient disconnected.");
    }
}


// Subroutines:

/*
 * Establish Wi-Fi connection & start web server
 */
void initWifi() {
	Serial.println("");
	Serial.print("Connecting to: ");
	Serial.print(ssid);
	WiFi.begin(ssid, password);

	int timeout = 25 * 4; // 25 seconds
	while(WiFi.status() != WL_CONNECTED  && (timeout-- > 0)) {
		delay(250);
		Serial.print(".");
	}
	Serial.println("");

	if(WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Failed to connect");
	}
	else {
        Serial.print("WiFi connected in: ");
        Serial.print(currentMillis);
        Serial.print(", IP address: ");
        Serial.println(WiFi.localIP());

        // Starting the web server
        server.begin();
        Serial.println("Web server running....");
	}
}


/*
 * MQTT client reconnect
 */
void mqttReconnect() {
    while (!mqttClient.connected() && (currentMillis - mqttMillis > mqttInterval)) {
        Serial.print("Attempting MQTT connection...");

        // Attempt to connect
        if (mqttClient.connect(clientId.c_str(), willTopic, willQoS, willRetain, willMessage)) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            mqttClient.publish(topic, ("connected " + clientId).c_str() , true );
            // ... and resubscribe
            // topic + clientID + in
            mqttClient.subscribe(topic_control);
            Serial.print("subscribed to : ");
            Serial.println(topic_control);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.print(" wifi=");
            Serial.println(WiFi.status());
            mqttMillis = currentMillis;
        }
    }
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

    if (strcmp((char *)mypayload, (char *)control_message1) == 0) { // AwningTrigger
        AwningControl();
    }
}


/*
 * Awning signal control
 */
void AwningControl() {
    Serial.println("request awning control");
    if(control == true){
        digitalWrite(pbpower, HIGH);
        digitalWrite(pbbutton, HIGH);
        Serial.println("**awning button pushed**");
        mqttClient.publish(topic_monitor, monitor_message1, false);
        delay(awningpushtime);

        digitalWrite(pbbutton, LOW);
        digitalWrite(pbpower, LOW);
        Serial.println("**awning button released**");
        mqttClient.publish(topic_monitor, monitor_message2, true);
        mqttClient.publish(topic_control, monitor_message2, true); // clear control msg

        awningcontrolMillis = currentMillis;
        control = false;
    }
}


/*
 * GetTemperature from DS18B20
 */
void getTemperature() {
    return;
    /* do { */
    /*         DS18B20.requestTemperatures(); */
    /*         tempC = DS18B20.getTempCByIndex(0); */
    /*         dtostrf(tempC, 2, 2, temperatureCString); */
    /*         tempF = DS18B20.getTempFByIndex(0); */
    /*         dtostrf(tempF, 3, 2, temperatureFString); */
    /*         delay(100); */
    /*     } while (tempC == 85.0 || tempC == (-127.0)); */
    /*     Serial.print(ServerTitle); */
    /*     Serial.print("Temperature in Celsius: "); */
    /*     Serial.print(temperatureCString); */
    /*     Serial.print("   Temperature in Fahrenheit: "); */
    /*     Serial.println(temperatureFString); */
}
