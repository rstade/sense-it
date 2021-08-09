/*
    Name:       sense-it.ino
    Created:	09.08.2021 14:46:55
    Author:     Bodo7\Rainer
*/

// configure the sensors being used:
#define FEATURE_MHZ19
#define FEATURE_BME280
#undef FEATURE_DS18B20

#include <BluetoothSerial.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <MQTT.h>

#ifdef FEATURE_BME280
	#include <Adafruit_Sensor.h>
	#include <Adafruit_BME280.h>
#endif

#ifdef FEATURE_MHZ19
	#include "MHZ19.h"
#endif

#ifdef FEATURE_DS18B20
	#include <OneWire.h>
	#include <DallasTemperature.h>
#endif

#include <HardwareSerial.h>          
#include "driver/uart.h"

#define MEASURE_PERIOD 120000 // in millis
#define MEASURE_PERIOD_DS18B20 5000 // in millis
#define MIN_TEMP_CHANGE 0.5  // minimum temperature change for publishing

#define NO_NVDATA 5 // number of nonvolatile strings
#define SLOTSZ 32
#define NO_SLOTS 8 // it must be NO_SLOTS >= NO_NVDATA
#define NO_KEYS NO_NVDATA+1 // number of keywords

// setup a unique name for Bluetooth network and MQTT client:
const char* cUniqueID="CO2_sensor";
//const char* cUniqueID="WW_Temp"; 

const char* cVersion="v0.20";


String ssid= "";
String password= "";
String rootSubTopic="";
String rootPubTopic="";
String brokerAddress="";

String* nvStrings[NO_NVDATA] = { &ssid, &password, &rootSubTopic, &rootPubTopic, &brokerAddress };

const char* keywords[NO_KEYS] = {
	"SSID!",
	"PW!",
	"SubTopic!",   // so far not used
	"PubTopic!",   // e.g. /rainer
	"Broker!",
	"quit",
};

// for each managed string the clear text used in BT dialogs
const char* clearText[NO_KEYS] = {
	"SSID",
	"password",
	"subscription topic",
	"publication topic",
	"broker address",
	"quit",
};

WiFiClient net;
MQTTClient client;
BluetoothSerial BTSerial;
char msg[64];
char topic[64];
unsigned long lastMillis = 0;
unsigned long lastMillis_DS18B20 = 0;

RTC_DATA_ATTR float temp = 0.;
RTC_DATA_ATTR float pressure = 0.;
RTC_DATA_ATTR float pressureNN = 0.;
RTC_DATA_ATTR float humidity = 0.;
RTC_DATA_ATTR int co2_ppm = 0;
RTC_DATA_ATTR float last_temp0 = 0.;
RTC_DATA_ATTR float last_temp1 = 0.;

#ifdef FEATURE_BME280
Adafruit_BME280 bme; // I2C
#endif

#ifdef FEATURE_MHZ19
MHZ19 mhz19;
#endif

#ifdef FEATURE_DS18B20
// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);
#endif

enum dialogState {
	KEYWORD,
	ENTERSTRING,
	CONFIRM,
};

uint8_t indexOfKeyword(String &keyword) {
	// returns index of keyword,  if not recognized returns 255
	uint8_t i=255;
	for (i=0; i< NO_KEYS; i++) {
		if (keyword == keywords[i]) break;
	}
	if (i< NO_KEYS) return i; else return 255;
}


int getAddressOfPromSlot(unsigned int indexKeyword) {
	int addr=0;
	// we use fixed slots of SLOTSZ bytes each
	if (indexKeyword < 8) return indexKeyword*SLOTSZ;
	else return -1;
}


bool storeIntoProm(unsigned int indexKeyword, String& buffer) {
	unsigned int addr=0;
	int _addr=getAddressOfPromSlot(indexKeyword);
	if(_addr<0) return false; else addr=(unsigned int) _addr;
	unsigned int size=buffer.length();
	if (size > 31) return false; // space for terminating \0 required
	else {
		for(unsigned int i=0; i<size; i++){
			EEPROM.write(addr+i,buffer[i]);
			//Serial.print(buffer[i]);
		}
		//Serial.println("");
		EEPROM.write(addr+size,'\0');   //Add termination null character
		EEPROM.commit();
		return true;
	}
}

String readFromProm(unsigned int indexKeyword) {
	unsigned int addr=0;
	int _addr=getAddressOfPromSlot(indexKeyword);
	//Serial.print("reading from address ");Serial.println(_addr);
	if(_addr<0) return String(""); else addr=(unsigned int) _addr;
	char buf[SLOTSZ];
	int i=0;
	char c;
	c= EEPROM.read(addr);
	while(c != '\0' && i<31) {  // read until null character
		buf[i]=c;
		//Serial.print(c);
		i++;
		c= EEPROM.read(addr+i);
	}
	//Serial.print("\nread characters: "); Serial.println(i);
	buf[i]='\0';
	return String(buf);
}

bool readNVData() {
	for (int i=0; i<NO_NVDATA; i++) {
		*nvStrings[i]=readFromProm(i);
	}
}


bool runBTDialogue(unsigned int timeoutMillis) {
	String stringToStore;
	String buffer;
	int indexKeyword;
	dialogState currentState=dialogState::KEYWORD;
	if (BTSerial.connected(timeoutMillis)) {
		unsigned long startMillis = millis();
		BTSerial.print(cUniqueID);
		BTSerial.print(" ");
		BTSerial.println(cVersion);
		BTSerial.println("?");
		while (millis() - startMillis < 4*timeoutMillis) {
			if (BTSerial.available()) {
				buffer=BTSerial.readStringUntil('\n');
				buffer.trim();
				switch (currentState) {
					case dialogState::KEYWORD:
					indexKeyword= indexOfKeyword(buffer);
					if (indexKeyword < NO_NVDATA) {
						BTSerial.println("enter "+String(clearText[indexKeyword]));
						currentState= dialogState::ENTERSTRING;
						} else if (indexKeyword == NO_KEYS-1) {
						BTSerial.println("quitting dialog");
						return true;
						} else {
						BTSerial.println("unknown keyword");
					}
					break;
					case dialogState::ENTERSTRING:
					stringToStore=buffer.substring(0,min(31u,buffer.length()));
					BTSerial.println("store new "+ String(clearText[indexKeyword])+" '"+stringToStore+"' ? (Y, n)");
					currentState=dialogState::CONFIRM;
					break;
					case dialogState::CONFIRM: default:
					if (buffer.startsWith("Y")) {
						if (storeIntoProm(indexKeyword, stringToStore)) BTSerial.println("done"); else BTSerial.println("failed");
					}
					currentState=KEYWORD;
					break;
				}
				// we expect some input
				BTSerial.println("?");
			}
			delay(100);
		}
		BTSerial.println("timeout");
	}
	else Serial.println("timeout waiting for BT connection");
	return false;
}

size_t print2(const String& s) {
	BTSerial.print(s);
	return Serial.print(s);
}

size_t print2ln(const String& s) {
	BTSerial.println(s);
	return Serial.println(s);
}

size_t print2(const char* s) {
	BTSerial.print(s);
	return Serial.print(s);
}

size_t print2ln(const char* s) {
	BTSerial.println(s);
	return Serial.println(s);
}

size_t print2(unsigned char n) {
	BTSerial.print(n);
	return Serial.print(n);
}

size_t print2ln(unsigned char n) {
	BTSerial.println(n);
	return Serial.println(n);
}

size_t print2(int n) {
	BTSerial.print(n);
	return Serial.print(n);
}

size_t print2ln(int n) {
	BTSerial.println(n);
	return Serial.println(n);
}

size_t print2(unsigned int n) {
	BTSerial.print(n);
	return Serial.print(n);
}

size_t print2ln(unsigned int n) {
	BTSerial.println(n);
	return Serial.println(n);
}

size_t print2ln() {
	BTSerial.println();
	return Serial.println();
}

size_t print2ln(const IPAddress& ipaddr ) {
	BTSerial.println(ipaddr);
	return Serial.println(ipaddr);
}

size_t print2(float& d) {
	BTSerial.print(d);
	return Serial.print(d);
}

size_t print2ln(float& d) {
	BTSerial.println(d);
	return Serial.println(d);
}

bool connect2broker() {
	bool success;
	print2("\nConnecting to broker: " + brokerAddress);
	client.setKeepAlive(60);
	client.begin(brokerAddress.c_str(), net);
	client.onMessage(messageReceived);
	success=client.connect(cUniqueID, "public", "public");
	if (success) {
		/*
		Serial.print("\nconnected to broker! \nTrying to subscribe to topic "+rootSubTopic+"#: ");
		BTSerial.print("\nconnected to broker! \nTrying to subscribe to topic "+rootSubTopic+"#: ");
		if (client.subscribe(rootSubTopic+"#")) {
			Serial.println("subscription successful");
			BTSerial.println("subscription successful");
		}
		else {
			Serial.println("subscription failed");
			BTSerial.println("subscription failed");
		}
		*/
		print2("\nconnected to broker! \n");
	}
	else { 
		lwmqtt_err_t err=client.lastError();
		print2("client not connected to broker, error= ");
		print2ln(err);
	}
	return success;
}

void messageReceived(String &topic, String &payload) {
	print2ln("incoming: " + topic + " - " + payload);


	// Note: Do not use the client in the callback to publish, subscribe or
	// unsubscribe as it may cause deadlocks when other things arrive while
	// sending and receiving acknowledgments. Instead, change a global variable,
	// or push to a queue and handle it in the loop after calling `client.loop()`.
}

void publishValues() {

#ifdef FEATURE_BME280
	print2("Temperature = ");
	print2(temp);
	print2(" *C --> ");

	sprintf(msg,"%3.1f",temp);
	sprintf(topic,"%s/bme280/temperature", rootPubTopic.c_str());
	print2ln(topic);
	client.publish(topic, msg);
	
	print2("Pressure = ");
	print2(pressure);
	print2(" hPa --> ");

	sprintf(msg,"%4.1f",pressure);
	sprintf(topic,"%s/bme280/pressure", rootPubTopic.c_str());
	print2ln(topic);
	client.publish(topic, msg);
	
	sprintf(msg,"%4.1f",pressureNN);
	sprintf(topic,"%s/bme280/pressure_sea", rootPubTopic.c_str());
	client.publish(topic, msg);

	print2("Humidity = ");
	print2(humidity);
	print2(" % --> ");

	sprintf(msg,"%3.1f",humidity);
	sprintf(topic,"%s/bme280/humidity", rootPubTopic.c_str());
	print2ln(topic);
	client.publish(topic, msg);
#endif

#ifdef FEATURE_MHZ19	
	print2("CO2: ");
	print2(co2_ppm);
	print2(" ppm --> ");
	
	sprintf(msg,"%d",co2_ppm);
	sprintf(topic,"%s/CO2/co2ppm", rootPubTopic.c_str());
	print2ln(topic);
	client.publish(topic, msg);
#endif

#ifdef FEATURE_BME280 || FEATURE_MHZ19
	print2ln();
#endif
}

#ifdef FEATURE_DS18B20
	void publishTemp(int index, float temp) {
		
		sprintf(msg,"%3.1f",temp);
		//sprintf(topic, "/heizung/warmwasser/temperature/%d", index );
		sprintf(topic, "%s/%d", rootPubTopic.c_str(), index );
		client.publish(topic, msg);

		print2("published to topic ");
		print2(topic);
		print2(": ");
		print2ln(msg);
	}
	
	// function to print a device address
	void printAddress(DeviceAddress deviceAddress) {
		for (uint8_t i = 0; i < 8; i++){
			if (deviceAddress[i] < 16) Serial.print("0");
			Serial.print(deviceAddress[i], HEX);
		}
	}

	// Number of temperature devices found
	int numberOfDevices;
	// We'll use this variable to store a found device address
	DeviceAddress tempDeviceAddress;
#endif


#ifdef FEATURE_MHZ19
void printDeviceInfo(MHZ19& mhz19) {
  /*
    getVersion(char array[]) returns version number to the argument. The first 2 char are the major 
    version, and second 2 bytes the minor version. e.g 02.11
  */

  char myVersion[4];          
  mhz19.getVersion(myVersion);

  Serial.print("\nFirmware Version: ");
  for(byte i = 0; i < 4; i++)
  {
    Serial.print(myVersion[i]);
    if(i == 1)
      Serial.print(".");    
  }
   Serial.println("");

   Serial.print("Range: ");
   Serial.println(mhz19.getRange());   
   Serial.print("Background CO2: ");
   Serial.println(mhz19.getBackgroundCO2());
   Serial.print("Temperature Cal: ");
   Serial.println(mhz19.getTempAdjustment());
   Serial.print("ABC Status: "); mhz19.getABC() ? Serial.println("ON") :  Serial.println("OFF");
	
}
#endif

bool waitForWifi() {
	print2ln();
	print2("Connecting to ");
	print2ln(ssid);
	unsigned int tries= 10;
	while (WiFi.status() != WL_CONNECTED && tries > 0) {
		delay(500);
		print2(".");
		tries --;
	}
	print2ln();
	return WiFi.status() == WL_CONNECTED;
}


void setup()
{

	Serial.begin(115200); // USB interface
	BTSerial.begin(cUniqueID);
	if (!EEPROM.begin(NO_SLOTS*SLOTSZ)) {
		print2ln("cannot initialize EEPROM");
	}
	runBTDialogue(30000);
	readNVData();

	Serial2.begin(9600);  // interface to MH-Z19
	WiFi.begin(ssid.c_str(), password.c_str());
#ifdef FEATURE_MHZ19	
	//MHZ19 sensor
	mhz19.begin(Serial2);
	mhz19.autoCalibration();
	printDeviceInfo(mhz19);
#endif

#ifdef FEATURE_BME280
    // BME 280 sensor
    unsigned status;
    status = bme.begin(0x76);
    if (!status) {
	    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
	    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
	    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
	    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
	    Serial.print("        ID of 0x60 represents a BME 280.\n");
	    Serial.print("        ID of 0x61 represents a BME 680.\n");
	 } else {
	    temp = bme.readTemperature();
	    pressure = bme.readPressure() / 100.0F;
	    pressureNN = bme.seaLevelForAltitude(645.0, pressure);
	    humidity = bme.readHumidity();
	}
#endif

#ifdef FEATURE_DS18B20
	//DS18B20
	sensors.begin();
	// Grab a count of devices on the wire
	numberOfDevices = sensors.getDeviceCount();
	
	// Loop through each device, print out address
	for(int i=0;i<numberOfDevices; i++) {
		// Search the wire for address
		if(sensors.getAddress(tempDeviceAddress, i)) {
			print2("Found device ");
			print2(i);
			print2(" with address: ");
			printAddress(tempDeviceAddress);
			print2ln();
		} else {
			print2("Found ghost device at ");
			print2(i);
			print2ln(" but could not detect address. Check power and cabling");
		}
	}
#endif

	if (waitForWifi()) {
		print2ln("WiFi connected");
		print2ln("IP address: ");
		print2ln(WiFi.localIP());
		connect2broker();
	}
}


void measure() {
#ifdef FEATURE_BME280
	temp = bme.readTemperature();
	pressure = bme.readPressure() / 100.0F;
	pressureNN = bme.seaLevelForAltitude(645.0, pressure);
	humidity = bme.readHumidity();
#endif
#ifdef FEATURE_MHZ19	
	co2_ppm = mhz19.getCO2();
#endif
	publishValues();
}

#ifdef FEATURE_DS18B20
	void measure_DS18B20() {
		sensors.requestTemperatures(); // Send the command to get temperatures
		// Loop through each device, print out temperature data
		for(int i=0;i<numberOfDevices; i++){
			float last_temp=0.;
			if (i==0) last_temp=last_temp0;
			if (i==1) last_temp=last_temp1;
			// Search the wire for address
			if(sensors.getAddress(tempDeviceAddress, i)){
				// publish the data
				float tempC = sensors.getTempC(tempDeviceAddress);
				if (fabs(tempC-last_temp) >= MIN_TEMP_CHANGE) {
					publishTemp(i, tempC);
					if (i==0) last_temp0=tempC;
					if (i==1) last_temp1=tempC;
				}
			}
		}
	}
#endif

void loop()
{
	client.loop();	// keep alive etc.
	unsigned long now = millis();

	if (lastMillis == 0 || (now - lastMillis >= min (MEASURE_PERIOD, MEASURE_PERIOD_DS18B20))) {
		if (WiFi.status() != WL_CONNECTED) {
			print2ln("lost WiFi connection, trying to re-connect:");
			WiFi.begin(ssid.c_str(), password.c_str());
			if (waitForWifi()) {
				connect2broker();
			}
		}
		if (WiFi.status() == WL_CONNECTED) {
			if (client.connected()) {
				if (lastMillis == 0 || (now - lastMillis >= MEASURE_PERIOD)) measure();
#ifdef FEATURE_DS18B20
				if (lastMillis_DS18B20 == 0 || (now - lastMillis_DS18B20 >= MEASURE_PERIOD_DS18B20)) measure_DS18B20();
#endif
			}
			else {
				lwmqtt_err_t err=client.lastError();
				print2("client not connected to broker, error= ");
				print2ln(err);
				print2("Trying to reconnect. ");
				print2ln("Broker: " + brokerAddress);
				if (connect2broker()) {
					if (lastMillis == 0 || (now - lastMillis >= MEASURE_PERIOD)) measure();
#ifdef FEATURE_DS18B20
					if (lastMillis_DS18B20 == 0 || (now - lastMillis_DS18B20 >= MEASURE_PERIOD_DS18B20)) measure_DS18B20();
#endif
				}
			}
		}
		if (lastMillis == 0 || now - lastMillis >= MEASURE_PERIOD) lastMillis = now;
	    if (lastMillis_DS18B20 == 0 ||now - lastMillis_DS18B20 >= MEASURE_PERIOD_DS18B20) lastMillis_DS18B20 = now;
		Serial.flush();
		BTSerial.flush();
	}

}

