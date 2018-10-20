/*
 * This code is copied from https://github.com/Roboy/the_claw
 */

/* The ESP32 has four SPi buses, however as of right now only two of
 * them are available to use, HSPI and VSPI. Simply using the SPI API 
 * as illustrated in Arduino examples will use HSPI, leaving VSPI unused.
 * 
 * However if we simply intialise two instance of the SPI class for both
 * of these buses both can be used. However when just using these the Arduino
 * way only will actually be outputting at a time.
 * 
 * Logic analyser capture is in the same folder as this example as
 * "multiple_bus_output.png"
 * 
 * created 30/04/2018 by Alistair Symonds
 */
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <SPI.h>
#include <Print.h>

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "ai4robotics"
#define WLAN_PASS       "84jf93gh03!"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "192.168.0.224"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "roboy"
#define AIO_KEY         "11111"

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/
Adafruit_MQTT_Subscribe pos = Adafruit_MQTT_Subscribe(&mqtt, "motor/pos", MQTT_QOS_1);

static const int spiClk = 2000000; // 2 MHz

struct SPISTREAM {
  union {
    struct {
      uint16_t startOfFrame = 0x8000;
      int16_t pwmRef;
      uint16_t controlFlags1 : 16;
      uint16_t controlFlags2 : 16;
      uint16_t dummy : 16;
      int32_t actualPosition : 32;
      int16_t actualVelocity : 16;
      int16_t actualCurrent : 16;
      int16_t springDisplacement : 16;
      int16_t sensor1 : 16;
      int16_t sensor2 : 16;
    };
    uint8_t TxBuffer[24];
    uint16_t TxBuffer2[12];
  };
};

static volatile SPISTREAM motor;

//uninitalised pointers to SPI objects
SPIClass * hspi = NULL;

static volatile float KP = 0.2, KD = 0, prev_error = 0, setpoint = 0;

int pwm_controller(int32_t current_val, int32_t setpoint){
  
  float error = KP * (setpoint - current_val);
  float result = error + KD*(prev_error - error);
  if(result>=1000)
    result = 1000;
  else if(result <= -1000)
    result = -1000;
  prev_error = error;

  return result;
}

void positioncallback(double value){
    Serial.printf("position callback %lf\n", value);
    setpoint = value;
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifndef STANDALONE
      Serial.print(".");
    #endif
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  hspi = new SPIClass(HSPI);
  
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin(); 
  motor.pwmRef = 0;
  
  //alternatively route through GPIO pins
  //hspi->begin(25, 26, 27, 32); //SCLK, MISO, MOSI, SS

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(26, OUTPUT); // SS

  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    Serial.println("mqtt already connected");
    return;
  }

  pos.setCallback(positioncallback);
  mqtt.subscribe(&pos);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  #ifndef STANDALONE
    Serial.print("Connecting to MQTT... ");
  #endif
  
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection");
       mqtt.disconnect();
       retries--;
//       if (retries == 0) {
//         // basically die and wait for WDT to reset me
//         while (1);
//       }
  }
  #ifndef STANDALONE
    Serial.println("MQTT Connected!");
  #endif
}

void loop() {
// Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  motor.pwmRef = pwm_controller(motor.actualPosition, setpoint);
  
  //use the SPI buses
  hspiCommand();
//  Serial.printf("----------------------------------------\n",motor.startOfFrame);
//  Serial.printf("startOfFrame:       %d\n",motor.startOfFrame);
  Serial.printf("pwmRef:             %d\n",motor.pwmRef);
//  Serial.printf("controlFlags1:      %d\n",motor.controlFlags1);
//  Serial.printf("controlFlags2:      %d\n",motor.controlFlags2);
//  Serial.printf("dummy:              %d\n",motor.dummy);
  Serial.printf("actualPosition:     %d\n",motor.actualPosition);
//  Serial.printf("actualVelocity:     %d\n",motor.actualVelocity);
//  Serial.printf("actualCurrent:      %d\n",motor.actualCurrent);
//  Serial.printf("springDisplacement: %d\n",motor.springDisplacement);
//  Serial.printf("sensor1:            %d\n",motor.sensor1);
//  Serial.printf("sensor2:            %d\n",motor.sensor2);

  // this is our 'wait for incoming subscription packets and callback em' busy subloop
  // try to spend your time here:
  mqtt.processPackets(1);
  
  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  
//  if(! mqtt.ping()) {
//    mqtt.disconnect();
//  }
}

void hspiCommand() {
  hspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE1));
  bool write = true;
  uint16_t data[2];
  for(int i=0;i<12;i++){
    if(i==4){
      write = false;
    }
    digitalWrite(26, LOW);
    delayMicroseconds(1);
    if(i==0)
      hspi->transfer16(motor.TxBuffer2[i]);
    else if(i==1)
      hspi->transfer16((motor.TxBuffer2[i]& 0x7fff));
    else{
      switch(i){
        case 4:
          data[0] = hspi->transfer16(0);
          break;  
        case 5:
          data[1] = hspi->transfer16(0);
          motor.actualPosition =  ((data[0]>>8)<<24|(data[0]&0xff)<<16|(data[1]>>8)<<8|(data[1]&0xff));
          break; 
        case 6:
          motor.actualVelocity = hspi->transfer16(0);
          break; 
        case 7:
          motor.actualCurrent = hspi->transfer16(0);
          break; 
        default:
         hspi->transfer16(0);
         break;
        }
      }
    digitalWrite(26, HIGH);
    delayMicroseconds(1);
  }
  hspi->endTransaction();
}
