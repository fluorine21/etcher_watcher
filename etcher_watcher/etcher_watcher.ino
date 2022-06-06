#include <WiFi.h> //Wifi library
#include "max6675.h"//Thermocouple readout
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include "LiquidCrystal_PCF8574.h"



#include <stdio.h>
#include <string.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

#include <WiFiClient.h>



String lip; //local ip

#define OTA_UPDATE_INTERVAL (2*60*1000)//2min
unsigned long ota_update_timer;


#define EAP_IDENTITY "jawillia" //if connecting from another corporation, use identity@organisation.domain in Eduroam 
#define EAP_USERNAME "jawillia" //oftentimes just a repeat of the identity
#define EAP_PASSWORD "T22[$Q,qGFG}" //your Eduroam password
const char* ssid = "Caltech Secure"; // Eduroam SSID
const char* host = "google.com"; //external server domain for HTTP connection after authentification
int counter = 0;

// NOTE: For some systems, various certification keys are required to connect to the wifi system.
//       Usually you are provided these by the IT department of your organization when certs are required
//       and you can't connect with just an identity and password.
//       Most eduroam setups we have seen do not require this level of authentication, but you should contact
//       your IT department to verify.
//       You should uncomment these and populate with the contents of the files if this is required for your scenario (See Example 2 and Example 3 below).
//const char *ca_pem = "insert your CA cert from your .pem file here";
//const char *client_cert = "insert your client cert from your .crt file here";
//const char *client_key = "insert your client key from your .key file here";


//Adafruit IO Definitions
#define IO_USERNAME "fluorine21"
#define IO_KEY "f7c7ec84639545c59312d8fdd3759489"
#define IO_SERVER "io.adafruit.com"
#define IO_SERVERPORT  1883 // use 8883 for SSL
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, IO_SERVER, IO_SERVERPORT, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Publish dpt = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.diff-pump-temp");
Adafruit_MQTT_Publish dpt_cw_in = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.diff-cw-in");
Adafruit_MQTT_Publish dpt_cw_out = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.diff-cw-out");
Adafruit_MQTT_Publish gun_cw_in = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.gun-cw-in");
Adafruit_MQTT_Publish gun_cw_out = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.gun-cw-out");
Adafruit_MQTT_Publish troom_mqtt = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.troom-mqtt");
Adafruit_MQTT_Publish diff_status_mqtt = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.diff-status");
Adafruit_MQTT_Publish ion_status_mqtt = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.ion-status");
Adafruit_MQTT_Publish ion_fullscale_mqtt = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.ion-fullscale");
Adafruit_MQTT_Publish ion_gauge_mqtt = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.ion-gauge");
#define publish_interval 30000 //30 second publish interval
unsigned long p_last;

//Pushbutton definitions
#define diff_reset_pin 22
#define diff_release_pin 23
int diff_reset;

//Ion gauge controll definitions
#define ion_on_pin 15
#define ion_off_pin 2
#define ion_fullscale_pin 36
#define ion_gauge_pin 39
#define ion_status_pin 5
#define ion_interval 10*60*60000 //every 10 hours
#define ion_warm_time 30*60000 //30 min for gauge to warm up
int ion_status;
int ion_state;
unsigned long ion_last;
float ion_fullscale, ion_gauge;//Holds the readings from the ion gauge
int ion_new;
int ar;

//LCD Definitions
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display
#define I2C_SDA 25
#define I2C_SCL 26


//Thermocouple definitions
float tdiff, tdiff_in, tdiff_out, tgun_in, tgun_out, troom;
int diff_status;
#define diff_status_pin 4
#define tdiff_max 140////////Diff setpoint

#define readout_interval 10000 //10 seconds for each readout
unsigned long readout_last;

//Diff pump
#define diff_thermo_clk 14
#define diff_thermo_cs 12 
#define diff_thermo_sd 33
MAX6675 diff_thermo(diff_thermo_clk, diff_thermo_cs, diff_thermo_sd);

//Diff water
#define diff_in_cs 27
MAX6675 diff_thermo_in(diff_thermo_clk, diff_in_cs, diff_thermo_sd);
#define diff_out_cs 13
MAX6675 diff_thermo_out(diff_thermo_clk, diff_out_cs, diff_thermo_sd);

//Gun water
#define gun_in_cs 32
MAX6675 gun_thermo_in(diff_thermo_clk, gun_in_cs, diff_thermo_sd);
#define gun_out_cs 18
MAX6675 gun_thermo_out(diff_thermo_clk, gun_out_cs, diff_thermo_sd);

//Room
#define room_cs 19
MAX6675 room_thermo(diff_thermo_clk, room_cs, diff_thermo_sd);

//Function headers
void MQTT_connect();
void i2c_scan();
void read_temps();
void check_buttons();
int time_check(unsigned long tref, unsigned long interval);
void check_wifi();
char * sci(double number, int digits);
void setup_webserver();
void OTA_Update();

//IRSr
int d_rst, d_rl;
void diff_reset_isr(){diff_status = 1; diff_reset = 1;d_rst = 1;}
void diff_release_isr(){diff_reset = 0;d_rl = 1;}

void setup() {

  //Variable initialization
  diff_status = 1;
  troom = 0;
  tgun_out = 0;
  p_last = 0;
  ion_status = 0;
  ion_state = 0;

  //Timers
  readout_last = 0;
  p_last = 0;
  ion_last = 0;

  pinMode(diff_status_pin, OUTPUT);

  //Pushbutton and interrupt configuration
  pinMode(diff_reset_pin, INPUT_PULLUP);
  pinMode(diff_release_pin, INPUT_PULLUP);
  attachInterrupt(diff_reset_pin, diff_reset_isr, FALLING);
  attachInterrupt(diff_release_pin, diff_release_isr, FALLING);
  
  Serial.begin(115200);
  delay(10);
  //Serial.println("Boot delay of 5 seconds");
  for(int i = 0; i < 5; i++)
  {
    //delay(1000);
    Serial.print(".");  
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  i2c_scan();
  lcd.begin(20, 4);
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.print("Booting"); 
  
  Serial.println();
  Serial.print("Connecting to network: ");
  Serial.println(ssid);
  WiFi.disconnect(true);  //disconnect form wifi to set new wifi connection
  WiFi.mode(WIFI_STA); //init wifi mode
  
  // Example1 (most common): a cert-file-free eduroam with PEAP (or TTLS)
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);

  // Example 2: a cert-file WPA2 Enterprise with PEAP
  //WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD, ca_pem, client_cert, client_key);
  
  // Example 3: TLS with cert-files and no password
  //WiFi.begin(ssid, WPA2_AUTH_TLS, EAP_IDENTITY, NULL, NULL, ca_pem, client_cert, client_key);
  
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    counter++;
    if(counter>=60){ //after 30 seconds timeout - reset board
      lcd.setCursor(4, 1);
      lcd.print("fail");
      ESP.restart();
    }

    if(counter%4==0)
    {
      lcd.print(".");  
    } 
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address set: "); 
  lip = WiFi.localIP().toString();
  Serial.println(lip); //print LAN IP
  lcd.setCursor(3, 1);
  lcd.print(lip);

}

void ion_read()
{
  //Do the fullscale read first
  int ion_vals = 0;
  for(int i = 0; i < 10; i++)
  {
    ion_vals += analogRead(ion_fullscale_pin);
    delay(10);  
  }
  ion_vals = ion_vals / 10;

  ion_fullscale = float(ion_vals) * ((float)(3.3/4095));
  ion_fullscale = ion_fullscale * (5/3.3);
  //ion_fullscale = (1e-3) * pow(10, -1*ion_fullscale);

  //Then do the gauge read
  ion_vals = 0;
  for(int i = 0; i < 10; i++)
  {
    ion_vals += analogRead(ion_gauge_pin);
    delay(10);  
  }
  ion_vals = ion_vals / 10;
  ion_gauge = float(ion_vals) * ((float)(3.3/4095));
  //ion_gauge = ion_gauge * (10/2.4);

  Serial.println("Ion fullscale was " + String(sci(ion_fullscale,2)) + ", gauge was " + String(sci(ion_gauge,2)));
  ion_new = 1;

}

int time_check(unsigned long tref, unsigned long interval)
{
  long long tnow = millis();
  if(tnow - tref > interval || tnow-((long long)tref)<0)
  {
    return 1;
  }
  return 0;
}

void service_ion()
{
  if(ion_state == 0)//wait and warmup
  {
    if(time_check(ion_last, ion_interval))
    {
      Serial.println("Ion gauge on");
      ion_last = millis();
      ion_state = 1;

      //Turn on the gauge and wait 5 seconds
      digitalWrite(ion_on_pin, HIGH);
      delay(10);
      digitalWrite(ion_on_pin, LOW);
      delay(5000);

      //Check if it's on
      ion_status = digitalRead(ion_status_pin);
        
    }
  }
  else if(ion_state == 1)//read
  {
    if(time_check(ion_last, ion_warm_time))
    {
      Serial.println("Ion gauge off");
      ion_last = millis();
      ion_state = 0;  
      
      //Check if it's on
      ion_status = digitalRead(ion_status_pin);

      //Take a reading
      ion_read();

      //Shut the gauge off
      digitalWrite(ion_off_pin, HIGH);
      delay(10);
      digitalWrite(ion_off_pin, LOW);
      
    } 
  }
  else{ion_state = 0;}
}

void update_lcd()
{
    lcd.home();
    lcd.clear();
    String ss = (diff_status ? "ON" : "OFF");
    lcd.print("Diff " + ss + " " + String(tdiff,1) + "C");
    if(diff_reset)
    {
      lcd.print(" RESET");
    }
    lcd.setCursor(0, 1);
    lcd.print("CW IN "+ String(tdiff_in,1) + " OUT " + String(tdiff_out,1));
    lcd.setCursor(0, 2);
    lcd.print("Gun IN "+String(tgun_in,1)+" OUT " +String(tgun_out,1));
    lcd.setCursor(0, 3);
    lcd.print("Rm "+String(troom,1) + " ");
    if(WiFi.status() == WL_CONNECTED)
    {
      lcd.print(lip);  
    }
    else
    {
      lcd.print("DOWN");  
    }
}

void publish_all()
{

  //Connected to wifi here
  MQTT_connect();  
  unsigned long tnow = millis();
  if(time_check(p_last, publish_interval))
  {
    p_last = tnow;
    if (! dpt.publish(tdiff)) {
      Serial.println(F("dpt publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
    if (! dpt_cw_in.publish(tdiff_in)) {
      Serial.println(F("dpt cw in publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
    if (! dpt_cw_out.publish(tdiff_out)) {
      Serial.println(F("dpt cw out publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
    if (! gun_cw_in.publish(tgun_in)) {
      Serial.println(F("gun cw in publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
    if (! gun_cw_out.publish(tgun_out)) {
      Serial.println(F("gun cw out publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
    if (! troom_mqtt.publish(troom)) {
      Serial.println(F("room publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
    if (! diff_status_mqtt.publish(diff_status)) {
      Serial.println(F("diff status publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
    if (! ion_status_mqtt.publish(ion_status)) {
      Serial.println(F("ion status publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }

    if (ion_new && ion_fullscale_mqtt.publish(ion_fullscale)) {
      Serial.println(F("publish OK!"));
      
    } else {
      Serial.println(F("ion fullscale publish failed"));
    }
    if (ion_new && ion_gauge_mqtt.publish(ion_gauge)) {
      Serial.println(F("publish OK!"));
      ion_new = 0;
    } else {
      Serial.println(F("ion gauge publish failed"));
    }
  
    if(! mqtt.ping()) {
      mqtt.disconnect();
    }
  }
}

void loop() {

  digitalWrite(diff_status_pin, diff_status);

  if(d_rst)
  {
    lcd.home();lcd.clear();lcd.print("DIFF RESET");  
    d_rst = 0;
    delay(2000);
  }
  if(d_rl)
  {
    lcd.home();lcd.clear();lcd.print("DIFF RELEASE");
    d_rl = 0;
    delay(2000);  
  }

  service_ion();

  unsigned long tnow = millis();
  if(time_check(readout_last, readout_interval))
  {
    readout_last = tnow;
    check_wifi();

    //Readout temperatures
    read_temps();
  
    publish_all();
  
    update_lcd();

    //Report the gauge reading for debugging porpoises
    Serial.println("Ion fullscale was " + String(sci(ion_fullscale,2)) + ", gauge was " + String(sci(ion_gauge,2)));

    OTA_Update();
  }
}



void check_wifi()
{
  if (WiFi.status() == WL_CONNECTED) { //if we are connected to Eduroam network
    counter = 0; //reset counter
    Serial.print("Wifi is still connected with IP: "); 
    Serial.println(WiFi.localIP());   //inform user about his IP address
  }else if (WiFi.status() != WL_CONNECTED) { //if we lost connection, retry
    WiFi.begin(ssid);      
  }
  while (WiFi.status() != WL_CONNECTED) { //during lost connection, print dots
    delay(500);
    Serial.print(".");
    counter++;
    if(counter>=60){ //30 seconds timeout - reset board
    ESP.restart();
    }
  }
}

float read_temp(MAX6675 * tc)
{
  int num_avgs = 10;
  float t_sum = 0;
  for(int i = 0; i < num_avgs; i++)
  {
      t_sum += tc->readCelsius();
      delay(1);
  }  
  return t_sum/num_avgs; 
}

void read_temps()
{
  //Readout temperatures
  Serial.print("Diff pump C = "); 
  tdiff = read_temp(&diff_thermo);
  Serial.println(tdiff);
  if(tdiff > tdiff_max && diff_reset == 0)
  {
    diff_status = 0;  
  }

  Serial.print("Diff pump water in C = "); 
  tdiff_in = read_temp(&diff_thermo_in);
  Serial.println(tdiff_in);
  
  Serial.print("Diff pump water out C = "); 
  tdiff_out = read_temp(&diff_thermo_out);
  Serial.println(tdiff_out);

  Serial.print("Gun water in C = "); 
  tgun_in = read_temp(&gun_thermo_in);
  Serial.println(tgun_in);

  Serial.print("Gun water out C = "); 
  tgun_out = read_temp(&gun_thermo_out);
  Serial.println(tgun_out);

  Serial.print("Room temp C = "); 
  troom = read_temp(&room_thermo);
  Serial.println(troom);

  ar = analogRead(36);
  Serial.println("VP was " + String(ar));
  ar = analogRead(39);
  Serial.println("VN was " + String(ar));
  

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         ESP.restart();
       }
  }
  Serial.println("MQTT Connected!");
}


void i2c_scan()
{
byte error, address;
  int nDevices;
  Serial.println("Scanning I2C bus...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          

}


char __mathHelperBuffer[17];


//////////////////////////////////////////////////
//
// FLOAT REPRESENTATION HELPERS
//
char * sci(double number, int digits)
{
  int exponent = 0;
  int pos = 0;

  // Handling these costs 13 bytes RAM
  // shorten them with N, I, -I ?
  if (isnan(number)) 
  {
    strcpy(__mathHelperBuffer, "nan");
    return __mathHelperBuffer;
  }
  if (isinf(number))
  {
    if (number < 0) strcpy(__mathHelperBuffer, "-inf");
    strcpy(__mathHelperBuffer, "inf");
    return __mathHelperBuffer;
  }

  // Handle negative numbers
  bool neg = (number < 0.0);
  if (neg)
  {
    __mathHelperBuffer[pos++] = '-';
    number = -number;
  }

  while (number >= 10.0)
  {
    number /= 10;
    exponent++;
  }
  while (number < 1 && number != 0.0)
  {
    number *= 10;
    exponent--;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
  {
    rounding *= 0.1;
  }
  number += rounding;
  if (number >= 10)
  {
    exponent++;
    number /= 10;
  }


  // Extract the integer part of the number and print it
  uint8_t d = (uint8_t)number;
  double remainder = number - d;
  __mathHelperBuffer[pos++] = d + '0';   // 1 digit before decimal point
  if (digits > 0)
  {
    __mathHelperBuffer[pos++] = '.';  // decimal point TODO:rvdt CONFIG?
  }


  // Extract digits from the remainder one at a time to prevent missing leading zero's
  while (digits-- > 0)
  {
    remainder *= 10.0;
    d = (uint8_t)remainder;
    __mathHelperBuffer[pos++] = d + '0';
    remainder -= d;
  }


  // print exponent
  __mathHelperBuffer[pos++] = 'E';
  neg = exponent < 0;
  if (neg)
  {
    __mathHelperBuffer[pos++] = '-';
    exponent = -exponent;
  }
  else __mathHelperBuffer[pos++] = '+';


  // 3 digits for exponent;           // needed for double
  // d = exponent / 100;
  // __mathHelperBuffer[pos++] = d + '0';
  // exponent -= d * 100;

  // 2 digits for exponent
  d = exponent / 10;
  __mathHelperBuffer[pos++] = d + '0';
  d = exponent - d*10;
  __mathHelperBuffer[pos++] = d + '0';

  __mathHelperBuffer[pos] = '\0';

  return __mathHelperBuffer;
}














long cv = 0 ;

#define WIFI_CONNECT_SSID ""        // To edit for your own code.
#define WIFI_CONNECT_PSWD ""        // To edit for your own code.
#define WIFI_CONNECT_WAIT_TIME  500
#define WIFI_CONNECT_WAIT_LIMIT 20
WiFiMulti wm ;

String otaDirURL      = "https://github.com/fluorine21/etcher_watcher/blob/main/etcher_watcher/" ; // To edit for your own code.
String otaCheckURL    = otaDirURL + "otaCheck.php" ;
String otaBinFilename = "etcher_watcher.ino.esp32.bin" ;                 // To edit for your own code.
String otaBinURL      = otaDirURL + otaBinFilename ;

#define EEPROM_ID_STR      "OTA_BASIC"
#define EEPROM_ID_STR_SIZE 10
#define EEPROM_SIZE        (EEPROM_ID_STR_SIZE + sizeof(cv))

void ReSet()
{  
  ESP.restart() ;  
}


void EEPROM_WriteData(void)
{
  int i ;
  byte data[EEPROM_SIZE] ;

  memcpy(data, EEPROM_ID_STR, EEPROM_ID_STR_SIZE) ;
  memcpy(data + EEPROM_ID_STR_SIZE, &cv, sizeof(cv)) ;
  for (i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, data[i]) ;
  EEPROM.commit() ;
}

void EEPROM_ReadData(void)
{
  int i ;
  byte data[EEPROM_SIZE] ;

  for (i = 0; i < EEPROM_SIZE; i++) data[i] = (byte)EEPROM.read(i) ;
  if (memcmp(data, EEPROM_ID_STR, EEPROM_ID_STR_SIZE - 1) == 0) {
    memcpy(&cv, data + EEPROM_ID_STR_SIZE, sizeof(cv)) ;
  }
}

void OTA_Update()
{
  //timer return
  if(!time_check(ota_update_timer, OTA_UPDATE_INTERVAL))
  {
    return;
  }
  ota_update_timer = millis();
  
  int hrc ;
  t_httpUpdate_return hur ;
  long nv ;
  HTTPClient http ;
  WiFiClient otaClient ;

  http.begin(otaCheckURL) ;
  hrc = http.GET() ;
  if (hrc > 0) {
    Serial.print("HTTP Response code : ") ;
    Serial.println(hrc) ;
    nv = http.getString().toInt() ;
    Serial.print("Current version : ") ;
    Serial.println(cv) ;
    Serial.print("    New version : ") ;
    Serial.println(nv) ;
  } else {
    Serial.print("Error code : ") ;
    Serial.println(hrc) ;
  }
  http.end() ;
  if (hrc <= 0) return ;
  if (nv > cv) { // New version
    Serial.println("OTA Updating...") ;
    httpUpdate.rebootOnUpdate(false) ;
    hur = httpUpdate.update(otaClient, otaBinURL) ;
    switch (hur) {
      case HTTP_UPDATE_FAILED :
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n\n",
         httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str()) ;
        break ;
      case HTTP_UPDATE_NO_UPDATES :
        Serial.println("HTTP_UPDATE_NO_UPDATES\n") ;
        break ;
      case HTTP_UPDATE_OK :
        cv = nv ;
        EEPROM_WriteData() ;
        Serial.println("HTTP_UPDATE_OK\n") ;
        ReSet() ;
        break ;
    }
  }
  Serial.println("OTA update: no update neccessary");
}
