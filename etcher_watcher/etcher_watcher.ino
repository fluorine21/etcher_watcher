#include <WiFi.h> //Wifi library
#include "max6675.h"//Thermocouple readout
#include "esp_wpa2.h" //wpa2 library for connections to Enterprise networks
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include "LiquidCrystal_PCF8574.h"


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
#define IO_SERVER      "io.adafruit.com"
#define IO_SERVERPORT  1883                   // use 8883 for SSL
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, IO_SERVER, IO_SERVERPORT, IO_USERNAME, IO_KEY);
Adafruit_MQTT_Publish dpt = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/etcher-feed-group.diff-pump-temp");
#define publish_interval 30000 //30 second publish interval
unsigned long p_last;

//LCD Definitions
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27 for a 16 chars and 2 line display
#define I2C_SDA 25
#define I2C_SCL 33


//Thermocouple definitions
float tdiff, tdiff_in, tdiff_out, tgun_in, tgun_out, troom;
int diff_status;
#define tdiff_max 30

//Diff pump
#define diff_thermo_clk 14
#define diff_thermo_cs 12
#define diff_thermo_sd 35
MAX6675 diff_thermo(diff_thermo_clk, diff_thermo_cs, diff_thermo_sd);

//Diff water
#define diff_in_cs 32
MAX6675 diff_thermo_in(diff_thermo_clk, diff_in_cs, diff_thermo_sd);
#define diff_out_cs 13
MAX6675 diff_thermo_out(diff_thermo_clk, diff_out_cs, diff_thermo_sd);

//Gun water
#define gun_in_cs 27
MAX6675 gun_thermo_in(diff_thermo_clk, gun_in_cs, diff_thermo_sd);
#define diff_out_cs 13
//MAX6675 diff_thermo_out(diff_thermo_clk, diff_out_cs, diff_thermo_sd);

//Function headers
void MQTT_connect();
void i2c_scan();


void setup() {

  //Variable initialization
  diff_status = 0;
  troom = 0;
  tgun_out = 0;
  p_last = 0;
  
  Serial.begin(115200);
  delay(10);
  Serial.println("Boot delay of 5 seconds");
  for(int i = 0; i < 5; i++)
  {
    delay(1000);
    Serial.print(".");  
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  i2c_scan();
  lcd.begin(20, 4);
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
  lcd.print("Booting..."); 
  
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
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address set: "); 
  Serial.println(WiFi.localIP()); //print LAN IP
  
}

void update_lcd()
{
    lcd.home();
    lcd.clear();
    String ss = (diff_status ? "ON" : "OFF");
    lcd.print("Diff " + ss + " " + String(tdiff,1) + "C");
    lcd.setCursor(0, 1);
    lcd.print("CW IN "+ String(tdiff_in,1) + " OUT " + String(tdiff_out,1));
    lcd.setCursor(0, 2);
    lcd.print("Gun IN "+String(tgun_in,1)+" OUT " +String(tgun_out,1));
    lcd.setCursor(0, 3);
    String ws = (WiFi.status() == WL_CONNECTED ? "UP" : "DOWN");
    lcd.print("Room "+String(troom,1) + " WiFi " + ws);
}

void publish_all()
{

  //Connected to wifi here
  MQTT_connect();
  
  unsigned long tnow = millis();
  if(tnow-p_last > publish_interval || tnow-p_last < 0)
  {
    if (! dpt.publish(tdiff)) {
      Serial.println(F("publish failed"));
    } else {
      Serial.println(F("publish OK!"));
    }
  
    if(! mqtt.ping()) {
      mqtt.disconnect();
    }
  }
}

void loop() {
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



  //Readout temperatures
  Serial.print("Diff pump C = "); 
  tdiff = diff_thermo.readCelsius();
  Serial.println(tdiff);
  diff_status = (tdiff > tdiff_max ? 0 : 1);

  Serial.print("Diff pump water in C = "); 
  tdiff_in = diff_thermo_in.readCelsius();
  Serial.println(tdiff_in);
  
  Serial.print("Diff pump water out C = "); 
  tdiff_out = diff_thermo_out.readCelsius();
  Serial.println(tdiff_out);

  Serial.print("Gun water in C = "); 
  tgun_in = gun_thermo_in.readCelsius();
  Serial.println(tgun_in);

  publish_all();

  update_lcd();
  
  delay(5000);
  
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
