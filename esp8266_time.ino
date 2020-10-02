/*  
  The sketch can read time from a NTP server.
  Don't forget to set up your wifi credentials and time zone where you live.
  Tested with:
    Arduino 1.8.12
  Required libraries: (see menu: Sketch -> Include Library -> Manage Libraries...)

  - NTPClient
    Version 3.2.0
    https://github.com/arduino-libraries/NTPClient
  - If you don't have experience with esp8266 ​read here: https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/
    to learn more about installation the board in Arduino IDE.

     Wiring:

        esp8266        VFD shield

          GND  <------->  GND

          VIN  <------->  5V
    
          RX  <-------->  TX

          TX  <-------->  RX

  This program sends Unix epoch value with commands thru the serial port (9600 baud):

  - Command 'T': Set the date and time by means of a Unix epoch value. The command
    must be followed by a decimal value. For example:

    T 1431472660

    Sets the date and time to 2015-05-12 23:17:40.
    
  Author:  Kiril Kirov, Instructables
  Date:    2020-04-18
  License: public domain
*/
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "bg.pool.ntp.org");
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
// NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

const char* ssid     = "Plamena 2.4GHz"; // set your WIFI name here
const char* password = "19691969"; // set your wifi password here

void setup() {
  Serial.begin(9600);
  Serial.println(" ");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  timeClient.begin();
// GMT +1 = 3600
// GMT +8 = 28800
// GMT -1 = -3600
// GMT 0 = 0
// GMT +3 = 10800
  timeClient.setTimeOffset(10800);}// set your time zone here according to the example shown a little bit higher
  
void loop() {
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  Serial.print("T ");
  Serial.println(timeClient.getEpochTime()); // printing thru serial monitor to VFD shield clock
  ESP.deepSleep(9e8); // the esp is put to deep sleep for 15 minutes
}
