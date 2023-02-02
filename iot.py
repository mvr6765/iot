Aim: Write a program on Arduino/Raspberry Pi to publish temperature data to MQTT
broker
Procedure:
Step 1: Download &amp; install Raspeberri Pi from https://www.raspberrypi.com/software/
Step 2: Open the raspberry pi terminal and write program
Program:
import sys
importAdafruit_DHT
import time
frompaho.mqtt.client import *

client=Client(&quot;Device1&quot;)
c=client.connect(&#39;broker.hivemq.com&#39;,1883)
if c==0:
print(&#39;client connected&#39;)
while True:
hum,temp=Adafruit_DHT.read_retry(11,4)
print(&#39;Temperature:&#39;,temp)
print(&#39;Humidity:&#39;, Hum)
message=&#39;temp=&#39;+str(temp)+&quot;,hum=&quot;+str(hum)
client.publish(&#39;iotdata&#39;,message)
time.sleep(2)
Step 3: Now save it

Output:
pi@raspberrypi:~ $ cd iot
pi@raspberrypi:~/iot $ python3 dhtpublish.py
client connected
Temperature: 28.0
Humidity: 95.0
Temperature: 29.0
Humidity: 94.0
Temperature: 28.0
Humidity: 94.0
Temperature: 29.0
Humidity: 95.0
Temperature: 28.0
Humidity: 94.0
Temperature: 29.0
Humidity: 94.0
Temperature: 28.0
Humidity: 93.0
To see the result in Smartphone
Step 1: Download MQTT Dash app from Play Store.
Step 2: Open MQTT Dash app in mobile and click on “+” icon.
Step 3: Enter Name like “mysensor1” and enter “broker.hivemq.com” in Address field.
Step 4: Save it.
Step 5: Now click on “mysensor1”

To see the result in Website
Open“ hivemq.com/demos/websocket-client/ “ in google chrome.
=============================================================================================
5
Requirements:

1. DHT22 sensor
2. Arduino Uno microcontroller board (ATmega328p)
3. Jumper wires
4. SSD1306 OLED display

Procedure:
Step 1: Open www.wokwi.com in browser and select “Arduino Uno” microcontroller.
Step 2: In the Simulation part, select the above list by clicking “+” symbol, which are specified
in above requirements.
Step 3:
By using jumper wires,
I. Give 5 volts power to pin dht1:VCC
II. Connect dht1: SDA to digital pin3 of Arduino Uno microcontroller board
(ATmega328p)
III. Connect dht1: GND to ground.
IV. Make connections between
● Oled1:DATA toAnalog pin A4
● Oled1:CLK toAnalog pin A5
● Oled1:VIN to 5 volt power
● Oled1:GND to ground
Step 4: Write program in “sketch.ino”.
Note:Download libraries Adafruit SSD1306 and DHT sensor library.

Program:
#include&lt;Adafruit_Sensor.h&gt;
#include&lt;Adafruit_GFX.h&gt;
#include&lt;Adafruit_SSD1306.h&gt;
#include&lt;DHT.h&gt;
#include&lt;DHT_U.h&gt;
#include&lt;Wire.h&gt;
Adafruit_SSD1306 display(128,64,&amp;Wire);
DHT Sensor(3,DHT22);
voidsetup() {
  // put your setup code here, to run once:
Sensor.begin();
Serial.begin(9600);
display.begin(SSD1306_SWITCHCAPVCC,0X3C);
display.setTextColor(WHITE);
}
voidloop() {
  // put your main code here, to run repeatedly:
floattempr= Sensor.readTemperature();
float humid= Sensor.readHumidity();
display.setCursor(0,10);
display.setTextSize(1);
display.print(&quot;Temperature= &quot;);
display.println(tempr);
display.print(&quot;Humidity= &quot;);
display.println(humid);
display.display();
delay(500);
display.clearDisplay();
}


next 

#include&lt;Adafruit_Sensor.h&gt;
#include&lt;DHT.h&gt;
#include&lt;DHT_U.h&gt;
DHT Sensor(3,DHT22);
voidsetup() {
  // put your setup code here, to run once:
Sensor.begin();
Serial.begin(9600);
}
voidloop() {
  // put your main code here, to run repeatedly:
floattempr = Sensor.readTemperature();
Serial.print(&quot;Temperature = &quot;);
Serial.print(tempr);
Serial.print(&quot;, &quot;);
delay(1000);
float humid = Sensor.readHumidity();
Serial.print(&quot;Humidity = &quot;);
Serial.println(humid);
}

===============================================================================

3
int flag=0;
voidsetup() {
  // put your setup code here, to run once:
pinMode(3, OUTPUT);
pinMode(5, INPUT);
}
voidloop() {
  // put your main code here, to run repeatedly:
int value=digitalRead(5);
if(value==1&amp;&amp; flag==0)
{
  digitalWrite(3,HIGH);
  delay(1000);
  flag=1;
}
elseif(value==1&amp;&amp; flag==1)
{  
  digitalWrite(3,LOW);
  delay(1000);
  flag=0;
}
}