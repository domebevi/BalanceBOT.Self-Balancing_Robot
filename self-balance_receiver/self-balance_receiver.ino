#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3);
int c;
void setup() 
{
    Serial.begin(9600);
    Serial.println("Arduino with HC-05 is ready");
 
    BTserial.begin(9600);  
    Serial.println("BTserial started at 9600");
}
 
void loop()
{
 
     // Keep reading from HC-05 and send to Arduino Serial Monitor
    if (BTserial.available())
    {  
        Serial.write(BTserial.read());
    }
 
    // Keep reading from Arduino Serial Monitor and send to HC-05
    if (Serial.available())
    {
        c=Serial.read();
        BTserial.write(c);  
    }
 
}
