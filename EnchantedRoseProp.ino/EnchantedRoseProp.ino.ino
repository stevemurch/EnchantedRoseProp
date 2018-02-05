#include <SPI.h>
#include <EEPROM.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include "Boards.h"
#include <Adafruit_TiCoServo.h>

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
   #include <avr/power.h>
#endif

#define NEOPIXEL_PIN  12

 Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// BOARD SETUP

// attach control pins for servos as follows:
// SERVO PINS:  2,3,4,5 -- THIS IS HARDCODED IN THE PROGRAM
// servos WILL NOT WORK on 0, 1 pins

#define PROTOCOL_MAJOR_VERSION   0 //
#define PROTOCOL_MINOR_VERSION   0 //
#define PROTOCOL_BUGFIX_VERSION  2 // bugfix

//Servo   servo[4];               // the servo objects

Adafruit_TiCoServo myservo;
int pos = 0;

byte    pin_servo[4];           // holds the set rotation value of the servo

static byte buf_len = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial.println("Enchanted Rose Prop");

// neopixel
  strip.begin();
  strip.setBrightness(10);
  strip.show(); // Initialize all pixels to 'off'

  doNeoPixelColor();


  /* Default all to digital input */


  /*
  for (int pin = 0; pin < TOTAL_PINS; pin++)
  {
    // Set pin to input with internal pull up
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);

    // Save pin mode and state
    //pin_mode[pin] = INPUT;
    //pin_state[pin] = LOW;
  }
  */


  
  
  // test servo
 

//moveServo(2,0);
//moveServo(3,0);
//moveServo(5,0);
//moveServo(6,0);


/*
moveServoTest(4); // 2 and 3 work, 4 and 5 work
moveServoTest(5);
*/

  /*
   for(int pin = 0; pin<1; pin++)
   {
      if (servo[pin].attached())
      {
        servo[pin].detach();
      }
      servo[pin].attach(pin);
   }
   */

 


/*
  int pos = 0;
  
   for (pos = -60; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    Serial.println(pos);
    servo[0].write(pos);              // tell servo to go to position in variable 'pos'
    delay(1000);                       // waits 15ms for the servo to reach the position
  }
  delay(3000);
  for (pos = 180; pos >= -60; pos -= 1) { // goes from 180 degrees to 0 degrees
    Serial.println(pos);
    servo[0].write(pos);              // tell servo to go to position in variable 'pos'
    delay(1000);                       // waits 15ms for the servo to reach the position
  }
  */

  
  


  
  Serial.println("Starting Bluetooth");
  ble_begin();

  byte buf[] = {'O','n','l','i','n','e'};         
  ble_write_string(buf,6 );
  

  
}

void moveServoTest(int servoPin)
{
  pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);

  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  myservo.detach();
  
}


void doNeoPixelOff()
{
  

  colorWipe(strip.Color(0, 0, 0), 15); // Off
}

void doNeoPixelColor()
{
  colorWipe(strip.Color(0, 255, 0), 5); // Green
  colorWipe(strip.Color(255, 0, 0), 5); // Red
  colorWipe(strip.Color(0, 0, 255), 5); // Blue

  doNeoPixelOff();
}

// Fill the dots one after the other with a color

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}


void resetProp() {
  // turn off light
  Serial.println("Resetting prop");


  
  // reset all servos
}

void moveServo(int servoPin, int drop1reset0)
{

  Serial.println("Move servo [servoPin, state]");
  Serial.println(servoPin);
  Serial.println(drop1reset0);

  if (myservo.attached())
  {
    myservo.detach();
  }

  if (drop1reset0==0)
  {
  pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);

  int startingPos = myservo.read();
  // get to 0

  Serial.println("RESETTING FROM STARTING SERVO STATE, WHICH IS ");
  Serial.println(startingPos);




  for (pos = startingPos; pos >=0; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  myservo.detach();
  } 

  else 
  {
    pinMode(servoPin, OUTPUT);
    myservo.attach(servoPin);
    int startingPos = myservo.read();

  Serial.println("MOVING TO 180 FROM STARTING SERVO STATE, WHICH IS ");
  Serial.println(startingPos);
    
    for (pos = startingPos; pos <=180; pos += 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    myservo.detach();
  }
  
  
}





void moveServoToPosition(int servoPin, int toPosition)
{

  Serial.println("Move servo to [servoPin, toPosition]");
  Serial.println(servoPin);
  Serial.println(toPosition);

  if (myservo.attached())
  {
    myservo.detach();
  }

  pinMode(servoPin, OUTPUT);
  myservo.attach(servoPin);

  int startingPos = myservo.read();
  // get to 0

  Serial.println("RESETTING FROM STARTING SERVO STATE, WHICH IS ");
  Serial.println(startingPos);

  if (toPosition<= startingPos) 
  {
  for (pos = startingPos; pos >=toPosition; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  myservo.detach();
  } else 
  {

for (pos = startingPos; pos < toPosition; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
  }
  myservo.detach();
   
  }
}




// BLUETOOTH FUNCTIONS ---------------------------

void ble_write_string(byte *bytes, uint8_t len)
{
  if (buf_len + len > 20)
  {
    for (int j = 0; j < 15000; j++)
      ble_do_events();
    
    buf_len = 0;
  }
  
  for (int j = 0; j < len; j++)
  {
    ble_write(bytes[j]);
    buf_len++;
  }
    
  if (buf_len == 20)
  {
    for (int j = 0; j < 15000; j++)
      ble_do_events();
    
    buf_len = 0;
  }  
}

void reportPinServoData(byte pin)
{
 
  byte value = pin_servo[pin];
  byte buf[] = {'G', pin, value};         
  ble_write_string(buf, 3);
}

void reportPinDigitalData(byte pin)
{
  byte state = digitalRead(pin);
  byte buf[] = {'G', pin, state};         
  ble_write_string(buf, 3);
}

void sendCustomData(uint8_t *buf, uint8_t len)
{
  uint8_t data[20] = "Z";
  memcpy(&data[1], buf, len);
  ble_write_string(data, len+1);
}




// LOOP
// Listen to Bluetooth.
// When receiving a command, process it.

byte queryDone = false;

void loop() {



 
while(ble_available())
  {
    byte cmd;
    cmd = ble_read();
    Serial.println("[COMMAND RECEIVED]");
    Serial.println(cmd);
    Serial.println("[END OF COMMAND]");

// Parse data here
    switch (cmd)
    {

      case 'H': // SET light to 0 or 1 or 2 
      {
         byte style = ble_read();
         Serial.print("Set light to value: ");
         Serial.println(style);
         break;
      }

      case 'L': // SET Servo Position
        {
          byte pin = ble_read();
          byte state = ble_read();
          
          moveServoToPosition(pin, state);
        }
        break;


      case 'G': // GET pin data -- "G3" will get pin 3's state
        {
          byte pin = ble_read();
          reportPinDigitalData(pin);
        }
        break;

      case 'K':
      {
        Serial.println("RESET PROP");
        moveServo(2,0);
        moveServo(3,0);
        moveServo(5,0);
        moveServo(6,0);
        
      }
        
      case 'T': // set pin digital state -- T31 will set pin 3 to HIGH
                // only pins that are valid are 2, 3, 4, 5 <<< SERVO
        {
          byte pin = ble_read();
          byte state = ble_read();

          Serial.println("pin is ");
          Serial.println(pin);

          

          if ((pin!=2) && (pin!=3) && (pin!=5) && (pin != 6) && (pin!=7))
          {
            Serial.println("Invalid pin value.");
            break;
          }

          if ((pin==7) && (state==1))
          {
             
             
          } 
          else 
          {
              moveServo(pin, state);
          }
          
          //digitalWrite(pin, state);
          //reportPinDigitalData(pin);
                   
        }
        break;
         
      case 'Z': // free form text
        {
          byte len = ble_read();
          byte buf[len];
          for (int i=0;i<len;i++)
            buf[i] = ble_read();
          Serial.println("->");
          Serial.print("Received: ");
          Serial.print(len);
          Serial.println(" byte(s)");
          Serial.print(" Hex: ");
          for (int i=0;i<len;i++)
            Serial.print(buf[i], HEX);
          Serial.println();
        }
       
    }

    // send out any outstanding data
    ble_do_events();
    buf_len = 0;
    
    return; // only do this task in this loop



    
  }

  // process text data
  if (Serial.available())
  {
    byte d = 'Z';
    ble_write(d);

    delay(5);
    while(Serial.available())
    {
      d = Serial.read();
      ble_write(d);
    }
    
    ble_do_events();
    buf_len = 0;
    
    return;    
  }

  // No input data, no commands, process analog data
  if (!ble_connected())
    queryDone = false; // reset query state
    
  if (queryDone) // only report data after the query state
  { 
    //byte input_data_pending = reportDigitalInput();  
    /*
    if (input_data_pending)
    {
      ble_do_events();
      buf_len = 0;
      
      return; // only do this task in this loop
    }
    */
  
    //reportPinAnalogData();
    
    ble_do_events();
    buf_len = 0;
    
    return;  
  }
    
  ble_do_events();
  buf_len = 0;


}
