/*
MINIMAL EXAMPLE FOR INTERFACING THE LD2450 RADAR SENSOR.

CONNECTIONS:

LD2450 => ESP32
5V => VBUS
GND => GND
RX => GPIO0 / GP3
TX => GPIO1 / GP1
*/

#include <Arduino.h>
#include <LD2450.h>
#include <count.h>

#define ledPin 2


// FOR RPI PICO USAGE AND ITS SERIAL PORT PLEASE REFER TO CORE MANUAL AND/OR
// https://forum.arduino.cc/t/pi-pico-uart-use-in-arduino-ide/881957/6
#ifdef ARDUINO_ARCH_MBED_RP2040
UART Serial();
#endif


// SENSOR INSTANCE
LD2450 ld2450;

// Initialize previous targets
target prev_target[3];


void setup()
{
  //SERIAL FOR HOST / DEBUG MESSAGES
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  
  Serial.println("SETUP_STARTED");
  
  // BUILD-IN LED TO INDICATED RADAR TARGETS AS AN EXAMPLE
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // SETUP SENSOR
  // HERE THE HARDWARE SERIAL INTERFACE 1 IS USED
  ld2450.begin(Serial, false);


  
  if(!ld2450.waitForSensorMessage()){
    Serial.println("SENSOR CONNECTION SEEMS OK");
  }else{
    Serial.println("SENSOR TEST: GOT NO VALID SENSORDATA - PLEASE CHECK CONNECTION!");
  }

  prev_target[0].x = 0;
  prev_target[0].y = 0;
  prev_target[1].x = 0;
  prev_target[1].y = 0;
  prev_target[2].x = 0;
  prev_target[2].y = 0;
  

  Serial.println("SETUP_FINISHED");
  
}

void loop()
{
  static uint8_t counter = 0;
  bool count_changed = 0;


  // READ FUNCTION MUST BE CALLED IN LOOP TO READ THE INCOMMING DATA STREAM
  // RETURNS -1 or -2 as error flag and 0 to getSensorSupportedTargetCount() if valid targets found
  const int sensor_got_valid_targets = ld2450.read();
  if (sensor_got_valid_targets > 0)
  {

    /*
    PRINT DEBUG DATA STREAM LIKE THIS: 
    TARGET ID=1 X=-19mm, Y=496mm, SPEED=0cm/s, RESOLUTION=360mm, DISTANCE=496mm, VALID=1
    TARGET ID=2 X=-1078mm, Y=1370mm, SPEED=0cm/s, RESOLUTION=360mm, DISTANCE=1743mm, VALID=1
    TARGET ID=3 X=0mm, Y=0mm, SPEED=0cm/s, RESOLUTION=0mm, DISTANCE=0mm, VALID=0
    */
    //Serial.print(ld2450.getLastTargetMessage());


    // GET THE DETECTED TARGETS
    // TARGET RANGE CAN BE FROM 0 TO ld2450.getSensorSupportedTargetCount(), DEPENDS ON SENSOR HARDWARE. REFER TO LD2450 DATASHEET
    for (int i = 0; i < ld2450.getSensorSupportedTargetCount(); i++)
    {
      const LD2450::RadarTarget result_target = ld2450.getTarget(i);
      if (result_target.valid) {

        if (prev_target[i].x < 0 && result_target.x >= 0) {
          counter++;
          count_changed = 1;
        } else if (prev_target[i].x > 0 && result_target.x <= 0 && counter > 0) {
          counter--;
          count_changed = 1;
        }

        /*Serial.print("Previous Target: ");
        Serial.println(i);
        Serial.print("X = ");
        Serial.print(prev_target[i].x);
        Serial.print(", Y = ");
        Serial.println(prev_target[i].y);*/

        /*Serial.print("Target: ");
        Serial.println(i);
        //Serial.print("X = ");
        //Serial.print(result_target.x);
        Serial.print("Y = ");
        Serial.print(result_target.y);
        //Serial.print(", Speed = ");
        //Serial.print(result_target.speed);
        //Serial.println();
        Serial.println();*/

        prev_target[i].x = result_target.x;
        prev_target[i].y = result_target.y;
      }

      
    }
  };
  if (count_changed == 1) {
    Serial.print("Count = ");
    Serial.println(counter);
    //Serial.print("Previous Target: ");
    //Serial.print("X = ");
    //Serial.print(prev_target[0].x);
    Serial.print("Current Target: ");
    Serial.print("X = ");
    Serial.println(ld2450.getTarget(0).x);


  }
 
}
