/********************************
Co2 Laser safety interlock and monitor using digital DS18B20 temperature sensor

Code modified by wolfpaw98 based on codes by Anthony Bolgar and SigmazGFX

Pin Assignments for Arduino Nano board:
D2 - Coolant flow monitor (orange wire to D2, red wire to +5V, black wire to ground)
D3 - Door Switch (Hook up switch between D3 and ground)
     (Additional microswitches may be added in series to protect the laser tube door and controls door)
D4 - Warning buzzer or status LED (Buzzer+ to D4, buzzer- to ground)
D5 - Relay trigger to diasable laser fire (+5V to relay VCC, D5 connects to Relay IN,
     one side of laser enable to relay common, other side of laser enable to relay NO)
D6 - Coolant temperature monitor (Sensor - DS18B20) (Pin1 to ground, Pin2 to D6, Pin3 to +5V, 4.7k ohm resister
     between pins 2 and 3)
A4 - I2C LCD pin SDA. LCD is a common 20x4 lcd panel driven by an I2C board.
A5 - I2C LCD pin SCL. LCD is a common 20x4 lcd panel driven by an I2C board.

************************************/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define I2C_ADDR 0x27 // Define I2C Address for LCD backpack
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin); // Configure LCD display

unsigned long duration; // used for flow meter timer to watch if the flow stops

#define FLOWSENSORPIN 2 // Connect flow sensor data line  to this digital Pin
#define DOORSWITCH 3    // Connect safety interlock switch to this digital pin other side to gnd (high = open; low = closed)
#define ALARMPIN 4      // Connect + side of Alarm buzzer to this digital pin ( - side to gnd )
#define INTERLOCK 5     // Connect relay data line to this digital pin that enables or disables laser (safety system)
#define ONE_WIRE_BUS 6  // Connect temperature sensor date line to this digital pin and a 4.7k ohm resistor between this pin and +

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Setup alarm flags

int doorstate = 1;                        // default to pin HIGH (door open)
int displayalarm = 1;                     // Toggle when alarm messages are on the LCD

volatile uint16_t pulses = 0;             // Flow sensor pulse counter
volatile uint8_t lastflowpinstate;        // track the state of the pulse pin
volatile uint32_t lastflowratetimer = 0;  // you can try to keep time of how long it is between pulses
volatile float flowrate;                  // and use that to calculate a flow rate


// Interrupt is called once a millisecond, looks for any pulses from the sensor!

SIGNAL(TIMER0_COMPA_vect)
{
    uint8_t x = digitalRead(FLOWSENSORPIN);
    if (x == lastflowpinstate)
    {
        lastflowratetimer++;  // nothing changed!
        return;
    }
    if (x == HIGH)
    {        
        pulses++;             // low to high transition!
    }
    lastflowpinstate = x;
    flowrate = 1000.0;
    flowrate /= lastflowratetimer; // in hertz
    lastflowratetimer = 0;
}
void useInterrupt(boolean v)
{
    if (v)
    {
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
    } else {
       TIMSK0 &= ~_BV(OCIE0A);
    }
}
void setup()
{
    lcd.begin (20, 4);
    // Switch on the backlight
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
    // Start up the library
    sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
    // Setup the pins
    pinMode(INTERLOCK, OUTPUT);
    digitalWrite(INTERLOCK, LOW);
    pinMode(FLOWSENSORPIN, INPUT);
    digitalWrite(FLOWSENSORPIN, HIGH);
    pinMode(ALARMPIN, OUTPUT);
    digitalWrite(ALARMPIN,LOW);
    pinMode(DOORSWITCH, INPUT);
    digitalWrite(DOORSWITCH, HIGH);
    lastflowpinstate = digitalRead(FLOWSENSORPIN);
    useInterrupt(true);
    lcd.clear();
}
void loop()
{
    Interlock();     // run interlock subroutine
}
void Interlock()
{
    DoorSafety();    // loop that keeps an eye on the door switch state

    //Watchdog for the water flow
    duration = pulseIn(FLOWSENSORPIN, LOW,3000000);
    if (duration <= 0)
    {
        lcd.clear();
        lcd.setCursor(2,1);
        lcd.print("NO COOLANT FLOW!");
        lcd.setCursor(5,2);
        lcd.print("Check Pump");
        displayalarm=1;
        digitalWrite(ALARMPIN,HIGH); //pin for buzzer or light. Your choice.
        delay(1000);
    }
    
    FlowSensor();    //loop that runs the coolant flow animation
    TempSensor();    //loop that measures the temp output.

    //this section shuts down the laser enable ... Switch Interlock Pin states to meet your needs. (i.e. High for relay activation, Low to gnd pin directly from arduino)

    if (displayalarm==1)  //1=ALARM! 0=all good
    {
        digitalWrite(INTERLOCK, LOW);
    } else {
        digitalWrite(INTERLOCK,HIGH);
    }
}
void DoorSafety() //watch the door switch
{
    doorstate = digitalRead(DOORSWITCH);
    if (doorstate==1)
    {
        lcd.clear();
        lcd.setCursor(5,1);
        lcd.print("Door Open");
        displayalarm=1;
        delay(500);
        lcd.clear();
    }
    if (doorstate==0)
    {
        displayalarm=0;
    }
}
void FlowSensor() //coolant flow animation
{
    if (displayalarm == 0)
    {
        lcd.setCursor(4, 0);
        lcd.print("Coolant Flow");
        lcd.setCursor(0,2);
        lcd.print(" ");
        lcd.setCursor(2,1);
        lcd.print("[");
        lcd.setCursor(17,1);
        lcd.print("]");
        if (pulses >48)
        {
            pulses=0;
            lcd.setCursor(3,1);
            lcd.print("  ");
        } else {
            lcd.setCursor((pulses)/4 +3 ,1);
            lcd.print("->");
        }
    }
}
void TempSensor()  //Temperature sensor loop
{
    // call sensors.requestTemperatures() to issue a global temperature request to all devices on the bus
    sensors.requestTemperatures();            // Send the command to get temperatures
    Serial.print(sensors.getTempCByIndex(0)); // "byIndex" called because multiple ICs can be on the same bus; 0 refers to the first
    lcd.setCursor(2, 2);
    if (sensors.getTempCByIndex(0) > 21)      // Set to the upper temperature limit.  For the K40 it is suggested to never exceed 21C
    {
        lcd.setCursor(3,0);
        lcd.print("TEMP WARNING!!");
        lcd.setCursor(2,1);
        lcd.print(sensors.getTempCByIndex(0));
        lcd.print(" degrees C");
        lcd.setCursor(2,2);
        lcd.print("IMMEDIATE ACTION");
        lcd.setCursor(6,3);
        lcd.print("REQUIRED");
        digitalWrite(ALARMPIN,HIGH);
        digitalWrite(INTERLOCK, LOW);
        displayalarm = 1;
        delay(700);
        lcd.clear();
    } else {
        if (displayalarm == 0)
        {
            lcd.setCursor(2,3);
            lcd.print(sensors.getTempCByIndex(0));
            lcd.print(" degrees C");
            digitalWrite(ALARMPIN,LOW);
        }
        delay(50);
    }
}
