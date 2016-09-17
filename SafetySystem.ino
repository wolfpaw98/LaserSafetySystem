/********************************
Co2 Laser safety interlock and monitor

Code by Anthony Bolgar based upon the code by SigmazGFX (+Jon Bruno)

LCD is a common 20x4 lcd panel driven by an I2C board.

Pin Assignments for Arduino Nano board:

D2 - Coolant flow monitor (flow meter output{orange wire} to D2, red wire to +5V, black wire to ground))
D3 - Door Switch (Hook up switch between ground and D3)
     (You can add additional microswitches in series to protect the laser tube door and controls door)
D4 - Warning buzzer or status LED (Buzzer+ to D4, buzzer- to ground)
D5 - Relay trigger to diasable laser fire (+5V to relay VCC, D5 connects to Relay IN,
     one side of laser enable to relay common, othe side of laser enable to relay NO)
A0 - Coolant temperature monitor (Sensor - TMP36) (Pin1 to A0, Pin2 to +5V, Pin3 to ground)



************************************/
#include <Wire.h>
#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#define I2C_ADDR 0x27 // Define I2C Address for LCD backpack
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);

unsigned long duration; //used for flow meter timer to watch if the flow stops

#define INTERLOCK 5 //pin that enables or disables laser (safety system)
#define ALARMPIN 4 //Connect + side of Alarm buzzer to this Digital pin ( - side to gnd )
#define FLOWSENSORPIN 2 //Connect flow sensor output to this Digital Pin
#define DOORSWITCH 3 //Connect safety interlock switch to this Digital pin other side to gnd(high=open - low=closed)

int TempSensorPin = A0;
//the analog pin the TMP36's Vout (sense) pin is connected to A0
//the resolution is 10 mV / degree centigrade with a
//500 mV offset to allow for negative temperatures

//Setup alarm flags

int doorstate = 1;

//default to pin HIGH (door open)

int displayalarm = 1; //Toggle when alarm messages are on the LCD

//Flow sensor 
// count how many pulses!

volatile uint16_t pulses = 0;

// track the state of the pulse pin

volatile uint8_t lastflowpinstate;

// you can try to keep time of how long it is between pulses

volatile uint32_t lastflowratetimer = 0;

// and use that to calculate a flow rate

volatile float flowrate;

// Interrupt is called once a millisecond, looks for any pulses from the sensor!

SIGNAL(TIMER0_COMPA_vect)
{
    uint8_t x = digitalRead(FLOWSENSORPIN);
    if (x == lastflowpinstate)
    {
        lastflowratetimer++;
        return;
        // nothing changed!
    }
    if (x == HIGH)
    {
        //low to high transition!
        pulses++;
    }
    lastflowpinstate = x;
    flowrate = 1000.0;
    flowrate /= lastflowratetimer;
    // in hertz
    lastflowratetimer = 0;
}
void useInterrupt(boolean v)
{
    if (v)
    {
        OCR0A = 0xAF;
        TIMSK0 |= _BV(OCIE0A);
    }
    else
    {
       TIMSK0 &= ~_BV(OCIE0A);
    }
}
void setup()
{
    lcd.begin (20, 4);
    // Switch on the backlight
    lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
    lcd.setBacklight(HIGH);
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
    Interlock();
}
void Interlock()
{
    DoorSafety();    //loop that keeps an eye on the door switch state

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
    TempSensor();    //loop that measures the TMP36 temp output.

    //this section shuts down the laser enable ... Switch Interlock Pin states to meet your needs. (i.e. High for relay activation, Low to gnd pin directly from arduino)

    if (displayalarm==1)  //1=ALARM! 0=all good
    {
        digitalWrite(INTERLOCK, LOW);
    }
    else
    {
        digitalWrite(INTERLOCK,HIGH);
    }
}
void DoorSafety()//watch the door switch
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
        }
        else
        {
            lcd.setCursor((pulses)/4 +3 ,1);
            lcd.print("->");
        }
    }
}
void TempSensor()  //Temp sensor loop
{
    int reading = analogRead(TempSensorPin);
    // converting that reading to voltage, for 3.3v arduino use 3.3
    float voltage = reading * 5.0;
    voltage /= 1024.0;
    float temperatureC = (voltage - 0.5) * 100 ;
    //converting from 10 mv per degree wit 500 mV offset
    //to degrees ((voltage - 500mV) times 100)
    lcd.setCursor(2, 2);
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
    if (temperatureF > 70) //Set this to the upper temperature limit . for the K40 it's been suggested to never exceed 70F or 21C
    {
        lcd.setCursor(0,0);
        lcd.print(" TEMP WARNING!!!!");
        lcd.setCursor(2,1);
        lcd.print(temperatureF);
        lcd.print(" degrees F");
        lcd.setCursor(2,2);
        lcd.print("IMMEDIATE ACTION");
        lcd.setCursor(6,3);
        lcd.print("REQUIRED");
        digitalWrite(ALARMPIN,HIGH);
        digitalWrite(INTERLOCK, LOW);
        displayalarm = 1;
        delay(700);
        lcd.clear();
    }
    else
    {
        if (displayalarm == 0)
        {
            lcd.setCursor(2,3);
            lcd.print(temperatureF);
            lcd.print(" degrees F");
            digitalWrite(ALARMPIN,LOW);
        }
        delay(50);
    }
}

