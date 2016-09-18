#include <Bounce2.h>
#include <Servo.h>

// Pin definitions
const int relayPin = 2;
const int servoPin = 3;
const int KL15Pin = 4;
const int TPin = 5;
const int potiPin = A0;
const int LedPin = 6;
const int LedPin2 = 7;
const int FanSpeedPin = 9;

// Objects
Servo servo;  // servo object for flap control
Bounce bouncerKL15 = Bounce(); //debouncing KL15 input
Bounce bouncerT = Bounce(); //debouncing T-Switch input

// Global variables
int relay_val = HIGH;
int servo_val = LOW;
int KL15_val = LOW;
int T_val = LOW;
int poti_val = 0;
int heating_mode; // -1 = heater off; 0 = driving; 1 = non-driving

void setup() {

  // set pins
  pinMode(relayPin, OUTPUT); // relay control
  pinMode(servoPin, OUTPUT); // servo control
  pinMode(KL15Pin, INPUT); // KL15
  pinMode(TPin, INPUT); // T-Switch
  pinMode(potiPin, INPUT); // poti
  pinMode(LedPin, OUTPUT); // Led
  pinMode(LedPin2, OUTPUT); // Led
  pinMode(FanSpeedPin, OUTPUT); // FanSpeed PWM Pin

  // attach and set debouncer
  bouncerKL15.attach( KL15Pin );
  bouncerT.attach( TPin );
  bouncerKL15.interval(5);
  bouncerT.interval(5);

  // attach Server object
  servo.attach(servoPin);
  servo.write(90); // default position

  // turn relay off, relay HIGH is OFF
  digitalWrite(relayPin, HIGH);
}

void loop() {

  // local variables
  bool changed = false;

  // read KL15
  bouncerKL15.update();
  KL15_val = bouncerKL15.read();

  // read T-Switch
  bouncerT.update();   
  T_val = bouncerT.read();

  // read poti
  poti_val = analogRead(potiPin);

  // set fan speed
  poti_val = map(poti_val, 0, 1023, 0, 255);     // scale it to use it with the servo (value between 0 and 180)
  analogWrite(FanSpeedPin, poti_val);
  delay(30);

  // state machine:
  // if KL15 is high, we can always assume to turn the heater on
  // if KL15 goes from high to low we need to turn off the heater
  // if T goes from low to high we need tu turn on the heater, assuming KL15 is low
  // if T IS high we can't assume the heater to be turned on, only on "rose"
  // if T goes from high to low we need to check for KL15, as KL15 overrides T behavior

  // KL15 LOW => HIGH
  if(bouncerKL15.rose())
  {
    digitalWrite(LedPin, HIGH);
    relay_val = LOW;
    heating_mode = 0;
    changed = true;
  }

  // KL15 HIGH => LOW
  else if(bouncerKL15.fell())
  {
    relay_val = HIGH;
    digitalWrite(LedPin, LOW);
    heating_mode = -1;
    changed = true;
  }

  // T-Switch LOW => HIGH
  if(bouncerT.rose() && KL15_val == LOW)
  {
    digitalWrite(LedPin2, HIGH);
    relay_val = LOW;
    heating_mode = 1;
    changed = true;
  }

  // T-Switch HIGH => LOW
  else if(bouncerT.fell() && KL15_val == LOW)
  {
    digitalWrite(LedPin2, LOW);
    relay_val = HIGH;
    heating_mode = -1;
    changed = true;
  }

  // if the state has changed we need to update the heater settings
  if(changed)
  {
    // turn heater ON/OFF
    digitalWrite(relayPin, relay_val);

    // set heating mode - flap control
    if(heating_mode == 0)
      servo.write(0);
    else if(heating_mode == 1)
      servo.write(170);
    else
      servo.write(90);

    // let's give that poor guy a break
    // after setting the servo it's advisable to wait
    delay(30);
  }
}

