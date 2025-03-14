#include <Arduino.h>

// We communicate with the Arduino at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 1

// Encoder Pins
const byte encoder0pinA = 2;
const byte encoder1pinA = 3;
const byte encoder0pinB = 4;
const byte encoder1pinB = 5;

// Stepper Pins
int M1dirpin = 4;  // Motor 1 direction pin
int M1stepPin = 2; // Motor 1 pulse pin
int M1enpin = 7;   // Motor 1 enable pin

// Defining other variables
long USduration;
int distance;
byte encoder0PinALast;
byte encoder1PinALast;
long lPulses;
long rPulses;
int duration;
boolean LDirection; // the rotation direction
boolean RDirection;

void stepperInit()
{
    pinMode(M1dirpin, OUTPUT);
    pinMode(M1stepPin, OUTPUT);
    pinMode(M1enpin, OUTPUT);
}

void setup()
{
    encoderInit(); // Initialise the encoder pins
    stepperInit(); // Initialise the stepper motor pins
    Serial.begin(SERIAL_BAUD);
}

void encoderInit()
{
    pinMode(encoder0pinB, INPUT);
    pinMode(encoder1pinB, INPUT);
    attachInterrupt(digitalPinToInterrupt(2), LSpeed, CHANGE); // The interrupt triggers once per pulse
    attachInterrupt(digitalPinToInterrupt(3), RSpeed, CHANGE);
}

void LSpeed() // Increases or decreases
{
    int Lstate = digitalRead(encoder0pinA);
    if ((encoder0PinALast == LOW) && Lstate == HIGH)
    {
        int val = digitalRead(encoder0pinB);
        if (val == LOW && LDirection)
        {
            LDirection = false; // Reverse
        }
        else if (val == HIGH && !LDirection)
        {
            LDirection = true; // Forward
        }
    }
    encoder0PinALast = Lstate;

    if (!LDirection)
        lPulses++; // Increases or decreases pulses depending on whether the wheel moves forwards or backwards
    else
        lPulses--;
}

void RSpeed()
{
    int Rstate = digitalRead(encoder1pinA);
    if ((encoder1PinALast == LOW) && Rstate == HIGH)
    {
        int val = digitalRead(encoder1pinB);
        if (val == LOW && RDirection)
        {
            RDirection = false; // Reverse
        }
        else if (val == HIGH && !RDirection)
        {
            RDirection = true; // Forward
        }
    }
    encoder1PinALast = Rstate;

    if (!RDirection)
        rPulses++;
    else
        rPulses--;
}

void GetL()
{
    Serial.print(lPulses); // Returns the number of left pulses to the python
}

void GetR()
{
    Serial.print(rPulses); // Returns the number of right pulses to the python
}

void FullRot() // Rotates the stepper motors 360 degrees
{
    Serial.print("Rotation time");

    digitalWrite(M1enpin, LOW);

    digitalWrite(M1dirpin, HIGH);
    for (int j = 0; j <= 800; j++)
    { // number of pulses sent - 200 is one full rotation (with no microstepping)
        digitalWrite(M1stepPin, HIGH);
        delayMicroseconds(500); // lower = faster
        digitalWrite(M1stepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(M1dirpin, LOW); // Reverse Direction
    for (int j = 0; j <= 800; j++)
    { // number of pulses sent - 200 is one full rotation (with no microstepping)
        digitalWrite(M1stepPin, HIGH);
        delayMicroseconds(500); // lower = faster
        digitalWrite(M1stepPin, LOW);
        delayMicroseconds(500);
    }

    digitalWrite(M1enpin, HIGH);
}

// Bunch of built in functions
int read_pin()
{
    while (!Serial.available())
        ;
    int pin = Serial.read();
    return (int)(pin - 'a');
}

void command_read()
{
    int pin = read_pin();
    // Read from the expected pin.
    int level = digitalRead(pin);
    // Send back the result indicator.
    if (level == HIGH)
    {
        Serial.write('h');
    }
    else
    {
        Serial.write('l');
    }
}

void command_analog_read()
{
    int pin = read_pin();
    int value = analogRead(pin);
    Serial.print(value);
}

void command_write(int level)
{
    int pin = read_pin();
    digitalWrite(pin, level);
}

void command_mode(int mode)
{
    int pin = read_pin();
    pinMode(pin, mode);
}

void loop()
{

    duration = 0;
    delay(100);
    // Fetch all commands that are in the buffer
    while (Serial.available())
    {
        int selected_command = Serial.read();
        // Do something different based on what we got:
        switch (selected_command)
        {
        case 'a':
            command_analog_read();
            break;
        case 'r':
            command_read();
            break;
        case 'l':
            command_write(LOW);
            break;
        case 'h':
            command_write(HIGH);
            break;
        case 'i':
            command_mode(INPUT);
            break;
        case 'o':
            command_mode(OUTPUT);
            break;
        case 'p':
            command_mode(INPUT_PULLUP);
            break;
        case 'v':
            Serial.print("SRcustom:");
            Serial.print(FW_VER);
            break;
        case '<':
            GetL();
            break;
        case '>':
            GetR();
            break;
        case 'f':
            FullRot();
            break;
        case 't':
            Serial.print("Hello, the arduino is on");
            break;
        default:
            // A problem here: we do not know how to handle the command!
            // Just ignore this for now.
            break;
        }
        Serial.print("\n");
    }
}
