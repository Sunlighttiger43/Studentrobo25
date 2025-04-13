#include <Arduino.h>

// We communicate with the Arduino at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 1

// Defining pins
const byte encoder0pinA = 2;
const byte encoder1pinA = 3;
const byte encoder0pinB = 6;
const byte encoder1pinB = 7;
const byte laserPin = 8;
const int UtrigPin = 12;
const int UechoPin = 11;
const int DtrigPin = 10;
const int DechoPin = 9;
// Defining other variables
int laserState = 0; // Whether the laser is tripped or not
long USduration;
int distance;
byte encoder0PinALast;
byte encoder1PinALast;
long lPulses;
long rPulses;
int duration;
boolean LDirection; // the rotation direction
boolean RDirection;

/*
This sample code is for testing the 2 stepper motors
The rotation velocity can be adjusted by the code switch
Microcontroller: Arduino UNO
*/
int M1dirpin = 4; // Motor 1 direction pin
int M1pulse = 5;  // Motor 1 pulse pin

void FullRot() // Rotates the stepper motors 360 degrees
{
    Serial.print("Rotation time");
    int j;
    delayMicroseconds(20);
    digitalWrite(M1dirpin, LOW);
    for (j = 0; j <= 2000; j++)
    { // number of pulses sent - 200 is one full rotation
        digitalWrite(M1pulse, LOW);
        delayMicroseconds(200); // lower = faster
        digitalWrite(M1pulse, HIGH);
        delayMicroseconds(200); // millis
    }
}

void CustomRot() // Rotates the stepper motors by a custom amount
{
    while (Serial.available() == 0)
    {
    }
    int steps = Serial.read();
    Serial.print(steps);
    int j;
    delayMicroseconds(2);
    digitalWrite(M1dirpin, LOW);
    for (j = 0; j < steps; j++)
    { // number of pulses sent - 200 is one full rotation
        digitalWrite(M1pulse, LOW);
        delayMicroseconds(500); // lower = faster
        digitalWrite(M1pulse, HIGH);
        delay(1); // millis
    }
}
void ReverseCustomRot() // Rotates the stepper motors by a custom amount
{
    while (Serial.available() == 0)
    {
    }
    int steps = Serial.read();
    Serial.print(steps);
    int j;
    delayMicroseconds(2);
    digitalWrite(M1dirpin, HIGH);
    for (j = 0; j < steps; j++)
    { // number of pulses sent - 200 is one full rotation
        digitalWrite(M1pulse, LOW);
        delayMicroseconds(500); // lower = faster
        digitalWrite(M1pulse, HIGH);
        delay(1); // millis
    }
}

void setup()
{
    EncoderInit(); // Initialise the stepper motor pins
    Serial.begin(SERIAL_BAUD);
    pinMode(UtrigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(UechoPin, INPUT);  // Sets the echoPin as an Input
    pinMode(DtrigPin, OUTPUT);
    pinMode(DechoPin, INPUT);
    pinMode(laserPin, INPUT_PULLUP); // Sets the laser pin to input with a pullup resistor
    pinMode(M1dirpin, OUTPUT);
    pinMode(M1pulse, OUTPUT);
}

void EncoderInit()
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

void GetLaserState()
{
    Serial.print(laserState); // Gets the state of the laser, whether it is tripped or not
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

void GetDistance(int trigPin, int echoPin) // Gets the distance of the object in front of the specified ultrasound sensor
{
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    USduration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = USduration * 0.034 / 2;
    Serial.print(distance);
    // Prints the distance on the Serial Monitor
    // Serial.print("Distance: ");
    // Serial.println(distance);
}

void loop()
{

    duration = 0;
    delay(100);
    // Fetch all commands that are in the buffer
    laserState = digitalRead(laserPin); // Gets the state of the laser
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
        case 'u':
            GetDistance(UtrigPin, UechoPin);
            break;
        case 'd':
            GetDistance(DtrigPin, DechoPin);
            break;
        case 'b':
            GetLaserState();
            break;
        case 'f':
            FullRot();
            break;
        case 't':
            Serial.print("Test Sufficient, Arduino isn't bugging, your code is");
            break;
        default:
            // A problem here: we do not know how to handle the command!
            // Just ignore this for now.
            break;
        }
        Serial.print("\n");
    }
}
