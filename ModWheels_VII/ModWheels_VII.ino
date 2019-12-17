/* MW_V2.2 - ArxCruiser
 * -Kordell Tan 
 * 
 * Telemetry and Custom Command Arxterra 3DoT Code
 *    This file includes the definitions for the Fall 2019 ModWheels: ArxCruiser
 *    Features:
 *      + Turn Signaling
 *      + Horn Simulation
 *      + Differential Drive
 *      + D-Pad Control
 */
static uint8_t speedA = 0x00;
static uint8_t speedB = 0x00;
static uint8_t LM     = 0x00;
static uint8_t RM     = 0x00;
static uint8_t brakes = 0x00;

// include C:\Program Files (x86)\Arduino 1_6_5\hardware\arduino\avr\libraries
#include <ArxRobot.h>    // instantiated as ArxRobot at end of class header
#include <EEPROM.h>
#include <Wire.h>        // I2C support

ArxRobot ArxRobot;       // instantiated as ArxRobot at end of class header

// Steps for current limiter
uint8_t N = 40;

/*
 * Setting Up the I2C
 */
#define address      0x38
#define shaft        0x39
#define IN           0x00
#define OUT          0x01

void I2Cwrite(uint8_t config_, uint8_t data){
  Wire.beginTransmission(address);
  Wire.write(config_);
  Wire.write(data);
  Wire.endTransmission();
  delay(50);
}

static uint8_t encoder = 0x00;
// NOTE: No parameters, function will save shaft input to variable 'encoder'
void I2Cread() {
  Wire.beginTransmission(address);
  Wire.write(OUT);
  Wire.requestFrom(address,2);
  encoder = Wire.read();
  Wire.endTransmission();
  delay(50);
}

/*
 * NOTE: Custom command address space 0x40 - 0x5F.
 * We will keep SERVO defined here for manual servo adjustment as needed
 */
#define SERVO                0x41
#define STBY                    8
static uint8_t angle      =  100;
static uint8_t checkturn  = 0x00;
static uint8_t checklight = 0x00;

/*
 * Program to play a simple melody at start-up
 *
 * Tones are created by quickly pulsing a speaker on and off 
 *   using PWM, to create signature frequencies.
 *
 * Each note has a frequency, created by varying the period of 
 *  vibration, measured in microseconds. We'll use pulse-width
 *  modulation (PWM) to create that vibration.

 * We calculate the pulse-width to be half the period; we pulse 
 *  the speaker HIGH for 'pulse-width' microseconds, then LOW 
 *  for 'pulse-width' microseconds.
 *  This pulsing creates a vibration of the desired frequency.
 *
 * (cleft) 2005 D. Cuartielles for K3
 * Refactoring and comments 2006 clay.shirky@nyu.edu
 * 
 */
 
#define speakerOut      22
//  timePWMHigh = 1/(2 * frequency) = period / 2
//      note         period    frequency
#define  c            3830    // 261 Hz 
#define  d            3400    // 294 Hz 
#define  e            3038    // 329 Hz 
#define  f            2864    // 349 Hz 
#define  g            2550    // 392 Hz 
#define  a            2272    // 440 Hz 
#define  b            2028    // 493 Hz 
#define  C            1912    // 523 Hz
// Rest
#define  R               0

int melody[] = {  C,  b,  g,  C,  b,   e,  R,  C,  c,  g, a, C };
int beats[]  = { 16, 16, 16,  8,  8,  16, 32, 16, 16, 16, 8, 8 }; 
int MAX_COUNT = sizeof(melody) / 2; // Melody length, for looping.
  
// Set overall tempo
  long tempo = 10000;
  
// Set length of pause between notes
  int pause = 1000;
  
// Loop variable to increase Rest length
  int rest_count = 100;
  int tone_      =   0;
  int beat       =   0;
  long duration  =   0;

void song()
{
  long elapsed_time = 0;
  if (tone_ > 0) {                    // if this isn't a Rest beat, while the tone has 
                                      //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < duration) {

      digitalWrite(speakerOut,HIGH);
      delayMicroseconds(tone_ / 2);

      // DOWN
      digitalWrite(speakerOut, LOW);
      delayMicroseconds(tone_ / 2);   // use delayMicroseconds because period is measured in microseconds

      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    } 
  }
  else {                                    // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++) {
      delayMicroseconds(duration);  
    }                                
  }
}

void play()
{
  for (int i=0; i<MAX_COUNT; i++) 
      {
          tone_ = melody[i];
          beat = beats[i];

          duration = beat * tempo; // Set up timing

          song(); 
          // A pause between notes...
          delayMicroseconds(pause);
      }
}

/*
 * CUSTOM SHIELD ADDRESSES
 * - Turn Signal Blinkers
 * - Variant Car Sounds (Horn)
 * - Differential Drive
 */
#define BRAKE      0x40
#define TURNSIGNAL 0x50
#define HEADLIGHTS 0x51
#define BEEP       0x52
#define CRUISE     0x53

// Encoder Set-Up

// PID Controller Timer Set-Up

// Define DPINS for Custom Shield
#define lightsOff  0x00
#define leftBlink  0x01
#define rightBlink 0x08
#define headlight  0x06
#define hazard     0x09
#define Llights    0x07
#define Rlights    0x0E
#define Hlights    0x0F

// Speaker Pin
#define horn         22

Motor motorA;       // Create Motor A
Motor motorB;       // Create Motor B
Servo servo11;      // Create servo object

const uint8_t CMD_LIST_SIZE = 6;   // we are adding 4 commands (MOVE, SERVO1, TURNSIGNAL, BEEP, HEADLIGHTS, BRAKE)

void turnsig (uint8_t cmd, uint8_t param[]){
  checkturn = param[0];
    // Update Turn Signal Blinkers
} // turnsig

void blinker()
{
    switch (checkturn)
  {
    case 0x00:
    if (checklight == 0x01) {
      I2Cwrite(OUT,headlight);
    }
    if (checklight == 0x00) {
      I2Cwrite(OUT,lightsOff);
    }
    break;
    case 0x01:
    // Turning Right
    if (checklight == 0x01) {
      I2Cwrite(OUT,Rlights);
      delay(500);
      I2Cwrite(OUT,headlight);
      delay(500);
    }
    if (checklight == 0x00) {
      I2Cwrite(OUT,rightBlink);
      delay(500);
      I2Cwrite(OUT,lightsOff);
      delay(500);
    }
    break;
    case 0x02:
    // Turning Left
    if (checklight == 0x01) {
      I2Cwrite(OUT,Llights);
      delay(500);
      I2Cwrite(OUT,headlight);
      delay(500);
    }
    if (checklight == 0x00) {
      I2Cwrite(OUT,leftBlink);
      delay(500);
      I2Cwrite(OUT,lightsOff);
      delay(500);
    }
    break;
    case 0x03:
    // Hazards ON
    if (checklight == 0x01) {
      I2Cwrite(OUT,Hlights);
      delay(500);
      I2Cwrite(OUT,headlight);
      delay(500);
    }
    if (checklight == 0x00) {
      I2Cwrite(OUT,hazard);
      delay(500);
      I2Cwrite(OUT,lightsOff);
      delay(500);
    }
    break;
  }
  return;
}

void headlights (uint8_t cmd, uint8_t param[])
{
  checklight = param[0];
  // Update Headlights
}

void beep (uint8_t cmd, uint8_t param[])
{
  int i = param[0];
}

ArxRobot::cmdFunc_t onCommand[CMD_LIST_SIZE] = {{MOVE,moveHandler}, {SERVO,servoHandler}, {TURNSIGNAL,turnsig}, {HEADLIGHTS,headlights}, {BEEP,beep}, {BRAKE,brake}};

Packet motorPWM(MOTOR2_CURRENT_ID);  // initialize the packet properties to default values

void setup()
{
  // Play Start-Up Sound
  play();

  // Attach Steering Servo
  servo11.attach(11);
  
  // I2C Pin Access
  Wire.begin();
  I2Cwrite(0x03,0xF0);                 // I2C Set I/O Pins using Config Code 0x03
  delay(5);
  
  // Horn on DPIN 22
  pinMode(horn,OUTPUT);

  // Mechanical Setup
  ServoTest();                         // Run ServoTest to ensure functional SERVO
  motorPWM.setAccuracy(.001);          // change sensor accuracy from +/- 2 DN to +/- 1 DN
  motorPWM.setSamplePeriod(500000);    // sample period from 1 second to 0.5 seconds
  motorA.begin(AIN1,AIN2,PWMA);        // begin(controlpin1,controlpin2,pwmPin)
  motorB.begin(BIN1,BIN2,PWMB);
  pinMode(STBY,OUTPUT);
  digitalWrite(STBY, HIGH);
  
  Serial.begin(57600);               // default = 115200
  ArxRobot.begin();
  ArxRobot.setCurrentLimit(N);
  
  ArxRobot.setOnCommand(onCommand, CMD_LIST_SIZE);
}

void loop()
{
  ArxRobot.loop();
  /*
   * Continuously update LEDs
   */
  blinker();
//  Monitor Data/Values
//  Serial.print("Motor A:");   // Speed of Motor A
//  Serial.println(speedA);
//  Serial.print("Motor B:");   // Speed of Motor B
//  Serial.println(speedB);
//  Serial.print("Encoder:");
//  Serial.println(encoder);    // Check Encoder
    /*
     * Telemetry
     *         Read sensor and send packet
     *         To simulate the data stream coming from the sensor we will read ATmega32U4
     *         Register OCR4D which controls the duty cycle of MOTOR 2. This code segment
     *         uses the preproccessor conditional directives #if to make sure that an MCU
     *         with a ATmega32U4 or ATmega16U4 is selected under Tools > Board.
     */
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    uint16_t pwm_reading = (uint16_t) OCR4D;  // read 8-bit Output Compare Register Timer 4D and cast to 16-bit signed word
    motorPWM.sendSensor(pwm_reading);
  #else
    // Timer/Counter 0 registers set by UNO Bootloader (freq. = 1 Khz)
    // Timer/Counter Control Registers TCCR0B = 0x03 (Prescaler of 64) and TCCR0A (Fast PWM, TOP = 0xFF) = 0x03
    uint16_t pwm_reading = (uint16_t) OCR0B;
    motorPWM.sendSensor(pwm_reading);
  #endif
}

void ServoTest() {
/*
 * Initialize Servo to Test Performance
 */
  for (byte x = 0; x < 1; x++)
  {
    servo11.write(100);
    delay(150);
    servo11.write(80);
    delay(150);
    servo11.write(120);
    delay(150); 
  }
  servo11.attach(11);
  servo11.write(angle);
}

// Define Speed Parameters and Range
uint8_t maxspd = 148;
uint8_t minspd = 73;
void moveHandler (uint8_t cmd, uint8_t param[], uint8_t n)
{
  LM = param[0];               // Check direction of motors
  RM = param[2];
    Serial.write(cmd);             // move command = 0x01
    Serial.write(n);               // number of param = 4
    for (int i=0;i<n;i++)          // param = L_dir L_spd R_dir R_spd
    {
      Serial.write(param[i]);
    }
    // AUTOMATIC BRAKE
    if (brakes == 0x01) {
      if (LM == 0x04) {
        for (speedA;speedA > 0x00; speedA--) {
          motorA.go(LM,speedA);
          delay(100);
          break;
        }
        for (speedB;speedB > 0x00; speedB--) {
          motorB.go(RM,speedB);
          delay(100);
          break;
        }
      }
    }
//    /*
//     * D-PAD Control
//     *    AND Logic compares direction of both motors ***(RM is reverse polarity)***
//     *      +  01,01 = FORWARD 
//     *      +  02,02 = BACK
//     *      +  01,02 = RIGHT
//     *      +  02,01 = LEFT
//     */
     if (LM == 0x02 & RM == 0x02)   // Turn Right
     {
      // Differential Drive (LEFT MOTOR > RIGHT MOTOR)
        servo11.write(angle-15);
        if (speedB > minspd) {
          speedB = speedB - 0x15;
          motorB.go(0x02,speedB);
          delay(50);
        }
        if (speedA < maxspd) {
          speedA = speedA + 0x15;
          motorA.go(0x01,speedA);
          delay(50);
        }
     }
     if (LM == 0x01 & RM == 0x01)   // Turn Left
     {
      // Differential Drive (RIGHT MOTOR > LEFT MOTOR)
        servo11.write(angle+20);
        if (speedA > minspd) {
          speedA = speedA - 0x15;
          motorA.go(0x01,speedA);
          delay(50);
        }
        if (speedB < maxspd) {
          speedB = speedB + 0x15;
          motorB.go(0x02,speedB);
          delay(50);
        }
     }
/*
 * FORWARD/REVERSE DRIVE
 */
     if (LM == 0x01 & RM == 0x02 || LM == 0x02 & RM == 0x01)
     {
      /*
       * Gradual increase/decrease in speed in the forward/reverse drive modes
       * Maximum PWM amplitude is designated 231 (6V motors)
       * Minimum PWM amplitude is 43
       * increments are in steps of 15
       */
      if (LM == 0x01) {
        if (speedA < maxspd && speedA < param[1]) {
        speedA = speedA + 0x15;
        delay(250);
        }
        if  (speedB < maxspd & speedB < param[3]) {
        speedB = speedB + 0x15;
        delay(250);
        }
      }
      else if (LM == 0x02) {
        if (speedA > minspd && speedA > param[1]) {
        speedA = speedA - 0x15;
        delay(250);
        }
        if (speedB > minspd && speedB > param[3]) {
        speedB = speedB - 0x15;
        delay(250);
        }
      }
        motorA.go(LM,speedA);
        motorB.go(RM,speedB);
        servo11.write(100);                   // Reorient servo to the middle
     }
}  // moveHandler

void servoHandler (uint8_t cmd, uint8_t param[])
{
    int n = param[1];
    for (angle;angle<n;angle++)
    {
      servo11.write(angle);       // turn servo to specified angle
      delay(100);
    }
    for (angle;angle>n;angle--)
    {
      servo11.write(angle);       // turn servo to specified angle
      delay(100);
    }
}  // servoHandler 

void brake (uint8_t cmd, uint8_t param[])
{
  brakes = param[0];
}
