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
static uint8_t LM = 0x00;
static uint8_t RM = 0x00;

// include C:\Program Files (x86)\Arduino 1_6_5\hardware\arduino\avr\libraries
#include <ArxRobot.h>    // instantiated as ArxRobot at end of class header
#include <EEPROM.h>
#include <Wire.h>        // I2C support


ArxRobot ArxRobot;       // instantiated as ArxRobot at end of class header

// Steps for current limiter
uint8_t N = 40;

/*
 * NOTE: Custom command address space 0x40 - 0x5F.
 * We will keep SERVO defined here for manual servo adjustment as needed
 */
#define SERVO 0x41
#define STBY  8
static uint8_t angle = 100;
static uint8_t checkturn = 0x00;
/*
 * CUSTOM SHIELD ADDRESSES
 * - Turn Signal Blinkers
 * - Variant Car Sounds (Horn)
 * - Differential Drive
 */
#define BEEP       0x51
#define TURNSIGNAL 0x50
#define CRUISE     0x52

// I2C Set-Up
// -I2C 9534A  /
#define INADD 0x38  // Input Address
#define OUTADD 0x39 // Output Address

// Encoder Set-Up

// PID Controller Timer Set-Up

// Define DPINS for Custom Shield
#define blinker_r 14
#define blinker_l 15   
#define horn      22

Motor motorA;       // Create Motor A
Motor motorB;       // Create Motor B
Servo servo11;      // Create servo object

const uint8_t CMD_LIST_SIZE = 4;   // we are adding 4 commands (MOVE, SERVO1, TURNSIGNAL, BEEP, CRUISE)

void turnsig (uint8_t cmd, uint8_t param[])
{
  checkturn = param[0];
} // turnsig

void beep (uint8_t cmd, uint8_t param[])
{
  byte freq = 100;
  // Press to honk horn
  byte honk = param[0];
  if (horn == 0x01)
  {
    // Will sound twice
    tone(horn,freq,200);
    delay(100);
    tone(horn,freq,1000);
  }
}
ArxRobot::cmdFunc_t onCommand[CMD_LIST_SIZE] = {{MOVE,moveHandler}, {SERVO,servoHandler}, {TURNSIGNAL,turnsig}, {BEEP,beep}};

Packet motorPWM(MOTOR2_CURRENT_ID);  // initialize the packet properties to default values

void setup()
{
  servo11.attach(11);
  // I2C Pin Access
  pinMode(2,INPUT);
  Wire.begin();
  expanderSetInput(INADD, 0xFF);
  // Blinkers on DPIN 14, 16
  pinMode(blinker_r,OUTPUT);
  pinMode(blinker_l,OUTPUT);

  // Horn on DPIN 22
  pinMode(horn,OUTPUT);
 
  pinMode(STBY,OUTPUT);
  ServoTest();                         // Run ServoTest to ensure functional SERVO
  motorPWM.setAccuracy(.001);          // change sensor accuracy from +/- 2 DN to +/- 1 DN
  motorPWM.setSamplePeriod(500000);    // sample period from 1 second to 0.5 seconds
  motorA.begin(AIN1,AIN2,PWMA);     // begin(controlpin1,controlpin2,pwmPin)
  motorB.begin(BIN1,BIN2,PWMB);

  pinMode(14,INPUT);
  pinMode(16,INPUT);
  digitalWrite(STBY, HIGH);
  
  Serial.begin(57600);               // default = 115200
  ArxRobot.begin();
  ArxRobot.setCurrentLimit(N);
  
  ArxRobot.setOnCommand(onCommand, CMD_LIST_SIZE);
}

void expanderSetInput(int i2caddr,byte dir) {
  Wire.beginTransmission(i2caddr);
  Wire.write(dir);   // outputs high for input
  Wire.endTransmission();
}

byte expanderRead(int i2caddr) {
  int _data = -1;
  Wire.requestFrom(i2caddr,1);
  if(Wire.available()) {
    _data = Wire.read();
  }
  return _data;
}

void expanderWrite(int i2caddr, byte data) {
  Wire.beginTransmission(i2caddr);
  Wire.write(data);
  Wire.endTransmission();
}

void loop()
{
  // Check data received from I2C
  int data = expanderRead(INADD);
  ArxRobot.loop();
  // Update Turn Signal Blinkers
  switch (checkturn)
  {
    case 0x00:
    digitalWrite(blinker_r,LOW);
    digitalWrite(blinker_l,LOW);
    break;
    case 0x01:
    // Turning Right
    digitalWrite(blinker_r,HIGH);
    delay(500);
    digitalWrite(blinker_r,LOW);
    delay(500);
    break;
    case 0x02:
    // Turning Left
    digitalWrite(blinker_r,LOW);
    digitalWrite(blinker_l,HIGH);
    delay(500);
    digitalWrite(blinker_l,LOW);
    digitalWrite(blinker_r,LOW);
    delay(500);
    break;
    case 0x03:
    // Hazards ON
    digitalWrite(blinker_r,HIGH);
    digitalWrite(blinker_l,HIGH);
    delay(500);
    // Hazards OFF
    digitalWrite(blinker_r,LOW);
    digitalWrite(blinker_l,LOW);
    delay(500);
    break;
  }
  /*
   * Monitor Data/Values
   */
    Serial.println(data);
//  Serial.print("Motor A:");   // Speed of Motor A
//  Serial.println(speedA);
//  Serial.print("Motor B:");   // Speed of Motor B
//  Serial.println(speedB);
  Serial.println(checkturn);  // Check the Turn Signal Flag
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
    servo11.write(90);
    delay(100);
    servo11.write(70);
    delay(100);
    servo11.write(110);
    delay(100); 
  }
  servo11.attach(11);
  servo11.write(angle);
}

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
    // BRAKE
//    if (LM == 0x04) {
//      for (speedA;speedA > 0x00; speedA--) {
//        motorA.go(LM,speedA);
//        delay(100);
//        break;
//      }
//      for (speedB;speedB > 0x00; speedB--) {
//        motorB.go(RM,speedB);
//        delay(100);
//        break;
//      }
//    }
//    
//    /*
//     * D-PAD Control
//     *    AND Logic compares direction of both motors ***(RM is reverse polarity)***
//     *      +  01,01 = FORWARD 
//     *      +  02,02 = BACK
//     *      +  01,02 = RIGHT
//     *      +  02,01 = LEFT
//     */
     if (LM == 0x01 & RM == 0x01)   // Turn Right
     {
      // Differential Drive (LEFT MOTOR > RIGHT MOTOR)
        servo11.write(angle-15);
        if (speedB > 43) {
          speedB = speedB - 0x15;
          motorB.go(0x02,speedB);
          delay(50);
        }
        if (speedA < 231) {
          speedA = speedA + 0x15;
          motorA.go(0x01,speedA);
          delay(50);
        }
     }
     if (LM == 0x02 & RM == 0x02)   // Turn Left
     {
      // Differential Drive (RIGHT MOTOR > LEFT MOTOR)
        servo11.write(angle+20);
        if (speedA > 43) {
          speedA = speedA - 0x15;
          motorA.go(0x01,speedA);
          delay(50);
        }
        if (speedB < 231) {
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
        if (speedA < 231 && speedA < param[1]) {
        speedA = speedA + 0x15;
        delay(500);
        }
        if  (speedB < 231 & speedB < param[3]) {
        speedB = speedB + 0x15;
        delay(500);
        }
      }
      else if (LM == 0x02) {
        if (speedA > 43 && speedA > param[1]) {
        speedA = speedA - 0x15;
        delay(500);
        }
        if (speedB > 43 && speedB > param[3]) {
        speedB = speedB - 0x15;
        delay(500);
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
