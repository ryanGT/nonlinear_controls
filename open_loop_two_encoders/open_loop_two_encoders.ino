#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
//#include <Servo.h>
//#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
 
#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc0(2, 4);
Encoder enc1(3, 5);

//#include <avr/io.h>
//#include <avr/interrupt.h>
#define LEDPIN 13

// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.
 
// Quadrature encoders
//!#define encoderPinA 2
//!#define encoderPinB 4
//!#define isrPin 5
#define sendPin 8
#define receivePin 7
#define triggerPin 12
#define directionPin 9

int analogPin = 0;
int accel = 0;
 
//  encoder
//#define c_EncoderInterrupt 3
//#define c_EncoderPinA 3
//#define c_EncoderPinB 5
volatile bool _EncoderBSet;
volatile long _EncTicks0 = 0;
volatile long _EncTicks1 = 0;

int n;
int n2;
int nIn, nOut, nISR;
int v1, v2;
#define offset 141

int inByte;
int outByte;

int fresh;
bool send_ser;

 
//Servo _Servo;  // create servo object to control right motor
//Servo _LeftServo;  // create servo object to control left motor
 
//int potpin = 0;  // analog pin used to connect the potentiometer
//int val;    // variable to read the value from the analog pin


int pwmA = 6;//3           // the pin that the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
 
//unsigned char MSB = 0;  // 1 byte in Arduino
//unsigned char LSB = 0;  // 1 byte  in Arduino

void setup()
{
  Serial.begin(115200);
 
  send_ser = false;

  //_Servo.attach(2);  // attaches the servo on specified pin to the servo object
  //_LeftServo.attach(3);  // attaches the servo on specified pin to the servo object

  pinMode(pwmA, OUTPUT);
  pinMode(directionPin, OUTPUT);
  digitalWrite(directionPin, LOW);

  // Quadrature encoders
  // Left encoder
  //!pinMode(encoderPinA, INPUT); 
  //!pinMode(encoderPinB, INPUT); 
  //!pinMode(isrPin, OUTPUT);
  pinMode(sendPin, OUTPUT);
  pinMode(receivePin, OUTPUT);
  pinMode(triggerPin, OUTPUT);

  //if send pin will be different from isrpin, then we need to set up
  //its output mode as well
  //pinMode(sendPin, OUTPUT);
  //!digitalWrite(isrPin, LOW);
  digitalWrite(receivePin, LOW);
  digitalWrite(triggerPin, LOW);
  digitalWrite(sendPin, LOW);
  // turn on pullup resistors
  //!digitalWrite(encoderPinA, HIGH);
  //!digitalWrite(encoderPinB, HIGH);

  // encoder pin on interrupt 0 (pin 2)
  //!attachInterrupt(0, doEncoder, RISING);

  Serial.print("open loop two encoders 3/26/15");
  Serial.print("\n");
  n = 0;
  n2 = 0; 
  nIn = 0;
  nOut = 0;
  nISR = 0;
  inByte = 0;
  outByte = 0;
  fresh = 0;
  v1 = offset;
  v2 = 0;//offset;
  send_ser = false;

  //---------------------------
  //
  // Timer Interrupt stuff
  //
  //---------------------------
  pinMode(LEDPIN, OUTPUT);
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 90;//serial test 1 - roughly 150 Hz
  //OCR1A = 80;//serial test 2 - roughly 175 Hz
  //OCR1A = 65;//serial test 3 - roughly 200Hz


  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);

  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
 
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);

  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin

  analogWrite(pwmA, offset);

  //----------------------------
  //
  // Attempting fast PWM
  //
  //----------------------------
  //the 2 lines below work well
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00); 
  TCCR0B = _BV(CS00);
  ////TCCR0B = _BV(CS01);
  ////TCCR0B |= (1 << CS11); 
  //// enable global interrupts:
  sei();
}

unsigned char getsecondbyte(int input){
    unsigned char output;
    output = (unsigned char)(input >> 8);
    return output;
}

 

int reassemblebytes(unsigned char msb, unsigned char lsb){
    int output;
    output = (int)(msb << 8);
    output += lsb;
    return output;
}

int readtwobytes(void){
    unsigned char msb, lsb;
    int output;
    int iter = 0;
    while (Serial.available() <2){
      iter++;
      if (iter > 1e5){
	break;
      }
    }
    msb = Serial.read();
    lsb = Serial.read();
    output = reassemblebytes(msb, lsb);
    return output;
}

void SendTwoByteInt(int intin){
    unsigned char lsb, msb;
    lsb = (unsigned char)intin;
    msb = getsecondbyte(intin);
    Serial.write(msb);
    Serial.write(lsb);
}


void loop()
{
  //val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  //val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180)
 
  //_Servo.write(val);
  //_LeftServo.write(val);

  //Serial.print("main loop print");
  //Serial.print("\n");
 
  //delay(10);
  //analogWrite(pwmA, brightness);    

  //digitalWrite(12, HIGH); //Establishes forward direction of Channel A
  digitalWrite(12, LOW); //Establishes forward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A

  //delay(10);
 
  if (Serial.available() > 0) {
    digitalWrite(receivePin, HIGH);
    // get incoming byte:
    //If using the Arudino Serial Monitor, you need to subtract '0' to offset the ASCII code by 48 (I think)
    inByte = Serial.read();
    if (inByte == 1){
      //main control case
      send_ser = true;
      nIn = readtwobytes();
      v1 = readtwobytes();
      v1 += offset;
    }
    else if (inByte == 2){
      //start new test
      //!_EncTicks0 = 0;
      //enc0.write(0);
      //enc1.write(0);
      send_ser = true;
      digitalWrite(triggerPin, HIGH);
      nISR = -1;
      v1 = offset;
    }
    else if (inByte == 3){
      send_ser = false;
      v1 = offset;
      digitalWrite(triggerPin, LOW);
    }
    else if (inByte == 4){
      //reset encoders
      enc0.write(0);
      enc1.write(0);
    }
    else if (inByte == 5){
      _EncTicks0 = enc0.read();
      _EncTicks1 = enc1.read();
      SendTwoByteInt(_EncTicks0);
      SendTwoByteInt(_EncTicks1);
      Serial.write(10);
    }
    digitalWrite(receivePin, LOW);
  }
  
  if (fresh > 0){
    fresh = 0;
    if (send_ser){
      //send_ser = false;
      digitalWrite(sendPin, HIGH);
      SendTwoByteInt(nISR);
      SendTwoByteInt(v1);
      _EncTicks0 = enc0.read();
      _EncTicks1 = enc1.read();
      SendTwoByteInt(_EncTicks0);
      SendTwoByteInt(_EncTicks1);
      Serial.write(10);
      digitalWrite(sendPin, LOW);
    }
    if (nISR > 30000){
      send_ser = false;
      v1 = offset;
      digitalWrite(triggerPin, LOW);
    }
  }
}
 
// Interrupt service routines for the right motor's quadrature encoder
//void doEncoder()
//{
//  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
//  n++;
//
//  _EncoderBSet = digitalRead(encoderPinB);   // read the input pin
//  
//  // and adjust counter + if A leads B
//  if (_EncoderBSet){
//    _EncTicks0 ++;
//  }
//  else {
//    _EncTicks0 --;
//  }
//}

ISR(TIMER1_COMPA_vect)
{     
  //digitalWriteFast(isrPin, HIGH);
  //!digitalWrite(isrPin, HIGH);
  nISR++;
  nOut = v1*v1;
  if ( v1 < 0 ){
    digitalWrite(directionPin, HIGH);
    analogWrite(pwmA, -v1); 
  }
  else{
    digitalWrite(directionPin, LOW);
    analogWrite(pwmA, v1); 
  }
  //analogWrite(pwmA, v1); 
  //accel = analogRead(analogPin);
  fresh = 1;
  //!digitalWrite(isrPin, LOW);
}

