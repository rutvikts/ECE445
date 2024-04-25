#include <Wire.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SPI.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x04

uint8_t motor_bytes[3];
int request_count = 0;
int actionItem = 1;

// Defining Motors
Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo

// Define Input Connections
#define CH1 4
#define CH2 5
#define CH4 6

// Define PWM Output Connections
#define PWM1 9
#define PWM2 10
#define PWM3 11

// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
int ch4Value; 

int RXLED = 17; // The RX LED has a defined Arduino pin
int TXLED = 30; // The TX LED has a defined Arduino pin

// PWM Motor Speeds
int speed1 = 0;
int speed2 = 0;
int speed3 = 0;

// Setup for SPI bus
volatile boolean received;
volatile byte receivedData;
volatile byte tempReceivedData;

//Inerrrput routine function for SPI slave setup
ISR (SPI_STC_vect) {
  receivedData = SPDR;   // Get the received data from SPDR register
  received = true;       // Sets received as True 
}

unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

void setup(){
  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH4, INPUT);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT); // Set RX LED as an output
  pinMode(TXLED, OUTPUT); // Set TX LED as an output
  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  digitalWrite(RXLED, HIGH); // turn the LED off by making the voltage LOW
  digitalWrite(TXLED, LOW); // turn the LED off by making the voltage LOW


  // Setting up 
  myservo1.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo3.attach(11);  // attaches the servo on pin 9 to the servo object

  // Setting up SPI Bus
  pinMode(MISO,OUTPUT);   //Sets MISO as OUTPUT
  SPCR |= _BV(SPE);       //Turn on SPI in Slave Mode
  received = false;
  SPI.attachInterrupt();  //Activate SPI Interuupt 

  // Setting up PWM outputs
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
}


void loop() {
  // Do This

  if(received) {       
    // SPDR = receivedData;    // send back the received data, this is not necessary, only for demo purpose
    SPDR = 0;
    SPDR = 0x88;
    // Serial.print("SPI Received: ");
    received = false;

    digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)

    // SPDR = receivedData + 1;
    if (receivedData == 0x67) {
      digitalWrite(TXLED, HIGH); // turn the LED on (HIGH is the voltage level)
      tempReceivedData = reverse(receivedData);
    }
    else {
      digitalWrite(TXLED, LOW); // turn the LED on (HIGH is the voltage level)
    }            
  }
  else{
    digitalWrite(LED_BUILTIN, LOW); // turn the LED on (HIGH is the voltage level)
  }
}

void requestEventSPI(){
  if (actionItem == 1) {
          // action item 1 code
          SPDR = motor_bytes[0];
          actionItem = 2;
      } else if (actionItem == 2) {
          // action item 2 code
          SPDR = motor_bytes[1];
          actionItem = 3;
      } else if (actionItem == 3) {
          // action item 3 code
          SPDR = motor_bytes[2];
          actionItem = 1;
      }
}