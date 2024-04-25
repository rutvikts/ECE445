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
// #define CH3 6
#define CH4 6
// #define CH5 10
// #define CH6 11

// Define PWM Output Connections
#define PWM1 9
#define PWM2 10
#define PWM3 11

// Integers to represent values from sticks and pots
int ch1Value;
int ch2Value;
// int ch3Value;
int ch4Value; 
// int ch5Value;

// Boolean to represent switch value
// bool ch6Value;

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

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void find_inverse(double x, double y, double w) { //, uint8_t motor_bytes[]
    // Define the angle offsets for each wheel
    double wheel_offsets[] = {270, 150, 30}; // For motors 1, 2, 3

    // Define the maximum speed for each wheel (in the range of 0 to 1)
    double max_speed = 1.0;

    double a[3][1] = {{x}, 
					  {y}, 
					  {w}};
    double inv_mat[3][3] = {{2.0/3.0, 0.0, 1.0/3.0},
                            {-1.0/3.0, -20000.0/34641.0, 1.0/3.0},
                            {-1.0/3.0, 20000.0/34641.0, 1.0/3.0}};

    double f[3][1] = {{0}, 
					  {0}, 
					  {0}};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 1; j++) {
            for (int k = 0; k < 3; k++) {
                f[i][j] += inv_mat[i][k] * a[k][j];
            }
        }
    }

    // Calculate the speed and direction for each wheel
    for (int i = 0; i < 3; i++) {
        // Calculate the speed for the wheel based on the angle
        double wheel_speed = max_speed * f[i][0];
        wheel_speed = ((wheel_speed) * 3.1); //fabs(wheel_speed)

        uint8_t speed_byte = (uint8_t)(fabs(wheel_speed) * 3.1);

        // Determine the direction of rotation for this wheel
        uint8_t direction_byte = (f[i][0] < 0) ? 1 : 0;

        // Determine the motor number byte
        uint8_t motor_number_bits = i + 1;

        // Combine the bytes into a single value and append to the output array
        uint8_t motor_value = 0;
		    motor_value = (motor_number_bits << 6) | (direction_byte << 5) | speed_byte;
        // Serial.print("Motor: ");
        // Serial.print(i);
        // Serial.print(" | Motor Value: ");
        // Serial.print(motor_value); 
        // Serial.print(" | ");
        // Serial.print(" | Motor Direction: ");
        // Serial.print(direction_byte); 
        // Serial.println(" | ");
        motor_bytes[i] = motor_value;

        if (i == 0){
          // Serial.print("Wheel speed 1: ");
          // Serial.print(wheel_speed);

          if (direction_byte == 0){
            speed1 = map(motor_value, 64, 96, 90, 0);
          }
          else {
            speed1 = map(motor_value, 64, 124, 90, 180);
          }
          speed1 = constrain(speed1, 0, 180);
          myservo1.write(speed1);
          // Serial.print(", M1: ");
          // Serial.println(speed1);
        }
        else if (i == 1){
          // Serial.print("Wheel speed 2: ");
          // Serial.print(wheel_speed);

          if (direction_byte == 0){
            speed2 = map(motor_value, 128, 160, 90, 0);
          }
          else {
            speed2 = map(motor_value, 128, 188, 90, 180);
          }
          speed2 = constrain(speed2, 0, 180);
          myservo2.write(speed2);

          // Serial.print(", M2: ");
          // Serial.println(speed2);
        }
        else if (i == 2){
          // Serial.print("Wheel speed 3: ");
          // Serial.print(wheel_speed);

          if (direction_byte == 0){
            speed3 = map(motor_value, 192, 224, 90, 0);
          }
          else {
            speed3 = map(motor_value, 192, 252, 90, 180);
          }
          speed3 = constrain(speed3, 0, 180);

          myservo3.write(speed3);

          // Serial.print(", M3: ");
          // Serial.println(speed3);

        }

        // ---------------
    }
}

// unsigned char reverse(unsigned char b) {
//    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
//    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
//    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
//    return b;
// }

void setup(){
  // Set up serial monitor
  Serial.begin(115200);
  // Serial.begin(1000000);
  
  // Set all pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  // pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  // pinMode(CH5, INPUT);
  // pinMode(CH6, INPUT);

  // Setting up 
  myservo1.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo3.attach(11);  // attaches the servo on pin 9 to the servo object

  // Setting up SPI Bus
  pinMode(MISO,OUTPUT);   //Sets MISO as OUTPUT
  SPCR |= _BV(SPE);       //Turn on SPI in Slave Mode
  received = false;
  SPI.attachInterrupt();  //Activate SPI Interuupt 

  // // Setting up PWM outputs
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);

}


void loop() {
  // Do This
	double x_vector = (double)readChannel(CH1, -10, 10, 0); //channel, min, max, default
	double y_vector = (double)readChannel(CH2, -10, 10, 0);
  double w_vector = (double)readChannel(CH4, 10, -10, 0);

  if (abs(x_vector) < 3){
    x_vector = 0.0;
  }
  if (abs(y_vector) < 3){
    y_vector = 0.0;
  }
  if (abs(w_vector) < 3){
    w_vector = 0.0;
  }
	find_inverse(x_vector, y_vector, w_vector); //, motor_bytes

  // Print to Serial Monitor
  // Serial.print("X: ");
  // Serial.print(x_vector);
  // Serial.print(" | Y: ");
  // Serial.print(y_vector);
  // Serial.print(" | W: ");
  // Serial.println(w_vector);
  
	delay(5);

  // if(received) {       
  //   // SPDR = receivedData;    // send back the received data, this is not necessary, only for demo purpose
   

  //   tempReceivedData = reverse(receivedData);

  //   Serial.print("SPI Received: ");
  //   received = false;
  //   Serial.print(receivedData, HEX);
  //   Serial.print(" | ");
  //   Serial.println(tempReceivedData, HEX); //, HEX
  //   if (receivedData == 0x88) {
  //     Serial.println("SPI Received Motor Byte request");
  //     // uint8_t temp[3];
  //     // temp[0] = 0x12;
  //     // temp[1] = 0x34;
  //     // temp[2] = 0x56;
  //     // SPDR = temp;
  //     // requestEventSPI();
  //   }            
    
  // }
  
  // // Get values for each channel
  // ch1Value = readChannel(CH1, -100, 100, 0);
  // ch2Value = readChannel(CH2, -100, 100, 0);
  // // ch3Value = readChannel(CH3, -100, 100, -100);
  // ch4Value = readChannel(CH4, -100, 100, 0);
  // // ch5Value = readChannel(CH5, -100, 100, 0);
  // // ch6Value = readSwitch(CH6, false);
  
  // // Print to Serial Monitor
  // Serial.print("Ch1: ");
  // Serial.print(ch1Value);
  // Serial.print(" | Ch2: ");
  // Serial.print(ch2Value);
  // // Serial.print(" | Ch3: ");
  // // Serial.print(ch3Value);
  // Serial.print(" | Ch4: ");
  // Serial.println(ch4Value);
  // Serial.print(" | Ch5: ");
  // Serial.print(ch5Value);
  // Serial.print(" | Ch6: ");
  // Serial.println(ch6Value);
  
  // delay(500);
}



// void requestEventSPI(){
//   if (actionItem == 1) {
//           // action item 1 code
//           SPDR = motor_bytes[0];
//           actionItem = 2;
//       } else if (actionItem == 2) {
//           // action item 2 code
//           SPDR = motor_bytes[1];
//           actionItem = 3;
//       } else if (actionItem == 3) {
//           // action item 3 code
//           SPDR = motor_bytes[2];
//           actionItem = 1;
//       }
// }