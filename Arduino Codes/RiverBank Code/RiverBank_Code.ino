// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX
//
// Last modified: SK and JE 12.57pm 11 Apr 2019

#include <SPI.h>
#include <RH_RF95.h>

// for feather m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

byte N;
byte inChar = N;

struct dataStruct {
  float pressure_top;
  float pressure_bottom;
  float myIMU_ax;
  float myIMU_ay;
  float myIMU_az;
  float myIMU_gx;
  float myIMU_gy;
  float myIMU_gz;
  float battery_voltage;
} SensorReadings;

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

  Serial.println("Feather LoRa TX Test!");

  SensorReadings.pressure_top = 0;
  SensorReadings.pressure_bottom = 0;

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  if (Serial.available()) {

    inChar = Serial.read();
    int Nthrowaway = emtpySerialQueue();
    //Serial.print lines used for troubleshooting
    //Serial.print("I received character in Serial:  "); Serial.println(inChar);
    //Serial.print("Also threw away N number of bytes read from Serial =  "); Serial.println(Nthrowaway);

    delay(100); // Wait 1 second between transmits, could also 'sleep' here!
    //Serial.println("Transmitting..."); // Send a message to rf95_server

    byte radiopacket[1] = {inChar};
    //itoa(packetnum, radiopacket+1, 16);
    //Serial.print("Sending "); Serial.println(radiopacket[0]);

    //Serial.println("Sending Radiopacket RF95...");
    delay(50);
    rf95.send((uint8_t *)radiopacket, 1);


    //Serial.println("Waiting for packet to complete...");
    delay(10);
    rf95.waitPacketSent();
    Serial.println("finished packet Tx");
    delay(200);

    // Enumerate all possible modular actions here.  User send serial character to trigger these events.
    switch (inChar) {

      case 'S':  //Start!!
        Serial.println();
        Serial.println("Started logging to SD");
        break;

      case 'F': //close SD log file, JE 07 Dec 2018
        Serial.println();
        Serial.println("Finished logging to SD");
        break;

      case 'R': //turn radio on
        Serial.println();
        while (!rf95.waitAvailableTimeout(500)) {
          Serial.println("Sending character R");
          rf95.send((uint8_t *)radiopacket, 1);
          rf95.waitPacketSent();
          Serial.println("finished sending R. Waiting 500ms for response");
        }
        Serial.println("Received a response from SmartRock radio");
        break;

      case 'O': //turn radio off
        Serial.println();
        break;

      case 'B': //call battery voltage
        Serial.println();
        Serial.print("Battery voltage = "); Serial.print(SensorReadings.battery_voltage); Serial.println("V");
       delay(3000);
        break;

      case 'G': //Go Signal
       Serial.println();
        Serial.println("G = Go Signal - write to SD card, turn off radio, reset baseline, turn off serial port");
        break;

      case 'Z': //manual trigger for fast
       Serial.println();
        Serial.println("Fast Sampling Rate");
        break;

      case 'Y': //manual trigger for slow
        Serial.println();
        Serial.println("Slow Sampling Rate");
        break;

      case 'L': //toggle test LED
        Serial.println();
        Serial.println("Toggle test LED");
        break;

      case '?': //Explain the possbile commands
        Serial.println();
        Serial.println("Help menu:");
        Serial.println("S = Start logging to SD");
        Serial.println("F = Finish logging to SD");
        Serial.println("R = Turn radio on");
        Serial.println("O = Turn radio off");
        Serial.println("G = Go Signal - write to SD card, turn off radio, reset baseline, turn off serial port");
        Serial.println("B = View Battery voltage");
        Serial.println("Z = Manual Fast Sampling Rate");
        Serial.println("Y = Manual Slow Sampling Rate");
        Serial.println("L = Toggle Test LED");
        Serial.println();
        delay(5000);
        break;

      default:
        Serial.println("Sorry, this command not recognized. Use characters: S, F, R, O, G, B, G, Z, Y, or L. For help, enter: ? ");
        delay(500);
        break;
    }
  }

  Serial.println();
  Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(500))
  {
    RecieveFromSensors();
  }
  else
  {
    Serial.println("Receive failed");
  }
}

//subfunction to clear serial queue. typically used to clear /r /n characters we don't want to send

int emtpySerialQueue(void) {

  int numChucked = 0;
  while (Serial.available()) {
    Serial.read();
    numChucked += 1;
  }
  return numChucked;
}

void RecieveFromSensors()
{

  // Dont put this on the stack:
  uint8_t buf[sizeof(SensorReadings)];
  uint8_t from;
  uint8_t len = sizeof(buf);

  if (rf95.available())
  {
    {
      // Wait for a message addressed to us from the client
      if (rf95.recv(buf, &len))
      {
        memcpy(&SensorReadings, buf, sizeof(SensorReadings));
        Serial.println("--------------------------------------------");
        Serial.print("Got message from unit: ");
        Serial.println(from, DEC);

        Serial.print("Pressure top  ");
        Serial.println(SensorReadings.pressure_top);

        Serial.print("Pressure bottom ");
        Serial.println(SensorReadings.pressure_bottom);

        Serial.print("X-acceleration: "); Serial.print(SensorReadings.myIMU_ax);
        Serial.println(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(SensorReadings.myIMU_ay);
        Serial.println(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(SensorReadings.myIMU_az);
        Serial.println(" mg ");

        // Print gyro values in rad/sec
        Serial.print("X-gyro rate: "); Serial.print(SensorReadings.myIMU_gx);
        Serial.println(" rad/sec ");
        Serial.print("Y-gyro rate: "); Serial.print(SensorReadings.myIMU_gy);
        Serial.println(" rad/sec ");
        Serial.print("Z-gyro rate: "); Serial.print(SensorReadings.myIMU_gz);
        Serial.println(" rad/sec");
      }
    }
  }
}
