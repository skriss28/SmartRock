/* SmartRock code developed by Stevan Kriss and Laws Smith with the help of
   Professor Erickson. Created April 2019

     Last Modified:
     31 Mar 2020 JE:
        - fixed SPI conflict between SD card and radio
        - fixed 'S' via Serial option for radio (prev did not enable SD writes)
        - documenting libraries used

     09 Apr 2020 JE:
        - SD dump updated: initialize card if it isn't already.  This allows us to dump straight away
         without initializing using 'I'.
         - SD initialize 'I' now also lists files currently on the SD card.  Should we make this a separate sd command?

     10 Apr 2020 JE:
        - Setup updated. SD card now initializes with other hardware sensors. Execution halts if any eror encountered

     12 April 2020 LS:
       - Cleaned up settings code to work with math portion at bottom of Settings.h
       - Verified that the code works with the full system and can be finalized

     23 April 2020 JE:
        - SD card operations updated for listing files at configure and dumpd
        - SD card wipe option added ('E' option for erase)

     24 April 2020 JE:
       - SD card dump modified to sync COM port operations with matlab.

     06 May 2020 LS:
       - Changed code so that array size is fixed (allows user to change basline window, average moving array, and sample size)
       - Changed timing code so that sleep function is at the end of the code
       - Final (fingers crossed) code uploaded to final module

     04 September 2020 LS
       - updated code to FINALLY give to Harbor
       - switched pressure sensor readings from bottom to top becasue testing is donechan

       
     

*/


//library files
#include <RHDatagram.h>
#include <RH_ASK.h>
#include <SPI.h>
#include <RH_RF95.h> // Using RH library downloaded from adafruit: https://cdn-learn.adafruit.com/assets/assets/000/035/106/original/RadioHead-1.62.zip?1472068723
#include <Wire.h>  // The Wire library carries out I2C communication
//JE 31 Mar 2020: Using Luke Miller's library for MS5803: https://github.com/millerlp/MS5803_14. Requires duplicate copy library for 2x pressure sensors on same I2C port.
#include <MS5803_14.h>  //Pressure Sensor 1 library
#include <MS5802_14_2.h> //Duplicate Pressure Sensor 2 library
#include <MPU9250.h> //IMU library from Bolder Flight Systems: https://github.com/bolderflight/MPU9250
#include "SdFat.h" //SD fat library from bill Greiman: https://github.com/greiman/SdFat
#include "elapsedMillis.h" //timing library from Paul Stoffregen (download using library manager)
#include "SDWrite.h"
#include "SDDumpfile.h"
#include "SDwipe.h"
#include "Settings.h" //local library to control variables

//building structure for radio transmission
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
  float sampling_rate_reading;
  float totaltime;
} SensorReadings;

//for feather M0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

elapsedMillis transmit_time = 0; //used to maintain regularly intervaled transmits

// Declare 2 pressure sensors

/*
  // JE 31 mar 2020: Tried sing Simon D. Levy's MS_5803 library: https://github.com/simondlevy/MS5803_14
  // Requires only one library to declare 2 sensors with explicit I2C address.
  Problem: This only works with Teensy 3.x hardware
  MS_5803 sensor(0x76);
  MS_5803 sensor2(0x77);


  /* The following uses Luke Miller's library implementation: https://github.com/millerlp/MS5803_14.
    It required a duplicate library to properly and explicilty  set two unique I2C addresses.
    Worked, but messy. In future, would be advisable to update MS_5803 to take I2C address as input arg.
*/
// Enter the oversampling value as an argument. Valid choices are
// 256, 512, 1024, 2048, 4096. Library default = 512.


// Initialize pressure sensors 1 and 2
MS_5803 sensor = MS_5803(512);
MS_5802 sensor2 = MS_5802(512); //Note renaming constructor with duplicate library.


// Initialize MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

// Boolean to keep track of initialization status
// will change to false if any of sensors, radio, or SD card fail to correctly initialize
int InitStatus = true;

//Create variables to transmit data
float pressure_top, pressure_bottom;
float myIMU_ax, myIMU_ay, myIMU_az, myIMU_gx, myIMU_gy, myIMU_gz;

//defining the size of the structure
byte buf[sizeof(SensorReadings)] = {0};

#define VBATPIN A7 //define battery PIN
float battery_voltage; //create variable

//keep track of how many bytes written to SD card
static double TotalNumBytesWrittenToSD = 0;
static int NbytesWrite;

//initializing array used to write to SD card
float PIMU[11] = {0};

//defining switch for case S
boolean WriteData = false;

//switch to stop transmission after SD dump
//boolean AfterDataDump = false;

//defining switch for case R
boolean SendRadio = true;

//defining switch case for T
boolean manual_trigger = false;

//defining switch case for Settings Update
boolean update_settings = false;

//initialize basline set up
boolean Baseline_Established = false;
int numBaseline_Readings = (Baseline_Window / Slow_Sampling_Period);
int Baseline_index = 0;
float Baseline_Value = 0;

//initialize moving average set up
int numMovingAvg = (Moving_Avg_Window / Slow_Sampling_Period);

//globals for moving array calculations
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
int MovingAvg = 0;                // the average

//other globals
float delta_P = 0;
float sleep = 0;
elapsedMillis looptime;
elapsedMillis looptime2;
float sampling_rate = 0;
elapsedMillis totaltime; //in milliseconds
elapsedMillis moving_array_time;

//globals for radio stuff
elapsedMillis radio_time = radio_off_time;
elapsedMillis radio_open = radio_on_time;
boolean radio_on = true;
boolean radio_port_on = true;



/*
  __________________________________________________________________________________________
  __________________________________________________________________________________________
  __________________________________________________________________________________________
  __________________________________________________________________________________________
*/




void setup()
{
  SensorReadings.pressure_top = 0;
  SensorReadings.pressure_bottom = 0;

  pinMode(RFM95_RST, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  Serial.begin(9600);
  if (serial_on) {
    while (!Serial) {}; //this line will block the rest of the function if serial port is not connected
    Serial.println();
    Serial.println("SmartRock Code (date ver: 09-04-2020)\n\n");
    Serial.println("Feather LoRa TX Test!");
  }

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);


  // ------ Initialize Radio ------------
  while (!rf95.init()) {
    InitStatus = false;
    if (serial_on) {
      Serial.println("LoRa radio init failed");
    }
  }

  // if we get out of while loop above, rf95 radio initialized successfully
  InitStatus = true;
  if (serial_on) {
    Serial.println("LoRa radio init OK.");
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    InitStatus = false;
    if (serial_on) {
      Serial.println("setFrequency failed");
    }
  }

  else { //radio frequency set ok.

    // Now set transmitter power from 5 to 23 dBm:
    rf95.setTxPower(radio_transmit_power, false);

    if (serial_on) {
      Serial.print("Radio frequency set to (MHz): "); Serial.println(RF95_FREQ);
      Serial.print("Tx power set to (dBm): "); Serial.println(radio_transmit_power);
    }

  }


  // ----- Initialize the pressure sensors ---------
  if (sensor.initializeMS_5803(false)) {
    if (serial_on) {
      Serial.println( "Top Sensor check OK." );
    }
  }
  else {
    InitStatus = false;
    if (serial_on) {
      Serial.println( "Top Sensor check FAILED!" );
    }
  }
  delay(300);

  if (sensor2.initializeMS_5802(false)) {
    if (serial_on) {
      Serial.println( "Botom Sensor check OK." );
    }
  }
  else {
    InitStatus = false;
    if (serial_on) {
      Serial.println( "Bottom Sensor check FAILED!" );
    }
  }
  delay(300);




  // ---- Initialize  IMU ----

  if (serial_on) {
    Serial.println("Initializing IMU, takes 5 seconds");
  }

  int IMUstatus = IMU.begin();

  if (IMUstatus < 0) { // IMU initialization failed case
    InitStatus = false;
    if (serial_on) {
      Serial.println();
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status/erorr code: ");      Serial.println(IMUstatus);
    }
  }
  else { //IMU initialized ok, now set ranges
    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);
    if (serial_on) {
      Serial.println();
      Serial.println("IMU initialization successful, initializing SD card");
      Serial.println();
    }

  }


  // ----- Initialize SD card (added JE: 10 Apr 2020) --------------
  //Initialize an SdFat object. begin () Initializes the SD card, SD volume, and root directory.
  // https://www.if.ufrj.br/~pef/producao_academica/artigos/audiotermometro/audiotermometro-I/bibliotecas/SdFat/Doc/html/class_sd_fat.html#a044bc27ba8b195519a56481484fca78b


  SDcardInitStatus = initSDcard();   // initialize SDfat volume
  if (!SDcardInitStatus) {
    InitStatus = false;
    //other informative messages printed from within initSDcard();
  }


  if (!InitStatus) { //something has gone wrong during initialization, spin wheels here
    if (Halt_On_Failed_Init) { //JE 23 Apr 2020,
      Serial.println(F("SmartRock Initialization failed.  SmartRock operation halted   :( \n"));
      Serial.println(F("Check status/error messages to diagnose and try again"));

      // spin forever in this loop so that we can't proceed.
      while (1) {}
    }
    else { //bypass initialization failure, let rest of module code execute
      Serial.println(F("Initialization of some sensors/instruments failed.  Check log above.  Proceeding in 2 sec..."));
      delay(2000);
    }
  }


  else { //everything going swell, all hardware properly intialized, print informative messages about parameters used
    // -------- Print informative serial messages --------------
    if (serial_on) {
      Serial.println(); Serial.println("__________User Defined Values___________");
      Serial.print("Baseline Window Duration: "); Serial.print(Baseline_Length); Serial.println(" minutes");
      if (numBaseline_Readings > sizeof(Baseline_Readings) / 4) { // Safety check to make sure array size is large enough. LS May 05 2020
        ArrayIssue(); // goes to settings to ajust array size so that the user can continue. LS May 05 2020
      }
      Serial.print("Moving Average Window Duration: "); Serial.print(Moving_Average_Length); Serial.println(" minutes");
      if (numMovingAvg > sizeof(MovingAvg_Readings) / 4) {
        ArrayIssue();
      }
      Serial.print("Slow Sampling Period: "); Serial.print(Sample_Period_Slow); Serial.println(" seconds");
      Serial.print("Fast Sampling Period: "); Serial.print(Sample_Period_Fast); Serial.println(" seconds");
      Serial.print("Trigger Threshold: ");  Serial.print(P_thresh); Serial.println(" millibars");
      Serial.print("Radio Sleep Interval: "); Serial.print(radio_off_time_seconds); Serial.println(" seconds");
      Serial.print("Radio Receive Interval: "); Serial.print(radio_on_time_seconds); Serial.println(" seconds");
      Serial.print("Radio Transmit Power: ");  Serial.print(radio_transmit_power);
      Serial.println(); Serial.println();
      Serial.print("Set up complete. Entering Loop in 3 ");
      delay(1000);
      Serial.print("2 ");
      delay(1000);
      Serial.print("1");
      delay(1000);
      Serial.println(); Serial.println();
    }

    //reset time stamps before beginning loop
    totaltime = 0;
    looptime = 0;
  }
}

int16_t packetnum = 0;  // packet counter, we increment per xmission


/*
  __________________________________________________________________________________________
  __________________________________________________________________________________________
  __________________________________________________________________________________________
  __________________________________________________________________________________________
*/


void loop() {

  // --------- Serial input code ------------//

  if (Serial.available()) {

    char inChar = Serial.read();
    if (serial_on) {
      Serial.println();
      Serial.print("I received character  "); Serial.println(inChar);
    }

    delay(100);
    serialFlush(); //make sure serial buffer is flushed so we detect next user input properly

    // Enumerate all possible modular actions here.  User send serial character to trigger these events.
    switch (inChar) {

      case 'W':  //Start writing data to SD card
        if (serial_on) {
          Serial.println();
          Serial.println("Started writing to SD");
        }
        WriteData = true; // JE 31 Mar 2020, make sure to enable data writing with 'W' command
        //  parallels the code for radio commands.
        break;

      // JE 10 Apr 2020.  This option now only configures log file.  SD card initialization occurs in setup

      case 'C': // configure file name and preallocate SD file to write
        //SDcardInitStatus = initSDcard();   // initialize SDfat volume

        // JE updated 23 Apr 2020 - this occurs in SDcardCreateLogFile() function.

        SDcardFileOpenStatus = SDcardCreateLogFile();
        serialFlush();
        if (SDcardFileOpenStatus && serial_on) {
          Serial.println();
          Serial.println(F("SD log file successfuly configured"));
        }

        if (!SDcardFileOpenStatus) { //error encountered preallocating file, warn user,
          Serial.println();
          Serial.println(F("WARNING: SD log file failed to configure."));
          if (Halt_On_Failed_Init) { //JE 23 Apr 2020, decide whether to spin in loop forever if error encountered.
            Serial.println(F("ERROR: SmartRock module halting operation.  Please try again to properly format SD card and configure log file."));
            while (1) {}; // spin in the loop o' death (better than giving false sense that data is being collected
          }
          else {
            Serial.println(F("WARNING: No data will be written to SD card. Please try again.  You may need to properly configure file and/or format SD card"));
          }
        }
        break;

      case 'F': //close SD log file, JE 07 Dec 2018
        serial_on = true;
        FinishLogFile(TotalNumBytesWrittenToSD);
        if (serial_on) {
          Serial.println();
          Serial.println("Finished logging to SD");
        }
        break;

      case 'D': //dumps SD over the serial port
        if (serial_on) {
          Serial.println("Start SD Card data dump over serial port");
        }
        DumpSDcardfile();
        break;

      case 'E': // erase ('wipe') add data files from SD card, add 23 Apr 2020 JE
        if (serial_on) {
          Serial.println("User requested erasing all files on SD card. ");
        }
        //serialFlush(); //need to
        wipeSDcard();

        break;

      case 'R': // turn radio on
        SendRadio = true;
        radio_port_on = true;
        serial_on = true; // turns back on serial monitor
        if (serial_on) {
          Serial.println();
          Serial.println("Radio turned on");
        }
        Pulse(2, 100, 100);
        delay(300);
        Pulse(1, 700, 0);
        break;

      case 'O': // turn radio off
        radio_time = 0; //reset radio time to zero so it will turn off again
        radio_on = false; //turns off the radio
        radio_port_on = false;
        SendRadio = false;
        if (serial_on) {
          Serial.println();
          Serial.println("Radio turned off");
        }
        Pulse(1, 700, 300);
        Pulse(2, 100, 100);
        break;

      case 'B': // call battery voltage
        battery_voltage = analogRead(VBATPIN);
        battery_voltage *= 2;    // we divided by 2, so multiply back
        battery_voltage *= 3.3;  // Multiply by 3.3V, our reference voltage
        battery_voltage /= 1024; // convert to voltage
        if (serial_on); {
          Serial.print("Battery Voltage: " );
          Serial.println(battery_voltage);
          delay(3000);
        }
        break;

      case 'G': // Go Signal
        radio_time = 0; //reset radio time to zero so it will turn off again
        radio_on = false; //turns off the radio
        radio_port_on = false;
        SendRadio = false;
        WriteData = true;
        Baseline_Established = false; //resets baseline to begin seting baseline
        Baseline_index = 0;
        if (serial_on); {
          Serial.println();
          Serial.println("Radio turned off");
          Serial.println();
          Serial.println("Started logging to SD");
          Serial.println("Baseline Reset");
          Serial.println("Serial print turning off...");
          Serial.println();
        }
        serial_on = false; //turns off serial monitor
        //WriteData = true; // Duplicate with above line ~390
        Pulse(3, 200, 800);
        Pulse(1, 1000, 0);
        test_LED = false;
        break;

      case 'Z': //manual trigger for fast
        manual_trigger = true;
        if (serial_on); {
          Serial.println();
          Serial.println("Fast Sampling Rate");
        }
        Pulse(6, 50, 50);
        break;

      case 'Y': //manual trigger for slow
        manual_trigger = false;
        if (serial_on); {
          Serial.println();
          Serial.println("Slow Sampling Rate");
        }
        Pulse(2, 1000, 1000);
        break;

      case 'M': //toggles serial monitor prints
        if (serial_on) {
          Serial.println("Serial Monitor prints turning off");
          Serial.println();
          serial_on = false;
          digitalWrite(LED_BUILTIN, LOW);
        }
        else {
          serial_on = true;
          Serial.println("Serial Monitor prints turned on");
          Serial.println();
        }
        break;

      case 'L':
        if (test_LED) {
          if (serial_on) {
            Serial.println("Test LED turning off");
            Serial.println();
          }
          test_LED = false;
        }
        else {
          test_LED = true;
          if (serial_on) {
            Serial.println("Test LED turned on");
            Serial.println();
          }
        }
        break;


      case 'S': // Update User Settings
        UpdateSettings();
        if (serial_on) {
          Serial.println();
        }
        break;


      case '?': //Explain the possbile commands
        if (serial_on) {
          Serial.println();
          Serial.println("Help menu:");
          Serial.println("G = Go Signal - start writting to SD card, reset baseline, turn off radio, turn off serial port, turn off LED");
          Serial.println("C = Configure Log File for SD");
          Serial.println("W = Start writing to SD");
          Serial.println("F = Finish logging to SD");
          Serial.println("D = Dump SD file");
          Serial.println("R = Turn radio on");
          Serial.println("O = Turn radio off");
          Serial.println("B = View Battery voltage");
          Serial.println("M = Turn toggle on/off Serial Monitor");
          Serial.println("Z = Manual Fast Sampling Rate");
          Serial.println("Y = Manual Slow Sampling Rate");
          Serial.println("L = Turn on/off Test LED");
          Serial.println("S = Update User Settings");
          Serial.println("E = Erase (wipe) all files from SD card");
          Serial.println();
          delay(8000);
        }
        break;

      default:
        if (serial_on) {
          Serial.println("Sorry, this command not recognized. Use characters: G, C, W, F, D, R, O, B, M, Z, Y, L, S or E. For help, enter: ? ");
          delay(1000);
        }
        break;
    }
  }


  //____________________________________________________________________________________________
  //checks if user sent any radio input

  if (radio_time >= radio_off_time) { //enough time has elapsed for the radio to re-open
    radio_on = true;
    radio_open = 0;
    radio_time = 0;

  }
  if (!radio_port_on) { // keeps radio from turning off when connected with user
    if (radio_open >= radio_on_time) { //the open interval time has elapsed
      radio_on = false;
    }
  }
  if (radio_on) {
    rf95.available();

    if (rf95.waitAvailableTimeout(500)) {

      // Should be a message for us now
      uint8_t buf2[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len2 = sizeof(buf2);

      if (rf95.recv(buf2, &len2))
      {
        if (serial_on) {
          Serial.println();
          Serial.print("Got: ");
          Serial.println(buf2[0]);
        }

      }
      // Enumerate all possible modular actions here.  User send serial character to trigger these events.
      switch (buf2[0]) {

        case 'W':  //Start Writing!!
          if (serial_on) {
            Serial.println();
            Serial.println("Started writing to SD");
          }
          WriteData = true;
          break;

        case 'F': //close SD log file, JE 07 Dec 2018
          serial_on = true;
          FinishLogFile(TotalNumBytesWrittenToSD);
          if (serial_on) {
            Serial.println();
            Serial.println("Finished logging to SD");
          }
          break;

        case 'R': // turn radio on
          SendRadio = true;
          radio_port_on = true;
          //serial_on = true; // turns back on serial monitor, LS 4.12.20 commented out so that serial port isn't on while radio is in the water
          if (serial_on) {
            Serial.println();
            Serial.println("Radio turned on");
          }
          Pulse(2, 100, 100);
          delay(300);
          Pulse(1, 700, 0);
          if (test_LED) {
            digitalWrite(LED_BUILTIN, HIGH);
          }
          break;

        case 'O': // turn radio off
          Pulse(1, 700, 300);
          Pulse(2, 100, 100);
          radio_time = 0; //reset radio time to zero so it will turn off again
          radio_on = false; //turns off the radio
          radio_port_on = false; //
          SendRadio = false;
          if (serial_on) {
            Serial.println();
            Serial.println("Radio turned off");
          }
          serial_on = false;
          break;

        case 'B': // call battery voltage
          battery_voltage = analogRead(VBATPIN);
          battery_voltage *= 2;    // we divided by 2, so multiply back
          battery_voltage *= 3.3;  // Multiply by 3.3V, our reference voltage
          battery_voltage /= 1024; // convert to voltage
          if (serial_on); {
            Serial.print("Battery Voltage: " );
            Serial.println(battery_voltage);
          }
          break;

        case 'G': // Go Signal
          radio_time = 0; //reset radio time to zero so it will turn off again
          radio_on = false; //turns off the radio
          radio_port_on = false; //
          SendRadio = false;
          WriteData = true;
          Baseline_Established = false;
          Baseline_index = 0;
          if (serial_on) {
            Serial.println();
            Serial.println("Radio turned off");
            Serial.println("Started logging to SD");
            Serial.println("Baseline Reset");
            Serial.println("Serial print turning off...");
            Serial.println();
          }
          serial_on = false;
          delay(200);
          Pulse(3, 200, 800);
          Pulse(1, 1000, 0);
          test_LED = false;
          break;

        case 'Z': //manual trigger for fast
          manual_trigger = true;
          if (serial_on) {
            Serial.println();
            Serial.println("Fast Sampling Rate");
          }
          Pulse(6, 50, 50);
          serial_on = false;
          break;

        case 'Y': //manual trigger for slow
          manual_trigger = false;
          if (serial_on) {
            Serial.println();
            Serial.println("Slow Sampling Rate");
          }
          Pulse(2, 1000, 1000);
          break;

        case 'L':
          if (test_LED) {
            if (serial_on) {
              Serial.println("Test LED turning off");
              Serial.println();
            }
            test_LED = false;
          }
          else {
            test_LED = true;
            if (serial_on) {
              Serial.println("Test LED turned on");
              Serial.println();
            }
          }
          break;

        default:
          if (serial_on) {
            Serial.println("Sorry, this command not recognized. Use characters: G, C, W, F, D, R, O, B, M, Z, Y, L, or S. For help, enter: ? ");
          }
          break;
      }
    }
  }


  else { //if radio_on == false
    if (serial_on) {
      //Serial.println("Radio Turned Off");
      Serial.println();
    }
    rf95.sleep();

  }


  //____________________________________________________________________________________________

  //Use readSensor() function to get pressure readings.
  sensor.readSensor();
  sensor2.readSensor();

  //define the pressure readings as the appropiate variables
  pressure_top = sensor.pressure();
  pressure_bottom = sensor2.pressure();

  battery_voltage = analogRead(VBATPIN);
  battery_voltage *= 2;    // we divided by 2, so multiply back
  battery_voltage *= 3.3;  // Multiply by 3.3V, our reference voltage
  battery_voltage /= 1024; // convert to voltage

  if (transmit_time >= 1000) {
    // Wait 1 second between transmits,
    //define all variables within the structure to send over radio
    if (SendRadio == true) {
      SensorReadings.pressure_top = pressure_top;
      SensorReadings.pressure_bottom = pressure_bottom;
      SensorReadings.myIMU_ax = myIMU_ax;
      SensorReadings.myIMU_ay = myIMU_ay;
      SensorReadings.myIMU_az = myIMU_az;
      SensorReadings.myIMU_gx = myIMU_gx;
      SensorReadings.myIMU_gy = myIMU_gy;
      SensorReadings.myIMU_gz = myIMU_gz;
      SensorReadings.battery_voltage = battery_voltage;
      SensorReadings.sampling_rate_reading = sampling_rate;
      SensorReadings.totaltime = totaltime;
      byte zize = sizeof(SensorReadings);
      memcpy (buf, &SensorReadings, zize);

      if (serial_on) {
        Serial.println(" ");
        //Serial.println("Transmitting..."); // Send a message to rf95_server
        Serial.println("Sending...");
      }
      rf95.send(buf, zize);
      if (serial_on) {
        Serial.println("Waiting for packet to complete...");
        Serial.println();
      }
      rf95.waitPacketSent();
      transmit_time = 0; //reset to zero
    }
  }

  //read the accelerometer readings
  IMU.readSensor();

  //define the IMU readings as the appropiate variables
  myIMU_ax = IMU.getAccelX_mss(), 6;
  myIMU_ay = IMU.getAccelY_mss(), 6;
  myIMU_az = IMU.getAccelZ_mss(), 6;
  myIMU_gx = IMU.getGyroX_rads(), 6;
  myIMU_gy = IMU.getGyroY_rads(), 6;
  myIMU_gz = IMU.getGyroZ_rads(), 6;


  packetnum++;

  if (serial_on) {
    // Show pressure1
    Serial.print("Pressure Top = "); Serial.print(pressure_top);
    Serial.println(" mbar");

    // Show pressure2
    Serial.print("Pressure Bottom = "); Serial.print(pressure_bottom);
    Serial.println(" mbar");
    Serial.println();

    // Print acceleration values in gs
    Serial.print("X-acceleration and gyro rate: ");
    Serial.print(myIMU_ax); Serial.print(" mg "); Serial.print("\t");
    Serial.print(myIMU_gx); Serial.println(" rad/sec ");
    Serial.print("Y-acceleration and gyro rate: ");
    Serial.print(myIMU_ay); Serial.print(" mg "); Serial.print("\t");
    Serial.print(myIMU_gy); Serial.println(" rad/sec ");
    Serial.print("Z-acceleration and gyro rate: ");
    Serial.print(myIMU_az); Serial.print(" mg "); Serial.print("\t");
    Serial.print(myIMU_gz); Serial.println(" rad/sec");
    Serial.println();

    // Print Battery Voltage
    Serial.print("Battery Voltage: "); Serial.print(battery_voltage);
    Serial.println(" Volts ");
    Serial.println();

  }



  //Timing stuff
  //____________________________________________________________________________________________

  if (manual_trigger) { //triggered manually, skips over baseline

    if (serial_on) {
      Serial.println("< < < MANUALLY TRIGGERED > > >");
    }
    if (looptime < Fast_Sampling_Period) {
      sleep = Fast_Sampling_Period - looptime;
      sampling_rate = looptime; // LS May 05 2020: Moved samping rate and looptime calculation before sleep so as to have sleep as last function during the loop. Makes it easer to see in the serial monitor
      looptime = looptime - Fast_Sampling_Period;
      if (serial_on) {
        PrintTime();
        Serial.print("Sleep: ");
        Serial.println(sleep);
      }
      delay(sleep);
    }
    else { //sampling rate is not fast enough for "fast sampling rate"
      sampling_rate = looptime; //still gets the looptime
      looptime = 0; //resets to get for each round
      if (serial_on) {
        PrintTime();
        Serial.println("********ERROR: Loop too slow for Fast Sampling Rate********");
      }
    }
  } //  manual trigger


  else { //not manual trigger

    if (!Baseline_Established) {
      if (serial_on) {
        Serial.println("ESTABLISHING BASELINE");
      }

      Baseline_Readings[Baseline_index] = pressure_top;
      if (serial_on) {
        Serial.print("Index and Value: ");
        Serial.print(Baseline_index);
        Serial.print("  ");
        Serial.println(Baseline_Readings[Baseline_index]);
      }
      Baseline_index++;


      if (Baseline_index >= numBaseline_Readings) {
        Baseline_Established = true;
        if (serial_on) {
          Serial.println();
          Serial.println();
          Serial.println();
          Serial.println("___________________________________________________________________________________________________________________________________________________");
          Serial.println("___________________________________________________________________________________________________________________________________________________");
          Serial.println("___________________________________________________________________________________________________________________________________________________");
          Serial.println("___________________________________________________________________________________________________________________________________________________");
          Serial.println("___________________________________________________________________________________________________________________________________________________");
          Serial.println("BASELINE ESTABLISHED");
        }

        //take average of Baseline Reading array from function
        Baseline_Value = averagefunction(Baseline_Readings, numBaseline_Readings);
        if (serial_on) {
          Serial.print("Baseline Pressure Value: ");
          Serial.println(Baseline_Value);
          Serial.println();
          Serial.println();
          Serial.println();
        }


        //initialize moving average array to baseline value
        for (int j = 0; j < numMovingAvg; j++) {
          MovingAvg_Readings[j] = Baseline_Value;
        }

        total = numMovingAvg * Baseline_Value; //compputes total by taking average mutiplied by number of elements

      } // closing baseline_index = numBaseline_Readings


      if (looptime < Slow_Sampling_Period) {
        sleep = Slow_Sampling_Period - looptime;
        sampling_rate = looptime;
        looptime = looptime - Slow_Sampling_Period; //set looptime to zero (or close to zero to correct itself)
        if (serial_on) {
          PrintTime();
          Serial.print("sleep for: ");
          Serial.println(sleep);
        }
        delay(sleep);
      }

    } // closing baseline established


    else { //baseline established
      if (moving_array_time >= Slow_Sampling_Period - 0.5 * Fast_Sampling_Period) { // used so that when sampling quickly, gets the closest to having the same moving average window as the slow

        //subtract last reading
        total = total - MovingAvg_Readings[readIndex];
        //  read from the sensor:
        MovingAvg_Readings[readIndex] = pressure_top;
        if (serial_on) {
          Serial.print("Moving Array Index and New Input: ");
          Serial.print(readIndex);
          Serial.print(" ");
          Serial.println(MovingAvg_Readings[readIndex]);
        }
        // add the reading to the total:
        total = total + MovingAvg_Readings[readIndex];
        // advance to the next position in the array:
        readIndex++;

        //  if we're at the end of the array...
        if (readIndex >= numMovingAvg) {
          // ...wrap around to the beginning:
          readIndex = 0;
        }//closing reset read index loop

        //calculate the moving average:
        MovingAvg = total / numMovingAvg;
        delta_P = MovingAvg - Baseline_Value;
        if (serial_on) {
          Serial.print("Average Pressure Value: ");
          Serial.print(MovingAvg); Serial.println(" mbar");
          Serial.print("Difference between Average and Baseline: ");
          Serial.print(delta_P); Serial.println(" mbar");
          Serial.println();
        }
        moving_array_time = 0;
      }


      if (delta_P > P_thresh) {
        if (serial_on) {
          Serial.println("< < < PRESSURE DIFFERENCE TRIGGERED > > >");
        }
        if (looptime < Fast_Sampling_Period) {
          sleep = Fast_Sampling_Period - looptime;
          sampling_rate = looptime;
          looptime = looptime - Fast_Sampling_Period;
          if (serial_on) {
            PrintTime();
            Serial.print("Sleep: ");
            Serial.println(sleep);
          }
          delay(sleep); //change to sleep later
        }
        else { //sampling rate is not fast enough for "fast sampling rate"
          sampling_rate = looptime; //still gets the looptime
          looptime = 0; //resets to get for each round
          if (serial_on) {
            PrintTime();
            Serial.println("********ERROR: Loop too slow for Fast Sampling Rate******");
          }
        }
      } // close trigger stuff


      else {
        if (serial_on) {
          Serial.println("_ _ _ NOT triggered _ _ _");
        }
        if (looptime < Slow_Sampling_Period) {
          sleep = Slow_Sampling_Period - looptime;
          sampling_rate = looptime;
          looptime = looptime - Slow_Sampling_Period;
          if (serial_on) {
            PrintTime();
            Serial.print("sleep for: ");
            Serial.println(sleep);
          }
          delay(sleep); //change to sleep later
        }
      } // close ~not~ triggered stuff
    } // close baseline established
  } // close ~not~ manual trigger (regular function)




  // if test LED is on, this will toggle the LED on and off based on how fast the code is running
  // LED will be on for one cycle and then off for the next cycle and repeat
  if (test_LED) {
    if (looptime2 > 50) {
      if (digitalRead(LED_BUILTIN) == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else {
        digitalWrite(LED_BUILTIN, LOW);
      }
      looptime2 = 0;
    }
  }




  if (looptime > Slow_Sampling_Period + 0.2 * Slow_Sampling_Period) { //failsafe if looptime becomes too large
    if (serial_on) {
      Serial.print("Loop too long, resetting");
      PrintTime();
    }
    looptime = 0; //reset loop time
  }

  //____________________________________________________________________________________________

  //defining array to write to the SD card
  if (WriteData) {
    //Define PIMU array
    PIMU[0] = totaltime; //in milliseconds
    PIMU[1] = pressure_top;
    PIMU[2] = pressure_bottom;
    PIMU[3] = myIMU_ax;
    PIMU[4] = myIMU_ay;
    PIMU[5] = myIMU_az;
    PIMU[6] = myIMU_gx;
    PIMU[7] = myIMU_gy;
    PIMU[8] = myIMU_gz;
    PIMU[9] = battery_voltage;
    PIMU[10] = 282828;       //Stevan's secret code

    // Write data to SD card, needs to be at the end of void loop
    if (TotalNumBytesWrittenToSD + NbytesWrite <= log_file_size) { //check to make sure there is sufficient space to write bytes
      NbytesWrite = dataFile.write((uint8_t*)PIMU, sizeof(PIMU));
      TotalNumBytesWrittenToSD += NbytesWrite;

      // JE 31 Mar 2020: for debug.  currently see no bytes being written to file.
      if (serial_on) {
        Serial.print("Bytes written this loop: ");
        Serial.println(NbytesWrite);
      }
    }
    else { //no more space
      FinishLogFile(TotalNumBytesWrittenToSD);
    }
  }
  else {
    //do nothing
  }
  if (serial_on) {
    Serial.println();
    Serial.println();
    Serial.println("_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _");
  }
} //closing void loop

/*
  __________________________________________________________________________________________
  __________________________________________________________________________________________
  __________________________________________________________________________________________
  __________________________________________________________________________________________
*/

//Functions

//take average
float averagefunction (float * array, int len)  // assuming array is int.
{
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}


// Subroutine FinishLogFile(), closes SD card data file, blinks LED to let user know logging is finished

void FinishLogFile(double TotalBytesWritten) {
  dataFile.truncate(TotalBytesWritten);
  dataFile.close();
  Serial.print("Finishing Log File with Total Bytes Written to SD = ");
  Serial.println(TotalBytesWritten);

}


// Subroutine to print timing. allows to print timing before sleep function LS May 05 2020
void PrintTime() {
  Serial.println();
  Serial.print("Time Between Samples: "); Serial.print(sampling_rate); Serial.println("ms");
  Serial.print("Total Time: "); Serial.print(totaltime); Serial.print("ms  or  "); Serial.print(totaltime / 3600000); Serial.println("hrs");
  Serial.println();
}

// Subroutine Pulse, activates green LED to give user feedback

void Pulse(long num, long dur_on, long dur_off) {
  for (int i = 1; i <= num; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(dur_on);
    digitalWrite(LED_BUILTIN, LOW);
    delay(dur_off);
  }
}


// --------------------
//    Subroutine: FLUSH SERIAL INPUT--Little helper function to help control user input, not INTAN related per se
// ----------------

void serialFlush() {
  while (Serial.available() > 0) {
    char chuckaway = Serial.read();
    //Serial.print("Throwing Away: ");
    //Serial.println(chuckaway);
    delay(100);
  }
}

void UpdateSettings() {

  update_settings = true;

  while (update_settings == true) {

    char problem2 = Serial.read();

    serialFlush();
    Serial.println();
    Serial.println();
    Serial.println("Current Settings, [enter character] to update");
    Serial.println();
    Serial.print("B = Baseline Duration: "); Serial.print(Baseline_Length); Serial.println(" minutes");
    Serial.print("M = Moving Average Duration: "); Serial.print(Moving_Average_Length); Serial.println(" minutes");
    Serial.print("S = Slow Sampling Period: "); Serial.print(Sample_Period_Slow); Serial.println(" seconds");
    Serial.print("F = Fast Sampling Period: "); Serial.print(Sample_Period_Fast); Serial.println(" seconds");
    Serial.print("T = Threshold Pressure: "); Serial.print(P_thresh); Serial.println(" millibar");
    Serial.print("O = Radio Sleep Time: "); Serial.print(radio_off_time_seconds); Serial.println(" seconds");
    Serial.print("R = Radio On Window: "); Serial.print(radio_on_time_seconds); Serial.println(" seconds");
    Serial.print("P = Radio Transmit Power: "); Serial.println(radio_transmit_power);
    Serial.println("E = Exit Settings");



    while (!Serial.available()) { //waits for user input
      delay(100);
    }

    char inSetting = Serial.read();
    Serial.println();
    Serial.print("I received character  "); Serial.println(inSetting);

    delay(100);
    serialFlush(); //make sure serial buffer is flushed so we detect next user input properly

    // Enumerate all possible modular actions here.  User send serial character to trigger these events.
    switch (inSetting) {

      case 'B': { //Change the Baseline Duration
          UpdateBaseline(); // changed to void loop so that it can be used if issue with slow sampling as well LS May 05 2020

          if (numBaseline_Readings > sizeof(Baseline_Readings) / 4) { // Safety check to make sure array size is large enough. LS May 05 2020
            ArrayIssue(); // moved to void loop so that if slow sampling is changed, will check to make sure not an issue for moving average as well. LS May 05 2020
          }

          Baseline_index = 0;
          Baseline_Value = 0;
          Baseline_Established = false;

          Serial.println("Other dependent variables updated");
          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'M': { //Change the Moving Average Duration
          UpdateMovingAvg();

          if (numMovingAvg > sizeof(MovingAvg_Readings) / 4) {
            ArrayIssue();
          }

          readIndex = 0;
          total = 0;
          MovingAvg = 0;

          Serial.println("Other dependent variables updated");
          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'S': { //Change the Slow Sampling Period
          UpdateSlowSampling();

          if (numBaseline_Readings > sizeof(Baseline_Readings) / 4 || numMovingAvg > sizeof(MovingAvg_Readings) / 4) {
            ArrayIssue();
          }

          Baseline_index = 0;
          Baseline_Value = 0;
          readIndex = 0;
          total = 0;
          MovingAvg = 0;

          Serial.println("Other dependent variables updated");
          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'F': { //Change the Fast Sampling Period
          Serial.println();
          Serial.print("Current Fast Sampling Period: "); Serial.print(Sample_Period_Fast); Serial.println(" seconds");
          Serial.println("Change Fast Sampling Period to [enter value]");
          Serial.println();
          while (!Serial.available()) { //waits for user input
            delay(100);
          }
          Sample_Period_Fast = Serial.parseInt();
          Serial.print("Fast Sampling Period updated to "); Serial.print(Sample_Period_Fast); Serial.println(" seconds");

          Fast_Sampling_Period = Sample_Period_Fast * 1000;

          Serial.println("Other dependent variables updated");
          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'T': { //Change the Threshold Pressure
          Serial.println();
          Serial.print("Current Threshold Pressure: "); Serial.print(P_thresh); Serial.println(" millibar");
          Serial.println("Change Threshold Pressure to [enter value]");
          Serial.println();
          while (!Serial.available()) { //waits for user input
            delay(100);
          }
          P_thresh = Serial.parseInt();
          Serial.print("Threshold Pressure updated to "); Serial.print(P_thresh); Serial.println(" millibar");

          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'O': { //Change the Radio Sleep Time
          Serial.println();
          Serial.print("Current Radio Sleep Time: "); Serial.print(radio_off_time_seconds); Serial.println(" seconds");
          Serial.println("Change Radio Sleep Time to [enter value]");
          Serial.println();
          while (!Serial.available()) { //waits for user input
            delay(100);
          }
          radio_off_time_seconds = Serial.parseInt();
          Serial.print("Radio Sleep Time updated to "); Serial.print(radio_off_time_seconds); Serial.println(" seconds");

          radio_off_time = radio_off_time_seconds * 1000;
          radio_time = radio_off_time;

          Serial.println("Other dependent variables updated");
          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'R': { //Change the Radio On Window
          Serial.println();
          Serial.print("Current Radio On Window: "); Serial.print(radio_on_time_seconds); Serial.println(" seconds");
          Serial.println("Change Radio On Window to [enter value]");
          Serial.println();
          while (!Serial.available()) { //waits for user input
            delay(100);
          }
          radio_on_time_seconds = Serial.parseInt();
          Serial.print("Radio On Window updated to "); Serial.print(radio_on_time_seconds); Serial.println(" seconds");

          radio_on_time = radio_on_time_seconds * 1000;
          radio_open = radio_on_time;

          Serial.println("Other dependent variables updated");
          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'P': { //Change the Radio Transmit Power
          Serial.println();
          Serial.print("Current Radio Transmit Power: "); Serial.println(radio_transmit_power);
          Serial.println("Change Radio Transmit Power to [enter value]");
          Serial.println();
          while (!Serial.available()) { //waits for user input
            delay(100);
          }
          radio_transmit_power = Serial.parseInt();
          Serial.print("Radio Transmit Power updated to "); Serial.println(radio_transmit_power);

          rf95.setTxPower(radio_transmit_power, false);

          Serial.println("Other dependent variables updated");
          Serial.println();
          serialFlush();
          delay(1000);
        }
        break;

      case 'E': //Exit Settings
        Serial.println();
        update_settings = false;
        Serial.println("Exiting Settings");
        Serial.println();
        break;

      case '?': //Explain the possbile commands
        Serial.println();
        Serial.println("Help menu:");
        Serial.println("B = Update the Baseline Duration");
        Serial.println("M = Update the Moving Average Duration");
        Serial.println("S = Update the Slow Sampling Period");
        Serial.println("F = Update the Fast Sampling Period");
        Serial.println("T = Update the Threshold Pressure");
        Serial.println("O = Update the Radio Sleep Time");
        Serial.println("R = Update the Radio On Window");
        Serial.println("P = Update the Radio Transmit Power");
        Serial.println("E = Exit Settings");
        Serial.println();
        break;

      default:
        Serial.println("Sorry, this command not recognized. Use characters: B, M, S, F, T, O, R, P, or E. For help, enter: ? ");
        Serial.println();

        break;
    }
  }
}

void UpdateBaseline() {// moved to void loop in order to be called on in case of array issues. LS May 05 2020
  Serial.println();
  Serial.print("Current Baseline Duration: "); Serial.print(Baseline_Length); Serial.println(" minutes");
  Serial.println("Change Baseline Duration to [enter value]");
  Serial.println();
  while (!Serial.available()) { //waits for user input
    delay(100);
  }
  Baseline_Length = Serial.parseInt();
  Serial.print("Baseline Duration updated to "); Serial.print(Baseline_Length); Serial.println(" minutes");

  Baseline_Window = Baseline_Length * 60 * 1000;
  numBaseline_Readings = (Baseline_Window / Slow_Sampling_Period);

  char Problem3 = Serial.read();
  serialFlush();
  delay(100);
}

void UpdateMovingAvg() {// moved to void loop in order to be called on in case of array issues. LS May 05 2020
  Serial.println();
  Serial.print("Current Moving Average Duration: "); Serial.print(Moving_Average_Length); Serial.println(" minutes");
  Serial.println("Change Moving Average Duration to [enter value]");
  Serial.println();
  while (!Serial.available()) { //waits for user input
    delay(100);
  }
  Moving_Average_Length = Serial.parseInt();
  Serial.print("Moving Average Duration updated to "); Serial.print(Moving_Average_Length); Serial.println(" minutes");

  Moving_Avg_Window = Moving_Average_Length * 60 * 1000;
  numMovingAvg = (Moving_Avg_Window / Slow_Sampling_Period);

  char Problem4 = Serial.read();
  serialFlush();
  delay(1000);
}

void UpdateSlowSampling() { // moved to void loop in order to be called on in case of array issues. LS May 05 2020
  Serial.println();
  Serial.print("Current Slow Sampling Period: "); Serial.print(Sample_Period_Slow); Serial.println(" seconds");
  Serial.println("Change Slow Sampling Period to [enter value]");
  Serial.println();
  while (!Serial.available()) { //waits for user input
    delay(100);
  }
  Sample_Period_Slow = Serial.parseInt();
  Serial.print("Slow Sampling Period updated to "); Serial.print(Sample_Period_Slow); Serial.println(" seconds");

  Slow_Sampling_Period = Sample_Period_Slow * 1000;
  numBaseline_Readings = (Baseline_Window / Slow_Sampling_Period);
  numMovingAvg = (Moving_Avg_Window / Slow_Sampling_Period);

  Baseline_index = 0;
  Baseline_Value = 0;
  readIndex = 0;
  total = 0;
  MovingAvg = 0;

  Serial.println("Other dependent variables updated");
  Serial.println();
  char Problem5 = Serial.read();
  serialFlush();
  delay(1000);
}


void ArrayIssue() { // solves issues of windows being too large for arrays. LS May 05 2020

  while (numBaseline_Readings > sizeof(Baseline_Readings) / 4 || numMovingAvg > sizeof(MovingAvg_Readings) / 4) { // User will be in loop until they imput a baseline that fits in the array LS May 05 2020

    Serial.println(); Serial.println(); Serial.println();
    Serial.println("**********ERROR**********");
    Serial.println();
    Serial.println("Array size insufficient for window duration. Either decrease window size or increase slow sampling period");
    Serial.println();
    if (numBaseline_Readings > sizeof(Baseline_Readings) / 4) {
      Serial.print("Baseline array size: "); Serial.println(sizeof(Baseline_Readings) / 4);
      Serial.print("Number of Baseline Samples ("); Serial.print(numBaseline_Readings); Serial.print(") = Baseline Window Duration("); Serial.print(Baseline_Length); Serial.print("min) / Slow Sampling Period("); Serial.print(Sample_Period_Slow); Serial.println("seconds)");
      Serial.println();
    }
    if (numMovingAvg > sizeof(MovingAvg_Readings) / 4) {
      Serial.print("Moving Average array size: "); Serial.println(sizeof(MovingAvg_Readings) / 4);
      Serial.print("Number of Moving Average Samples ("); Serial.print(numMovingAvg); Serial.print(") = Moving Average Window Duration("); Serial.print(Moving_Average_Length); Serial.print("min) / Slow Sampling Period("); Serial.print(Sample_Period_Slow); Serial.println("seconds)");
      Serial.println();
    }
    Serial.println("Change Baseline, Moving Average or Slow Sampling Period. Enter 'B', 'M' or 'S'");
    while (!Serial.available()) { //waits for user input
      delay(100);
    }
   
    char ArrayChoice = Serial.read();
    delay(100);
    serialFlush(); //make sure serial buffer is flushed so we detect next user input properly

    // Enumerate all possible modular actions here.  User send serial character to trigger these events.
    switch (ArrayChoice) {
      case 'B': {
          UpdateBaseline();
        }
        break;

      case 'M': {
          UpdateMovingAvg();
        }
        break;

      case 'S': {
          UpdateSlowSampling();
        }
        break;
    }
  }
  Serial.println();
  Serial.println("Windows sized appropriately, returning to code");
}
