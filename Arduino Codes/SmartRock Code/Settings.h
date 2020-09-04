// Trigger definitions
/*
Help menu:
 G = Go Signal - start writting to SD card, reset baseline, turn off radio, turn off serial port, turn off LED
 C = Configure Log File for SD
 W = Start writing to SD
 F = Finish logging to SD
 D = Dump SD file
 R = Turn radio on
 O = Turn radio off
 B = View Battery voltage
 M = Turn toggle on/off Serial Monitor
 Z = Manual Fast Sampling Rate
 Y = Manual Slow Sampling Rate
 L = Turn on/off Test LED
 S = Update User Settings
 E = Erase (wipe) all files from SD card
*/

// unit: mintues
double Baseline_Length = 5;                // input baseline window duration in minutes (maximum of 60 min ish)
                                           // how much time you want the baseline to cover
                                           
double Moving_Average_Length = 2;          // input moving average window duration in minutes (maximum of 30 min ish)
                                           // how much time you want the moving average window to cover
                                                                                   
// unit: seconds                               
long Sample_Period_Slow = 1;               // input slow sampling period in seconds (minimum of 1s ish)
                                           // how often you want to take samples when not triggered
                                    
double Sample_Period_Fast = 0.01;          // input fast sampling period in seconds (minimum of 0.01s ish)
                                           // how often you want to take samples when triggered

// unit: millibar
int P_thresh = 100;                        // input threshold pressure in millibars
                                           // threshold calculated by the river rising 100cm and using p = rho*g*deltah
                                         

// radio settings _________________________________________________________________________________________________________________

float radio_off_time_seconds = 60;        // input radio sleep time in seconds
                                          // more time = more power saved, but also longer waiting time to "catch" signal

long radio_on_time_seconds = 2;           // input radio-on window to catch signal in seconds
                                          // less time = more power saved, but also lower chance of "catching" signal 
                            
int radio_transmit_power = 5;             // power scale for transmit. Can range from 5 to 23
                                          // higher means more current draw and decreased battery life

// array declarations LS May 05 2020 __________________________________________________________________________________________________________________

float Baseline_Readings[3600];            // sets baseline max array to 3600 samples (60min baseline max / 1s slow sampling min = 3600 samples)

float MovingAvg_Readings[1800];           // sets moving average max array to 1800 samples (30min moving average max / 1s slow sampling min = 1800 samples)

// testing features ___________________________________________________________________________________________________________________

boolean serial_on = true;                 // set to "true" if working with code. "Go" switch will automatically turn off serial monitor
boolean test_LED = true;                  // set to "true" if testing LED. "Go" switch will automatically turn of test LED when running code

const boolean Halt_On_Failed_Init = true;  // boolean switch that will stop code exectuion in event that setup/initialization fails
                                           // Recommend settting to false for debugging stages (e.g only testing with 1 out of 2 pressure sensors
                                           // Set this variable to true for normal operation.

//Conversions _____________________________________________________________________________________________________________________
//converts user defined values into units needed for rest of code, no need to adjust

float Baseline_Window = Baseline_Length * 60 * 1000;             // conversion from minutes to milliseconds
                                                             
long Slow_Sampling_Period = Sample_Period_Slow * 1000;          // conversion from seconds to milliseconds
                                     
long Fast_Sampling_Period = Sample_Period_Fast * 1000;          // conversion from seconds to milliseconds
                                      
float Moving_Avg_Window = Moving_Average_Length *60*1000;       // conversion from minutes to milliseconds
      
float radio_off_time = radio_off_time_seconds * 1000;           // conversion from seconds to milliseconds

float radio_on_time = radio_on_time_seconds *1000;              // conversion from seconds to milliseconds
