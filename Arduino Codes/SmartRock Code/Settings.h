// Trigger definitions
/*
Help menu:
S = Start logging to SD
I = Initialize SD
F = Finish logging to SD
R = Turn radio on
O = Turn radio off
G = Go Signal - write to SD card, turn off radio, reset baseline, turn off serial port
D = Dump SD file
B = View Battery Voltage
Z = Manual Fast Sampling Rate
Y = Manual Slow Sampling Rate
M = Turn on/off Serial Monitor
*/


// unit: mintues
const double Baseline_Length = 5;         // input baseline window length in minutes
                                          // how much time you want the baseline to cover

const double Moving_Average_Length = 2;   // input moving average window length in minutes
                                          // how much time you want the moving average window to cover

// unit: seconds                               
const long Sample_Period_Slow = 1;        // input slow sampling period in seconds
                                          // how often you want to take samples when not triggered
                                    
const double Sample_Period_Fast = 0.01;   // input fast sampling period in seconds
                                          // how often you want to take samples when triggered

// unit: millibar
int P_thresh = 15;                        // input threshold pressure in millibars
                                          // threshold calculated by the river rising 15 cm and using p = rho*g*deltah
                                         

// radio settings _________________________________________________________________________________________________________________

float radio_off_time_seconds = 20;       // input radio sleep time in seconds
                                         // more time = more power saved, but also longer waiting time to "catch" signal

long radio_on_time_seconds = 2;          // input radio-on window to catch signal in seconds
                                         // less time = more power saved, but also lower chance of "catching" signal 
                            
int radio_transmit_power = 5;            // power scale for transmit. Can range from 5 to 23
                                         // higher means more current draw and decreased battery life

// testing features ___________________________________________________________________________________________________________________

boolean serial_on = true;                 // set to "true" if working with code. "Go" switch will automatically turn off serial monitor
boolean test_LED = true;                  // set to "true" if testing LED. "Go" switch will automatically turn of test LED when running code


//Conversions _____________________________________________________________________________________________________________________
//converts user defined values into units needed for rest of code, no need to adjust

const float Baseline_Window = Baseline_Length * 60 *1000;       // conversion from minutes to milliseconds
                                                               
const long Slow_Sampling_Period = Sample_Period_Slow * 1000;    // conversion from seconds to milliseconds
                                       
const long Fast_Sampling_Period = Sample_Period_Fast * 1000;    // conversion from seconds to milliseconds
                                      

const float Moving_Avg_Window = Moving_Average_Length *60*1000; // conversion from minutes to milliseconds

float radio_off_time = radio_off_time_seconds * 1000;           // conversion from seconds to milliseconds

float radio_on_time = radio_on_time_seconds *1000;              // conversion from seconds to milliseconds
