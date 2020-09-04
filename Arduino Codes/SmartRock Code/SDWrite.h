// ------------------------------
// Configuring SD card for writes
// -----------------------------

// ------Create sd card volume  and datafile to write to
SdFat sdEx;  //  JE changed from sdio to sdioEX 20 Nov 2018, fastest implementation, requires newer/fast SD card
SdFile dataFile;   // data file instance, changed from File --> SdFile JE 10 Apr 2020


// ----------- SmartRock hardware configured for pin 10 chip select ----------
const int chipSelect = 10;



//JE: 10 Apr 2020, Deprecated way of listing files on card. Using SdFat class ls() instead
// SdFile file; //file system object, following: https://codebender.cc/example/SdFat/OpenNext#OpenNext.ino
// define a serial output stream
//ArduinoOutStream cout(Serial);


boolean SDcardInitStatus = false;
boolean SDcardFileOpenStatus = false;

// file size to be logged. when full, logging with automatically stop
uint32_t log_file_size;

//for flushing buffered SD to FAT
elapsedMillis flushSDElapsedTime;
const int SD_FLUSH_TIMER_MS = 60*1000; // data will be flushed every 10 s (worst case we lose last 10 s of data)

// for defining file name
#define FILE_BASE_NAME "sr"

// Define function prototype to get user settings 
extern boolean initSDcard();
extern boolean SDcardCreateLogFile(); // added JE 2018 Nov 20, creating a contiguous, pre-erased file to minimize logging latency
