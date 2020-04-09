// ----------------------------------------
// Initialize SD card volume and open file
// Get SD card file name from serial input
// Filename is restricted to 64 characters max
//  11 July 2018, JE
// ---------------------------------------




boolean initSDcard() {

  boolean SDcardInit_success = false;
  //Serial.println("Iniatalize SD card...");

/* JE 31 Mar 2020: working on bug fix for sd init on adafruit feather m0 lora.
*  Issue may be in setting appropriate SPI bus, e.g. code that works in tests: 
*  if (!sd.cardBegin(SD_CHIP_SELECT, SD_SCK_MHZ(SD_SPI_CLK_MAX))) 
 SDfat documentation here:https://www.if.ufrj.br/~pef/producao_academica/artigos/audiotermometro/audiotermometro-I/bibliotecas/SdFat/Doc/html/class_sd_fat.html#a9a4deb665fb0140daf22c62de3ae497d

 bool SdFat::begin ( uint8_t   chipSelectPin = SD_CHIP_SELECT_PIN,
                     uint8_t   sckRateID = SPI_FULL_SPEED 
                    ) 
  */
// SPI Speed options: _FULL, _HALF, _QUARTER, _SIXTEENTH
  if (!sdEx.begin(chipSelect, SPI_HALF_SPEED)) {
  //if (!sdEx.begin()) {
    Serial.println("SdFatSdioEX begin() failed. Check SD card is present in microSD slot.");
    sdEx.initErrorPrint();
  }
  else { // SD memory card initiailzed; now open a file
    sdEx.chvol(); // make sdEx the current volume.
    SDcardInit_success = true;

    Serial.println("Finished Initializing");
  }

  return SDcardInit_success;
}




/* ----------------------------------------------------------------------------------
    Create pre-allocated, pre-erased file. Should drastically reduce SD write latency
    Block of code adapted from:
    https://github.com/tni/teensy-samples/blob/master/SdFatSDIO_low_latency_logger.ino

  ---------------------------------------------------------------------------------------*/

boolean SDcardCreateLogFile() { // added JE 2018 Nov 20, creating a contiguous, pre-erased file to minimize logging latency


  boolean SDcardOpenFile_success = false;


  // Set file name suffix.  Base file name is "Intsy_"
  // filename for file.createcontigous for SD must be DOS 8.3 format:
  //  http://www.if.ufrj.br/~pef/producao_academica/artigos/audiotermometro/audiotermometro-I/bibliotecas/SdFat/Doc/html/class_sd_base_file.html#ad14a78d348219d6ce096582b6ed74526

  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  const uint8_t FILE_NAME_DIM  = BASE_NAME_SIZE + 7;
  char binName[FILE_NAME_DIM] = FILE_BASE_NAME "00.txt";
  char fname[FILE_NAME_DIM];
  strcpy(fname, binName);
  //Serial.println("Input SD card filename extension:");
  Serial.println(F("\nEnter two digit file suffix xx (Rock_xx.txt)"));
  //Serial.write(name, BASE_NAME_SIZE);

  for (int i = 0; i < 2; i++) {
    while (!Serial.available()) {
      delay(100);
      //SysCall::yield();
    }
    char c = Serial.read();
    //Serial.write(c);
    /* Skip checking this for now, user could enter anything
      if (c < '0' || c > '9') {
      Serial.println(F("\nInvalid digit"));
      return;
      }
    */
    fname[BASE_NAME_SIZE + i] = c;
  }

  serialFlush();

  Serial.print("Set SDcard data file name: ");
  Serial.println(fname);
  //Serial.write(TERM_CHAR); // print termination character, to keep synced with LabView VISA R/W operations.



  //for pre-allocated, pre-erased file.  Copied from Teensy36_SDFatSDIO_LowLatencyLogger, JE 20 Nov 2018
  //uint32_t first_sector = 0;
  //uint32_t last_sector = 0;
  uint32_t next_sector = 0;


  // Create a pre-allocated, pre-erased contiguous file. Any existing file with the same name, is erased.
  // Return true on success.
  //bool create(const char* name, uint32_t size) {
  Serial.println(F("Creating file (will remove any old file by the same name): "));
  Serial.println(fname);
  //if(next_sector) close();
  //last_error = E_ok;
  sdEx.remove(fname); // will remove file if already exist.  should probably warn user of this JE 13 Dec 2018

  // User input for file size
  Serial.println(F("Enter data file size (MB)"));

  // wait for user to input desired sampling rate
  while (!Serial.available()) {
    delay(100);
    // SysCall::yield();    // wait for user input
  }

  uint32_t LogFileSz_MB = Serial.parseInt();
    char problem = Serial.read();
    //Serial.print("Problem child: ");
    //Serial.println(problem);

  log_file_size = LogFileSz_MB * 1024 * 1024;

  //uint32_t log_file_size = 2ull * 1024 * 1024 * 1024;  // 2GB %should be determined by Nchans x Fs x Experiment Duration

  Serial.print(F("File Size (MB) set to: "));
  Serial.println(LogFileSz_MB);

  //serialFlush();

  // probably just have user input log_file_size directly
  if (!dataFile.createContiguous(sdEx.vwd(), fname , log_file_size)) { //requires const char* input ie: "logJE001.bin"
    Serial.println("ContigFile: createContiguous() failed");
    //last_error = E_create_contiguous;
    return false;
  }
  uint32_t first_sector, last_sector;
  if (!dataFile.contiguousRange(&first_sector, &last_sector)) {
    Serial.println("PreallocatedFile: contiguousRange() failed");
    // last_error = E_create_contiguous;
    return false;
  }
  uint32_t first_erase_sector = first_sector;
  const size_t erase_count = 64 * 1024; // number of sectors to erase at once
  while (first_erase_sector <= last_sector) {
    if (!sdEx.card()->erase(first_erase_sector, min(first_erase_sector + erase_count, last_sector))) {
      Serial.println("PreallocatedFile: erase() failed");
      //last_error = E_erase;
      return false;
    }
    first_erase_sector += erase_count;
  }
  Serial.print("First sector, Last sector: ");
  Serial.print(first_sector);
  Serial.print(",");
  Serial.println(last_sector);

  //this->first_sector = first_sector;
  //this->last_sector = last_sector;
  next_sector = first_sector;

  dataFile.flush();

  Serial.println(F("Finished pre-allocating and pre-erasing log file"));
  serialFlush();
  return true;
}
