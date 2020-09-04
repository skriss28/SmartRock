/*
  SD card file dump

  This example shows how to read a file from the SD card using the
  SD library and send it over the serial port.

  The circuit:
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created  22 December 2010
  by Limor Fried
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/
boolean DumpSDcardfile() {

  //initialize boolean to return
  boolean DumpSDcardfile_success = false;

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // Verify SD card is initialized. JE 10 Apr 2020, moved this up above request for file name
  if (!SDcardInitStatus ) { //SD card not initialized
    // JE 10 Apr 2020: not sure how this case can ever occur, leaving it for posterity for now
    // SD card has to be initialized before gathering data.
    // If file dump occuring after power reset, SD card always initializes in setup

    Serial.println("SD card not initialized. Retrying SD card initialize...\n");

    SDcardInitStatus = initSDcard();
    if (!SDcardInitStatus) { //failed on 2nd try, tell user file can't dump and return and return
      Serial.println("SD card could not be initialized. Data file transfer can not be completed.\n");
      Serial.println("Recommendation: carefully open smartrock module; release SD card from socket and re-insert it; then try file transfer again.");
      delay(3000);
    }
  }


  else { // SD memory card previously initiailzed
    Serial.println("SD Card previously initialized. Ready to proceed with file dump \n");
  }


  // Request file suffix and open a file
  const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  const uint8_t FILE_NAME_DIM  = BASE_NAME_SIZE + 11;
  char binName[FILE_NAME_DIM] = FILE_BASE_NAME "YYMMDD.txt";
  char dname[FILE_NAME_DIM];
  strcpy(dname, binName);

  // JE 23 Apr 2020, updating to print files on card and take in 6-digit file extension
  Serial.println(F("\n Enter 6 digit file suffix YYMMDD (srYYMMDD.txt)to retrieve. \n\n "));
  Serial.println(" ----- List of files on SD card ---------- ");
  Serial.println("       Size, Name ");
  sdEx.ls(&Serial, LS_SIZE);

 //  Sequence of characters printed at prompting for file name input. Added: JE 24 Apr 2020
 // Purpose is to help stay synced with matlab dump file.
 // Should have no practical effect otherwise. 
  Serial.write(0x02); 
  Serial.write(0x02); 
  Serial.write(0x02);

  for (int i = 0; i < 6; i++) {
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
    dname[BASE_NAME_SIZE + i] = c;
  }

  serialFlush();

  Serial.print(" SDcard file to be dumped: ");
  Serial.println(dname);
  Serial.println();



  // 10 Apr 2020, possible issue here with file access.  Could be trouble if file wasn't previously closed properly
  // First check if data file is open, if not open it for read only
  if (dataFile.isOpen()) {
    //first close an open file, then we can reopen it below
    dataFile.close();
    Serial.println("Data file to be dumped was still open.  Closed it before re-opening and reading.\n");
  }


  File dumpFile = sdEx.open(dname, O_READ);



  int n_bytes_dumped = 0;
  Serial.write(0x04); //Start marker for the dumped bytes
  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x04);

  // if the file is available, read from it:
  if (dumpFile) {
    while (dumpFile.available()) {
      Serial.write(dumpFile.read());
      n_bytes_dumped++;
    }
    Serial.write(0x03); //End marker for the dumped bytes
    Serial.write(0x03);
    Serial.write(0x03);
    Serial.write(0x03);
    // serial_on = false;
    if (serial_on) { //JE 31 mar 2020: we'll never see this since serial is turned off in line above.
      Serial.println();
      Serial.print("Bytes dumped: ");
      Serial.println(n_bytes_dumped);
    }
    dumpFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening and retrieving datafile");
    Serial.println("\t Possible issue 1: SD card not initialized. Use command 'I' to initialize");
    Serial.println("\t Possible issue 2: File requested not available. Check file requested is the right one.");
    delay(3000);
  }
}
