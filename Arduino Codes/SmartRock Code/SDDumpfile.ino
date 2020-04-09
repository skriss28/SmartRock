/*
  SD card file dump

 This example shows how to read a file from the SD card using the
 SD library and send it over the serial port.

 The circuit:
 * SD card attached to SPI bus as follows:
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

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
  const uint8_t FILE_NAME_DIM  = BASE_NAME_SIZE + 7;
  char binName[FILE_NAME_DIM] = FILE_BASE_NAME "00.txt";
  char dname[FILE_NAME_DIM];
  strcpy(dname, binName);
  //Serial.println("Input SD card filename extension to dump: ");
  Serial.println(F("\nEnter two digit file suffix xx (Rock_xx.txt)to retrieve"));
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
    dname[BASE_NAME_SIZE + i] = c;
  }

  serialFlush();

  Serial.print(" SDcard file to be dumped: ");
  Serial.println(dname);
  Serial.println();

  boolean DumpSDcardfile_success= false;
  //Serial.println("Iniatalize SD card...");
  
  if (SDcardInitStatus == false) {
    Serial.println("SdFatSdioEX begin() failed. Check that card is initialized or present in slot.");
  }
  else { // SD memory card initiailzed; now open a file
    sdEx.chvol(); // make sdEx the current volume.
    //DumpSDcardfile_success = true;

    Serial.println("Card was initialized and present");
  }

  File dataFile = sdEx.open(dname, O_READ);   

  int n_bytes_dumped = 0;
  Serial.write(0x04); //Start marker for the dumped bytes
  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x04);

  // if the file is available, read from it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
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
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("Error opening and retrieving datafile");
  }
}
