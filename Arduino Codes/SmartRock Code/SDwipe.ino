boolean wipeSDcard() {

  //initialize boolean to return
  boolean WipeSDcardfile_success = false;
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  // Verify SD card is initialized.
  if (!SDcardInitStatus ) { //SD card not initialized
    // JE 10 Apr 2020: not sure how this case can ever occur, leaving it for posterity for now
    // SD card has to be initialized before gathering data.
    // If file dump occuring after power reset, SD card always initializes in setup

    Serial.println("SD card not initialized. Retrying SD card initialize...\n");

    SDcardInitStatus = initSDcard();
    if (!SDcardInitStatus) { //failed on 2nd try, tell user file can't dump and return and return
      Serial.println("SD card could not be initialized. Data file transfer can not be completed.\n");
      Serial.println("Recommendation: carefully open smartrock module; release SD card from socket and re-insert it; then try file transfer again.");
    }
  }


  else { // SD memory card previously initiailzed
    Serial.println("SD Card previously initialized. Ready to proceed with wiping files \n");
  }


  // ----------
  //  Do the  actual wipe(). Check with user first, this could be a perilous operation.
  // ------------
  serialFlush(); //need to make sure read buffer is empty b/c we need more user input

  Serial.println("Are you sure you want to proceed? Type 'Y' to wipe all data.");
  while (!Serial.available()) {
    SysCall::yield();
  }
  int wipeChar = Serial.read();
  if (wipeChar != 'Y') {
    Serial.println("Quitting wipe, you did not type 'Y'.");
    return false;
  }


  //else we are proceeding with wipe
  Serial.println("SD card wipe request confirmed.");

  // Use wipe() for no dot progress indicator.
  if (!sdEx.wipe(&Serial)) {
    Serial.println("Wipe failed.");
    return false;
  }
  else { //wipe successfully completed
    Serial.println("Wipe successful!\n\n Reinitializing card now (required after wipe)...");


  }

  // Must reinitialize after wipe.

  SDcardInitStatus = initSDcard();   // initialize SDfat volume
  if (SDcardInitStatus) {
    Serial.println("Success!");
    return true;
  }
  else {
    Serial.println("Error - reinitializing SD card failed.");
    return false;
  }

}
