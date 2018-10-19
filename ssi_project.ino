/*
 * This code was used on a Teensy sent to 30km.
 * The code monitors data from various sensors and records it to an SD card.
 * 
 * Team Grass
 */

#include <SPI.h>
#include "SdFat.h"

SdFat sd;
SdFile file;
File myFile;

const int chipSelect = 10;

void setup()
{
  //Set IO pins
  SPI.setMOSI(11); //DO pin; connected to DI on the sd reader.
  SPI.setMISO(12); //DI pin; connected to DO on the sd reader.
  SPI.setSCK(13); //clock pin
  
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    SysCall::yield();
  }

  Serial.print("Initializing SD card...");

  //Initialize at quarter speed (may prevent various issues)
  if (!sd.begin(chipSelect, SPI_QUARTER_SPEED)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // Note: only one file may be opened at one time, so you
  // must close it before opening another file.
  
  writeLineToFile("test.txt", "This is a test line!");
  readFileToConsole("test.txt");
  
  
}

void writeLineToFile(char filename[], char testLine[]) {
  
  myFile = sd.open(filename, FILE_WRITE);
  
  // If file is successfully opened, proceed
  if (myFile) {
    Serial.print("Writing to ");
    Serial.print(filename);
    Serial.print("...  ");
    
    myFile.println(testLine);
  // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}

//This method reads the contents of a file to the Serial Monitor
void readFileToConsole(char filename[]) {
  
  myFile = sd.open(filename);

  // If file is successfully opened, proceed
  if (myFile) {
    Serial.print(filename);
    Serial.println(":");
    
    // Read from the file  to console until done
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // Close the file:
    myFile.close();
  } else {
    // If the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}

void loop()
{
  // nothing happens after setup
}
