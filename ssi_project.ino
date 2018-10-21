/*
 * This code was used on a Teensy sent to 30km.
 * The code monitors data from various sensors and records it to an SD card.
 * 
 * Team Grass
 */

#include <SPI.h>
#include "SdFat.h"

#include "Adafruit_MAX31855.h"

//-------------------
// Example creating a thermocouple instance with software SPI on any three
// digital IO pins.
#define MAXDO   12
#define MAXCS   9//10
#define MAXCLK  13

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
//-------------------

// Set USE_SDIO to zero for SPI card access. 
#define USE_SDIO 0
/*
 * Set DISABLE_CHIP_SELECT to disable a second SPI device.
 * For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
 * to 10 to disable the Ethernet controller.
 */
const int8_t DISABLE_CHIP_SELECT = -1;

SdFat sd;
File myFile;

const int chipSelect = 10;

void setup()
{
  //Set IO pins
//  SPI.setMOSI(11); //DO pin; connected to DI on the sd reader.
//  SPI.setMISO(12); //DI pin; connected to DO on the sd reader.
//  SPI.setSCK(13); //clock pin

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    delay(1);
  }

  if (!initializeSd()) {
    return;
  }
  
//  writeLineToFile("test.txt", "This is a test line!");
  readFileToConsole("test.txt");
  
}

bool initializeSd() {
  Serial.print("Initializing SD card...");

  //Initialize at quarter speed (may prevent various issues)
  if (!sd.begin(chipSelect, SPI_QUARTER_SPEED)) {
    Serial.println("initialization failed!");
    return false;
  }
  Serial.println("initialization done.");
  return true;
}

// Note: only one file may be opened at one time, so you
// must close it before opening another file.
void writeLineToFile(String filename, String testLine) {
  
  myFile = sd.open(filename, FILE_WRITE);
  
  // If file is successfully opened, proceed
  if (myFile) {
    Serial.print("Writing to ");
    Serial.print(filename);
    Serial.print("...  ");
    
    myFile.println(testLine);
  // close the file:
    myFile.close();
    Serial.println("done writing.");
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

char* doubleToString(double d, int digits) {
  char str[digits];
  sprintf(str, "%f", d);
  return str;
}

void loop()
{
  Serial.print("Internal Temp = ");
   Serial.println(thermocouple.readInternal());

   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("C = "); 
     Serial.println(c);

//     char* str = strcat("C = ", doubleToString(c, 4));
//     writeLineToFile("test.txt", str);
//     Serial.println(str);

//     char s[9];
//     sprintf(s, "%.3e", c);
//     char *line = strcat("C = ", s);
//     Serial.println(line);

      String thermoLog = "[THERMO] C: ";
      thermoLog += c;
      Serial.println(thermoLog);
      writeLineToFile("thermo.txt", thermoLog);
   }
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFarenheit());
 
   delay(1000);
}
