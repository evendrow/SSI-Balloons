/*
 * This code was used on a Teensy sent to 30km.
 * The code monitors data from various sensors and records it to an SD card.
 * 
 * Edward Vendrow, Hale Konopka
 * Team Grass
 */

#include <SPI.h>

#include "SdFat.h" //SD card
#include "Adafruit_MAX31855.h" //thermocouple
#include <Adafruit_BNO055.h> //accelerometer
#include <Adafruit_BMP280.h> //temp/pressure (BMP280)

#define SAMPLERATE_DELAY_MS (1000)

//--------------------------------------
// Define data structures for sensor data transfer
typedef struct {
  double temp;
} Thermo_t;

typedef struct {
  double latitude;
  double longitude;
} GPS_t;

typedef struct {
  double temp;
  double pressure;
  double alt;
} BMP_t;

typedef struct {
  double x;
  double y;
  double z;
} Accel_t;
//--------------------------------------

//--------------------------------------
//Define global sensor variables
 
Thermo_t thermo = {-1};
GPS_t gps = {-1, -1};
BMP_t bmp = {-1, -1, -1};
Accel_t accel = {-1, -1, -1};

//--------------------------------------

//--------------------------------------
// Create a thermocouple instance with software SPI 
// on any three digital IO pins.
#define MAXDO   7//12
#define MAXCS   9//10
#define MAXCLK  14//13

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

//--------------------------------------

//--------------------------------------
//Create temp/pressure sensor with software SPI
#define BMP_SCK 14
#define BMP_MISO 7
#define BMP_MOSI 6
#define BMP_CS 8

//Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
Adafruit_BMP280 bmpChip(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
//--------------------------------------

//--------------------------------------
// initialize the Accelerometer
Adafruit_BNO055 bno = Adafruit_BNO055(55);
//--------------------------------------

//--------------------------------------
// SD Card setup

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

//--------------------------------------

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

  //If SD fails to initialize, crash
  if (!initializeSd()) {
    return;
  }

  //Try to initialize BMP
  if (!bmpChip.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }

  //Try to initialize accelerometer
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  bno.setExtCrystalUse(true);

  
  

  //Delay to wait for sensors to load
  delay(1000);

  //  writeLineToFile("test.txt", "This is a test line!");
  readFileToConsole("test.txt");
}

/*
 * Initializes the SD Card at quarter speed
 * Returns true if initialization succeeded
 * 
 * Note: only one file may be opened at one time, so you
 * must close it before opening another file.
 */
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

/*
 * Writes a single given line to a file.
 * 
 * This method opens and closes the file on every write,
 * which may cause slowdown but protects data in case of
 * sudden power failure or some other issue.
 */
void writeLineToFile(String filename, String testLine) {
  
  myFile = sd.open(filename, FILE_WRITE);
  
  // If file is successfully opened, proceed
  if (myFile) {
    Serial.print("Writing to ");
    Serial.print(filename);
    Serial.print("...  ");
    
    myFile.println(testLine);
    
    // Close the file:
    myFile.close();
    Serial.println("done writing.");
  } else {
    // if the file didn't open, print an error:
    Serial.print("error opening ");
    Serial.println(filename);
  }
}

/*
 * This method reads the contents of a file to the Serial Monitor
 */
void readFileToConsole(String filename) {
  
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

// May or may not work
/*char* doubleToString(double d, int digits) {
  char str[digits];
  sprintf(str, "%f", d);
  return str;
}*/

/*
 * Updates the thermocouple data structure with current data
 * Sets to -1 if temperature reading fails.
 */
void updateThermoTemp() {
  double c = thermocouple.readCelsius();
  if (isnan(c)) {
   Serial.println("Something wrong with thermocouple!");
   thermo.temp = -1;
  }

  thermo.temp = c;
}

/*
 * Updates the accelerometer data structure with current data
 */
void updateAccelData() {
  sensors_event_t event;
  bno.getEvent(&event);

  accel.x = event.orientation.x;
  accel.y = event.orientation.y;
  accel.z = event.orientation.z;
}

void updateBMPData() {
  bmp.temp = bmpChip.readTemperature();
  bmp.pressure = bmpChip.readPressure();
  bmp.alt = bmpChip.readAltitude(1013.25); // this should be adjusted to your local forcase
}

/*
 * Updates sensor data 
 */
void updateSensorData() {
  updateThermoTemp();
  updateAccelData();
  updateBMPData();
}

void loop() {

  updateSensorData();
  
  //Log thermo data
  String thermoLog = "[THERMO] C: ";
  thermoLog += thermo.temp;
  Serial.println(thermoLog);
  
  //Log bmp data
  Serial.print("[BMP280] Temperature = ");
  Serial.print(bmp.temp);
  Serial.println(" *C");

  Serial.print("[BMP280] Pressure = ");
  Serial.print(bmp.pressure);
  Serial.println(" Pa");

  Serial.print("[BMP280] Approx altitude = ");
  Serial.print(bmp.alt); // this should be adjusted to your local forcase
  Serial.println(" m");

  //Log accelerometer data
  Serial.print("[ACCEL] ");
  Serial.print("X: ");
  Serial.print(accel.x, 4);
  Serial.print("\tY: ");
  Serial.print(accel.y, 4);
  Serial.print("\tZ: ");
  Serial.println(accel.z, 4);
   
  // Here we create a single line to input into the log file
  String dataLog = "";
  dataLog += millis();        dataLog += ","; // Timestamp
  dataLog += thermo.temp;     dataLog += ","; // Thermocouple temperature
  dataLog += bmp.temp;        dataLog += ","; // BMP temperature
  dataLog += bmp.pressure;    dataLog += ","; // BMP pressure
  dataLog += bmp.alt;         dataLog += ","; // BMP altitude
  dataLog += accel.x;         dataLog += ","; // Accelerometer x
  dataLog += accel.y;         dataLog += ","; // Accelerometer y
  dataLog += accel.z;       //dataLog += ","; // Accelerometer z

  Serial.println(dataLog);
  Serial.println("");
  
  // Write the log to the log file
  // The log file is in CSV format for easy data analysis
//  writeLineToFile("thermo.txt", thermoLog);

  // Delay the next loop by a predetermined sample rate
  // Since data collection and logging take time, the
  // actual sample rate will be higher
   delay(SAMPLERATE_DELAY_MS);
}
