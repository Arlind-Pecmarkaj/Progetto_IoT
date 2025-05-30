// ESP32 Sensor Monitoring Sketch - FIXED VERSION
// Sensors: MH-Z19B (SERIAL_1), ENS160+AHT2x (I2C_1), AHT20+BMP280 (I2C_2), SPS30 (I2C_3)

// Pin Definitions
#define PIN_BUTTON 35
#define PIN_SDA     21
#define PIN_SCL     22
#define RXD2        19
#define TXD2        18
#define LED_PIN     32

// Library Includes
#include <Wire.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "sps30.h"
#include "MHZCO2.h"

// Sensor Objects
MHZCO2               co2Sensor;           // SERIAL_1: MH-Z19B
DFRobot_ENS160_I2C   ens160(&Wire, 0x53); // I2C_1:    ENS160+AHT2x  
Adafruit_AHTX0       aht20;               // I2C_2:    AHT20+BMP280
Adafruit_BMP280      bmp280;              // I2C_2:    AHT20+BMP280
struct sps30_measurement sps30_meas;      // I2C_3:    SPS30

// Variables for sensor stability
unsigned long lastSensorRead = 0;
bool sensorsWarmedUp = false;

void setup() {
  // Initialize pins
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Serial communications
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 Multi-Sensor Monitor ===");
  
  // Initialize I2C
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); // 100kHz for better compatibility
  
  // Initialize Serial2 for MH-Z19B
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("\n--- Initializing Sensors ---");
  
  // 1. SERIAL_1: MH-Z19B CO2 Sensor
  Serial.print("1. MH-Z19B (SERIAL_1): ");
  co2Sensor.begin(&Serial2);
  delay(500);
  Serial.println("Initialized (warming up...)");
  
  // 2. I2C_1: ENS160+AHT2x Combined Sensor
  Serial.print("2. ENS160+AHT2x (I2C_1): ");

  int ens160BeginResult = ens160.begin(); 
  if (ens160BeginResult != 0) {
    Serial.print("[ERROR] ENS160 init failed "); Serial.println(ens160BeginResult);
  } else {
    Serial.println("Initialized");
    // Set ENS160 to standard mode
    ens160.setPWRMode(ENS160_STANDARD_MODE);
    delay(500);
  }
  
  // 3. I2C_2: AHT20+BMP280 Combined Sensor
  Serial.print("3a. AHT20 (I2C_2): ");
  if (!aht20.begin()) { 
    Serial.println("[ERROR] AHT20 failed to start");
  } else {
    Serial.println("Initialized");
  }
  
  Serial.print("3b. BMP280 (I2C_2): ");
  if (!bmp280.begin()) { 
    Serial.println("[ERROR] BMP280 not detected");
  } else {
    Serial.println("Initialized");
  }
  
  // 4. I2C_3: SPS30 Particulate Matter Sensor
  Serial.print("4. SPS30 (I2C_3): ");
  sensirion_i2c_init();
  if (sps30_probe() != 0) {
    Serial.println("[ERROR] SPS30 not detected");
  } else {
    Serial.println("Initialized");
    sps30_start_measurement();
    delay(1000);
  }
  
  Serial.println("\n--- Setup Complete ---");
  Serial.println("Sensors warming up for 30 seconds...\n");
  delay(3000); // Initial delay for sensor stabilization
}

void loop() {
  // Button and LED control
  bool buttonPressed = (digitalRead(PIN_BUTTON) == LOW);
  digitalWrite(LED_PIN, buttonPressed);
  
  // Check if sensors have warmed up (30 seconds)
  if (millis() > 30000) {
    sensorsWarmedUp = true;
  }
  
  // Timestamp
  Serial.print("=== Timestamp: ");
  Serial.print(millis());
  if (!sensorsWarmedUp) {
    Serial.print(" (warming up...)");
  }
  Serial.println(" ===\n");
  
  // 1. SERIAL_1: MH-Z19B CO2 Sensor
  Serial.println("1. MH-Z19B CO2 Sensor (SERIAL_1):");
  int co2ppm = co2Sensor.getCO2();
  if (co2ppm > 0) {
    Serial.print("   CO2: "); Serial.print(co2ppm); Serial.println(" ppm");
  } else {
    Serial.println("   CO2: [Warming up or error]");
  }
  Serial.println();
  
  // 2. I2C_1: ENS160+AHT2x Combined Sensor
  Serial.println("2. ENS160+AHT2x Combined Sensor (I2C_1):");
  uint8_t ens_status = ens160.getENS160Status();
  if (ens_status == 0) {
    uint16_t tvoc = ens160.getTVOC();
    uint16_t eco2 = ens160.getECO2();
    uint8_t aqi = ens160.getAQI();
    
    Serial.print("   eCO2: "); Serial.print(eco2); Serial.println(" ppm");
    Serial.print("   TVOC: "); Serial.print(tvoc); Serial.println(" ppb");
    Serial.print("   AQI: "); Serial.println(aqi);
    // NOTA: Aggiungere la temperatura e umidità relativa.
  } else {
    Serial.println("   ENS160: [Not ready or error]");
  }
  Serial.println();
  
  // 3. I2C_2: AHT20+BMP280 Combined Sensor
  Serial.println("3. AHT20+BMP280 Combined Sensor (I2C_2):");
  
  // AHT20 readings
  sensors_event_t humidity, temp;
  if (aht20.getEvent(&humidity, &temp)) {
    if (temp.temperature > -40 && temp.temperature < 55) { // Valid range check
      Serial.print("   AHT20 Temperature: "); Serial.print(temp.temperature); Serial.println(" °C");
      Serial.print("   AHT20 Humidity: "); Serial.print(humidity.relative_humidity); Serial.println(" %");
    } else {
      Serial.println("   AHT20: [Invalid reading - check connections]");
    }
  } else {
    Serial.println("   AHT20: [Read error]");
  }
  
  // BMP280 readings
  float bmpTemp = bmp280.readTemperature();
  float bmpPressure = bmp280.readPressure();
  float bmpAltitude = bmp280.readAltitude(1013.25); // Sea level pressure
  
  if (bmpTemp > -40 && bmpTemp < 55) { // Valid range check
    Serial.print("   BMP280 Temperature: "); Serial.print(bmpTemp); Serial.println(" °C");
    Serial.print("   BMP280 Pressure: "); Serial.print(bmpPressure / 100.0F); Serial.println(" hPa");
    Serial.print("   BMP280 Altitude: "); Serial.print(bmpAltitude); Serial.println(" m");
  } else {
    Serial.println("   BMP280: [Invalid reading - check connections]");
  }
  Serial.println();
  
  // 4. I2C_3: SPS30 Particulate Matter Sensor
  Serial.println("4. SPS30 Particulate Matter Sensor (I2C_3):");
  uint16_t ready;
  if (sps30_read_data_ready(&ready) == 0 && ready) {
    if (sps30_read_measurement(&sps30_meas) == 0) {
      // Mass Concentrations (μg/m³)
      Serial.print("   PM1.0: "); Serial.print(sps30_meas.mc_1p0); Serial.println(" u/m³");
      Serial.print("   PM2.5: "); Serial.print(sps30_meas.mc_2p5); Serial.println(" ug/m³");
      Serial.print("   PM4.0: "); Serial.print(sps30_meas.mc_4p0); Serial.println(" ug/m³");
      Serial.print("   PM10: "); Serial.print(sps30_meas.mc_10p0); Serial.println(" ug/m³");
      
      // Number Concentrations (#/cm³)
      Serial.print("   NC0.5: "); Serial.print(sps30_meas.nc_0p5); Serial.println(" 1/cm³");
      Serial.print("   NC1.0: "); Serial.print(sps30_meas.nc_1p0); Serial.println(" 1/cm³");
      Serial.print("   NC2.5: "); Serial.print(sps30_meas.nc_2p5); Serial.println(" 1/cm³");
      Serial.print("   NC4.0: "); Serial.print(sps30_meas.nc_4p0); Serial.println(" 1/cm³");
      Serial.print("   NC10: "); Serial.print(sps30_meas.nc_10p0); Serial.println(" 1/cm³");
      
      // Typical Particle Size
      Serial.print("   Typical Size: "); Serial.print(sps30_meas.typical_particle_size); Serial.println(" μm");
    } else {
      Serial.println("   SPS30: [Read error]");
    }
  } else {
    Serial.println("   SPS30: [Data not ready]");
  }
  
  Serial.println("\n" + String('=', 50) + "\n");
  delay(5000); // 5 second delay between readings
}
