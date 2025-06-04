// ESP32 Sensor Monitoring Sketch - InfluDB + LED Alert 
// Sensors: MH-Z19B (SERIAL_1), ENS160+AHT2x (I2C_1), AHT20+BMP280 (I2C_2), SPS30 (I2C_3)

// Pin Definitions
#define PIN_SDA    21     /* Pin SDA per I2C */
#define PIN_SCL    22     /* Pin SCL per I2C */
#define RXD2       19     /* Pin RX per Serial2 (MH-Z19B) */
#define TXD2       18     /* Pin TX per Serial2 (MH-Z19B) */
#define LED_PIN    32     /* Pin per il LED di allerta qualità aria */

// WiFi Credentials (REPLACE WITH YOURS!)
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

// InfluxDB Configuration (REPLACE WITH YOURS)
#define INFLUXDB_URL "INFLUXDB_URL"
#define INFLUXDB_TOKEN "INFLUXDB_TOKEN"
#define INFLUXDB_ORG "INFLUXDB_ORG"
#define INFLUXDB_BUCKET "INFLUXDB_BUCKET"
#define INFLUXDB_MEASUREMENT "INFLUXDB_MEASUREMENT"

// Device Tags (REPLACE WITH YOUR ACTUALS!)
#define DEVICE_LOCATION "LOCATION" 
#define DEVICE_ROOM "ROOM" 

// Soglie per l'allerta LED
#define AQI_ALERT_THRESHOLD 4      // AQI >= 4 (Scarso o Peggio)
#define PM25_ALERT_THRESHOLD 35.5f // PM2.5 > 35.5 µg/m³

// Library Includes
#include <WiFi.h>             /* per protocollo wifi */
#include <InfluxDbClient.h>   /* per libreria di comunicazione con InfluxDB */
#include <InfluxDbCloud.h>    /* per gestire il token di sicurezza InfluxDB v2 */
#include <Wire.h>             /* per comunicazione I2C */
#include <DFRobot_ENS160.h>   /* per sensore ENS160 */
#include <Adafruit_AHTX0.h>   /* per sensore AHT20 */
#include <Adafruit_BMP280.h>  /* per sensore BMP280 */
#include "sps30.h"            /* per sensore SPS30 */
#include "MHZCO2.h"           /* per sensore MH-Z19B */

// Sensor Objects
MHZCO2               mhz19bSensor;
DFRobot_ENS160_I2C   ens160(&Wire, 0x53);
Adafruit_AHTX0       aht20;
Adafruit_BMP280      bmp280;
struct sps30_measurement sps30_data; // Struct per i dati SPS30

// InfluxDB Client and Point Object
InfluxDBClient       influxClient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN);
Point                envDataPoint(INFLUXDB_MEASUREMENT); // Oggetto Point globale per i dati ambientali

// Global variables for sensor readings
bool    sensorsWarmedUpGeneral = false;
float   currentTempAHT20 = -273.15f; // Gradi Celsius
float   currentHumAHT20 = -1.0f;    // Percentuale %
int     co2MHZ19B = 0;              // ppm
uint16_t tvocENS160 = 0;            // ppb
uint16_t eco2ENS160 = 0;            // ppm
uint8_t  aqiENS160 = 0;             // Indice AQI UBA
float   tempBMP280 = -273.15f;      // Gradi Celsius
float   pressureBMP280 = 0.0f;      // hPa
float   altitudeBMP280 = 0.0f;      // metri
bool    sps30HasValidData = false;  // Flag per validità dati SPS30

void connectWiFi() {
  Serial.print("Connessione a WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int wifi_retries = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_retries < 30) {
    delay(500);
    Serial.print(".");
    wifi_retries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connesso! Indirizzo IP: ");
    Serial.println(WiFi.localIP());
    configTime(0, 0, "pool.ntp.org", "time.nist.gov"); // Sincronizzazione NTP
    Serial.print("In attesa della sincronizzazione dell'ora");
    time_t now = time(nullptr);
    while (now < 8 * 3600 * 2) {
      delay(500);
      Serial.print(".");
      now = time(nullptr);
    }
    Serial.println("\nOra sincronizzata.");
  } else {
    Serial.println("\nConnessione WiFi FALLITA. Procedo senza InfluxDB.");
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED spento all'inizio

  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 Multi-Sensor Monitor ===");

  connectWiFi();

  if (WiFi.isConnected()) {
    Serial.print("Verifica connessione InfluxDB: ");
    if (influxClient.validateConnection()) {
      Serial.print("Connesso a InfluxDB!");
      // Aggiungo i tag globali all'oggetto Point
      envDataPoint.addTag("host", "ESP"); // Uso il MAC address come ID univoco dell'host
      envDataPoint.addTag("location", DEVICE_LOCATION);
      envDataPoint.addTag("room", DEVICE_ROOM);
    } else {
      Serial.print("Connessione a InfluxDB FALLITA: ");
      Serial.println(influxClient.getLastErrorMessage());
    }
  }
  
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); 
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println("\n--- Inizializzazione Sensori ---");
  
  Serial.print("1. MH-Z19B (SERIAL_1): ");
  mhz19bSensor.begin(&Serial2);
  Serial.println(mhz19bSensor.getCO2() > 0 ? "Inizializzato (riscaldamento...)" : "[ERRORE] MH-Z19B non risponde");
  
  Serial.print("2. ENS160 (I2C_1): ");
  if (ens160.begin() == 0) {
    Serial.println("Inizializzato");
    ens160.setPWRMode(ENS160_STANDARD_MODE);
  } else {
    Serial.println("[ERRORE] ENS160 init fallito");
  }
  
  Serial.print("3a. AHT20 (I2C_2): ");
  Serial.println(aht20.begin() ? "Inizializzato" : "[ERRORE] AHT20 init fallito");
  
  Serial.print("3b. BMP280 (I2C_2): ");
  if (bmp280.begin()) {
    Serial.println("Inizializzato");
    bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_1000);
  } else {
    Serial.println("[ERRORE] BMP280 init fallito");
  }
  
  Serial.print("4. SPS30 (I2C_3): ");
  sensirion_i2c_init();
  if (sps30_probe() == 0) {
    Serial.println("Inizializzato");
    if (sps30_start_measurement() != 0) Serial.println("[ERRORE] SPS30 start measurement fallito");
  } else {
    Serial.println("[ERRORE] SPS30 init fallito");
  }
  
  Serial.println("\n--- Setup Completato ---");
  Serial.println("Riscaldamento sensori per circa 60 secondi (potrebbe impiegarci di più)...");
  delay(6000); 
}

void readAllSensors() {
  Serial.println("\n--- Lettura Sensori ---");

  // AHT20 (Temperatura e Umidità)
  sensors_event_t hum_event, temp_event;
  bool aht20_success = aht20.getEvent(&hum_event, &temp_event);
  if (aht20_success && temp_event.temperature > -40 && temp_event.temperature < 85 && hum_event.relative_humidity >= 0 && hum_event.relative_humidity <= 100) {
    currentTempAHT20 = temp_event.temperature;
    currentHumAHT20 = hum_event.relative_humidity;
    Serial.printf("AHT20: Temp=%.2f°C, Hum=%.2f%%\n", currentTempAHT20, currentHumAHT20);
  } else {
    currentTempAHT20 = -273.15f; // Reset a valore non valido
    currentHumAHT20 = -1.0f;     // Reset a valore non valido
    Serial.println("AHT20: [Errore lettura o valori non validi]");
  }

  // MH-Z19B (CO2)
  co2MHZ19B = mhz19bSensor.getCO2();
  if (co2MHZ19B > 0 && co2MHZ19B < 10000) {
    Serial.printf("MH-Z19B: CO2=%d ppm\n", co2MHZ19B);
  } else {
    co2MHZ19B = 0; // Reset a valore non valido/default
    Serial.println("MH-Z19B: [Riscaldamento o errore]");
  }

  // ENS160 (eCO2, TVOC, AQI) - con compensazione da AHT20
  if (currentTempAHT20 > -41.0f && currentHumAHT20 >= 0.0f) { // Se AHT20 ha fornito dati validi
    ens160.setTempAndHum(currentTempAHT20, currentHumAHT20);
     Serial.printf("ENS160: Compensazione Temp=%.1f°C, Hum=%.1f%%\n", currentTempAHT20, currentHumAHT20);
  }
  uint8_t ens_status = ens160.getENS160Status();
  if (ens_status == 0) { // Status 0 (Normal), 1 (Warmup), 2 (Initial Startup)
      eco2ENS160 = ens160.getECO2();
      tvocENS160 = ens160.getTVOC();
      aqiENS160 = ens160.getAQI();
      Serial.printf("ENS160: eCO2=%d ppm, TVOC=%d ppb, AQI=%d (Status: %d)\n", eco2ENS160, tvocENS160, aqiENS160, ens_status);
  } else if (ens_status == 1 || ens_status == 2) {
      Serial.println("ENS160: Il sensore è in fase di riscaldamento/startup.");
  } else { 
      Serial.println("ENS160: Errore lettura status o sensore non pronto.");
      eco2ENS160 = 0; tvocENS160 = 0; aqiENS160 = 0;
  }

  // BMP280 (Temperatura, Pressione, Altitudine)
  tempBMP280 = bmp280.readTemperature();
  pressureBMP280 = bmp280.readPressure() / 100.0F; // Converti Pa a hPa
  if (tempBMP280 > -40 && tempBMP280 < 85 && pressureBMP280 > 300 && pressureBMP280 < 1100) {
    Serial.printf("BMP280: Temp=%.2f°C, Press=%.2f hPa\n", tempBMP280, pressureBMP280);
  } else {
    tempBMP280 = -273.15f; pressureBMP280 = 0; // Reset
    Serial.println("BMP280: [Errore lettura o valori non validi]");
  }

  // SPS30 (Particolato)
  uint16_t sps_data_ready;
  sps30HasValidData = false;
  if (sps30_read_data_ready(&sps_data_ready) == 0 && sps_data_ready) {
    if (sps30_read_measurement(&sps30_data) == 0) {
      sps30HasValidData = true;
      Serial.printf("SPS30: PM2.5=%.2f µg/m³, PM10=%.2f µg/m³\n", sps30_data.mc_2p5, sps30_data.mc_10p0);
      // Potresti stampare altri valori SPS30 se necessario
    } else {
      Serial.println("SPS30: Errore lettura misurazione");
    }
  } else {
    Serial.println("SPS30: Dati non pronti o errore flag");
  }
}

void processSensorAlerts() {
  bool alert = false;
  if (aqiENS160 >= AQI_ALERT_THRESHOLD) {
    alert = true;
    Serial.println("ALLERTA: AQI elevato!");
  }
  if (sps30HasValidData && sps30_data.mc_2p5 > PM25_ALERT_THRESHOLD) {
    alert = true;
    Serial.println("ALLERTA: PM2.5 elevato!");
  }
  digitalWrite(LED_PIN, alert);
}

void writeToInfluxDB() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi non connesso. Questa serie di dati non verrà inviata a InfluxDB.");
    connectWiFi(); 
    return;
  }

  envDataPoint.clearFields(); // Pulisco i campi precedenti

  // Aggiungo solo i campi con valori validi/sensati
  if (currentTempAHT20 > -41.0f) envDataPoint.addField("temperature_aht20", currentTempAHT20);
  if (currentHumAHT20 >= 0.0f) envDataPoint.addField("humidity_aht20", currentHumAHT20);
  if (co2MHZ19B > 0) envDataPoint.addField("co2_mhz19b", co2MHZ19B);
  
  if (eco2ENS160 > 0 ) envDataPoint.addField("eco2_ens160", eco2ENS160); // ENS160 può dare 0 durante il riscaldamento iniziale
  if (tvocENS160 >= 0 ) envDataPoint.addField("tvoc_ens160", tvocENS160); // TVOC può essere 0
  if (aqiENS160 > 0 ) envDataPoint.addField("aqi_ens160", (int)aqiENS160);
  if (currentTempAHT20 > -41.0f && currentHumAHT20 >= 0.0f) { // Log compensazione se usata
      envDataPoint.addField("comp_temp_ens160", currentTempAHT20);
      envDataPoint.addField("comp_hum_ens160", currentHumAHT20);
  }

  if (tempBMP280 > -41.0f) envDataPoint.addField("temperature_bmp280", tempBMP280);
  if (pressureBMP280 > 300) envDataPoint.addField("pressure_bmp280", pressureBMP280);
  if (altitudeBMP280 != 0 || pressureBMP280 > 300) envDataPoint.addField("altitude_bmp280", altitudeBMP280); // Altitudine può essere 0

  if (sps30HasValidData) {
    envDataPoint.addField("pm1p0_sps30", sps30_data.mc_1p0);
    envDataPoint.addField("pm2p5_sps30", sps30_data.mc_2p5);
    envDataPoint.addField("pm4p0_sps30", sps30_data.mc_4p0);
    envDataPoint.addField("pm10p0_sps30", sps30_data.mc_10p0);
    envDataPoint.addField("typical_particle_size_sps30", sps30_data.typical_particle_size);
  }

  if (!envDataPoint.hasFields()) {
      Serial.println("Nessun dato valido da inviare a InfluxDB.");
      return;
  }

  Serial.print("Scrittura su InfluxDB: ");
  Serial.println(envDataPoint.toLineProtocol());
  if (!influxClient.writePoint(envDataPoint)) {
    Serial.print("Scrittura InfluxDB FALLITA: ");
    Serial.println(influxClient.getLastErrorMessage());
  } else {
    Serial.println("Scrittura InfluxDB RIUSCITA!");
  }
}

void loop() {
  if (!sensorsWarmedUpGeneral && millis() > 60000) { // Riscaldamento generale di 60s
    sensorsWarmedUpGeneral = true;
    Serial.println("--- Riscaldamento generale sensori (60s) completato ---");
  }
  
  Serial.print("=== Timestamp Loop: "); Serial.print(millis()); Serial.println(" ===");

  readAllSensors();
  processSensorAlerts();
  writeToInfluxDB();
  
  Serial.println("\n" + String('=', 50) + "\n");
  delay(15000); // Pausa di 15 secondi tra i cicli di lettura/invio
}