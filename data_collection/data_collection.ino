// =================================================================
// Monitor Ambientale Multi-Sensore per ESP32
//
// Monitora diversi sensori di qualità dell'aria e ambientali e invia
// i dati a un database InfluxDB. Include un sistema di allerta
// locale tramite LED.
//
// Sensori:
// - MH-Z19B (CO2 + Temp) via Seriale
// - ENS160 + AHT2x (VOC, eCO2, AQI, Temp, Hum) via I2C
// - BMP280 (Pressione, Temp) via I2C
// - SPS30 (Particolato) via I2C
//
// Autore: Arlind Pecmarkaj
// Ultimo Aggiornamento: 2025-06-07
// =================================================================

// -- Definizione dei Pin --
#define PIN_SDA   21    // Pin Dati I2C (SDA)
#define PIN_SCL   22    // Pin Clock I2C (SCL)
#define RXD2      19    // Pin RX per Serial2 (collegato a MH-Z19B)
#define TXD2      18    // Pin TX per Serial2 (collegato a MH-Z19B)
#define LED_PIN   32    // Pin per il LED di allerta qualità dell'aria

// -- Credenziali WiFi --
#define WIFI_SSID "HiddenSSID"
#define WIFI_PASSWORD "HiddenPassword"

// -- Configurazione InfluxDB --
#define INFLUXDB_URL "http://yourdb.com/"
#define INFLUXDB_TOKEN "yourtoken"
                       
#define INFLUXDB_ORG "uniurb"
#define INFLUXDB_BUCKET "esercitazioni"
#define INFLUXDB_MEASUREMENT "corso_IoT"

// -- Metadati del Dispositivo (Tags) --
#define DEVICE_LOCATION "YourLocation"
#define DEVICE_ROOM "YourRoom"
#define DEVICE_HOST "YourHost" // Identificatore univoco per questo dispositivo

// -- Soglie per gli Allarmi --
#define AQI_ALERT_THRESHOLD 4      // Allarme se AQI è >= 4 (Qualità scarsa o peggio)
#define PM25_ALERT_THRESHOLD 25f // Allarme se PM2.5 supera questo valore (µg/m³)
#define PM10_ALERT_THRESHOLD 50f // Allarme se PM10 supera questo valore (µg/m³)

// -- Inclusione Librerie --
#include <WiFi.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <Wire.h>
#include <DFRobot_ENS160.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include "sps30.h"
#include "MHZCO2.h"

// -- Oggetti dei Sensori --
MHZ19B mhz19bSensor;
DFRobot_ENS160_I2C ens160(&Wire, 0x53);
Adafruit_AHTX0 aht20;
Adafruit_BMP280 bmp280;
struct sps30_measurement sps30_data; // Struct per i dati del sensore SPS30

// -- Client InfluxDB --
InfluxDBClient influxClient(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN,
                      InfluxDbCloud2CACert);
Point envDataPoint(INFLUXDB_MEASUREMENT); // Oggetto Point riutilizzabile per i dati dei sensori

// -- Variabili Globali per le Letture dei Sensori --
#define SENSOR_INVALID_TEMP -273.15f // Valore che rappresenta una lettura di temperatura non valida
#define SENSOR_INVALID_HUM -1.0f     // Valore che rappresenta una lettura di umidità non valida

bool sensorsWarmedUpGeneral = false;
float currentTempAHT20 = SENSOR_INVALID_TEMP;
float currentHumAHT20 = SENSOR_INVALID_HUM;
float tempMHZ19B = SENSOR_INVALID_TEMP;
int co2MHZ19B = 0;
uint16_t tvocENS160 = 0;
uint16_t eco2ENS160 = 0;
uint8_t aqiENS160 = 0;
float tempBMP280 = SENSOR_INVALID_TEMP;
float pressureBMP280 = 0.0f;
bool sps30HasValidData = false;

// -- Prototipi delle Funzioni --
void connectWiFi();
void initializeSensors();
void readAllSensors();
void processSensorAlerts();
void writeToInfluxDB();

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Inizia con il LED spento

    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== Monitor Ambientale Multi-Sensore v2.1 ===");

    connectWiFi();

    // Inizializza le interfacce I2C e Seriale per i sensori
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000); // Imposta il clock I2C a 100kHz
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

    initializeSensors();
    
    // Aggiunge i tag permanenti al data point di InfluxDB
    envDataPoint.addTag("host", DEVICE_HOST);
    envDataPoint.addTag("location", DEVICE_LOCATION);
    envDataPoint.addTag("room", DEVICE_ROOM);

    Serial.println("\n--- Setup Completato ---");
    Serial.println("Periodo di riscaldamento iniziale dei sensori (60 secondi)... le letture potrebbero essere instabili.");
}

void loop() {
    // Un semplice flag per tracciare il completamento del riscaldamento generale iniziale
    if (!sensorsWarmedUpGeneral && millis() > 60000) {
        sensorsWarmedUpGeneral = true;
        Serial.println("\n--- Riscaldamento iniziale di 60s completato ---");
    }
    
    Serial.print("\n=== Nuovo Ciclo di Misurazione | Timestamp: "); Serial.print(millis()); Serial.println(" ms ===");

    readAllSensors();
    processSensorAlerts();
    
    if (WiFi.status() == WL_CONNECTED) {
        writeToInfluxDB();
    } else {
        Serial.println("[Attenzione] WiFi disconnesso. Salto l'invio a InfluxDB e tento la riconnessione.");
        connectWiFi(); // Tenta la riconnessione per il ciclo successivo
    }
    
    Serial.println("\n" + String('=', 60));
    delay(30000); // Attendi 30 secondi prima del ciclo successivo
}

void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        return; // Già connesso, esce dalla funzione
    }
    Serial.print("Connessione al WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("\nWiFi Connesso! Indirizzo IP: ");
        Serial.println(WiFi.localIP());

        // Sincronizza l'ora via NTP per timestamp accurati in InfluxDB
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        Serial.print("In attesa della sincronizzazione dell'ora");
        time_t now = time(nullptr);
        while (now < 8 * 3600 * 2) {
            delay(500);
            Serial.print(".");
            now = time(nullptr);
        }
        Serial.println("\nOra sincronizzata.");

        // Verifica la connessione a InfluxDB
        Serial.print("Verifica connessione InfluxDB... ");
        if (influxClient.validateConnection()) {
            Serial.println("Successo!");
            Serial.print("Bucket di destinazione: "); Serial.print(INFLUXDB_ORG); Serial.print("/"); Serial.println(INFLUXDB_BUCKET);
        } else {
            Serial.print("Fallita: ");
            Serial.println(influxClient.getLastErrorMessage());
        }
    } else {
        Serial.println("\nConnessione WiFi FALLITA. Riproverò più tardi.");
    }
}

void initializeSensors() {
    Serial.println("\n--- Inizializzazione Sensori ---");

    // 1. MH-Z19B (Seriale)
    Serial.print("1. MH-Z19B (CO2 & Temp): ");
    mhz19bSensor.begin(&Serial2);
    mhz19bSensor.measure(); // Questo comando serve a 'svegliare' il sensore
    Serial.println("Comando di init inviato. In fase di riscaldamento...");

    // 2. ENS160 (I2C)
    Serial.print("2. ENS160 (AQI, eCO2, TVOC): ");
    if (ens160.begin() == 0) {
        ens160.setPWRMode(ENS160_STANDARD_MODE);
        Serial.println("Inizializzato.");
    } else {
        Serial.println("[ERRORE] Non trovato. Controllare cablaggio o indirizzo.");
    }
    
    // 3a. AHT20 (I2C)
    Serial.print("3a. AHT20 (Temp & Umidità): ");
    if (aht20.begin()) {
        Serial.println("Inizializzato.");
    } else {
        Serial.println("[ERRORE] Non trovato. Controllare cablaggio.");
    }

    // 3b. BMP280 (I2C)
    Serial.print("3b. BMP280 (Pressione & Temp): ");
    if (bmp280.begin()) { // L'indirizzo alternativo comune è 0x77
        bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_1000);
        Serial.println("Inizializzato.");
    } else {
        Serial.println("[ERRORE] Non trovato. Controllare cablaggio o indirizzo (0x76/0x77).");
    }

    // 4. SPS30 (I2C)
    Serial.print("4. SPS30 (Particolato): ");
    sensirion_i2c_init();
    if (sps30_probe() == 0) {
        if (sps30_start_measurement() != 0) {
            Serial.println("[ERRORE] Avvio misurazione fallito.");
        } else {
            Serial.println("Inizializzato e in misurazione.");
        }
    } else {
        Serial.println("[ERRORE] Non trovato. Controllare cablaggio.");
    }
}


void readAllSensors() {
    Serial.println("\n--- Lettura di Tutti i Valori dei Sensori ---");

    // AHT20 (Temperatura e Umidità primarie per la compensazione)
    sensors_event_t hum_event, temp_event;
    if (aht20.getEvent(&hum_event, &temp_event)) {
        currentTempAHT20 = temp_event.temperature;
        currentHumAHT20 = hum_event.relative_humidity;
        Serial.printf("AHT20:  Temp = %.2f *C, Umidità = %.2f %%\n", currentTempAHT20, currentHumAHT20);
    } else {
        currentTempAHT20 = SENSOR_INVALID_TEMP;
        currentHumAHT20 = SENSOR_INVALID_HUM;
        Serial.println("AHT20:  [ERRORE] Lettura dati fallita.");
    }

    // MH-Z19B (CO2 e la sua temperatura interna)
    if (mhz19bSensor.measure() == 0) {
        co2MHZ19B = mhz19bSensor.getCO2();
        tempMHZ19B = mhz19bSensor.getTemperature();
        if (co2MHZ19B >= 400 && co2MHZ19B <= 10000) {
             Serial.printf("MH-Z19B: CO2 = %d ppm, Temp = %.1f *C\n", co2MHZ19B, tempMHZ19B);
        } else {
             Serial.printf("MH-Z19B: CO2 = %d ppm (valore fuori range, probabile riscaldamento), Temp = %.1f *C\n", co2MHZ19B, tempMHZ19B);
             co2MHZ19B = 0; // Invalida le letture di CO2 fuori range
        }
    } else {
        co2MHZ19B = 0;
        tempMHZ19B = SENSOR_INVALID_TEMP;
        Serial.println("MH-Z19B: [ERRORE] Lettura dati fallita.");
    }

    // ENS160 (eCO2, TVOC, AQI) - con compensazione di temperatura e umidità
    if (currentTempAHT20 != SENSOR_INVALID_TEMP && currentHumAHT20 != SENSOR_INVALID_HUM) {
        ens160.setTempAndHum(currentTempAHT20, currentHumAHT20);
    }
    uint8_t ens_status = ens160.getENS160Status();
    if (ens_status == 0 || ens_status == 1) { // 0=Normale, 1=Riscaldamento (fornisce comunque dati)
        eco2ENS160 = ens160.getECO2();
        tvocENS160 = ens160.getTVOC();
        aqiENS160 = ens160.getAQI();
        const char* status_str = (ens_status == 0) ? "Normale" : "Riscaldamento";
        Serial.printf("ENS160:  eCO2 = %d ppm, TVOC = %d ppb, AQI = %d (Stato: %s)\n", eco2ENS160, tvocENS160, aqiENS160, status_str);
    } else { // 2=Avvio Iniziale, 3=Output Invalido
        eco2ENS160 = 0; tvocENS160 = 0; aqiENS160 = 0;
        Serial.printf("ENS160:  Sensore non pronto (Stato: %d)\n", ens_status);
    }

    // BMP280 (Temperatura & Pressione)
    tempBMP280 = bmp280.readTemperature();
    pressureBMP280 = bmp280.readPressure() / 100.0F; // Converte da Pa a hPa
    if (!isnan(tempBMP280) && !isnan(pressureBMP280) && pressureBMP280 > 300 && pressureBMP280 < 1200) {
        Serial.printf("BMP280:  Temp = %.2f *C, Pressione = %.2f hPa\n", tempBMP280, pressureBMP280);
    } else {
        tempBMP280 = SENSOR_INVALID_TEMP; pressureBMP280 = 0;
        Serial.println("BMP280:  [ERRORE] Lettura dati fallita.");
    }

    // SPS30 (Particolato)
    uint16_t sps_data_ready;
    sps30HasValidData = false;
    if (sps30_read_data_ready(&sps_data_ready) == 0 && sps_data_ready) {
        if (sps30_read_measurement(&sps30_data) == 0) {
            sps30HasValidData = true;
            Serial.printf("SPS30:   PM2.5 = %.2f ug/m3, PM10 = %.2f ug/m3\n", sps30_data.mc_2p5, sps30_data.mc_10p0);
        } else {
            Serial.println("SPS30:   [ERRORE] Lettura misurazione fallita.");
        }
    } else {
        Serial.println("SPS30:   Dati non ancora pronti.");
    }
}

void processSensorAlerts() {
    bool alertState = false;
    Serial.print("--- Controllo Condizioni di Allerta... ");

    if (aqiENS160 >= AQI_ALERT_THRESHOLD) {
        alertState = true;
        Serial.print("Allerta AQI! ");
    }
    if (sps30HasValidData && sps30_data.mc_2p5 > PM25_ALERT_THRESHOLD && sps30_data.mc_10p0 >PM10_ALERT_THRESHOLD) {
        alertState = true;
        Serial.print("Allerta PM2.5! ");
    }

    digitalWrite(LED_PIN, alertState);
    if (alertState) {
        Serial.println("LED ACCESO.");
    } else {
        Serial.println("Nominale. LED SPENTO.");
    }
}

void writeToInfluxDB() {
    envDataPoint.clearFields();

    // --- Logica per la Media delle Temperature ---
    float temp_sum = 0.0f;
    int temp_count = 0;

    if (currentTempAHT20 != SENSOR_INVALID_TEMP) {
        temp_sum += currentTempAHT20;
        temp_count++;
    }
    if (tempBMP280 != SENSOR_INVALID_TEMP) {
        temp_sum += tempBMP280;
        temp_count++;
    }
    if (tempMHZ19B != SENSOR_INVALID_TEMP) {
        temp_sum += tempMHZ19B;
        temp_count++;
    }

    // Se almeno un sensore ha fornito una temperatura valida, calcola e aggiungi la media
    if (temp_count > 0) {
        float avgTemp = temp_sum / temp_count;
        envDataPoint.addField("temperature_avg", avgTemp);
    }

    // --- Aggiunta degli Altri Campi dei Sensori ---
    if (currentHumAHT20 != SENSOR_INVALID_HUM) envDataPoint.addField("humidity", currentHumAHT20);
    if (co2MHZ19B > 0) envDataPoint.addField("co2", co2MHZ19B);
    if (eco2ENS160 > 0) envDataPoint.addField("eco2", eco2ENS160);
    if (tvocENS160 >= 0) envDataPoint.addField("tvoc", tvocENS160);
    if (pressureBMP280 > 0) envDataPoint.addField("pressure", pressureBMP280);

    if (sps30HasValidData) {
        envDataPoint.addField("pm1p0", sps30_data.mc_1p0);
        envDataPoint.addField("pm2p5", sps30_data.mc_2p5);
        envDataPoint.addField("pm4p0", sps30_data.mc_4p0);
        envDataPoint.addField("pm10p0", sps30_data.mc_10p0);
        envDataPoint.addField("typical_particle_size", sps30_data.typical_particle_size);
    }

    // --- Scrittura su InfluxDB ---
    if (!envDataPoint.hasFields()) {
        Serial.println("--- InfluxDB: Nessun dato valido da inviare. Salto l'operazione di scrittura.");
        return;
    }

    Serial.println("--- Dati Pronti per l'Invio a InfluxDB ---");
    Serial.print("Dati Inviati (Formato Line Protocol): ");
    Serial.println(envDataPoint.toLineProtocol());

    if (!influxClient.writePoint(envDataPoint)) {
        Serial.print("[ERRORE] Scrittura su InfluxDB fallita: ");
        Serial.println(influxClient.getLastErrorMessage());
    } else {
        Serial.println("Scrittura su InfluxDB riuscita!");
    }
}