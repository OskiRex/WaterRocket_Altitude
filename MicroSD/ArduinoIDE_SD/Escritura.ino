#include <SPI.h>
#include <SD.h>
// Prueba de escritura y lectura de SD con modulo a ESP32
// Pines SPI para ESP32
#define PIN_SD_MISO  2
#define PIN_SD_MOSI 15
#define PIN_SD_SCK  14
#define PIN_SD_CS   13

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("================================");
    Serial.println("Prueba de tarjeta microSD");
    Serial.println("================================");

    SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

    if (!SD.begin(PIN_SD_CS))
    {
        Serial.println("ERROR: No se pudo inicializar la tarjeta SD.");
        return;
    }

    Serial.println("SD inicializada correctamente.");

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print("Capacidad: ");
    Serial.print(cardSize);
    Serial.println(" MB");

    File file = SD.open("/log.txt", FILE_WRITE);

    if (!file)
    {
        Serial.println("ERROR: No se pudo abrir log.txt");
        return;
    }

    file.println("Hola JHON desde ESP32");
    file.println("Prueba de escritura correcta.");
    file.close();

    Serial.println("Archivo escrito correctamente.");

    // Leer el archivo
    file = SD.open("/log.txt");

    if (!file)
    {
        Serial.println("ERROR: No se pudo abrir log.txt para lectura.");
        return;
    }

    Serial.println();
    Serial.println("Contenido del archivo:");
    Serial.println("----------------------");

    while (file.available())
    {
        Serial.write(file.read());
    }

    file.close();

    Serial.println();
    Serial.println("----------------------");
}

void loop()
{
}