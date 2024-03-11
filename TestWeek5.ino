#define BLYNK_TEMPLATE_ID "user11"
#define BLYNK_TEMPLATE_NAME "user11@server.wyns.it"
#define BLYNK_PRINT Serial

int pin = 4;
const int potPin = 35;
int potVal = 0;
int stepHValue = 0;
int lastChangedValue = 0;  // Houdt de laatst gewijzigde waarde bij
unsigned long lastUpdateTime = 0;  // Houdt de tijd van de laatste update bij

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

#define VIRTUAL_PIN_TEMPERATURE V0
#define VIRTUAL_PIN_STEP_H V2

Adafruit_BMP280 bmp;  // I2C

char auth[] = "tvT2lhZZp5DGrswafZQlCbykCpasbL4k";
char ssid[] = "embed";
char pass[] = "weareincontrol";

BLYNK_WRITE(VIRTUAL_PIN_STEP_H) {
  // Get the value from the Step H widget
  stepHValue = param.asInt();
  lastChangedValue = stepHValue;  // Update lastChangedValue
  lastUpdateTime = millis();  // Update lastUpdateTime
}

void updatePotValue() {
  // Lees de waarde van de fysieke potentiometer
  int currentPotVal = map(analogRead(potPin), 0, 1023, 10, 15);
  
  // Controleer of de waarde is gewijzigd en of het al meer dan 2 seconden geleden is
  if (currentPotVal != potVal && (millis() - lastUpdateTime) > 2000) {
    potVal = currentPotVal;
    lastChangedValue = potVal;  // Update lastChangedValue
    lastUpdateTime = millis();  // Update lastUpdateTime
  }
}

void setup() {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  Serial.begin(115200);

  updatePotValue();  // Initialisatie van potVal

  Blynk.virtualWrite(VIRTUAL_PIN_TEMPERATURE, bmp.readTemperature());

  delay(10);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);
  int wifi_ctr = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");

  Blynk.begin(auth, ssid, pass, "server.wyns.it", 8081);

  while (!Serial)
    delay(100);
  Serial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1)
      delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void loop() {
  updatePotValue();  // Update potVal

  Blynk.run();

  Serial.print("gewenste temperatuur:  ");
  Serial.print(lastChangedValue);
  Serial.println("*C");

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  if (lastChangedValue >= bmp.readTemperature()) {
    digitalWrite(pin, HIGH);
  } else {
    digitalWrite(pin, LOW);
  }

  Blynk.virtualWrite(VIRTUAL_PIN_TEMPERATURE, bmp.readTemperature());

  delay(2000);
}
