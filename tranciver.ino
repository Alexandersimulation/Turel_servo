//Передатчик
#define ARDUINO_USB_MODE 1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Светодиод
#define LED_PIN 16
#define LED_COUNT 1
Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Пины управления
#define LORA_AUX_PIN 2       // Пин AUX LoRa-модуля
#define BUTTON1_PIN 3        // Первая кнопка (реле 1)
#define BUTTON2_PIN 4        // Вторая кнопка (реле 2)
#define JOY_X_PIN A0         // Джойстик X
#define JOY_Y_PIN A1         // Джойстик Y

// Настройки передачи
#define TX_INTERVAL_MS 50
#define AUX_WAIT_TIMEOUT 200
#define JOYSTICK_DEADZONE 10
#define IDLE_TIMEOUT_MS 3000

// Структура пакета
#pragma pack(push, 1)
struct ControlPacket {
  uint8_t header = 0x55;
  uint8_t x;
  uint8_t y;
  uint8_t button1;
  uint8_t button2;
  uint8_t checksum;
};
#pragma pack(pop)

// Состояние
uint32_t lastTxTime = 0;
uint32_t lastActivityTime = 0;
bool isTransmitting = true;

void waitForLORAReady() {
  uint32_t start = millis();
  while (digitalRead(LORA_AUX_PIN) == LOW) {
    if (millis() - start > AUX_WAIT_TIMEOUT) break;
    delay(1);
  }
}

uint8_t calculateChecksum(const ControlPacket& pkt) {
  return pkt.x ^ pkt.y ^ pkt.button1 ^ pkt.button2;
}

bool isInZeroPosition(uint8_t x, uint8_t y, uint8_t b1, uint8_t b2) {
  return (abs(x - 128) < JOYSTICK_DEADZONE) &&
         (abs(y - 128) < JOYSTICK_DEADZONE) &&
         b1 == 0 && b2 == 0;
}

void setup() {
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LORA_AUX_PIN, INPUT);

  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();
  delay(200);
  pixel.clear();
  pixel.show();

  Serial1.begin(9600);
  delay(500);

  pixel.setPixelColor(0, pixel.Color(0, 255, 255));
  pixel.show();
  delay(200);
  pixel.clear();
  pixel.show();
}

void loop() {
  ControlPacket pkt;
  pkt.x = map(analogRead(JOY_X_PIN), 0, 1023, 0, 255);
  pkt.y = map(analogRead(JOY_Y_PIN), 0, 1023, 0, 255);
  pkt.button1 = !digitalRead(BUTTON1_PIN);
  pkt.button2 = !digitalRead(BUTTON2_PIN);
  pkt.checksum = calculateChecksum(pkt);

  if (!isInZeroPosition(pkt.x, pkt.y, pkt.button1, pkt.button2)) {
    lastActivityTime = millis();
    if (!isTransmitting) isTransmitting = true;
  }

  if (isTransmitting && millis() - lastActivityTime > IDLE_TIMEOUT_MS) {
    isTransmitting = false;
  }

  if (isTransmitting && millis() - lastTxTime >= TX_INTERVAL_MS) {
    if (digitalRead(LORA_AUX_PIN) == HIGH) {
      Serial1.write((uint8_t*)&pkt, sizeof(pkt));
      waitForLORAReady();
      lastTxTime = millis();
    }
  }

  delay(10);
}
