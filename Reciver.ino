//Приёмник
#define ARDUINO_USB_MODE 1

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Светодиод
#define LED_PIN 16
#define LED_COUNT 1
Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Пины
#define LORA_AUX_PIN 2
#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10
#define RELAY1_PIN 11       // Реле 1
#define RELAY2_PIN 12       // Реле 2 

// Параметры сервоприводов
#define SERVO_360_STOP 1500
#define SERVO_360_MIN 1300
#define SERVO_360_MAX 1700
#define SERVO_DEADZONE 10

// Тайминги
#define PACKET_TIMEOUT_MS 3500
#define SERVO_UPDATE_MS 20

// Логика реле
#define RELAY1_ACTIVE_HIGH false    // Реле 1 включается LOW
#define RELAY2_ACTIVE_HIGH false    // Реле 2 включается LOW

uint8_t relayMode1 = 0;             // 0 = импульс, 1 = удержание
uint8_t relayMode2 = 0;             // 0 = импульс, 1 = удержание

uint32_t relay1PulseDuration = 1000;  // мс
uint32_t relay2PulseDuration = 1000;   // мс

// Структура пакета
#pragma pack(push, 1)
struct ControlPacket {
  uint8_t header;
  uint8_t x;
  uint8_t y;
  uint8_t button1;
  uint8_t button2;
  uint8_t checksum;
};
#pragma pack(pop)

// Состояние
uint32_t lastPacketTime = 0;
int currentPulseX = SERVO_360_STOP;
int currentPulseY = SERVO_360_STOP;
bool connectionActive = false;

bool relay1State = false;
bool relay2State = false;
uint32_t relay1OffTime = 0;
uint32_t relay2OffTime = 0;

void writeServoPulse(uint8_t pin, int pulseWidth) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
}

bool validatePacket(const ControlPacket& pkt) {
  return pkt.header == 0x55 &&
         pkt.checksum == (pkt.x ^ pkt.y ^ pkt.button1 ^ pkt.button2);
}

void setup() {
  pinMode(SERVO_X_PIN, OUTPUT);
  pinMode(SERVO_Y_PIN, OUTPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LORA_AUX_PIN, INPUT);

  // Выключить оба реле при старте
  digitalWrite(RELAY1_PIN, RELAY1_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(RELAY2_PIN, RELAY2_ACTIVE_HIGH ? LOW : HIGH);

  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0, 255, 0));
  pixel.show();
  delay(200);
  pixel.clear();
  pixel.show();

  Serial1.begin(9600);
  delay(500);

  pixel.setPixelColor(0, pixel.Color(255, 255, 0));
  pixel.show();
  delay(200);
  pixel.clear();
  pixel.show();

  for (int i = 0; i < 10; i++) {
    writeServoPulse(SERVO_X_PIN, SERVO_360_STOP);
    writeServoPulse(SERVO_Y_PIN, SERVO_360_STOP);
    delay(20);
  }
}

void loop() {
  if (Serial1.available() >= sizeof(ControlPacket)) {
    ControlPacket pkt;
    Serial1.readBytes((uint8_t*)&pkt, sizeof(pkt));

    if (validatePacket(pkt)) {
      lastPacketTime = millis();
      connectionActive = true;

      int joyX = (int)pkt.x - 128;
      int joyY = (int)pkt.y - 128;

      currentPulseX = (abs(joyX) > SERVO_DEADZONE)
        ? map(abs(joyX), SERVO_DEADZONE, 127, SERVO_360_STOP, (joyX > 0) ? SERVO_360_MAX : SERVO_360_MIN)
        : SERVO_360_STOP;

      currentPulseY = (abs(joyY) > SERVO_DEADZONE)
        ? map(abs(joyY), SERVO_DEADZONE, 127, SERVO_360_STOP, (joyY > 0) ? SERVO_360_MAX : SERVO_360_MIN)
        : SERVO_360_STOP;

      // ==== Реле 1 ====
      if (pkt.button1 && !relay1State) {
        digitalWrite(RELAY1_PIN, RELAY1_ACTIVE_HIGH ? HIGH : LOW);
        relay1State = true;
        if (relayMode1 == 0) relay1OffTime = millis() + relay1PulseDuration;
      } else if (!pkt.button1 && relayMode1 == 1 && relay1State) {
        digitalWrite(RELAY1_PIN, RELAY1_ACTIVE_HIGH ? LOW : HIGH);
        relay1State = false;
      }

      // ==== Реле 2 ====
      if (pkt.button2 && !relay2State) {
        digitalWrite(RELAY2_PIN, RELAY2_ACTIVE_HIGH ? HIGH : LOW);
        relay2State = true;
        if (relayMode2 == 0) relay2OffTime = millis() + relay2PulseDuration;
      } else if (!pkt.button2 && relayMode2 == 1 && relay2State) {
        digitalWrite(RELAY2_PIN, RELAY2_ACTIVE_HIGH ? LOW : HIGH);
        relay2State = false;
      }
    }
  }

  // Выключение по таймеру (импульсный режим)
  if (relay1State && relayMode1 == 0 && millis() >= relay1OffTime) {
    digitalWrite(RELAY1_PIN, RELAY1_ACTIVE_HIGH ? LOW : HIGH);
    relay1State = false;
  }
  if (relay2State && relayMode2 == 0 && millis() >= relay2OffTime) {
    digitalWrite(RELAY2_PIN, RELAY2_ACTIVE_HIGH ? LOW : HIGH);
    relay2State = false;
  }

  // Потеря связи
  if (connectionActive && millis() - lastPacketTime > PACKET_TIMEOUT_MS) {
    connectionActive = false;
    currentPulseX = SERVO_360_STOP;
    currentPulseY = SERVO_360_STOP;
    digitalWrite(RELAY1_PIN, RELAY1_ACTIVE_HIGH ? LOW : HIGH);
    digitalWrite(RELAY2_PIN, RELAY2_ACTIVE_HIGH ? LOW : HIGH);
    relay1State = false;
    relay2State = false;
  }

  // Обновление сервоприводов
  static uint32_t lastServoUpdate = 0;
  if (millis() - lastServoUpdate >= SERVO_UPDATE_MS) {
    writeServoPulse(SERVO_X_PIN, currentPulseX);
    writeServoPulse(SERVO_Y_PIN, currentPulseY);
    lastServoUpdate = millis();
  }

  delay(1);
}
