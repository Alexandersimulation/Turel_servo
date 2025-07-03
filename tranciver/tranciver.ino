/*
 * ПЕРЕДАТЧИК ДЛЯ СИСТЕМЫ УПРАВЛЕНИЯ 
 * Версия для LLCC68 с защитой от помех
 * Реализовано:
 * - Управление джойстиком
 * - Защита Network ID
 * - Обработка ошибок LLCC68
 * - Автоматическая калибровка
 */

#include <RadioLib.h>

// ============ КОНФИГУРАЦИЯ ============
#define LORA_CS   5      // Пин CS (Chip Select)
#define LORA_RX   7      // Пин RX (MISO)
#define LORA_TX   6      // Пин TX (MOSI)
#define LORA_SCK  2      // Пин SCK (опционально)

// Настройки безопасности
#define NETWORK_ID 0x12   // Общий идентификатор сети
#define TRANSMITTER_ID 1  // Уникальный ID передатчика
#define RECEIVER_ID 2     // ID приемника

// Настройки джойстика
const int JOYSTICK_X = 26;  // Пин оси X (ADC0)
const int JOYSTICK_Y = 27;  // Пин оси Y (ADC1)
const int BUTTON_1 = 0;     // Пин кнопки 1
const int BUTTON_2 = 1;     // Пин кнопки 2
const int DEADZONE = 20;    // Мертвая зона джойстика

// Создаем объект LLCC68
LLCC68 lora = new Module(LORA_CS, LORA_RX, LORA_TX);

// CRC-16-CCITT implementation (polynomial 0x1021)
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Структура защищенного пакета
#pragma pack(push, 1)
typedef struct {
  uint8_t header[2] = {0xAA, 0x55};  // Сигнатура пакета
  uint8_t network_id = NETWORK_ID;   // Идентификатор сети
  uint8_t sender_id = TRANSMITTER_ID;// ID отправителя
  uint8_t target_id = RECEIVER_ID;   // ID получателя
  uint8_t x;          // Положение X (0-180)
  uint8_t y;          // Положение Y (0-180)
  uint8_t speed;      // Скорость (60-120)
  uint8_t buttons;    // Состояние кнопок (битовая маска)
  uint16_t crc;       // Контрольная сумма
} SecurePacket;
#pragma pack(pop)

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Инициализация пинов
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);

  // Инициализация LoRa модуля
  Serial.println("Initializing LoRa...");
  int state = lora.begin(
    433.0,      // Частота 433 МГц
    125.0,      // Полоса пропускания 125 кГц
    9,          // Spreading Factor 9
    7,          // Coding Rate 4/7
    NETWORK_ID, // Sync Word (совпадает с Network ID)
    17,         // Мощность передачи 17 дБм (22 может быть слишком много)
    8,          // Длина преамбулы 8 символов
    0           // Усиление приемника (0 = авто)
  );

  // Обработка ошибок инициализации
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed: ");
    printLoraError(state);
    while(true);
  }

  // Калибровка джойстика
  calibrateJoystick();
  Serial.println("Transmitter ready!");
}

void loop() {
  static uint32_t lastSendTime = 0;
  if (millis() - lastSendTime < 50) return; // Троттлинг 20 Гц

  SecurePacket packet;
  
  // Чтение и обработка ввода
  int16_t rawX = analogRead(JOYSTICK_X) - 512;
  int16_t rawY = analogRead(JOYSTICK_Y) - 512;
  
  // Применение мертвой зоны
  packet.x = (abs(rawX) < DEADZONE) ? 90 : map(rawX, -512, 512, 0, 180);
  packet.y = (abs(rawY) < DEADZONE) ? 90 : map(rawY, -512, 512, 0, 180);
  
  // Расчет скорости
  int16_t deviation = max(abs(rawX), abs(rawY));
  packet.speed = map(deviation, 0, 512, 60, 120);
  
  // Состояние кнопок
  packet.buttons = (!digitalRead(BUTTON_1)) | ((!digitalRead(BUTTON_2)) << 1);
  
  // Расчет CRC (исключая поле crc)
  packet.crc = calculateCRC16((uint8_t*)&packet, sizeof(SecurePacket) - 2);
  
  // Отправка пакета
  int state = lora.startTransmit((uint8_t*)&packet, sizeof(SecurePacket));
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("Send error: ");
    printLoraError(state);
  }

  lastSendTime = millis();
}

// Калибровка центрального положения джойстика
void calibrateJoystick() {
  delay(1000); // Пауза для отпускания джойстика
  Serial.println("Calibrating joystick...");
  int x = 0, y = 0;
  for(int i=0; i<16; i++) {
    x += analogRead(JOYSTICK_X);
    y += analogRead(JOYSTICK_Y);
    delay(10);
  }
  Serial.print("Calibration values - X: ");
  Serial.print(x/16);
  Serial.print(", Y: ");
  Serial.println(y/16);
}

// Вывод ошибок LLCC68
void printLoraError(int state) {
  switch(state) {
    case RADIOLIB_ERR_WRONG_MODEM: 
      Serial.println("Wrong module type"); break;
    case RADIOLIB_ERR_INVALID_FREQUENCY: 
      Serial.println("Invalid frequency"); break;
    case RADIOLIB_ERR_INVALID_BANDWIDTH: 
      Serial.println("Invalid bandwidth"); break;
    default: 
      Serial.print("Error code: 0x"); 
      Serial.println(state, HEX); break;
  }
}