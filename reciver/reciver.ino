/*
 * ПРИЕМНИК ДЛЯ СИСТЕМЫ УПРАВЛЕНИЯ 
 * Версия для LLCC68 с защитой от помех
 * Реализовано:
 * - Проверка Network ID и отправителя
 * - Контроль целостности CRC-16
 * - Таймаут связи 500 мс
 * - Управление сервоприводами и устройствами
 */

#include <RadioLib.h>
#include <Servo.h>

// ============ КОНФИГУРАЦИЯ ============
#define LORA_CS   5      // Пин CS (Chip Select)
#define LORA_RX   7      // Пин RX (MISO)
#define LORA_TX   6      // Пин TX (MOSI)
#define LORA_SCK  2      // Пин SCK (опционально)

// Настройки безопасности (должны совпадать с передатчиком)
#define NETWORK_ID 0x12   // Общий идентификатор сети
#define RECEIVER_ID 2     // ID этого устройства
#define ALLOWED_SENDER 1  // Разрешенный отправитель

// Настройки сервоприводов
const int SERVO_X_PIN = 9;
const int SERVO_Y_PIN = 10;
const int DEVICE_1_PIN = 3;
const int DEVICE_2_PIN = 4;
const uint32_t TIMEOUT_MS = 500; // Таймаут связи

// Создаем объекты
LLCC68 lora = new Module(LORA_CS, LORA_RX, LORA_TX);
Servo servoX, servoY;

// Структура пакета (должна совпадать с передатчиком)
#pragma pack(push, 1)
typedef struct {
  uint8_t header[2];     // Сигнатура пакета (0xAA, 0x55)
  uint8_t network_id;    // Идентификатор сети
  uint8_t sender_id;     // ID отправителя
  uint8_t target_id;     // ID получателя
  uint8_t x;            // Положение X (0-180)
  uint8_t y;            // Положение Y (0-180)
  uint8_t speed;        // Скорость (60-120)
  uint8_t buttons;      // Состояние кнопок (битовая маска)
  uint16_t crc;         // Контрольная сумма
} SecurePacket;
#pragma pack(pop)

// Реализация CRC-16-CCITT
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

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Инициализация сервоприводов и выходов
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  pinMode(DEVICE_1_PIN, OUTPUT);
  pinMode(DEVICE_2_PIN, OUTPUT);
  stopServos();

  // Инициализация LoRa модуля
  Serial.println("Initializing LoRa receiver...");
  int state = lora.begin(
    433.0,      // Частота 433 МГц
    125.0,      // Полоса пропускания 125 кГц
    9,          // Spreading Factor 9
    7,          // Coding Rate 4/7
    NETWORK_ID, // Sync Word
    17,         // Мощность передачи (17 dBm)
    8,          // Длина преамбулы
    0           // Усиление приемника
  );

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed: ");
    printLoraError(state);
    while(true);
  }

  Serial.println("Receiver ready!");
}

void loop() {
  static uint32_t lastPacketTime = 0;
  
  // Проверка таймаута связи
  if (millis() - lastPacketTime > TIMEOUT_MS) {
    stopServos();
    digitalWrite(DEVICE_1_PIN, LOW);
    digitalWrite(DEVICE_2_PIN, LOW);
    return;
  }

  SecurePacket packet;
  int state = lora.receive((uint8_t*)&packet, sizeof(SecurePacket));
  
  if (state == RADIOLIB_ERR_NONE) {
    // Проверка заголовка пакета
    if (packet.header[0] != 0xAA || packet.header[1] != 0x55) {
      Serial.println("Invalid packet header");
      return;
    }
    
    // Проверка Network ID
    if (packet.network_id != NETWORK_ID) {
      Serial.println("Wrong network ID");
      return;
    }
    
    // Проверка получателя
    if (packet.target_id != RECEIVER_ID) {
      Serial.println("Packet not for this receiver");
      return;
    }
    
    // Проверка отправителя
    if (packet.sender_id != ALLOWED_SENDER) {
      Serial.println("Unauthorized sender");
      return;
    }
    
    // Проверка CRC
    uint16_t receivedCrc = packet.crc;
    packet.crc = 0; // Обнуляем для расчета
    uint16_t calculatedCrc = calculateCRC16((uint8_t*)&packet, sizeof(SecurePacket)-2);
    if (calculatedCrc != receivedCrc) {
      Serial.println("CRC mismatch");
      return;
    }
    
    lastPacketTime = millis();
    processPacket(packet);
  }
  else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.print("Receive error: ");
    printLoraError(state);
  }
}

void processPacket(SecurePacket p) {
  // Управление сервоприводами
  servoX.write(map(p.x, 0, 180, 0, 180)); // Преобразование для сервопривода
  servoY.write(map(p.y, 0, 180, 0, 180));
  
  // Управление устройствами по кнопкам
  digitalWrite(DEVICE_1_PIN, p.buttons & 0x01);
  digitalWrite(DEVICE_2_PIN, (p.buttons >> 1) & 0x01);
  
  // Отладочный вывод
  Serial.print("Valid packet from ");
  Serial.print(p.sender_id);
  Serial.print(": X=");
  Serial.print(p.x);
  Serial.print(", Y=");
  Serial.print(p.y);
  Serial.print(", Speed=");
  Serial.print(p.speed);
  Serial.print(", Buttons=");
  Serial.println(p.buttons, BIN);
}

void stopServos() {
  servoX.write(90); // Нейтральное положение
  servoY.write(90);
}

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