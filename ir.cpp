#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define SDA_PIN 21           // Пин SDA для I2C (если используется внешнее оборудование)
#define SCL_PIN 22           // Пин SCL для I2C
#define DEVICE_ADDR 0x68     // Адрес устройства (если используется DSP через I2C)

#define guitarSignalPin 34   // Пин для аналогового сигнала с гитары
#define dacPin 25            // Пин для аналогового вывода (ЦАП или PWM)

#define SD_CS_PIN 5          // Пин CS для SD карты (если используется)

File impulseFile;  // Файл импульсного отклика

void setup() {
  Serial.begin(115200);  // Для отладки
  pinMode(guitarSignalPin, INPUT); // Пин для гитарного сигнала
  pinMode(dacPin, OUTPUT);  // Пин для вывода сигнала

  // Инициализация SD карты
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Ошибка инициализации SD карты!");
    return;
  }
  Serial.println("SD карта инициализирована");

  // Открываем файл импульсного отклика
  impulseFile = SD.open("/impulse.raw", FILE_READ);
  if (!impulseFile) {
    Serial.println("Не удалось открыть файл импульса!");
    return;
  }
  Serial.println("Файл импульса открыт");
}

void loop() {
  int guitarSignal = analogRead(guitarSignalPin);  // Чтение сигнала с гитары
  int processedSignal = processSignal(guitarSignal);  // Обработка сигнала
  sendToDAC(processedSignal);  // Отправка обработанного сигнала на выход
  delay(10);  // Задержка для стабильной работы
}

int processSignal(int guitarSignal) {
  // Пример простого фильтра (уменьшаем громкость)
  int processedSignal = guitarSignal * 0.8;  // Уменьшаем громкость
  return processedSignal;
}

void sendToDAC(int processedSignal) {
  // Преобразование в аналоговый сигнал через PWM (если нет внешнего ЦАП)
  int outputSignal = constrain(processedSignal, 0, 255);  // Ограничиваем значение сигнала
  analogWrite(dacPin, outputSignal);  // Отправка сигнала через PWM на динамик
}
