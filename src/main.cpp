/*
  ESP8266 NodeMCU Pinout:
  #define D0 16
  #define D1 5 // I2C Bus SCL (clock)
  #define D2 4 // I2C Bus SDA (data)
  #define D3 0
  #define D4 2 // Same as "LED_BUILTIN", but inverted logic
  #define D5 14 // SPI Bus SCK (clock)
  #define D6 12 // SPI Bus MISO
  #define D7 13 // SPI Bus MOSI
  #define D8 15 // SPI Bus SS (CS)
  #define D9 3 // RX0 (Serial console)
  #define D10 1 // TX0 (Serial console)

Library:
- plerup/EspSoftwareSerial@^8.1.0
- emelianov/modbus-esp8266@^4.1.0

Devices:
1. Anemometer: 
  - VCC: 10 - 30 V
  - https://community.dfrobot.com/makelog-312253.html
  - https://github.com/philippedc/Arduino-ESP8266-RS485-MODBUS-Anemometer

2.Halisense NPK, EC, pH, Temperature, Soil Moisture Sensor: 
  - VCC: 4.5 - 30 V (0.5W @24VDC)
  - Suhu:
      Rentang pengukuran: -40℃-80℃
      Akurasi: ± 5℃ (25℃)
      Stabilitas jangka panjang: ≤ 0.1%℃/Y
      Waktu tanggapan: ≤ 15 detik

  - Kelembaban:
      Rentang pengukuran: 0-100% RH
      Akurasi: 2% dalam 0-50%, 3% dalam 50-100%
      Stabilitas jangka panjang: ≤ 1% RH/y
      Waktu tanggapan: ≤ 4 detik

  - Konduktivitas (EC):
      Rentang pengukuran: 0-20000us/cm
      Akurasi: ± 3% (0-10000 AS/cm); ± 5% (10000-20000 AS/cm)
      Stabilitas jangka panjang: ≤ 1% AS/cm
      Waktu tanggapan: ≤ 1 detik

  - PH
      Rentang pengukuran: 3-9 PH
      Akurasi: ± 0,3 PH
      Stabilitas jangka panjang: ≤ 5%/tahun
      Waktu tanggapan: ≤ 10 detik

  - Nitrogen, Fosfor, Kalium
      Rentang pengukuran: 1-1999 mg/kg(mg/L)
      Resolusi：1 mg/kg(mg/L)
      Waktu tanggapan:＜1S
  - Modbus RTU 4800,8N1
      - Read Input Register(0x04): 
          Start Addr: 0, Num of Coils: 9

Test Result:
* RS485 module with MAX3485ESA with auto direction control:
  - Power Supply: 3.3V 
  - MAX3485ESA rated voltage: 4.75V to 5.25V
  TX Result: 115200 baud: OK, 9600 baud: OK

* MAX13487EESA RS485 module with auto direction control:
  - Power Supply: 3.3V 
  - MAX13487ESA rated voltage: 4.75V to 5.25V
  - Pin 3: ~SHDN (Shutdown), Shutdown Supply Current: 10 uA
  TX Result: 115200 baud: OK, 9600 baud: OK  
*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ModbusRTU.h>

#define SW_UART_RXD  D1 // IO05
#define SW_UART_TXD  D2 // IO16

SoftwareSerial rs485;
ModbusRTU modbus;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(500);
  rs485.begin(4800, SWSERIAL_8N1, SW_UART_RXD, SW_UART_TXD, false, 64);
  // bool fInit = modbus.begin(&rs485);
  // modbus.master();
  // Serial.printf("ModbusRTU master init: %s\n",fInit?"OK":"Failed");
}

void readSensorData()
{
  static uint16_t arSoilSensorData[9]={0}; 
  modbus.readIreg(1, 0x00, arSoilSensorData, 9, [](Modbus::ResultCode event, uint16_t , void*)->bool{
    bool fSuccess = (event==Modbus::EX_SUCCESS);
    if (fSuccess)
    {
      Serial.printf("Temp: %.2f VWC: %.2f%% EC:%.2f ms/cm Salinity: %d mg/L TDS: %d mg/L Epsilon: %.2f\n",
        arSoilSensorData[0]/100.0f, arSoilSensorData[1]/100.0f, arSoilSensorData[2]/1000.0f,
        arSoilSensorData[3], arSoilSensorData[4], arSoilSensorData[5]/100.0f);
    }
    else
    {
      Serial.printf("Read sensor data failed: %d\n", event);
    }
    return fSuccess;
  });
}

// int nCount=0;
u_long tLast=0;
int nCount=0;
void loop() {
  // modbus.task();
  u_long tNow = millis();
  if (tNow - tLast > 1000) {
    tLast = tNow;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Serial.printf("Hello World!\n");
    // rs485.printf("%03d Hello World!\r\n", nCount);
    // nCount++;
  }
  yield();
}

