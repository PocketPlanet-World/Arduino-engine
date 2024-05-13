#include <Wire.h> //I2C通訊函式庫
#include <AHT10.h> //AHT10函式庫
#include <Arduino.h> 

#define SoilM A0
#define SoilT A1
#define Phpin A2
#define PumpPin 13

AHT10 myAHT10(0x38); // AHT10 I2C通訊位址

void setup() {
  Wire.begin(); 
  Serial.begin(9600); 
  Serial.println("Initialising AHT10 Sensor"); //初始化AHT10感測器
  if (!myAHT10.begin()) { 
    Serial.println("Sensor error!"); //錯誤發送
    while (1);
  }
  pinMode(PumpPin, OUTPUT);
}

void loop() {

delay(1000);
  if (Serial.available() > 0)
  {
    // 當串口可用時，即樹莓派發送請求時向樹莓派發送數據
    String receivedData = Serial.readStringUntil('\n');
    if (receivedData.startsWith("request_data"))
    {

  float t = myAHT10.readTemperature(); //定義空氣溫度變數
  float h = myAHT10.readHumidity(); //定應空氣濕度變數

  int SoilMe = analogRead(SoilM);
  int SoilTe = analogRead(SoilT);
  int PhV = analogRead(Phpin);

  float ASM = map(SoilMe, 0, 1023, 0, 100); //感測器數值轉換土壤濕度
  float AST = map(SoilTe, 0, 1023, -10, 50); //感測器數值轉換土壤溫度
  float APV = map(PhV, 0, 1023, 0, 14); //感測器數值轉換PH

      Serial.print("airhumidity: ");
      Serial.print(h);
      Serial.print(" \t , ");
      Serial.print("airtemperature: ");
      Serial.print(t);
      Serial.print(" \t , ");
      Serial.print("SoilMoisture: ");
      Serial.print(ASM);
      Serial.print(" \t , ");
      Serial.print("SoilTemperature: ");
      Serial.print(AST);
      Serial.print(" \t , ");
      Serial.print("PH: ");
      Serial.print(APV);
      Serial.println();
    } // loop()
    else if (receivedData.startsWith("control_pump"))
    {
      // 如果樹莓派發送控制水泵的指令
      digitalWrite(PumpPin, HIGH); // 開啟水泵
      delay(5000);                 // 水泵工作5秒鐘
      digitalWrite(PumpPin, LOW);  // 關閉水泵
    }
   }
  }