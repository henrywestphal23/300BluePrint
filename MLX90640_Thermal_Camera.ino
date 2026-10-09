#include "Arduino.h"
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define EMMISIVITY 0.97 //how well an object reflects thermal radiation
#define TA_SHIFT 8 //ambient temperature of the device?

paramsMLX90640 mlx90640;
const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
static float tempValues[32 * 24];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0) {
    Serial.println("MLX90640 not detected at default I2C address. Starting scan the device addr...");
    Device_Scan();
//    while(1);
  }
  else {
    Serial.println("MLX90640 online!");
  }
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) Serial.println("Parameter extraction failed");
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); 
  Wire.setClock(800000);
}

void loop(void) {
  readTempValues();
  delay(30); //delay in milliseconds, min of 15? 64Hz
}

void readTempValues() {
  int Tpix = 0;
  for (byte x = 0 ; x < 2 ; x++) 
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; 

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues); //able to add thresholding to this function, prevents recursion
  }



  Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
  for (int i = 0; i < 768; i++) {
    if (((i % 32) == 0) && (i != 0)) {
      Serial.println(" ");
    }
    if ((int)tempValues[i] > 0) {
      Serial.print("█");
      Tpix++;
    }
    else {
      Serial.print(" ");
    }
    // if ((int)tempValues[i] <= 35 && (int)tempValues[i] > 30) {
    //   Serial.print("█");
    // }
    // else if ((int)tempValues[i] <= 30 && (int)tempValues[i] > 25) {
    //   Serial.print("▓");
    // }
    // else if ((int)tempValues[i] <= 25 && (int)tempValues[i] > 20) {
    //   Serial.print("▒");
    // }
    // else if ((int)tempValues[i] <= 20) {
    //   Serial.print("░");
    // }
    // else {
    //   Serial.print("∙");
    // }
    Serial.print(" ");
  }
  Serial.println(Tpix);
  Serial.println("\r\n===========================WaveShare MLX90640 Thermal Camera===============================");
}

void Device_Scan() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("done");
}
