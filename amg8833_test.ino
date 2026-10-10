#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);   // SDA, SCL

  if (!amg.begin()) {
    Serial.println("AMG8833 not found!");
    while (1);
  }

  Serial.println("AMG8833 connected!");
}

void loop() {
  float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

  amg.readPixels(pixels);

  for (int i = 0; i < 64; i++) {
    Serial.print(pixels[i]);
    Serial.print(" ");

    if ((i + 1) % 8 == 0)
      Serial.println();
  }

  Serial.println();
  delay(1000);
}
