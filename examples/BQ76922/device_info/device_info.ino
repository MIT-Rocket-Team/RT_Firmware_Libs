#include "BQ76942.h"
#include "Wire.h"

BQ76922 bms(&Wire, 0x08);

void setup() {
  // put your setup code here, to run once:
  Serial.setTx(PA9);
  Serial.setRx(PA10);
  Serial.begin(115200);
  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  bms.begin();
  delay(100);
  bms.enterConfigMode();
  bms.cellConfig(3);
  bms.minCellVoltage();
  bms.cellUVOnly();
  //bms.disableProtections();
  bms.ddsgConfig();
  //bms.dfetoffConfig();
  bms.enableFet();
  bms.exitConfigMode();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("FET STATUS: 0x");
  Serial.println(bms.fetStatus(), HEX);
  delay(2000);
}
