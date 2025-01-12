#include <Arduino.h>
#include <Wire.h> // Arduino library for I2C - see https://docs.arduino.cc/language-reference/en/functions/communication/wire/
#include "lee_ventus_spm_i2c.h"

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

int switchState = 0;
float measured_power = 0, measured_pressure = 0;


void setup() {
  Serial.begin(9600); //initialize serial communication
  Serial.println("SPM I2C demo");

  Wire.begin(); // join i2c bus (address optional for master)

  pinMode(3, OUTPUT);
  //pinMode(4, OUTPUT);
  //pinMode(5, OUTPUT);

  pinMode(2, INPUT);

  lcd.begin(16, 2);

}

void loop() {
// -----------------------------------------------------------------------------
// Manual control example
// -----------------------------------------------------------------------------
  
  switchState = digitalRead(2);
  
  if (switchState == LOW) {
    digitalWrite(3, LOW);
    //digitalWrite(4, HIGH);
    //digitalWrite(5, LOW);
    spm_i2c_write_int16(SPM_DEFAULT_I2C_ADDRESS, REGISTER_PUMP_ENABLE, 0);

    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Pump is off");

  }
  else {
    digitalWrite(3, HIGH);
    //digitalWrite(4, HIGH);
    //digitalWrite(5, HIGH);

    Serial.print("\n\nMANUAL POWER CONTROL");

    //set to manual power control
    spm_i2c_setup_manual_power_control(SPM_DEFAULT_I2C_ADDRESS);
    //and enable pump
    spm_i2c_write_int16(SPM_DEFAULT_I2C_ADDRESS, REGISTER_PUMP_ENABLE, 1);

    String target_power_display, measured_power_display, measured_pressure_display;

    //cycle through different power levels  
    for (int target_power = 100; target_power <= 900; target_power += 100)
    {
      spm_i2c_write_float(SPM_DEFAULT_I2C_ADDRESS, REGISTER_SET_VAL, target_power);
      delay(100);

      Serial.print("\nTarget power: ");
      Serial.print(target_power);
      Serial.print("mW");

      // LCD print target power
      lcd.clear();
      lcd.setCursor(0, 0);
      target_power_display = "Trgt Pwr= " + String(target_power) + " mW"; // 2 decimal places
      lcd.print(target_power_display);
      delay(1500);
      lcd.clear();

      for(int j=0; j<4; j++)
      {
        //float measured_power, measured_pressure;
        //String measured_power_display, measured_pressure_display;

        measured_power = spm_i2c_read_float(SPM_DEFAULT_I2C_ADDRESS, REGISTER_MEAS_DRIVE_MILLIWATTS);
        measured_pressure = spm_i2c_read_float(SPM_DEFAULT_I2C_ADDRESS, REGISTER_MEAS_DIGITAL_PRESSURE);

        Serial.print("\nPower: ");
        Serial.print(measured_power);
        Serial.print(" mW, ");
        Serial.print("Pressure: ");
        Serial.print(measured_pressure);
        Serial.print(" mbar");

        measured_power_display = "Pwr = " + String(measured_power, 2) + " mW"; // 2 decimal places
        lcd.setCursor(0, 0);
        lcd.print(measured_power_display);

        measured_pressure_display = "Pres = " + String(measured_pressure, 2) + " mbar"; // 2 decimal places
        lcd.setCursor(0, 1);
        lcd.print(measured_pressure_display);

        delay(1500);
      }
    }

    lcd.clear();

  }
}
