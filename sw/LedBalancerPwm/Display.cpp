/*
Author:  Stefano Di Paolo
License: MIT, https://en.wikipedia.org/wiki/MIT_License
Date:    2017-12-31

Led Balancer (http://www.microentropie.com)
Arduino nano based.

Main project entry.
The following libraries are required:
* LiquidCrystal-I2C (https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library)
* ManyButtons (https://github.com/microentropie/ManyButtons)
* IEC60063 (https://github.com/microentropie/IEC60063)

Sources repository: https://github.com/microentropie/
*/

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


#include <Arduino.h>
#include "display.h"

#include "src/Macro_BuildDATE.h"

// There may be different Serial interfaces for HD44780 displays:
// https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/schematics
// I2C adapter: PCF8574, A0 A1 A2 pull-up (contact open => address: 0x27)

// Address 0x27, 20 chars, 4 line display
LiquidCrystal_I2C lcd(0x27, QtColums, QtRows);
// Remember to set the contrast, or you may see nothing !

// display with cache
static String Rows[QtRows] = { String(""), String(""), String(""), String("") };


void display_init(const char *PROGRAM)
{
  // initialize the LCD
  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.setCursor(0, 0);   // Set the cursor to col 0, row 0
  lcd.print(PROGRAM);
  lcd.setCursor(0, 1);   // Set the cursor to col 0, row 1
  //lcd.print(F(__DATE__));
  Arduino_DATE_print(lcd);
  lcd.setCursor(0, 2);   // Set the cursor to col 0, row 2
  lcd.print(F(__TIME__));
  lcd.setCursor(0, 3);   // Set the cursor to col 0, row 3
  lcd.print(F("(C)microentropie.com"));

  // init display cache
  for (int i = 0; i < QtRows; ++i)
    Rows[i] = String("");
}

void DisplayIfChanged(byte row, const char *pNewValues)
{
  String newValues = pNewValues;
  DisplayIfChanged(row, newValues);
}

void DisplayIfChanged(byte row, String &newValues)
{
  if (newValues.length() > QtColums)
    newValues = newValues.substring(0, QtColums);
  else
    while (newValues.length() < QtColums)
      newValues.concat(" ");

  if (newValues != Rows[row])
  {
    Rows[row] = newValues;
    lcd.setCursor(0, row);
    lcd.print(newValues);
  }
}
