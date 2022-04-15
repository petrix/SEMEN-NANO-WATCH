#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "TimeLib.h"
#include <iarduino_RTC.h>
#include <EncButton2.h>
EncButton2<EB_ENCBTN> enc(INPUT_PULLUP, 2, 3, 4);  // энкодер с кнопкой

#define TIME_HEADER   'T'   // Header tag for serial time sync message

#define HOUR   'H'
#define MIN   'I'
#define SEC   'S'
#define DAY   'D'
#define MONTH   'M'
#define YEAR   'Y'
unsigned long currentTime;
unsigned long syncTime;
unsigned long backlightTimeout;
unsigned long unixTime;

//  Определяем системное время:                           // Время загрузки скетча.
const char* strM = "JanFebMarAprMayJunJulAugSepOctNovDec"; // Определяем массив всех вариантов текстового представления текущего месяца.

const char* sysT = __TIME__;                              // Получаем время компиляции скетча в формате "SS:MM:HH".
const char* sysD = __DATE__;                              // Получаем дату  компиляции скетча в формате "MMM:DD:YYYY", где МММ - текстовое представление текущего месяца, например: Jul.
//  Парсим полученные значения sysT и sysD в массив i:    // Определяем массив «i» из 6 элементов типа int, содержащий следующие значения: секунды, минуты, часы, день, месяц и год компиляции скетча.
const int i[6] {
  (sysT[6] - 48) * 10 + (sysT[7] - 48),
  (sysT[3] - 48) * 10 + (sysT[4] - 48),
  (sysT[0] - 48) * 10 + (sysT[1] - 48),
  //  (sysD[4] - 48) * 10 + (sysD[5] - 48),
  (sysD[5] - 48),
  ((int)memmem(strM, 36, sysD, 3) + 3 - (int)&strM[0]) / 3,
  (sysD[9] - 48) * 10 + (sysD[10] - 48)
};

//const byte minOffset = 60;
//const unsigned int hourOffset = (unsigned int) 60 * 60;
//const unsigned long dayOffset = (unsigned long) 60 * 60 * 24;
//const unsigned long monthOffset = (unsigned long) 60 * 60 * 24 * 30;
//const unsigned long yearOffset = (unsigned long) 60 * 60 * 24 * 30 * 12;


iarduino_RTC rtc(RTC_DS1302, 8, 9, 10);
LiquidCrystal_I2C lcd(0x27, 16, 2);
//uint8_t selector[8] = {0x10, 0x09, 0x05, 0x03, 0x0F, 0x00, 0x00, 0x00};
uint8_t arrow[8] = {0x04, 0x02, 0x09, 0x02, 0x04, 0x00, 0x00, 0x00};
byte dd[] = {0x0E, 0x0A, 0x0E, 0x00, 0x0E, 0x0A, 0x0E, 0x00};
uint8_t s3[8] = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00};
uint8_t s2[8] = {0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00};
uint8_t s1[8] = {0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00};
uint8_t s0[8] = {0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
void setup() {
  //  delay(300);

  //  lcd.cursor_on();
  //lcd.blink_on();
  rtc.begin();
  //  rtc.settime(i[0], i[1], i[2], i[3], i[4], i[5]);
  setTime(rtc.Unix);
  //  Serial.begin(115200);
  Serial.begin(9600);
  while (!Serial);  // Wait for Arduino Serial Monitor to open
  //  delay(100);
  if (timeStatus() != timeSet) {
    Serial.println("Unable to sync with the RTC");
  } else {
    Serial.println("RTC has set the system time");
  }
  Serial.println(rtc.Unix);
  Serial.println(rtc.gettime("d-M-Y, H:i:s"));
  ///////////////////////////////////////////////////////////
  Serial.println(sysT);
  Serial.println(sysD);

  pinMode(13, OUTPUT);
  setSyncProvider(requestSync);  //set function to call when sync required
  setSyncInterval(100);

  backlightTimeout = millis();

  lcd.init();
  lcd.backlight();
  lcd.display();
  lcd.createChar(0, s0);
  lcd.createChar(1, s1);
  lcd.createChar(2, s2);
  lcd.createChar(3, s3);
  lcd.createChar(5, arrow);
  lcd.createChar(6, dd);
  //  lcd.createChar(12, selector2);
  printDotsAndSpaces();
  drawClockValues();
  drawDateValues();
}
boolean backLT_state = 1;
boolean editMode = 0;
boolean editH = 0;
boolean editI = 0;
boolean editS = 0;
boolean editD = 0;
boolean editM = 0;
boolean editY = 0;
int8_t h_Value = hour();
int8_t i_Value = minute();
int8_t s_Value = second();
int8_t d_Value = day();
int8_t m_Value = month();
int16_t y_Value = year() - 2000;
const char* monthArray[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
void loop() {
  /////////////////////////

  /////////////////////////
  enc.tick();                       // опрос происходит здесь

  if (millis() - currentTime > 1000) {
    currentTime = millis();
    digitalWrite(13, HIGH);
    if (editMode == 0) {
      drawClockValues();
      drawDateValues();
    }

    ////////////////
    if (Serial.available() > 1) { // wait for at least two characters
      char c = Serial.read();
      Serial.println(c);
      if ( c == TIME_HEADER) {
        processSyncMessage();
      }
    }
    ///////////////

    if (millis() - currentTime > 10) digitalWrite(13, LOW);
    //    delay(10);
    //    digitalWrite(13, LOW);
  }
  if (millis() - syncTime > 60000) {
    syncTime = millis();
    Serial.println("unix");
    rtc.gettime();
    //    const unsigned long unix = rtc.gettimeUnix();
    unixTime = rtc.gettimeUnix();
    //    Serial.println(rtc.Unix);
    Serial.println(unixTime);
    //    setTime(rtc.Unix);
    setTime(unixTime);
  }
  //  if (millis() - backlightTimeout > 30000) {
  //    lcd.noBacklight();
  //    lcd.noDisplay();
  //  }
  //////////////////////////////////////////
  //////////////////////////////////////////
  //////////////////////////////////////////
  if (enc.held()) {
    if (backLT_state == 1) {
      if (editMode == 0) {
        h_Value = hour();
        i_Value = minute();
        s_Value = second();
        d_Value = day();
        m_Value = month();
        y_Value = year() - 2000;
        Serial.println("Edit Mode");
        Serial.println("Edit Hours");
        editMode = 1;
        editH = 1;
        lcd.setCursor(3, 0);
        lcd.write(5);
      } else if (editMode == 1) {
        //        if (editH == 1) rtc.settime(-1, -1, h_Value);
        //        else if (editI == 1) rtc.settime(-1, i_Value);
        //        else if (editS == 1) rtc.settime( s_Value);
        //        else if (editD == 1) rtc.settime(-1, -1, -1,  d_Value);
        //        else if (editM == 1) rtc.settime(-1, -1, -1, -1, m_Value);
        //        else if (editY == 1) rtc.settime(-1, -1, -1, -1, -1, y_Value);
        //////////////////////////////////////////////////////////////
        rtc.settime(s_Value, i_Value, h_Value, d_Value, m_Value, y_Value);
        //////////////////////////////////////////////////////////////

        unixTime = rtc.gettimeUnix();
        setTime(unixTime);
        Serial.println(unixTime);
        editMode = 0;
        editH = 0;
        editI = 0;
        editS = 0;
        editD = 0;
        editM = 0;
        editY = 0;
        printDotsAndSpaces();
        drawClockValues();
        drawDateValues();
        Serial.println("Exit from Edit Mode");
      }
    }
  }
  //////////////////////////////////////////
  //////////////////////////////////////////
  //////////////////////////////////////////
  if (enc.hasClicks(1)) {
    if (editMode == 0) {
      if (backLT_state == 1) {
        backLT_state = 0;
        Serial.print("backLT_state - ");
        Serial.println(backLT_state);
        lcd.noBacklight();
        lcd.noDisplay();
      } else {
        backLT_state = 1;
        Serial.print("backLT_state - ");
        Serial.println(backLT_state);
        lcd.backlight();
        lcd.display();
      }
    } else if (editMode == 1) {
      if (editH == 1) {
        printDotsAndSpaces();
        lcd.setCursor(6, 0);
        lcd.write(5);
        editH = 0;
        editI = 1;
        //        rtc.settime(-1, -1, h_Value);
        //        unixTime = rtc.gettimeUnix();
        //        Serial.println(unixTime);
        //        setTime(unixTime);
        Serial.println("editMode - switching to minutes");
      } else if (editI == 1) {
        printDotsAndSpaces();
        lcd.setCursor(9, 0);
        lcd.write(5);
        editI = 0;
        editS = 1;
        //        rtc.settime(-1, i_Value);
        //        unixTime = rtc.gettimeUnix();
        //        Serial.println(unixTime);
        //        setTime(unixTime);
        Serial.println("editMode - switching to seconds");
      } else if (editS == 1) {
        printDotsAndSpaces();
        lcd.setCursor(4, 1);
        lcd.write(5);
        editS = 0;
        editD = 1;
        //        rtc.settime( s_Value);
        //        unixTime = rtc.gettimeUnix();
        //        Serial.println(unixTime);
        //        setTime(unixTime);
        Serial.println("editMode - switching to days");
      } else if (editD == 1) {
        printDotsAndSpaces();
        lcd.setCursor(7, 1);
        lcd.write(5);
        editD = 0;
        editM = 1;
        //        rtc.settime(-1, -1, -1, d_Value);
        //        unixTime = rtc.gettimeUnix();
        //        Serial.println(unixTime);
        //        setTime(unixTime);
        Serial.println("editMode - switching to month");
      } else if (editM == 1) {
        printDotsAndSpaces();
        lcd.setCursor(11, 1);
        lcd.write(5);
        editM = 0;
        editY = 1;
        //        rtc.settime(-1, -1, -1, -1, m_Value);
        //        unixTime = rtc.gettimeUnix();
        //        Serial.println(unixTime);
        //        setTime(unixTime);
        Serial.println("editMode - switching to year");
      } else if (editY == 1) {
        printDotsAndSpaces();

        editY = 0;
        editMode = 0;
        //        rtc.settime( -1, -1, -1, -1, -1, y_Value);
        rtc.settime(s_Value, i_Value, h_Value, d_Value, m_Value, y_Value);
        unixTime = rtc.gettimeUnix();
        Serial.println(unixTime);
        setTime(unixTime);
        drawClockValues();
        drawDateValues();
        Serial.println("editMode - exit from editMode");
      }
    }
  }
  //////////////////////////////////////////
  //////////////////////////////////////////
  //////////////////////////////////////////
  if (enc.turn() ) {
    if (editMode == 1) {
      if (editH == 1) {
        h_Value += enc.dir();
        if (h_Value < 0)  h_Value = 23;
        h_Value %= 24;
        Serial.print("h_Value - ");
        Serial.println(h_Value);
        lcd.setCursor(4, 0);
        if (h_Value < 10) lcd.print("0");
        lcd.print(h_Value);
      }
      if (editI == 1) {
        i_Value += enc.dir();
        if (i_Value < 0)  i_Value = 59;
        i_Value %= 60;
        Serial.print("i_Value - ");
        Serial.println(i_Value);
        lcd.setCursor(7, 0);
        if (i_Value < 10) lcd.print("0");
        lcd.print(i_Value);
      }
      if (editS == 1) {
        s_Value += enc.dir();
        if (s_Value < 0)  s_Value = 59;
        s_Value %= 60;
        Serial.print("s_Value - ");
        Serial.println(s_Value);
        lcd.setCursor(10, 0);
        if (s_Value < 10) lcd.print("0");
        lcd.print(s_Value);
      }
      if (editD == 1) {
        d_Value += enc.dir();
        if (d_Value < 1)  d_Value = 31;
        if (d_Value > 31)  d_Value = 1;
        Serial.print("d_Value - ");
        Serial.println(d_Value);
        lcd.setCursor(5, 1);
        if (d_Value < 10) lcd.print("0");
        lcd.print(d_Value);
      }
      if (editM == 1) {
        m_Value += enc.dir();
        if (m_Value < 1)  m_Value = 12;
        if (m_Value > 12)  m_Value = 1;
        Serial.print("m_Value - ");
        Serial.println(monthArray[m_Value - 1]);
        lcd.setCursor(8, 1);
        lcd.print(monthArray[m_Value - 1]);
      }
      if (editY == 1) {
        y_Value += enc.dir();
        if (y_Value < 0)  y_Value = 99;
        if (y_Value > 99)  y_Value = 0;
        Serial.print("y_Value - ");
        Serial.println(y_Value);
        lcd.setCursor(12, 1);
        lcd.print("20");
        if (y_Value < 10) lcd.print("0");
        lcd.print(y_Value);
      }
    }
  }
  //////////////////////////////////////////
  //////////////////////////////////////////
  //////////////////////////////////////////
}

void printDotsAndSpaces() {
  lcd.setCursor(0, 0);
  lcd.write(3);
  lcd.write(2);
  lcd.write(1);
  lcd.write(0);
  lcd.setCursor(12, 0);
  lcd.write(0);
  lcd.write(1);
  lcd.write(2);
  lcd.write(3);
  //  lcd.print("    ");
  lcd.setCursor(6, 0);
  lcd.write(2);
  //  lcd.print(":");
  lcd.setCursor(9, 0);
  lcd.write(2);
  //  lcd.print(":");
  //  lcd.setCursor(0, 1);
  //  lcd.write(0);
  //  lcd.print("_");
  lcd.setCursor(3, 1);
  lcd.write(1);
  lcd.write(0);
  //  lcd.print("_");
  lcd.setCursor(7, 1);
  lcd.write(0);
  //  lcd.print("_");
  lcd.setCursor(11, 1);
  lcd.write(0);
  //  lcd.print("_");
}
void drawClockValues() {
  //********************
  //**    12:34:56    **
  //**                **
  //********************
  //
  //  Serial.println(rtc.Unix);
  //  lcd.setCursor(0, 0);
  //  lcd.write(second());
  // digital clock display of the time
  ////////////////////////////////////
  lcd.setCursor(4, 0);
  if (hour() < 10) lcd.print("0");
  lcd.print(hour());
  lcd.setCursor(7, 0);
  if (minute() < 10) lcd.print("0");
  lcd.print(minute());
  lcd.setCursor(10, 0);
  if (second() < 10) lcd.print("0");
  lcd.print(second());
}
//////////////////////////////////////
void drawDateValues() {
  lcd.setCursor(0, 1);
  lcd.print(dayShortStr(weekday()));
  lcd.setCursor(5, 1);
  if (day() < 10) lcd.print("0");
  lcd.print(day());
  lcd.setCursor(8, 1);
  lcd.print(monthShortStr(month()));
  lcd.setCursor(12, 1);
  lcd.print(year());
  //////////////////////////////////////
  //  lcd.setCursor(4, 0);
  //  if (rtc.gettime("H") < 10) lcd.print('0');
  //  lcd.print(rtc.gettime("H"));
  //  lcd.print(':');
  //  if (rtc.minutes < 10) lcd.print('0');
  //  lcd.print(rtc.minutes);
  //  lcd.print(':');
  //  if (rtc.seconds < 10) lcd.print('0');
  //  lcd.print(rtc.seconds);
  ///////////////////////////////////////
  //  lcd.setCursor(1, 1);
  //  //  lcd.print(rtc.weekday);
  //  lcd.print(rtc.gettime("D"));
  //  lcd.print(' ');
  //  lcd.print(rtc.day);
  //  lcd.print(' ');
  //  lcd.print(rtc.gettime("M"));
  //  lcd.print(' ');
  //  lcd.print(rtc.gettime("Y"));
  ///////////////////////////////////////
}

void processSyncMessage() {
  unsigned long value;
  char c = Serial.read();
  Serial.println(c);
  switch (c) {
    case HOUR:
      value = Serial.parseInt();
      //      if (value < 24)
      rtc.settime(-1, -1, value);
      Serial.println("HOUR");
      Serial.println(value);
      break;
    case MIN:
      value = Serial.parseInt();
      //      if (value < 60)
      rtc.settime(-1, value);
      Serial.println("MIN");
      Serial.println(value);
      break;
    case SEC:
      value = Serial.parseInt();
      //      if (value < 60)
      rtc.settime( value);
      Serial.println("SEC");
      Serial.println(value);
      break;
    case DAY:
      value = Serial.parseInt();
      //      if (value < 32)
      rtc.settime(-1, -1, -1, value);
      Serial.println("DAY");
      Serial.println(value);
      break;
    case MONTH:
      value = Serial.parseInt();
      //      if (value < 13)
      rtc.settime(-1, -1, -1, -1, value);
      Serial.println("MONTH");
      Serial.println(value);
      break;
    case YEAR:
      value = Serial.parseInt();
      rtc.settime(-1, -1, -1, -1, -1, value);
      Serial.println("YEAR");
      Serial.println(value);
      break;
    default:
      // выполнить, если val ни 1 ни 2
      // default опционален
      break;
  }
  //  unsigned long pctime;
  //  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013 - paul, perhaps we define in time.h?
  //
  //  pctime = Serial.parseInt();
  //  Serial.println(pctime);
  //  if ( pctime >= DEFAULT_TIME) { // check the integer is a valid time (greater than Jan 1 2013)
  //    setTime(pctime); // Sync Arduino clock to the time received on the serial port
  //  }
  //  const unsigned long unix = rtc.gettimeUnix();
  unixTime = rtc.gettimeUnix();
  Serial.println(unixTime);
  setTime(unixTime);
}

time_t requestSync() {
  return 0; // the time will be sent later in response to serial mesg
}
