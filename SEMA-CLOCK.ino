#include <Wire.h>
#include "LiquidCrystal_I2C.h"
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
}

void loop()
{
  enc.tick();                       // опрос происходит здесь

  if (millis() - currentTime > 1000) {
    currentTime = millis();
    digitalWrite(13, HIGH);
    digitalClockDisplay();
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
  if (millis() - backlightTimeout > 30000) {
    lcd.noBacklight();
    lcd.noDisplay();
  }
  if (enc.turn() || enc.press()) {
    lcd.backlight();
    lcd.display();
    backlightTimeout = millis();


  }
  if (enc.held()) {
    Serial.println("Edit Mode");
    lcd.setCursor(4, 0);
    lcd.cursor_on();
    lcd.blink_on();
  }
}

void digitalClockDisplay() {

  //  Serial.println(rtc.Unix);
  //  lcd.setCursor(0, 0);
  //  lcd.write(second());
  // digital clock display of the time
  ////////////////////////////////////
  lcd.setCursor(4, 0);
  if (hour() < 10) lcd.print('0');
  lcd.print(hour());
  lcd.print(':');
  if (minute() < 10) lcd.print('0');
  lcd.print(minute());
  lcd.print(':');
  if (second() < 10) lcd.print('0');
  lcd.print(second());
  //////////////////////////////////////
  lcd.setCursor(1, 1);
  lcd.print(dayShortStr(weekday()));
  lcd.print(' ');
  lcd.print(day());
  lcd.print(' ');
  lcd.print(monthShortStr(month()));
  lcd.print(' ');
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
      if (value < 24) rtc.settime(-1, -1, value);
      Serial.println("HOUR");
      Serial.println(value);
      break;
    case MIN:
      value = Serial.parseInt();
      if (value < 60) rtc.settime(-1, value);
      Serial.println("MIN");
      Serial.println(value);
      break;
    case SEC:
      value = Serial.parseInt();
      if (value < 60) rtc.settime( value);
      Serial.println("SEC");
      Serial.println(value);
      break;
    case DAY:
      value = Serial.parseInt();
      if (value < 32) rtc.settime(-1, -1, -1, value);
      Serial.println("DAY");
      Serial.println(value);
      break;
    case MONTH:
      value = Serial.parseInt();
      if (value < 13) rtc.settime(-1, -1, -1, -1, value);
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
