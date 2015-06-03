// ============= Arduino 1.0.5 ====================
// ============= TE-Mini 328P =====================
// 08.05.2014 by Roma Kuzmin
// 17.05 Romik

#include <TinyGPS.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <avr/pgmspace.h>
#include <DallasTemperature.h>

#define RED      2
#define GREEN    1
#define OFF      0
#define CO2      1 // A1 MQ-135
#define VIN      3 // A3 Voltage Input

#define MAX_DISPLAY 3

boolean upload = false; // ---- Прежде чем показывать нужно загрузить все это ------

byte glyphs[5][8] = {
  {B11111,B11111,B00000,B00000,B00000,B00000,B00000,B00000},
  {B00000,B00000,B00000,B00000,B00000,B00000,B11111,B11111},
  {B11111,B11111,B00000,B00000,B00000,B00000,B11111,B11111},
  {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111}, 
  {B00000,B00000,B00000,B00000,B00000,B01110,B01110,B01110} 
};

const int digitWidth = 3; 

const char bigDigitsTop[10][digitWidth]={ 3,0,3,0,3,32,2,2,3,0,2,3,3,1,3,3,2,2,3,2,2,0,0,3,3,2,3,3,2,3 };
const char bigDigitsBot[10][digitWidth]={ 3,1,3,1,3,1,3,1,1,1,1,3,32,32,3,1,1,3,3,1,3,32,32,3,3,1,3,1,1,3 };

char buffer[12];

// ----------------------------------------------------------------------------------------------

#define TEMPERATURE_PRECISION 12

#define COMOUT 0

#define LRed     12    // PWM D12
#define LGreen   11    // PWM D11
#define buzzer   10    // PWM D10

#define MAXARRAY 10    // Максимальное значение в масиве

#define CEL      0x99  // Значок градусов

#define UP       char(0xD9) // Стрклка вверх
#define DOWN     char(0xDA) // Стрелка вниз

#define DS1307_ADDRESS 0x68

// Кнопка

#define  IN_PIN   2 // A2 Кнопка --- 10 Ком

#define   R1     105 // R1 KOM
#define   R2     10  // R2 KOM

int tArray[MAXARRAY] = { 0,0,0,0,0,0,0,0 };

DeviceAddress sensor = { 0x28, 0xd4, 0x38, 0xf7, 0x02, 0x00, 0x00, 0x04 }; // D18B20+

OneWire oneWire(1); // D1

DallasTemperature ds18(&oneWire);

int Zmax;
int Zmin;

const int eeprom = 0x50; // I2C bus address for the EEPROM chip int on

TinyGPS gps;

SoftwareSerial gps_p(2,3);

unsigned long TimePreviousInterval = 0; 
unsigned long tochkiPreviousInterval = 0;
unsigned long displayPreviousInterval = 0;
unsigned long onePreviousInterval = 0;
unsigned long gpsPreviousInterval = 0;
unsigned long dtPreviousInterval = 0;

boolean s_tochki = false;

LiquidCrystal lcd(8,9,4,5,6,7); // PWM D8 D9 and D4 D5 D6 D7

#define ds oneWire

// OneWire  ds(1); // --- DS18B20 --- PWM D1

// byte sensor[8]; // = {0x28, 0x46, 0xBD, 0x19, 0x03, 0x00, 0x00, 0x35};

byte c[8] = {           
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

unsigned long Interval = 1800;        // Для печати Даты и обновления Графика 900 Очень быстро
unsigned long P_Interval = 0;

unsigned long currentMillis;

byte display = 0; // Default Big Clock

#define LCD_LIGHT  13  // D13

boolean lcd_status =  false;

// ------------------------- Music -----------------------------------

const int speakerPin = buzzer; // connect speaker to pin 9

char noteNames[] = { 'C','D','E','F','G','a','b' };
unsigned int frequencies[] = { 262,294,330,349,392,440,494 };
const byte noteCount = sizeof(noteNames); 
char score[] = "CCGGaaGFFEEDDC GGFFEEDGGFFEED CCGGaaGFFEEDDC ";
const byte scoreLen = sizeof(score); 

// ------------------------- Setup -----------------------------------

void setup() {

  unsigned int ep = 0;
  byte data_out = 99;
  byte data_in = 0;

  unsigned int i = 0;

  Wire.begin(); // Attach I2C 

  pinMode(LRed, OUTPUT);    
  pinMode(LGreen, OUTPUT);     
  pinMode(buzzer, OUTPUT); 
  pinMode(LCD_LIGHT,OUTPUT);

  digitalWrite(LCD_LIGHT,HIGH);

  lcd.begin(16, 2);

  ep=7;

  for (i=0;i<8;i++) {
    c[ep] = B11111;
    lcd.createChar(i, c);
    ep--;
  }

  Green_Red(RED);
  
  lcd.setCursor(0,1); 
  lcd.print("Write to 24LC256");
  delay(1000);
  lcd.clear();

  Green_Red(RED);  

  // Запись работает

  lcd.setCursor(0,0); 
  lcd.print("W:");
  lcd.setCursor(0,1); 
  lcd.print("R:");

  for (ep=0;ep < 13;ep++) {
    lcd.setCursor(ep+2,0);
    lcd.print(char(data_out));

    writeData(eeprom, ep, data_out++ );  
    data_in = readData(eeprom, ep);

    lcd.setCursor(ep+2,1);
    lcd.print(char(data_in));

    delay(250);
  }

  delay(1000);

  Green_Red(GREEN);
  
  lcd.clear();
  lcd.print("TE-MINI-328P"); 

  delay(500);

  gps_p.begin(4800);  // GPS L30

  lookUpSensors();

  DallasTemperature sensors(&oneWire);

  sensors.begin();
  sensors.setResolution(sensor, TEMPERATURE_PRECISION);

  delay(1000);
  lcd.clear();

  pinMode(LCD_LIGHT,OUTPUT);
  digitalWrite(LCD_LIGHT,LOW);
  lcd.setCursor(0,0);
  lcd.print("LCD OFF (D0)");
  delay(1000);

  digitalWrite(LCD_LIGHT,HIGH);
  lcd.setCursor(0,1);
  lcd.print("LCD ON (D0)");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Buzzer D10");
  buzz(buzzer, 2500, 500);
  delay(100);
  
  lcd.setCursor(0,1);
  lcd.print("Buzzer More");
  buzz(buzzer, 5500, 400);
  delay(100);
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(GetTemp());
  lcd.print(" GPS:");
  lcd.setCursor(0,1);
  lcd.print("VCC: ");
  lcd.print(GetVin());
  lcd.print("  ");
  lcd.print(TinyGPS::library_version());
  delay(1000);
  
  Green_Red(RED);

 lcd.clear();
}

//////////////////////////////////////////////////////////////////////////////////////
// ------------------------ Основной Цикл ------------------------------------------//
//////////////////////////////////////////////////////////////////////////////////////

void loop() {

  currentMillis = millis();


  if (display == 3) { // IF DISPLAY == 3
   if(currentMillis - dtPreviousInterval > 1000) {  // Выводим большие часы
    dtPreviousInterval = currentMillis;  
    simple_date();
   }
  }

  if (display == 2) { // IF DISPLAY == 2
   if(currentMillis - gpsPreviousInterval > 1000) {  // Выводим большие часы
    gpsPreviousInterval = currentMillis;  
    print_gps_stat();
   }
  }
  
 if (display == 0) { // IF DISPLAY == 0
  if(currentMillis - displayPreviousInterval > 1000) {  // Выводим большие часы
   displayPreviousInterval = currentMillis;  
   big_clock(); 
  }

  if(currentMillis - tochkiPreviousInterval > 500) {  // Выводим точки
   tochkiPreviousInterval = currentMillis;  
   if (s_tochki) {
    tochki(1); 
   } else {
     tochki(0);
   }
   s_tochki = !s_tochki;   
  }
 
 } // IF DISPLAY = 0

  if (display == 1) { // IF DISPLAY = 1
   if(currentMillis - onePreviousInterval > 1000) {
      onePreviousInterval = currentMillis;  

    lcd.setCursor(0,0);
    lcd.print("Temp: ");
    lcd.print(GetTemp());
    lcd.print(" GPS:");
    lcd.setCursor(0,1);
    lcd.print("VCC:  ");
    lcd.print(GetVin());
    lcd.print("B ");
    lcd.print(TinyGPS::library_version());
    lcd.print("  ");
   }
  }
  
   int vIN_PIN = analogRead(IN_PIN); // Кнопку нажали

   if (vIN_PIN > 1000 ) {  // Кнопку нажали
    delay(10);
    vIN_PIN = analogRead(IN_PIN);
     if (vIN_PIN > 1000 ) {
      lcd.clear();
      vIN_PIN = 0;
      display++;
      if (display > MAX_DISPLAY ) display = 0;       
     }
   }

          
  if (gps_p.available()) {
    while (gps_p.available()) {
      char c = gps_p.read();
      if (gps.encode(c)) { 
        if(currentMillis - TimePreviousInterval > 5000) {
          TimePreviousInterval = currentMillis;
          set_date_time_from_gps();
          check_gps_stat();
        } 
      }    
    }
  }
  
}
// ---------------------------- Дополнительные Функции ----------------------------------

void buzz(int targetPin, long frequency, long length) {

  long delayValue = 1000000/frequency/2; 
  long numCycles = frequency * length/ 1000;

  for (long i=0; i < numCycles; i++) { 
    digitalWrite(targetPin,HIGH); 
    delayMicroseconds(delayValue); 
    digitalWrite(targetPin,LOW); 
    delayMicroseconds(delayValue);
  }

}

byte decToBcd(byte val) {
  return ( (val/10*16) + (val%10) );
}

byte bcdToDec(byte val) {
  return ( (val/16*10) + (val%16) );
}

#ifdef SETTIME

void setDateTime() {

  byte second =      00; //0-59
  byte minute =      45; //0-59
  byte hour =        16; //0-23
  byte weekDay =     2;  //1-7
  byte monthDay =    13; //1-31
  byte month =       3;  //1-12
  byte year  =       12; //0-99

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0); //stop Oscillator

  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour));
  Wire.write(decToBcd(weekDay));
  Wire.write(decToBcd(monthDay));
  Wire.write(decToBcd(month));
  Wire.write(decToBcd(year));

  Wire.write(0); //start
  Wire.endTransmission();

}

#endif

// -------------------------- printTime ---------------------------


void printTime(byte display) {

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_ADDRESS, 7);

  int second = bcdToDec(Wire.read());
  int minute = bcdToDec(Wire.read());
  int hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
  int weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  int monthDay = bcdToDec(Wire.read());
  int month = bcdToDec(Wire.read());
  int year = bcdToDec(Wire.read());

  if (display == 3) {

    lcd.clear();

    while(1) { 

      if (!upload) {  
        for(int i=0; i < 5; i++) lcd.createChar(i, glyphs[i]); 
        upload = true; 
      }

      lcd.setCursor(7,0); 
      lcd.print("*");
      lcd.setCursor(7,1); 
      lcd.print("*");

      if (hour < 10) {
        showNumber(0,0);
        showNumber(hour, 1);
      } 
      else showNumber(hour,0);
      if (minute < 10) {
        showNumber(0,2);
        showNumber(minute, 3);
      } 
      else showNumber(minute,2);

      delay(500);
      lcd.setCursor(7,0); 
      lcd.print(" ");
      lcd.setCursor(7,1); 
      lcd.print(" ");
      delay(500);

      Wire.beginTransmission(DS1307_ADDRESS);
      Wire.write(0);
      Wire.endTransmission();
      Wire.requestFrom(DS1307_ADDRESS, 7);

      second = bcdToDec(Wire.read());
      minute = bcdToDec(Wire.read());
      hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
      weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
      monthDay = bcdToDec(Wire.read());
      month = bcdToDec(Wire.read());
      year = bcdToDec(Wire.read());

      int vIN_PIN = analogRead(IN_PIN);

      if (vIN_PIN > 1000 ) {
        delay(10);
        vIN_PIN = analogRead(IN_PIN);
        if (vIN_PIN > 1000 ) {
          lcd.clear();
          display > 0;
          delay(10);
          upload = false;
          byte ep=7;
          for (byte i=0;i<8;i++) {
            c[ep] = B11111;
            lcd.createChar(i, c);
            ep--;
          }
          break;
        }
      }      

    }

  }

  if (lcd_status == false) { 
    if (hour > 9 && hour < 19) digitalWrite(LCD_LIGHT,LOW);
    else digitalWrite(LCD_LIGHT,HIGH);
    // bluetooth.println(lcd_status);
    lcd_status = true;
  }

  if (minute == 30 && hour == 8 && second == 59) {

    if (hour > 9 && hour < 19) 
      digitalWrite(LCD_LIGHT,LOW);
    else digitalWrite(LCD_LIGHT,HIGH);

    for (int i = 0; i < scoreLen; i++)
    {
      int duration = 333; // each note lasts for a third of a second
      playNote(score[i], duration); // play the note
    }
  }

  if (display == 0) {

    lcd.print("Time: ");
    if (hour < 10)   lcd.print("0"); 
    lcd.print(hour,DEC);
    lcd.print(":");
    if (minute < 10) lcd.print("0"); 
    lcd.print(minute,DEC);
    lcd.print(":");
    if (second < 10) lcd.print("0"); 
    lcd.print(second,DEC);

  }  
  else if (display == 4) {
    lcd.print("Sa:");
    lcd.print(gps.satellites());
    lcd.print(" Sp:");
    lcd.setCursor(8,1); 
    lcd.print("       ");
    lcd.setCursor(8,1); 
    lcd.print(gps.f_speed_kmph());

  }  
  else if (display == 1) {

    lcd.print("Date: ");
    if (monthDay < 10) lcd.print("0"); 
    lcd.print(monthDay); 
    lcd.print("-");
    if (month < 10)    lcd.print("0"); 
    lcd.print(month); 
    lcd.print("-");
    lcd.print(year,DEC);

  } 
  else if (display == 2) {

    if (hour < 10)   lcd.print("0"); 
    lcd.print(hour,DEC);
    lcd.print(":");
    if (minute < 10) lcd.print("0"); 
    lcd.print(minute,DEC);
    lcd.print(":");
    if (second < 10) lcd.print("0"); 
    lcd.print(second,DEC);
    lcd.print(" ");

    int value = 0;
    float vout;
    float vin;

    for(int n=0;n<10;n++) {
      value = value + analogRead(VIN);
    }    

    value = value / 10;

    vout= (value * 5.0)/1024.0;         // vout= (value * 5.0)/1024.0;
    vin = vout / (10.0/(10.0+110.0));   // vin = vout / (R2/(R1+R2));

    lcd.print(vin,2);

  }

}

// ----------------------- Получить входящие напряжение питания -----------------------

float GetVin( void ) {
  
 float vin;
 int value = 0;
 float vout;
 
 for(int n=0;n<10;n++) {
  value = value + analogRead(VIN);
 }    

 value = value / 10;

 vout= (value * 5.0)/1024.0;         // vout= (value * 5.0)/1024.0;
 vin = vout / (10.0/(10.0+110.0));   // vin = vout / (R2/(R1+R2));

 return(vin);

}
// ---------------------- Поиск Темперетаурного датчика DS18x20 ----------------------

void lookUpSensors() {
  byte address[8];
  int i=0;
  byte ok = 0, tmp = 0;

  while (ds.search(address)){
    tmp = 0;
    //0x10 = DS18S20
    if (address[0] == 0x10){
      lcd.setCursor(0,1); 
      lcd.print("Device: DS18S20");
      tmp = 1;
    } 
    else {
      //0x28 = DS18B20
      if (address[0] == 0x28){
        lcd.setCursor(0,1); 
        lcd.print("Device: DS18B20");
        tmp = 1;
      }
    }
    //display the address, if tmp is ok
    if (tmp == 1){
      if (OneWire::crc8(address, 7) != address[7]){
        lcd.setCursor(0,1); 
        lcd.println("Not valid CRC!");
      } 
      else {
        lcd.setCursor(0,0);
        //all is ok, display it
        for (i=0;i<8;i++){
          if (address[i] < 9){
            lcd.print("0");
          }
          lcd.print(address[i],HEX);
          sensor[i] = address[i]; // Заполняем Адрес для DS18x20
        }
        ok = 1;
      }
    } //end if tmp
  } //end while
  if (ok == 0){
    lcd.setCursor(0,1); 
    lcd.println("No devices found");
    delay(2000);
  }
}

void writeTimeToScratchpad(byte* address){
  ds.reset();
  ds.select(address);
  ds.write(0x44,1);
  delay(10);
}

void readTimeFromScratchpad(byte* address, byte* data){
  ds.reset();
  ds.select(address);
  ds.write(0xBE);
  for (byte i=0;i<9;i++){
    data[i] = ds.read();
  }
}

// ----------------------------------- getTemperature (no delay) -------------------

float getTemperature(byte* address){
  int tr;
  byte data[12];

  writeTimeToScratchpad(address);
  readTimeFromScratchpad(address,data);

  tr = data[0];

  if (data[1] > 0x80){
    tr = !tr + 1; 
    tr = tr * -1; 
  }

  int cpc = data[7];
  int cr = data[6];

  tr = tr >> 1;

  return tr - (float)0.25 + (cpc - cr)/(float)cpc;
}

float f2c(float val){
  float aux = val - 32;
  return (aux * 5 / 9);
}

// ------------------------------ Выводим Температуру на LCD ------------------------ 

void printTemp(void) {
  int tempC;

  tempC = GetTemp();

  // tempC = getTemperature(sensor);
  // tempC = f2c(tempC);

  if (tempC == 0) {  
   buzz(buzzer, 3000, 600); 
  }
  
  lcd.setCursor(0,0);
  lcd.print(tempC);
  lcd.print(byte(CEL));
  lcd.print("C ");

  if (COMOUT) {
    // bluetooth.print("|");
    // bluetooth.print(tempC);
    // bluetooth.print("|");
  }
}

// ------------------------- Чтение из EEPROM ---------------------------------

byte readData(int device, unsigned int eeaddress) {
  delay(7);
  byte rdata = 0xFF;
  Wire.beginTransmission(device);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(device,1);
  if (Wire.available()) rdata = Wire.read(); 
  return rdata;
}

// --------------------- Звпись в EEPROM --------------------------------

void writeData(int device, unsigned int eeaddress, byte data) {
  delay(7);
  Wire.beginTransmission(device);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
}

// ----------------------- Get Temperature ------------------------------------

float GetTemp() {

  ds18.requestTemperatures();
  return(ds18.getTempC(sensor));

}

void playNote(char note, int duration) {

  // play the tone corresponding to the note name

  for (int i = 0; i < noteCount; i++)
  {
    // try and find a match for the noteName to get the index to the note
    if (noteNames[i] == note) // find a matching note name in the array
      tone(speakerPin, frequencies[i], duration); // play the note using the frequency
  }
  // if there is no match then the note is a rest, so just do the delay
  delay(duration);
}



void showDigit(int digit, int position) {

  lcd.setCursor(position * (digitWidth + 1), 0);

  for(int i=0; i < digitWidth; i++) lcd.print(bigDigitsTop[digit][i]);

  lcd.setCursor(position * (digitWidth + 1), 1);

  for(int i=0; i < digitWidth; i++) lcd.print(bigDigitsBot[digit][i]);

}

void showNumber(int value, int position) {

  int index; 
  itoa(value, buffer, 10);

  for(index = 0; index < 10; index++) {
    char c = buffer[index]; 
    if( c == 0) return; 
    c = c - 48; 
    showDigit(c, position + index);
  }

}

boolean check_gps_stat( void ) {

  int year;
  byte month, day, hour, minutes, second, hundredths;
  unsigned long age;

  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &age);

  if (age != TinyGPS::GPS_INVALID_AGE) {
    Green_Red(GREEN); 
    return(true); 
  } else {
    Green_Red(RED); 
    return(false);
  }
}

void set_date_time_from_gps( void ) {

  int year;
  byte month, day, hour, minutes, second, hundredths;
  unsigned long age;

  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &age);

  if (age != TinyGPS::GPS_INVALID_AGE) {

    byte weekDay = 1;  // 1-7 
    year = year - 2000; // 2014-2000;

    hour = hour + 4;
    if ( hour > 23 ) hour = hour - 24; 

    Wire.beginTransmission(DS1307_ADDRESS);
    Wire.write(0); //stop Oscillator

    Wire.write(decToBcd(second));
    Wire.write(decToBcd(minutes));
    Wire.write(decToBcd(hour));
    Wire.write(decToBcd(weekDay));
    Wire.write(decToBcd(day));
    Wire.write(decToBcd(month));
    Wire.write(decToBcd(byte(year)));

    Wire.write(0); //start
    Wire.endTransmission();

  }

}

void Green_Red(byte stat) {

  if (stat == 0) {          // Off
    digitalWrite(LRed,LOW);
    digitalWrite(LGreen,LOW);
  } 
  else if (stat == 1 ) {  // Green
    digitalWrite(LRed,LOW);
    digitalWrite(LGreen,HIGH);
  } 
  else if (stat == 2 ) {  // Red
    digitalWrite(LGreen,LOW);
    digitalWrite(LRed,HIGH);
  }

}

// ----------------- Большие Часы ----------------------------------

void big_clock() {
  
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_ADDRESS, 7);

  int second = bcdToDec(Wire.read());
  int minute = bcdToDec(Wire.read());
  int hour = bcdToDec(Wire.read() & 0b111111); // 24 hour time
  int weekDay = bcdToDec(Wire.read());         // 0-6 -> sunday - Saturday
  int monthDay = bcdToDec(Wire.read());
  int month = bcdToDec(Wire.read());
  int year = bcdToDec(Wire.read());

  // lcd.clear();
  
  if (s_tochki) tochki(1);
  
  if (!upload) {  
   for(int i=0; i < 5; i++) lcd.createChar(i, glyphs[i]); 
   upload = true; 
  }

  if (hour < 10) {
   showNumber(0,0);
   showNumber(hour, 1);
  } else showNumber(hour,0);
  
  if (minute < 10) {
   showNumber(0,2);
   showNumber(minute, 3);
  } else showNumber(minute,2);

}

void tochki( byte stat ) {
  
  if (stat == 1) {
   lcd.setCursor(7,0); 
   lcd.print("*");
   lcd.setCursor(7,1); 
   lcd.print("*");
  } else {
   lcd.setCursor(7,0); 
   lcd.print(" ");
   lcd.setCursor(7,1); 
   lcd.print(" ");
  }
  
}

void print_gps_stat() {
  
  unsigned long chars;
  unsigned short sentences, failed_checksum;
  float flat, flon;
  unsigned long fix_age;

  int year;
  byte month, day, hour, minutes, second, hundredths;
  unsigned long age;

  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &age);

  gps.stats(&chars, &sentences, &failed_checksum);
  
  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  
  gps.f_get_position(&flat, &flon, &fix_age);
  
  lcd.setCursor(0,1);

  if (fix_age == TinyGPS::GPS_INVALID_AGE) {
   lcd.print("No fix detected ");
   Green_Red(RED); 
   lcd.setCursor(0,0);
   lcd.print(chars);
   lcd.print(" ");
   lcd.print(sentences);
   lcd.print(" ");
   lcd.print(failed_checksum);
 }
  else if (fix_age > 5000) {
   lcd.print("Error stale data");
   lcd.setCursor(0,0);
   lcd.print(chars);
   lcd.print(" ");
   lcd.print(sentences);
   lcd.print(" ");
   lcd.print(failed_checksum);
   Green_Red(RED); 
 }
  else {
    Green_Red(GREEN); 
   lcd.setCursor(0,0);
   lcd.print("Lat: ");
   lcd.print(flat);
   lcd.setCursor(0,1);
   lcd.print("Lon: ");   
   lcd.print(flon);   
 //  lcd.print(" ");
 //  lcd.print(year);
  }
  
}

void simple_date() {

  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(DS1307_ADDRESS, 7);

  int second = bcdToDec(Wire.read());
  int minute = bcdToDec(Wire.read());
  int hour = bcdToDec(Wire.read() & 0b111111); //24 hour time
  int weekDay = bcdToDec(Wire.read()); //0-6 -> sunday - Saturday
  int monthDay = bcdToDec(Wire.read());
  int month = bcdToDec(Wire.read());
  int year = bcdToDec(Wire.read());

//  lcd.clear();

  lcd.setCursor(0,0);
  
    lcd.print("Time: ");
    if (hour < 10)   lcd.print("0"); 
    lcd.print(hour,DEC);
    lcd.print(":");
    if (minute < 10) lcd.print("0"); 
    lcd.print(minute,DEC);
    lcd.print(":");
    if (second < 10) lcd.print("0"); 
    lcd.print(second,DEC);
    lcd.print("  ");

   lcd.setCursor(0,1);

    lcd.print("Date: ");
    if (monthDay < 10) lcd.print("0"); 
    lcd.print(monthDay); 
    lcd.print("-");
    if (month < 10)    lcd.print("0"); 
    lcd.print(month); 
    lcd.print("-");
    lcd.print(year,DEC);
    lcd.print("  ");
  
}

