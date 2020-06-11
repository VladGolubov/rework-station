/*   --------------------------------------------------
   | Имя    : Паяльная станция                          |
   | Автор  : Влад Голубов                              |
   | Дата   :  05.01.2020                               |
   | Версия :  2.0.2                                    |
     --------------------------------------------------
   Добавлено:
   1.PID регулятор
   2.Нижний подогрев
   Функции:
   1. Проверка температуры термопарой(терморезистором) на паяльнике(s_thermPin),
      если температура меньше установленной резистором (s_res),
      то включается высокий сигнал на выводе(s_ten)
   2. Проверка температуры термопарой(терморезистором) на фене(f_thermPin),
      если температура меньше установленной резистором (f_res),
      то включается высокий сигнал на выводе(f_ten)
   3. Если темература фена(f_thermPin) > минимальной температуры(f_fanAlarm),
      то на вывод (f_fan) включается высокий сигнал
   Материалы:

*/

#include      <max6675.h>                              // Подключаем библиотеку max6675 для работы с датчиком температуры 
const int miso         = 12;
const int sck          = 13;
const byte max6675_num = 3;
const int max6675_cs_pins[max6675_num] = {1, 4, 5};    // Выводы на которые подключены max6675


MAX6675   max6675s[max6675_num] = {
  MAX6675(sck, max6675_cs_pins[0], miso),
  MAX6675(sck, max6675_cs_pins[1], miso),
  MAX6675(sck, max6675_cs_pins[2], miso),
};

//----------------------------------------------------------------------------------------------


#include <PIDController.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//----------------------------------------------------------------------------------------------


LiquidCrystal_I2C lcd(0x27, 16, 2);
/*Константы*/
#define max_temp  450
#define f_fanAlarm 50
#define alarm_temp 450
#define error_temp 0
/*
#define k_s 2  // Коэффициенты для калибровки температуры паяльника
#define k_f 2  // Паяльного фена
#define k_dh 2 // Нижнего подогрева
*/
/* если используем терморезистор
  #define B 3950 // B-коэффициент
  #define SERIAL_R 220 // сопротивление последовательного резистора, 220 Ом
  #define THERMISTOR_R 60 // номинальное сопротивления термистора, 60 Ом
  #define NOMINAL_T 25 // номинальная температура (при которой TR = 60 Ом)
*/


/*Аналоговые входы*/
//#define dh_thermPin A4   PC4
//#define f_thermPin A2    PC2
//#define s_thermPin A3    PC3
#define s_res A1         //PC1
#define f_res A0         //PC0
#define dh_res A2        //PC5 

/*Цифровые входы*/
int16_t  s_ten = 9;   //PB1
int16_t  f_ten = 10;  //PB2
int16_t  dh_ten = 11; //PB4
int16_t  f_fan = 6;   //PB3
int16_t  SW1 = 8;     //PD6  паяльник
int16_t  SW2 = 7;     //PD7  фен
int16_t  SW3 = 2;     //PB0  геркон
int16_t  SW4 = 3;     //PB5  нижний подогрев

/*Переменнные*/
int16_t dh_thermo;
int16_t f_thermo;
int16_t s_thermo;
int16_t s_resd;
int16_t f_resd;
int16_t dh_resd;
//float steinhart; // если используем терморезистор, то разкомментировать
//float tr;        // и перепаять на плате

uint32_t timing_lcd, timing_s, timing_f, timing_dh;


PIDController pid_solder;
PIDController pid_fen;
PIDController pid_dh;


byte degree[8] = // кодируем 1 символ градуса
{
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000,
};
byte degree2[8] = // кодируем 2 символ градуса
{
  B00111,
  B00101,
  B00111,
  B01100,
  B10000,
  B10000,
  B01100,
};

/*Рабочий вариант для терморезистора
   float tr = 1023.0 / s_thermo - 1;
    tr = SERIAL_R / tr;
    float steinhart;
    steinhart = tr / THERMISTOR_R; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_T + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15;
  steinhart = s_thermo;*/


void setup() {

  Serial.end();
  lcd.init();
  lcd.backlight();
  delay(500);
  lcd.setCursor(0, 0);
  lcd.print("Rework");
  lcd.setCursor(0, 1);
  lcd.print("Station");
  delay(500);

  lcd.createChar(1, degree);   // Создаем символ под номером 1
  lcd.createChar(2, degree2);  // Создаем символ под номером 1

  pid_solder.begin();
  pid_fen.begin();
  pid_dh.begin();
  pid_dh.tune(75, 0.07, 0.2);
  pid_dh.limit(0, 255);
  pid_fen.tune(75, 0.07, 0.2);
  pid_fen.limit(0, 255);
  pid_solder.tune(90, 0.07, 0.2); // Tune the PID, arguments: kP, kI, kD
  pid_solder.limit(0, 255);       // Limit the PID output between 0 and 255

  pinMode(f_fan, OUTPUT);
  pinMode(s_ten, OUTPUT);
  pinMode(f_ten, OUTPUT);
  pinMode(dh_ten, OUTPUT);
  //pinMode(s_thermPin, INPUT);
  //pinMode(f_thermPin, INPUT);
  //pinMode(dh_thermPin, INPUT);
  pinMode(s_res, INPUT);
  pinMode(f_res, INPUT);
  pinMode(dh_res, INPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(SW4, INPUT);
  lcd.clear();
};


void loop() {


  //void control_solder(){
  if (digitalRead(SW1) == HIGH ) {
    s_resd = analogRead(s_res);
    s_resd = map(s_resd, 0, 1024, 0, 500);
    s_resd = constrain(s_resd, 0, 500);
    if (millis() - timing_s > 310) {
      timing_s = millis();
      s_thermo = max6675s[0].readCelsius();
   //   s_thermo = s_thermo / k_s;
    };
    /*    if (s_thermo < s_resd) {
          digitalWrite(s_ten, HIGH);
        } else {
          digitalWrite(s_ten, LOW);
        };*/
    pid_solder.setpoint(s_resd);
    int16_t output1 = pid_solder.compute(s_thermo);
    analogWrite(s_ten, output1);

    if (s_thermo == alarm_temp ) {
      digitalWrite(s_ten, LOW);
    };
  } else {
    digitalWrite(s_ten, LOW);
  };

  //void control_fen(){
  if (digitalRead(SW2) == HIGH && digitalRead(SW3) == LOW ) {
    f_resd = analogRead(f_res);
    f_resd = map(f_resd, 0, 1024, 0, 500);
    f_resd = constrain(f_resd, 0, 500);
    if (millis() - timing_f > 320) {
      timing_f = millis();
      f_thermo = max6675s[1].readCelsius();
 //     f_thermo = f_thermo / k_f;
   };
    /* if(f_thermo < f_resd){
       digitalWrite(f_ten, HIGH);
      }else{
        digitalWrite(f_ten, LOW);
      }
    */
    pid_fen.setpoint(f_resd);
    int16_t output2 = pid_fen.compute(f_thermo);
    analogWrite(f_ten, output2);
    if (f_thermo >  f_fanAlarm  || digitalRead(SW3) == HIGH ) {
      digitalWrite(f_fan, HIGH);
    } else {
      digitalWrite(f_fan, LOW);
    }
    if (f_thermo == alarm_temp ) {
      digitalWrite(f_ten, LOW);
    };
  } else if (digitalRead(SW2) == LOW) {
    digitalWrite(f_ten, LOW);
  };

  //void control_dh(){
  if (digitalRead(SW4) == HIGH ) {
    dh_resd = analogRead(dh_res);
    dh_resd = map(dh_resd, 0, 1024, 0, 500);
    dh_resd = constrain(dh_resd, 0, 500);
    if (millis() - timing_dh > 330) {
      timing_dh = millis();
      dh_thermo = max6675s[2].readCelsius();
    //  dh_thermo = dh_thermo / k_dh;
    };
    /* if(dh_thermo < dh_resd){
       digitalWrite(dh_ten, HIGH);
       delay(50);
      }else{
        digitalWrite(dh_ten, LOW);
      } рабочий кусок кода
      }*/
    pid_dh.setpoint(dh_resd);
    int16_t output3 = pid_dh.compute(dh_thermo);
    analogWrite(dh_ten, output3);
      if (dh_thermo == alarm_temp ) {
        digitalWrite(dh_ten, LOW);
      };
    
  } else {
    digitalWrite(dh_ten, LOW);
  };

  //void lcd_r(){
  if (millis() - timing_lcd > 300) {
    timing_lcd = millis();
    lcd.clear();
    //Solder
    lcd.setCursor(0, 0);
    lcd.print("S:");
    lcd.setCursor(0, 1);
    lcd.print("H:");
    if (digitalRead(SW1) == HIGH) {
      lcd.setCursor(2, 0);
      lcd.print( s_thermo );
      lcd.setCursor(2, 1);
      lcd.print(  s_resd);
      if (s_thermo  == error_temp && alarm_temp) {
        lcd.setCursor(2, 0);
        lcd.print("Err");
        lcd.setCursor(2, 1);
        lcd.print("Err");
      };
    } else {
      lcd.setCursor(2, 0);
      lcd.print("off");
      lcd.setCursor(2, 1);
      lcd.print("off");
    };

    //Fen
    lcd.setCursor(5, 0);
    lcd.print("F:");
    lcd.setCursor(5, 1);
    lcd.print("H:");
    if (digitalRead(SW2) == HIGH) {
      lcd.setCursor(7, 0);
      lcd.print( f_thermo );
      lcd.setCursor(7, 1);
      lcd.print( f_resd);
      if ( f_thermo == error_temp  && alarm_temp) {
        lcd.setCursor(7, 0);
        lcd.print("Err");
        lcd.setCursor(7, 1);
        lcd.print("Err");
      };
    } else {
      lcd.setCursor(7, 0);
      lcd.print("off");
      lcd.setCursor(7, 1);
      lcd.print("off");
    };

    //Downheating
    lcd.setCursor(15, 0);
    lcd.print("\2C");
    lcd.setCursor(15, 1);
    lcd.print("\1C");
    lcd.setCursor(10, 0);
    lcd.print("D:");
    lcd.setCursor(10, 1);
    lcd.print("H:");
    if (digitalRead(SW4) == HIGH) {
      lcd.setCursor(12, 0);
      lcd.print( dh_thermo);
      lcd.setCursor(12, 1);
      lcd.print(  dh_resd);
      if (dh_thermo  == error_temp && alarm_temp) {
        lcd.setCursor(12, 0);
        lcd.print("Err");
        lcd.setCursor(12, 1);
        lcd.print("Err");
      };
    } else {
      lcd.setCursor(12, 0);
      lcd.print("off");
      lcd.setCursor(12, 1);
      lcd.print("off");
    };

  };
};
