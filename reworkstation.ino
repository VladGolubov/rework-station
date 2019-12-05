 /*   --------------------------------------------------
  * | Имя    : Паяльная станция                        |
  * | Автор  : Влад Голубов                            |
  * | Дата   :  08.06.2019                             |
  * | Версия : 0.4                                     |
  *  --------------------------------------------------
  * Добавлено:
  * 1.PID регулятор 
  * 2.Нижний подогрев
  * Функции:
  * 1. Проверка температуры термопарой(терморезистором) на паяльнике(s_thermPin), 
  *    если температура меньше установленной резистором (s_res), 
  *    то включается высокий сигнал на выводе(s_ten) 
  * 2. Проверка температуры термопарой(терморезистором) на фене(f_thermPin), 
  *    если температура меньше установленной резистором (f_res), 
  *    то включается высокий сигнал на выводе(f_ten) 
  * 3. Если темература фена(f_thermPin) > минимальной температуры(f_fanAlarm), 
  *    то на вывод (f_fan) включается высокий сигнал
  * Материалы: 
  * . 
 */

#include      <max6675.h>                            // Подключаем библиотеку max6675 для работы с датчиком температуры 
const uint8_t thermoDO  = 4;                         // Определяем константу с указанием № вывода Arduino к которому подключён вывод DO  ( SO, MISO ) модуля на чипе MAX6675
const uint8_t thermoCS  = 5;                         // Определяем константу с указанием № вывода Arduino к которому подключён вывод CS  ( SS )       модуля на чипе MAX6675
const uint8_t thermoCLK = 6;                         // Определяем константу с указанием № вывода Arduino к которому подключён вывод CLK ( SCK )      модуля на чипе MAX6675
MAX6675       thermoco(thermoCLK, thermoCS, thermoDO); // Объявляем объект thermo для работы с функциями и методами библиотеки max6675, указывая выводы ( CLK , CS , DO )


// #include <LiquidCrystal.h>
 #include <PIDController.h>
 #include <Wire.h> 
 #include <LiquidCrystal_I2C.h> // Подключение библиотеки

 LiquidCrystal_I2C lcd(0x27,16,2);
 //Константы
 #define max_temp = 400
 #define f_fanAlarm 50
 #define alarm_temp 450
 #define error_temp 0 
#define B 3950 // B-коэффициент
#define SERIAL_R 220 // сопротивление последовательного резистора, 220 Ом
#define THERMISTOR_R 60 // номинальное сопротивления термистора, 60 Ом
#define NOMINAL_T 25 // номинальная температура (при которой TR = 60 Ом)
 //Аналоговые входы
 //#define f_thermPin A2 //PC2 --
 //#define s_thermPin A3 //PC3 --
 #define s_res A1 //PC1
 #define f_res A0 //PC0
//#define dh_thermPin A4 //PC4 A6 --
// #define dh_res A2 //PC5 A7
 //Цифровые входы
 #define s_ten 9 //PB1
 #define f_ten 10 //PB2 поменять на 8
// #define dh_ten 11 //PB4, перепаять с вентилятора на нижний подогрев!
 #define f_fan 12 //PB3
 #define SW1 8 //PD6 d6 0 A4 паяльник
 #define SW2 7 //PD7 d7 1 A5 фен
 #define SW3 2 //PB0 8 геркон 
// #define SW4 51 3 //PB5 3
 
 //Переменнные 
//int dh_thermo 
int f_thermo ;
int s_thermo ;
int s_resd ;
int f_resd ;
//int dh_resd ;

//float steinhart;
//float tr;

//PIDController pid;
PIDController pid_solder;
PIDController pid_fen;
//PIDController pid_dh;

//LiquidCrystal lcd( 27, 26, 25, 24, 23, 22); //PD0, PD1, PD2, PD3, PD4, PD5 
//                 rs 6, en 7, d4 2, d5 3, d6 4, d7 5

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("light_side v1.2");
  lcd.setCursor(0,1);
  lcd.print("Rework Station");
  delay(1000);
  lcd.clear();
  
   pid_solder.begin(); 
   pid_fen.begin();   
//   pid_dh.begin();      
//   pid_dh.tune(1, 1, 1);    // Tune the PID, arguments: kP, kI, kD
//   pid_dh.limit(0, 255);    
   pid_fen.tune(1, 1, 1);    // Tune the PID, arguments: kP, kI, kD
   pid_fen.limit(0, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
   pid_solder.tune(1, 1, 1);    // Tune the PID, arguments: kP, kI, kD
   pid_solder.limit(0, 255); // Limit the PID output between 0 and 255, t
   pid_solder.setpoint(400); 
   pid_fen.setpoint(400);  
  pinMode(f_fan, OUTPUT);
  pinMode(s_ten, OUTPUT);
  pinMode(f_ten, OUTPUT);
//  pinMode(dh_ten, OUTPUT);
  pinMode(s_thermPin, INPUT);
  pinMode(f_thermPin, INPUT);
//  pinMode(dh_thermPin, INPUT);
  pinMode(s_res, INPUT);
  pinMode(f_res, INPUT);
//  pinMode(dh_res, INPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
//  pinMode(SW4, INPUT);
}


void loop() {
/* void control_dh(){  
 dh_thermo = analogRead(dh_thermPin);
 dh_thermo = map(dh_thermo, 0, 1024, 0, 400);
 dh_thermo = constrain(dh_thermo, 0, 400);
  dh_resd = analogRead(dh_res);
  dh_resd = map(dh_resd, 0, 1024, 0, 400);
 dh_resd = constrain(dh_resd, 0, 400); 
  if ( SW4 == HIGH){
       pid_dh.setpoint(dh_resd);   
    int output3 = pid_dh.compute(dh_thermo);
    analogWrite(dh_ten, output3);
    }
 }*/
/* void display_lcd();
 //Sold 
 lcd.clear();
 if(digitalRead(SW1) == HIGH){
  lcd.setCursor(0,1);
   lcd.print("Fen:");
   lcd.print(f_thermo);
   lcd.print(" ");
   lcd.print(f_resd);
   }else{
//    lcd.setCursor(0,1);
   lcd.print("Fen:off");}
   
 lcd.setCursor(0,0);
    lcd.print("S:");
    lcd.setCursor(0,1);
    lcd.print("H:");  
if(digitalRead(SW1) == HIGH){  
  lcd.setCursor(2,0);
   lcd.print(s_thermo);
   //delay(10);
  lcd.setCursor(2,1);
   lcd.print(s_resd);
 //  delay(10);
   }else{
    if(s_thermo == error_temp && alarm_temp){  
    lcd.setCursor(2,0);
    lcd.print("Err");
    lcd.setCursor(2,1);
    lcd.print("Err");
  }else{
  lcd.setCursor(2,0);
    lcd.print("off");
    lcd.setCursor(2,1);
    lcd.print("off");
}
 //Sold
 //Fen  
    lcd.setCursor(5,0);
    lcd.print("F:");
    lcd.setCursor(5,1);
    lcd.print("H:");
if(digitalRead(SW2) == HIGH){
  lcd.setCursor(7,0);
  lcd.print(f_thermo);
 // delay(10);
  lcd.setCursor(7,1);
  lcd.print(f_resd);
//  delay(10);
}else{
  if(f_thermo == error_temp  && alarm_temp){  
    lcd.setCursor(7,0);
    lcd.print("Err");
    lcd.setCursor(7,1);
    lcd.print("Err");
  }else{
    lcd.setCursor(7,0);
    lcd.print("off");
    lcd.setCursor(7,1);
    lcd.print("off");
  }
 //Fen
 //Downheating

  lcd.setCursor(10,0);
    lcd.print("D:");
    lcd.setCursor(10,1);
    lcd.print("H:"); 
 if(digitalRead(SW4) == HIGH){
  lcd.setCursor(12,0);
   lcd.print(dh_thermo);
 //  delay(10);
  lcd.setCursor(12,1);
   lcd.print(dh_resd);
 //  delay(10);
 }else{
  if(dh_thermo == error_temp && alarm_temp){  
    lcd.setCursor(12,0);
    lcd.print("Err");
    lcd.setCursor(12,1);
    lcd.print("Err");
  }else{
   lcd.setCursor(12,0);
    lcd.print("off");
    lcd.setCursor(12,1);
    lcd.print("off"); 
  //Downheating*/
 

//void control_solder();
/*s_thermo = analogRead(s_thermPin);
 s_thermo = map(s_thermo, 0, 1024, 0, 400);
 s_thermo = constrain(s_thermo, 0, 400);
/*float tr = 1023.0 / s_thermo - 1;
    tr = SERIAL_R / tr;  
    float steinhart;
    steinhart = tr / THERMISTOR_R; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_T + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; 
  steinhart = s_thermo;*/
 s_thermo = thermoco.readCelsius();  
 delay(100);
  s_resd = analogRead(s_res);
   s_resd = map(s_resd, 0, 1024, 0, 400);
    s_resd = constrain(s_resd, 0, 400);
 if (SW1 == HIGH ){
  //     pid_solder.setpoint(s_resd);   
    int output1 = pid_solder.compute(s_thermo);
    analogWrite(f_ten, output1);
  }
// void control_fen();  
 f_thermo = analogRead(f_thermPin);
 f_thermo = map(f_thermo, 0, 1024, 0, 400);
 f_thermo = constrain(f_thermo, 0, 400);
  f_resd = analogRead(f_res);
  f_resd = map(f_resd, 0, 1024, 0, 400);
 f_resd = constrain(f_resd, 0, 400); 
  if (SW2 == HIGH &&  SW3 == LOW){
    //   pid_fen.setpoint(f_resd);   
    int output2 = pid_fen.compute(f_thermo);
    analogWrite(f_ten, output2);
    }
//void control_turn();
 // if (f_thermo >  f_fanAlarm && SW3 == HIGH ){ ошибка 
 //digitalWrite(f_fan, HIGH);}
 // else{ 
 //  digitalWrite(f_fan, LOW);} 
// void ALARMdisp();
  if(f_thermo > 450 ){
   digitalWrite(f_ten, LOW);}
 //  }else{ 
 //  digitalWrite(f_ten, HIGH);
 //  }
   if(s_thermo > 450 ){
   digitalWrite(s_ten, LOW);}
 //  }else{ 
 //  digitalWrite(s_ten, HIGH);
 //  }
//   if(dh_thermo > 450 ){
  // digitalWrite(dh_ten, LOW);
  // }else{ 
  // digitalWrite(dh_ten, HIGH);
  //}

   
// lcd.clear();
  lcd.setCursor(0, 0);
    lcd.print("Solder:");
  if (digitalRead(SW1) == HIGH){
    lcd.print(s_thermo);
    lcd.print(" ");
    lcd.setCursor(12, 0);
    lcd.print(s_res);
    lcd.print(" ");
  }
  else{
    lcd.print(" Off");
  }
  // Данные фена на дисплей
  lcd.setCursor(0, 1);
  lcd.print("Fen:");
  lcd.setCursor(4, 1);
  if (digitalRead(SW2) == HIGH){
    lcd.print(f_thermo);
    lcd.print(" ");
    lcd.setCursor(9, 1);
    lcd.print(f_res);
    lcd.print(" ");
  }
 else{
    lcd.print(" Off");
  }
  delay(200);
   }
