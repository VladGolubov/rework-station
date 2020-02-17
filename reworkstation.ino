 /*   --------------------------------------------------
  * | Имя    : Паяльная станция                          |
  * | Автор  : Влад Голубов                              |
  * | Дата   :  05.01.2020                               |
  * | Версия :  2.0.2                                    |
  *   --------------------------------------------------
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
  * 
 */

/*#include      <max6675.h>                            // Подключаем библиотеку max6675 для работы с датчиком температуры 
const uint8_t thermoDO  = 4;                           // Определяем константу с указанием № вывода Arduino к которому подключён вывод DO  ( SO, MISO ) модуля на чипе MAX6675
const uint8_t thermoCS  = 5;                           // Определяем константу с указанием № вывода Arduino к которому подключён вывод CS  ( SS )       модуля на чипе MAX6675
const uint8_t thermoCLK = 6;                           // Определяем константу с указанием № вывода Arduino к которому подключён вывод CLK ( SCK )      модуля на чипе MAX6675
MAX6675       thermoco(thermoCLK, thermoCS, thermoDO); // Объявляем объект thermo для работы с функциями и методами библиотеки max6675, указывая выводы ( CLK , CS , DO )
не знаю заработакт или нет*/
//----------------------------------------------------------------------------------------------

 #include <SPI.h> 
 #include <PIDController.h>
 #include <Wire.h> 
 #include <LiquidCrystal_I2C.h> 

//----------------------------------------------------------------------------------------------
 

 LiquidCrystal_I2C lcd(0x27,16,2);
 /*Константы*/
 #define max_temp = 500
 #define f_fanAlarm 50
 #define alarm_temp 550
 #define error_temp 0 
 /*
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
 int16_t  f_ten = 10;  //PB2 поменять на 8
 int16_t  dh_ten = 11; //PB4, перепаять с вентилятора на нижний подогрев!
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
//float steinhart; если используем терморезистор, то разкомментировать
//float tr;        и перепаять на плате 

PIDController pid;
PIDController pid_solder;
PIDController pid_fen;
PIDController pid_dh;

/*рабочий вариант отображения

  lcd.setCursor(0, 0);
    lcd.print("Solder:");
  if (digitalRead(SW1) == HIGH){
    //delay(200);
    lcd.print(s_thermo);
    lcd.print(" ");
    lcd.setCursor(12, 0);
    lcd.print(s_resd);
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
// delay(200);
    lcd.print(f_thermo);
    lcd.print(" ");
    lcd.setCursor(9, 1);
    lcd.print(f_resd);
    lcd.print(" ");
  }
 else{
  
    lcd.print(" Off");
  }
  delay(300);

lcd.clear();
*/
/*Рабочий вариант для терморезистора
 * float tr = 1023.0 / s_thermo - 1;
    tr = SERIAL_R / tr;  
    float steinhart;
    steinhart = tr / THERMISTOR_R; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart /= B; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_T + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invert
    steinhart -= 273.15; 
  steinhart = s_thermo;*/

void control_solder(){
 if (digitalRead(SW1)== HIGH ){
  s_resd = analogRead(s_res);
   s_resd = map(s_resd, 0, 1024, 0, 500);
    s_resd = constrain(s_resd, 0, 500);
 /* if(s_thermo < s_resd){
    digitalWrite(s_ten, HIGH);
    delay(50);
  }else{
     digitalWrite(s_ten, LOW);
  } рабочий кусок кода
 }*/
    pid_solder.setpoint(s_resd);   
    int16_t output1 = pid_solder.compute(s_thermo);
    analogWrite(s_ten, output1);
    if(s_thermo > alarm_temp ){
      
    }
   digitalWrite(s_ten, LOW);
 }
};

void control_fen(){
 if (digitalRead(SW2)== HIGH && digitalRead(SW3)== LOW ){
  f_resd = analogRead(f_res);
   f_resd = map(f_resd, 0, 1024, 0, 500);
    f_resd = constrain(f_resd, 0, 500);
 /* if(f_thermo < f_resd){
    digitalWrite(f_ten, HIGH);
    delay(50);
  }else{
     digitalWrite(f_ten, LOW);
  }
 }
*/
     pid_fen.setpoint(f_resd);   
     int16_t output2 = pid_fen.compute(f_thermo);
     analogWrite(f_ten, output2);
      if (f_thermo >  f_fanAlarm  || digitalRead(SW3) == HIGH ){
         digitalWrite(f_fan, HIGH);}
      else{ 
         digitalWrite(f_fan, LOW);} 
     if (f_thermo > alarm_temp ){
         digitalWrite(f_ten, LOW);
   }
 }
};

void control_dh(){
 if (digitalRead(SW4)== HIGH ){
  dh_resd = analogRead(dh_res);
   dh_resd = map(dh_resd, 0, 1024, 0, 500);
    dh_resd = constrain(dh_resd, 0, 500);
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
      if (dh_thermo > alarm_temp ){
         digitalWrite(s_ten, LOW);
       }
   }
}; 

void display_lcd(){
 //Solder
 lcd.setCursor(0,0);
    lcd.print("S:");
    lcd.setCursor(0,1);
    lcd.print("H:"); 
  
if(digitalRead(SW1) == HIGH){  
  lcd.setCursor(1,0);
   lcd.print(s_thermo);
   //delay(10);
  lcd.setCursor(1,1);
   lcd.print(s_res);
 //  delay(10);
   }else{
    if(s_thermo == error_temp && alarm_temp){  
    lcd.setCursor(1,0);
    lcd.print("Err");
    lcd.setCursor(1,1);
    lcd.print("Err");
  }else{
  lcd.setCursor(1,0);
    lcd.print("off");
    lcd.setCursor(1,1);
    lcd.print("off");
  }
   }
 //Fen  
    lcd.setCursor(5,0);
    lcd.print("F:");
    lcd.setCursor(5,1);
    lcd.print("H:");
if(digitalRead(SW2) == HIGH){
  lcd.setCursor(6,0);
  lcd.print(f_thermo);
 // delay(10);
  lcd.setCursor(6,1);
  lcd.print(f_res);
//  delay(10);
}else{
  if(f_thermo == error_temp  && alarm_temp){  
    lcd.setCursor(6,0);
    lcd.print("Err");
    lcd.setCursor(6,1);
    lcd.print("Err");
  }else{
    lcd.setCursor(6,0);
    lcd.print("off");
    lcd.setCursor(6,1);
    lcd.print("off");
  }
}

 //Downheating
  lcd.setCursor(10,0);
    lcd.print("D:");
    lcd.setCursor(10,1);
    lcd.print("H:"); 
 if(digitalRead(SW4) == HIGH){
  lcd.setCursor(11,0);
   lcd.print(dh_thermo);
 //  delay(10);
  lcd.setCursor(11,1);
   lcd.print(dh_res);
 //  delay(10);
 }else{
  if(dh_thermo == error_temp && alarm_temp){  
    lcd.setCursor(11,0);
    lcd.print("Err");
    lcd.setCursor(11,1);
    lcd.print("Err");
  }else{
   lcd.setCursor(11,0);
    lcd.print("off");
    lcd.setCursor(11,1);
    lcd.print("off"); 
  //Downheating
  }
 }
}


double readCelsius(uint8_t cs) { 
 uint16_t v; 

 digitalWrite(cs, LOW); 
 v = SPI.transfer(0x00); 
 v <<= 8; 
 v |= SPI.transfer(0x00); 
 digitalWrite(cs, HIGH); 

 if (v & 0x4) { 
  // uh oh, no thermocouple attached! 
  return NAN; 
 } 

 v >>= 3; 

 return v*0.25; 
} 
 
void setup() {
  SPI.begin();
 Serial.end();
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
   pid_dh.begin();      
   pid_dh.tune(1, 1, 1);    
   pid_dh.limit(0, 500);    
   pid_fen.tune(1, 1, 1);    
   pid_fen.limit(0, 500);   
   pid_solder.tune(1, 1, 1); // Tune the PID, arguments: kP, kI, kD
   pid_solder.limit(0, 500); // Limit the PID output between 0 and 255, t
 
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
 pinMode(1, OUTPUT); 
 pinMode(4, OUTPUT); 
 pinMode(5, OUTPUT); 
 digitalWrite(1, HIGH); 
 digitalWrite(4, HIGH); 
 digitalWrite(5, HIGH); 

}


void loop() {
  
   s_thermo = readCelsius(1);
   f_thermo = readCelsius(4);
   dh_thermo = readCelsius(5);

   analogRead(s_res);
   analogRead(f_res);
   analogRead(dh_res);
 control_solder();
 control_dh();
 control_fen();
 display_lcd();

 }
  
  
