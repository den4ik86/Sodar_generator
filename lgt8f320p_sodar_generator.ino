#include "math.h"
#include "AD9833.h"
#include <LiquidCrystal_I2C.h>

#define CS1  8
#define CS2  9
#define CS3  10
#define TTL1 7
#define TTL2 6
#define btn_long_push 1000   // Длительность долинного нажатия кнопки

volatile uint8_t lastcomb=7, enc_state, btn_push=0;
volatile int enc_rotation=0, btn_enc_rotate=0;
volatile boolean btn_press=0;
volatile uint32_t timer;

uint8_t  heightIndex = 0;
uint8_t  stepIndex = 0;
uint8_t  bufferOut[300]={0};
bool     StopFlag = true;
uint8_t  flag = 0;
int      frq1;
int      frq2;
int      frq3;
uint8_t  rotate_i = 0;
uint16_t count = 0;
int      i = 0;
uint8_t  j = 0;

byte InBuff_UART = B00000000;

int height[4] = {2941, 4117, 5882, 200};
double tukiVal[2] = {0.0067, 0.00335};
int FrqTable[11] = {1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500, 2600};

AD9833 AD1, AD2, AD3;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void tukeyWin(){
  float r = 0.4;
  float x = 0.0;
  float y = 0.0;
  bufferOut[300]={0};

  while(r/2 > x && x >= 0){
    y = (1+cos((3.1415926535*2)/r*(x-r/2)))*0.5;
    x += tukiVal[stepIndex];
    bufferOut[i] = int(y*255);
    Serial.println(bufferOut[i]);
    i++;
  }
  while((r/2) <= x && x < (1-r/2)){
    y=1;
    x += tukiVal[stepIndex];
    bufferOut[i] = int(y*255);
    Serial.println(bufferOut[i]);
    i++;
  }
  while((1-r/2) <=x && x <= 1){
    y = (1+cos((3.1415926535*2)/r*(x-(1+r/2))))*0.5;
    x += tukiVal[stepIndex];
    bufferOut[i] = int(y*255);
    Serial.println(bufferOut[i]);
    i++;
  }
  count = i;
  i = 0;
  Serial.println(count);
}

ISR(TIMER1_COMPA_vect){
  if(flag == 1){
    analogWrite(DAC0, bufferOut[i]);
    i++;
    if(count == i){
      digitalWrite(TTL2, LOW);
      flag = 0;
    }
  }
}

int rotate(){
  Serial.println("Start rotate");
  if(rotate_i == 0){
    AD1.setFrequency(frq1, 0);
    AD2.setFrequency(frq2, 0);
    AD3.setFrequency(frq3, 0);
    rotate_i++;

    lcd.setCursor(0,1);
    lcd.print(frq1);
    lcd.setCursor(6, 1);
    lcd.print(frq2);
    lcd.setCursor(12, 1);
    lcd.print(frq3);
  
    return rotate_i;
  }
  if(rotate_i == 1){
    AD1.setFrequency(frq2, 0);
    AD2.setFrequency(frq3, 0);
    AD3.setFrequency(frq1, 0);
    rotate_i++;
      
    lcd.setCursor(0,1);
    lcd.print(frq2);
    lcd.setCursor(6, 1);
    lcd.print(frq3);
    lcd.setCursor(12, 1);
    lcd.print(frq1);
  
    return rotate_i;
  }
  if(rotate_i == 2){
    AD1.setFrequency(frq3, 0);
    AD2.setFrequency(frq1, 0);
    AD3.setFrequency(frq2, 0);
    rotate_i = 0;  
    
    lcd.setCursor(0,1);
    lcd.print(frq3);
    lcd.setCursor(6, 1);
    lcd.print(frq1);
    lcd.setCursor(12, 1);
    lcd.print(frq2);
  
    return rotate_i;
  }
}

void menu(){
  if(j == 1){
    lcd.clear();
    lcd.print("Set F CH1:");
    lcd.setCursor(0, 1);
  }
  if(j == 2){
    frq1 = FrqTable[enc_rotation];
    lcd.clear();
    lcd.print("Set F CH2:");
    lcd.setCursor(0, 1);
  }  
  if(j == 3){
    frq2 = FrqTable[enc_rotation];
    lcd.clear();
    lcd.print("Set F CH3:");
    lcd.setCursor(0, 1);
  }
  if(j == 4){
    frq3 = FrqTable[enc_rotation];
    enc_rotation = 0;
    lcd.clear();
    lcd.print("Height meters:");
    lcd.setCursor(0, 1);
  }
  if(j == 5){
    enc_rotation = 0;
    lcd.clear();
    lcd.print("Duration Millis:");
    lcd.setCursor(0, 1);
  }  
}

void normalMode(){
  lcd.clear();
  lcd.print(" F1    F2    F3 ");
  lcd.setCursor(0,1);
  lcd.print(frq1);
  lcd.setCursor(6, 1);
  lcd.print(frq2);
  lcd.setCursor(12, 1);
  lcd.print(frq3);
}

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.print("Starting system");
  lcd.setCursor(0, 1);
  lcd.print("testing...");
  delay(1000);
  lcd.clear();
  
  frq1 = FrqTable[3];
  frq2 = FrqTable[5];
  frq3 = FrqTable[7];

  lcd.print(" F1    F2    F3 ");
  lcd.setCursor(0,1);
  lcd.print(frq1);
  lcd.setCursor(6, 1);
  lcd.print(frq2);
  lcd.setCursor(12, 1);
  lcd.print(frq3);
  
  noInterrupts();
  TCNT1 = 0;
  TCCR1A = 0;
  TCCR1C = 0;
  byte n = 2;

  TCCR1B = (1 << WGM12) | n;
  OCR1A = 4000;// 0-65535
  TIMSK1 |= (1 << OCIE1A);
  
  //_ctc = f_sys / (2 * n * (1 + OCR1A))
  // 32 000 000 / (2 * 1024 * (1 + 9999)) = 1.5625
/*
  pinMode(A1,INPUT_PULLUP); // ENC-A
  pinMode(A2,INPUT_PULLUP); // ENC-B
  pinMode(A3,INPUT_PULLUP); // BUTTON
*/
  PCICR  |=  (1 << PCIE1); // PCICR |= (1<<PCIE1); Включить прерывание PCINT1
  PCMSK1  = 0b00001110; // Разрешить прерывание для  A1, A2, A3

  Serial.begin(115200);
  
  analogReference(DEFAULT);
  pinMode(DAC0, ANALOG);

  pinMode(TTL1, OUTPUT);
  digitalWrite(TTL1, LOW);
  pinMode(TTL2, OUTPUT);
  digitalWrite(TTL2, LOW);
  
  AD1.begin(CS1, 11, 13);  //  software SPI
  AD2.begin(CS2, 11, 13);
  AD3.begin(CS3, 11, 13);
 
  AD1.setFrequency(frq1, 0);
  AD2.setFrequency(frq2, 0);
  AD3.setFrequency(frq3, 0);

  AD1.setPhase(0, 0);
  AD2.setPhase(0, 0);
  AD3.setPhase(0, 0);

  AD1.setWave(AD9833_SINE);
  AD2.setWave(AD9833_SINE);
  AD3.setWave(AD9833_SINE);

  tukeyWin();
  interrupts();
}
  
void loop() {
  if(millis()%height[heightIndex] == 0 && StopFlag){
    rotate();
    digitalWrite(TTL1, HIGH);
    delay(10);
    digitalWrite(TTL2, HIGH);
    flag = 1;
  } 
  if(count == i){
    i = 0;
    delay(10);
    digitalWrite(TTL1, LOW);
      
  }

///////////////////////////////////////////////////////////////////////////  
  if(!StopFlag){
    if(j < 4){
      if(enc_rotation >10){
        enc_rotation = 0;
      }
      if(enc_rotation < 0){
        enc_rotation = 10;         
      }
      Serial.println("setup mode");
      lcd.setCursor(0, 1);
      lcd.print(FrqTable[enc_rotation]);
      
    }
    if(j == 4){
      heightIndex = enc_rotation;
      if(enc_rotation == 0){
        lcd.setCursor(0, 1);
        lcd.print("500");
      }
      if(enc_rotation == 1){
        lcd.setCursor(0, 1);
        lcd.print("700");
      }
      if(enc_rotation == 2){
        lcd.setCursor(0, 1);
        lcd.print("1000");
      } 
      if(enc_rotation == 3){
        lcd.setCursor(0, 1);
        lcd.print("test mode");
      }
      if(enc_rotation > 3 | enc_rotation < 0){
        enc_rotation = 0;
        menu();
      }
    }
    if(j == 5){
      if(enc_rotation == 0){
        lcd.setCursor(0, 1);
        lcd.print("75 ms");
        stepIndex = 0;
      }
      if(enc_rotation == 1){
        lcd.setCursor(0, 1);
        lcd.print("150 ms");
        stepIndex = 1;
      }

      if(enc_rotation > 1 | enc_rotation < 0){
        enc_rotation = 0;
        menu();
      }
    }
}
//////////////отслеживание энкодера.
  switch (enc_state){
    case 1:{
      Serial.print("Вращение без нажатия ");
      lcd.backlight(); 
      //Serial.println(FrqTable[enc_rotation]);
    }
    break;
   
    case 2:{
      Serial.print("Вращение с нажатием ");
      Serial.println(btn_enc_rotate);  
    }
    break;
    
    case 3:{
      Serial.println("Нажатие кнопки ");
      j++;
      if(j == 1){
        StopFlag = false;
        menu();
      }
      if(j == 2){
        StopFlag = false;
        menu();
      }
      if(j == 3){
        StopFlag = false;
        menu();
      }
      if(j == 4){
        StopFlag = false;
        menu();
      }
      if(j == 5){
        StopFlag = false;
        menu();
      }

      if(j == 6){
        tukeyWin();
        normalMode();
        StopFlag = true;
        j = 0;
      }
    }
    break;

    case 4:{
      Serial.println("Длинное нажатие кнопки ");
      lcd.noBacklight();
    }
    break;
  }
  enc_state=0; //обнуляем статус энкодера
}

////// Оброботчик преываний от энкодера
ISR (PCINT1_vect){ //Обработчик прерывания от пинов A1, A2, A3
  //Serial.println("interrupt");
  uint8_t comb = bitRead(PINC, 3) << 2 | bitRead( PINC, 2)<<1 | bitRead(PINC, 1); //считываем состояние пинов энкодера и кнопки
  //Serial.println(comb);
  if (comb == 3 && lastcomb == 7) btn_press=1; //Если было нажатие кнопки, то меняем статус
 
  if (comb == 4){                         //Если было промежуточное положение энкодера, то проверяем его предыдущее состояние 
    if (lastcomb == 5){ 
      --enc_rotation; //вращение по часовой стрелке
    }
    if (lastcomb == 6){
      ++enc_rotation; //вращение против часовой
    }
    enc_state=1;                       // был поворот энкодера    
    btn_enc_rotate=0;                  //обнулить показания вращения с нажатием
  }
  
   if (comb == 0)                      //Если было промежуточное положение энкодера и нажатие, то проверяем его предыдущее состояние 
   {
    if (lastcomb == 1) --btn_enc_rotate; //вращение по часовой стрелке
    if (lastcomb == 2) ++btn_enc_rotate; //вращение против частовой
    enc_state=2;                        // был поворот энкодера с нажатием  
    enc_rotation=0;                     //обнулить показания вращения без нажатия
    btn_press=0;                         //обнулить показания кнопки
   }

   if (comb == 7 && lastcomb == 3 && btn_press) //Если было отпускание кнопки, то проверяем ее предыдущее состояние 
   {
    if (millis() - timer > btn_long_push)         // проверяем сколько прошло миллисекунд
    {
      enc_state=4;                              // было длинное нажатие 
    } else {
             enc_state=3;                    // было нажатие 
            }
      btn_press=0;                           //обнулить статус кнопки
    }
   
  timer = millis();                       //сброс таймера
  lastcomb = comb;                        //сохраняем текущее состояние энкодера
}
