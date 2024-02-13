//Projekt inzynierski "Programowalny robot jezdzacy po linii", Albert Pintera, 2022

#include <QTRSensors.h>

//Nastawy regulatora PID
#define Kp 0.08
#define Ki 0.01
#define Kd 0.2

double kp = Kp;
double ki = Ki;
double kd = Kd;

//Zmienne predkosci
#define max_speed 160
#define def_speed 130
#define speed_change 140

int def_speed_change = def_speed;

//Silniki
#define left_motor_in1 A1
#define left_motor_in2 A2
#define left_motor_PWM 11
#define right_motor_in3 A3
#define right_motor_in4 A4
#define right_motor_PWM 10

#define left_sensor A0
#define right_sensor A5

QTRSensors qtr;

int temp = 0;

const uint8_t sensor_count = 8; //Liczba czujnikow
uint16_t sensor_values[sensor_count]; //Tablica czujnikow

void setup()
{
  //Inicjalizacja listwy czujnikow odbiciowych QTR-8RC
  qtr.setTypeRC(); //Typ czujnikow - cyfrowy
  qtr.setSensorPins((const uint8_t[]) {3, 4, 5, 6, 7, 8, 9, 12}, sensor_count);
  qtr.setEmitterPin(2);

  //Wyjscia sygnalowe silnikow
  pinMode(left_motor_in1, OUTPUT);
  pinMode(left_motor_in2, OUTPUT);
  pinMode(left_motor_PWM, OUTPUT);
  pinMode(right_motor_in3, OUTPUT);
  pinMode(right_motor_in4, OUTPUT);
  pinMode(right_motor_PWM, OUTPUT);

  //Wejscia listwy czujnikow odbiciowych QTR-8RC
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(12, INPUT);

  //Wejscia czujnikow odbiciowych IR - LM393
  pinMode(left_sensor, INPUT);
  pinMode(right_sensor, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  delay(2000);
  Serial.begin(9600);

  //Inicjalizacja wyswietlania stanu poczatkowego
  Serial.print("*LR0G0B0*");
  Serial.print("*RR0G130B0*");
  Serial.print("*WR0G0B0*");
}

//Zmienne predkosci
int motor_speed;
int right_motor_speed, left_motor_speed;
int last_error = 0;
int last_sensor = 0;

//Zmienna okreslajaca aktualna pozycje czujnikow odbiciowych QTR-8RC
uint16_t sensors_position;

//Zmienne regulatora PID
int P, I = 0, D;
int error;

//Zmienne okreslajace aktualny tryb pracy robota
int state = 1;
int counter = 0;

void loop()
{
  //Tryb 0 - podazanie po linii
  if(state == 0)
  {
    if(counter == 1)
    {
      //Kalibracja listwy czujnikow odbiciowych QTR-8RC
      calibration();

      def_speed_change = 80;
      
      counter++;
      Serial.print("Tryb podazania po linii");
      Serial.print("*LR255G255B255*");
      Serial.print("*RR0G0B0*");
      Serial.print("*WR0G0B0*");

      Serial.print("*sSP=");
      Serial.print(def_speed_change);
      Serial.print("*");

      Serial.println();
    }

    //Kalibracja listwy czujnikow odbiciowych QTR-8RC
    calibration();
    
    //Rozpoznawanie pozycji czujnikow na postawie wykrywania czarnej linii
    sensors_position = qtr.readLineBlack(sensor_values);
    
   
    //Korekcja jazdy na granicach
    if(!digitalRead(A0) == 0 && digitalRead(A5) == 0)
    {
      move_motors(2, speed_change, 2);
      move_motors(1, speed_change, 1);
      return;
    }
    else if(digitalRead(A0) == 0 && !digitalRead(A5) == 0)
    {
      move_motors(2, speed_change, 1);
      move_motors(1, speed_change, 2);
      return;
    }
    else if(digitalRead(A0) == 0 && digitalRead(A5) == 0)
    {
      move_motors(2, def_speed_change, 2);
      move_motors(1, def_speed_change, 2);
    }
    
    //Zachowanie robota w przypadku najechania na poprzeczną linię - zatrzymanie się
    else if(!digitalRead(A0) == 0 && !digitalRead(A5) == 0)
    {
      counter = 1;
      state = 2;
    }

     //Regulator PID
     error = sensors_position - 3500; //Uchyb regulacji;

     //Wyliczanie wartosci poszczegolnych czlonow regulatora
     P = kp * error;
     I = ki * (I + error);
     D = kd * (error - last_error);

     motor_speed = P + I + D;

     last_error = error;

     right_motor_speed = def_speed_change + motor_speed;
     left_motor_speed = def_speed_change - motor_speed;

     //Ograniczenia predkosci maksymalnej i minimalnej
     if (right_motor_speed > max_speed) {
       right_motor_speed = max_speed;
     }
     if (left_motor_speed > max_speed) {
       left_motor_speed = max_speed;
     }
     if (right_motor_speed < 0) {
       right_motor_speed = 0;
     }
     if (left_motor_speed < 0) {
       left_motor_speed = 0;
     }

     //Ruch robota
     move_motors(2, right_motor_speed, 2);
     move_motors(1, left_motor_speed, 2);
    
    //Zdalna zmiana trybu oraz zmiany nastaw regulatora
    if(Serial.available())
    {
      uint8_t data_change = Serial.read();
      bluetooth(data_change);
    }
  }
  
  //Tryb 1 - zdalne sterowanie
  if(state == 1)
  { 
    if(counter == 1)
    { 
      counter++;
      def_speed_change = 130;
      Serial.print("Tryb zdalnego sterowania");
      Serial.print("*LR0G0B0*");
      Serial.print("*RR255G255B255*");
      Serial.print("*WR0G0B0*");

      Serial.print("*pKp*");
      Serial.print("*iKi*");
      Serial.print("*dKd*");

      Serial.print("*sSP=");
      Serial.print(def_speed_change);
      Serial.print("*");
      
      Serial.println();
    }
    if(Serial.available())
    {
      uint8_t data_change = Serial.read();
      bluetooth(data_change);
    }
  }

  //Tryb 2 - zatrzymanie i oczekiwanie na komende
  if(state == 2)
  {
    if(counter == 1)
    {
      counter++;
      Serial.print("Tryb oczekiwania");
      Serial.print("*LR0G0B0*");
      Serial.print("*RR0G0B0*");
      Serial.print("*WR255G255B255*");
      Serial.println();
    }
    stop_motors();
    if(Serial.available())
    {
      uint8_t data_change = Serial.read();
      bluetooth(data_change);
    }
  }
}

//Zatrzymanie silnikow
void stop_motors()
{
  analogWrite(right_motor_PWM, 0);
  analogWrite(left_motor_PWM, 0);
}

//Ruch robota
void move_motors(int motor, int mspeed, int move_direction)
{
  boolean in1;
  boolean in2;

  //Ruch do tylu
  if (move_direction == 1)
  {
    in1 = HIGH;
    in2 = LOW;
  }
  //Ruch do przodu
  if (move_direction == 2)
  {
    in1 = LOW;
    in2 = HIGH;
  }

  //Silnik lewy
  if (motor == 1)
  {
    digitalWrite(left_motor_in1, in1);
    digitalWrite(left_motor_in2, in2);
    analogWrite(left_motor_PWM, mspeed);
  }
  //Silnik prawy
  if (motor == 2)
  {
    digitalWrite(right_motor_in3, in1);
    digitalWrite(right_motor_in4, in2);
    analogWrite(right_motor_PWM, mspeed);
  }
}

//Komunikacja bluetooth
void bluetooth(uint8_t data)
{
  if(state == 0)
  {
    switch(data)
    {
      case 'x':
          break;
      case '2':
          counter = 1;
          stop_motors();
          state = 1;
          break;
      case 'p':
          kp = kp - 0.01;
          Serial.println("Kp = ");
          Serial.print(kp);
          Serial.print("*pKp=");
          Serial.print(kp);
          Serial.print("*");
          Serial.println();
          break;
      case 'P':
          kp = kp + 0.01;
          Serial.println("Kp = ");
          Serial.print(kp);
          Serial.print("*pKp=");
          Serial.print(kp);
          Serial.print("*");
          Serial.println();
          break;
      case 'i':
          ki = ki - 0.01;
          Serial.println("Ki = ");
          Serial.print(ki);
          Serial.print("*iKi=");
          Serial.print(ki);
          Serial.print("*");
          Serial.println();
          break;
      case 'I':
          ki = ki + 0.01;
          Serial.println("Ki = ");
          Serial.print(ki);
          Serial.print("*iKi=");
          Serial.print(ki);
          Serial.print("*");
          Serial.println();
          break;
      case 'd':
          kd = kd - 0.01;
          Serial.println("Kd = ");
          Serial.print(kd);
          Serial.print("*dKd=");
          Serial.print(kd);
          Serial.print("*");
          Serial.println();
          break;
      case 'D':
          kd = kd + 0.01;
          Serial.println("Kd = ");
          Serial.print(kd);
          Serial.print("*dKd=");
          Serial.print(kd);
          Serial.print("*");
          Serial.println();
          break;
      case 's':
          if(def_speed_change >= 0)
          {
            def_speed_change = def_speed_change - 10;
            Serial.print("*sSP=");
            Serial.print(def_speed_change);
            Serial.print("*");
            Serial.println();
          }
          break;
      case 'S':
          if(def_speed_change <= 255)
          {
            def_speed_change = def_speed_change + 10;
            Serial.print("*sSP=");
            Serial.print(def_speed_change);
            Serial.print("*");
            Serial.println();
          }
          break;
      case 'R':
          kp = Kp;
          ki = Ki;
          kd = Kd;
          Serial.print("*pKp=");
          Serial.print(kp);
          Serial.print("*");
          Serial.print("*iKi=");
          Serial.print(ki);
          Serial.print("*");
          Serial.print("*dKd=");
          Serial.print(kd);
          Serial.print("*");
      default:
          break;
    }
  }
  else if(state == 1)
  {
    switch(data)
    {
      case '0':
          stop_motors();
          break;
      case '1':
          counter = 1;
          state = 0;
          temp = 0;
          break;
      case '2':
          counter = 0;
          state = 1;
          stop_motors();
          break;
      case '3':
          move_motors(1,def_speed_change,2);
          move_motors(2,def_speed_change,2);
          break;
      case '4':
          move_motors(1,def_speed_change,1);
          move_motors(2,def_speed_change,1);
          break;
      case '5':
          move_motors(1,160,2);
          move_motors(2,160,1);
          break;
      case '6':
          move_motors(1,160,1);
          move_motors(2,160,2);
          break;
      case 's':
          if(def_speed_change >= 0)
          {
            def_speed_change = def_speed_change - 10;
            Serial.print("*sSP=");
            Serial.print(def_speed_change);
            Serial.print("*");
            Serial.println();
          }
          break;
      case 'S':
          if(def_speed_change <= 255)
          {
            def_speed_change = def_speed_change + 10;
            Serial.print("*sSP=");
            Serial.print(def_speed_change);
            Serial.print("*");
            Serial.println();
          }
          break;
      default:
          break;
    }
  }
  else if(state == 2)
  {
    switch(data)
    {
      case '0':
          stop_motors();
          break;
      case '1':
          counter = 1;
          state = 0;
          temp = 0;
          break;
      case '2':
          counter = 1;
          state = 1;
          stop_motors();
          break;
      default:
          break;
    }
  }
  delay(100);
}

//Kalibracja listwy czujnikow odbiciowych
void calibration()
{
  if (temp == 0)
  {
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.print("*LR255G0B0*");
    Serial.print("*RR0G0B0*");
    Serial.print("*WR0G0B0*");

    //Mozliwosc sterowania robotem podczas trwania procesu kalibracji - zapewnienie wiekszej dokladnosci    
    for (int i = 0; i < 100; i++)
    {
      uint8_t data_change = Serial.read();

      switch(data_change)
      {
        case '0':
          stop_motors();
          break;
        case '5':
          move_motors(1,140,2);
          move_motors(2,140,1);
          break;
        case '6':
          move_motors(1,140,1);
          move_motors(2,140,2);
          break;
        default:
          break;
      }
      qtr.calibrate();
      delay(50);
    }
    stop_motors();
    digitalWrite(LED_BUILTIN, LOW);
    delay(2000);
    temp = 1;
  }
}
