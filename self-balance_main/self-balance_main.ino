#include <Wire.h>
#define LED_PIN 12
#define START_PIN 12
#define BATT_PIN A0
#define DURATA 4000                              //Durata loop in microsecondi       
#define NBYTES 5                                 //Lunghezza pacchetti inviati

//---------------------------------Costanti giroscopio------------------------------------------//
int gyro_address = 0x68;
int offset_acc = 1682;

//---------------------------------Variabili giroscopio-----------------------------------------//
int gyr_rawX, gyr_rawY, gyr_rawZ, acc_rawZ;
long offset_yaw, offset_pitch;
float angolo_acc, angolo, setpoint_baricentro;

//-------------------------------------Variabili PID--------------------------------------------//
float Kp = 20;
float Ki = 0.5;
float Kd = 50;
float vel_rotazione = 50;
float max_target_speed = 250;
float errore, pid_integrale, pid_setpoint, gyro_input, pid_output, errore_prec;
float output_DX, output_SX;

//-------------------------------------Calcolo impulsi stepper----------------------------------//
int motore_DX, f_impulso_motoriDX, subroutine_contatoreDX, subroutine_memoryDX;
int motore_SX, f_impulso_motoriSX, subroutine_contatoreSX, subroutine_memorySX;

//--------------------------------------------Tempo---------------------------------------------//
unsigned long durata_loop;

//------------------------------------------Utility---------------------------------------------//
byte start, batt_scarica;
bool disattiva;
int contatore_loop, direzione;
float batteria;
char byte_ricevuto;
int bytes_inviati;
int f_campionamento = 96;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//****************************************************************************************************************************************************//
//**************************************************************   AVVIO ROBOT  **********************************************************************//
//****************************************************************************************************************************************************//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(START_PIN, OUTPUT);
  pinMode(BATT_PIN, INPUT);
  digitalWrite(START_PIN, HIGH);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                        SETUP SUBROUTINE
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.begin();                                                             //Avvia bus I2C come master
  TWBR = 12;                                                                //Frequenza I2C a 400kHz
  TCCR2A = 0;                                                               //Inizializza il registro TCCR2A  a 0
  TCCR2B = 0;                                                               //Inizializza il registro TCCR2A  a 0
  TIMSK2 |= (1 << OCIE2A);                                                  //Setta il bit di interrupt enable OCIE2A nel registro TIMSK2
  TCCR2B |= (1 << CS21);                                                    //Setta il bit CS21 nel registro TCCRB per avere un prescaler a 8
  OCR2A = 39;                                                               //Il registro comparatore è settato a 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                        SETUP GYRO&ACC                                                             //
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Sveglia l'MPU-5060
  Wire.write(0x6B);
  Wire.write(0x00);                                                         //Setta bit registro a 00000000 per attivare il giroscopio
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1B);
  Wire.write(0x00);                                                         //Setta bit registro a 00000000 (250dps full scale)
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1C);
  Wire.write(0x08);                                                         //Setta bit registro a 00001000 (+/- 4g full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(gyro_address);
  Wire.write(0x1A);
  Wire.write(0x03);                                                         //Setta bit registro a 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();

  //----------------Calibrazione offset giroscopio-----------------//
  for (int i = 0; i < 1000; i++)
  {
    Wire.beginTransmission(gyro_address);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 4);
    offset_yaw += Wire.read() << 8 | Wire.read();
    offset_pitch += Wire.read() << 8 | Wire.read();
    delayMicroseconds(1000);
  }
  offset_pitch /= 1000;
  offset_yaw /= 1000;

  //-------------------------Registra tempo-----------------------//
  contatore_loop = 0;
  durata_loop = micros() + DURATA;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//****************************************************************************************************************************************************//
//**************************************************************   MAIN LOOP  ************************************************************************//
//****************************************************************************************************************************************************//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                        ANGOLO
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //---------------------Calcolo Accellerometro------------------//
  Wire.beginTransmission(gyro_address);
  Wire.write(0x3F);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 2);
  acc_rawZ = Wire.read() << 8 | Wire.read();
  acc_rawZ += offset_acc;
  if (acc_rawZ > 8200)acc_rawZ = 8200;
  if (acc_rawZ < -8200)acc_rawZ = -8200;
  angolo_acc = -asin((float)acc_rawZ / 8200.0) * 57.296;
  if (angolo_acc > 0)angolo_acc = angolo_acc + angolo_acc * 0.24;
  if (angolo_acc > 90)angolo_acc = 90;
  if (angolo_acc < -90)angolo_acc = -90;

  //---------------------Calcolo Giroscopio----------------------//
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);
  gyr_rawX = Wire.read() << 8 | Wire.read();
  gyr_rawY = Wire.read() << 8 | Wire.read();
  gyr_rawY -= offset_pitch;
  gyr_rawY = gyr_rawY / 131.0;                                                  //Velocità angolare in gradi/s

  //---------Calcolo angolo filtro complementare acc+gyr)---------//
  angolo = 0.999 * (angolo + gyr_rawY * 0.004061) + 0.001 * angolo_acc;
  if (angolo > 90) angolo = 90;
  if (angolo < -90)angolo = -90;

  //*****************ATTIVAZIONE BILANCIAMENTO*******************//
  batteria = analogRead(BATT_PIN) / 70.8;
  if (start == 0 && disattiva == false && angolo_acc >  -0.5 && angolo_acc <  0.5)
  {
    angolo = angolo_acc;
    start = 1;
    digitalWrite(START_PIN, LOW);
  }
  //****************DISATTIVAZIONE BILANCIAMENTO******************//
  if (angolo > 30 || angolo < -30 || start == 0 || batt_scarica == 1 || disattiva == true)
  {
    pid_output = 0;
    pid_integrale = 0;
    setpoint_baricentro = 0;
    start = 0;
    digitalWrite(START_PIN, HIGH);
  }
  //*************************************************************//

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                            PID
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  errore = angolo - setpoint_baricentro - pid_setpoint;

  if (pid_output > 10 || pid_output < -10)errore += pid_output * 0.015 ;           //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_integrale +=  errore;
  if (Ki * pid_integrale > 400)pid_integrale = 400 / Ki;                           //Limita il controllo integrale al massimo dell'output di controllo
  else if (Ki * pid_integrale < -400)pid_integrale = -400 / Ki;

  pid_output = Kp * errore + Ki * pid_integrale + Kd * (errore - errore_prec);
  errore_prec = errore;

  if (pid_output > 400)pid_output = 400;                                           //Limita il controllo
  else if (pid_output < -400)pid_output = -400;

  //if (pid_output < 1 && pid_output > -1)pid_output = 0;                          //Dead-band

  if (pid_setpoint == 0 && pid_output < 0)setpoint_baricentro += 0.0015;           //Incrementa il setpoint del baricentro se il robot va ancora avanti
  if (pid_setpoint == 0 && pid_output > 0)setpoint_baricentro -= 0.0015;           //Decrementa il setpoint del baricentro se il robot va ancora indietro

  output_SX = pid_output;
  output_DX = pid_output;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                  COMUNICATION
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //----------------------Ricezione comandi----------------------//
  if (Serial.available())
  {
    byte_ricevuto = Serial.read();
    if (byte_ricevuto == 'p')
    {
      Kp = Serial.parseFloat();
    }
    if (byte_ricevuto == 'i')
    {
      Ki = Serial.parseFloat();
      pid_integrale = 0;
    }
    if (byte_ricevuto == 'd')
    {
      Kd = Serial.parseFloat();
    }
    if (byte_ricevuto == 'm')
    {
      disattiva = Serial.parseInt();
    }

    if (byte_ricevuto == 'c')
    {
      f_campionamento = Serial.parseInt();
      contatore_loop = 0;
    }

    if (byte_ricevuto == 'e')
      direzione = 4;

    if (byte_ricevuto == 'q')
      direzione = 3;

    if (byte_ricevuto == 's')
      direzione = 2;

    else if (byte_ricevuto == 'w')
      direzione = 1;

    if (byte_ricevuto == 'f')
      direzione = 0;
  }
  //----------------------Invio dati------------------------------//
  else {

    if (contatore_loop == f_campionamento / 24) {
      bytes_inviati = Serial.print("G");
      bytes_inviati += Serial.print((int)gyr_rawY);
      for (int i = 0; i < (NBYTES - bytes_inviati); i++)
        Serial.print("#");
    }
    else if (contatore_loop == (f_campionamento / 24) * 2)
    {
      bytes_inviati = Serial.print("A");
      bytes_inviati += Serial.print(round(angolo_acc * 10));
      for (int i = 0; i < (NBYTES - bytes_inviati); i++)
        Serial.print("#");
    }
    else if (contatore_loop == (f_campionamento / 24) * 3)
    {
      bytes_inviati = Serial.print("T");
      bytes_inviati += Serial.print(round(angolo * 10));
      for (int i = 0; i < (NBYTES - bytes_inviati); i++)
        Serial.print("#");
    }
    else if (contatore_loop == (f_campionamento / 24) * 4)
    {
      bytes_inviati = Serial.print("M");
      bytes_inviati += Serial.print((int)pid_output * start);
      for (int i = 0; i < (NBYTES - bytes_inviati); i++)
        Serial.print("#");
    }
    else if (contatore_loop == (f_campionamento / 24) * 5)
    {
      bytes_inviati = Serial.print("B");
      bytes_inviati += Serial.print(round(batteria * 10));
      for (int i = 0; i < (NBYTES - bytes_inviati); i++)
        Serial.print("#");
    }
    else if (contatore_loop == (f_campionamento / 24) * 6)
    {
      bytes_inviati = Serial.print("E");
      bytes_inviati += Serial.print(round(errore * 10));
      for (int i = 0; i < (NBYTES - bytes_inviati); i++)
        Serial.print("#");
      contatore_loop = 0;
    }
    contatore_loop++;
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                        PILOTAGGIO
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  switch (direzione)
  {
    case 0:
      if (pid_setpoint > 0.5)pid_setpoint -= 0.05;
      else if (pid_setpoint < -0.5)pid_setpoint += 0.05;
      else pid_setpoint = 0;
      break;
    case 1:
      if (pid_setpoint < 3)pid_setpoint += 0.05;
      if (pid_output < max_target_speed)pid_setpoint += 0.005;
      break;
    case 2:
      if (pid_setpoint > -3)pid_setpoint -= 0.05;
      if (pid_output > max_target_speed * -1)pid_setpoint -= 0.005;
      break;
    case 3:
      output_SX -= vel_rotazione;
      output_DX += vel_rotazione;
      if (pid_setpoint < 2.8)pid_setpoint += 0.1;
      break;
    case 4:
      output_SX += vel_rotazione;
      output_DX -= vel_rotazione;
      if (pid_setpoint < 2.8)pid_setpoint += 0.1;
      break;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                          COMPENSAZIONE NON-LINEARE STEPPER
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if (output_DX > 0)output_DX = 405 - (1 / (output_DX + 9)) * 5500;
  else if (output_DX < 0)output_DX = -405 - (1 / (output_DX - 9)) * 5500;

  if (output_SX > 0)output_SX = 405 - (1 / (output_SX + 9)) * 5500;
  else if (output_SX < 0)output_SX = -405 - (1 / (output_SX - 9)) * 5500;

  if (output_DX > 0)f_impulso_motoriDX = 400 - output_DX;
  else if (output_DX < 0)f_impulso_motoriDX = -400 - output_DX;
  else f_impulso_motoriDX = 0;

  if (output_SX > 0)f_impulso_motoriSX = 400 - output_SX;
  else if (output_SX < 0)f_impulso_motoriSX = -400 - output_SX;
  else f_impulso_motoriSX = 0;



  while (durata_loop > micros());
  durata_loop += DURATA;

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                               SUB-ROUTINE IMPULSI MOTORI
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER2_COMPA_vect) {

  subroutine_contatoreDX ++;                                                             //Incrementa il contatore ad ogni loop della subroutine (quindi ogni 20 us)
  if (subroutine_contatoreDX > subroutine_memoryDX) {                                    //Se il numero del loop è maggiore del valore memorizzato
    subroutine_contatoreDX = 0;                                                          //Resetta il contatore
    subroutine_memoryDX = f_impulso_motoriDX;                                            //Carica in memoria la prossima frequenza di impulso
    if (subroutine_memoryDX < 0) {                                                       //Se la memoria è negativa
      PORTB &= ~(1 << PORTB3);                                                           //Setta la direzione negativa al pin 11 (LOW)
      subroutine_memoryDX *= -1;                                                         //Rendi positiva la memoria (per il confronto col contatore al prossimo loop)
    }
    else PORTB |= (1 << PORTB3);                                                         //Altrimenti setta la direzione positiva al pin 11 (HIGH)
  }
  else if (subroutine_contatoreDX == 1)PORTD |= B01000000;                               //Al primo loop fai partire l'impulso ai pin degli step 6
  else if (subroutine_contatoreDX == 2)PORTD &= B10111111;                               //Al secondo loop fai terminare l'impulso ai pin degli step 6 (impulso da 20us)

  subroutine_contatoreSX ++;                                                             //Incrementa il contatore ad ogni loop della subroutine (quindi ogni 20 us)
  if (subroutine_contatoreSX > subroutine_memorySX) {                                    //Se il numero del loop è maggiore del valore memorizzato
    subroutine_contatoreSX = 0;                                                          //Resetta il contatore
    subroutine_memorySX = f_impulso_motoriSX;                                            //Carica in memoria la prossima frequenza di impulso
    if (subroutine_memorySX < 0) {                                                       //Se la memoria è negativa
      PORTB &= ~(1 << PORTB2);                                                           //Setta la direzione negativa al pin 10 (LOW)
      subroutine_memorySX *= -1;                                                         //Rendi positiva la memoria (per il confronto col contatore al prossimo loop)
    }
    else PORTB |= (1 << PORTB2);                                                         //Altrimenti setta la direzione positiva al pin 10 (HIGH)
  }
  else if (subroutine_contatoreSX == 1)PORTD |= B00100000;                               //Al primo loop fai partire l'impulso ai pin degli step 5
  else if (subroutine_contatoreSX == 2)PORTD &= B11011111;                               //Al secondo loop fai terminare l'impulso ai pin degli step 5 (impulso da 20us)

}
