#include <Servo.h>

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <math.h>

#include <heltec.h>
#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 4
#define SCL_PIN 15
#endif
static const int servosPins[6] = {12, 16, 13, 23, 21,18};
static const int base[6] = {128, 48, 105, 60, 120, 55};
static const int soma1[10] = {10, 20, 30, 40, 50, 40, 30, 20, 10, 0};
static const int mult[10] = {1, 1, 2, 2, 3, 3, 2, 2, 1, 1};

MPU9250_asukiaaa mySensor;
Servo servo[6];

int QE = 0;
int QD = 1;
int JE = 2;
int JD = 3;
int TE = 4;
int TD = 5;


//leitura dos sensores em accel. x y e z
float mediaSensorAccelX(){
  float maX = 0;
  for(int i = 0; i < 10; i++){
    if(mySensor.accelUpdate() == 0){
      maX += mySensor.accelX();
    }
  }
  maX = maX/10;
  return maX;
}
float mediaSensorAccelY(){
  float maY = 0;
  for(int i = 0; i < 10; i++){
    if(mySensor.accelUpdate() == 0){
      maY += mySensor.accelY();
    }
  }
  maY = maY/10;
  return maY;
}
float mediaSensorAccelZ(){
  float maZ = 0;
  for(int i = 0; i < 10; i++){
    if(mySensor.accelUpdate() == 0){
      maZ += mySensor.accelZ();
    }
  }
  maZ = maZ/10;
  return maZ;
}

//leitura dos sensores em gyroscópio, x y e z
float mediaSensorGyroX(){
  float mgX = 0;
  for (int i = 0; i < 10; i++){
    if(mySensor.gyroUpdate() == 0){
      mgX += mySensor.gyroX();
    }
  }
  mgX = mgX / 10;
  return mgX;
}
float mediaSensorGyroY(){
  float mgY = 0;
  for (int i = 0; i < 10; i++){
    if(mySensor.gyroUpdate() == 0){
      mgY += mySensor.gyroY();
    }
  }
  mgY = mgY / 10;
  return mgY;
}
float mediaSensorGyroZ(){
  float mgZ = 0;
  for (int i = 0; i < 10; i++){
    if(mySensor.gyroUpdate() == 0){
      mgZ += mySensor.gyroZ();
    }
  }
  mgZ = mgZ / 10;
  return mgZ;
}

void lerSensorAccel(float *maX, float *maY, float *maZ){
  *maX = mediaSensorAccelX();
  *maY = mediaSensorAccelY();
  *maZ = mediaSensorAccelZ();
}

void lerSensorGyro(float *mgX, float *mgY, float *mgZ){
  *mgX = mediaSensorGyroX();
  *mgY = mediaSensorGyroY();
  *mgZ = mediaSensorGyroZ();
}

float getAngle(float *accelFrente, float *accelCima, char eixo){
  float aux_vel1, aux_vel2, accelGyro;
  float alpha, beta, angle;
  switch (eixo){
     case 'x':
       aux_vel1 = mySensor.gyroX();
       delay(15);
       aux_vel2 = mySensor.gyroX();
       break;
     case 'y':
       aux_vel1 = mySensor.gyroY();
       delay(15);
       aux_vel2 = mySensor.gyroY();
       break;
     case 'z':
       aux_vel1 = mySensor.gyroZ();
       delay(15);
       aux_vel2 = mySensor.gyroZ();
       break;
  }
  accelGyro = ((aux_vel2 - aux_vel1)/0.015);
  beta = (0 + (aux_vel1*0.015) + (accelGyro/2)*(0.015*0.015));
  alpha = atan((*accelFrente)/(*accelCima));
  angle = alpha*0.98 + beta*0.02;
  angle = (angle*180)/(3.14159);
  return angle;
}


float getZzmp(float *cm, float* accel){
  float AccelZ = (*accel)*9.8;
  float aux_vel1, aux_vel2;
  float zmp1 = AccelZ*(*cm)/(9.8);
  aux_vel1 = mySensor.gyroX();
  delay(15);
  aux_vel2 = mySensor.gyroX();
  float gyro = (aux_vel2-aux_vel1)/15;
  gyro = gyro*(3.14159)/180;
  float zmp2 = (gyro*(pow(*cm,2))/(9.8));
  zmp1 = 0.98*zmp1 + 0.02*zmp2;
  return zmp1;
}

float getAlpha(float *dist, float *junta){
  float alpha;
  alpha = (atan((*dist)/(*junta)));
  return alpha;
}

float getBeta(float* alpha, float* dist, float* junta){
  float aux, leg = 0.0685, beta;
  aux = (*dist)/sin(*alpha);
  beta = (acos(aux/(2*leg)));
  return beta;
}

float getEpsilon(float *dist, float *junta){
  float ep = atan(2*(*dist)/(*junta));
  return ep;
}

float getGama(float *dist, float *junta){
  float leg = 0.0685;
  float c = sqrt(pow(2*(*dist),2) + pow ((*junta),2));
  float gama = acos((c*c - 2*leg*leg)/(2*leg*leg));
  return gama;
}


void applyPenduloD2(float* cm, float* junta){
  float Zzmp, alpha, beta, angleZ, p, leg = 0.0685;
  float varQD, varJD, varTD, varQE, varJE, varTE;
  float mgX, mgY, mgZ;
  float maX, maY, maZ;
  
  //abrir a perna, levantar quadril
  lerSensorAccel(&maX, &maY, &maZ);
  lerSensorGyro(&mgX, &mgY, &mgZ);
  Zzmp = getZzmp(cm, &maZ);
  Serial.print("Zmp: ");
  Serial.print(Zzmp);
  Serial.print("\n");
  alpha = getAlpha(&Zzmp,junta);
  beta = getBeta(&alpha, &Zzmp, junta);
  alpha = alpha*180/3.14159;
  beta = beta*180/3.14159;
  p =  180 - 2*beta;
  
  
  varQD = abs((alpha + beta) - abs(90 - base[TD] - 20));
  varJD = abs(p - (180 - base[JD]));
  varTD = abs((180 - (90 - alpha) - beta) - abs(base[TD]) + 30);
  varQE = abs(alpha + abs(90 - base[TE] - 10) - beta);
  varJE = abs (p - base[JE] - 15);;
  varTE = abs((90 - alpha - beta) - abs(180 - base[TE] + 15));
 

  
  for(int i = 1; i <= 10; i++){
    servo[QD].write(base[QD] - mult[i -1]*(varQD));
    servo[QE].write(base[QE] - i*(varQE)/10);
    servo[JD].write(base[JD] + soma1[i-1] - i*(varJD)/10);
    servo[JE].write(base[JE] - i*(varJE)/10);
    servo[TD].write(base[TD] - soma1[i-1] + i*varTD/10);
    servo[TE].write(base[TE] + i*(varTE)/10);   
    if(mySensor.accelUpdate() == 0){
      maZ = mySensor.accelZ();
    }
    Serial.print("Estágio 1: ");
    Serial.print(getZzmp(cm,&maZ));
    Serial.print("\n");
    delay(50);
  }

  //"""pousar"""
  delay(1000);
  // novos cálculos
  float ep = getEpsilon(&Zzmp, junta);
  float gama = getGama(&Zzmp, junta);
  ep = ep*180/3.14159;
  gama = gama*180/3.14159;
  float eta = (180 - gama)/2;

  float delta = 90 - eta - ep;

  float lambda = base[TE] + varTE;
  float d = sqrt(leg*leg + Zzmp*Zzmp + 2*leg*Zzmp*cos(lambda*3.14159/180));
  float v = acos((leg*leg - d*d - Zzmp*Zzmp)/(2*d*Zzmp));
  v = v*180/3.14159;
  float kapa = delta - v;
  float varQE2 = acos( (d*d/2*leg) - leg/2 + d*cos(kapa*3.14159/180)); 
  varQE2 = varQE2*180/3.14159 - 25;
  
  float varJE2, varTE2;
  
  varJE2 = abs (gama - (base[JE] + varJE));
  varTE2 = abs((kapa + v) - abs(180 - base[TE] - varTE) + 25);
  

  //ajusta o corpo para frente
  for(int i = 1; i <= 10; i++){
    servo[QE].write(base[QE] - varQE - i*varQE2/10);
    servo[QD].write(base[QD] - varQD + i*varQD/10);
    servo[JE].write(base[JE] + varJE + i*varJE2/10);
    servo[JD].write(base[JD] - varJD + i*varJD/10);
    servo[TE].write(base[TE] + varTE - i*varTE2/10);
    servo[TD].write(base[TD] + varTD - i*varTD/10);
    if(mySensor.accelUpdate() == 0){
      maZ = mySensor.accelZ();
    }
    Serial.print("Estágio 2: ");
    Serial.print(getZzmp(cm,&maZ));
    Serial.print("\n");
    delay(50);
  }

  delay(1000);

  //puxa a perna
  for(int i = 1; i <= 10; i++){
    servo[QE].write(base[QE] - varQE - varQE2 + i*varQE/10 + i*varQE2/10);
    servo[JE].write(base[JE] - soma1[i-1] + varJE - i*varJE/10 + varJE2 - i*varJE2/10);
    servo[TE].write(base[TE] + soma1[i-1] +varTE - varTE2 - i*varTE/10 + i*varTE2/10 );
    if(mySensor.accelUpdate() == 0){
      maZ = mySensor.accelZ();
    }
    Serial.print("Estágio 3: ");
    Serial.print(getZzmp(cm,&maZ));
    Serial.print("\n");
    delay(50);
  }

  //setar base
  setBase();
  delay(1000);
}

void setBase(){
  for(int i = 0; i< 6; i++){
    servo[i].write(base[i]);
  }
}


void setup() {
  Serial.begin(115200);
  Heltec.display->init(); 
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "I'm Batman");
  Heltec.display->drawString(0, 10, "RBPI");
  Heltec.display->drawString(0, 20, "Teste: RBPI8.5");
  Heltec.display->display();
  delay(1000);
  Heltec.display->clear(); 
  
  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN,SCL_PIN);
  mySensor.setWire(&Wire);
  #endif

  mySensor.beginAccel();
  mySensor.beginGyro();

  for (int i = 0; i < 6; i++){
    if(!servo[i].attach(servosPins[i])){
      Heltec.display->clear();
      Heltec.display->drawString(0,10*i,"Error at servo");
      Heltec.display->drawString(40,10*i,(String)i);
      Heltec.display->display();
      delay(1000);
      
    }
  }

}

void loop() {
  float angleZ;
  float Zzmp, tempoZ;
  float cm = 0.1906;
  float junta = 0.1080;

  setBase();
  delay(5000);
  //applyPenduloD2(&cm, &junta);

  

}
