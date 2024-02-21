#define BLYNK_TEMPLATE_ID "TMPLog-noKXa"
#define BLYNK_DEVICE_NAME "IC Robótica"
#define BLYNK_AUTH_TOKEN "NVoMKS5IlRQhnccyjx98zjtQyM7dl2YP";

#define BLYNK_PRINT Serial

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>

#include <heltec.h>
#include <Servo.h>
#include <MPU9250_asukiaaa.h>
#define BLYNK_PRINT Serial

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 4
#define SCL_PIN 15
#endif
static const int servosPins[2] = {14,12};

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Monica";
char pass[] = "demopejoma";

BlynkTimer timer;
MPU9250_asukiaaa mySensor;
Servo servo[2];

int QD = 0;
int QE = 1;
int JD = 2;
int JE = 3;
int TD = 4;
int TE = 5;


//// This function is called every time the Virtual Pin 0 state changes
//void atualizaGrafico(float *anglex, float *angley){
//  // Update state
//  Blynk.virtualWrite(V1, *anglex);
//  Blynk.virtualWrite(V0, *angley);
//}
//
//// This function is called every time the device is connected to the Blynk.Cloud
//BLYNK_CONNECTED(){
//  // Change Web Link Button message to "Congratulations!"
//  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
//  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
//  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
//}

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


float getZzmp(float *gyro){
  float zmp = -(*gyro)/(9.8*0.5);
  return zmp;
}

float getAlpha(float *dist){
  float alpha;
  alpha = acos((pow((*dist),2) - 380.88)/(-380.88));
  alpha = alpha*180/(3.14159256);
  return alpha;
 }

float getTempo(float *dist, float *angle, float *gyro){
  float tempo;
  float gama = acos(((*dist))/(27.6));
  gama = gama*180/(3.14159256);
  float teta = 90 - gama;
  tempo = (teta - *angle)/(*gyro);  
  return tempo;
} 

void applyPenduloD(float* alpha, float *tempo){
  int mov_joelho = (int) alpha /3;
  int mov_quad = (int) alpha /2;
  
  //levantar perna direita, joelho direito, abre quadril esquerdo

  servo[JD].write(90 - mov_joelho/10);
  servo[QD].write(90 + mov_quad/10);
  servo[QE].write(90 + mov_quad/10);

  //abaixa o joelho (corrigir dps)
  servo[JD].write(90);
  
  //propulsiona o tornozelo de trás e levanta quadril
  int pos_quad = 90 + mov_quad;
  int pos_joelho = 90 -mov_joelho;
  servo[TE].write(90 - (45)/10);  
  servo[QE].write(pos_quad - mov_quad/10);
  servo[JE].write(pos_joelho + mov_joelho/10);
  servo[QD].write(pos_quad - mov_quad/10);
  servo[TE].write(90);
  
}

void applyPenduloE(float* alpha, float *tempo){
  int mov_joelho = (int) alpha /3;
  int mov_quad = (int) alpha /2;
  
  //levantar perna direita, joelho direito, abre quadril esquerdo

  servo[JE].write(90 - mov_joelho/10);
  servo[QE].write(90 + mov_quad/10);
  servo[QD].write(90 + mov_quad/10);

  //abaixa o joelho (corrigir dps)
  servo[JE].write(90);
  
  //propulsiona o tornozelo de trás e levanta quadril
  int pos_quad = 90 + mov_quad;
  int pos_joelho = 90 -mov_joelho;
  servo[TD].write(90 - (45)/10);  
  servo[QD].write(pos_quad - mov_quad/10);
  servo[JD].write(pos_joelho + mov_joelho/10);
  servo[QE].write(pos_quad - mov_quad/10);

  servo[TD].write(90);
  
}


void setup() {
  Heltec.display->init(); 
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "I'm Batman");
  Heltec.display->drawString(0, 10, "RBPI");
  Heltec.display->drawString(0, 20, "Teste: RBPI1-2");
  Heltec.display->display();
  delay(1000);
  Heltec.display->clear();
  
  #ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN,SCL_PIN);
  mySensor.setWire(&Wire);
  #endif

  mySensor.beginAccel();
  mySensor.beginGyro();

  //Blynk.begin(auth, ssid, pass);

  for (int i = 0; i < 2; i++){
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
  float mgX, mgY, mgZ;
  float maX, maY, maZ;
  float angleX, angleZ;
  float Zzmp, tempoX, dist;
  float alpha;
  lerSensorAccel(&maX, &maY, &maZ);
  lerSensorGyro(&mgX, &mgY, &mgZ);

  angleX = getAngle(&maX,&maY,'z');
  angleZ = getAngle(&maZ,&maY,'x');

  Zzmp = getZzmp(&mgY);
  dist = 2 * Zzmp;
  tempoX = getTempo(&dist, &angleX, &mgY);

  if (angleZ < -5 || angleZ > 5){
    alpha = getAlpha(&dist);
    if(angleX > 0){
      applyPenduloD(&alpha, &tempoX);
    }else {
      applyPenduloE(&alpha, &tempoX);
    }
  }

  
  
  //Blynk.run();
  //timer.run();

  //atualizaGrafico(&angleX,&angleY);
  Heltec.display->clear();
  Heltec.display->drawString(0,0,"Ângulo X =");
  Heltec.display->drawString(55,0,(String)angleX);
  Heltec.display->drawString(0,10,"Ângulo Y =");
  Heltec.display->drawString(55,10,(String)angleZ);
  Heltec.display->display();
  delay(10);

}
