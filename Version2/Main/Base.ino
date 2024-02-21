#include <MPU9250_asukiaaa.h>
#include <Servo.h>

MPU9250_asukiaaa mySensor = MPU9250_asukiaaa(0x68);
MPU9250_asukiaaa mySensor2 = MPU9250_asukiaaa(0x69);


Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;


// Variaveis da cinemática inversa
float ang_quad_dir_x = 130, ang_quad_dir_y = 90, ang_quad_esq_x = 60, ang_quad_esq_y = 90;
float ang_joel_dir = 70, ang_joel_esq = 120, ang_torn_dir = 140, ang_torn_esq = 30;
float leg = 0.060;
float posCM[3] = {0, 0, 0.145}; //alt_max = 14.5
float posPD[3] = {0, 0.045, 0};
float posPE[3] = {0, -0.045, 0};
float offset = atan(0.04 / 0.045);
float dQ = 0.0602, t = 0;
float leg2 = leg * leg;
float retaH_dir, retaH_esq, dif_reta_dir;
float retaH_dir2, retaH_esq2, dif_reta_esq;
float toRad = 3.14159 / 180;
float toAng = 180 / 3.14159;

// Variaveis do controle
float matrix_A [3][3] = {{1.0156, 0.0251, -0.0156}, {1.2540, 1.0156, -1.2540}, {0, 0, 1}};
float vector_B [3] = { -0.0001, -0.0156, 0.0250};
float vector_C [3] = {0, 0, 1};
float result_L [3] = {0, 0, 0}, result_B[3] = {0, 0, 0};
float result_L_x[3] = {0, 0, 0}, result_B_x[3] = {0, 0, 0};
float vector_L [3] = { -10421, -129.9237, 107.0006};
float state_vector [3] = {0, 0, 0};
float state_vector_x[3] = {0, 0, 0};
float Gi = 0.0025, Gx[3] = {0, 0, 0.0025}, Gd = 65;
float Gix = 0.0005, Gxx[3] = {0, 0, 0.0025}, Gdx = 25;
int  Nx = 30, Ny = 8;
float somatorio = 0, somatorio_x = 0;
float r = 0.125, g = -9.8, aux, aux2, aux3, calc_zmp, aux4, aux5, aux6;
float auxx, aux2x, aux3x, calc_zmp_x, aux4x, aux5x, aux6x;
float distPDE = 0.02, altura = 0.03, count = -3, sideAngle = 0, frontAngle = 0;
int walk = 0, control = 0, agachar = 1, k = 0;
float pref[63] = {0.000000, -0.006678, -0.013084, -0.018971, -0.024150,
                  -0.028496, -0.031964, -0.034576, -0.036416, -0.037607,
                  -0.038291, -0.038614, -0.038704, -0.038671, -0.038593,
                  -0.038526, -0.038500, -0.038526, -0.038593, -0.038671,
                  -0.038704, -0.038614, -0.038291, -0.037607, -0.036416,
                  -0.034576, -0.031964, -0.028496, -0.024150, -0.018972,
                  -0.013084, -0.006679, -0.000000, 0.006678, 0.013084,
                  0.018971, 0.024150, 0.028496, 0.031964, 0.034576,
                  0.036416, 0.037607, 0.038291, 0.038614, 0.038704, 
                  0.038671, 0.038593, 0.038526, 0.038500, 0.038526, 
                  0.038593, 0.038671, 0.038704, 0.038614, 0.038291, 
                  0.037607, 0.036416, 0.034576, 0.031964, 0.028497, 
                  0.024150, 0.018972, 0.006679};
float prefx[63] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
                   0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
                   0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
                   0.000000, 0.000000, 0.000000, 0.000000, 0.000800,
                   0.001600, 0.002400, 0.003200, 0.004000, 0.004800,
                   0.005600, 0.006400, 0.007200, 0.008000, 0.008800,
                   0.009600, 0.010400, 0.011200, 0.012000, 0.012800,
                   0.013600, 0.014400, 0.015200, 0.016000, 0.016800,
                   0.016800, 0.016800, 0.016800, 0.016800, 0.016800,
                   0.016800, 0.016800, 0.016800, 0.016800, 0.016800,
                   0.016800, 0.016800, 0.016800, 0.016800, 0.016800,
                   0.016800, 0.016800, 0.016800, 0.016800, 0.016800,
                   0.016800, 0.016800, 0.016800
                  };
float preview_action = 0, preview_action_x = 0;
float a[5] = {0,0,0,0,0};
float ax[5] = {0,0,0,0,0};
float az[5] = {0,0,0,0,0};
float filtered_a, filtered_ax, filtered_az;



// variaveis do sensor
double accely1, accely2, accelx1, accelx2, accelz1, accelz2;

void setup() {
  Serial.begin(115200);

  myservo1.attach(2);
  myservo2.attach(3);
  myservo3.attach(4);
  myservo4.attach(5);
  myservo5.attach(6);
  myservo6.attach(7);
  myservo7.attach(8);
  myservo8.attach(9);

  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  mySensor.beginAccel();
  mySensor.beginGyro();

  mySensor2.beginAccel();
  mySensor2.beginGyro();


}

void getAngle() {

  ang_quad_dir_y = atan(0.04 / (abs(posCM[1] - posPD[1])));
  retaH_dir = posCM[2] + dQ * sin(ang_quad_dir_y) - 0.075 - posPD[2];
  dif_reta_dir = (posCM[0] - posPD[0]);
  retaH_dir2 = pow(retaH_dir, 2) + pow(dif_reta_dir, 2);
  ang_joel_dir = acos((retaH_dir2 / (-2 * leg2)) + 1);
  ang_torn_dir = 90 * toRad - ang_joel_dir / 2;
  ang_quad_dir_x = 90 * toRad - ang_joel_dir / 2 - asin(dif_reta_dir / retaH_dir);

  ang_quad_esq_y = atan(0.04 / (abs(posCM[1] - posPE[1])));
  retaH_esq = posCM[2] + dQ * sin(ang_quad_esq_y) - 0.075 - posPE[2];
  dif_reta_esq = (posCM[0] - posPE[0]);
  retaH_esq2 = pow(retaH_esq, 2) + pow(dif_reta_esq, 2);
  ang_joel_esq = acos(((retaH_esq2) / (-2 * leg2)) + 1);
  ang_torn_esq = 90 * toRad - ang_joel_esq / 2;
  ang_quad_esq_x = 90 * toRad - ang_joel_esq / 2 - asin(dif_reta_esq / retaH_esq);
  //Serial.println(String("Ang_quad_y_(dir-esq): ") + String(ang_quad_dir_y) + String(" - ") + String(ang_quad_esq_y));

  //Serial.println(String("PosCM - PosPD - PosPE:")+ String(posCM[0])+ String(" - ") + String(posPD[0])  + String(" - ") + String(posPE[0]));
  //Serial.println("RetaH_(dir - esq):"+ String(retaH_dir) + String(retaH_esq));
  //Serial.println(String("Asin dir - esq: %.5f - %.5f") + String(asin(dif_reta_dir/retaH_dir))+ String(asin(dif_reta_esq/retaH_esq)));


}

void setAngles() {
  myservo1.write(90 + (ang_torn_dir)*toAng);
  myservo2.write(90 - (ang_torn_esq)*toAng);
  myservo3.write(150 - (180 * toRad - ang_joel_dir)*toAng);
  myservo4.write(30 + (180 * toRad - ang_joel_esq)*toAng);
  myservo5.write(90 + ang_quad_dir_x * toAng);
  myservo6.write(90 - ang_quad_esq_x * toAng);
  myservo7.write(90 - (ang_quad_dir_y - offset)*toAng);
  myservo8.write(90 + (ang_quad_esq_y - offset)*toAng);
  //Serial.println(String("Ang_quad_y_(dir-esq): ") + String(90 + (ang_quad_dir_y-offset)*toAng) + String(" - ") + String(90 - (ang_quad_esq_y-offset)*toAng));

}

void readSensor() {
  accely1 = 0;
  accelx1 = 0;
  accelz1 = 0;
  accely2 = 0;
  accelx2 = 0;
  accelz2 = 0;
  for (int i = 0; i < 10; i++) {
    if (mySensor.accelUpdate() == 0) {
      accely1 += mySensor.accelY();
      accelx1 += mySensor.accelX();
      accelz1 += mySensor.accelZ();
    }

    if (mySensor2.accelUpdate() == 0) {
      accely2 += mySensor2.accelY();
      accelx2 += mySensor2.accelX();
      accelz2 += mySensor2.accelZ();
    }
  }
  accely1 = accely1 * 9.8/10;
  accelx1 = accelx1 * 9.8/10;
  accelz1 = accelz1 * 9.8/10;
  accely2 = accely2 * 9.8/10;
  accelx2 = accelx2 * 9.8/10;
  accelz2 = accelz2 * 9.8/10;
}

float multiVxV(float* v1, float* v2) {
  if (sizeof(v1) != sizeof(v2)) {
    Serial.println("Erro multiVxV");
    return -99;
  }
  float result;
  result = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  return result;
}

void applyControl() {
  preview_action = 0;
  preview_action_x = 0;
  for(int i = 0; i < Ny; i++){
    preview_action += Gd*pref[(i+k)%63];
  }
  if(k == (62-Nx)){
    for(int i =0; i < Nx; i++){
      prefx[i] = prefx[i] + prefx[62];
    }
  }
  for(int i = 0; i < Nx; i++){
    preview_action_x += Gdx*prefx[(i+k)%63];
  }
  if(k == (62)){
    for(int i = 0; i < Nx; i++){
      prefx[i] = prefx[i] - prefx[62];
    }
  }
  if(k < 5){
    a[k] = (accely1 + accely2)/2;
    ax[k] = (accelx1 + accelx2)/2;
    az[k] = (accelz1 + accelz2)/2;
  } else {
    for(int i = 0; i < 4; i++){
       a[i] = a[i+1];
       ax[i] = ax[i+1];
       az[i] = az[i+1];
    }
    a[4] = (accely1 + accely2)/2;
    ax[4] = (accelx1 + accelx2)/2;
    az[4] = (accelz2 + accelz2)/2;
  }

  filtered_a = 0;
  filtered_ax = 0;
  filtered_az = 0;
  for(int i = 0; i < 5; i++){
    filtered_a += a[i];
    filtered_ax += ax[i];
    filtered_az += az[i];
  }
  filtered_a = filtered_a/5;
  filtered_ax = filtered_ax/5;
  filtered_az = filtered_az/5;

  sideAngle = atan(filtered_a/filtered_az);
  frontAngle = atan(filtered_ax/filtered_az);

  filtered_a = filtered_a*cos(abs(sideAngle));
  filtered_ax = filtered_ax*cos(abs(frontAngle));

  calc_zmp = state_vector[0] - (r * filtered_a) / g;
  calc_zmp_x = state_vector_x[0] - (r * filtered_ax) / g;

  //soma com vetorC*vetorU
  aux = multiVxV(vector_C , state_vector);
  auxx = multiVxV(vector_C, state_vector_x);
  aux = calc_zmp - aux;
  auxx = calc_zmp_x - auxx;

  //Multiplica por L
  result_L[0] = vector_L[0] * aux;
  result_L[1] = vector_L[1] * aux;
  result_L[2] = vector_L[2] * aux;
  result_L_x[0] = vector_L[0] * auxx;
  result_L_x[1] = vector_L[1] * auxx;
  result_L_x[2] = vector_L[2] * auxx;

  // soma result_L com state_vector
  result_L[0] = -result_L[0] + state_vector[0];
  result_L[1] = -result_L[1] + state_vector[1];
  result_L[2] = -result_L[2] + state_vector[2];
  result_L_x[0] = -result_L_x[0] + state_vector_x[0];
  result_L_x[1] = -result_L_x[1] + state_vector_x[1];
  result_L_x[2] = -result_L_x[2] + state_vector_x[2];

  // Multiplica result_L por vector_C
  aux2 = multiVxV(vector_C, result_L);
  aux2x = multiVxV(vector_C, result_L_x);

  // soma pref com aux2
  aux3 = aux2 - pref[k];
  aux3x = aux2x - prefx[k];

  // somatório
  somatorio += aux3;
  somatorio_x += aux3x;

  // multiplica por Gi
  aux4 = Gi * somatorio;
  aux4x = Gix * somatorio_x;

  // multiplica result_L por Gx
  aux5 = multiVxV(Gx, result_L);
  aux5x = multiVxV(Gxx, result_L_x);


  //soma de aux7
  aux6 = -preview_action - aux4 - aux5;
  aux6x = -preview_action_x - aux4x - aux5x;

  // multiplica aux6 por b
  result_B[0] = vector_B[0] * aux6;
  result_B[1] = vector_B[1] * aux6;
  result_B[2] = vector_B[2] * aux6;
  result_B_x[0] = vector_B[0] * aux6x;
  result_B_x[1] = vector_B[1] * aux6x;
  result_B_x[2] = vector_B[2] * aux6x;

  //multiplica result_B pela matrix_A
  state_vector[0] = multiVxV(matrix_A[0], result_B);
  state_vector[1] = multiVxV(matrix_A[1], result_B);
  state_vector[2] = multiVxV(matrix_A[2], result_B);
  state_vector_x[0] = multiVxV(matrix_A[0], result_B_x);
  state_vector_x[1] = multiVxV(matrix_A[1], result_B_x);
  state_vector_x[2] = multiVxV(matrix_A[2], result_B_x);

  //atualiza o valor do posCM[1]
  posCM[1] = state_vector[0];
  posCM[0] = state_vector_x[0];

  //Serial.print("PosCm: ");
  //Serial.print(posCM[1], 5);
  //Serial.print(" K: ");
  //Serial.print(k);
  //Serial.print("\n");

}

void applyWalkPattern() {
  if (agachar == 1) {
    posCM[2] -= 0.025 / 63;
  }

  if (walk == 1) {
    // definir posições dos pés para o movimento
    if (k > 43 && k < 50) {
      posPD[2] = altura * (-pow((count / 3), 2) + 1);
      count++;
      posPD[0] += distPDE / (6);

    } else {
      posPD[2] = 0;
    }

    if (k > 17 && k < 43) {
      count = -3;
    }


    if (k > 10 && k < 17) {
      posPE[2] = altura * (-pow((count / 3), 2) + 1);
      count++;
      posPE[0] += distPDE / (6);
    } else {
      posPE[2] = 0;
    }

    if (k > 50) {
      count = -3;
    }
  }
}

void loop() {
  digitalWrite(12, HIGH);
  if (k == 63) {
    if (control == 0) {
      Serial.println("Inicia controle");
    }
    k = 0;

    walk = 1;
    control = 1;
    agachar = 0;
    somatorio_x=0;
    posPD[0] = 0;
    posPE[0] = 0;

  }
  
  readSensor();
  if (control == 1) {
    applyControl();
  }
  applyWalkPattern();
  getAngle();
  setAngles();
  Serial.print("PosCM: ");
  Serial.print(posCM[1], 4);
  Serial.print(",");
  Serial.print("Pref:");
  Serial.print(pref[k], 4);
  Serial.print(",");
  Serial.print("Zmp_calc: ");
  Serial.print(calc_zmp, 4);
  Serial.print(",");
  Serial.print("PosCMx: ");
  Serial.print(posCM[0], 4);
  Serial.print(",");
  Serial.print("Pref:");
  Serial.print(prefx[k], 4);
  Serial.print(",");
  Serial.print("Zmp_calc: ");
  Serial.print(calc_zmp_x, 4);
  Serial.print(",");
  Serial.print("Tempo:");
  Serial.print(millis());
  Serial.print("\n");
  k++;
  delay(1);
}
