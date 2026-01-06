#include <Balboa32U4.h>
#include <LSM6.h>

const int16_t  MOTOR_MAX_SPEED = 400;

// PID-verdier 
float Kp = 25.0;
float Ki = 120;
float Kd = 0.7;

Balboa32U4Motors motors; // opprett motorobjekt/variabel
LSM6 imu;//opprett imu objekt/variabel
Balboa32U4Encoders enc; //encoders trenges for å måle avstand kjørt

static int ticksPerMeter=7800;//omtrentlig verdi etter målinger. tilsvarer ca 

static float graderperM=30.0;
static float meterTF=0.1;

float degree_offset_kjort=0;//verdi for å gi roboten en liten tilt i retning tilbake til startposisjon;

static float degrees_offset = 10; //så mange grader vi må justere i forhold til rett opp - for å finne balansepunktet. 
float angle = 0;      // grader (for bruk med integrator gyroskop)
uint32_t lastTime=0;
int lastSec=0; //siste sekund, brukes kun for å gjøre utskrift til serial 1 gang per sekund. 
int32_t gyroBias   = 0;  //gjennomsnittlig gyro bias, settes i setup
int32_t avstandStart; //for bruk for å prøve å få roboten til å stå stille/bevge seg mot startpunkt/delvis for å kalibrere for at å finne 90 grader på akselerometeret ikke nødvendigvis er balansepunktet. 
int motorSpeed;
float previousAngle = 0;    // for derivat
float integral = 0;         // for integral
float derivative=0;
double L=0; // for antall rotasjoner hjul (venstre hjul men har ingen betydning)

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  if (!imu.init()) {
    Serial.println("IMU init failed");
    while(1);
  }
  imu.enableDefault();

  Serial.println("Kjører... Viser vinkel i grader.");
  lastTime=micros();

  const int samples = 1000;  //antall tester for å finne gjennomsnittlig gyro bias
  int32_t sum = 0;
  for (int i = 0; i < samples; i++)
  {
    imu.read();
    sum += imu.g.y;
    delay(2);
  }
  gyroBias = sum / samples; //lagrer gyro bias
  motors.allowTurbo(true);
  enc.init();
  motors.setSpeeds(0,0);
}

void loop() {
  imu.read(); //les akselrometer og gyroskop data og lagre i variabel imu. 
  uint32_t now   = micros();
  float dt = (now - lastTime)/1e6; //gir dt i sekunder 
  
  lastTime=now;
L +=enc.getCountsAndResetLeft();

degree_offset_kjort=((L/ticksPerMeter)*graderperM);
degree_offset_kjort = constrain(degree_offset_kjort, -(graderperM*meterTF), graderperM*meterTF);//begrenser ofsett til maks å bry seg om 1 meter.


  // Akselerometer-verdier
  float accX = imu.a.x;
  float accZ = imu.a.z;

float gyro_dps = (imu.g.y - gyroBias) * 0.004375f*2.04;

  // vinkel i grader (ca. 0 når oppreiset, +-90 når horisontal) 
float accAngle = ((atan2(accX, accZ) * 180.0 / PI)-90);

  float tau = 0.5;        // 0.5 sek (hvor lang tid vi skal bruke på å gå over til å stole på akselerometeret, 0.5 sek gir ca alpha lik 0.99 ved 5 ms loop, og alpha lik 0.98 ved 10 ms loop)
  float alpha = tau / (tau + dt);
  //alpha =1; //midlertidig for å teste at gyro gir noenlunde rikitge resultater 
angle = (alpha*(angle + gyro_dps * dt)+(1-alpha)*-(accAngle + degrees_offset-degree_offset_kjort));

integral += (angle * dt);
integral = constrain((integral), -(MOTOR_MAX_SPEED/Ki), MOTOR_MAX_SPEED/Ki);

derivative= (angle-previousAngle) / dt;
previousAngle = angle;
float pidOut = (Kp * angle + Ki * integral + Kd * derivative);
motorSpeed = constrain((int)(pidOut), -(MOTOR_MAX_SPEED), MOTOR_MAX_SPEED);

if (angle > 35 || angle < -35 ){
  motorSpeed = 0;
  L=0;
  integral=0;
  delay(500);
  
}

motors.setSpeeds(motorSpeed, motorSpeed);

  // Skriv ut vinkelen
  int nowSec=now/300000;//i teorien så kan  /1e6 erstattes med ett annet tall for å få en annen utskriftsfrekvens.  /1000000 (1e6) gir en utskrift per sekund, /100000 (1e5) gir 10 utskrifter per sekund
  if (nowSec!=lastSec){
    lastSec=nowSec;

Serial.println(
  "gyroBias: " + String(gyroBias) + "  " +
  "Kp: " + String(Kp) + "  " +
  "derivative: " + String(derivative) + "  " +
  "motorSpeed: " + String(motorSpeed) + "  " +
  "pidOut: " + String(pidOut) + "  " +
  "angle: " + String(angle) + "  " +
  "loopFreq: " + String(1.0/dt) + "  " + 
  "DoK: " + String(degree_offset_kjort) + "  " + 
    "L: " + String(L) + "  " + 
  "Integral " + String(integral)
);
  }
}