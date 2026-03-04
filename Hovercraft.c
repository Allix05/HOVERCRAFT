// TEAM 11 FINAL HOVERCRAFT CODE ENGR 290

#include <Wire.h>
#include <Servo.h>

// The hovercraft successfully completed the maze ten times in a row during testing with an A+ final grade. 

/* ---------------- PINOUT ---------------- */
const uint8_t LIFT_PWM_PIN      = 6;
const uint8_t THRUST_PWM_PIN    = 5;
const uint8_t THRUST_SERVO_PIN  = 9;
const uint8_t US_TRIG_PIN       = 13;
const uint8_t US_ECHO_PIN       = 3;
const uint8_t IR_PIN            = A0;
const uint8_t LED_PIN           = 13;



/* ---------------- MPU6050 ---------------- */
const uint8_t MPU_ADDR = 0x68;
const uint8_t MPU_PWR_MGMT = 0x6B;
const uint8_t MPU_ACCEL_XOUT_H = 0x3B;
const float ACCEL_SENS = 16384.0f;
const float GYRO_SENS  = 131.0f;
const float RAD2DEG    = 57.2958f;



/* ---------------- PARAMETERS ---------------- */
const int SERVO_CENTER = 80;
const int SERVO_LEFT_DETECTION   = SERVO_CENTER - 70;
const int SERVO_RIGHT_DETECTION  = SERVO_CENTER + 70;
const int SERVO_CORR_LEFT   = SERVO_CENTER - 60;
const int SERVO_CORR_RIGHT  = SERVO_CENTER + 60;
const int SERVO_TURN_LEFT  = SERVO_CENTER - 35;  
const int SERVO_TURN_RIGHT = SERVO_CENTER + 35;




const float angle_rotation_max = 160.0f;
const int SCAN_STEPS[] = { -45, -30, -15, 0, 15, 30, 45 };
const uint8_t SCAN_N = sizeof(SCAN_STEPS)/sizeof(SCAN_STEPS[0]);
const unsigned long SCAN_SETTLE_MS = 140;
unsigned long us_cooldown_until = 0;
float headingTarget = 0;



// Thresholds 
const uint16_t WALL_DISTANCE_CM = 65;  
const int IR_BAR_THRESHOLD = 600;       
const uint16_t US_READ_SAMPLES = 5;


// Power
uint8_t LIFT_PWM_START    = 255;
uint8_t THRUST_PWM_CRUISE = 230;  
uint8_t THRUST_PWM_SLOW   = 120;
const uint8_t THRUST_PWM_STOP = 0;



// PID Yaw tuning
float KP_YAW = 0.45f;
float KI_YAW = 0.00f;
float KD_YAW = 0.06f;
const float YAW_MAX_CORR = 22.0f;  
float yaw_integral = 0, yaw_last_err = 0;


/* ---------------- GLOBALS ---------------- */
Servo thrustServo;
int16_t ax_raw=0, ay_raw=0, az_raw=0, gx_raw=0, gy_raw=0, gz_raw=0;
float roll=0, pitch=0, yawAngle=0;
unsigned long imu_last_us=0, lastLoopMs=0;
const unsigned long LOOP_DT_MS = 25;
unsigned long turnCooldownUntil = 0;
int rotationDirection = SERVO_CENTER;

bool headingInitialized = false;    
float initialHeading = 0.0f;        

bool flipped = false;







enum State { S_LIFT, S_NAVIGATE, S_SCAN, S_TURN_TO_DIR, S_TURN_180, S_STOP };
float turnStartYaw = 0.0f;
int rotation_state = 0;  
State state = S_LIFT;
unsigned long stateEnterMs = 0;
int chosen_scan_angle = 0;








/* ---------------- DECLARATIONS ---------------- */
void mpu_init();
void mpu_read_raw();
void imu_update(float dt);
uint16_t us_read_cm_median(uint8_t samples);
void setLiftPWM(uint8_t);
void setThrustPWM(uint8_t);
int scan_best_sector();
float pid_yaw_correction(float target, float dt);
void enterState(State);
float angleDiff(float a, float b);






/* ---------------- SETUP ---------------- */
void setup() {
  Serial.begin(9600);
  Wire.begin();


  pinMode(LIFT_PWM_PIN, OUTPUT);
  pinMode(THRUST_PWM_PIN, OUTPUT);
  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);


  analogWrite(LIFT_PWM_PIN, 0);
  analogWrite(THRUST_PWM_PIN, 0);


  digitalWrite(US_TRIG_PIN, LOW);


  thrustServo.attach(THRUST_SERVO_PIN);
  thrustServo.write(SERVO_CENTER);
  delay(150);


  mpu_init();
  imu_last_us = micros();
  lastLoopMs = millis();



  enterState(S_LIFT);
  Serial.println("=== Hovercraft Initialized (fixed) ===");
}








/* ---------------- LOOP ---------------- */
void loop() {
  unsigned long now = millis();
  if (now - lastLoopMs < LOOP_DT_MS) return;
  float dt = (now - lastLoopMs) / 1000.0f;
  lastLoopMs = now;


  unsigned long now_us = micros();
  mpu_read_raw();
  imu_update((now_us - imu_last_us) / 1e6f);
  imu_last_us = now_us;


  int irCenter = analogRead(IR_PIN);
  uint16_t usDist = us_read_cm_median(3);
 



  switch(state) {
    case S_LIFT:
      setLiftPWM(LIFT_PWM_START);
      setThrustPWM(0);
      digitalWrite(LED_PIN, HIGH);
      if (millis() - stateEnterMs > 800) enterState(S_NAVIGATE);
      break;



    case S_NAVIGATE:
      setLiftPWM(LIFT_PWM_START);
      setThrustPWM(THRUST_PWM_CRUISE);



      // TURN COOLDOWN: ignore sensors 
      if (millis() < turnCooldownUntil) {
        setThrustPWM(THRUST_PWM_CRUISE);
        float corr = pid_yaw_correction(headingTarget, dt) * 2.0;
        int servoAngle = constrain(
    (int)(SERVO_CENTER - corr),
    SERVO_CORR_LEFT,
    SERVO_CORR_RIGHT
);




        thrustServo.write(servoAngle);
        break; 
      }

      if (irCenter >= 300 && irCenter <= 600) {
            Serial.println("Bar detected → STOP");
            enterState(S_STOP);
            break;
          }


      if (rotation_state == 0 && usDist > 0 && usDist <= WALL_DISTANCE_CM) {
        Serial.println("Wall detected -> TURN 180");
        enterState(S_TURN_180);
        break;
      }


   

      {
        if (rotation_state == 0) {
         
          float corr = pid_yaw_correction(headingTarget, dt);
          corr *= 2.3;
          int servoAngle = constrain(
    (int)(SERVO_CENTER - corr),
    SERVO_CORR_LEFT,
    SERVO_CORR_RIGHT
);




          thrustServo.write(servoAngle);
        } else {



// ---- ROTATION MODE (180°) ----

float d = fabs(angleDiff(yawAngle, turnStartYaw));
if (d >= angle_rotation_max) {
  Serial.println("Rotation complete");

  rotation_state = 0;

  
  thrustServo.write(SERVO_CENTER);
  delay(80);

 
  // ---- SET NEW HEADING AFTER 180 ----

if (headingInitialized) {

    if (!flipped) {
        headingTarget = initialHeading + 180.0f;
        flipped = true;
    } else {
        headingTarget = initialHeading;
        flipped = false;
    }

    // correction to compensate drift
    headingTarget += 8.0f;     // <<< to be tuned

    
    if (headingTarget > 180) headingTarget -= 360;
    if (headingTarget < -180) headingTarget += 360;

    Serial.print("NEW corrected headingTarget = ");
    Serial.println(headingTarget);

} else {
    headingTarget = yawAngle;
}




  // Reset PID
  yaw_last_err = 0;
  yaw_integral = 0;

  // counter-steer pulse
  int oppositeServo =
    (rotationDirection == SERVO_TURN_LEFT)  ? SERVO_TURN_RIGHT :
    (rotationDirection == SERVO_TURN_RIGHT) ? SERVO_TURN_LEFT  :
                                             SERVO_CENTER;

  setThrustPWM(THRUST_PWM_SLOW);
  thrustServo.write(oppositeServo);
  delay(120);

  thrustServo.write(SERVO_CENTER);
  setThrustPWM(THRUST_PWM_CRUISE);

  turnCooldownUntil = millis() + 2500;
}


        }
        break;
      }

    case S_TURN_180: {
      rotation_state = 1;
      setLiftPWM(0);
      delay(200);
      Serial.println("Choosing turn direction with IR…");

    // if(THRUST_PWM_CRUISE!=230){
    //   THRUST_PWM_CRUISE=230;
    // }



      setThrustPWM(0);
      thrustServo.write(SERVO_CENTER);
      delay(200);

      uint16_t usLeft, usRight;








      // --------- PEEK LEFT ----------
      Serial.println("Peeking LEFT");
      thrustServo.write(SERVO_LEFT_DETECTION);
      delay(100);
      setThrustPWM(THRUST_PWM_SLOW);
      delay(300);
      setThrustPWM(0);
      delay(80);
      usLeft = us_read_cm_median(3);
      Serial.print("US LEFT = ");
      Serial.println(usLeft);








      // --------- PEEK RIGHT ----------
      Serial.println("Peeking RIGHT");
      thrustServo.write(SERVO_RIGHT_DETECTION);
      delay(100);
      setThrustPWM(THRUST_PWM_SLOW);
      delay(600);
      setThrustPWM(0);
      delay(80);
      usRight = us_read_cm_median(3);
      Serial.print("US RIGHT = ");
      Serial.println(usRight);



      // --------- CHOOSE SIDE ----------
     
 
      if (usLeft > usRight) {
        Serial.println("Turning toward LEFT side (more space)");
        rotationDirection = SERVO_TURN_LEFT;
      } else {
        Serial.println("Turning toward RIGHT side (more space)");
        rotationDirection = SERVO_TURN_RIGHT;
      }



      thrustServo.write(rotationDirection);
      delay(120);
      turnStartYaw = yawAngle;


      delay(150);
      setLiftPWM(LIFT_PWM_START);
      enterState(S_NAVIGATE);

//       headingTarget = turnStartYaw + 180;
// if (headingTarget > 180) headingTarget -= 360;
// if (headingTarget < -180) headingTarget += 360;

      break;
    }


    case S_STOP:
      setThrustPWM(0);
      setLiftPWM(0);
      thrustServo.write(SERVO_CENTER);
      digitalWrite(LED_PIN, LOW);
      break;
  }
}


/* ---------------- FUNCTIONS ---------------- */


void enterState(State s) {
  state = s;
  stateEnterMs = millis();


if (s == S_NAVIGATE && rotation_state == 0 && !headingInitialized) {
    initialHeading = yawAngle;
    headingTarget = initialHeading;
    headingInitialized = true;
    yaw_last_err = 0;
    yaw_integral = 0;
    Serial.print("Initial heading saved: ");
    Serial.println(initialHeading);
 
  }

}



void mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT);
  Wire.write(0);
  Wire.endTransmission();
  delay(50);
}


void mpu_read_raw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() >= 14) {
    ax_raw = (Wire.read() << 8) | Wire.read();
    ay_raw = (Wire.read() << 8) | Wire.read();
    az_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    gx_raw = (Wire.read() << 8) | Wire.read();
    gy_raw = (Wire.read() << 8) | Wire.read();
    gz_raw = (Wire.read() << 8) | Wire.read();
  }
}



void imu_update(float dt) {
  if (dt <= 0) return;

  float gx = gx_raw / GYRO_SENS;
  float gy = gy_raw / GYRO_SENS;
  float gz = gz_raw / GYRO_SENS;

  float accRoll  = atan2(ay_raw, az_raw) * RAD2DEG;
  float accPitch = atan2(-ax_raw, sqrt(ay_raw * ay_raw + az_raw * az_raw)) * RAD2DEG;

  roll  = 0.98f * (roll  + gx * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + gy * dt) + 0.02f * accPitch;




  yawAngle += gz * dt;
  if (yawAngle > 180) yawAngle -= 360;
  if (yawAngle < -180) yawAngle += 360;
}




uint16_t us_read_cm_median(uint8_t samples) {
  if (samples < 1) samples = 1;
  if (samples > 9) samples = 9;


  uint16_t arr[9];
  for (uint8_t i = 0; i < samples; i++) {
    digitalWrite(US_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);


    unsigned long pulse = pulseIn(US_ECHO_PIN, HIGH, 30000);
    arr[i] = (pulse == 0) ? 0 : (pulse / 58);
    delay(10);
  }


  for (uint8_t i = 0; i < samples - 1; i++)
    for (uint8_t j = i + 1; j < samples; j++)
      if (arr[j] < arr[i]) {
        uint16_t tmp = arr[i];
        arr[i] = arr[j];
        arr[j] = tmp;
      }

  return arr[samples / 2];
}


void setLiftPWM(uint8_t v) { analogWrite(LIFT_PWM_PIN, v); }
void setThrustPWM(uint8_t v) { analogWrite(THRUST_PWM_PIN, v); }

float pid_yaw_correction(float target, float dt) {
  float err = target - yawAngle;
  while (err > 180) err -= 360;
  while (err < -180) err += 360;


  yaw_integral += err * dt;
  float deriv = (dt > 0.0001f) ? (err - yaw_last_err) / dt : 0;
  yaw_last_err = err;


  float out = KP_YAW * err + KI_YAW * yaw_integral + KD_YAW * deriv;

  if (out > YAW_MAX_CORR) out = YAW_MAX_CORR;
  if (out < -YAW_MAX_CORR) out = -YAW_MAX_CORR;


  return out;
}



float angleDiff(float a, float b) {
  float d = a - b;
  while (d > 180)  d -= 360;
  while (d < -180) d += 360;
  return d;
}
