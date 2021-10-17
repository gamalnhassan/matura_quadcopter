#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <RunningMedian.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

Servo esc_fl; //front left
Servo esc_fr; //front right
Servo esc_rl; //rear left
Servo esc_rr; //rear right

const byte ESC_FL_PIN{5};
const byte ESC_FR_PIN{6};
const byte ESC_RL_PIN{9};
const byte ESC_RR_PIN{10};

const byte LED_PIN{13};

const double LOW_PASS{0.05};
const double HIGH_PASS{1 - LOW_PASS};
const double G_CONST{9.81};

double KP{2.5};
double KI{0.05};
double KD{0};

const size_t X {0}, Y {1};
double pid[2]{0};
const double I_PID_MAX{200};
const double D_PID_MAX{50};
const double PID_MAX{800};

RunningMedian accel_data[2]{RunningMedian(15), RunningMedian(15)};
double angle[2]{0};
double current_error[2]{0};
double previous_error[2]{0};
double desired_angle[2]{0};
double dt{0}; //seconds elapsed since last loop

double initial_throttle{1100};

boolean paused{true}; //false if manually unpaused through drone controller
boolean bt_connected{false}; //wheter or not the system is connected to the drone controller app
boolean emergency{false}; //set to true when a safety measurement can't be met

void evaluate_imu(){
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  
  double accel_roll{-atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG};
  accel_data[X].add(accel_roll);
  double filtered_accel_roll{accel_data[X].getMedian()};         //remove spikes with a median filter
  double gyro_roll{g.gyro.x * RAD_TO_DEG * dt};
  angle[X] = (HIGH_PASS * (angle[X] + gyro_roll)) + (LOW_PASS * filtered_accel_roll);
  current_error[X] = angle[X] - desired_angle[X];  //angle error at present time

  double accel_pitch{asin(max(min(a.acceleration.x / G_CONST, 1), -1)) * RAD_TO_DEG};
  accel_data[Y].add(accel_pitch);
  double filtered_accel_pitch{accel_data[Y].getMedian()};         //remove spikes with a median filter
  double gyro_pitch{g.gyro.y * RAD_TO_DEG * dt};
  angle[Y] = (HIGH_PASS * (angle[Y] + gyro_pitch)) + (LOW_PASS * filtered_accel_pitch);
  current_error[Y] = angle[Y] - desired_angle[Y];  //angle error at present time

  //turn emergency mode on if the drone exceeds specific angles for safety
  if (abs(current_error[X]) > 40 || abs(current_error[Y]) > 40){
    Serial.println("Turning on emergency mode. Drone exceeded safety angle");
    emergency = true;
  }
}

void compute_pid(size_t axis){
  double p_pid = KP * current_error[axis]; 
  static double i_pid = 0;
  i_pid = i_pid + (KI * current_error[axis]);
  double d_pid = KD * ((current_error[axis] - previous_error[axis]) / dt);
  d_pid = max(-D_PID_MAX, min(D_PID_MAX, d_pid));  //make sure d_pid doesn't exceed its limit
  pid[axis] = max(-PID_MAX, min(PID_MAX, p_pid + i_pid + d_pid));  //combine all controls into pid and make sure pid doesn't exceed its limit
/*  Serial.print(p_pid); Serial.print(", ");
  Serial.print(i_pid); Serial.print(", ");
  Serial.println(d_pid); */
}

void compute_pids(){
  compute_pid(X);
  compute_pid(Y);
/*  double speed_esc_fl = initial_throttle - pid[X] - pid[Y];
  double speed_esc_fr = initial_throttle + pid[X] - pid[Y];
  double speed_esc_rl = initial_throttle - pid[X] + pid[Y];
  double speed_esc_rr = initial_throttle + pid[X] + pid[Y];
  Serial.print(speed_esc_fl); Serial.print(", "); Serial.print(speed_esc_fr); Serial.print(", "); Serial.print(speed_esc_rl); Serial.print(", "); Serial.print(speed_esc_rr);
  Serial.print(", ");  Serial.print(pid[Y]); Serial.print(", ");  Serial.println(pid[X]); */
}

void compute_motors(){
  double speed_esc_fl = initial_throttle - pid[X] - pid[Y];
  double speed_esc_fr = initial_throttle + pid[X] - pid[Y];
  double speed_esc_rl = initial_throttle - pid[X] + pid[Y];
  double speed_esc_rr = initial_throttle + pid[X] + pid[Y];

  speed_esc_fl = min(1600, max(1000, speed_esc_fl));  //limit speed to a value from 1000 to 2000 because the esc only accepts values in μs between 1000-2000;
  speed_esc_fr = min(1600, max(1000, speed_esc_fr));
  speed_esc_rl = min(1600, max(1000, speed_esc_rl));
  speed_esc_rr = min(1600, max(1000, speed_esc_rr));
  /* 
  speed_esc_fl = map(speed_esc_fl, 1000, 2000, 1100, 1600);
  speed_esc_fr = map(speed_esc_fr, 1000, 2000, 1100, 1600);
  speed_esc_rl = map(speed_esc_rl, 1000, 2000, 1100, 1600);
  speed_esc_rr = map(speed_esc_rr, 1000, 2000, 1100, 1600); */

  //send speed in μs to the esc
  esc_fl.writeMicroseconds(speed_esc_fl);
  esc_fr.writeMicroseconds(speed_esc_fr);
  esc_rl.writeMicroseconds(speed_esc_rl);
  esc_rr.writeMicroseconds(speed_esc_rr);
}

void setup() {
  esc_fl.attach(ESC_FL_PIN);
  esc_fr.attach(ESC_FR_PIN);
  esc_rl.attach(ESC_RL_PIN);
  esc_rr.attach(ESC_RR_PIN);

  esc_fl.writeMicroseconds(1000);
  esc_fr.writeMicroseconds(1000);
  esc_rl.writeMicroseconds(1000);
  esc_rr.writeMicroseconds(1000);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  if (!lsm.begin()) {
    Serial.println("Couldn't find LSM9DS1! Please check wiring");
    while (1);
  }
  Serial.println("Found LSM9DS1...");

  while(!bt_connected){
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == '1'){
        bt_connected = true;
        Serial.println("Connected to Drone Controller...");
        Serial.println("Press Start on the Drone Controller to continue.");
        break;
      }
    }
  }

  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);

  pinMode(LED_PIN, OUTPUT);

}

void loop() {
  if (emergency){
    esc_fl.writeMicroseconds(1000);
    esc_fr.writeMicroseconds(1000);
    esc_rl.writeMicroseconds(1000);
    esc_rr.writeMicroseconds(1000);
    return;
  }

  static unsigned long last_bt_signal{0};  //the last time a '1' signal was received from the bluetooth device
  if (Serial.available() > 0) {
    char input = Serial.read();
    switch (input) {
      case '1':
        last_bt_signal = millis();
        break;
      case 's':
        Serial.println("Starting Drone...");
        paused = false;
        break;
      case 'p':
        Serial.println("Pausing Drone...");
        paused = true;
        break;
      case 'd':
        initial_throttle = max(initial_throttle - 20, 1200);
        Serial.println("Decreasing throttle...");
        Serial.print("Throttle: ");
        Serial.println(initial_throttle);
        break;
      case 'i':
        initial_throttle = min(initial_throttle + 20, 2000);
        Serial.println("Increasing throttle...");
        Serial.print("Throttle: ");
        Serial.println(initial_throttle);
        break;
      case '5':
        KP += 0.2;
        break;
      case '6':
        KP -= 0.2;
        if (KP < 0){
          KP = 0;
        }
        break;
      case '7':
        KI += 0.001;
        break;
      case '8':
        KI -= 0.001;
        if (KI < 0){
          KI = 0;
        }
        break;
    }
  }

  //if no signal was received from the Drone Controller for longer than 2 seconds turn light off and pause
  if (last_bt_signal < (millis() - 2000)) {
    digitalWrite(13, LOW);
    bt_connected = false;
  } else {
    digitalWrite(13, HIGH);
    bt_connected = true;
  }

  if (paused || !bt_connected) {
    esc_fl.writeMicroseconds(1000);
    esc_fr.writeMicroseconds(1000);
    esc_rl.writeMicroseconds(1000);
    esc_rr.writeMicroseconds(1000);
    return;
  }

  static unsigned long last_loop = millis();  //millis() of last loop
  unsigned long t = millis();  //current millis() of loop
  dt = (t - last_loop) / (double)1000;
  last_loop = t;

  evaluate_imu();
  compute_pids();
  compute_motors();
}