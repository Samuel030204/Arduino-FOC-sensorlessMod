/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 *
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include <SimpleFOC.h>

#define pin_sig 44

float pole_pairs = 14;
float phase_resistance = 6.883;
float phase_inductance = 2.033 * 1e-3;
float KV = 43.5;  // openloop experiment: 43.5

int HPF_ENB = 1;
float FLO_angle_offset = _3PI_2;
SensorlessFluxLinkageObserver sensor = SensorlessFluxLinkageObserver(FLO_angle_offset, HPF_ENB);
float SL_velocity = 15;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(pole_pairs, phase_resistance, KV, phase_inductance);  // pp, R, KV, L
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8); // A, B, C, en1, en2, en3
float motor_voltage_use = 4;

// inline current sensor instance
// check if your board has R010 (0.01 ohm resistor) or R006 (0.006 mOhm resistor)
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, A2, A0);

// velocity set point variable
float target_velocity = 0;
float target_velocity_str = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity_str, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

float var_aux1 = 1;
float var_aux2 = 0;
float var_aux3 = 0;
float var_aux4 = 0;
void aux1(char* cmd) { command.scalar(&var_aux1, cmd); }
void aux2(char* cmd) { command.scalar(&var_aux2, cmd); }
void aux3(char* cmd) { command.scalar(&var_aux3, cmd); }
void aux4(char* cmd) { command.scalar(&var_aux4, cmd); }

void setup() {
  // communication with the other Arduino board
  pinMode(pin_sig, INPUT);

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // -------------------- FLO management needed -------------------- //
  sensor.linkFOCMotor(&motor);
  sensor.linkCurrentSense(&current_sense);
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 11.1;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 11.1;
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  current_sense.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
  // set torque mode:
  // TorqueControlType::dc_current
  // TorqueControlType::voltage
  // TorqueControlType::foc_current
  // motor.torque_controller = TorqueControlType::voltage;
  motor.torque_controller = TorqueControlType::foc_current;
  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  motor.voltage_limit = 0;

  // foc currnet control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P = 5;
  motor.PID_current_q.I = 300;
  motor.PID_current_q.limit = motor_voltage_use;  // limits voltage, not the current
  // motor.PID_current_q.output_ramp = 1000;
  motor.LPF_current_q.Tf = 0.05f;

  motor.PID_current_d.P= 5;
  motor.PID_current_d.I = 300;
  motor.PID_current_d.limit = motor_voltage_use;  // limits voltage, not the current
  // motor.PID_current_d.output_ramp = 1000;
  motor.LPF_current_d.Tf = 0.05f;

  // contoller configuration
  // default parameters in defaults.h
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  // motor.PID_velocity.output_ramp = 1000;

  // downsampling value
  motor.motion_downsample = 0; // - times (default 0 - disabled)
  // default voltage_power_supply

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle loop velocity limit
  motor.velocity_limit = 20;

  // motor default direction
  motor.sensor_direction = Direction::CW;

  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();

  // current sense init and linking
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // -------------------- FLO management needed -------------------- //
  // invert phase a gain
  current_sense.gain_a *=-1;

  // skip alignment
  current_sense.skip_align = true;

  // align sensor and start FOC
  motor.initFOC();

  // start with open-loop control
  motor.controller = MotionControlType::velocity_openloop;

  // add target command T and M
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  command.add('R', aux1, "aux1");
  command.add('W', aux2, "aux2");
  command.add('M', aux3, "aux3");
  command.add('F', aux4, "aux4");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity (rad/s) using serial terminal:"));
  _delay(1000);
}


float lps = 0;
unsigned long comm_now = 0;
unsigned long comm_timestamp = 0;
float comm_period = 1;
void loop() {
  comm_now = _micros();
  float dT_comm = (float)(comm_now - comm_timestamp) * 1e-6;
  if ( ( var_aux1 != 0 ) && ( dT_comm >= comm_period ) ) {
    if ( digitalRead(pin_sig) ){
      target_velocity_str = SL_velocity;
    }
    else {
      target_velocity_str = 0;
    }
    comm_timestamp = comm_now;
  }

  // serial communication for displaying parameters
  if ( var_aux2 != 0 ) { 
    lps = lpsUpdate();
    serialPrintAux(); 
  }

  // automatic control
  if ( var_aux3 == 0 ) { SLcontrol(); }
  // manual control enforcement
  else {
    target_velocity = target_velocity_str;
    if ( var_aux4 == 0 ) { motor.controller = MotionControlType::velocity_openloop; }
    else { motor.controller = MotionControlType::velocity; };
  }

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}

int flag_stdy = 0;
int flag_FOCokay = 1;
void SLcontrol() {
  flag_stdy = stdyChecker();
  if ( flag_stdy == 0 ) {
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = motor_voltage_use;
    targVelRamp();
  }
  else {
    targVelRampUpd();
    if ( target_velocity == 0 ) { motor.voltage_limit = 0; }
    else { 
      if ( FOCval() == 0 ) { flag_FOCokay = 0; } 
      else;
      if ( flag_FOCokay ) { motor.controller = MotionControlType::velocity; }
      else { motor.controller = MotionControlType::velocity_openloop; }
    }
  }
}

float T_settle = 2;
int chkr_timer = 1;
int stdyChecker() {
  if ( ( target_velocity == target_velocity_str ) && ( chkr_timer == 0 ) ) {
    custom_timer_1(T_settle);
    chkr_timer = 1;
  }
  else if ( ( target_velocity == target_velocity_str ) && ( chkr_timer == 1 ) ) {
    if ( custom_timer_1(0) ) { return 1; }
    else { return 0; }
  }
  else {
    chkr_timer = 0;
    return 0;
  }
}

unsigned long ct1_prev = 0;
unsigned long ct1_now = 0;
float ct1_Tset = 0;
int custom_timer_1(float Tset) {
  if ( Tset > 0 ) {
    ct1_Tset = Tset;
    ct1_prev = micros();
    return 1;
  }
  else {
    ct1_now = micros();
    float Tpast = (float)( ct1_now - ct1_prev ) * 1e-6;
    if ( ( Tpast >= ct1_Tset ) || ( Tpast < 0 ) ) {
      return 1;
    }
    else {
      return 0;
    }
  }
}

float target_velocity_ramp = 5; // rad/s^2
unsigned long tvr_now, tvr_timestamp;
void targVelRamp() {
  tvr_now = _micros();
  float dT = (float)(tvr_now - tvr_timestamp) * 1e-6;
  tvr_timestamp = tvr_now;
  float diff = target_velocity_str - target_velocity;
  target_velocity = target_velocity + _sign(diff)*target_velocity_ramp * dT;
  float diff2 = target_velocity_str - target_velocity;
  if ( _sign(diff) != _sign(diff2) ) { target_velocity = target_velocity_str; }
}
void targVelRampUpd() {
  tvr_timestamp = _micros();
}

float vel_diff_crit = 5;
int FOCval() {
  float vel_diff = fabs( target_velocity - motor.shaft_velocity_rec );
  if ( ( vel_diff > vel_diff_crit ) || ( _sign(target_velocity) != _sign(motor.shaft_velocity_rec) ) ) {
    return 0;
  }
  else { return 1; }
}


unsigned long loop_now, loop_timestamp; 
float lpsUpdate() {
  loop_now = _micros();
  float lps_loc = 1 / ( (float)(loop_now - loop_timestamp) * 1e-6 );
  loop_timestamp = loop_now;
  return lps_loc;
}

unsigned long serial_now, serial_timestamp; 
float serial_period = 0.01;
void serialPrintAux() {
  serial_now = _micros();
  if ( (float)(serial_now - serial_timestamp) * 1e-6 >= serial_period ) {
    serial_timestamp = serial_now;
    
    // Serial.print(lps);
    // Serial.print("\t");

    Serial.print(flag_stdy);
    // Serial.print("\t");

    Serial.print("\t");
    Serial.print(target_velocity_str);
    Serial.print("\t");
    Serial.print(target_velocity);

    // Serial.print(sensor.FLO_angle);
    // Serial.print("\t");
    // Serial.print(sensor.FLO_angle_el);
    // Serial.print("\t");
    // Serial.print(sensor.flux_a*1000);
    // Serial.print("\t");
    // Serial.print(sensor.flux_b*1000);

    // Serial.print("\t");
    // Serial.print(motor.electrical_angle);
    // Serial.print("\t");
    // Serial.print(motor.shaft_angle_rec);
    // Serial.print("\t");
    // Serial.print(motor.shaft_velocity_rec);

    // Serial.print("\t");
    // Serial.print(motor.current_sp);

    Serial.print("\t");
    Serial.print(motor.current.q*1000);
    Serial.print("\t");
    Serial.print(motor.current.d*1000);
    Serial.print("\t");
    Serial.print(motor.voltage.q*1000);
    Serial.print("\t");
    Serial.print(motor.voltage.d*1000);

    // Serial.print("\t");
    // Serial.print(sensor.ABCurrent.alpha*1000);
    // Serial.print("\t");
    // Serial.print(sensor.ABCurrent.beta*1000);
    // Serial.print("\t");
    // Serial.print(sensor.ABVoltage.alpha*1000);
    // Serial.print("\t");
    // Serial.print(sensor.ABVoltage.beta*1000);
    
    /*
    Serial.print("\t");
    Serial.print();
    */
    Serial.println();
  }
}
