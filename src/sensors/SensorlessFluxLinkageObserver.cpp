#include "SensorlessFluxLinkageObserver.h"

SensorlessFluxLinkageObserver::SensorlessFluxLinkageObserver(float _FLO_angle_offset, int _HPF_ENB){
	FLO_angle_offset = _FLO_angle_offset;
	HPF_ENB = _HPF_ENB;
}

void SensorlessFluxLinkageObserver::linkFOCMotor(FOCMotor* _motor){
	motor = _motor;
}

void SensorlessFluxLinkageObserver::linkCurrentSense(CurrentSense* _current_sense){
	current_sense = _current_sense;
}


void SensorlessFluxLinkageObserver::init(){
  
  FLO_timestamp = _micros();
  flux_a = 0;
  flux_b = 0;
  FLO_angle_el_cntr = 0;

  this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float SensorlessFluxLinkageObserver::getSensorAngle(){
  
  currents = current_sense->getPhaseCurrents();
  ABCurrent = current_sense->getABCurrents(currents);
  
  ABVoltage.alpha = motor->Ualpha;
  ABVoltage.beta = motor->Ubeta;
  phase_resistance = motor->phase_resistance;
  phase_inductance = motor->phase_inductance;
  
  pole_pairs = motor->pole_pairs;
  targ_vel = motor->shaft_velocity_sp;

  // Flux linkage observer
  now = _micros();
  Ts = ( now - FLO_timestamp ) * 1e-6f; 
  FLO_timestamp = now;
  FLO_lps = 1 / Ts;
  
  a_intgrl = ABVoltage.alpha - phase_resistance * ABCurrent.alpha;
  b_intgrl = ABVoltage.beta - phase_resistance * ABCurrent.beta;
  
  a_intgrl = LPF_vri_alpha(a_intgrl);
  b_intgrl = LPF_vri_beta(b_intgrl);
  
  if ( HPF_ENB == 1 ) {
  a_intgrl = HPF_vri_alpha(a_intgrl);
  b_intgrl = HPF_vri_beta(b_intgrl);
  }
  
  flux_a = a_intgrl - phase_inductance * ABCurrent.alpha;
  flux_b = b_intgrl - phase_inductance * ABCurrent.beta;
  
  if ( HPF_ENB == 1 ) {
  flux_a = HPF_flux_alpha(flux_a);
  flux_b = HPF_flux_beta(flux_b);
  }
  
  // Calculate angle
  if ( (flux_b == 0) && (flux_a == 0) ) { flux_a = 1e-6; }
  FLO_angle_el = _normalizeAngle(_atan2(flux_b, flux_a) + _sign(targ_vel)*FLO_angle_offset);
  
  if ( (FLO_angle_el - FLO_angle_el_prev) < ((-1)*_PI) ) {
  	FLO_angle_el_cntr += 1;
  	if ( FLO_angle_el_cntr >= pole_pairs ) { FLO_angle_el_cntr = 0; }
  }
  
  if ( (FLO_angle_el - FLO_angle_el_prev) > (_PI) ) {
  	FLO_angle_el_cntr -= 1;
  	if ( FLO_angle_el_cntr <= (-pole_pairs) ) { FLO_angle_el_cntr = 0; }
  }
  
  FLO_angle = _normalizeAngle( ( FLO_angle_el + 2*_PI*(float)FLO_angle_el_cntr ) / (float)pole_pairs );
  FLO_angle_el_prev = FLO_angle_el;
  return FLO_angle;
}

int SensorlessFluxLinkageObserver::needsSearch(){
  return 1;
}


int SensorlessFluxLinkageObserver::issensorless(){
  return 1;
}
