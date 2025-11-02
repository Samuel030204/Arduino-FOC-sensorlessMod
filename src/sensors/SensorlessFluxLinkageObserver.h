#ifndef SENSORLESSFLUXLINKAGEOBSERVER_LIB_H
#define SENSORLESSFLUXLINKAGEOBSERVER_LIB_H

#include "Arduino.h"
#include "../common/base_classes/Sensor.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/lowpass_filter.h"
#include "../common/highpass_filter.h"

class SensorlessFluxLinkageObserver: public Sensor{
 public:
    /**
     * SensorlessFluxLinkageObserver class constructor
     */
    SensorlessFluxLinkageObserver(float _FLO_angle_offset, int _HPF_ENB);
    
    /** sensor initialization */
    void init();
    
    void linkFOCMotor(FOCMotor* motor);
    void linkCurrentSense(CurrentSense* current_sense);
    
    FOCMotor* motor;
    CurrentSense* current_sense;
    
    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getSensorAngle() override;
    
    /**
     * returns 0 if it does need search for absolute zero
     * 0 - encoder without index 
     * 1 - encoder with index
     */
    int needsSearch() override;
    int issensorless() override;
    
    float FLO_angle_offset;
    int HPF_ENB;
    
    float FLO_angle_el;
    float FLO_angle_el_prev;
	int FLO_angle_el_cntr;
    float FLO_angle;
    
    PhaseCurrent_s currents;
    ABCurrent_s ABCurrent;
    ABVoltage_s ABVoltage;
    
    float a_intgrl;
    float b_intgrl;
    
    float flux_a;
    float flux_b;
    
	float FLO_lps;
	
	LowPassFilter LPF_vri_alpha{DEF_VRI_LP_FILTER_Tf};
	LowPassFilter LPF_vri_beta{DEF_VRI_LP_FILTER_Tf};
	
	HighPassFilter HPF_vri_alpha{DEF_VRI_HP_FILTER_Tf};
	HighPassFilter HPF_vri_beta{DEF_VRI_HP_FILTER_Tf};
	
	HighPassFilter HPF_flux_alpha{DEF_FLUX_HP_FILTER_Tf};
	HighPassFilter HPF_flux_beta{DEF_FLUX_HP_FILTER_Tf};

  private:
//    PhaseCurrent_s currents;
//    ABCurrent_s ABCurrent, ABCurrent_prev;
//    
//    float flux_a;
//    float flux_b;

	float phase_resistance;
	float phase_inductance;
	int pole_pairs;
	float targ_vel;
    
    float now;
    float FLO_timestamp;
    float Ts;
};


#endif
