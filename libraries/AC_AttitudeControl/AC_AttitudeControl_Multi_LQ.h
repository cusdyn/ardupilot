#pragma once

/// @file    AC_AttitudeControl_Multi_LQ.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_HAL/utility/Socket.h>

// default rate controller PID gains
#ifndef AC_ATC_MULTI_RATE_RP_P
  # define AC_ATC_MULTI_RATE_RP_P           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_I
  # define AC_ATC_MULTI_RATE_RP_I           0.135f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_D
  # define AC_ATC_MULTI_RATE_RP_D           0.0036f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_IMAX
 # define AC_ATC_MULTI_RATE_RP_IMAX         0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_RP_FILT_HZ
 # define AC_ATC_MULTI_RATE_RP_FILT_HZ      20.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_P
 # define AC_ATC_MULTI_RATE_YAW_P           0.180f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_I
 # define AC_ATC_MULTI_RATE_YAW_I           0.018f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_D
 # define AC_ATC_MULTI_RATE_YAW_D           0.0f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_IMAX
 # define AC_ATC_MULTI_RATE_YAW_IMAX        0.5f
#endif
#ifndef AC_ATC_MULTI_RATE_YAW_FILT_HZ
 # define AC_ATC_MULTI_RATE_YAW_FILT_HZ     2.5f
#endif

#ifndef AC_ATC_LQ_STATE_COUNT
  #define AC_ATC_LQ_STATE_COUNT 6
#endif

#ifndef AC_ATC_LQ_CMD_COUNT
  #define AC_ATC_LQ_CMD_COUNT 4
#endif

#define AC_ATC_LQ_PHY_KV_DEFAULT                 880.0f    // Kv (RPM/volt)
#define AC_ATC_LQ_PHY_MOT_R_DEFAULT              0.115f    // Motor internal resistance (ohms)
#define AC_ATC_LQ_PHY_ESC_R_DEFAULT              0.01f     // ESC internal resistance (ohms)
#define AC_ATC_LQ_PHY_PCONST_DEFAULT             1.13f     // prop power coefficient
#define AC_ATC_LQ_PHY_TCONST_DEFAULT             1.0f      // prop torque coefficient
#define AC_ATC_LQ_PHY_PROP_DIA_DEFAULT           0.245     // prop diameter (m)
#define AC_ATC_LQ_PHY_PROP_MASS_DEFAULT          0.0125    // prop mass (kg)
#define AC_ATC_LQ_PHY_AIR_DENSITY_DEFAULT        1.225     // (kg/m^3)
#define AC_ATC_LQ_PHY_ARMLEN_DEFAULT             0.225     // Hexsoon 450mm frame (m)

typedef struct {
        Vector3f gyro;
        Vector3f motor_rpy;
        Vector3f target_euler_rpy;
        Vector3f target_angle_rate_rpy;
        float    u[4];
        float    thrust;
} data_to_send;


class AC_AttitudeControl_Multi_LQ : public AC_AttitudeControl {
public:
	AC_AttitudeControl_Multi_LQ(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

	// empty destructor to suppress compiler warning
	virtual ~AC_AttitudeControl_Multi_LQ() {}

    // pid accessors
    AC_PID& get_rate_roll_pid() override { return _pid_rate_roll; }
    AC_PID& get_rate_pitch_pid() override { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() override { return _pid_rate_yaw; }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    // run lowest level body-frame rate controller and send outputs to the motors
    void rate_controller_run() override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    AP_MotorsMulticopter& _motors_multi;
    AC_PID                _pid_rate_roll;
    AC_PID                _pid_rate_pitch;
    AC_PID                _pid_rate_yaw;

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)

    AP_Float              _phy_kv;          // motor assumed velocity constant, RPM per Volt.
    AP_Float              _phy_esc_r;       // esc resistance (ohms).
    AP_Float              _phy_mot_r;       // motor internal resistance (ohms).
    AP_Float              _phy_pconst;      // prop power coefficient (for drag).
    AP_Float              _phy_tconst;      // prop thrust coefficient.
    AP_Float              _phy_propdia;     // prop diameter (m).
    AP_Float              _phy_propmass;    // prop mass (kg) used for prop inertia model.
    AP_Float              _phy_rho;         // air density (kg/m^3). 
    AP_Float              _phy_armlen;      // motor arm length (m). 

private:
    void CalcLQoutput();
    void NormalizedThrustToActual( float normthrust );
    void InitializeFileConstants();
    int  _dataStreamCounter;
    void InitializeLQ_K();
    void InitializeLQ_W();
    void InitializePhysicalConstantsLQ();
    void diag_data_out();
    SocketAPM sock{true};

    float _k[AC_ATC_LQ_CMD_COUNT][AC_ATC_LQ_STATE_COUNT];
    float _W[AC_ATC_LQ_CMD_COUNT][AC_ATC_LQ_CMD_COUNT];
    float _u[AC_ATC_LQ_STATE_COUNT]; 

    float _L[AC_ATC_LQ_CMD_COUNT][AC_ATC_LQ_CMD_COUNT];
    float _U[AC_ATC_LQ_CMD_COUNT][AC_ATC_LQ_CMD_COUNT];
    float _P[AC_ATC_LQ_CMD_COUNT][AC_ATC_LQ_CMD_COUNT];

    float _thrust_command_actual;

    float _prop_inertia;
    float _motor_kt;
    float _last_nominal_rpm;
    float _b;   // thrust factor
    float _d;   // drag factor

    float _omega[AC_ATC_LQ_CMD_COUNT];

};
