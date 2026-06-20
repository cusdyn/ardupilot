#include "AC_AttitudeControl_Multi_LQ.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <stdio.h>
#include <string>
#include <sstream>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#define LQ_SEND_TEXT(severity, format, args...) gcs().send_text(severity, format, ##args)

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Multi_LQ::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl, 0),

    // @Param: RAT_RLL_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.01 0.5
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_RLL_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_RLL_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_RLL_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_RLL_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_roll, "RAT_RLL_", 1, AC_AttitudeControl_Multi_LQ, AC_PID),

    // @Param: RAT_PIT_P
    // @DisplayName: Pitch axis rate controller P gain
    // @Description: Pitch axis rate controller P gain.  Converts the difference between desired pitch rate and actual pitch rate into a motor speed output
    // @Range: 0.01 0.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_PIT_I
    // @DisplayName: Pitch axis rate controller I gain
    // @Description: Pitch axis rate controller I gain.  Corrects long-term difference in desired pitch rate vs actual pitch rate
    // @Range: 0.01 2.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_IMAX
    // @DisplayName: Pitch axis rate controller I gain maximum
    // @Description: Pitch axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_PIT_D
    // @DisplayName: Pitch axis rate controller D gain
    // @Description: Pitch axis rate controller D gain.  Compensates for short-term change in desired pitch rate vs actual pitch rate
    // @Range: 0.0 0.05
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FF
    // @DisplayName: Pitch axis rate controller feed forward
    // @Description: Pitch axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_PIT_FLTT
    // @DisplayName: Pitch axis rate controller target frequency in Hz
    // @Description: Pitch axis rate controller target frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTE
    // @DisplayName: Pitch axis rate controller error frequency in Hz
    // @Description: Pitch axis rate controller error frequency in Hz
    // @Range: 0 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_FLTD
    // @DisplayName: Pitch axis rate controller derivative frequency in Hz
    // @Description: Pitch axis rate controller derivative frequency in Hz
    // @Range: 5 100
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_PIT_SMAX
    // @DisplayName: Pitch slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_pitch, "RAT_PIT_", 2, AC_AttitudeControl_Multi_LQ, AC_PID),

    // @Param: RAT_YAW_P
    // @DisplayName: Yaw axis rate controller P gain
    // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
    // @Range: 0.10 2.50
    // @Increment: 0.005
    // @User: Standard

    // @Param: RAT_YAW_I
    // @DisplayName: Yaw axis rate controller I gain
    // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
    // @Range: 0.010 1.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_IMAX
    // @DisplayName: Yaw axis rate controller I gain maximum
    // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: RAT_YAW_D
    // @DisplayName: Yaw axis rate controller D gain
    // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
    // @Range: 0.000 0.02
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FF
    // @DisplayName: Yaw axis rate controller feed forward
    // @Description: Yaw axis rate controller feed forward
    // @Range: 0 0.5
    // @Increment: 0.001
    // @User: Standard

    // @Param: RAT_YAW_FLTT
    // @DisplayName: Yaw axis rate controller target frequency in Hz
    // @Description: Yaw axis rate controller target frequency in Hz
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTE
    // @DisplayName: Yaw axis rate controller error frequency in Hz
    // @Description: Yaw axis rate controller error frequency in Hz
    // @Range: 0 20
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_FLTD
    // @DisplayName: Yaw axis rate controller derivative frequency in Hz
    // @Description: Yaw axis rate controller derivative frequency in Hz
    // @Range: 5 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: RAT_YAW_SMAX
    // @DisplayName: Yaw slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 3, AC_AttitudeControl_Multi_LQ, AC_PID),

    // @Param: THR_MIX_MIN
    // @DisplayName: Throttle Mix Minimum
    // @Description: Throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.25
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MIN", 4, AC_AttitudeControl_Multi_LQ, _thr_mix_min, AC_ATTITUDE_CONTROL_MIN_DEFAULT),

    // @Param: THR_MIX_MAX
    // @DisplayName: Throttle Mix Maximum
    // @Description: Throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.5 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAX", 5, AC_AttitudeControl_Multi_LQ, _thr_mix_max, AC_ATTITUDE_CONTROL_MAX_DEFAULT),

    // @Param: THR_MIX_MAN
    // @DisplayName: Throttle Mix Manual
    // @Description: Throttle vs attitude control prioritisation used during manual flight (higher values mean we prioritise attitude control over throttle)
    // @Range: 0.1 0.9
    // @User: Advanced
    AP_GROUPINFO("THR_MIX_MAN", 6, AC_AttitudeControl_Multi_LQ, _thr_mix_man, AC_ATTITUDE_CONTROL_MAN_DEFAULT),

    // @Param: PHY_KV
    // @DisplayName: Motor Kv
    // @Description: Motor velocity constant (RPM/volt)
    // @Range: 0 to 10000
    // @User: Advanced
    AP_GROUPINFO("PHY_KV", 7, AC_AttitudeControl_Multi_LQ, _phy_kv, AC_ATC_LQ_PHY_KV_DEFAULT ),

    // @Param: PHY_ESC_R
    // @DisplayName: ESC Electrical Resistance
    // @Description: ESC resistance (Ohms)
    // @Range: 0 to 10 
    // @User: Advanced
    AP_GROUPINFO("PHY_ESC_R", 8, AC_AttitudeControl_Multi_LQ, _phy_esc_r, AC_ATC_LQ_PHY_ESC_R_DEFAULT ),

   // @Param: PHY_MOT_R
    // @DisplayName: Motor Internal Resistance
    // @Description: Motor internal resistance (Ohms)
    // @Range: 0 to 10 
    // @User: Advanced
    AP_GROUPINFO("PHY_MOT_R", 9, AC_AttitudeControl_Multi_LQ, _phy_mot_r, AC_ATC_LQ_PHY_MOT_R_DEFAULT ),

    // @Param: PHY_PCONST
    // @DisplayName: Prop Power Coefficient
    // @Description: Lumped motor internal resistance (Ohms)
    // @Range: 0 to 10 
    // @User: Advanced
    AP_GROUPINFO("PHY_PCONST", 10, AC_AttitudeControl_Multi_LQ, _phy_pconst, AC_ATC_LQ_PHY_PCONST_DEFAULT ),


    // @Param: PHY_TCONST
    // @DisplayName: Prop Power Coefficient
    // @Description: Lumped motor internal resistance (Ohms)
    // @Range: 0 to 10 
    // @User: Advanced
    AP_GROUPINFO("PHY_TCONST", 11, AC_AttitudeControl_Multi_LQ, _phy_tconst, AC_ATC_LQ_PHY_TCONST_DEFAULT ),

    // @Param: PHY_PROP_DIA
    // @DisplayName: Prop Diameter
    // @Description: Propeller Diameter (m)
    // @Range: 0 to 1
    // @User: Advanced
    AP_GROUPINFO("PHY_PROP_DIA", 12, AC_AttitudeControl_Multi_LQ, _phy_propdia, AC_ATC_LQ_PHY_PROP_DIA_DEFAULT ),

    // @Param: PHY_PROP_MASS
    // @DisplayName: Prop Mass
    // @Description: Propeller Mass (kg)
    // @Range: 0 to 1
    // @User: Advanced
    AP_GROUPINFO("PHY_PROPMASS", 13, AC_AttitudeControl_Multi_LQ, _phy_propmass, AC_ATC_LQ_PHY_PROP_MASS_DEFAULT ),

    // @Param: PHY_PROP_AIR_DENSITY
    // @DisplayName: Prop Mass
    // @Description: Propeller Mass (kg)
    // @Range: 0 to 10
    // @User: Advanced
    AP_GROUPINFO("PHY_AIRRHO", 14, AC_AttitudeControl_Multi_LQ, _phy_rho, AC_ATC_LQ_PHY_AIR_DENSITY_DEFAULT ),

    // @Param: PHY_ARMLEN
    // @DisplayName: Motor Arm Length
    // @Description: Motor Arm Length (m)
    // @Range: 0 to 1
    // @User: Advanced
    AP_GROUPINFO("PHY_ARMLEN", 15, AC_AttitudeControl_Multi_LQ, _phy_armlen, AC_ATC_LQ_PHY_ARMLEN_DEFAULT ),

    // @Param: PHY_NOMV
    // @DisplayName: Nominal battery voltage
    // @Description: Nominal battery voltage used as fallback for wmax calculation when battery monitor is unavailable.
    // @Units: V
    // @Range: 6.0 60.0
    // @User: Advanced
    AP_GROUPINFO("PHY_NOMV", 16, AC_AttitudeControl_Multi_LQ, _phy_nominal_voltage, AC_ATC_LQ_PHY_NOMV_DEFAULT ),

    AP_GROUPEND
};

AC_AttitudeControl_Multi_LQ::AC_AttitudeControl_Multi_LQ(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl(ahrs, aparm, motors, dt),
    _motors_multi(motors),
    _pid_rate_roll(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_pitch(AC_ATC_MULTI_RATE_RP_P, AC_ATC_MULTI_RATE_RP_I, AC_ATC_MULTI_RATE_RP_D, 0.0f, AC_ATC_MULTI_RATE_RP_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, 0.0f, AC_ATC_MULTI_RATE_RP_FILT_HZ, dt),
    _pid_rate_yaw(AC_ATC_MULTI_RATE_YAW_P, AC_ATC_MULTI_RATE_YAW_I, AC_ATC_MULTI_RATE_YAW_D, 0.0f, AC_ATC_MULTI_RATE_YAW_IMAX, AC_ATC_MULTI_RATE_RP_FILT_HZ, AC_ATC_MULTI_RATE_YAW_FILT_HZ, 0.0f, dt)
{
    AP_Param::setup_object_defaults(this, var_info);

    _dataStreamCounter = 0;
    _thrust_command_actual = 0;
    _prop_inertia = 0;
    _motor_kt = 0;
    _last_nominal_rpm = 0;
    _b = 0;
    _d=0;

    // read LQ configuration from SD card
    InitializeFileConstants();
}

// Update Alt_Hold angle maximum
void AC_AttitudeControl_Multi_LQ::update_althold_lean_angle_max(float throttle_in)
{
    // calc maximum tilt angle based on throttle
    float thr_max = _motors_multi.get_throttle_thrust_max();

    // divide by zero check
    if (is_zero(thr_max)) {
        _althold_lean_angle_max = 0.0f;
        return;
    }

    float althold_lean_angle_max = acosf(constrain_float(throttle_in / (AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX * thr_max), 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_AttitudeControl_Multi_LQ::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        // Apply angle boost
        throttle_in = get_throttle_boosted(throttle_in);
    } else {
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
    _motors.set_throttle(throttle_in);
    _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));\

    NormalizedThrustToActual(throttle_in);
}

void AC_AttitudeControl_Multi_LQ::set_throttle_mix_max(float ratio)
{
    ratio = constrain_float(ratio, 0.0f, 1.0f);
    _throttle_rpy_mix_desired = (1.0f - ratio) * _thr_mix_min + ratio * _thr_mix_max;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi_LQ::get_throttle_boosted(float throttle_in)
{
    if (!_angle_boost_enabled) {
        _angle_boost = 0;
        return throttle_in;
    }
    // inverted_factor is 1 for tilt angles below 60 degrees
    // inverted_factor reduces from 1 to 0 for tilt angles between 60 and 90 degrees

    float cos_tilt = _ahrs.cos_pitch() * _ahrs.cos_roll();
    float inverted_factor = constrain_float(10.0f * cos_tilt, 0.0f, 1.0f);
    float cos_tilt_target = cosf(_thrust_angle);
    float boost_factor = 1.0f / constrain_float(cos_tilt_target, 0.1f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    _angle_boost = constrain_float(throttle_out - throttle_in, -1.0f, 1.0f);
    return throttle_out;
}

// returns a throttle including compensation for roll/pitch angle
// throttle value should be 0 ~ 1
float AC_AttitudeControl_Multi_LQ::get_throttle_avg_max(float throttle_in)
{
    throttle_in = constrain_float(throttle_in, 0.0f, 1.0f);
    return MAX(throttle_in, throttle_in * MAX(0.0f, 1.0f - _throttle_rpy_mix) + _motors.get_throttle_hover() * _throttle_rpy_mix);
}

// update_throttle_rpy_mix - slew set_throttle_rpy_mix to requested value
void AC_AttitudeControl_Multi_LQ::update_throttle_rpy_mix()
{
    // slew _throttle_rpy_mix to _throttle_rpy_mix_desired
    if (_throttle_rpy_mix < _throttle_rpy_mix_desired) {
        // increase quickly (i.e. from 0.1 to 0.9 in 0.4 seconds)
        _throttle_rpy_mix += MIN(2.0f * _dt, _throttle_rpy_mix_desired - _throttle_rpy_mix);
    } else if (_throttle_rpy_mix > _throttle_rpy_mix_desired) {
        // reduce more slowly (from 0.9 to 0.1 in 1.6 seconds)
        _throttle_rpy_mix -= MIN(0.5f * _dt, _throttle_rpy_mix - _throttle_rpy_mix_desired);
    }
    _throttle_rpy_mix = constrain_float(_throttle_rpy_mix, 0.1f, AC_ATTITUDE_CONTROL_MAX);
}

void AC_AttitudeControl_Multi_LQ::rate_controller_run()
{
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    // Ensure euler angle targets are synchronized with attitude target quaternion for CalcLQoutput
    _attitude_target.to_euler(_euler_angle_target.x, _euler_angle_target.y, _euler_angle_target.z);

    _ang_vel_body += _sysid_ang_vel_body;

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    _motors.set_roll(get_rate_roll_pid().update_all(_ang_vel_body.x, gyro_latest.x, _motors.limit.roll) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(_ang_vel_body.y, gyro_latest.y, _motors.limit.pitch) + _actuator_sysid.y);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(_ang_vel_body.z, gyro_latest.z, _motors.limit.yaw) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff()*_feedforward_scalar);

    _sysid_ang_vel_body.zero();
    _actuator_sysid.zero();

    control_monitor_update();

    CalcLQoutput();

    //diag_data_out();

}


// Apply Control law for body torques u = -Kx from state feedback x.
// With roll, pitch, yaw body torques and body-axis aggregate thrust given from Z-controller
// solve motor speeds (4 equations in 4 unknown speeds)
void AC_AttitudeControl_Multi_LQ::CalcLQoutput()
{
    Quaternion q;
    VectorN<float, AC_ATC_LQ_STATE_COUNT> x; 
    VectorN<float, AC_ATC_LQ_STATE_COUNT> r; 
    VectorN<float, AC_ATC_LQ_CMD_COUNT> B; 
    VectorN<float, AC_ATC_LQ_CMD_COUNT> c; 
    VectorN<float, AC_ATC_LQ_CMD_COUNT> w; 
    float cmd[AC_ATC_LQ_CMD_COUNT]; 
    float nroll;
    float npitch;
    float nyaw;
    float Tmax;
    float Ymax;
    float wms;

    Vector3f gyro = _ahrs.get_gyro_latest();

    _ahrs.get_quat_body_to_ned(q);

    // state feedback: body angles (in radians ) and rates.
    x[0] = q.get_euler_roll();
    x[1] = gyro.x;
    x[2] = q.get_euler_pitch();
    x[3] = gyro.y;
    x[4] = q.get_euler_yaw();
    x[5] = gyro.z;

    
    // reference inputs are desired angles and zero rates
    r[0] = _attitude_target.get_euler_roll();
    r[1] = 0;
    r[2] = _attitude_target.get_euler_pitch();
    r[3] = 0;
    r[4] = _attitude_target.get_euler_yaw();
    r[5] = 0;

    for( int i=0; i < AC_ATC_LQ_CMD_COUNT; i++){
        cmd[i]=0;
        for( int j=0; j < AC_ATC_LQ_STATE_COUNT; j++){
            cmd[i] += -_k[i][j]*x[j] + _Nb[i][j]*r[j]; 
        }        
    }

    // These are body angle commands.

    // u1 = roll torgue
    // u2 = pitch torque
    // u3  = yaw torque
    // u4 = prop gyro effect.

    // u1=3 and aggregate thrust T are used to solve for motor speed commands.
    // 'b' vector for Aw=b as b=[T u1 u2 u3]' where T
    B[0] = _thrust_command_actual;
    B[1] = cmd[0];
    B[2] = cmd[1];
    B[3] = cmd[2];

    // forward substitute
    c[0] = B[0];
    c[1] = B[1] - _L[1][0]*c[0];
    c[2] = B[2] - _L[2][0]*c[0] - _L[2][1]*c[1];
    c[3] = B[3] - _L[3][0]*c[0] - _L[3][1]*c[1] - _L[3][2]*c[2];

    // back substitute for prop speeds from c...
    w[3] = c[3]/_U[3][3];
    w[2] = 1/_U[2][2]*(c[2] - _U[2][3]*w[3]);
    w[1] = 1/_U[1][1]*(c[1] - _U[1][2]*w[2] - _U[1][3]*w[3]);
    w[0] = 1/_U[0][0]*(c[0] - _U[0][1]*w[1] - _U[0][2]*w[2] - _U[0][3]*w[3]);


    // map to Ardu '+' frame quat motor layout as omegas...
    _omega[0] = safe_sqrt(fabsf(w[0]));
    _omega[1] = safe_sqrt(fabsf(w[1]));
    _omega[2] = safe_sqrt(fabsf(w[2]));
    _omega[3] = safe_sqrt(fabsf(w[3]));

    // get battery voltage for motor speed scaling
    float vbat = AP::battery().voltage(0);
    if (!is_positive(vbat)) {
        vbat = _phy_nominal_voltage;
    }
    _wmax = _phy_kv * vbat * M_2PI / 60;

    wms = _wmax*_wmax;
    Tmax = _b*_phy_armlen*wms;
    Ymax = _d*2*wms;

    nroll  = MIN(MAX(cmd[0]/Tmax, -1.0f), 1.0f);
    npitch = MIN(MAX(cmd[1]/Tmax, -1.0f), 1.0f);
    nyaw   = MIN(MAX(cmd[2]/Ymax, -1.0f), 1.0f);

    _motors.set_roll(nroll);
    _motors.set_roll_ff(0.0);

    _motors.set_pitch(npitch);
   _motors.set_pitch_ff(0.0);

   _motors.set_yaw(nyaw);
   _motors.set_yaw_ff(0.0);


}



// sanity check parameters.  should be called once before takeoff
void AC_AttitudeControl_Multi_LQ::parameter_sanity_check()
{
    // sanity check throttle mix parameters
    if (_thr_mix_man < 0.1f || _thr_mix_man > AC_ATTITUDE_CONTROL_MAN_LIMIT) {
        // parameter description recommends thr-mix-man be no higher than 0.9 but we allow up to 4.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_man.set_and_save(constrain_float(_thr_mix_man, 0.1, AC_ATTITUDE_CONTROL_MAN_LIMIT));
    }
    if (_thr_mix_min < 0.1f || _thr_mix_min > AC_ATTITUDE_CONTROL_MIN_LIMIT) {
        _thr_mix_min.set_and_save(constrain_float(_thr_mix_min, 0.1, AC_ATTITUDE_CONTROL_MIN_LIMIT));
    }
    if (_thr_mix_max < 0.5f || _thr_mix_max > AC_ATTITUDE_CONTROL_MAX) {
        // parameter description recommends thr-mix-max be no higher than 0.9 but we allow up to 5.0
        // which can be useful for very high powered copters with very low hover throttle
        _thr_mix_max.set_and_save(constrain_float(_thr_mix_max, 0.5, AC_ATTITUDE_CONTROL_MAX));
    }
    if (_thr_mix_min > _thr_mix_max) {
        _thr_mix_min.set_and_save(AC_ATTITUDE_CONTROL_MIN_DEFAULT);
        _thr_mix_max.set_and_save(AC_ATTITUDE_CONTROL_MAX_DEFAULT);
    }
}


//void AC_AttitudeControl_Multi_LQ:: 

void AC_AttitudeControl_Multi_LQ::diag_data_out( )
{

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

   Quaternion q;
   _ahrs.get_quat_body_to_ned(q);

   if(++_dataStreamCounter%5==0)
   {
       data_to_send dts;
       dts.gyro = gyro_latest;
       //dts.motor = att;
       dts.motor_rpy.x = _motors.get_roll();
       dts.motor_rpy.y = _motors.get_pitch();
       dts.motor_rpy.z = _motors.get_yaw();
       dts.target_euler_rpy = _euler_angle_target;
       dts.target_angle_rate_rpy = _ang_vel_body;
       memcpy(dts.u,_u,sizeof(_u));
       dts.thrust = _thrust_command_actual;
 //      sock.sendto(&dts, sizeof(dts), "10.1.1.10", 9003);    
   }
     
}

void AC_AttitudeControl_Multi_LQ::InitializeFileConstants()
{
    InitializeLQ_K();
    InitializeLQ_Nb();
    InitializeLQ_W();
}


// Scaled body Z-axis 0-1 scaled thrust command to an actual
// thrust command in Newtons.
void AC_AttitudeControl_Multi_LQ::NormalizedThrustToActual( float normthrust )
{
    float volts;
    float Kt;
    float current;
    float torque;
    float dragtorque;
    float prop_inertia;
    float w;
    float w1;
    float rps;
    float thrust;



    prop_inertia = _phy_propmass*powf(_phy_propdia,2)/(float)12.0;
    volts   = normthrust*AP::sitl()->batt_voltage;
    Kt      = M_2PI/(60*_phy_kv);
    current = (_phy_kv*volts - _last_nominal_rpm)/(_phy_esc_r+_phy_mot_r);
    torque  = current * Kt;
    dragtorque = _phy_pconst * _phy_rho * powf(_last_nominal_rpm/60, 2.0) * powf(_phy_propdia, 5.0); 

    w  = _last_nominal_rpm*M_2PI/60;

    w1 = w + (torque-dragtorque)/prop_inertia * 1/AP::scheduler().get_loop_rate_hz();

    rps = w1/M_2PI;

    // single motor thrust
    thrust = _phy_tconst * _phy_rho * powf(rps,2.0) * powf(_phy_propdia, 4.0);

    _thrust_command_actual = 4*thrust; // quad so we return aggregate desired thrust in Newtons

    _last_nominal_rpm = rps*60;



}


// This is the A matrix for Ax=b solver for desired motor speeds x given
// thrust and cross-body torques in b.
void AC_AttitudeControl_Multi_LQ::InitializeLQ_W()
{
    // Thrust coefficient 'b' 
    _b = _phy_tconst*_phy_rho*powf(_phy_propdia,4)/(4*powf(M_PI,2));
    
    // drag torque coefficient 'd'
    _d = _phy_pconst*_phy_rho*powf(_phy_propdia,5)/(8*powf(M_PI,3));

    // MIKE TO DO: for the X frame you need to change 
    // total body up axis thrust is first row
    _W[0][0] = _b;
    _W[0][1] = _b;
    _W[0][2] = _b;
    _W[0][3] = _b;

    _W[1][0] = -_b*_phy_armlen;
    _W[1][1] = _b*_phy_armlen;
    _W[1][2] = 0;
    _W[1][3] = 0;

    _W[2][0] = 0;
    _W[2][1] = 0;
    _W[2][2] = -_b*_phy_armlen;
    _W[2][3] = _b*_phy_armlen;;

    _W[3][0] = _d;
    _W[3][1] = _d;
    _W[3][2] = -_d;
    _W[3][3] = -_d;

    // LU decompose it for run-time solver
    mat_LU_decompose((const float*)&_W[0][0],&_L[0][0],&_U[0][0],&_P[0][0],AC_ATC_LQ_CMD_COUNT);
}

void AC_AttitudeControl_Multi_LQ::InitializeLQ_K()
{
    char filebuf[600];
    int row=0;

    int fd;
    
    fd = AP::FS().open("LQ/k.txt", O_RDWR|O_CREAT);
    if (fd == -1) 
    {
        LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Open LQ K Gain File failed.");
    }
    else
    {
        LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Successfully Opened  controller Gain File");
        int cnt = AP::FS().read(fd, filebuf, 600);
        LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Read %u bytes", cnt);
        filebuf[cnt] = 0; // null the end of the string

        std::string str(filebuf,cnt);
        std::istringstream ss(str);
        std::string line;

        row = 0;
        while (std::getline(ss,line) && row < AC_ATC_LQ_CMD_COUNT) {
            // process line
            sscanf(line.c_str(),"%f,%f,%f,%f,%f,%f", 
                                 &_k[row][0], &_k[row][1], &_k[row][2],
                                 &_k[row][3], &_k[row][4], &_k[row][5]);
            row++;
            LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Line: %s", (char *)line.c_str());
        }
    }
}

void AC_AttitudeControl_Multi_LQ::InitializeLQ_Nb()
{
    char filebuf[600];
    int row=0;

    int fd;
    
    fd = AP::FS().open("LQ/Nb.txt", O_RDWR|O_CREAT);
    if (fd == -1) 
    {
        LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Open LQ Nb Gain File failed.");
    }
    else
    {
        LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Successfully Opened  controller feedforward Gain File");
        int cnt = AP::FS().read(fd, filebuf, 600);
        LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Read %u bytes", cnt);
        filebuf[cnt] = 0; // null the end of the string

        std::string str(filebuf,cnt);
        std::istringstream ss(str);
        std::string line;

        row = 0;
        while (std::getline(ss,line) && row < AC_ATC_LQ_CMD_COUNT) {
            // process line
            sscanf(line.c_str(),"%f,%f,%f,%f,%f,%f", 
                                 &_Nb[row][0], &_Nb[row][1], &_Nb[row][2],
                                 &_Nb[row][3], &_Nb[row][4], &_Nb[row][5]);
            row++;
            LQ_SEND_TEXT(MAV_SEVERITY_INFO, "Line: %s", (char *)line.c_str());
        }
    }
}


void AC_AttitudeControl_Multi_LQ::input_thrust_vector_rate_heading(const Vector3f& thrust_vector, float heading_rate_cds, bool slew_yaw)
{
    // Call base class implementation to set up attitude targets
    AC_AttitudeControl::input_thrust_vector_rate_heading(thrust_vector, heading_rate_cds, slew_yaw);
}

void AC_AttitudeControl_Multi_LQ::input_thrust_vector_heading(const Vector3f& thrust_vector, float heading_angle_cd, float heading_rate_cds)
{
    // Call base class implementation to set up attitude targets
    AC_AttitudeControl::input_thrust_vector_heading(thrust_vector, heading_angle_cd, heading_rate_cds);
}