// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AC_AttitudeControl.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl::var_info[] = {

    // 0, 1 were RATE_RP_MAX, RATE_Y_MAX

    // @Param: SLEW_YAW
    // @DisplayName: Yaw target slew rate
    // @Description: Maximum rate the yaw target can be updated in Loiter, RTL, Auto flight modes
    // @Units: Centi-Degrees/Sec
    // @Range: 500 18000
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("SLEW_YAW",    2, AC_AttitudeControl, _slew_yaw, AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT_CDS),

    // 3 was for ACCEL_RP_MAX

    // @Param: ACCEL_Y_MAX
    // @DisplayName: Acceleration Max for Yaw
    // @Description: Maximum acceleration in yaw axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 72000
    // @Values: 0:Disabled, 18000:Slow, 36000:Medium, 54000:Fast
    // @Increment: 1000
    // @User: Advanced
    AP_GROUPINFO("ACCEL_Y_MAX",  4, AC_AttitudeControl, _accel_yaw_max, AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_CDSS),

    // @Param: RATE_FF_ENAB
    // @DisplayName: Rate Feedforward Enable
    // @Description: Controls whether body-frame rate feedfoward is enabled or disabled
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("RATE_FF_ENAB", 5, AC_AttitudeControl, _rate_bf_ff_enabled, AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT),

    // @Param: ACCEL_R_MAX
    // @DisplayName: Acceleration Max for Roll
    // @Description: Maximum acceleration in roll axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_R_MAX", 6, AC_AttitudeControl, _accel_roll_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // @Param: ACCEL_P_MAX
    // @DisplayName: Acceleration Max for Pitch
    // @Description: Maximum acceleration in pitch axis
    // @Units: Centi-Degrees/Sec/Sec
    // @Range: 0 180000
    // @Increment: 1000
    // @Values: 0:Disabled, 72000:Slow, 108000:Medium, 162000:Fast
    // @User: Advanced
    AP_GROUPINFO("ACCEL_P_MAX", 7, AC_AttitudeControl, _accel_pitch_max, AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_CDSS),

    // IDs 8,9,10,11 RESERVED (in use on Solo)

    // @Param: ANGLE_BOOST
    // @DisplayName: Angle Boost
    // @Description: Angle Boost increases output throttle as the vehicle leans to reduce loss of altitude
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ANGLE_BOOST", 12, AC_AttitudeControl, _angle_boost_enabled, 1),

    // @Param: ANG_RLL_P
    // @DisplayName: Roll axis angle controller P gain
    // @Description: Roll axis angle controller P gain.  Converts the error between the desired roll angle and actual angle to a desired roll rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_roll, "ANG_RLL_", 13, AC_AttitudeControl, AC_P),

    // @Param: ANG_PIT_P
    // @DisplayName: Pitch axis angle controller P gain
    // @Description: Pitch axis angle controller P gain.  Converts the error between the desired pitch angle and actual angle to a desired pitch rate
    // @Range: 3.000 12.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_pitch, "ANG_PIT_", 14, AC_AttitudeControl, AC_P),

    // @Param: ANG_YAW_P
    // @DisplayName: Yaw axis angle controller P gain
    // @Description: Yaw axis angle controller P gain.  Converts the error between the desired yaw angle and actual angle to a desired yaw rate
    // @Range: 3.000 6.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_angle_yaw, "ANG_YAW_", 15, AC_AttitudeControl, AC_P),

    AP_GROUPEND
};

void AC_AttitudeControl::relax_bf_rate_controller()	//# 设参考角速率为当前角速率,清空PID控制器的I, p,q,r=>dphi,dtheta,dpsi(i.e. bf->ef).
{
    // Set reference angular velocity used in angular velocity controller equal
    // to the input angular velocity and reset the angular velocity integrators.
    // This zeros the output of the angular velocity controller.
    _ang_vel_target_rads = _ahrs.get_gyro();
    get_rate_roll_pid().reset_I();
    get_rate_pitch_pid().reset_I();
    get_rate_yaw_pid().reset_I();

    // Write euler derivatives derived from vehicle angular velocity to
    // _att_target_euler_rate_rads. This resets the state of the input shapers.
    ang_vel_to_euler_rate(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _ang_vel_target_rads, _att_target_euler_rate_rads);
							//# i.e. ang_vel: p,q,r. ==> euler_rate: derivatives of phi, theta, and psi.
}

void AC_AttitudeControl::shift_ef_yaw_target(float yaw_shift_cd)
{
    _att_target_euler_rad.z = wrap_2PI(_att_target_euler_rad.z + radians(yaw_shift_cd*0.01f));
}

		//# 姿态控制器 1. 计算控制器目标角速度_ang_vel_target_rads(bf)(=角度误差*P + 前馈目标角速度rf->bf).
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw_smooth(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds, float smoothing_gain)		//#  smooth makes it too complicated!
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle_rad = radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle_rad = radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_rate_rads = radians(euler_yaw_rate_cds*0.01f);

    // Sanity check smoothing gain
    smoothing_gain = constrain_float(smoothing_gain,1.0f,50.0f);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

    		//# 1. 更新参考目标角度_att_target_euler_rad(ef)和参考目标角速度_att_target_euler_rate_rads(ef).
		//# (注意函数的输入参数:roll&pitch为目标角度, yaw为目标角速度.)
		//# roll&pitch:若有角加速度限幅和前馈使能,则使用sqrt_controller()计算参考目标角速度,然后更新参考目标角度(含前馈项[目标角速度*dt]);
		//# 		否则直接设函数输入参数为参考目标角度,参考目标角速度为0.
		//# yaw:	若有角加速度限幅,则对函数输入的角速度限幅,作为参考目标角速度;否则,直接设置函数输入量为参考目标角速度.
		//# 		然后更新参考目标角度(含前馈项). 
    if ((get_accel_roll_max_radss() > 0.0f) && _rate_bf_ff_enabled) {	//# 1a 用两拍参考目标角度之差计算参考目标角速度,然后用前馈量更新参考目标角度.
        // When roll acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler roll-axis
        // angular velocity that will cause the euler roll angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.		//# 1a.1 计算参考目标角速度_att_target_euler_rate_rads(ef).
        float euler_rate_desired_rads = sqrt_controller(euler_roll_angle_rad-_att_target_euler_rad.x, smoothing_gain, get_accel_roll_max_radss());

        // Acceleration is limited directly to smooth the beginning of the curve.	//# 1a.2 根据角加速度的限幅,对参考目标角速度(ef)限幅.
        float rate_change_limit_rads = get_accel_roll_max_radss() * _dt;
        _att_target_euler_rate_rads.x = constrain_float(euler_rate_desired_rads, _att_target_euler_rate_rads.x-rate_change_limit_rads, _att_target_euler_rate_rads.x+rate_change_limit_rads);

											//# 1a.3 参考目标角速度作为前馈项更新目标角度
        // The output rate is used to update the attitude target euler angles and is fed forward into the rate controller.
        update_att_target_roll(_att_target_euler_rate_rads.x, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX_RAD);
    } else {					//# 1b 直接用函数输入角度替换上一拍参考目标角度,参考目标角速度为0(i.e.无前馈项[目标角速度*dt]和限幅)
        // When acceleration limiting and feedforward are not enabled, the target roll euler angle is simply set to the
        // input value and the feedforward rate is zeroed.
        _att_target_euler_rad.x = euler_roll_angle_rad;	
        _att_target_euler_rate_rads.x = 0;
    }
    _att_target_euler_rad.x = constrain_float(_att_target_euler_rad.x, -get_tilt_limit_rad(), get_tilt_limit_rad());	//# 最大倾角限幅.

    if ((get_accel_pitch_max_radss() > 0.0f) && _rate_bf_ff_enabled) {
        // When pitch acceleration limiting and feedforward are enabled, the sqrt controller is used to compute an euler pitch-axis
        // angular velocity that will cause the euler pitch angle to smoothly stop at the input angle with limited deceleration
        // and an exponential decay specified by smoothing_gain at the end.
        float euler_rate_desired_rads = sqrt_controller(euler_pitch_angle_rad-_att_target_euler_rad.y, smoothing_gain, get_accel_pitch_max_radss());

        // Acceleration is limited directly to smooth the beginning of the curve.
        float rate_change_limit_rads = get_accel_pitch_max_radss() * _dt;
        _att_target_euler_rate_rads.y = constrain_float(euler_rate_desired_rads, _att_target_euler_rate_rads.y-rate_change_limit_rads, _att_target_euler_rate_rads.y+rate_change_limit_rads);

        // The output rate is used to update the attitude target euler angles and is fed forward into the rate controller.
        update_att_target_pitch(_att_target_euler_rate_rads.y, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX_RAD);
    } else {
        _att_target_euler_rad.y = euler_pitch_angle_rad;
        _att_target_euler_rate_rads.y = 0;
    }
    _att_target_euler_rad.y = constrain_float(_att_target_euler_rad.y, -get_tilt_limit_rad(), get_tilt_limit_rad());

							//# yaw:若有角加速度限幅,则对函数输入的角速度限幅,作为参考目标角速度;
    if (get_accel_yaw_max_radss() > 0.0f) {		//# 否则,直接设置函数输入量为参考目标角速度.	然后更新参考目标角度(含前馈项). 
        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;
        _att_target_euler_rate_rads.z += constrain_float(euler_yaw_rate_rads - _att_target_euler_rate_rads.z, -rate_change_limit_rads, rate_change_limit_rads);

        // The output rate is used to update the attitude target euler angles and is fed forward into the rate controller.
        update_att_target_yaw(_att_target_euler_rate_rads.z, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    } else {
        // When yaw acceleration limiting is disabled, the attitude target is simply rotated using the input rate and the input rate
        // is fed forward into the rate controller.
        _att_target_euler_rate_rads.z = euler_yaw_rate_rads;
        update_att_target_yaw(_att_target_euler_rate_rads.z, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    }

    		//# 2.参考目标角速度由惯性系ef(_att_target_euler_rate_rads)==>参考系rf(_att_target_ang_vel_rads).
		//# ef: phi_d',theta_d',psi_d' ==> rf: p_d,q_d,r_d.
    // Convert euler angle derivative of desired attitude into a body-frame angular velocity vector for feedforward
    if (_rate_bf_ff_enabled) {
        euler_rate_to_ang_vel(_att_target_euler_rad, _att_target_euler_rate_rads, _att_target_ang_vel_rads);
    } else {
        euler_rate_to_ang_vel(_att_target_euler_rad, Vector3f(0,0,_att_target_euler_rate_rads.z), _att_target_ang_vel_rads);
    }

    		//# 3.调用欧拉角姿态控制器.计算控制器目标角速度_ang_vel_target_rads(bf)(=角度误差*P + 前馈目标角速度rf->bf).
    // Call attitude controller
    attitude_controller_run_euler(_att_target_euler_rad, _att_target_ang_vel_rads);
}

			//# 姿态控制器 2. 计算控制器目标角速度_ang_vel_target_rads(bf).
void AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle_rad = radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle_rad = radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_rate_rads = radians(euler_yaw_rate_cds*0.01f);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

							//# 1. 更新参考目标角度_att_target_euler_rad(ef)和参考目标角速度_att_target_euler_rate_rads(ef).
    // Set roll/pitch attitude targets from input.	//# 1.1 roll&pitch: 直接设置函数输入量为参考目标角度
    _att_target_euler_rad.x = constrain_float(euler_roll_angle_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());
    _att_target_euler_rad.y = constrain_float(euler_pitch_angle_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());

    // Zero the roll and pitch feed-forward rate.	//# 参考目标角速度设为0.
    _att_target_euler_rate_rads.x = 0;
    _att_target_euler_rate_rads.y = 0;

							//# 1.2 yaw:若有角加速度限幅,则对函数输入的角速度限幅,作为参考目标角速度;
    if (get_accel_yaw_max_radss() > 0.0f) {		//#	否则,直接设置函数输入量为参考目标角速度.	然后更新参考目标角度(含前馈项). 
        // When yaw acceleration limiting is enabled, the yaw input shaper constrains angular acceleration about the yaw axis, slewing
        // the output rate towards the input rate.
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;
        _att_target_euler_rate_rads.z += constrain_float(euler_yaw_rate_rads - _att_target_euler_rate_rads.z, -rate_change_limit_rads, rate_change_limit_rads);

        // The output rate is used to update the attitude target euler angles and is fed forward into the rate controller.
        update_att_target_yaw(_att_target_euler_rate_rads.z, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    } else {
        // When yaw acceleration limiting is disabled, the attitude target is simply rotated using the input rate and the input rate
        // is fed forward into the rate controller.
        _att_target_euler_rate_rads.z = euler_yaw_rate_rads;
        update_att_target_yaw(_att_target_euler_rate_rads.z, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);
    }

    							//# 2.参考目标角速度由惯性系ef(_att_target_euler_rate_rads)==>参考系rf(_att_target_ang_vel_rads).
    // Convert euler angle derivatives of desired attitude into a body-frame angular velocity vector for feedforward
    euler_rate_to_ang_vel(_att_target_euler_rad, _att_target_euler_rate_rads, _att_target_ang_vel_rads);

    // Call attitude controller				//# 3.调用欧拉角姿态控制器.计算控制器目标角速度_ang_vel_target_rads(bf)
    attitude_controller_run_euler(_att_target_euler_rad, _att_target_ang_vel_rads);
}

			//# 姿态控制器 3. 
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_angle_rad = radians(euler_roll_angle_cd*0.01f);
    float euler_pitch_angle_rad = radians(euler_pitch_angle_cd*0.01f);
    float euler_yaw_angle_rad = radians(euler_yaw_angle_cd*0.01f);

    // Add roll trim to compensate tail rotor thrust in heli (will return zero on multirotors)
    euler_roll_angle_rad += get_roll_trim_rad();

    // Set attitude targets from input.			//# 1.以函数输入量作为参考目标角度_att_target_euler_rad(ef).
    _att_target_euler_rad.x = constrain_float(euler_roll_angle_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());
    _att_target_euler_rad.y = constrain_float(euler_pitch_angle_rad, -get_tilt_limit_rad(), get_tilt_limit_rad());
    _att_target_euler_rad.z = euler_yaw_angle_rad;

    // If slew_yaw is enabled, constrain yaw target within get_slew_yaw_rads() of _ahrs.yaw
    if (slew_yaw) {
        // Compute constrained angle error
        float angle_error = constrain_float(wrap_PI(_att_target_euler_rad.z - _ahrs.yaw), -get_slew_yaw_rads(), get_slew_yaw_rads());

        // Update attitude target from constrained angle error
        _att_target_euler_rad.z = angle_error + _ahrs.yaw;
    }

    // Call attitude controller				//# 2.调用欧拉角姿态控制器.计算控制器目标角速度_ang_vel_target_rads(bf)
    attitude_controller_run_euler(_att_target_euler_rad, Vector3f(0.0f,0.0f,0.0f));	//# 参考目标角速度设为0.

    // Keep euler derivative updated			//# 3.更新欧拉角速度.
    ang_vel_to_euler_rate(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _ang_vel_target_rads, _att_target_euler_rate_rads);
}

			//# 姿态控制器 4. 
void AC_AttitudeControl::input_euler_rate_roll_pitch_yaw(float euler_roll_rate_cds, float euler_pitch_rate_cds, float euler_yaw_rate_cds)
{
    // Convert from centidegrees on public interface to radians
    float euler_roll_rate_rads = radians(euler_roll_rate_cds*0.01f);
    float euler_pitch_rate_rads = radians(euler_pitch_rate_cds*0.01f);
    float euler_yaw_rate_rads = radians(euler_yaw_rate_cds*0.01f);

    // Compute acceleration-limited euler roll rate
    if (get_accel_roll_max_radss() > 0.0f) {					//# 1.更新参考目标角速度_att_target_euler_rate_rads(ef).
        float rate_change_limit_rads = get_accel_roll_max_radss() * _dt;
        _att_target_euler_rate_rads.x += constrain_float(euler_roll_rate_rads - _att_target_euler_rate_rads.x, -rate_change_limit_rads, rate_change_limit_rads);
    } else {
        _att_target_euler_rate_rads.x = euler_roll_rate_rads;
    }

    // Compute acceleration-limited euler pitch rate
    if (get_accel_pitch_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_pitch_max_radss() * _dt;
        _att_target_euler_rate_rads.y += constrain_float(euler_pitch_rate_rads - _att_target_euler_rate_rads.y, -rate_change_limit_rads, rate_change_limit_rads);
    } else {
        _att_target_euler_rate_rads.y = euler_pitch_rate_rads;
    }

    // Compute acceleration-limited euler yaw rate
    if (get_accel_yaw_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;
        _att_target_euler_rate_rads.z += constrain_float(euler_yaw_rate_rads - _att_target_euler_rate_rads.z, -rate_change_limit_rads, rate_change_limit_rads);
    } else {
        _att_target_euler_rate_rads.z = euler_yaw_rate_rads;
    }

    // Update the attitude target from the computed euler rates			//# 2.更新参考目标角度_att_target_euler_rad(ef).
    update_att_target_roll(_att_target_euler_rate_rads.x, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX_RAD);
    update_att_target_pitch(_att_target_euler_rate_rads.y, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX_RAD);
    update_att_target_yaw(_att_target_euler_rate_rads.z, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX_RAD);

    // Apply tilt limit
    _att_target_euler_rad.x = constrain_float(_att_target_euler_rad.x, -get_tilt_limit_rad(), get_tilt_limit_rad());
    _att_target_euler_rad.y = constrain_float(_att_target_euler_rad.y, -get_tilt_limit_rad(), get_tilt_limit_rad());

    							//# 3.参考目标角速度由惯性系ef(_att_target_euler_rate_rads)==>参考系rf(_att_target_ang_vel_rads).
    // Convert euler angle derivatives of desired attitude into a body-frame angular velocity vector for feedforward
    euler_rate_to_ang_vel(_att_target_euler_rad, _att_target_euler_rate_rads, _att_target_ang_vel_rads);

    // Call attitude controller				//# 4.调用欧拉角姿态控制器.计算控制器目标角速度_ang_vel_target_rads(bf)
    attitude_controller_run_euler(_att_target_euler_rad, _att_target_ang_vel_rads);
}

			//# 姿态控制器 5. 
void AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float yaw_rate_bf_cds)
{
    // Convert from centidegrees on public interface to radians
    float roll_rate_bf_rads = radians(roll_rate_bf_cds*0.01f);
    float pitch_rate_bf_rads = radians(pitch_rate_bf_cds*0.01f);
    float yaw_rate_bf_rads = radians(yaw_rate_bf_cds*0.01f);

											//# 1.更新参考目标角速度_att_target_ang_vel_rads(rf).
    // Compute acceleration-limited body-frame roll rate
    if (get_accel_roll_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_roll_max_radss() * _dt;
        _att_target_ang_vel_rads.x += constrain_float(roll_rate_bf_rads - _att_target_ang_vel_rads.x, -rate_change_limit_rads, rate_change_limit_rads);
    } else {
        _att_target_ang_vel_rads.x = roll_rate_bf_rads;
    }

    // Compute acceleration-limited body-frame pitch rate
    if (get_accel_pitch_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_pitch_max_radss() * _dt;
        _att_target_ang_vel_rads.y += constrain_float(pitch_rate_bf_rads - _att_target_ang_vel_rads.y, -rate_change_limit_rads, rate_change_limit_rads);
    } else {
        _att_target_ang_vel_rads.y = pitch_rate_bf_rads;
    }

    // Compute acceleration-limited body-frame yaw rate
    if (get_accel_yaw_max_radss() > 0.0f) {
        float rate_change_limit_rads = get_accel_yaw_max_radss() * _dt;
        _att_target_ang_vel_rads.z += constrain_float(yaw_rate_bf_rads - _att_target_ang_vel_rads.z, -rate_change_limit_rads, rate_change_limit_rads);
    } else {
        _att_target_ang_vel_rads.z = yaw_rate_bf_rads;
    }

    // Compute quaternion target attitude				//# 2.计算参考目标角度的四元数.
    Quaternion att_target_quat;
    att_target_quat.from_euler(_att_target_euler_rad.x,_att_target_euler_rad.y,_att_target_euler_rad.z);

    // Rotate quaternion target attitude using computed rate		//# 参考目标角度=前一拍的参考目标角度+函数输入量(角速度)*dt.
    att_target_quat.rotate(_att_target_ang_vel_rads*_dt);
    att_target_quat.normalize();

    // Call attitude controller						//# 3.调用四元数姿态控制器,计算控制器目标角速度_ang_vel_target_rads(bf).
    attitude_controller_run_quat(att_target_quat, _att_target_ang_vel_rads);

    // Keep euler derivative updated
    ang_vel_to_euler_rate(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _ang_vel_target_rads, _att_target_euler_rate_rads);
}

			//# 姿态控制器 6. 
void AC_AttitudeControl::input_att_quat_bf_ang_vel(const Quaternion& att_target_quat, const Vector3f& att_target_ang_vel_rads)
{
    // Call attitude controller
    attitude_controller_run_quat(att_target_quat, att_target_ang_vel_rads);

    // Keep euler derivative updated						//# 更新_att_target_euler_rate_rads(rf).
    ang_vel_to_euler_rate(Vector3f(_ahrs.roll,_ahrs.pitch,_ahrs.yaw), _ang_vel_target_rads, _att_target_euler_rate_rads);
}

			//# 欧拉角姿态控制器: 计算控制器目标角速度_ang_vel_target_rads(bf).
			//# IN: 参考目标角度_att_target_euler_rad(ef)和参考目标角速度_att_target_ang_vel_rads(rf);
			//# 参考目标角度_att_target_euler_rad(ef)转换为四元数,再调用四元数的姿态控制器.
void AC_AttitudeControl::attitude_controller_run_euler(const Vector3f& att_target_euler_rad, const Vector3f& att_target_ang_vel_rads)
{
    // Compute quaternion target attitude
    Quaternion att_target_quat;
    att_target_quat.from_euler(att_target_euler_rad.x, att_target_euler_rad.y, att_target_euler_rad.z);

    // Call quaternion attitude controller
    attitude_controller_run_quat(att_target_quat, att_target_ang_vel_rads);
}

			//# 四元数姿态控制器: 计算控制器目标角速度_ang_vel_target_rads(bf).
			//# IN: 参考目标角度的四元数(ef)和参考目标角速度向量(rf). OUT:控制器目标角速度_angle_vel_target_rads(bf).
			//# 1.角度误差_att_error_rot_vec_rad -> sqrt_controller()/P -> 控制器目标角速度_angle_vec_target_rads(bf).
			//# 2.+前馈项：参考目标角速度_att_target_ang_vel_rads(rf->bf).
void AC_AttitudeControl::attitude_controller_run_quat(const Quaternion& att_target_quat, const Vector3f& att_target_ang_vel_rads)
{
    // Update euler attitude target and angular velocity target
    att_target_quat.to_euler(_att_target_euler_rad.x,_att_target_euler_rad.y,_att_target_euler_rad.z);//# 把目标角度的四元数转回欧拉角,写入类成员变量
    _att_target_ang_vel_rads = att_target_ang_vel_rads;

    // Retrieve quaternion vehicle attitude
    // TODO add _ahrs.get_quaternion()
    Quaternion att_vehicle_quat;
    att_vehicle_quat.from_rotation_matrix(_ahrs.get_rotation_body_to_ned());	//# 获得表示机体姿态的四元数(ef).
    			//# bf->ef的旋转矩阵转换为四元数(i.e.机体实际欧拉角的四元数).
    			//### 关于方向:
			//# 考虑二维情况,设机体相对地面有一顺时针角度,机体系到地面系的旋转矩阵将平面内某一位置在机体系中的坐标转换到地面系中的坐标.
			//# 注意到这一位置在地面系中的坐标与其在机体系中的坐标相比,也有一顺时针角度,
			//# 从而说明: 机体系到地面系的旋转矩阵可以描述机体相对于地面的角度,即其姿态.
			//### 结论: a->b的旋转矩阵可表示a相对于b的姿态!!

    // Compute attitude error
    (att_vehicle_quat.inverse()*att_target_quat).to_axis_angle(_att_error_rot_vec_rad);	//# 0.计算角度误差,存入_att_error_rot_vec_rad(bf).

    // Compute the angular velocity target from the attitude error
    update_ang_vel_target_from_att_error();		//# 1.角度误差_att_error_rot_vec_rad(bf) ==> 控制器目标角速度_ang_vel_target_rads(bf)

    // Add the angular velocity feedforward, rotated into vehicle frame
    Matrix3f Trv;
    get_rotation_reference_to_vehicle(Trv);
    _ang_vel_target_rads += Trv * _att_target_ang_vel_rads;//# 2.参考目标角速度(_att_target_ang_vel_rads)rf->bf,作为前馈项并入控制器目标角速度(bf).
}

void AC_AttitudeControl::rate_controller_run()	//# 控制器目标角速度->PID->电机类的角度控制量_XX_control_input.
{						//# _XX_control_input: desired roll/pitch/yaw control from attitude controllers, +/- 4500
    _motors.set_roll(rate_bf_to_motor_roll(_ang_vel_target_rads.x));	//# 1.rate_bf_to_motor_XX(): rate error -> PID -> motor angle(XX_in).
    _motors.set_pitch(rate_bf_to_motor_pitch(_ang_vel_target_rads.y));	//# 2.set_XX(): 	     _XX_control_input = XX_in;
    _motors.set_yaw(rate_bf_to_motor_yaw(_ang_vel_target_rads.z));
    control_monitor_update();
}

				//# 运动方程组	ef:dphi,dtheta,dpsi ==> bf:p,q,r.(参数1,2作为输入,3作为输出)
void AC_AttitudeControl::euler_rate_to_ang_vel(const Vector3f& euler_rad, const Vector3f& euler_rate_rads, Vector3f& ang_vel_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    ang_vel_rads.x = euler_rate_rads.x - sin_theta * euler_rate_rads.z;
    ang_vel_rads.y = cos_phi  * euler_rate_rads.y + sin_phi * cos_theta * euler_rate_rads.z;
    ang_vel_rads.z = -sin_phi * euler_rate_rads.y + cos_theta * cos_phi * euler_rate_rads.z;
}

				//# 运动方程组  ang_vel: p,q,r. ==> euler_rate: derivatives of phi, theta, and psi.
bool AC_AttitudeControl::ang_vel_to_euler_rate(const Vector3f& euler_rad, const Vector3f& ang_vel_rads, Vector3f& euler_rate_rads)
{
    float sin_theta = sinf(euler_rad.y);
    float cos_theta = cosf(euler_rad.y);
    float sin_phi = sinf(euler_rad.x);
    float cos_phi = cosf(euler_rad.x);

    // When the vehicle pitches all the way up or all the way down, the euler angles become discontinuous. In this case, we just return false.
    if (is_zero(cos_theta)) {
        return false;
    }

    euler_rate_rads.x = ang_vel_rads.x + sin_phi * (sin_theta/cos_theta) * ang_vel_rads.y + cos_phi * (sin_theta/cos_theta) * ang_vel_rads.z;
    euler_rate_rads.y = cos_phi  * ang_vel_rads.y - sin_phi * ang_vel_rads.z;
    euler_rate_rads.z = (sin_phi / cos_theta) * ang_vel_rads.y + (cos_phi / cos_theta) * ang_vel_rads.z;
    return true;
}

				//# 参考目标角度 = 实际角度 + 限幅后的角度误差 + 参考目标角速度*dt.
void AC_AttitudeControl::update_att_target_roll(float euler_roll_rate_rads, float overshoot_max_rad)
{
    // Compute constrained angle error
    float angle_error = constrain_float(wrap_PI(_att_target_euler_rad.x - _ahrs.roll), -overshoot_max_rad, overshoot_max_rad);

    // Update attitude target from constrained angle error
    _att_target_euler_rad.x = angle_error + _ahrs.roll;

    // Increment the attitude target
    _att_target_euler_rad.x += euler_roll_rate_rads * _dt;
    _att_target_euler_rad.x = wrap_PI(_att_target_euler_rad.x);
}

				//# 参考目标角度 = 实际角度 + 限幅后的角度误差 + 参考目标角速度*dt.
void AC_AttitudeControl::update_att_target_pitch(float euler_pitch_rate_rads, float overshoot_max_rad)
{
    // Compute constrained angle error
    float angle_error = constrain_float(wrap_PI(_att_target_euler_rad.y - _ahrs.pitch), -overshoot_max_rad, overshoot_max_rad);

    // Update attitude target from constrained angle error
    _att_target_euler_rad.y = angle_error + _ahrs.pitch;

    // Increment the attitude target
    _att_target_euler_rad.y += euler_pitch_rate_rads * _dt;
    _att_target_euler_rad.y = wrap_PI(_att_target_euler_rad.y);
}

				//# 参考目标角度 = 实际角度 + 限幅后的角度误差 + 参考目标角速度*dt.
void AC_AttitudeControl::update_att_target_yaw(float euler_yaw_rate_rads, float overshoot_max_rad)
{
    // Compute constrained angle error
    float angle_error = constrain_float(wrap_PI(_att_target_euler_rad.z - _ahrs.yaw), -overshoot_max_rad, overshoot_max_rad);

    // Update attitude target from constrained angle error
    _att_target_euler_rad.z = angle_error + _ahrs.yaw;

    // Increment the attitude target
    _att_target_euler_rad.z += euler_yaw_rate_rads * _dt;
    _att_target_euler_rad.z = wrap_2PI(_att_target_euler_rad.z);
}

void AC_AttitudeControl::integrate_bf_rate_error_to_angle_errors()	//# only called by Heli. IGNORE.
{
    // Integrate the angular velocity error into the attitude error
    _att_error_rot_vec_rad += (_att_target_ang_vel_rads - _ahrs.get_gyro()) * _dt;

    // Constrain attitude error
    _att_error_rot_vec_rad.x = constrain_float(_att_error_rot_vec_rad.x, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);
    _att_error_rot_vec_rad.y = constrain_float(_att_error_rot_vec_rad.y, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);
    _att_error_rot_vec_rad.z = constrain_float(_att_error_rot_vec_rad.z, -AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD, AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX_RAD);
}

			//# 角度误差_att_error_rot_vec_rad(bf) ==> 目标角速度_ang_vel_target_rads(bf)
void AC_AttitudeControl::update_ang_vel_target_from_att_error()	//# 角加速度限幅时用sqrt_controller,否则用P控制.	前馈????
{
    // Compute the roll angular velocity demand from the roll angle error
    if (_att_ctrl_use_accel_limit && _accel_roll_max > 0.0f) {
        _ang_vel_target_rads.x = sqrt_controller(_att_error_rot_vec_rad.x, _p_angle_roll.kP(), constrain_float(get_accel_roll_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS));
    }else{
        _ang_vel_target_rads.x = _p_angle_roll.kP() * _att_error_rot_vec_rad.x;
    }

    // Compute the pitch angular velocity demand from the roll angle error
    if (_att_ctrl_use_accel_limit && _accel_pitch_max > 0.0f) {
        _ang_vel_target_rads.y = sqrt_controller(_att_error_rot_vec_rad.y, _p_angle_pitch.kP(), constrain_float(get_accel_pitch_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_RP_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_RP_CONTROLLER_MAX_RADSS));
    }else{
        _ang_vel_target_rads.y = _p_angle_pitch.kP() * _att_error_rot_vec_rad.y;
    }

    // Compute the yaw angular velocity demand from the roll angle error
    if (_att_ctrl_use_accel_limit && _accel_yaw_max > 0.0f) {
        _ang_vel_target_rads.z = sqrt_controller(_att_error_rot_vec_rad.z, _p_angle_yaw.kP(), constrain_float(get_accel_yaw_max_radss()/2.0f,  AC_ATTITUDE_ACCEL_Y_CONTROLLER_MIN_RADSS, AC_ATTITUDE_ACCEL_Y_CONTROLLER_MAX_RADSS));
    }else{
        _ang_vel_target_rads.z = _p_angle_yaw.kP() * _att_error_rot_vec_rad.z;
    }

    // Add feedforward term that attempts to ensure that the copter yaws about the reference		//# ?????
    // Z axis, rather than the vehicle body Z axis.
    // NOTE: This is a small-angle approximation.
    _ang_vel_target_rads.x += _att_error_rot_vec_rad.y * _ahrs.get_gyro().z;
    _ang_vel_target_rads.y += -_att_error_rot_vec_rad.x * _ahrs.get_gyro().z;
}

float AC_AttitudeControl::rate_bf_to_motor_roll(float rate_target_rads)		//# rate error -> PID -> motor roll控制量.(I with preconditions.)
{
    float current_rate_rads = _ahrs.get_gyro().x;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    // pass error to PID controller
    get_rate_roll_pid().set_input_filter_d(rate_error_rads);
    get_rate_roll_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_roll_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.roll_pitch || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
        integrator = get_rate_roll_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_roll_pid().get_p() + integrator + get_rate_roll_pid().get_d();

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

float AC_AttitudeControl::rate_bf_to_motor_pitch(float rate_target_rads)	//# rate error -> PID -> motor pitch控制量.(I with preconditions.)
{
    float current_rate_rads = _ahrs.get_gyro().y;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    // pass error to PID controller
    get_rate_pitch_pid().set_input_filter_d(rate_error_rads);
    get_rate_pitch_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_pitch_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.roll_pitch || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
        integrator = get_rate_pitch_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_pitch_pid().get_p() + integrator + get_rate_pitch_pid().get_d();

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

float AC_AttitudeControl::rate_bf_to_motor_yaw(float rate_target_rads)		//# rate error -> PID -> motor yaw控制量.(I with preconditions.)
{
    float current_rate_rads = _ahrs.get_gyro().z;
    float rate_error_rads = rate_target_rads - current_rate_rads;

    // pass error to PID controller
    get_rate_yaw_pid().set_input_filter_all(rate_error_rads);
    get_rate_yaw_pid().set_desired_rate(rate_target_rads);

    float integrator = get_rate_yaw_pid().get_integrator();

    // Ensure that integrator can only be reduced if the output is saturated
    if (!_motors.limit.yaw || ((integrator > 0 && rate_error_rads < 0) || (integrator < 0 && rate_error_rads > 0))) {
        integrator = get_rate_yaw_pid().get_i();
    }

    // Compute output in range -1 ~ +1
    float output = get_rate_yaw_pid().get_p() + integrator + get_rate_yaw_pid().get_d();

    // Constrain output
    return constrain_float(output, -1.0f, 1.0f);
}

void AC_AttitudeControl::accel_limiting(bool enable_limits)	//# only called by do_aux_switch_function()(invoked by the ch7 or ch8 switch).
{
    if (enable_limits) {
        // If enabling limits, reload from eeprom or set to defaults
        if (is_zero(_accel_roll_max)) {
            _accel_roll_max.load();
        }
        if (is_zero(_accel_pitch_max)) {
            _accel_pitch_max.load();
        }
        if (is_zero(_accel_yaw_max)) {
            _accel_yaw_max.load();
        }
    } else {
        _accel_roll_max = 0.0f;
        _accel_pitch_max = 0.0f;
        _accel_yaw_max = 0.0f;
    }
}

void AC_AttitudeControl::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    _throttle_in_filt.apply(throttle_in, _dt);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (apply_angle_boost) {
        _motors.set_throttle(get_boosted_throttle(throttle_in));
    }else{
        _motors.set_throttle(throttle_in);
        // Clear angle_boost for logging purposes
        _angle_boost = 0.0f;
    }
}

void AC_AttitudeControl::set_throttle_out_unstabilized(float throttle_in, bool reset_attitude_control, float filter_cutoff)	//#
{
    _throttle_in = throttle_in;
    _throttle_in_filt.apply(throttle_in, _dt);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    if (reset_attitude_control) {
        relax_bf_rate_controller();
        set_yaw_target_to_current_heading();
    }
    _motors.set_throttle(throttle_in);
    _angle_boost = 0.0f;
}

			//# constrain second derivative???
float AC_AttitudeControl::sqrt_controller(float error, float p, float second_ord_lim)	//# 分段函数,linear_dist范围内为比例控制,超出范围比例系数逐渐变小
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return error*p;
    }

    float linear_dist = second_ord_lim/sq(p);

    if (error > linear_dist) {
        return safe_sqrt(2.0f*second_ord_lim*(error-(linear_dist/2.0f)));
    } else if (error < -linear_dist) {
        return -safe_sqrt(2.0f*second_ord_lim*(-error-(linear_dist/2.0f)));
    } else {
        return error*p;
    }
}

void AC_AttitudeControl::get_rotation_vehicle_to_ned(Matrix3f& m)	//# bf->ef rotation matrix.
{
    m = _ahrs.get_rotation_body_to_ned();
}

void AC_AttitudeControl::get_rotation_ned_to_vehicle(Matrix3f& m)	//#
{
    get_rotation_vehicle_to_ned(m);
    m = m.transposed();
}

					//# _att_target_euler_rad ==> (ref->ef rotation matrix). (可表示目标角度相对地面系的姿态!)
void AC_AttitudeControl::get_rotation_reference_to_ned(Matrix3f& m)
{
    m.from_euler(_att_target_euler_rad.x,_att_target_euler_rad.y,_att_target_euler_rad.z);
}

void AC_AttitudeControl::get_rotation_ned_to_reference(Matrix3f& m)	//#
{
    get_rotation_reference_to_ned(m);
    m = m.transposed();
}

void AC_AttitudeControl::get_rotation_vehicle_to_reference(Matrix3f& m)	//# vehicle->ned->ref
{
    Matrix3f Tvn;
    Matrix3f Tnr;
    get_rotation_vehicle_to_ned(Tvn);
    get_rotation_ned_to_reference(Tnr);
    m = Tnr * Tvn;
}

void AC_AttitudeControl::get_rotation_reference_to_vehicle(Matrix3f& m)	//#
{
    get_rotation_vehicle_to_reference(m);
    m = m.transposed();
}

float AC_AttitudeControl::max_rate_step_bf_roll()
{
    float alpha = get_rate_roll_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_roll_pid().kD())/_dt + get_rate_roll_pid().kP());
}

float AC_AttitudeControl::max_rate_step_bf_pitch()
{
    float alpha = get_rate_pitch_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_pitch_pid().kD())/_dt + get_rate_pitch_pid().kP());
}

float AC_AttitudeControl::max_rate_step_bf_yaw()
{
    float alpha = get_rate_yaw_pid().get_filt_alpha();
    float alpha_remaining = 1-alpha;
    return AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX/((alpha_remaining*alpha_remaining*alpha_remaining*alpha*get_rate_yaw_pid().kD())/_dt + get_rate_yaw_pid().kP());
}
