#include "Sub.h"

// manual_pro_init - initialise manual controller
bool Sub::manual_pro_init()
{
    // set target altitude to zero for reporting
    pos_control.set_pos_target_z_cm(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    return true;
}


// manual_pro_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::manual_pro_run()
{
    float target_roll, target_pitch;
    // GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Sonar range: %.2f", sonar_range);

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }


    // check if current altitude is below the avoidance_meters parameter
    if (g.cur_avoidance_m < g.max_avoidance_m) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Current avoidance meters for colision: %.2f", static_cast<double>(g.cur_avoidance_m));
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Max avoidance meters for  colision: %.2f", static_cast<double>(g.max_avoidance_m));


        get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);
    
        // get pilot's desired yaw rate
        float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // set target climb rate
        float cmb_rate = constrain_float(fabsf(wp_nav.get_default_speed_up()), 1, pos_control.get_max_speed_up_cms());

        // record desired climb rate for logging
        desired_climb_rate = cmb_rate;

        // update altitude target and call position controller
        pos_control.set_pos_target_z_from_climb_rate_cm(cmb_rate);
        pos_control.update_z_controller();

        // pilot has control for repositioning
        motors.set_forward(channel_forward->norm_input());
        motors.set_lateral(channel_lateral->norm_input());
    } else {


        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "SAFE");
        // if above, allow manual control
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        motors.set_roll(channel_roll->norm_input());
        motors.set_pitch(channel_pitch->norm_input());
        motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
        motors.set_throttle(channel_throttle->norm_input());
        motors.set_forward(channel_forward->norm_input());
        motors.set_lateral(channel_lateral->norm_input());
    }



}
