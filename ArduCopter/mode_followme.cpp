#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool ModeFollowMe::init(bool ignore_checks)
{
    // start in angle control mode
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeFollowMe::run()
{
    Quaternion attitude_quat;     //Quaternion(0.319,0.648,0.243,0.648);
    attitude_quat.from_euler(0,0.5,0);

        // check if the message's thrust field should be interpreted as a climb rate or as thrust
        const bool use_thrust = set_attitude_target_provides_thrust();

        float climb_rate_or_thrust;
        float thrust = 0.5;
        if (use_thrust) {
            // interpret thrust as thrust
            climb_rate_or_thrust = constrain_float(thrust, -1.0f, 1.0f);
        } else {
            // convert thrust to climb rate
            thrust = constrain_float(thrust, 0.0f, 1.0f);
            if (is_equal(thrust, 0.5f)) {
                climb_rate_or_thrust = 0.0f;
            } else if (thrust > 0.5f) {
                // climb at up to WPNAV_SPEED_UP
                climb_rate_or_thrust = (thrust - 0.5f) * 2.0f * wp_nav->get_default_speed_up();
            } else {
                // descend at up to WPNAV_SPEED_DN
                climb_rate_or_thrust = (0.5f - thrust) * 2.0f * -wp_nav->get_default_speed_down();
            }
        }

    Vector3f ang_vel;
    //set target
    set_angle(attitude_quat, ang_vel,
                climb_rate_or_thrust, use_thrust);

    // run angle controller
    ModeGuided::angle_control_run();
}

#endif
