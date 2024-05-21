#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool ModeFollowMe::init(bool ignore_checks)
{
    // init LPF for vertical speed control
    AngleCapture = false;
    targetAngleFilt.set_cutoff_frequency(5.0);
    // start in angle control mode
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more M_PI
void ModeFollowMe::run()
{
    Quaternion attitude_quat;           // Quaternion(0.319,0.648,0.243,0.648);
    float alphaFOV = DEG_TO_RAD * 93.0; // TODO make params
    float betaFOV = DEG_TO_RAD*93.0;

    Vector3F targetV;   // vetctor target norm
    float F = 0.2;    // lenght of vector f
    float compYspeed = 1.5;
    float thrust = 0.5;

    AP_OSD *osdobj = AP::osd();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    float phi,teta,psi;

    // vector power
    float Fteta,Fphi;
    if (osdobj->status_followme)
    {
        psi = (alphaFOV / 2) * osdobj->x_followme;
        teta = (betaFOV/2)*osdobj->y_followme;

        // calc target vector body frame
        targetV.x = cosF(psi);
        targetV.y = sinF(psi);
        targetV.z = cosF(teta);

        //targetV.rotate_xy(ahrs.get_yaw());

        // calc F
        Fteta = -asinF(F*targetV.x);
        Fphi = asinF(F*targetV.y) + compYspeed*ahrs.get_gyro().z;

        // vertical control
        if(!AngleCapture)
        {
            
            AngleCapture = true;
            targetAngleSetpoint = ahrs.get_pitch() - teta;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s%.4f", "Angle to target capture: ",targetAngleSetpoint);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s%.4f", "Teta: ",-teta);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s%.4f", "Pitch: ",ahrs.get_pitch());
        }
        targetAngleFilt.apply(ahrs.get_pitch() - teta);
        targetAngleFiltValue = targetAngleFilt.get();
        thrust = 0.5 + 15.0*(targetAngleFiltValue - targetAngleSetpoint);
    }else
    {
        Fteta = ahrs.get_pitch();
        Fphi = ahrs.get_roll();
        psi = 0;

        targetAngleFilt.reset();
        AngleCapture = false;
        thrust = 0.5;
    }

    (void)phi;
    (void)teta;

    attitude_quat.from_euler(Fphi, Fteta, ahrs.get_yaw()+psi);

    // check if the message's thrust field should be interpreted as a climb rate or as thrust
    const bool use_thrust = set_attitude_target_provides_thrust();

    float climb_rate_or_thrust;
    if (use_thrust)
    {
        // interpret thrust as thrust
        climb_rate_or_thrust = constrain_float(thrust, -1.0f, 1.0f);
    }
    else
    {
        // convert thrust to climb rate
        thrust = constrain_float(thrust, 0.0f, 1.0f);
        if (is_equal(thrust, 0.5f))
        {
            climb_rate_or_thrust = 0.0f;
        }
        else if (thrust > 0.5f)
        {
            // climb at up to WPNAV_SPEED_UP
            climb_rate_or_thrust = (thrust - 0.5f) * 2.0f * wp_nav->get_default_speed_up();
        }
        else
        {
            // descend at up to WPNAV_SPEED_DN
            climb_rate_or_thrust = (0.5f - thrust) * 2.0f * -wp_nav->get_default_speed_down();
        }
    }

    Vector3f ang_vel;
    // set target
    set_angle(attitude_quat, ang_vel,
              climb_rate_or_thrust, use_thrust);

    // run angle controller
    ModeGuided::angle_control_run();
}

#endif
