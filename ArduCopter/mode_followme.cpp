#include "Copter.h"

#if MODE_FOLLOWME_ENABLED == ENABLED

const AP_Param::GroupInfo ModeFollowMe::var_info[] = {

    // @Param: HFOV
    // @DisplayName: Horizontal camera FOV
    // @Description: Horizontal camera FOV
    // @Range: 0 180
    AP_GROUPINFO("HFOV", 1, ModeFollowMe, hFOV, 120.0f),

    // @Param: VFOV
    // @DisplayName: Vertical camera FOV
    // @Description: Horizontal camera FOV
    // @Range: 0 180
    AP_GROUPINFO("VFOV", 2, ModeFollowMe, vFOV, 90.0f),

    // @Param: FLEN
    // @DisplayName: lenght of vector f
    // @Description: lenght of vector f
    // @Range: 0 0.5
    AP_GROUPINFO("FLEN", 3, ModeFollowMe, F, 0.125f),

    // @Param: FLEN
    // @DisplayName: compensation side speed
    // @Description: compensation side speed
    // @Range: 0 3.0
    AP_GROUPINFO("COMPY", 4, ModeFollowMe, compYspeed, 1.5f),

    // @Param: FLEN
    // @DisplayName: proportional gain vertical velocity
    // @Description: proportional gain vertical velocity
    // @Range: 0 20.0
    AP_GROUPINFO("THRP", 5, ModeFollowMe, thrustP, 10.0f),

    AP_GROUPEND
};

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool ModeFollowMe::init(bool ignore_checks)
{
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s%.4f%s%.4f%s%.4f%s%.4f%s%.4f", "HFOV: ",(float)hFOV," VFOV: ",(float)vFOV," FLEN: ",(float)F," COMPY: ",(float)compYspeed," THRP: ",(float)thrustP);

    // init LPF for vertical speed control
    AngleCapture = false;
    targetAngleFilt.set_cutoff_frequency(0.5);
    PsiAngleFilt.set_cutoff_frequency(5.0);
    // start in angle control mode
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeFollowMe::run()
{
    Quaternion attitude_quat;           // Quaternion(0.319,0.648,0.243,0.648);
    Vector3F targetV;   // vetctor target norm

    AP_OSD *osdobj = AP::osd();
    WITH_SEMAPHORE(ahrs.get_semaphore());

    float teta = 0,psi = 0;

    // vector power
    float Fteta = 0,Fphi = 0;
    if (osdobj->status_followme)
    {
        psi = (DEG_TO_RAD*hFOV / 2) * osdobj->x_followme;
        teta = (DEG_TO_RAD*vFOV/2)*osdobj->y_followme;

        // filt psi
        PsiAngleFilt.apply(psi);
        psi = PsiAngleFilt.get();

        // calc target vector body frame
        targetV.x = cosF(psi);
        targetV.y = sinF(psi);
        targetV.z = cosF(teta);

        // calc F
        Fteta = -asinF(F*targetV.x);
        Fphi = asinF(F*targetV.y) + compYspeed*ahrs.get_gyro().z;

        // vertical control
        if(!AngleCapture)
        {
            
            AngleCapture = true;
            targetAngleSetpoint = ahrs.get_pitch() - teta;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s%.4f", "Angle to target capture: ",targetAngleSetpoint);
        }
        targetAngleFilt.apply(ahrs.get_pitch() - teta);
        targetAngleFiltValue = targetAngleFilt.get();
        thrust = 0.5 + thrustP*(targetAngleFiltValue - targetAngleSetpoint);
    }else
    {
        Fteta = ahrs.get_pitch();
        Fphi = ahrs.get_roll();
        psi = 0;

        targetAngleFilt.reset();
        PsiAngleFilt.reset();
        AngleCapture = false;
        thrust = 0.5;
    }

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

    // LOG
#if HAL_LOGGING_ENABLED
    // log at 10hz or if stage changes
    uint32_t now = AP_HAL::millis();
    if ((now - last_log_ms) > 100) {
        last_log_ms = now;

// @LoggerMessage: FLME
// @Description: FOLLOWME Mode messages
// @Field: TimeUS: Time since system startup
// @Field: targetAngleFiltValue: Current angle between target and horizont
// @Field: targetAngleSetpoint:  Setpoint angle between target and horizont
// @Field: thrust: Setpoint thrust (vertical velocity)
// @Field: psi: yaw align between target and vehicle
// @Field: teta: pitch align between target and vehicle

        AP::logger().WriteStreaming(
            "FLME",
            "TimeUS,AngT,AngS,ThrS,Psi,Teta,x,y",
            "Qfffffff",
            AP_HAL::micros64(),
            (double)RAD_TO_DEG*targetAngleFiltValue,
            (double)RAD_TO_DEG*targetAngleSetpoint,
            (double)thrust,
            (double)RAD_TO_DEG*psi,
            (double)RAD_TO_DEG*teta,
            (double)osdobj->x_followme,
            (double)osdobj->y_followme
        );
    }
#endif  // HAL_LOGGING_ENABLED
}

#endif
