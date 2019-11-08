#include "mode.h"
#include "Plane.h"

static uint32_t timer;
bool mission = false;

bool ModeDBFCDA::_enter()
{
    plane.throttle_allows_nudging = false; //changed
    plane.auto_throttle_mode = false;  //changed
    plane.auto_navigation_mode = false; //changed! - manual
    plane.auto_state.vtol_mode = false;

    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }
    timer = AP_HAL::micros();
    
    gcs().send_text(MAV_SEVERITY_INFO, "enter in mode_dbfcda.cpp");
    return true;
}

void ModeDBFCDA::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) //error here?
        {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "Exiting mode_dbfcda.cpp ??");
}

void ModeDBFCDA::update()
{
    // Will be triggered during mode switch from auto to dbfcda
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) { //understand this! - this is true

        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        //plane.set_mode(plane.mode_rtl, MODE_REASON_MISSION_END);
        //gcs().send_text(MAV_SEVERITY_INFO, "Aircraft has no waypoint...");
        //return;
    }

    //
    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

    if (nav_cmd_id == MAV_CMD_NAV_LAND) { //could potentially trigger this loop after detachment!
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        } else {
            plane.calc_throttle();
        }

    } else if(nav_cmd_id == MAV_CMD_NAV_WAYPOINT) {


        //----Input Variables!----
        // - Altitude, Velocity, Position, Pitch, Roll, Yaw
        //Better way to do this is to just index variables?

        //TECS Controller: Attitude.cpp::calc_nav_pitch, AP_TECS.h - NOT USEFUL - Want to see servo output
        int32_t m_commanded_pitch = plane.SpdHgt_Controller->get_pitch_demand(); //pitch angle demand: -9000 to +9000 rad
        //L1 Controller: Attitude.cpp::calc_nav_roll, AP_L1_Control.cpp
        int32_t m_commanded_roll = plane.nav_controller->nav_roll_cd(); //bank angle needed to achieve tracking from the last update_*() 
        
        float measured_pitch = plane.ahrs.get_pitch();
        float measured_roll = plane.ahrs.get_roll();
        //if (plane.ahrs.have_inertial_nav()) {
        Vector2f measured_vel = plane.ahrs.groundspeed_vector();
        //derivative of verticle position? plane.ahrs.get_vert_pos_rate
        //}
        Vector3f measured_GPS_vel = plane.gps.velocity();
        float measured_baro = plane.barometer.get_altitude(); //altitude relative to last calibrate() call
        //GPS data read
        Location measured_gps = plane.gps.location();
        int32_t measured_lat = measured_gps.lat;
        int32_t measured_long = measured_gps.lng;
        int32_t measured_alt = measured_gps.alt;


        //----Calculate Trajectory----
        //GPS - 
        float time_land = measured_baro/measured_GPS_vel.z;
        float xpo_land = measured_gps.lng + measured_vel.x*time_land;
        
        
        //----Output to mavlink----
        if ((AP_HAL::micros() - timer) > 2000 * 1000UL) { //run every .5 Hz?
            timer = AP_HAL::micros();
            //Add timer based update message to MP
            gcs().send_text(MAV_SEVERITY_INFO, "Pitch (Measured) : %f", measured_pitch); //Values: .08 - .1 = climb
            gcs().send_text(MAV_SEVERITY_INFO, "Roll (Measured) : %f", measured_roll); //Values: -.6 = bank left
            gcs().send_text(MAV_SEVERITY_INFO, "Velocity X : %f", measured_vel.x); //Values: m/s (+ = north) around 21
            gcs().send_text(MAV_SEVERITY_INFO, "Velocity Y : %f", measured_vel.y); //Values: m/s (+ = east)
            gcs().send_text(MAV_SEVERITY_INFO, "Velocity X (GPS) : %f", measured_GPS_vel.x); //Values: m/s (+ = north) around 21
            gcs().send_text(MAV_SEVERITY_INFO, "Velocity Y (GPS) : %f", measured_GPS_vel.y); //Values: m/s (+ = east)
            gcs().send_text(MAV_SEVERITY_INFO, "Velocity Z (GPS) : %f", measured_GPS_vel.z); //Values: m/s (+ = down)
            gcs().send_text(MAV_SEVERITY_INFO, "Position (lat) : %i", measured_lat); //Values: 334295519
            gcs().send_text(MAV_SEVERITY_INFO, "Position (long) : %i", measured_long); //Values: -841683625
            gcs().send_text(MAV_SEVERITY_INFO, "Altitude (baro) : %f", measured_baro); //Values: m around 148.1213
            gcs().send_text(MAV_SEVERITY_INFO, "Altitude (gps) : %i", measured_alt); //Values: 14098
            gcs().send_text(MAV_SEVERITY_INFO, "Commanded Pitch : %i", m_commanded_pitch); //Values:
            gcs().send_text(MAV_SEVERITY_INFO, "Commanded Roll : %i", m_commanded_roll); //Values:
        }

        if (measured_baro > 100.0f) {
            mission = true;
        }
        //insert else if to restart mission (climb again - plane.calc_...) mid test?

        if (mission) {
            //plane.nav_pitch_cd = -8000;
            plane.nav_roll_cd = constrain_int32(0, -plane.roll_limit_cd, plane.roll_limit_cd);  // works!
            plane.nav_pitch_cd = constrain_int32(5000, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get()); // works
            
            //Read stabilize_roll, stabilize_pitch from Attitude.cpp

            //plane.suppress_throttle = true; //-build error
            //SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);

            //SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, -3000); //range -4500 to 4500 - don't work
            //SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 50); //range -4500 to 4500 - don't work
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        }
        else {
            plane.calc_nav_roll();
            plane.calc_nav_pitch();
            plane.calc_throttle();
        }

        //Add tuning variable acess in MP
        
    } else { //could potentially use this loop to do nothing while attached to mothership!
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        //plane.calc_throttle();
        gcs().send_text(MAV_SEVERITY_INFO, "inside else loop? - no waypoint");
    }
    //gcs().send_text(MAV_SEVERITY_INFO, "update in mode_dbfcda.cpp");
}

