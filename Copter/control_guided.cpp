#include "Copter.h"
#include "PID.h"
/*
 * Init and run calls for guided flight mode
 */

#ifndef GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM
 # define GUIDED_LOOK_AT_TARGET_MIN_DISTANCE_CM     500     // point nose at target if it is more than 5m away
#endif

#define GUIDED_POSVEL_TIMEOUT_MS    3000    // guided mode's position-velocity controller times out after 3seconds with no new updates
#define GUIDED_ATTITUDE_TIMEOUT_MS  1000    // guided mode's attitude controller times out after 1 second with no new updates

static Vector3f guided_pos_target_cm;       // position target (used by posvel controller only)
static Vector3f guided_vel_target_cms;      // velocity target (used by velocity controller and posvel controller)
#ifdef ScModifies
PID VelToAcc_x(4250.0f,1000.0f,390.0f,180.0f),VelToAcc_y(4300.0f,2000.0f,390.0f,250.0f);
static Vector3f guided_new_vel_target_ms;   // velocity target (medium)
static uint32_t new_vel_update_time_ms;     // system time of last target update to velocity controller
bool debugtexttoken = false;
bool Sc_Landing_Token = false;
float desired_disarm_height = 15.0f;        // range from 10cm to 50cm
float desired_climbrate = 30.0f;
float desired_throttle = 0.0f;
#endif
static uint32_t posvel_update_time_ms;      // system time of last target update to posvel controller (i.e. position and velocity update)
static uint32_t vel_update_time_ms;         // system time of last target update to velocity controller

struct {
    uint32_t update_time_ms;
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    float yaw_rate_cds;
    float climb_rate_cms;
    bool use_yaw_rate;
} static guided_angle_state = {0,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false};

struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

#ifdef ScModifies
#define ScQueueMaxCapacity 5
#define ScAltitudeFilterMaxCapacity 3
struct Sc_RPY
{
    float roll_cd;
    float pitch_cd;
    float yaw_cd;
    Sc_RPY() : roll_cd(0.0f), pitch_cd(0.0f), yaw_cd(0.0f){};
    Sc_RPY(float _roll_cd, float _pitch_cd, float _yaw_cd) : roll_cd(_roll_cd), pitch_cd(_pitch_cd), yaw_cd(_yaw_cd){};
};

class Sc_RPY_Queue
{
private:
    uint8_t QueueCount;
    float ground_yaw;
    struct Node
    {
        Sc_RPY rpy;
        Node *next;
        Node() : next(nullptr){};
    };
    Node *HeadNode;
    Node *TailNode;
    void chainreaction(Node * _node)
    {
    	if(_node -> next != nullptr)
    		chainreaction(_node -> next);
    	delete _node;
    }
    void chain_yaw_set(Node* _node,float _yaw)
    {
        if(_node -> next != nullptr)
             chain_yaw_set(_node -> next,_yaw);
        _node->rpy.yaw_cd=_yaw;
    }

public:
    Sc_RPY_Queue()
    {
        HeadNode = new Node;
        TailNode = HeadNode;
        ground_yaw = 0.0f;
        QueueCount = 0;
    }
    void append(Sc_RPY _newmember)
    {
        static int debugcount = 0;
        debugcount++;
        
        static uint32_t debugtextlooptime = 0;
        if(millis() - debugtextlooptime > 200)
        {
            if(debugtexttoken)
                copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Queue count: %d\n", debugcount);
            debugtextlooptime = millis();
        }

        if(QueueCount == 0)
        {
            TailNode->rpy = _newmember;
            if(fabs(ground_yaw - 0.0f) <= 1e-6)
                ground_yaw = _newmember.yaw_cd;
            else
                TailNode->rpy.yaw_cd = ground_yaw;
            TailNode->next = new Node;
            TailNode = TailNode->next;
        }
        else
        {
            TailNode->rpy = _newmember;
            TailNode->rpy.yaw_cd = HeadNode->rpy.yaw_cd;   //fix yaw
            TailNode->next = new Node;
            TailNode = TailNode->next;
        }
        QueueCount++;
        if (QueueCount > ScQueueMaxCapacity)
        {
            Node *temp = HeadNode;
            HeadNode = HeadNode->next;
            delete temp;
            QueueCount--;
        }
    }
    void set_yaw(float _yaw)
    {
        chain_yaw_set(HeadNode,_yaw);
    }
    void set_ground_yaw(void)
    {
        this -> set_yaw(ground_yaw);
    }
    /*Sc_RPY getaverage(void)
    {
        if(QueueCount == 0)
            return Sc_RPY();
        int i = 0;
        Sc_RPY temp;
        Node *p = HeadNode;
        for (i = 0; i < QueueCount; i++)
        {
            temp.roll_cd += p->rpy.roll_cd;
            temp.pitch_cd += p->rpy.pitch_cd;
            temp.yaw_cd += p->rpy.yaw_cd;
            p = p->next;
            if (p == nullptr)
            {
                if(debugtexttoken)
                    copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING,"---------------------\nWRONG!\n---------------------\n");
                break;
            }
        }
        temp.roll_cd /= i;
        temp.pitch_cd /= i;
        temp.yaw_cd /= i;
        return temp;
    }*/
    Sc_RPY getaverage(void)
    {
        Sc_RPY temp;
        temp.roll_cd = 0.0f;
        temp.pitch_cd = 0.0f;
        temp.yaw_cd = HeadNode->rpy.yaw_cd;
        return temp;
    }
    void resetqueue(void)
    {
        chainreaction(HeadNode);
        HeadNode = new Node;
        TailNode = HeadNode;
        QueueCount = 0;
    }
} RPY_Queue;

class Sc_altitude_filter
{
private:
    uint8_t QueueCount;
    struct Node
    {
        float height;
        Node *next;
        Node() : next(nullptr){};
    };
    Node *HeadNode;
    Node *TailNode;
    void chainreaction(Node * _node)
    {
    	if(_node -> next != nullptr)
    		chainreaction(_node -> next);
    	delete _node;
	}
public:
    Sc_altitude_filter()
    {
        HeadNode = new Node;
        TailNode = HeadNode;
        QueueCount = 0;
    }
    float getaverage(void)
    {
        if(QueueCount == 0)
            return 0;
        int i = 0;
        int16_t temp=0;
        Node *p = HeadNode;
        for (i = 0; i < QueueCount; i++)
        {
            temp += p->height;
            p = p->next;
            if (p == nullptr)
            {
                if(debugtexttoken)
                    copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING,"---------------------\nWRONG!\n---------------------\n");
                break;
            }
        }
        temp /= i;
        return temp;
    }
    void append(float _newmember)
    {
        if(QueueCount == 0)
        {
            TailNode->height = _newmember;
            TailNode->next = new Node;
            TailNode = TailNode->next;
        }
        else
        {
            if(_newmember > 200.0f)
                return;
            else
            {
                TailNode->height = _newmember;
                TailNode->next = new Node;
                TailNode = TailNode->next;
            }
        }
        QueueCount++;
        if (QueueCount > ScQueueMaxCapacity)
        {
            Node *temp = HeadNode;
            HeadNode = HeadNode->next;
            delete temp;
            QueueCount--;
        }
    }
    void resetqueue(void)
    {
        chainreaction(HeadNode);
        HeadNode = new Node;
        TailNode = HeadNode;
        QueueCount = 0;
    }
    float range(float _min, float _input, float _max)
    {
        return (_input > _max) ? _max : (_input < _min) ? _min : _input;
    }
} Altitude_Filter;
#endif



// guided_init - initialise guided controller
bool Copter::guided_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {
        // initialise yaw
        set_auto_yaw_mode(get_default_auto_yaw_mode(false));
        // start in position control mode
        guided_pos_control_start();
        return true;
    }else{
        return false;
    }
}


// guided_takeoff_start - initialises waypoint controller to implement take-off
bool Copter::guided_takeoff_start(float final_alt_above_home)
{
    guided_mode = Guided_TakeOff;

    // initialise wpnav destination
    Location_Class target_loc = current_loc;
    target_loc.set_alt_cm(final_alt_above_home, Location_Class::ALT_FRAME_ABOVE_HOME);

    if (!wp_nav->set_wp_destination(target_loc)) {
        // failure to set destination can only be because of missing terrain data
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // initialise yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    // get initial alt for WP_NAVALT_MIN
    auto_takeoff_set_start_alt();

#ifdef ScModifies
    RPY_Queue.resetqueue();
    Altitude_Filter.resetqueue();
    Sc_Landing_Token = false;
    RPY_Queue.append(Sc_RPY(ahrs.roll_sensor,ahrs.pitch_sensor,ahrs.yaw_sensor));
#endif
    
    return true;
}

// initialise guided mode's position controller
void Copter::guided_pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(stopping_point, false);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

// initialise guided mode's velocity controller
void Copter::guided_vel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Velocity;

    // initialise horizontal speed, acceleration and jerk
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_jerk_xy_to_default();

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control->set_accel_z(g.pilot_accel_z);

    // initialise velocity controller
    pos_control->init_vel_controller_xyz();
}

#ifdef ScModifies
    //finished
// initialise guided mode's new_vel controller
void Copter::guided_new_vel_control_start()
{
    // set guided_mode to new velocity controller
    guided_mode = Guided_NewVelocity;

    /******************************************************************/
    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    new_vel_update_time_ms = millis();
    guided_new_vel_target_ms.zero();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;
    guided_angle_state.yaw_rate_cds = 0.0f;
    guided_angle_state.use_yaw_rate = false;

    Altitude_Filter.append((float)rangefinder_state.alt_cm);

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
    /******************************************************************/
}
#endif

// initialise guided mode's posvel controller
void Copter::guided_posvel_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_PosVel;

    pos_control->init_xy_controller();

    // set speed and acceleration from wpnav's speed and acceleration
    pos_control->set_speed_xy(wp_nav->get_speed_xy());
    pos_control->set_accel_xy(wp_nav->get_wp_acceleration());
    pos_control->set_jerk_xy_to_default();

    const Vector3f& curr_pos = inertial_nav.get_position();
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // set target position and velocity to current position and velocity
    pos_control->set_xy_target(curr_pos.x, curr_pos.y);
    pos_control->set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// initialise guided mode's angle controller
void Copter::guided_angle_control_start()
{
    // set guided_mode to velocity controller
    guided_mode = Guided_Angle;

    // set vertical speed and acceleration
    pos_control->set_speed_z(wp_nav->get_speed_down(), wp_nav->get_speed_up());
    pos_control->set_accel_z(wp_nav->get_accel_z());

    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // initialise targets
    guided_angle_state.update_time_ms = millis();
    guided_angle_state.roll_cd = ahrs.roll_sensor;
    guided_angle_state.pitch_cd = ahrs.pitch_sensor;
    guided_angle_state.yaw_cd = ahrs.yaw_sensor;
    guided_angle_state.climb_rate_cms = 0.0f;
    guided_angle_state.yaw_rate_cds = 0.0f;
    guided_angle_state.use_yaw_rate = false;

    // pilot always controls yaw
    set_auto_yaw_mode(AUTO_YAW_HOLD);
}

// guided_set_destination - sets guided mode's target destination
// Returns true if the fence is enabled and guided waypoint is within the fence
// else return false if the waypoint is outside the fence
bool Copter::guided_set_destination(const Vector3f& destination)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination if outside the fence
    Location_Class dest_loc(destination);
    if (!fence.check_destination_within_fence(dest_loc)) {
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(destination, false);

    // log target
    Log_Write_GuidedTarget(guided_mode, destination, Vector3f());
    return true;
}

// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool Copter::guided_set_destination(const Location_Class& dest_loc)
{
    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!fence.check_destination_within_fence(dest_loc)) {
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    if (!wp_nav->set_wp_destination(dest_loc)) {
        // failure to set destination can only be because of missing terrain data
        Log_Write_Error(ERROR_SUBSYSTEM_NAVIGATION, ERROR_CODE_FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    // log target
    Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}

// guided_set_velocity - sets guided mode's target velocity
void Copter::guided_set_velocity(const Vector3f& velocity)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Velocity) {
        guided_vel_control_start();
    }

    // record velocity target
    guided_vel_target_cms = velocity;
    vel_update_time_ms = millis();

    // log target
    Log_Write_GuidedTarget(guided_mode, Vector3f(), velocity);
}

#ifdef ScModifies
    //finished
// guided_set_new_velocity - set our guided mode's velocity
void Copter::guided_set_new_velocity(const Vector3f& velocity)
{
    /* Tools to be used
     * ahrs.yaw;   // get yaw
     * ahrs.roll;  // get roll
     * ahrs.pitch; // get pitch
     *
     * (get velocity) change vecter3f to a Vector3f variable
     * EKF3.get_VelNED(-1,vector3f);
     * EKF3.AccelNED(vector3f);
     */
    // the input velocity is using body coordinate system
    // we need to convert it to attitude

    // check we are in new velocity control mode
    if (guided_mode != Guided_NewVelocity) {
        guided_new_vel_control_start();
    }
    /************************** think and change *****************************/
    // record velocity target
    guided_new_vel_target_ms = velocity;
    new_vel_update_time_ms = millis();
    //guided_angle_state.climb_rate_cms = _climb_rate_cms;
    Sc_Landing_Token = false;
    
    /***************************(to be change)********************************
    // convert quaternion to euler angles
    q.to_euler(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd);
    guided_angle_state.roll_cd = ToDeg(guided_angle_state.roll_cd) * 100.0f;
    guided_angle_state.pitch_cd = ToDeg(guided_angle_state.pitch_cd) * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd(ToDeg(guided_angle_state.yaw_cd) * 100.0f);
    guided_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    guided_angle_state.use_yaw_rate = use_yaw_rate;

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !ap.auto_armed && (guided_angle_state.climb_rate_cms > 0.0f)) {
        set_auto_armed(true);
    }
    **************************************************************************/
    // change velocity to NED before logging
    Vector3f logvelocity;
    float tempyaw = ahrs.yaw;
    logvelocity.x = velocity.x * cosf(tempyaw) - velocity.y * sinf(tempyaw);
    logvelocity.y = velocity.x * sinf(tempyaw) + velocity.y * cosf(tempyaw);
    logvelocity.z = velocity.z;
    // log target
    Log_Write_GuidedTarget(guided_mode, Vector3f(), logvelocity);
}

void Copter::guided_new_landing(const Vector3f& velocity, float _desired_disarm_height, float _desired_climbrate, float _desired_throttle)
{
    if (guided_mode != Guided_NewVelocity) {
        return;
    }

    if(velocity.z * 100 < _desired_disarm_height)
    {
        guided_new_vel_target_ms = velocity;
        new_vel_update_time_ms = millis();
        desired_disarm_height = fmaxf(fminf(50.0f,_desired_disarm_height), 10.0f);
        desired_climbrate = fabs(_desired_climbrate);
        desired_throttle = fmaxf(fminf(0.5f, _desired_throttle), 0.0f);
        Sc_Landing_Token = true;
    }
}

void Copter::set_yaw(float yaw)
{
    if(fabs(yaw - 8888.0f) <= 1e-6)
    {
        RPY_Queue.set_ground_yaw();
        return;
    }
    float yaw_cd=yaw*100;
    yaw_cd = ahrs.yaw_sensor + yaw_cd;
    RPY_Queue.set_yaw(yaw_cd);
}

void Copter::set_pid(int controller_id, float _kp, float _ki, float _kd, float _imax)
{
    debugtexttoken = false;
    if(controller_id==0)
    {
        VelToAcc_x.kP(_kp);
        VelToAcc_x.kI(_ki);
        VelToAcc_x.kD(_kd);
        VelToAcc_x.imax(_imax);
    }
    if(controller_id==1)
    {
        VelToAcc_y.kP(_kp);
        VelToAcc_y.kI(_ki);
        VelToAcc_y.kD(_kd);
        VelToAcc_y.imax(_imax);
    }
}
#endif

// set guided mode posvel target
void Copter::guided_set_destination_posvel(const Vector3f& destination, const Vector3f& velocity) {
    // check we are in velocity control mode
    if (guided_mode != Guided_PosVel) {
        guided_posvel_control_start();
    }

    posvel_update_time_ms = millis();
    guided_pos_target_cm = destination;
    guided_vel_target_cms = velocity;

    pos_control->set_pos_target(guided_pos_target_cm);

    // log target
    Log_Write_GuidedTarget(guided_mode, destination, velocity);
}

// set guided mode angle target
void Copter::guided_set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads)
{
    // check we are in velocity control mode
    if (guided_mode != Guided_Angle) {
        guided_angle_control_start();
    }

    // convert quaternion to euler angles
    q.to_euler(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd);
    guided_angle_state.roll_cd = ToDeg(guided_angle_state.roll_cd) * 100.0f;
    guided_angle_state.pitch_cd = ToDeg(guided_angle_state.pitch_cd) * 100.0f;
    guided_angle_state.yaw_cd = wrap_180_cd(ToDeg(guided_angle_state.yaw_cd) * 100.0f);
    guided_angle_state.yaw_rate_cds = ToDeg(yaw_rate_rads) * 100.0f;
    guided_angle_state.use_yaw_rate = use_yaw_rate;

    guided_angle_state.climb_rate_cms = climb_rate_cms;
    guided_angle_state.update_time_ms = millis();

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !ap.auto_armed && (guided_angle_state.climb_rate_cms > 0.0f)) {
        set_auto_armed(true);
    }

    // log target
    Log_Write_GuidedTarget(guided_mode,
                           Vector3f(guided_angle_state.roll_cd, guided_angle_state.pitch_cd, guided_angle_state.yaw_cd),
                           Vector3f(0.0f, 0.0f, guided_angle_state.climb_rate_cms));
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::guided_run()
{
    // call the correct auto controller
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        guided_takeoff_run();
        break;

    case Guided_WP:
        // run position controller
        guided_pos_control_run();
        break;

    case Guided_Velocity:
        // run velocity controller
        guided_vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        guided_posvel_control_run();
        break;

    case Guided_Angle:
        // run angle controller
        guided_angle_control_run();
        break;
#ifdef ScModifies
        // finished
    case Guided_NewVelocity:
        // run new velocity controller
        guided_new_vel_control_run();
        break;
#endif
    }
 }

// guided_takeoff_run - takeoff in guided mode
//      called by guided_run at 100hz or more
void Copter::guided_takeoff_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock()) {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else   // multicopters do not stabilize roll/pitch/yaw when disarmed
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // reset attitude control targets
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

#if FRAME_CONFIG == HELI_FRAME
    // helicopters stay in landed state until rotor speed runup has finished
    if (motors->rotor_runup_complete()) {
        set_land_complete(false);
    } else {
        // initialise wpnav targets
        wp_nav->shift_wp_origin_to_current_pos();
    }
#else
    set_land_complete(false);
#endif

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    auto_takeoff_attitude_run(target_yaw_rate);
}

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::guided_pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
}

// guided_vel_control_run - runs the guided velocity controller
// called from guided_run
void Copter::guided_vel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        // initialise velocity controller
        pos_control->init_vel_controller_xyz();
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !pos_control->get_desired_velocity().is_zero()) {
        guided_set_desired_velocity_with_accel_and_fence_limits(Vector3f(0.0f,0.0f,0.0f));
    } else {
        guided_set_desired_velocity_with_accel_and_fence_limits(guided_vel_target_cms);
    }




    // call velocity controller which includes z axis controller
    pos_control->update_vel_controller_xyz(ekfNavVelGainScaler);

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
}

// guided_posvel_control_run - runs the guided spline controller
// called from guided_run
void Copter::guided_posvel_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        // set target position and velocity to current position and velocity
        pos_control->set_pos_target(inertial_nav.get_position());
        pos_control->set_desired_velocity(Vector3f(0,0,0));
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        attitude_control->set_throttle_out(0,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
#endif
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;

    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set velocity to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - posvel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS && !guided_vel_target_cms.is_zero()) {
        guided_vel_target_cms.zero();
    }

    // calculate dt
    float dt = pos_control->time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= pos_control->get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // advance position target using velocity target
        guided_pos_target_cm += guided_vel_target_cms * dt;

        // send position and velocity targets to position controller
        pos_control->set_pos_target(guided_pos_target_cm);
        pos_control->set_desired_velocity_xy(guided_vel_target_cms.x, guided_vel_target_cms.y);

        // run position controller
        pos_control->update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF, ekfNavVelGainScaler, false);
    }

    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pos_control->get_roll(), pos_control->get_pitch(), target_yaw_rate, get_smoothing_gain());
    }else{
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(pos_control->get_roll(), pos_control->get_pitch(), get_auto_heading(), true, get_smoothing_gain());
    }
}

// guided_angle_control_run - runs the guided angle controller
// called from guided_run
void Copter::guided_angle_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || (ap.land_complete && guided_angle_state.climb_rate_cms <= 0.0f)) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f, get_smoothing_gain());
        attitude_control->set_throttle_out(0.0f,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0.0f,true,g.throttle_filt);
#endif
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);
    float yaw_rate_in = wrap_180_cd(guided_angle_state.yaw_rate_cds);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -fabsf(wp_nav->get_speed_down()), wp_nav->get_speed_up());

    // get avoidance adjusted climb rate
    climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
        yaw_rate_in = 0.0f;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // call attitude controller
    if (guided_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in, get_smoothing_gain());
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true, get_smoothing_gain());
    }

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
}

#ifdef ScModifies
#define DEBUGTEXT
    //under construction
#define LowAccThreshold 0.3f
void Copter::guided_new_vel_control_run()
{
    /* Tools to be used
     * ahrs.yaw;   // get yaw
     * ahrs.roll;  // get roll
     * ahrs.pitch; // get pitch
     * int rangefinder_state.alt_cm  //get altitude
     *
     * (get velocity) change vecter3f to a Vector3f variable
     * EKF3.getVelNED(-1,vector3f);
     * EKF3.getAccelNED(vector3f);
     */
    // get stabilized attitude and record it to a queue
#ifdef DEBUGTEXT
    bool transtoken = false;
    static uint32_t debugtextlooptime = 0;
    if(millis() - debugtextlooptime > 300)
    {
        transtoken = true;
        debugtextlooptime = millis();
    }
#endif

    float calculate_yaw = ahrs.yaw;
    Vector3f current_acceleration, current_velocity, current_body_velocity;
    EKF3.getAccelNED(current_acceleration);
    float current_yaw = ahrs.yaw_sensor, current_roll = ahrs.roll_sensor, current_pitch = ahrs.pitch_sensor;   //using cd
    if(fabs(current_acceleration.x) < LowAccThreshold && fabs(current_acceleration.y) < LowAccThreshold)
        RPY_Queue.append(Sc_RPY(current_roll,current_pitch,current_yaw));

// #ifdef DEBUGTEXT
//     static uint32_t debugtextlooptime = 0;
//     if(millis() - debugtextlooptime > 100)
//     {
//         //copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "AccX: %f AccY: %f\n", current_acceleration.x, current_acceleration.y);
//         debugtextlooptime = millis();
//     }
// #endif
    // calculate climb_rate_cms
    const float climb_rate_kp = 0.8f;
    Altitude_Filter.append((float)rangefinder_state.alt_cm);
    guided_angle_state.climb_rate_cms = climb_rate_kp * (guided_new_vel_target_ms.z * 100.0f - Altitude_Filter.getaverage());
#ifdef DEBUGTEXT
    if(transtoken == true)
    {
        if(debugtexttoken)
        {
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Altitude_Filter: %.4f\n", Altitude_Filter.getaverage());
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "guided_new_vel_target_ms.z: %.4f\n", guided_new_vel_target_ms.z);
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "rangefinder_state.alt_cm: %d\n", rangefinder_state.alt_cm);
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "climb_rate_cms: %.4f\n", guided_angle_state.climb_rate_cms);
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "\n");
        }
    }
#endif
    guided_angle_state.climb_rate_cms = Altitude_Filter.range(-25.0f, guided_angle_state.climb_rate_cms, 25.0f);

    // get current velocity
    EKF3.getVelNED(-1,current_velocity);
    guided_angle_state.update_time_ms = millis();
    current_body_velocity.x = current_velocity.x * cosf(calculate_yaw) + current_velocity.y * sinf(calculate_yaw);
    current_body_velocity.y = - current_velocity.x * sinf(calculate_yaw) + current_velocity.y * cosf(calculate_yaw);
    current_body_velocity.z = current_velocity.z;

    // set velocity to zero if no updates received for 3 seconds
    uint32_t _tnow = millis();
    if (_tnow - new_vel_update_time_ms > GUIDED_POSVEL_TIMEOUT_MS)
    {
        guided_new_vel_target_ms.zero();
    }

    // calculate attitude by velocity
    float desired_pitch,desired_roll;
    desired_pitch = - VelToAcc_x.get_pid(guided_new_vel_target_ms.x - current_body_velocity.x);
    desired_roll = VelToAcc_y.get_pid(guided_new_vel_target_ms.y - current_body_velocity.y);

    // send desired attitude to attitude controller
    Sc_RPY offset = RPY_Queue.getaverage();
    //Sc_RPY offset = Sc_RPY();
    guided_angle_state.roll_cd = desired_roll + offset.roll_cd;
    guided_angle_state.pitch_cd = desired_pitch + offset.pitch_cd;
    guided_angle_state.yaw_cd = offset.yaw_cd;
    guided_angle_state.yaw_rate_cds = 0.0f;
    guided_angle_state.use_yaw_rate = false;

/*#ifdef DEBUGTEXT
    if(transtoken == true)
    {
        if(debugtexttoken)
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "X:\nControl: %.4f Error: %.4f\n",desired_pitch, guided_new_vel_target_ms.x - current_body_velocity.x);
        if(debugtexttoken)
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Y:\nControl: %.4f Error: %.4f\n",desired_roll, guided_new_vel_target_ms.y - current_body_velocity.y);    
    }  
#endif*/

    //guided_angle_state.climb_rate_cms = climb_rate_cms;         //
    guided_angle_state.update_time_ms = millis();

    if(Sc_Landing_Token)
    {
        guided_angle_state.climb_rate_cms = -Altitude_Filter.range(10.0f, desired_climbrate, 50.0f);
        if(Altitude_Filter.getaverage() < desired_disarm_height)
        {
            attitude_control->set_throttle_out_unstabilized(desired_throttle,true,g.throttle_filt);
            if (ap.land_complete) 
            {
                init_disarm_motors();
                Sc_Landing_Token = false;
            }
        }
    }

    // interpret positive climb rate as triggering take-off
    if (motors->armed() && !ap.auto_armed && (guided_angle_state.climb_rate_cms > 0.0f))
    {
        set_auto_armed(true);
    }
    /**************************************************************************************/
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || (ap.land_complete && guided_angle_state.climb_rate_cms <= 0.0f)) {
#if FRAME_CONFIG == HELI_FRAME  // Helicopters always stabilize roll/pitch/yaw
        // call attitude controller
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, 0.0f, get_smoothing_gain());
        attitude_control->set_throttle_out(0.0f,false,g.throttle_filt);
#else
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control->set_throttle_out_unstabilized(0.0f,true,g.throttle_filt);
#endif
        pos_control->relax_alt_hold_controllers(0.0f);
        return;
    }

    // constrain desired lean angles
    float roll_in = guided_angle_state.roll_cd;
    float pitch_in = guided_angle_state.pitch_cd;
    float total_in = norm(roll_in, pitch_in);
    float angle_max = MIN(attitude_control->get_althold_lean_angle_max(), aparm.angle_max);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // wrap yaw request
    float yaw_in = wrap_180_cd(guided_angle_state.yaw_cd);
    float yaw_rate_in = wrap_180_cd(guided_angle_state.yaw_rate_cds);

    // constrain climb rate
    float climb_rate_cms = constrain_float(guided_angle_state.climb_rate_cms, -fabsf(wp_nav->get_speed_down()), wp_nav->get_speed_up());

    // get avoidance adjusted climb rate
    climb_rate_cms = get_avoidance_adjusted_climbrate(climb_rate_cms);

    // check for timeout - set lean angles and climb rate to zero if no updates received for 3 seconds
    uint32_t tnow = millis();
    if (tnow - guided_angle_state.update_time_ms > GUIDED_ATTITUDE_TIMEOUT_MS) {
        roll_in = 0.0f;
        pitch_in = 0.0f;
        climb_rate_cms = 0.0f;
        yaw_rate_in = 0.0f;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

/*#ifdef DEBUGTEXT
    if(transtoken == true)
    {
        if(debugtexttoken)
            copter.gcs_send_text_fmt(MAV_SEVERITY_WARNING, "roll_in = %f, pitch_in = %f\n",roll_in,pitch_in);
    }
#endif*/
    // call attitude controller
    if (guided_angle_state.use_yaw_rate) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(roll_in, pitch_in, yaw_rate_in, get_smoothing_gain());
    } else {
        attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true, get_smoothing_gain());
    }

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    pos_control->update_z_controller();
    /**************************************************************************************/
}
#endif

// helper function to update position controller's desired velocity while respecting acceleration limits
void Copter::guided_set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des)
{
    // get current desired velocity
    Vector3f curr_vel_des = pos_control->get_desired_velocity();

    // exit immediately if already equal
    if (curr_vel_des == vel_des) {
        return;
    }

    // get change in desired velocity
    Vector3f vel_delta = vel_des - curr_vel_des;

    // limit xy change
    float vel_delta_xy = safe_sqrt(sq(vel_delta.x)+sq(vel_delta.y));
    float vel_delta_xy_max = G_Dt * pos_control->get_accel_xy();
    float ratio_xy = 1.0f;
    if (!is_zero(vel_delta_xy) && (vel_delta_xy > vel_delta_xy_max)) {
        ratio_xy = vel_delta_xy_max / vel_delta_xy;
    }
    curr_vel_des.x += (vel_delta.x * ratio_xy);
    curr_vel_des.y += (vel_delta.y * ratio_xy);

    // limit z change
    float vel_delta_z_max = G_Dt * pos_control->get_accel_z();
    curr_vel_des.z += constrain_float(vel_delta.z, -vel_delta_z_max, vel_delta_z_max);

#if AC_AVOID_ENABLED
    // limit the velocity to prevent fence violations
    avoid.adjust_velocity(pos_control->get_pos_xy_kP(), pos_control->get_accel_xy(), curr_vel_des);
#endif

    // update position controller with new target
    pos_control->set_desired_velocity(curr_vel_des);
}

// Guided Limit code

// guided_limit_clear - clear/turn off guided limits
void Copter::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Copter::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Copter::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Copter::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = pv_get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}
