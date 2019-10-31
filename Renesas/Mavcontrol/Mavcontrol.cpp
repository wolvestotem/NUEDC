#include "math.h"
#include "mavlink.h"

#define SET_POSITION 3576
#define SET_VELOCITY 3527
#define SET_ATTITUDE_NO_RATE 63
#define GCS_ID 255
#define send_short_msg

class SCM
{
public:
	void init(
		unsigned short (*serialwrite)(unsigned char *buffer,uint16_t lenth),
		void (*Delay)(uint32_t),
		uint32_t (*Clock)(void),
		void (*Debug_text)(char *,...)
		);
protected:
	SCM(){};
	~SCM(){};

	void (*delay)(uint32_t);
	uint32_t (*clock)();
	void (*debug_text)(char *, ...);
	unsigned short (*Serialwrite)(unsigned char *buffer,uint16_t lenth);

	void eulerAnglesToQuaternion(const float* e,float q[4])
	{
		float cr2 = cosf(e[0]*0.5f);
	    float cp2 = cosf(e[1]*0.5f);
	    float cy2 = cosf(e[2]*0.5f);
	    float sr2 = sinf(e[0]*0.5f);
	    float sp2 = sinf(e[1]*0.5f);
	    float sy2 = sinf(e[2]*0.5f);
	    q[0] = cr2*cp2*cy2 + sr2*sp2*sy2;
	    q[1] = sr2*cp2*cy2 - cr2*sp2*sy2;
	    q[2] = cr2*sp2*cy2 + sr2*cp2*sy2;
	    q[3] = cr2*cp2*sy2 - sr2*sp2*cy2;
	}
	void quaternionToEulerAngles(const float* q,float e[3])
	{
		e[0] =atan2f(2.0f*(q[0]*q[1] + q[2]*q[3]), 1.0f - 2.0f*(q[1]*q[1] + q[2]*q[2]));
		e[1] =asin(2.0f*(q[0]*q[2] - q[3]*q[1]));
		e[2] =atan2f(2.0f*(q[0]*q[3] + q[1]*q[2]), 1.0f - 2.0f*(q[2]*q[2] + q[3]*q[3]));
	}

};

void SCM::init(
	unsigned short (*serialwrite)(unsigned char *buffer,uint16_t lenth),
	void (*Delay)(uint32_t),
	uint32_t (*Clock)(void),
	void (*Debug_text)(char *,...)
	)
{
	Serialwrite=serialwrite;
	clock=Clock;
	debug_text=Debug_text;
	delay=Delay;
}
class Mavlink:public SCM
{
public:
	bool S_set_yaw(float);
	bool gyro_calibrate();
	bool S_new_land(float,float,float,float,float,float);
protected:
	Mavlink();
	~Mavlink(){};
	bool S_heartbeat();
	bool S_command_long(MAV_CMD,float,float,float,float,float,float,float);
	bool S_set_mode(uint8_t base,uint32_t custom);
	bool S_set_position_local(uint16_t mask,float,float,float);
	bool S_set_attitude(float,float,float,float);
	bool S_set_velocity_body(float,float,float);
	bool S_set_pid(float,float,float,float,float);
	bool data_request(uint8_t );
	bool home_position();
	bool Receive(unsigned char a);

	mavlink_attitude_t attitude;
	mavlink_optical_flow_t opt;
	mavlink_local_position_ned_t localpoint;
	uint8_t wait_result();
private:
	mavlink_message_t sendmsg;
	mavlink_status_t recvstatus;
	mavlink_message_t recvdata;
	unsigned char copydata[sizeof(mavlink_message_t)];
	unsigned char senddata[sizeof(mavlink_message_t)];

	mavlink_heartbeat_t recv_heartbeat;
	mavlink_command_ack_t comm_result;
	mavlink_statustext_t recv_text;

	int last_cmd;
	float eular[3];
	float quat[4];
	uint16_t cut();
};

Mavlink::Mavlink()
{

}

bool Mavlink::S_heartbeat()
{
	mavlink_msg_heartbeat_pack(225,0,&sendmsg,6,8,192,0,4);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::S_command_long(MAV_CMD cmd,float a1,float a2,float a3,float a4,float a5,float a6,float a7)
{
	last_cmd=cmd;
	mavlink_msg_command_long_pack(GCS_ID,0,&sendmsg,1,1,cmd,0,a1,a2,a3,a4,a5,a6,a7);
	uint16_t valid_lenth=cut();
	return (Serialwrite(senddata,valid_lenth))&&(!(bool)wait_result());
}
bool Mavlink::S_set_mode(uint8_t base,uint32_t custom)
{
	last_cmd=11;//SET_MODE
	mavlink_msg_set_mode_pack(GCS_ID,0,&sendmsg,1,base,custom);
	uint16_t valid_lenth=cut();
	return (Serialwrite(senddata,valid_lenth))&&(!(bool)wait_result());
}
bool Mavlink::S_set_position_local(uint16_t mask,float a1,float a2,float a3)
{
	if(mask==SET_POSITION)
		mavlink_msg_set_position_target_local_ned_pack(GCS_ID,0,&sendmsg,clock(),1,1,1,mask,a1,a2,a3,0,0,0,0,0,0,0,0);
	if(mask==SET_VELOCITY)
		mavlink_msg_set_position_target_local_ned_pack(GCS_ID,0,&sendmsg,clock(),1,1,1,mask,0,0,0,a1,a2,a3,0,0,0,0,0);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::S_set_attitude(float a1,float a2,float a3,float a4=0.5)
{
	memset(eular,0,sizeof(eular));
	if(a4<0.4)
		a4=fabs(a4)/a4*0.4;
	if(a4>0.6)
		a4=fabs(a4)/a4*0.6;

	if(a1<1e-5)
		goto except_a1_0;
	if(fabs(a1)>0.14)
		a1=fabs(a1)/a1*0.14;
	if(fabs(a1)<0.015)
		a1=fabs(a1)/a1*0.015;
	except_a1_0 :
	if(a2<1e-5)
		goto except_a2_0;
	if(fabs(a2)>0.14)
		a2=fabs(a2)/a2*0.14;
	if(fabs(a2)<0.015)
		a2=fabs(a2)/a2*0.015;

	except_a2_0:
	eular[0]=a1;
	eular[1]=a2;
	eular[2]=a3;
	eulerAnglesToQuaternion(eular,quat);
	mavlink_msg_set_attitude_target_pack(GCS_ID,0,&sendmsg,clock(),0,0,SET_ATTITUDE_NO_RATE,quat,0,0,0,a4);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::S_set_velocity_body(float a1,float a2,float a3)
{
	mavlink_msg_set_velocity_body_pack(GCS_ID,0,&sendmsg,clock(),1,0,a1,a2,a3);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::S_new_land(float a1,float a2,float a3,float a4,float a5,float a6)
{
	mavlink_msg_new_land_pack(GCS_ID,0,&sendmsg,1,0,a1,a2,a3,a4,a5,a6);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::S_set_pid(float a1,float a2,float a3,float a4,float a5)
{
	mavlink_msg_set_pid_pack(GCS_ID,0,&sendmsg,1,0,a1,a2,a3,a4,a5);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::S_set_yaw(float a1)
{
	mavlink_msg_set_yaw_pack(GCS_ID,0,&sendmsg,1,0,a1);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::gyro_calibrate()
{
	mavlink_msg_command_long_pack(GCS_ID,0,&sendmsg,1,0,MAV_CMD_PREFLIGHT_CALIBRATION,0,0,0,0,0,2,0,0);
	uint16_t valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
}
bool Mavlink::data_request(uint8_t hz)
{
	mavlink_msg_request_data_stream_pack(GCS_ID,0,&sendmsg,0,1,0,1,1);
	uint16_t valid_lenth=cut();
	Serialwrite(senddata,valid_lenth);
	mavlink_msg_request_data_stream_pack(GCS_ID,0,&sendmsg,0,1,12,4,1);
	valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);
	/*
	mavlink_msg_request_data_stream_pack(GCS_ID,0,&sendmsg,0,1,10,10,1);
	valid_lenth=cut();
	Serialwrite(senddata,valid_lenth);
	mavlink_msg_request_data_stream_pack(GCS_ID,0,&sendmsg,0,1,6,1,1);
	valid_lenth=cut();
	return Serialwrite(senddata,valid_lenth);*/
}
bool Mavlink::home_position()
{
	float angle[4]={1,0,0,0};
	mavlink_msg_set_home_position_pack(GCS_ID,0,&sendmsg,1,1,1,0,0,0,0,angle,0,0,0);
	uint16_t valid_lenth=cut();
	debug_text("set home position");
	return Serialwrite(senddata,valid_lenth);
}
uint8_t Mavlink::wait_result()
{
	for(uint8_t i=0;i<20;i++)
	{
		if(comm_result.command==last_cmd)
			return comm_result.result;
		delay(150);
	}
	debug_text("no result");
	return 5;
}

bool Mavlink::Receive(unsigned char a)
{
	if(mavlink_parse_char(MAVLINK_COMM_0,a,&recvdata,&recvstatus))
	{
		switch(recvdata.msgid)
		{
		case 0: break;		//mavlink_msg_heartbeat_decode(&recvdata,&recv_heartbeat);
		case 77: mavlink_msg_command_ack_decode(&recvdata,&comm_result);  break;
		case 253:
			mavlink_msg_statustext_decode(&recvdata,&recv_text);
			debug_text(recv_text.text);
			break;
		case 100: mavlink_msg_optical_flow_decode(&recvdata,&opt); break;
		case 32: mavlink_msg_local_position_ned_decode(&recvdata,&localpoint);break;
		case 30: mavlink_msg_attitude_decode(&recvdata,&attitude);break;
			//case 1: break;
			//case 1: break;
			//case 1: break;
			//case 1: break;
			//case 1: break;
		}
		return true;
	}
	return false;
}
uint16_t Mavlink::cut()
{
#ifdef send_short_msg
	int i=0;
	uint16_t valid_lenth=sendmsg.len+8;
	memset(senddata,'\0',sizeof(mavlink_message_t));
	memcpy(copydata,&sendmsg,sizeof(mavlink_message_t));
	while(i<valid_lenth)
	{
		senddata[i]=copydata[i+2];
		i++;
	}
	return valid_lenth;
#else
	memcpy(senddata,&sendmsg,sizeof(mavlink_message_t));
	return sizeof(mavlink_message_t);
#endif // send_short_msg
}

class Mavcontrol:public Mavlink
{
public:
	Mavcontrol(){};
	~Mavcontrol(){};
	bool pre_arm(void);
	bool arm(void);
	bool disarm(void);
	bool takeoff_check(void);
	bool keep_heartbeat(void);
	bool mav_takeoff(float);
	bool mav_land(void);
	bool set_vel(float,float,float);
	bool set_vel_new(float,float,float);
	bool set_pid(float,float,float,float,float);
	bool set_point(float,float,float);
	bool set_point_vel(float,float,float);
	bool set_attitude(float,float,float,float);
	bool emergency_stop();
	bool receive(unsigned char a);
	float* get_height();
	float* get_velocity_x();
	float* get_velocity_y();
	float fuck();
	bool mav_takeoff_stabilize(void);
private:
	float takeoff_position[3];
	float stabilize_attitude[3];
	float land_yaw;
};

bool Mavcontrol::pre_arm(void)
{
	if(!data_request(4))
	{
		debug_text("request data failed");
		return false;
	}
	delay(200);
	if(!home_position())
	{
		debug_text("set home position failed");
		return false;
	}
	delay(200);
	if(!S_set_mode(1,4))
	{
		debug_text("set mode failed\n");
		return false;
	}
	return true;
}
bool Mavcontrol::arm(void)
{
	if(!S_set_mode(1,4))
	{
		debug_text("set mode failed\n");
		return false;
	}
	delay(200);
	if(!S_command_long(MAV_CMD_COMPONENT_ARM_DISARM,1,0,0,0,0,0,0))
	{
		debug_text("arm denied\n");
		return false;
	}
	return true;
}
bool Mavcontrol::disarm(void)
{
	if(!S_command_long(MAV_CMD_COMPONENT_ARM_DISARM,1,0,0,0,0,0,1))
	{
		debug_text("disarm\n");
		return false;
	}
	return true;
}
bool Mavcontrol::keep_heartbeat(void)
{
	return S_heartbeat();
}
bool Mavcontrol::takeoff_check(void)
{
	if((fabs(localpoint.x)>1e-6)&&(fabs(localpoint.y)>1e-6))
		return true;
	else
		return false;
}
bool Mavcontrol::mav_takeoff(float height)
{
	takeoff_position[0]=localpoint.x;
	takeoff_position[1]=localpoint.y;
	takeoff_position[2]=height;
	land_yaw=attitude.yaw;
	if(!S_command_long(MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,height))
	{
		debug_text("takeoff denied\n");
		return false;
	}
	return true;
}
bool Mavcontrol::mav_takeoff_stabilize(void)
{
	if(set_point_vel(takeoff_position[0],takeoff_position[1],-takeoff_position[2]))
	{
		delay(1000);
		stabilize_attitude[0]=attitude.roll;
		stabilize_attitude[1]=attitude.pitch;
		stabilize_attitude[2]=attitude.yaw;
		debug_text("get stabilize attitude ");
		return true;
	}
	return false;
}

bool Mavcontrol::set_vel(float a1,float a2,float a3)
{
	return S_set_position_local(SET_VELOCITY,a1,a2,a3);
}
bool Mavcontrol::set_vel_new(float a1,float a2,float a3)
{
	return S_set_velocity_body(a1,a2,a3);
}
bool Mavcontrol::set_pid(float a1,float a2,float a3,float a4,float a5)
{
	return S_set_pid(a1,a2,a3,a4,a5);
}
bool Mavcontrol::set_point(float a1,float a2,float a3)
{
	return S_set_position_local(SET_POSITION,a1,a2,a3);
}
bool Mavcontrol::set_point_vel(float a1,float a2,float a3)
{
	return S_set_position_local(SET_POSITION|SET_VELOCITY,a1,a2,a3);
}
bool Mavcontrol::set_attitude(float a1,float a2,float a3,float a4=0.5)//forward right
{
	a1=a1/180*3.1415926;
	a2=a2/180*3.1415926;
	a1-=stabilize_attitude[1];
	a2+=stabilize_attitude[0];
	return S_set_attitude(a2,-a1,land_yaw,a4); //roll pitch
}
bool Mavcontrol::mav_land(void)
{
	return S_command_long(MAV_CMD_NAV_LAND,0,0,0,0,localpoint.x,localpoint.y,0);
}

bool Mavcontrol::emergency_stop()
{
	return true;
}
bool Mavcontrol::receive(unsigned char a)
{
	return Receive(a);
}
float* Mavcontrol::get_height()
{
	return &opt.ground_distance;
}
float* Mavcontrol::get_velocity_x()
{
	return &localpoint.vx;
}
float* Mavcontrol::get_velocity_y()
{
	return &localpoint.vy;
}
float Mavcontrol::fuck()
{
	float a=0.111111;
	return a;
}

#include "Mavcontrol.h"
