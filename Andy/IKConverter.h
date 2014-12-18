#ifndef IKCONVERTER_H
#define IKCONVERTER_H

/* Arm dimensions( mm ) */
#define BASE_HGT 115      //base hight 
#define HUMERUS 180       //shoulder-to-elbow "bone" 
#define ULNA 160          //elbow-to-wrist "bone" 
#define GRIPPER 130       //gripper (incl.heavy duty wrist rotate mechanism) length

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion

/* Servo names/numbers */
/* Base servo */
#define BAS_SERVO 0
/* Shoulder Servo */
#define SHL_SERVO 1
/* Elbow Servo */
#define ELB_SERVO 2
/* Wrist servo  */
#define WRI_SERVO 3
/* Gripper servo */
#define GRI_SERVO 4  //unused here

struct ArmAngles {
	/* shoulder angle */
	float shl_angle_r;
	float shl_angle_d;
	/* elbow angle */
	float elb_angle_r;
	float elb_angle_d;
	float elb_angle_dn;
	/* wrist angle */
	float wri_angle_d;
	/*base angle*/
	float bas_angle_r;
	float bas_angle_d;
};

class IKConverter {

public:
	IKConverter();
	void convertXYZtoServoAngles(float x, float y, float z, float grip_angle_d, ArmAngles *angles);
	void circle(ArmAngles *angles);
	void line(ArmAngles *angles);
	void zero_x(ArmAngles *angles);
};


#endif