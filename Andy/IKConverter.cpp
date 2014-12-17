
#include <iostream>
#include <math.h>
#include <cmath>
#include "IKConverter.h"

#define PI 3.14159

using namespace std;


IKConverter::IKConverter(){}

float radians(float degrees)
{
	return (degrees * PI) / 180;
}

float degrees(float radians)
{
	return (radians * 180) / PI;
}

/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
void IKConverter::convertXYZtoServoAngles(float x, float y, float z, float grip_angle_d, ArmAngles *angles)
{
	float hum_sq = HUMERUS*HUMERUS;
	float uln_sq = ULNA*ULNA;
	float grip_angle_r = radians(grip_angle_d);    //grip angle in radians for use in calculations
	/* Base angle and radial distance from x,y coordinates */
	float bas_angle_r = atan2(x, y);
	float rdist = sqrt((x * x) + (y * y));
	/* rdist is y coordinate for the arm */
	y = rdist;
	/* Grip offsets calculated based on grip angle */
	float grip_off_z = (sin(grip_angle_r)) * GRIPPER;
	float grip_off_y = (cos(grip_angle_r)) * GRIPPER;
	/* Wrist position */
	float wrist_z = (z - grip_off_z) - BASE_HGT;
	float wrist_y = y - grip_off_y;
	/* Shoulder to wrist distance ( AKA sw ) */
	float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
	float s_w_sqrt = sqrt(s_w);
	/* s_w angle to ground */
	//float a1 = atan2( wrist_y, wrist_z );
	float a1 = atan2(wrist_z, wrist_y);
	/* s_w angle to humerus */
	float a2 = acos(((hum_sq - uln_sq) + s_w) / (2 * HUMERUS * s_w_sqrt));
	/* shoulder angle */
	float shl_angle_r = a1 + a2;
	float shl_angle_d = degrees(shl_angle_r);
	/* elbow angle */
	float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));
	float elb_angle_d = degrees(elb_angle_r);
	float elb_angle_dn = -(180.0 - elb_angle_d);
	/* wrist angle */
	float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;

	/* set angles */
	angles->shl_angle_r = shl_angle_r;
	angles->shl_angle_d = shl_angle_d;
	angles->elb_angle_r = elb_angle_r;
	angles->elb_angle_d = elb_angle_d;
	angles->elb_angle_dn = elb_angle_dn;
	angles->wri_angle_d = wri_angle_d;
		
}

void IKConverter::zero_x(ArmAngles *angles)
{
	for (double yaxis = 150.0; yaxis < 356.0; yaxis += 1) {
		convertXYZtoServoAngles(0, yaxis, 127.0, 0, angles);
		//delay(10);
	}
	for (double yaxis = 356.0; yaxis > 150.0; yaxis -= 1) {
		convertXYZtoServoAngles(0, yaxis, 127.0, 0, angles);
		//delay(10);
	}
}

/* moves arm in a straight line */
void IKConverter::line(ArmAngles *angles)
{
	for (double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5) {
		convertXYZtoServoAngles(xaxis, 250, 100, 0, angles);
		//delay(10);
	}
	for (float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5) {
		convertXYZtoServoAngles(xaxis, 250, 100, 0, angles);
		//delay(10);
	}
}

void IKConverter::circle(ArmAngles *angles)
{
#define RADIUS 80.0
	//float angle = 0;
	float zaxis, yaxis;
	for (float angle = 0.0; angle < 360.0; angle += 1.0) {
		yaxis = RADIUS * sin(radians(angle)) + 200;
		zaxis = RADIUS * cos(radians(angle)) + 200;
		convertXYZtoServoAngles(0, yaxis, zaxis, 0, angles);
		//delay(1);
	}
}

