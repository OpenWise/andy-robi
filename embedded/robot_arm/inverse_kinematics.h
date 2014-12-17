/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 */

#import <Arduino.h>
 
#pragma once

#define YES     0x1
#define NO      0x0

typedef struct {
	int base;
	int sholder;
	int elbow;
	int gripper;
} angles_t;

typedef struct {
	float 		hum_sq;
	float 		uln_sq;
	angles_t	angles;
} kinematics_t;

/* Arm dimensions( mm ) */
#define BASE_HGT_DIMM 	115			// base hight
#define HUMERUS_DIMM 	180      	// shoulder-to-elbow
#define ULNA_DIMM 		160			// elbow-to-wrist
#define GRIPPER_DIMM 	130			// gripper
 
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion

void
init_kinematics (kinematics_t * ctx) {
	ctx->hum_sq = HUMERUS_DIMM * HUMERUS_DIMM;
	ctx->uln_sq = ULNA_DIMM * ULNA_DIMM;
}

void
calculate_angle (kinematics_t * ctx, float x, float y, float z) {
	float base_angle 	= atan2 (y, x);
	float base_r		= sqrt ((x * x) + (y * y));
	
	ctx->angles.base = (int) degrees (base_angle);
	
	float grip_angle = radians (45);
	float wrist_r = base_r - (cos(grip_angle) * GRIPPER_DIMM);
	float wrist_z = z - BASE_HGT_DIMM + (sin(grip_angle) * GRIPPER_DIMM);
	float s_w = (wrist_z * wrist_z) + (wrist_r * wrist_r);
	float elbow_h = (sqrt (s_w)) / 2;
	
	ctx->angles.elbow = (int) degrees (asin (elbow_h / ULNA_DIMM));
	Serial.print(ctx->angles.elbow); Serial.print(", "); Serial.println(base_r);
}
