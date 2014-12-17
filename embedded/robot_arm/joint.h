/*
 * Author: Yevgeniy Kiveisha <yevgeniy.kiveisha@intel.com>
 * Copyright (c) 2014 Intel Corporation.
 */

#import <Arduino.h>

#pragma once

#define YES     0x1
#define NO      0x0

#define HOLD_POS     	0x1
#define DONT_HOLD_POS   0x0

#define GRIPPER_CLOSE  	0x1
#define GRIPPER_OPEN	0x0

#define MAX_TICK  	40
#define WAVE_LENGTH 20000
#define MIN_PULSE 	1000
#define MAX_PULSE 	2000
#define MIN_ANGLE 	0
#define MAX_ANGLE 	100

typedef void (* funcPtr) (void *);

enum type_t { BASE, MIDDLE, UPPER, GRIPPER };

typedef struct {
	int angle;
	int speed;
	int hold;
} position_t;

typedef struct {
	position_t*	pos;
	int  		index;
	int	 		size;
} flow_t;

typedef struct {
	type_t	type;
	int 	target_angle;
	int 	current_angle;
	int 	current_speed;
	
	int 	servo_pin;
	int 	pulse_min;
	int 	pulse_max;
	int 	polarity;
	int 	hold_position;
	int 	done;
	int 	callback;
	flow_t* flow;
	
	funcPtr finished_rotation;
} joint_context_t;

void
open_grripper (joint_context_t * ctx, int speed) {
	for (int cycle = 0; cycle < MAX_TICK; cycle++) {
		digitalWrite(ctx->servo_pin, HIGH);
		delayMicroseconds (MAX_PULSE);
		digitalWrite(ctx->servo_pin, LOW);
		delayMicroseconds (WAVE_LENGTH - MAX_PULSE);
		delay (speed);
	}
}

void
close_gripper (joint_context_t * ctx, int speed) {
	for (int cycle = 0; cycle < MAX_TICK; cycle++) {
		digitalWrite(ctx->servo_pin, HIGH);
		delayMicroseconds (MIN_PULSE);
		digitalWrite(ctx->servo_pin, LOW);
		delayMicroseconds (WAVE_LENGTH - MIN_PULSE);
		delay (speed);
	}
}

void
init_joint (joint_context_t * ctx, int joint_pin, type_t type, funcPtr fptr) {
	ctx->servo_pin 			= joint_pin;
	ctx->type				= type;
	ctx->finished_rotation 	= fptr;
	
	ctx->pulse_min 		= MIN_PULSE;
	ctx->pulse_max 		= MAX_PULSE;
	
	pinMode(ctx->servo_pin, OUTPUT);
}

void
rotate_joint (joint_context_t * ctx, int angle, int speed) {
	int width        = abs(ctx->pulse_min - ctx->pulse_max);
	int one_move     = width / MAX_ANGLE;
	int angle_width  = one_move * angle;
	int pulse_len    = 0;

	if (!ctx->polarity) {
		pulse_len = ctx->pulse_min + angle_width;
	} else {
		pulse_len = ctx->pulse_max - angle_width;
	}

	int ticks = (int)(MAX_TICK * ((float)(abs(ctx->current_angle - angle)) / (float)MAX_ANGLE)) + 1;
	for (int cycle = 0; cycle < ticks; cycle++) {
		digitalWrite(ctx->servo_pin, HIGH);
		delayMicroseconds (pulse_len);
		digitalWrite(ctx->servo_pin, LOW);
		delayMicroseconds (WAVE_LENGTH - pulse_len);
		delay (speed);
	}

	ctx->current_angle = angle;
}