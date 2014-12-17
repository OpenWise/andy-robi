#include <SPI.h>
#include <Ethernet.h>
#include <Servo.h>
#include "joint.h"
#include "inverse_kinematics.h"

#define SERVO_BASE_PIN     			3
#define SERVO_MIDDLE_PIN   			5
#define SERVO_GRIPPER_PIN  			6
#define SERVO_UPPER_PIN    			9
#define SERVO_SHOLDER_LEFT_PIN    	11
#define SERVO_SHOLDER_RIGHT_PIN    	10
#define SERVO_MOTION_UNIT			5
#define PORT						10500

#define R    	12
#define G    	11
#define B    	10

byte			mac[]			= { 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };

Servo 			base_joint;
Servo 			middle_joint;
Servo 			upper_joint;
Servo 			sholder_l_joint;
Servo 			sholder_r_joint;
joint_context_t gripper_ctx;
joint_context_t upper_ctx;

int base_angle 		= 50;
int middle_angle 	= 100;
int upper_angle 	= 0;
int sholder_angle 	= 50;
int gripper_state 	= GRIPPER_CLOSE;

uint8_t cmd = 0;

kinematics_t inverse_kinematics;

EthernetServer robot_api_server (PORT);

void setup () {
  Serial.begin (57600);
  
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);
  
  digitalWrite (R, HIGH);
  digitalWrite (G, HIGH);
  digitalWrite (B, HIGH);
  
  init_ethernet();
  robot_api_server.begin();
  
  base_joint.attach (SERVO_BASE_PIN, MIN_PULSE, MAX_PULSE);
  middle_joint.attach (SERVO_MIDDLE_PIN, MIN_PULSE, MAX_PULSE);
  sholder_l_joint.attach (SERVO_SHOLDER_LEFT_PIN, MIN_PULSE, MAX_PULSE);
  sholder_r_joint.attach (SERVO_SHOLDER_RIGHT_PIN, MIN_PULSE, MAX_PULSE);
  init_joint (&gripper_ctx, SERVO_GRIPPER_PIN, GRIPPER, NULL);
  init_joint (&upper_ctx, SERVO_UPPER_PIN, UPPER, NULL);
}

void
loop () {
	EthernetClient client = robot_api_server.available();
	if (client) {
		if (client.available() > 0) {
			cmd = client.read();
			switch (cmd) {
				case 49:
					if (sholder_angle < MAX_ANGLE) {
						sholder_angle += SERVO_MOTION_UNIT;
						sholder_l_joint.writeMicroseconds(convert_angle_to_pulse (sholder_angle, 10));
						sholder_r_joint.writeMicroseconds(convert_angle_to_pulse (100 - sholder_angle, 10));
						Serial.print("BACK - "); Serial.println(sholder_angle);
					}
				break;
				case 50:
					if (sholder_angle > MIN_ANGLE) {
						sholder_angle -= SERVO_MOTION_UNIT;
						sholder_l_joint.writeMicroseconds(convert_angle_to_pulse (sholder_angle, 10));
						sholder_r_joint.writeMicroseconds(convert_angle_to_pulse (100 - sholder_angle, 10));
						Serial.print("FORWARD - "); Serial.println(sholder_angle);
					}
				break;
				case 97:
					if (base_angle < MAX_ANGLE) {
						base_angle += SERVO_MOTION_UNIT;
						base_joint.writeMicroseconds(convert_angle_to_pulse (base_angle, 10));
						Serial.print("LEFT - "); Serial.println(base_angle);
					}
				break;
				case 100:
					if (base_angle > MIN_ANGLE) {
						base_angle -= SERVO_MOTION_UNIT;
						base_joint.writeMicroseconds(convert_angle_to_pulse (base_angle, 10));
						Serial.print("RIGHT - "); Serial.println(base_angle);
					}
				break;
				case 115:
					if (middle_angle < MAX_ANGLE) {
						middle_angle += SERVO_MOTION_UNIT;
						middle_joint.writeMicroseconds(convert_angle_to_pulse (middle_angle, 10));
						Serial.print("DOWN - "); Serial.println(middle_angle);
					}
				break;
				case 119:
					if (middle_angle > MIN_ANGLE) {
						middle_angle -= SERVO_MOTION_UNIT;
						middle_joint.writeMicroseconds(convert_angle_to_pulse (middle_angle, 10));
						Serial.print("UP - "); Serial.println(middle_angle);
					}
				break;
				case 122:
					if (upper_angle < MAX_ANGLE) {
						upper_angle += SERVO_MOTION_UNIT;
						rotate_joint (&upper_ctx, upper_angle, 5);
						Serial.print("UP GRIPP - "); Serial.println(upper_angle);
					}
				break;
				case 120:
					if (upper_angle > MIN_ANGLE) {
						upper_angle -= SERVO_MOTION_UNIT;
						rotate_joint (&upper_ctx, upper_angle, 5);
						Serial.print("DOWN GRIPP - "); Serial.println(upper_angle);
					}
				break;
				case 91:
					if (gripper_state == GRIPPER_CLOSE) {
						open_grripper (&gripper_ctx, 15);
						gripper_state = GRIPPER_OPEN;
						Serial.println("OPEN");
					}
				break;
				case 93:
					if (gripper_state == GRIPPER_OPEN) {
						close_gripper (&gripper_ctx, 15);
						gripper_state = GRIPPER_CLOSE;
						Serial.println("CLOSE");
					}
				break;
				default:
					Serial.println(cmd);
				break;
			}
			// read the bytes incoming from the client:
			// char thisChar = client.read();
			// echo the bytes back to the client:
			// robot_api_server.write (thisChar);
			// echo the bytes to the server as well:
			// Serial.write (thisChar);
		}
	}
	
	if (Serial.available() > 0) {
		cmd = Serial.read();		
		switch (cmd) {
			case 97:
				if (base_angle < MAX_ANGLE) {
					base_angle += SERVO_MOTION_UNIT;
					base_joint.writeMicroseconds(convert_angle_to_pulse (base_angle, 10));
					Serial.print("LEFT - "); Serial.println(base_angle);
				}
			break;
			case 100:
				if (base_angle > MIN_ANGLE) {
					base_angle -= SERVO_MOTION_UNIT;
					base_joint.writeMicroseconds(convert_angle_to_pulse (base_angle, 10));
					Serial.print("RIGHT - "); Serial.println(base_angle);
				}
			break;
			case 115:
				if (middle_angle < MAX_ANGLE) {
					middle_angle += SERVO_MOTION_UNIT;
					middle_joint.writeMicroseconds(convert_angle_to_pulse (middle_angle, 10));
					Serial.print("DOWN - "); Serial.println(middle_angle);
				}
			break;
			case 119:
				if (middle_angle > MIN_ANGLE) {
					middle_angle -= SERVO_MOTION_UNIT;
					middle_joint.writeMicroseconds(convert_angle_to_pulse (middle_angle, 10));
					Serial.print("UP - "); Serial.println(middle_angle);
				}
			break;
			case 122:
				if (upper_angle < MAX_ANGLE) {
					upper_angle += SERVO_MOTION_UNIT;
					rotate_joint (&upper_ctx, upper_angle, 5);
					Serial.print("UP GRIPP - "); Serial.println(upper_angle);
				}
			break;
			case 120:
				if (upper_angle > MIN_ANGLE) {
					upper_angle -= SERVO_MOTION_UNIT;
					rotate_joint (&upper_ctx, upper_angle, 5);
					Serial.print("DOWN GRIPP - "); Serial.println(upper_angle);
				}
			break;
			case 91:
				if (gripper_state == GRIPPER_CLOSE) {
					open_grripper (&gripper_ctx, 25);
					gripper_state = GRIPPER_OPEN;
					Serial.println("OPEN");
				}
			break;
			case 93:
				if (gripper_state == GRIPPER_OPEN) {
					close_gripper (&gripper_ctx, 25);
					gripper_state = GRIPPER_CLOSE;
					Serial.println("CLOSE");
				}
			break;
			default:
				Serial.println(cmd);
			break;
		}
	}
}

int
convert_angle_to_pulse (int angle, int notches) {
	Serial.println (MIN_PULSE + angle * notches);
	return MIN_PULSE + angle * notches;
}

void
init_ethernet() {
	IPAddress ip;
	if (Ethernet.begin(mac) == 0) {
		Serial.println("dhcp failed");
		while(true);
	}

	Serial.println("#[ethernet] Initialized");

	ip = Ethernet.localIP();
	for (byte thisByte = 0; thisByte < 4; thisByte++) {
		Serial.print(ip[thisByte], DEC);
		Serial.print("."); 
	} Serial.println();

	/* give the ethernet shield a second to initialize */
	delay(1000);
}