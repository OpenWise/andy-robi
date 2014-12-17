#pragma once

#include "PracticalSocket.h"  // For Socket and SocketException
#include <BaseTsd.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <io.h>
#include <fcntl.h>
#include <windows.h>

#define MAGIC_NUMBER	0xC0FFEE
#define SET_POSITION	01
#define SET_RESOLUTION	02
#define READ_POSITION	03
#define MOVE_LEFT		04
#define	MOVE_RIGHT		05
#define	MOVE_FORWARD	06
#define	MOVE_BACKWARD	07
#define MOVE_UP			08
#define MOVE_DOWN		09
#define SET_HAND_WIDTH	10

static const WORD MAX_CONSOLE_LINES = 500;

typedef struct {
	UINT8 joint1;
	UINT8 joint2;
	UINT8 joint3;
	UINT8 joint4;
	UINT8 joint5;
	UINT8 joint6;
} joints_t;

typedef struct {
	UINT32 magic;
	UINT16 cmd;
	UINT8 data[16];
} packet_t;

class RobotHelper
{
public:
	joints_t ConvertXYZPtoJoints(int x, int y, int z, int pinch);


};