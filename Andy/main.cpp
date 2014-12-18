#include "pxcsession.h"
#include "pxccapture.h"
#include "pxchandmodule.h"
#include "pxchanddata.h"
#include "pxcsensemanager.h"
#include "pxcvideomodule.h"
#include "pxchandconfiguration.h"
#include "pxchanddata.h"
#include "RobotFunctions.h"
#include "PracticalSocket.h"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <Windows.h>
#include <cstdint>
#include "IKConverter.h"

#define MIN_ROBOT_X -0.16
#define MAX_ROBOT_X 0.16 
#define MIN_ROBOT_Y 0
#define MAX_ROBOT_Y 0.13
#define MIN_ROBOT_Z 0
#define MAX_ROBOT_Z 0.42 
#define MIN_CAMERA_X -0.3
#define MAX_CAMERA_X 0.3
#define MIN_CAMERA_Y -0.26
#define MAX_CAMERA_Y 0.26
#define MIN_CAMERA_Z 0.8
#define MAX_CAMERA_Z 0.2



using namespace std;
using namespace cv;

typedef struct{
	uint8_t base;
	uint8_t sholder;
	uint8_t elbow;
	uint8_t wrist;
	uint8_t gripper;
} angle_t;

typedef struct{
	uint32_t magic;
	uint8_t cmd;
	uint8_t data[16];
} pkt_t;

typedef struct{
	float x;
	float y;
	float z;
} robot_pnt_t;

PXCSession *g_session = 0;
bool g_stop = false;
volatile bool g_connected = false;
bool showNormalizedSkeleton = false;
bool showExtremityPoint = false;
float maxRangeValue = 1000;
int counter = 0;
bool isRobotConnected = false;

VideoCapture* cap = NULL;
Mat frame;
TCPSocket* sock = NULL;
IKConverter G_IKConverter;
int g_pinch = 1;

/* Socket Control*/
int TCPconnect(string servAddress, unsigned short echoServPort)
{

	try {
		// Establish connection with the echo server
		sock = new TCPSocket(servAddress, echoServPort);
	}
	catch (SocketException &e) {
		cerr << e.what() << endl;
		return 1;
	}

	return 0;
}

void TCPsend(char* msg)
{
	// Send the string to the echo server
	int stringLen = strlen(msg);   // Determine 
	cout << msg;
	sock->send(msg, stringLen);
}

void TCPsendBuff(uint8_t* eth_packet)
{
	// Send the string to the echo server
	int buffLen = 32;   // Determine cout << eth_packet;
	sock->send(eth_packet, buffLen);
}


void TCPReceive(int size)
{
	char echoBuffer[32 + 1];    // Buffer for echo string + \0
	int bytesReceived = 0;              // Bytes read on each recv()
	int totalBytesReceived = 0;         // Total bytes read Receive the same string back from the server
	cout << "Received: ";               // Setup to print the echoed string
	while (totalBytesReceived < size) {
		// Receive up to the buffer size bytes from the sender
		if ((bytesReceived = (sock->recv(echoBuffer, 32))) <= 0) {
			cerr << "Unable to read";
			exit(1);
		}
		totalBytesReceived += bytesReceived;     // Keep tally of total bytes
		echoBuffer[bytesReceived] = '\0';        // Terminate the string!
		cout << echoBuffer;                      // Print the echo buffer
	}
	cout << endl;
}

void setMaxRangeValue(float value)
{
	maxRangeValue = value;
}

/* Checking if sensor device connect or not */
static bool DisplayDeviceConnection(bool state)
{
	if (state) {
		if (!g_connected) printf("Device Reconnected\n");
		g_connected = true;
	}
	else {
		if (g_connected) printf("Device Disconnected\n");
		g_connected = false;
	}
	return g_connected;
}


//-----------------------------------------------------------------------
float getPitch(PXCPoint4DF32 point)
{
	vector<float> m_afTuple;
	m_afTuple.resize(4);
	m_afTuple[0] = point.w;
	m_afTuple[1] = point.x;
	m_afTuple[2] = point.y;
	m_afTuple[3] = point.z;

	float fTx = 2.0f*m_afTuple[1];
	float fTz = 2.0f*m_afTuple[3];
	float fTwx = fTx*m_afTuple[0];
	float fTxx = fTx*m_afTuple[1];
	float fTyz = fTz*m_afTuple[2];
	float fTzz = fTz*m_afTuple[3];
	return atan2(fTyz + fTwx, 1.0f - (fTxx + fTzz));
}


float ConvertRange(float oldValue, float oldMin, float oldMax, float newMin, float newMax)
{
	return (((oldValue - oldMin) * (newMax - newMin)) / (oldMax - oldMin)) + newMin;
}

robot_pnt_t ConvertCameraXYZ2RobotXYZ(float x, float y, float z)
{
	robot_pnt_t robotPoints;

	robotPoints.x = ConvertRange(x, MIN_CAMERA_X, MAX_CAMERA_X, MIN_ROBOT_X, MAX_ROBOT_X);
	float tempY = ConvertRange(z, MIN_CAMERA_Z, MAX_CAMERA_Z, MIN_ROBOT_Y, MAX_ROBOT_Y);
	robotPoints.y = (tempY < 0) ? 0 : tempY;
	float tempZ = ConvertRange(y, MIN_CAMERA_Y, MAX_CAMERA_Y, MIN_ROBOT_Z, MAX_ROBOT_Z);
	robotPoints.z = (tempZ < 0) ? 0 : tempZ;

	return robotPoints;
}

void MoveRobot(float x, float y, float z, int pinch, float pitch, uint8_t* ethernet_buff)
{
	pkt_t* packet_ptr = (pkt_t *)ethernet_buff;
	angle_t* angle_ptr = (angle_t *)packet_ptr->data;

	angle_ptr->base = ConvertRange(x, MIN_CAMERA_X, MAX_CAMERA_X, 0, 100);
	angle_ptr->sholder = ConvertRange(z, MIN_CAMERA_Z, MAX_CAMERA_Z, 60, 0);
	angle_ptr->elbow = ConvertRange(y, MIN_CAMERA_Y, MAX_CAMERA_Y, 100, 0);
	if (pitch > 100) pitch = 100;
	angle_ptr->wrist = ConvertRange(pitch, 0, -0.07, 100, 0);

	cout << pitch << "\n";

	cout << "base:" << (float)angle_ptr->base << " elbow: " << (float)angle_ptr->elbow << " shoulder: " << (float)angle_ptr->sholder << \
		" wrist:" << pitch << "\n";

	if (pinch < 33)  g_pinch = 0;
	else if (pinch > 66) g_pinch = 1;
	angle_ptr->gripper = g_pinch;

	packet_ptr->magic = 0xAABB;
	packet_ptr->cmd = 0x1;
}

void MoveRobotAngels(float x, float y, float z, int pinch, uint8_t* ethernet_buff)
{
	
	pkt_t* packet_ptr = (pkt_t *)ethernet_buff;
	angle_t* angle_ptr = (angle_t *)packet_ptr->data;
	ArmAngles angles;
	robot_pnt_t robotPoints = ConvertCameraXYZ2RobotXYZ(x, y, z);

	G_IKConverter.convertXYZtoServoAngles(robotPoints.x, robotPoints.y, robotPoints.z, 45, &angles);
	cout << "robot_x:" << robotPoints.x << "\trobot_y: " << robotPoints.y << "\trobot_z: " << robotPoints.z << "\n";

	//test base only
	angles.elb_angle_d = 60;
	angles.shl_angle_d = 0;
	angles.wri_angle_d = 0;

	angle_ptr->base = angles.bas_angle_d + 45;
	angle_ptr->elbow = angles.elb_angle_d;
	angle_ptr->sholder = angles.shl_angle_d;
	angle_ptr->wrist = angles.wri_angle_d;

	if (pinch < 33)  g_pinch = 0;
	else if (pinch > 66) g_pinch = 1;
	angle_ptr->gripper = g_pinch;
	
	/*angle_ptr->base = 45;
	angle_ptr->elbow = 45;
	angle_ptr->gripper = 45;
	angle_ptr->sholder = 45;
	angle_ptr->wrist = 45;*/

	cout << "base:" << angles.bas_angle_d << " elbow: " << angles.elb_angle_d << " shoulder: " << angles.shl_angle_d << \
		" wrist:" << angles.wri_angle_d << "\n";

	packet_ptr->magic = 0xAABB;
	packet_ptr->cmd = 0x1;
}

/* Displaying current frames hand joints */
static void ProcessJoints(PXCHandData *handAnalyzer, pxcI64 timeStamp = 0) {

	PXCHandData::JointData nodes[2][PXCHandData::NUMBER_OF_JOINTS] = {};
	PXCHandData::ExtremityData extremitiesPointsNodes[2][PXCHandData::NUMBER_OF_EXTREMITIES] = {};
	uint8_t ethernet_buff[32];
	//Iterate hands
	for (pxcI32 i = 0; i < handAnalyzer->QueryNumberOfHands(); i++)
	{
		//Get hand by time of appearence
		PXCHandData::IHand* handData;
		if (handAnalyzer->QueryHandData(PXCHandData::AccessOrderType::ACCESS_ORDER_BY_TIME, i, handData) == PXC_STATUS_NO_ERROR)
		{
			PXCHandData::JointData jointData;
			//Iterate Joints
			for (int j = 0; j < PXCHandData::NUMBER_OF_JOINTS; j++)
			{

				if (showNormalizedSkeleton == false)
				{
					handData->QueryTrackedJoint((PXCHandData::JointType)j, jointData);
				}
				else
				{
					handData->QueryNormalizedJoint((PXCHandData::JointType)j, jointData);
				}

				nodes[i][j] = jointData;

			}
			if (showExtremityPoint == true){
				for (int j = 0; j < PXCHandData::NUMBER_OF_EXTREMITIES; j++)
				{
					handData->QueryExtremityPoint((PXCHandData::ExtremityType)j, extremitiesPointsNodes[i][j]);
				}
			}
		}

		cout << "x: " << nodes[0][1].positionWorld.x << "\ty: " << nodes[0][0].positionWorld.y << "\tz: " << nodes[0][0].positionWorld.z << "\topeness: " << handData->QueryOpenness() << "\n";
		circle(frame, Point(frame.size().width - nodes[0][1].positionImage.x, nodes[0][1].positionImage.y), 1 / nodes[0][1].positionWorld.z * 15, (0, 0, 255 * handData->QueryOpenness() / 100 ), -1);
		if (counter++ >= 0)
		{
			float pitch = nodes[0][PXCHandData::JointType::JOINT_MIDDLE_BASE].positionWorld.z - nodes[0][PXCHandData::JointType::JOINT_WRIST].positionWorld.z;
			//MoveRobotAngels(nodes[0][1].positionWorld.x, nodes[0][1].positionWorld.y, nodes[0][1].positionWorld.z, handData->QueryOpenness(), ethernet_buff);
			MoveRobot(nodes[0][1].positionWorld.x, nodes[0][1].positionWorld.y, nodes[0][1].positionWorld.z, handData->QueryOpenness(), pitch, ethernet_buff);
			if (isRobotConnected) TCPsendBuff(ethernet_buff);
			counter = 0;
		}


		//DrawJoints(hwndDlg, nodes, extremitiesPointsNodes);
	}
}

int main(int argc, char* argv[])
{
	namedWindow("AndyCam", CV_WINDOW_AUTOSIZE);
	
	/*
	Vector<VideoCapture> cams;
	// try to find up to 5 video devices
	for (int device = 0; device < 5; device++)
	{
		VideoCapture capTest(device); // open the default camera
		if (capTest.isOpened())  // check if we succeeded
			cams.push_back(capTest);
	}

	// if detected more then 1 device let user choose which to use
	if (cams.size() > 1)
	{
		UINT id;
		cout << "Detected " << cams.size() << " video devices please choose which to use:\n";
		cin >> id;
		while (id > cams.size())
		{
			cout << "Device doesn't exist please choose another: \n";
			cin >> id;
		}
		cap = &cams[id - 1];
		for (UINT i = 0; i < cams.size(); i++)
		{
			if (i == (id - 1)) continue;
			cams[i].release();
		}
	}
	else
		cap = &cams[0]; // if only one device use it 
	
	cams.clear(); */

	cap = new VideoCapture(0);

	g_session = PXCSession_Create();
	if (g_session == NULL) {
		printf("Failed to create an SDK session");
		return 1;
	}

	string addr;
	UINT port;
	string ans;
	cout << "Connect to Robot?:\n";
	cin >> ans;
	if (ans == "y")
	{
		isRobotConnected = true;
		cout << "Please Enter Robot IP Address[192.168.0.100]:\n";
		cin >> addr;
		if (addr == "0") addr = "192.168.0.100";
		cout << "Please Enter Robot port[10500]:\n";
		cin >> port;
		if (port == 0) port = 10500;
		int err = TCPconnect(addr, port);
		if (err) return 1;
	}

	PXCSenseManager *pp = g_session->CreateSenseManager();
	if (!pp)
	{
		printf("Failed to create SenseManager\n");
		return 1;
	}

	bool sts = true;
	/* Set Module */
	pxcStatus status = pp->EnableHand(0);
	PXCHandModule *handAnalyzer = pp->QueryHand();
	if (handAnalyzer == NULL || status != pxcStatus::PXC_STATUS_NO_ERROR)
	{
		printf("Failed to pair the gesture module with I/O\n");
		return 1;
	}

	if (pp->Init() >= PXC_STATUS_NO_ERROR)
	{
		PXCHandData* outputData = handAnalyzer->CreateOutput();

		// IF IVCAM Set the following properties
		PXCCapture::Device *device = pp->QueryCaptureManager()->QueryDevice();
		PXCCapture::DeviceInfo dinfo;
		pp->QueryCaptureManager()->QueryDevice()->QueryDeviceInfo(&dinfo);
		if (dinfo.model == PXCCapture::DEVICE_MODEL_IVCAM)
		{
			device->SetDepthConfidenceThreshold(1);
			device->SetMirrorMode(PXCCapture::Device::MIRROR_MODE_DISABLED);
			device->SetIVCAMFilterOption(6);
		}

		setMaxRangeValue(pp->QueryCaptureManager()->QueryDevice()->QueryDepthSensorRange().max);

		// Hand Module Configuration
		PXCHandConfiguration* config = handAnalyzer->CreateActiveConfiguration();
		config->EnableNormalizedJoints(showNormalizedSkeleton);
		if (showExtremityPoint) config->SetTrackingMode(PXCHandData::TRACKING_MODE_EXTREMITIES);
		config->EnableAllAlerts();
		config->EnableSegmentationImage(true);

		config->ApplyChanges();
		config->Update();

		while (!g_stop)
		{

			bool bSuccess = cap->read(frame);
			if (!bSuccess) printf("Cannot read a frame from video stream\n");

			pxcStatus sts = pp->AcquireFrame(true);
			if (DisplayDeviceConnection(pp->IsConnected()))
			{
				if (sts < PXC_STATUS_NO_ERROR)
					break;

				outputData->Update();

				const PXCCapture::Sample *sample = pp->QueryHandSample();
				ProcessJoints(outputData);
			}
			pp->ReleaseFrame();

			imshow("AndyCam", frame);

			if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
			{
				cout << "esc key is pressed by user" << endl;
				break;
			}
		}

		config->Release();
		outputData->Release();
	}

	return 0;
}