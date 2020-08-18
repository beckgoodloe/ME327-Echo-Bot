/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

Module Name:

main.cpp

Adapted from SensAble Example Code by:
Julie Walker
April 2019

Description:

This file includes all graphics functionality for running several different haptics
environments. Students can fill in the functions partA()...part(E) to generate
appropriate haptic effects for each part. The graphics and all back-end haptic
controls are handled independently of those functions to be edited.

*******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <conio.h>
#include <vector>
#include <math.h>

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include "helper.h"

#define signum(x) ((x > 0)?(1):(-1))

static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

// Glut callback functions used by helper.cpp
void displayFunction(void);
void handleIdle(void);
void handleMenu(void);

// Callbacks and helpful functions in this file
void keyboardCallback(unsigned char key, int xmouse, int ymouse);
void runLoop(void);
void exitHandler(void);
//double dot_product(double[3],double[3]);
//double* cross_product(double[3],double[3]);
HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData);

/* Haptic device record. */
struct DeviceDisplayState
{
	HHD m_hHD;
	hduVector3Dd position;
	hduVector3Dd force;
};

// Algorithms for each assignment part
hduVector3Dd partZ(hduVector3Dd pos);
hduVector3Dd partA(hduVector3Dd pos);
hduVector3Dd partB(hduVector3Dd pos);
hduVector3Dd partC(hduVector3Dd pos);
hduVector3Dd partD(hduVector3Dd pos);
hduVector3Dd partE(hduVector3Dd pos);
hduVector3Dd partF(hduVector3Dd pos);

// Variables for all parts of assignment
char part;
bool proxy = 1;
static double cursorRadius = 5.0;

// For selecting parts from keyboard keys
#define PART_Z 'Z'
#define PART_A 'A'
#define PART_B 'B'
#define PART_C 'C'
#define PART_D 'D'
#define PART_E 'E'
#define PART_F 'F'	// optional

// Variables for each part
// PART A
double floorHeight = -10.0;

// PART B
// components of plane normal
double a = 0.0;
double b = 1.0;
double c = 0.0;

// PART C AND E
hduVector3Dd boxCenter(0.0, 0.0, 0.0);
GLfloat sideLength = 100;

// PART D
double bigSphereRadius = 30.0;
hduVector3Dd proxyPos(0,0,0);

// PART E
hduVector3Dd spherePos(0.0, 0.0, 0.0), sphereVel(20.0, 10.0, 0.0); //[mm] and [mm/s]
double sphereRadius = 15.0;	// Radius of sphere [mm]
const float sphereColor[4] = {.8, .8, 0.0, .8};


/******************************************************************************
Main function.  DO NOT EDIT
******************************************************************************/
int main(int argc, char* argv[])
{


	HDErrorInfo error;

	printf("Starting application\n");


	// Initialize the device.  This needs to be called before any other
	// actions on the device are performed.
	ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		//exit(-1);
		return 0;
	}

	printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));

	hdEnable(HD_FORCE_OUTPUT);
	hdEnable(HD_MAX_FORCE_CLAMPING);

	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to start scheduler");
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}

	atexit(exitHandler);

	initGlut(argc, argv);

	// Get the workspace dimensions.
	HDdouble maxWorkspace[6];
	hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);

	// Low/left/back point of device workspace.
	hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
	// Top/right/front point of device workspace.
	hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
	initGraphics(LLB, TRF);
	glutKeyboardFunc(keyboardCallback);
	glutReshapeFunc(glutReshape);

	// Application loop.
	runLoop();

	printf("Done\n");
	return 0;
}

/*******************************************************************************
PART Z: Outside a Sphere
-------- CLASS DEMO ----------------
*******************************************************************************/
hduVector3Dd partZ(hduVector3Dd pos)
{
	// Variables for you to use
	hduVector3Dd forceVec(0,0,0);   // [N]
	double kWall = 0.25;		    // [N/mm]
	bigSphereRadius = 70.0;         // [mm], global variable used in graphics
	proxyPos = pos;                 // 3 element vector, global variable used in graphics
	cursorRadius = 5.0;


	///////////////////// Solution Code /////////////////////////////////

	// calculate forceVec and proxyPos if the cursor is penetrating the sphere:
	if(pos[0]*pos[0]+pos[1]*pos[1]+pos[2]*pos[2] > (bigSphereRadius-cursorRadius)*(bigSphereRadius-cursorRadius) ){
		proxyPos = (bigSphereRadius-cursorRadius)*normalize(pos);
		forceVec = -kWall*(pos-proxyPos);
	}
	// calculate forceVec and proxyPos if the cursor is not penetrating the sphere:
	else{
		forceVec.set(0.0,0.0,0.0);
		proxyPos = pos;
	}
	/////////////////////////////////////////////////////////////////////

	return forceVec;
}


/*******************************************************************************
PART A: a virtual floor
// x is positive to the right, y is positive up, and z is positive toward the user
*******************************************************************************/
hduVector3Dd partA(hduVector3Dd pos)
{
	// Variables for you to use
	double kWall = 0.5;		// [N/mm]
	floorHeight = -10.0;				// [mm], global variable used in graphics
	cursorRadius = 5.0;					// [mm]

	// Variables you must calculate
	hduVector3Dd forceVec(0,0,0);		// [N]
	proxyPos = pos;                     // 3 element vector, global variable used in graphics

	/////////////////////////////////////////////////////////////////////
	/////////////////// START HERE //////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	// calculate forceVec and proxyPos if the cursor is penetrating the floor:
	if(true){	// change to a condition that depends on the input variable "pos"
		if(pos[1] < floorHeight+cursorRadius){
			double outputForce = kWall * ((floorHeight+cursorRadius) - pos[1]);
			forceVec.set(0,outputForce,0);
			proxyPos[0] = pos[0];
			proxyPos[1] = floorHeight+cursorRadius;
			proxyPos[2] = pos[2];
		}else{
			forceVec.set(0,0,0);
		}

		// your code here:
		// forceVec = ?? or forceVec.set(??, ??, ??)
		// proxyPos = ??
	}
	// calculate forceVec and proxyPos if the cursor is not penetrating the floor:
	else{

		// your code here:
		// forceVec = ?? or forceVec.set(??, ??, ??)
		// proxyPos = ??
	}

	/////////////////////////////////////////////////////////////////////
	/////////////////// END HERE ////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	return forceVec;
}

/*******************************************************************************
PART B: a slanted virtual floor
// helpful website: https://mathinsight.org/distance_point_plane
*******************************************************************************/
hduVector3Dd partB(hduVector3Dd pos)
{

	// Variables for you to use
	double kWall = 0.5;		// [N/mm]
	floorHeight = 0.0;					// [mm], global variable used in graphics
	cursorRadius = 5.0;					// [mm]

	// Variables you must calculate
	hduVector3Dd forceVec(0,0,0);		//[N]
	proxyPos = pos;						// 3 element vector, global variable used in graphics

	/////////////////////////////////////////////////////////////////////
	/////////////////// START HERE //////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	// select values for the plane normal vector if you want to change the slant
	// plane equation:  d = ax+by+cz;
	a = 0.5;
	b = 0.9;
	c = 0.1;

	hduVector3Dd n2 = hduVector3Dd(a , b , c);
	hduVector3Dd n2_normal = normalize(n2);
	hduVector3Dd r = hduVector3Dd(pos[0]-a,pos[1]-b,pos[2]-c);
	if(hduVecDotProduct(r,n2) < 0){
		double d = abs(a*pos[0] + b * pos[1] + c * pos[2])/sqrt(a*a+b*b+c*c);
		forceVec.set(kWall * d * a, kWall * d * b, kWall*d*c);
		//hduVecCrossProduct(proxyPos,pos,n2);
		proxyPos = pos + (d * n2);
	}else{
		forceVec.set(0,0,0);
	}
	proxyPos = proxyPos + (cursorRadius * n2);



	///// DON'T CHANGE //////////////////////////////
	hduVector3Dd n(a, b, c);	// plane normal vector
	n.normalize();
	////////////////////////////////////////////////


	// calculate forceVec and proxyPos if the cursor is penetrating the plane:
	if(true){    // change "true" to a condition that depends on the input variable "pos"
		// forceVec = ?? or forceVec.set(??, ??, ??)
		// proxyPos = ??
	}

	// calculate forceVec and proxyPos if the cursor is not penetrating the plane:
	else{
		// forceVec = ?? or forceVec.set(??, ??, ??)
		// proxyPos = ??
	}
	/////////////////////////////////////////////////////////////////////
	/////////////////// END HERE ////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	return forceVec;
}

/*******************************************************************************
PART C: Inside a Box
*******************************************************************************/
hduVector3Dd partC(hduVector3Dd pos)
{
	// Variables for you to use
	double kWall = 0.5;					// [N/mm]
	sideLength = 100.0;					// [mm]
	cursorRadius = 5.0;					// [mm]

	// Variables you must calculate
	hduVector3Dd forceVec(0,0,0);		//[N]
	proxyPos = pos;						// 3 element vector, global variable used in graphics

	/////////////////////////////////////////////////////////////////////
	/////////////////// START HERE //////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	// calculate forceVec and proxyPos if the cursor is penetrating the walls
	// of the box:

	double forceX;
	double forceY;
	double forceZ;

	double positionXAbs = abs(pos[0]) - abs(sideLength / 2) + cursorRadius;
	forceX = -kWall * (positionXAbs < 0) ? (0) : ((pos[0] > 0) ? (-positionXAbs) : (positionXAbs));
	double positionYAbs = abs(pos[1]) - abs(sideLength / 2) + cursorRadius;
	forceY = -kWall * (positionYAbs < 0) ? (0) : ((pos[1] > 0) ? (-positionYAbs) : (positionYAbs));
	double positionZAbs = abs(pos[2]) - abs(sideLength / 2) + cursorRadius;
	forceZ = -kWall * (positionZAbs < 0) ? (0) : ((pos[2] > 0) ? (-positionZAbs) : (positionZAbs));


	forceVec.set(forceX, forceY, forceZ);

	proxyPos[0] = (forceX == 0) ? (pos[0]) : ((pos[0] > 0)?(sideLength / 2 - cursorRadius):(-((sideLength / 2) - cursorRadius)));
	proxyPos[1] = (forceY == 0) ? (pos[1]) : ((pos[1] > 0)?(sideLength / 2 - cursorRadius):(-((sideLength / 2) - cursorRadius)));
	proxyPos[2] = (forceZ == 0) ? (pos[2]) : ((pos[2] > 0)?(sideLength / 2 - cursorRadius):(-((sideLength / 2) - cursorRadius)));

	// calculate forceVec and proxyPos if the cursor is inside the box:
	// forceVec.set(??,??,??);
	// proxyPos = ??


	/////////////////////////////////////////////////////////////////////
	/////////////////// END HERE ////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	return forceVec;
}


/*******************************************************************************
PART D: Outside a Sphere
*******************************************************************************/
hduVector3Dd partD(hduVector3Dd pos)
{
	// Variables for you to use
	double kSphere = 0.5;					// [N/mm]
	bigSphereRadius = 30.0;					// [mm] global variable used in graphics
	cursorRadius = 5.0;						// [mm]

	// Variables you must calculate
	hduVector3Dd forceVec(0,0,0);			//[N]
	proxyPos = pos;							// 3 element vector, global variable used in graphics

	/////////////////////////////////////////////////////////////////////
	/////////////////// START HERE //////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	// calculate forceVec and proxyPos if the cursor is penetrating the sphere:
	double r = sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
	if(r<(bigSphereRadius+cursorRadius)){
		forceVec.set(pos[0]* kSphere * ((bigSphereRadius + cursorRadius) - r) / r,pos[1]*kSphere*((bigSphereRadius + cursorRadius) - r)/r,pos[2] * kSphere * ((bigSphereRadius + cursorRadius)-r)/r);
		proxyPos[0] = pos[0] * (bigSphereRadius + cursorRadius) / r;
		proxyPos[1] = pos[1] * (bigSphereRadius + cursorRadius) / r;
		proxyPos[2] = pos[2] * (bigSphereRadius + cursorRadius) / r;
	}
	else{
		forceVec.set(0,0,0);
	}
	// calculate forceVec and proxyPos if the cursor is not penetrating the sphere:

	/////////////////////////////////////////////////////////////////////
	/////////////////// END HERE ////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	return forceVec;
}

/*******************************************************************************
PART E: Dynamic Object Interactions
*******************************************************************************/
hduVector3Dd partE(hduVector3Dd pos)
{
	// Variables for you to use
	double kWall_cursor       = 0.5;  // Surface stiffness with cursor [N/mm]
	const double kWall_Sphere = 4.00; // Surface stiffness with sphere [N/mm]
	const double Ksphere      = 0.48;	// surface stiffness with cursor [n/mm]
	const double Bsphere      = 0.001; // sphere damping [n-s/mm]
	const double sphereMass   = 0.005; // Sphere mass [Kg]
	cursorRadius              = 5.0;	 // [mm]
	sphereRadius              = 15.0;	 // [mm]
  sideLength                = 100;	 // [mm]
	proxyPos = pos;

	// Variables you must calculate:
	hduVector3Dd cursor_wall_force(0,0,0);		// force on cursor from wall [N]
	hduVector3Dd cursor_sphere_force(0,0,0);	// force on cursor from sphere [N]
	hduVector3Dd sphere_cursor_force(0,0,0);	// force on sphere from cursor [N]
	hduVector3Dd sphere_wall_force(0,0,0);		// force on sphere from wall [N]
	hduVector3Dd sphere_force(0,0,0);			// total force on sphere [N]
	hduVector3Dd forceVec(0,0,0);				// force to output on omni[N]

	//defined globally:
	hduVector3Dd spherePos;
	hduVector3Dd sphereVel;

	/////////////////////////////////////////////////////////////////////
	/////////////////// START HERE //////////////////////////////////////
	/////////////////////////////////////////////////////////////////////

	// helpful intermediate variables
	double penetration_depth = 0.0;
	double penetration_direction;
	hduVector3Dd center_displacement, sphere_penetration_dir;

	// Wall + cursor interaction ///////////////////////////////////////////////////
	// Occurs when user is interacting with just the edges of the box. The user
	// feels a linear spring force proportional to the surface penetration.
	// Similar to part C.
	/////////////////////////////////////////////////////////////////////////////

	// Apply a linear spring force out of each penetrated wall
	// (use pos, sideLength, cursorRadius, kWall)
	// Keep proxy from penetrating box: (use pos, sideLength, cursorRadius)
	double proxy_bound = sideLength/2.0 - cursorRadius;
	for(int i=0; i<3; i++) {
		penetration_depth = abs(pos[i]) - proxy_bound;
		if ( penetration_depth > 0.0 ) {
			cursor_wall_force[i] = signum(pos[i]) * KWall_cursor * penetration_depth;
		}
	}

	// Truncate proxy position to prevent penetration of the box
	for(int i=0; i<3; i++) {
		if(abs(proxyPos[i]) > proxy_bound) {
			proxyPos[i] = signum(proxyPos[i]) * proxy_bound;
		}
	}


	// Sphere + cursor interaction /////////////////////////////////////////////////
	// Occurs when user is interacting with a free-flying sphere. In this case,
	// a pair of equal and opposite forces are applied to both the simulated
	// sphere and the cursor.
	// Simliar to part D.
	///////////////////////////////////////////////////////////////////////////////

	// Done for you: the vector from the cursor to the center of the sphere
	center_displacement = spherePos - pos;
	sphere_penetration_dir = center_displacement;
	sphere_penetration_dir.normalize();

	// Compute the penetration depth (using sphereRadius, cursorRadius)
	penetration_depth =  (sphereRadius + cursorRadius) - sphere_penetration_dir.magnitude();

	// Apply a linear spring force out of the surface (use KSphere)
	if (penetration_depth > 0.0) {
		cursor_sphere_force =  penetration_depth * KSphere;
	}

	// Equal and opposite force on sphere from cursor
	sphere_cursor_force = -1 * cursor_sphere_force;

	// Keep proxyPos outside sphere
	if(penetration_depth > 0.0){
		proxyPos = proxyPos - (sphere_penetration_dir * penetration_depth);
	}


	// Sphere + Wall interaction ////////////////////////////////////////////////
	// Occurs when the sphere penetrates the wall.
	// Similar to Part C.
	/////////////////////////////////////////////////////////////////////////////

	// Check if sphere is free-flying
	double sphereBound = sideLength/2.0 - sphereRadius;
	for(int i=0; i<3; i++) {
		// Compute distance from sphere to the side of the box
		penetration_depth = abs(pos[i]) - sphereBound;

		// Compute force from the surface in this axis (use kWall_Sphere)
		if ( penetration_depth > 0.0 ) {
		  sphere_wall_force[i] = signum(pos[i]) * penetration_depth * kWall_Sphere;
		}
	}

	// Don't worry about proxy for big sphere because kWall_Sphere is very high

	// Net Forces ///////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////

	// Compute net cursor force to be output on the haptic device
	forceVec = cursor_wall_force + cursor_sphere_force;

	// Compute net sphere force (include Bsphere to damp sphere motion)
	hduVector3Dd damping = -Bsphere * sphereVel;
	sphere_force = sphere_cursor_force + sphere_wall_force + damping;

	// Sphere dynamics //////////////////////////////////////////////////////////
	// Integrate the accelerations and velocities of the sphere.
	/////////////////////////////////////////////////////////////////////////////

	// Statically declare integration timestep
	double rate = 1000.0;
	hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &rate);
	double tstep = 1.0/rate;

	// Compute sphere acceleration ([m/s/s])
	hduVector3Dd sphereAcc = (sphere_force / sphereMass) * tstep;

	// Integrate state variables:
	// (global variables used in next iteration and in graphics)
	spherePos += sphereVel * tstep;
	sphereVel += sphereAcc * tstep;

	/////////////////////////////////////////////////////////////////////
	/////////////////// END HERE ////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////


	return forceVec;
}


/*******************************************************************************
PART F (Optional): Try out your own haptic effects! No graphics
*******************************************************************************/
hduVector3Dd partF(hduVector3Dd pos)
{
	// Variables you must calculate
	hduVector3Dd forceVec(0,0,0);			//[N]
	return forceVec;
}



/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//////////////////////  HELPER FUNCTIONS   //////////////////////////////
//////////////////////  DO NOT EDIT BELOW  //////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////





/*******************************************************************************
Graphics main loop function.
------------ DO NOT EDIT ------------
*******************************************************************************/
void displayFunction(void)
{
	// Setup model transformations.
	glMatrixMode(GL_MODELVIEW);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPushMatrix();
	glEnable( GL_COLOR_MATERIAL );
	setupGraphicsState();
	static const float cursorColor[4] = { .8, .2, .2, .8 };

	// Get the current position of end effector.
	DeviceDisplayState state;
	hdScheduleSynchronous(DeviceStateCallback, &state,
		HD_MIN_SCHEDULER_PRIORITY);


	GLUquadricObj* pQuadObj = gluNewQuadric();

	static const float floorColor[4] = {.8, .8, 0.0, .8};
	hduVector3Dd horizontalPlaneNormal(0.0, 0.98, 0.02);
	horizontalPlaneNormal.normalize();
	double sphere_proxy_bound;
	double proxy_bound;
	hduVector3Dd cursor_orbit = state.position - spherePos;
	hduVector3Dd cursor_orbit_dir = cursor_orbit;


	switch(part){
	case PART_Z:
		//draw plane
		//drawSphere(pQuadObj,hduVector3Dd(0,0,0),floorColor,bigSphereRadius);
		glutWireSphere(bigSphereRadius,15, 15);
		break;
	case PART_A:
		//draw plane
		drawPlane(pQuadObj, floorHeight, horizontalPlaneNormal, floorColor );
		break;
	case PART_B:
		//draw slanted plane
		drawPlane(pQuadObj, floorHeight, hduVector3Dd(a,b,c), floorColor );
		break;
	case PART_C:
		// draw box
		glEnable(GL_LIGHTING);
		glColor4fv(cursorColor);
		glutWireCube(sideLength);
		break;
	case PART_D:
		drawSphere(pQuadObj,hduVector3Dd(0,0,0),floorColor,bigSphereRadius);
		break;
	case PART_E:

		// Draw the box
		glutWireCube(sideLength);

		// Draw the  sphere
		drawSphere(pQuadObj, spherePos, sphereColor, sphereRadius);

		break;
	default:
		proxyPos = state.position;
		break;
	}

	// toggle whether the proxy is different from the device position
	if(!proxy){
		proxyPos = state.position;
	}

	// Draw a sphere to represent the haptic cursor
	drawSphere(pQuadObj,
		proxyPos,
		cursorColor,
		cursorRadius);

	gluDeleteQuadric(pQuadObj);

	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glutSwapBuffers();
}


/*******************************************************************************
Main callback that calculates and sets the force.
------------ DO NOT EDIT ------------
*******************************************************************************/
HDCallbackCode HDCALLBACK HapticsCallback(void *data)
{
	HHD hHD = hdGetCurrentDevice();

	hdBeginFrame(hHD);

	hduVector3Dd pos;
	hdGetDoublev(HD_CURRENT_POSITION,pos);
	hduVector3Dd forceVec;

	switch (part) {
	case PART_Z:
		forceVec = partZ(pos);
		break;
	case PART_A:
		forceVec = partA(pos);
		break;
	case PART_B:
		forceVec = partB(pos);
		break;
	case PART_C:
		forceVec = partC(pos);
		break;
	case PART_D:
		forceVec = partD(pos);
		break;
	case PART_E:
		forceVec = partE(pos);
		break;
	case PART_F:
		forceVec = partF(pos);
		break;
	default:
		forceVec.set(0.0, 0.0, 0.0);
		break;
	}

	//forceVec.set(0.0,0.0,0.0);
	hdSetDoublev(HD_CURRENT_FORCE, forceVec);

	hdEndFrame(hHD);

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error during scheduler callback");
		if (hduIsSchedulerError(&error))
		{
			return HD_CALLBACK_DONE;
		}
	}

	return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
Client callback used by the graphics main loop function.
Use this callback synchronously.
Gets data, in a thread safe manner, that is constantly being modified by the
haptics thread.
------------ DO NOT EDIT ------------
*******************************************************************************/
HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData)
{
	DeviceDisplayState *pDisplayState =
		static_cast<DeviceDisplayState *>(pUserData);

	hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->position);
	hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);

	// execute this only once.
	return HD_CALLBACK_DONE;
}


/*******************************************************************************
Schedules the force callback.
------------ DO NOT EDIT ------------
*******************************************************************************/
void runLoop()
{

	gSchedulerCallback = hdScheduleAsynchronous(
		HapticsCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "\nPress any key to quit.\n");
		getchar();
		exit(-1);
	}


	glutMainLoop(); // Enter GLUT main loop.
}

/******************************************************************************
This handler gets called when the process is exiting. Ensures that HDAPI is
properly shutdown
------------ DO NOT EDIT ------------
******************************************************************************/
void exitHandler()
{
	hdStopScheduler();
	hdUnschedule(gSchedulerCallback);

	if (ghHD != HD_INVALID_HANDLE)
	{
		hdDisableDevice(ghHD);
		ghHD = HD_INVALID_HANDLE;
	}
}


/******************************************************************************/
void keyboardCallback(unsigned char key, int xmouse, int ymouse)
{
	switch (key) {
	case 'Z':
	case 'z':
		printf("Rendering controller for Part Z.\n");
		part = PART_Z;
		break;
	case 'a':
	case 'A':
		printf("Rendering controller for Part A.\n");
		part = PART_A;
		break;
	case 'b':
	case 'B':
		printf("Rendering controller for Part B.\n");
		part = PART_B;
		break;
	case 'c':
	case 'C':
		printf("Rendering controller for Part C.\n");
		part = PART_C;
		break;
	case 'd':
	case 'D':
		printf("Rendering controller for Part D.\n");
		part = PART_D;
		break;
	case 'e':
	case 'E':
		printf("Rendering controller for Part E.\n");
		part = PART_E;
		break;
	case 'p':
	case 'P':
		proxy = !proxy;
		printf("Toggling Proxy.\n");
		break;
	default:
		printf("Rendering nothing.\n");
		part = 0;
		break;
	}
}

/*******************************************************************************
Called periodically by the GLUT framework.
*******************************************************************************/
void handleIdle(void)
{
	glutPostRedisplay();

	if (!hdWaitForCompletion(gSchedulerCallback, HD_WAIT_CHECK_STATUS))
	{
		printf("The main scheduler callback has exited\n");
		printf("Press any key to quit.\n");
		getchar();
		exit(-1);
	}
}

/******************************************************************************
Popup menu handler
******************************************************************************/
void handleMenu(int ID)
{
	switch(ID)
	{
	case 0:
		exit(0);
		break;
	case 1:

		break;
	}
}
