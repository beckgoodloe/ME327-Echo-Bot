/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module: 

  helper.cpp

Description:
        
  Utilities that set the graphics state.

*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#if defined(WIN32) || defined(linux)
# include <GL/glut.h>
#elif defined(__APPLE__)
# include <GLUT/glut.h>
#endif

#include <HDU/hduMatrix.h>

extern void displayFunction(void);
extern void handleIdle(void);
extern void handleMenu(int);

/******************************************************************************
 Initializes GLUT.
******************************************************************************/
void initGlut(int argc, char* argv[])
{
    // Initialize GLUT.
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(500, 500);
    glutCreateWindow("GLUT/Coulomb Forces Demo");

    // Setup GLUT callbacks.
    glutDisplayFunc(displayFunction); 
    glutIdleFunc(handleIdle);

    // Setup GLUT popup menu.
    glutCreateMenu(handleMenu); 
    glutAddMenuEntry("Reverse Charge", 1);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
    glutAddMenuEntry("Quit", 0);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

/******************************************************************************    
 Uses the haptic device coordinate space as model space for graphics.
 Defines orthographic projection to fit it.
 LLB: Low, Left, Back point of device workspace.
 TRF: Top, Right, Front point of device workspace. 
******************************************************************************/
void initGraphics(const hduVector3Dd &LLB, const hduVector3Dd &TRF)
{
    // Setup perspective projection.
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity();

    HDdouble centerScreen[3];
    centerScreen[0] = (TRF[0] + LLB[0])/2.0;
    centerScreen[1] = (TRF[1] + LLB[1])/2.0;
    centerScreen[2] = (TRF[2] + LLB[2])/2.0;

    HDdouble screenDims[3];
    screenDims[0] = TRF[0] - LLB[0];
    screenDims[1] = TRF[1] - LLB[1];
    screenDims[2] = TRF[2] - LLB[2];

    HDdouble maxDimXY = screenDims[0] > screenDims[1] ? 
        screenDims[0] : screenDims[1];
    HDdouble maxDim = maxDimXY > screenDims[2] ? 
        maxDimXY : screenDims[2];
    maxDim /= 2.0;

    glOrtho(centerScreen[0]-maxDim, centerScreen[0]+maxDim, 
            centerScreen[1]-maxDim, centerScreen[1]+maxDim,
            centerScreen[2]-maxDim, centerScreen[2]+maxDim);
    
    glShadeModel(GL_SMOOTH);

    // Setup model transformations.
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);



}


/******************************************************************************    
 Sets up graphics pipeline, lights etc.
******************************************************************************/
void setupGraphicsState()
{
    glShadeModel(GL_SMOOTH);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHT_MODEL_TWO_SIDE);
    glShadeModel(GL_SMOOTH);
    
    GLfloat lightZeroPosition[] = { 10.0, 4.0, 100.0, 0.0 };
    GLfloat lightZeroColor[] = { 0.6, 0.6, 0.6, 1.0 }; // green-tinted.
    GLfloat lightOnePosition[] = { -1.0, -2.0, -100.0, 0.0 };
    GLfloat lightOneColor[] = { 0.6, 0.6, 0.6, 1.0 }; // red-tinted.
    
    GLfloat light_ambient[] = { 0.8, 0.8, 0.8, 1.0 }; // White diffuse light.
    GLfloat light_diffuse[] = { 0.0, 0.0, 0.0, 1.0 }; // White diffuse light.
    GLfloat light_position[] = { 0.0, 0.0, 100.0, 1.0 }; // Infinite light loc.
    
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightfv(GL_LIGHT0, GL_POSITION, lightZeroPosition);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

}

/******************************************************************************
 Draws the cartesian axes.
******************************************************************************/
void drawAxes(double axisLength)
{
    glDisable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    glLineWidth(2.0);
    
    glBegin(GL_LINES);
    for (int i = 0; i < 3; i++) 
    {
        float color[3] = { 0, 0, 0 };
        color[i] = 1.0;
        glColor3fv(color);
        
        float vertex[3] = {0, 0, 0};
        vertex[i] = axisLength;
        glVertex3fv(vertex);
        glVertex3f(0, 0, 0);
    } 
    glEnd();
}

/******************************************************************************
 Draws a sphere.
******************************************************************************/
void drawSphere(GLUquadricObj* pQuadObj, 
                const hduVector3Dd &position,
                const float color[4],
                double sphereRadius)
{
    glMatrixMode(GL_MODELVIEW); 
    glPushMatrix();
    glEnable(GL_LIGHTING);
    glColor4fv(color);
    glTranslatef(position[0], position[1], position[2]);
    gluSphere(pQuadObj, sphereRadius, 20, 20); 
    glPopMatrix();
}

/******************************************************************************
 Draws a plane.
******************************************************************************/
void drawPlane(GLUquadricObj* pQuadObj,
                     const double &floorHeight,
					 hduVector3Dd norm,
                     const float color[4])
	
{
	hduVector3Dd forward(0.0, 0.0, 1.0);
	hduVector3Dd cross = forward.crossProduct(norm);

    glMatrixMode(GL_MODELVIEW); 
    glPushMatrix();
	glEnable(GL_LIGHTING);
	glTranslatef(0.0, floorHeight, 0.0);
	glRotatef(acos(norm.dotProduct(forward))*180/3.14159,cross[0],cross[1],cross[2]);
    glColor4fv(color);
    gluDisk(pQuadObj, 0.0, 1000, 10, 1); 
    glPopMatrix();
}
/******************************************************************************
 Draws a cube.
******************************************************************************/
void drawMeshCube(GLUquadricObj* pQuadObj,
                     const hduVector3Dd boxCenter,
					 const GLfloat boxLength )
{

	    glBegin(GL_LINES);
		glVertex3f(-boxLength, boxLength, boxLength);
		glVertex3f(-boxLength, -boxLength, boxLength);
		glVertex3f(boxLength, -boxLength, boxLength);
		glVertex3f(boxLength, boxLength, boxLength);

		glVertex3f(boxLength, boxLength, -boxLength);
		glVertex3f(boxLength, -boxLength, -boxLength);
		glVertex3f(-boxLength, -boxLength, -boxLength);
		glVertex3f(-boxLength, boxLength, -boxLength);
		    glEnd();


}
/******************************************************************************
 Draws the force vector.
******************************************************************************/
void drawForceVector(GLUquadricObj* pQuadObj,
                     const hduVector3Dd &position,
                     const hduVector3Dd &forceVector,
                     double arrowThickness)
{
    glDisable(GL_LIGHTING);
    
    glPushMatrix();

    glTranslatef(position[0], position[1], position[2]);

    // Change the force magnitude/direction by rotating the force vector.
    // Calculate the rotation angle.
    hduVector3Dd unitForceVectorAxis = normalize(forceVector);
    hduVector3Dd zAxis( 0.0, 0.0, 1.0 );
    hduVector3Dd toolRotAxis = zAxis.crossProduct(unitForceVectorAxis);
        
    double toolRotAngle = acos(unitForceVectorAxis[2]);
    hduMatrix rotMatrix = hduMatrix::createRotation(toolRotAxis, 
                                                    toolRotAngle);

    double rotVals[4][4];
    rotMatrix.get(rotVals);
    glMultMatrixd((double*) rotVals);

    // The force arrow: composed of a cylinder and a cone.
    glColor3f( 0.2, 0.7, 0.2 );
    
    double strength = forceVector.magnitude();
    
    // Draw arrow shaft.
    gluCylinder(pQuadObj,arrowThickness, arrowThickness, strength, 16, 2); 
    glTranslatef(0, 0, strength);
    glColor3f(0.2, 0.8, 0.3);
    
    // Draw arrow head.
    gluCylinder(pQuadObj, arrowThickness*2, 0.0, strength*.15, 16, 2); 
    
    glPopMatrix();
}

/******************************************************************************/

/******************************************************************************
 GLUT callback for reshaping the window. This is the main place where the 
 viewing and workspace transforms get initialized.
******************************************************************************/
void glutReshape(int width, int height)
{
    static const double kFovY = 40;
    static const double kCanonicalSphereRadius = 100; //sqrt(3.0);

	static hduVector3Dd gCameraPosWC;
	static int gWindowWidth, gWindowHeight;
	static const double kPI = 3.1415926535897932384626433832795;
	
    glViewport(0, 0, width, height);
    gWindowWidth = width;
    gWindowHeight = height;

    /* Compute the viewing parameters based on a fixed fov and viewing
       sphere enclosing a canonical box centered at the origin. */

    double nearDist = kCanonicalSphereRadius / tan((kFovY / 2.0) * kPI / 180.0);
    double farDist = nearDist + 2.0 * kCanonicalSphereRadius;
    double aspect = (double) width / height;
   
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(kFovY, aspect, nearDist, farDist);

    /* Place the camera down the Z axis looking at the origin. */
    gCameraPosWC[0] = 0;
    gCameraPosWC[1] = 0;
    gCameraPosWC[2] = nearDist + kCanonicalSphereRadius;
 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();            
    gluLookAt(gCameraPosWC[0], gCameraPosWC[1], gCameraPosWC[2],
              0, 0, 0,
              0, 1, 0);
     glutPostRedisplay();
}