/*********************************/
/* CS 590CGS Lab framework        */
/* (C) Bedrich Benes 2020        */
/* bbenes ~ at ~ purdue.edu      */
/* Press +,- to add/remove points*/
/*       r to randomize          */
/*       s to change rotation    */
/*       c to render curve       */
/*       t to render tangents    */
/*       p to render points      */
/*       s to change rotation    */
/*********************************/

#include <stdio.h>
#include <iostream>
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <string>
#include <vector>			//Standard template library class
#include <GL/freeglut.h>
#include <cassert>

#include <glm/glm.hpp>
#include <glm/vec3.hpp>

//in house created libraries
#include "trackball.h"
#include "lab03.h"

#pragma warning(disable : 4996)
#pragma comment(lib, "freeglut.lib")

using namespace std;
using glm::vec3;

//some trackball variables -> used in mouse feedback
TrackBallC trackball;
bool mouseLeft, mouseMid, mouseRight;


GLuint points=0;  //number of points to display the object
int steps=20;     //# of subdivisions
bool needRedisplay=false;
GLfloat  sign=+1; //direction of rotation
const GLfloat defaultIncrement=0.7f; //speed of rotation
GLfloat  angleIncrement=defaultIncrement;

/*********************************
	Lab 3 related
**********************************/
int subdivision_num = 10;
vector <vec3> v;  
vector <vec3> tree_box;
vector <vec3> visualize_points;

//window size
GLint wWindow=1200;
GLint hWindow=800;

//this defines what will be rendered
//see Key() how is it controlled
bool tangentsFlag = false;
bool pointsFlag = true;
bool curveFlag = true;

/*********************************
Some OpenGL-related functions DO NOT TOUCH
**********************************/
//displays the text message in the GL window
void GLMessage(char *message)
{
	static int i;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0.f, 100.f, 0.f, 100.f);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glColor3ub(0, 0, 255);
	glRasterPos2i(10, 10);
	for (i = 0; i<(int)strlen(message); i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, message[i]);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

//called when a window is reshaped
void Reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glEnable(GL_DEPTH_TEST);
	//remember the settings for the camera
	wWindow = w;
	hWindow = h;
}

//Some simple rendering routines using old fixed-pipeline OpenGL
//draws line from a to b with color 
void DrawLine(vec3 a, vec3 b, vec3 color= vec3(0.0f,0.0f,0.0f)) {

	glColor3fv(&color[0]);
	glBegin(GL_LINES);
		glVertex3fv(&a[0]);
		glVertex3fv(&b[0]);
	glEnd();
}

//draws point at a with color 
void DrawPoint(vec3 a, vec3 color) {

	glColor3fv(&color[0]);
	glPointSize(5);
	glBegin(GL_POINTS);
	glVertex3fv(&a[0]);
	glEnd();
}

/**********************
LAB related MODIFY
***********************/

// Call THIS for a new curve. It clears the old one first
void InitArray(int n)
{
	v.clear();
}

std::shared_ptr<tree_node> tree_head;
polygon_mesh result_mesh;
void init_base_tree() {
	// linked list to construct the trees
	tree_head = std::make_shared<tree_node>(nullptr, 3.0,3.0,3.0, vec3(0.0f), 0.0f);
	
	//   e  f
	//   | /
	// c d
	// |/
	// b
	// |
	// a
	// |
	// head

	std::shared_ptr<tree_node> a = std::make_shared<tree_node>(tree_head, 2.0, 2.0, 2.0, vec3(1.0f, 0.0f, 0.0f), 0.0f); tree_head->add_child(a);
	std::shared_ptr<tree_node> b = std::make_shared<tree_node>(a, 2.0, 4.0, 2.0, vec3(1.0f, 0.0f, 0.0f), 0.0f); a->add_child(b);
	std::shared_ptr<tree_node> c = std::make_shared<tree_node>(b, 1.0, 4.0, 1.0, vec3(1.0f, 0.0f, 0.0f), 30.0f); b->add_child(c);
	std::shared_ptr<tree_node> d = std::make_shared<tree_node>(b, 1.0, 6.0, 1.0, vec3(1.0f, 0.0f, 0.0f), -30.0f); b->add_child(d);
	std::shared_ptr<tree_node> e = std::make_shared<tree_node>(d, 0.5, 3.0, 0.5, vec3(0.0f, 0.0f, 1.0f), 45.0f); d->add_child(e);
	std::shared_ptr<tree_node> f = std::make_shared<tree_node>(d, 0.5, 3.0, 0.5, vec3(0.0f, 0.0f, 1.0f), -45.0f); d->add_child(f);

	tree2mesh(tree_head, 1, result_mesh);
	result_mesh.normalize();
}

void draw_polygon_mesh(polygon_mesh &mesh) {
	for(size_t i= 0; i < mesh.verts.size()/2; ++i) {
		DrawLine(mesh.verts[2 * i + 0], mesh.verts[2 * i + 1]);
	}
}

//display coordinate system
void CoordSyst() {
	vec3 a, b, c;
	vec3 origin(0, 0, 0);
	vec3 red(1, 0, 0), green(0, 1, 0), blue(0, 0, 1), almostBlack(0.1f, 0.1f, 0.1f), yellow(1, 1, 0);

	//draw the coordinate system 
	a = vec3(1, 0, 0);
	b = vec3(0, 1, 0);
	c = glm::cross(a, b); //use cross product to find the last vector
	glLineWidth(1);
	DrawLine(origin, a, red);
	DrawLine(origin, b, green);
	DrawLine(origin, c, blue);
	glLineWidth(1);

}

void Lab03() {
	vec3 origin(0, 0, 0);
	vec3 red(1, 0, 0), green(0, 1, 0), blue(0, 0, 1), almostBlack(0.1f, 0.1f, 0.1f), yellow(1, 1, 0);

	CoordSyst();
	//draw the points
	if (pointsFlag) {
		for (unsigned int i = 0; i < v.size(); i++) {
			DrawPoint(v[i], blue);
		}
	}

	// visualize box
	draw_polygon_mesh(result_mesh);
}

//the main rendering function
void RenderObjects()
{
	glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
	//set camera
	glMatrixMode(GL_MODELVIEW);
	trackball.Set3DViewCamera();
	//call the student's code from here
	Lab03();
}

//Add here if you want to control some global behavior
//see the pointFlag and how is it used
void Kbd(unsigned char a, int x, int y)//keyboard callback
{
	switch (a)
	{
	case 27: exit(0); break;
	case 't': tangentsFlag = !tangentsFlag; break;
	case 'p': pointsFlag = !pointsFlag; break;
	case 'c': curveFlag = !curveFlag; break;
	case 32: {
		if (angleIncrement == 0) angleIncrement = defaultIncrement;
		else angleIncrement = 0;
		break;
	}
	case 's': {sign = -sign; break; }
	case '-': {
			subdivision_num--;
			subdivision_num = std::max(1, subdivision_num);
		break;
	}
	case '+': {
			subdivision_num++;
		break;
	}
	}
	cout << "[# of subdivision]=[" << subdivision_num << "]" << endl;

	glutPostRedisplay();
}


/*******************
OpenGL code. Do not touch.
******************/
void Idle(void)
{
}

void Display(void)
{
	glClearColor(0.5f, 0.5f, 0.5f, 1); //background color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	GLMessage("Lab 2 - CS 590CGS");
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40, (GLfloat)wWindow / (GLfloat)hWindow, 0.01, 100); //set the camera
	glMatrixMode(GL_MODELVIEW); //set the scene
	glLoadIdentity();
	gluLookAt(0, 10, 10, 0, 0, 0, 0, 1, 0); //set where the camera is looking at and from. 
	static GLfloat angle = 0;
	angle += angleIncrement;
	if (angle >= 360.f) angle = 0.f;
	glRotatef(sign*angle, 0, 1, 0);
	RenderObjects();
	glutSwapBuffers();
}

void Mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		trackball.Set(true, x, y);
		mouseLeft = true;
	}
	if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
	{
		trackball.Set(false, x, y);
		mouseLeft = false;
	}
	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
	{
		trackball.Set(true, x, y);
		mouseMid = true;
	}
	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_UP)
	{
		trackball.Set(true, x, y);
		mouseMid = false;
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{
		trackball.Set(true, x, y);
		mouseRight = true;
	}
	if (button == GLUT_RIGHT_BUTTON && state == GLUT_UP)
	{
		trackball.Set(true, x, y);
		mouseRight = false;
	}
}

void MouseMotion(int x, int y) {
	if (mouseLeft)  trackball.Rotate(x, y);
	if (mouseMid)   trackball.Translate(x, y);
	if (mouseRight) trackball.Zoom(x, y);
	glutPostRedisplay();
}


int main(int argc, char **argv)
{ 
	srand(19950220);	// fake random numbers
	glutInitDisplayString("stencil>=2 rgb double depth samples");
	glutInit(&argc, argv);
	glutInitWindowSize(wWindow,hWindow);
	glutInitWindowPosition(500,100);
	glutCreateWindow("Surface of Revolution");
	//GLenum err = glewInit();
	// if (GLEW_OK != err){
	// fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
	//}
	glutDisplayFunc(Display);
	glutIdleFunc(Idle);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Kbd); //+ and -
	glutSpecialUpFunc(NULL); 
	glutSpecialFunc(NULL);
	glutMouseFunc(Mouse);
	glutMotionFunc(MouseMotion);
	InitArray(steps);
	init_base_tree();
	glutMainLoop();
	return 0;        
}
