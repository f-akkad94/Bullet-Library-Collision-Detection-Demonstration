/*
Collision Detection in Physics Engines
Fadl Akkad - May 2016
Swansea University

Demonstration for the Bsc project to investigate collision detection using
The Bullet library
*/

/*
Import necessary header files and dependencies including functionality
to enable OpenGL, SDL and Bullet.
*/
#include <windows.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <time.h>
#include <Math.h>
#include <cmath>
#include "SDL.h"
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <vector>
#include "camera.h"
#include "btBulletDynamicsCommon.h"

camera cam; // Initiate the interactive camera 

GLUquadricObj* quad; // A sphere is inherited from quadric objects in OpenGL

btDynamicsWorld* world;	// Establish Bullet functionality located in libraries for the scene.
						// 'DynamicsWorld' is a major super class in the library.
btDispatcher* dispatcher; // Dispatches calculations for overlapping pairs,
						  // thus when pairwise collision occurs between the spheres,
						  // this function will calculate contact points and derive the
						  // necessary consequent algorithm.
btCollisionConfiguration* collisionConfig;	// Processes what collision algorithm to use based on
											// collected information including contact points and shape
											// information.
btBroadphaseInterface* broadphase;	// Function which examines the scene to assign priority to more populated
									// segments of the scene, and establishes aabb(s) or axis-alligned bounding
									// boxes for faster overlap tests across that particular segment.
btConstraintSolver* solver;		// Function which solves collisions, apply forces, impulses and provides new
								// behaviour for the objects post collision.

								/*
								Construct 'objects' recognised by Bullet that collaberate with each OpenGL constructed sphere.
								Each object has its own unique identifier, aesthetic information and for the objective of collision
								a boolean that will be used to recognise when collision has occured.
								*/
struct bulletObject {
	int id;
	float r, g, b;
	bool hit;
	btRigidBody* body;
	bulletObject(btRigidBody* b, int i, float r0, float g0, float b0) :
		body(b), id(i), r(r0), g(g0), b(b0), hit(false) {}
};

std::vector<bulletObject*> bodies; // For each Bullet object deployed in the scene, provide them with
								   // a sequence container so that the objects can be stored.

bool threeD = false; // Initialize variable to query whether the user would prefer 2D deployment or 
					 // 3D deployment.
bool fast = false; // Initialize variable to query whether the user would prefer the spheres to move 
				   // at a fast speed or slow speed.
int sphereCount; // Initialize variable to query how many spheres the user would like to populate the scene
				 // with.
float xLocation; // Initialize variable to provide each sphere the x co-ordinate of its deployment position. 
float yLocation; // Initialize variable to provide each sphere the y co-ordinate of its deployment position.
float zLocation; // Initialize variable to provide each sphere the z co-ordinate of its deployment position.
int direction; // Initialize variable to provide each sphere its initial direction of movement.
bool reverse = false;
float speed; //Initialize variable to provide each sphere its initial speed.

			 /*
			 Method which deploys a sphere into the world, with provided information including the radius,
			 its position of deployment and its initial mass.
			 */
btRigidBody* addSphere(float rad, float x, float y, float z, float mass)
{
	btTransform t;	// Variable to generate the spheres position and rotation
	t.setIdentity();
	t.setOrigin(btVector3(x, y, z)); // Initialize Bullet function which allows for velocity and movement.
	btSphereShape* sphere = new btSphereShape(rad);	// Correct Bullet shape to use along with OpenGL quad.
	btVector3 inertia(0, 0, 0); // Initialize inertia function to be calculated. 
	if (mass != 0.0)
		sphere->calculateLocalInertia(mass, inertia); // Calculate inertia of the sphere.

	btMotionState* motion = new btDefaultMotionState(t); // Bullet function to establish motion.
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, sphere, inertia);
	btRigidBody* body = new btRigidBody(info);	// Create the Bullet body using these instructions.
	world->addRigidBody(body);
	// Provide a random number for the speed of the object, depending on the user's choice.
	// NOTE: Negative 'speed' implies moving backwards on axis.
	if (fast == true) {
		speed = rand() % 101 + -100;
	}
	else {
		speed = rand() % 4 + -3;
	}
	if (direction == 1) {
		body->setLinearVelocity(btVector3(speed, 0, 0)); // If the sphere was assigned to the X axis,
														 // instill movement on the X axis.
	}
	else if (direction == 2) {
		body->setLinearVelocity(btVector3(0, speed, 0)); // If the sphere was assigned to the Y axis,
														 // instill movement on the Y axis.
	}
	else if (direction == 3) {
		body->setLinearVelocity(btVector3(0, 0, speed)); // If the sphere was assigned to the Z axis,
														 // instill movement on the Z axis.
	}
	// Set a 'collision flag' to the sphere, a unique Bullet function to ensure Bullet is able to recognize
	// contact with another shape that also has a collision flag.
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	body->getMotionState()->getWorldTransform(t);
	bodies.push_back(new bulletObject(body, 1, 1.0, 0.0, 0.0));	// Store the sphere in a vector sequence container
																// complete with a unique identifier for the purpose
																// of recognizing collision.
	body->setUserPointer(bodies[bodies.size() - 1]);
	return body;
}

/*
Method to utilize OpenGL functions to provide graphical correlation with Bullet physics,
as to actually render visual information.
*/
void renderSphere(bulletObject* bulletobj)
{
	btRigidBody* sphere = bulletobj->body;
	if (sphere->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE)
		return;
	// Acquire information that will help ensure consistent aesthetic visuals.
	glColor3f(bulletobj->r, bulletobj->g, bulletobj->b);
	float r = ((btSphereShape*)sphere->getCollisionShape())->getRadius();
	btTransform t;
	sphere->getMotionState()->getWorldTransform(t);
	//sphere->applyCentralImpulse(btVector3(50, 0, 0));
	float mat[16];
	t.getOpenGLMatrix(mat);	// OpenGL matrix stores the rotation and orientation.
	glPushMatrix();
	glMultMatrixf(mat);	// Multiplying the current matrix with it moves the object in place.
						// Ensure consistent collaboration between Bullet and OpenGL.
	gluSphere(quad, r, 20, 20); // Finalize quadric object.
	glPopMatrix();
}

/*
Method to intialize world objects and functions to initiate the program
and have essential components operatable. This also includes generating
the number of spheres and their initial locations and properties,
predominantly through the use of randomly generated figures.
*/
void init(float angle)
{
	// Intitiate Bullet functions.
	collisionConfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
	world->setGravity(btVector3(0, 0, 0));

	for (int i = 1; i <= sphereCount; i++) {
		direction = rand() % 4 + 1; // Assign whether the sphere shall traverse the x, y or z axis.
		int randomX = rand() % 200 + 1; // Assign the x co-ordinate of the spheres position.
		xLocation = randomX;
		int randomY = rand() % 100 + 1; // Assign the y co-ordinate of the spheres position.
		yLocation = randomY;
		int randomZ = rand() % 200 + 1; // Assign the z co-ordinate of the spheres position.
		zLocation = randomZ;
		int randomRadius = rand() % 2 + 1; // Assign the size of the radius - very small variations.
		if (threeD == true) {
			addSphere(randomRadius, xLocation, yLocation, zLocation, 1.0);
		}
		else {
			addSphere(randomRadius, xLocation, yLocation, 0.0, 1.0);
		}
		// NOTE: 'zLocation will be 0.0 should the user prefer to have the spheres
		// operate on a 2D plane in our scene.
	}

	// Intitiate OpenGL functions.
	glClearColor(0, 0, 0, 1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(angle, 640.0 / 480.0, 1, 1000);
	glMatrixMode(GL_MODELVIEW);
	quad = gluNewQuadric();
	glEnable(GL_DEPTH_TEST);
	cam.setLocation(vector3d(10, 10, 50)); // Set the camera's initial position in the scene.
}

/*
Method to provide variations of impulse forces on the spheres throughout the scene in real time.
*/
void changeDirection(bulletObject* bulletobj)
{
	btRigidBody* sphere = bulletobj->body;
	int change = rand() % 11 + 1;
	if (change == 1) {
		sphere->applyCentralImpulse(btVector3(0.003, 0, 0));
	}
	else if (change == 2) {
		sphere->applyCentralImpulse(btVector3(0, 0.003, 0));
	}
	else if (change == 3) {
		sphere->applyCentralImpulse(btVector3(0, 0, 0.003));
	}
	else if (change == 4) {
		sphere->applyCentralImpulse(btVector3(-0.003, 0, 0));
	}
	else if (change == 5) {
		sphere->applyCentralImpulse(btVector3(0, -0.003, 0));
	}
	else if (change == 6) {
		sphere->applyCentralImpulse(btVector3(0, 0, 0.003));
	}
	else if (change == 7) {
		sphere->applyCentralImpulse(btVector3(0.003, 0.003, 0.003));
	}
	else if (change == 8) {
		sphere->applyCentralImpulse(btVector3(0.003, 0.003, 0.003));
	}
	else if (change == 9) {
		sphere->applyCentralImpulse(btVector3(0.003, 0.003, 0.003));
	}
	else if (change == 10) {
		sphere->applyCentralImpulse(btVector3(-0.003, -0.003, -0.003));
	}
	else if (change == 11) {
		sphere->applyCentralImpulse(btVector3(-0.003, -0.003, -0.003));
	}
	else if (change == 12) {
		sphere->applyCentralImpulse(btVector3(-0.003, -0.003, -0.003));
	}
}

/*
General method to call for the OpenGL renderSphere method, and load the camera.
*/
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	cam.Control();
	cam.UpdateCamera();
	for (int i = 0; i<bodies.size(); i++)
	{
		renderSphere(bodies[i]);
	}
}

/*
Method responsible for detecting collision between objects, by having two dedicated collision flags
collaberate.
*/
bool collisionDetection(btManifoldPoint& collide, const btCollisionObjectWrapper* obj1, int id1, int index1, const btCollisionObjectWrapper* obj2, int id2, int index2) {
	// The flag is located in the storied body of the collision object, a derivative
	// of the function 'btCollisionObjectWrapper'. A collision flag is composed of unique
	// binary number identifiers that Bullet recognizes, this helps pairwise collision
	// when a test is being conducted either outside or inside an aabb.
	std::cout << ((bulletObject*)obj1->getCollisionObject()->getUserPointer()) << " Collision Detected! " << ((bulletObject*)obj2->getCollisionObject()->getUserPointer()) << std::endl;
	return false;
}

/*
Main Method - Execute The Program
*/
int main(int argc, char *args[])
{
	// We ask the user whether they would like to have the spheres be deployed on a 2-Dimensional plane
	// ignoring the z axis (this will ensure more collisions for testing) or include the z axis for initial
	// deployment. We ask the same for the speed of the spheres.
	std::cout << "Please type 'y' to have spheres be positioned 3-Dimensionally or 'n' to eliminate the Z-Axis positioning ";
	std::string letter;
	std::cin >> letter;
	while (letter != "y" & letter != "n") {
		std::cout << "Please type 'y' to have spheres be positioned 3-Dimensionally or 'n' to eliminate the Z-Axis positioning ";
		std::cin >> letter;
	}
	if (letter == "y") {
		threeD = true;
	}
	else {
		threeD = false;
	}
	std::cout << "Please type 'f' to have spheres move fast or 's' to have spheres move slow ";
	std::string letter2;
	std::cin >> letter2;
	while (letter2 != "f" & letter2 != "s") {
		std::cout << "Please type 'f' to have spheres move fast or 's' to have spheres move slow ";
		std::cin >> letter2;
	}
	if (letter2 == "f") {
		fast = true;
	}
	else {
		fast = false;
	}
	std::cout << "Please enter the number of spheres you would like to populate the world with ";
	std::cout << "Suggested nominal number is 300 or less.";
	std::cout << "NOTE: maintain a nominal number so as not to exhaust computational power of one's own system. ";
	std::cin >> sphereCount; // Provide number of spheres.
							 // NOTE: maintain nominal number so as not to exhaust computational power
							 // of one's own system, otherwise the program will be ineffectual and thus
							 // counter intuitive to the purpose of this demonstration.

							 // Initiate SDL collaberation and implementation, allowing for the user to interact with the scene.
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_SetVideoMode(640, 480, 32, SDL_OPENGL);
	Uint32 start;
	SDL_Event event;
	bool running = true;
	float angle = 50; // camera perspective
	init(angle);
	gContactAddedCallback = collisionDetection; // SAY SOMETHING
												// General SDL programmed responses to user input
	while (running)
	{
		start = SDL_GetTicks();
		while (SDL_PollEvent(&event))
		{
			switch (event.type)
			{
			case SDL_QUIT:
				running = false;
				break;
			case SDL_KEYDOWN:
				switch (event.key.keysym.sym)
				{
				case SDLK_ESCAPE:
					running = false;
					break;
				case SDLK_y:
					cam.mouseIn(false);
					break;
				}
				break;
			case SDL_KEYUP:
				switch (event.key.keysym.sym)
				{

				}
				break;
			case SDL_MOUSEBUTTONDOWN:
				cam.mouseIn(true);
				break;
			}
		}
		world->stepSimulation(1 / 60.0); // The critical act of 'stepping' the scene, that is to say enable
										 // the scene to operate in real time motion using the concurrent
										 // time frame. This consistently updates the scene, maintaining Bullet,
										 // OpenGL and SDL actions and operations as to simulate a real time
										 // interactive scene.
		display();
		for (int i = 0; i<bodies.size(); i++)
		{
			changeDirection(bodies[i]);
		}
		SDL_GL_SwapBuffers();
		// Ensure SDL maintains input collection along with step simulation.
		if (1000.0 / 60>SDL_GetTicks() - start)
			SDL_Delay(1000.0 / 60 - (SDL_GetTicks() - start));
	}
	// Standard initiative to remove and 'clean' functions and operations
	// after execution.
	for (int i = 0; i<bodies.size(); i++)
	{
		world->removeCollisionObject(bodies[i]->body);
		btMotionState* motionState = bodies[i]->body->getMotionState();
		btCollisionShape* shape = bodies[i]->body->getCollisionShape();
		delete bodies[i]->body;
		delete shape;
		delete motionState;
		delete bodies[i];
	}
	delete dispatcher;
	delete collisionConfig;
	delete solver;
	delete broadphase;
	delete world;
	SDL_Quit();
	gluDeleteQuadric(quad);
	return 0;
}

