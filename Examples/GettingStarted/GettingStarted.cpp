//////////////////////////////////////////////////////////////////////////
//
// Getting Started Example
//
//////////////////////////////////////////////////////////////////////////

// Include necessary header files.
#include "Renderer/SimpleViewer.h"	// for rendering
#include "srg/srgGeometryDraw.h"	// for User rendering
#include "srDyn/srSpace.h"			// for dynamics


// Get srSimpleViewer instance.
srSimpleViewer& gViewer = srSimpleViewer::GetInstance();	// Simple Viewer (singleton)


//////////////////////////////////////////////////////////////////////////
// Space
//////////////////////////////////////////////////////////////////////////
srSpace gSpace; // Space that systems will be included.

srLinearSpring gSpring;
//////////////////////////////////////////////////////////////////////////
// System 1: Ground
//////////////////////////////////////////////////////////////////////////
srSystem gGroundSystem;	// system for ground plane
srLink	gGround;		// link for ground
srCollision gGroundCollision; // collision for ground plane
srCollision gWall[5];


//////////////////////////////////////////////////////////////////////////
// System 2: Mobile Robot
//////////////////////////////////////////////////////////////////////////
srSystem gSystem;	// system for mobile robot

srLink	gBodyLink;	// main body of mobile robot
srCollision gBodyLinkCollision; // collision for main body

srLink gFrontWheel1;				// first front wheel
srCollision gFrontWheel1Collision;	// collision for first front wheel
srRevoluteJoint gFrontWheel1Joint;	// revolute joint for first front wheel

srLink gFrontWheel2;				// second front wheel
srCollision gFrontWheel2Collision;	// collision for second front wheel
srRevoluteJoint gFrontWheel2Joint;	// revolute joint for second front wheel

srLink gRearWheelCaster;				// rear wheel
srCollision gRearWheelCasterCollision;	// collision for rear wheel
srWeldJoint gRearWheelCasterJoint;		// weld joint for rear wheel

srIRSensor gIRSensor;
srRangeFinder gRF;
//////////////////////////////////////////////////////////////////////////
// System 3: Uphill Obstacle
//////////////////////////////////////////////////////////////////////////
srSystem	gUphillSystem;		// system for uphill obstacle
srLink	gUphill;				// link for uphill obstacle
srCollision	gUphillCollision;	// collision for uphill obstacle


// For modeling your robots(systems), we need many codes. It would be too
// long to be in main() function. So we this many codes put into the function
// named User_Modeling().
void User_Modeling();			// modeling function

// For simulation, there are some necessary settings.
// User_SimulationSetting function will do that.
void User_SimulationSetting();  // simulation setting

//
void User_Simulation_Go_One_Step();	// simulation main loop
void User_Simulation_Pause();		    // empty function

// If you want to control your mobile robot, we need a contorl function.
// And this function must be registered at srSpace.
void User_CBFunc_ControlLoop();	// User control loop.

// Followings are callback functions for simulation control. We are going to connect
// these function with certain key.
void User_CBFunc_Pause_DYN(void); // Callback functions for User Key Control
void User_CBFunc_Run_DYN(void);   // Callback functions for User Key Control

// Followings are callback functions, too. But these are callback functions for 
// control of mobile robot. These functions will be connected with certain key, too.
void User_CBFunc_GoForward();			// Callback functions for User Key Control
void User_CBFunc_Stop();				// Callback functions for User Key Control
void User_CBFunc_GoBackward();			// Callback functions for User Key Control
void User_CBFunc_TurnLeft();
void User_CBFunc_TurnRight();
void User_CBFunc_ActivateSpring();		// Activate/Deactivate spring element.
void User_CBFunc_Impulse();				// Hit car!

// Callback function for user rendering
//   put your code to render what you want.
void User_CBFunc_Render(void* pvData);

double Lwheelvelocity = 0.0;		// wheel velocity
double Rwheelvelocity = 0.0;		// wheel velocity

dse3	gImpulse = dse3(0.0);		// Impulse

//////////////////////////////////////////////////////////////////////////
// main
//////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	// STEP 1: Viewer initialization
	// srSimpleViewer render scene using OpenGL GLUT.
	// ...OpenGL GLUT need argc and argv for initializing.
	gViewer.Init(&argc, argv);

	// STEP 2: Robot Modeling
	User_Modeling();
	
	// STEP 3: Simulation setting
	User_SimulationSetting();

	// STEP 4: Run window view.
	gViewer.Run();

	return 0;
}

// Let's see the code for robot modeling. It's a quite long code ;)
void User_Modeling()
{
	//////////////////////////////////////////////////////////////////////////
	// System 1: Ground
	//////////////////////////////////////////////////////////////////////////
	//- Set geometry information for ground.
	gGround.GetGeomInfo().SetShape(srGeometryInfo::PLANE);
	// Inertia update (It is not needed for fixed mass).
	gGround.UpdateInertia();
	// Set geometry information for collision entity of ground.
	gGroundCollision.GetGeomInfo().SetShape(srGeometryInfo::PLANE);
	// Set relative frame of collision entity.
	gGroundCollision.SetLocalFrame(SE3());
	// Add collision entity to ground.
	gGround.AddCollision(&gGroundCollision);


	for(int i = 0 ; i < 5 ; ++i) {
		gWall[i].GetGeomInfo().SetShape(srGeometryInfo::BOX);		
	}
	gWall[0].GetGeomInfo().SetDimension(0.1, 1.0, 1.0);
	gWall[1].GetGeomInfo().SetDimension(1.0, 0.1, 1.0);
	gWall[2].GetGeomInfo().SetDimension(0.1, 1.0, 1.0);
	gWall[3].GetGeomInfo().SetDimension(1.0, 0.1, 1.0);
	gWall[4].GetGeomInfo().SetDimension(1.0, 1.0, 0.1);
	gWall[0].SetLocalFrame(EulerZYX(Vec3(0), Vec3(0.5, 0.0, 0.5)));
	gWall[1].SetLocalFrame(EulerZYX(Vec3(0), Vec3(0.0, 0.5, 0.5)));
	gWall[2].SetLocalFrame(EulerZYX(Vec3(0), Vec3(-0.5, 0.0, 0.5)));
	gWall[3].SetLocalFrame(EulerZYX(Vec3(0), Vec3(0.0, -0.5, 0.5)));
	gWall[4].SetLocalFrame(EulerZYX(Vec3(0), Vec3(0.0, 0.0, 1.0)));

	for(int i = 1 ; i < 5 ; ++i) {
		gGround.AddCollision(&(gWall[i]));
	}

	// Set ground link as base link of ground system.
	gGroundSystem.SetBaseLink(&gGround);
	// Make base link fixed.
	gGroundSystem.SetBaseLinkType(srSystem::FIXED);
	// Add ground system to space.
	gSpace.AddSystem(&gGroundSystem);


	//////////////////////////////////////////////////////////////////////////
	// System 2: Mobile Robot
	//////////////////////////////////////////////////////////////////////////
	///- Main body
	//- Set geometry information for body.
//	gBodyLink.GetGeomInfo().SetShape(srGeometryInfo::BOX);		// Box shape
//	gBodyLink.GetGeomInfo().SetDimension(0.4, 0.3, 0.2);				// Width, depth, height 
//	gBodyLink.GetGeomInfo().SetColor(0.5, 0.5, 0.8);
	// Inertia update (argument is density).
//	gBodyLink.UpdateInertia(10);

	gBodyLink.GetGeomInfo().SetShape(srGeometryInfo::TDS);
	gBodyLink.GetGeomInfo().SetFileName("miniBear.3ds");
	gBodyLink.GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, SR_PI/2), Vec3(0.0, 0.0, 0.1)));
	gBodyLink.SetInertia(Inertia(2, 0.001, 0.001, 0.001));		// Inertia(Mass, Ixx, Iyy, Izz)

	// Set geometry information for collision entity.
	gBodyLinkCollision.GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gBodyLinkCollision.GetGeomInfo().SetDimension(0.4, 0.3, 0.2);
	// Set relative frame of collision entity.
	gBodyLinkCollision.SetLocalFrame(SE3());
	// Add collision entity to link.
	gBodyLink.AddCollision(&gBodyLinkCollision);
	// Initial position and orientation. (This matters because main body will be base link of system.)
	gBodyLink.SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 0.5)));

	gIRSensor.SetRange(1.0, 0.1);
	gIRSensor.SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(-0.2, 0.0, -0.07)));
	gBodyLink.AddSensor(&gIRSensor);

	gRF.SetRange(2.0, 0.1);
	gRF.SetSpread(120);	gRF.SetResolution(5);
	gRF.SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.2, 0.0, 0.0)));
	gBodyLink.AddSensor(&gRF);

	///- Connect spring to gBodyLink and environment.
	// Set links connected to both ends of spring.
	gSpring.SetLeftLink(&gGround);
	gSpring.SetRightLink(&gBodyLink);
	
	// Set relative position on each link w.r.t. the center of each link.
	gSpring.SetLeftLinkPosition(Vec3(0.5, 0.0, 1.0));
	gSpring.SetRightLinkPosition(Vec3(0.0, 0.0, 0.1));	// center of the top of gLinkBody

	// Set initial length of spring (release state)
	gSpring.SetInitialLength(0.1);	// 10cm
	// Set spring/damping coefficient.
	gSpring.SetK(50.0);
	gSpring.SetC(1.0);			// Default is zero.
	gSpring.Activate(false);	// Deactivate spring. Press 'C' key to toggle this.

	// Register this spring on Space. DO NOT MISS!!!!!!!!
	gSpace.AddSpring(&gSpring);

	///- FrontWheel1
	//- Set geometry information.
	gFrontWheel1.GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
	gFrontWheel1.GetGeomInfo().SetDimension(0.15, 0.05);				// Diameter, depth.
	gFrontWheel1.GetGeomInfo().SetColor(0.1, 0.1, 0.1);
	// Inertia update.
	gFrontWheel1.UpdateInertia(10);
	// Set friction coefficient.
//	gFrontWheel1.SetFriction(1.5);
	// Set geometry information for collision entity.
	gFrontWheel1Collision.GetGeomInfo().SetShape(srGeometryInfo::SPHERE); // Collision detection algorithm is not fully provided for cylinder geometry yet.
	gFrontWheel1Collision.GetGeomInfo().SetDimension(0.15);				// Diameter
	// Set relative frame of collision entity.
	gFrontWheel1Collision.SetLocalFrame(SE3());
	// Add collision entity.
	gFrontWheel1.AddCollision(&gFrontWheel1Collision);

	///- FrontWheel2
	//- Set geometry information.
	gFrontWheel2.GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
	gFrontWheel2.GetGeomInfo().SetDimension(0.15, 0.05);				// Diameter, depth.
	gFrontWheel2.GetGeomInfo().SetColor(0.1, 0.1, 0.1);
	// Inertia update.
	gFrontWheel2.UpdateInertia(10);
	// Set friction coefficient.
//	gFrontWheel2.SetFriction(1.5);
	// Set geometry information for collision entity.
	gFrontWheel2Collision.GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	gFrontWheel2Collision.GetGeomInfo().SetDimension(0.15);				// Diameter
	// Set relative frame of collision entity.
	gFrontWheel2Collision.SetLocalFrame(SE3());
	// Add collision entity.
	gFrontWheel2.AddCollision(&gFrontWheel2Collision);

	///- RearWheelCaster
	//- Set geometry information.
	gRearWheelCaster.GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	gRearWheelCaster.GetGeomInfo().SetDimension(0.1);				// Radius.
	// Inertia update.
	gRearWheelCaster.UpdateInertia(10);
	// Set friction coefficient.
	gRearWheelCaster.SetFriction(0.0);	// Attach rear wheel with weld joint and make it frictionless.
	// Set geometry information for collision entity.
	gRearWheelCasterCollision.GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	gRearWheelCasterCollision.GetGeomInfo().SetDimension(0.1);
	// Set relative frame of collision entity.
	gRearWheelCasterCollision.SetLocalFrame(SE3());
	// Add collision entity.
	gRearWheelCaster.AddCollision(&gRearWheelCasterCollision);


	///- FrontWheel1 Joint
	// Set actuator type.
	gFrontWheel1Joint.SetActType(srJoint::VELOCITY);
	// Make it not be limited in joint angle.
	gFrontWheel1Joint.MakePositionLimit(false);
	// Set parent link.
	gFrontWheel1Joint.SetParentLink(&gBodyLink);
	// Set child link.
	gFrontWheel1Joint.SetChildLink(&gFrontWheel1);
	// Set relative frames
	gFrontWheel1Joint.SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI/2),Vec3(0.1, 0.2, -0.1)));
	gFrontWheel1Joint.SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 0.0)));

	//- FrontWheel2 Joint
	// Set actuator type.
	gFrontWheel2Joint.SetActType(srJoint::VELOCITY);
	// Make it not be limited in joint angle.
	gFrontWheel2Joint.MakePositionLimit(false);
	// Set parent link.
	gFrontWheel2Joint.SetParentLink(&gBodyLink);
	// Set child link.
	gFrontWheel2Joint.SetChildLink(&gFrontWheel2);
	// Set relative frames
	gFrontWheel2Joint.SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI/2),Vec3(0.1, -0.2, -0.1)));
	gFrontWheel2Joint.SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 0.0)));

	//- RearWheel Joint
	// Set parent link.
	gRearWheelCasterJoint.SetParentLink(&gBodyLink);
	// Set child link.
	gRearWheelCasterJoint.SetChildLink(&gRearWheelCaster);
	// Set relative frames
	gRearWheelCasterJoint.SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(-0.2, 0.0, -0.125)));
	gRearWheelCasterJoint.SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.1, 0.0, 0.0)));

	// Set main body link as base link of mobile robot.
	gSystem.SetBaseLink(&gBodyLink);
	// Set base link of mobile robot dynamic.
	gSystem.SetBaseLinkType(srSystem::DYNAMIC);
	// Ignore self collision.
	gSystem.SetSelfCollision(false);
	// Add mobile robot system to space
	gSpace.AddSystem(&gSystem);


	//////////////////////////////////////////////////////////////////////////
	// System 3: Uphill Obstacle
	//////////////////////////////////////////////////////////////////////////
	//- Set geometry information of uphill obstacle link.
	gUphill.GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gUphill.GetGeomInfo().SetDimension(1, 1, 0.1);
	// Inertia update (It is not needed for fixed mass).
	gUphill.UpdateInertia();
	// Set geometry information for collision entity of ground.
	gUphillCollision.GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gUphillCollision.GetGeomInfo().SetDimension(1, 1, 0.1);
	// Set relative frame of collision entity.
	gUphillCollision.SetLocalFrame(SE3());
	// Add collision entity to ground.
	gUphill.AddCollision(&gUphillCollision);
	// Initial position and orientation.
	gUphill.SetFrame(EulerZYX(Vec3(0.0, -10.0 * SR_RADIAN, 0.0),Vec3(1.0, 0.0, 0.0)));
	// Set uphill link as base link of uphill obstacle system.
	gUphillSystem.SetBaseLink(&gUphill);
	// Make base link fixed.
	gUphillSystem.SetBaseLinkType(srSystem::FIXED);
	// Add uphill system to space.
	gSpace.AddSystem(&gUphillSystem);


	////- Space
	// Set simulation time step.
	gSpace.SetTimestep(0.001);
	// Set gravity
	gSpace.SetGravity(0.0, 0.0, -9.8);
	// Set number of sub-step for rendering
	gSpace.SetNumberofSubstepForRendering(50);

}

// 
void User_SimulationSetting()
{
	// Set user control loop function.
	gSpace.SET_USER_CONTROL_FUNCTION(User_CBFunc_ControlLoop);

	// Initialize for dynamics simulation.
	gSpace.DYN_MODE_PRESTEP();

	// Set target space to render.
	// Let srSimpleRenderer know what you want to draw on screen.
	gViewer.SetTarget(&gSpace);

	// Additional step: Set your user-render function.
	gViewer.SetUserRenderFunc(User_CBFunc_Render, NULL);

	// Key mapping
	// As I promised before, we are going to connect some functions with keys.
	// It's very simple.
	// One key can have only one function, but one function can be connected
	// with more than one key.
	gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'P');
	gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'p');

	gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'O');
	gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'o');

	gViewer.SetKeyFunc(User_CBFunc_GoForward, 'W');
	gViewer.SetKeyFunc(User_CBFunc_GoForward, 'w');

	gViewer.SetKeyFunc(User_CBFunc_Stop, 'X');
	gViewer.SetKeyFunc(User_CBFunc_Stop, 'x');

	gViewer.SetKeyFunc(User_CBFunc_GoBackward, 'S');
	gViewer.SetKeyFunc(User_CBFunc_GoBackward, 's');

	gViewer.SetKeyFunc(User_CBFunc_TurnLeft, 'A');
	gViewer.SetKeyFunc(User_CBFunc_TurnLeft, 'a');

	gViewer.SetKeyFunc(User_CBFunc_TurnRight, 'D');
	gViewer.SetKeyFunc(User_CBFunc_TurnRight, 'd');

	gViewer.SetKeyFunc(User_CBFunc_ActivateSpring, 'C');
	gViewer.SetKeyFunc(User_CBFunc_ActivateSpring, 'c');

	gViewer.SetKeyFunc(User_CBFunc_Impulse, 'I');
	gViewer.SetKeyFunc(User_CBFunc_Impulse, 'i');

}

// Every simulation step, this function will work.
void User_CBFunc_ControlLoop()
{
	// Set command signal for front wheel joints of mobile robot.
	gFrontWheel1Joint.m_State.m_rCommand = Lwheelvelocity;
	gFrontWheel2Joint.m_State.m_rCommand = Rwheelvelocity;

	gBodyLink.AddUserExternalForce(gImpulse);
	gImpulse = dse3(0.0);
}

// When we push the 'P' key, this function will work.
void User_CBFunc_Run_DYN()
{
	// Run dynamics simulation by setting glMainLoop idle function as dynamics simulation step forward function.
	gViewer.SetLoopFunc(User_Simulation_Go_One_Step);
}

// When we push the 'O' key, this function will work.
void User_CBFunc_Pause_DYN()
{
	// Pause dynamics simulation by setting glMainLoop idle function as empty function.
	gViewer.SetLoopFunc(User_Simulation_Pause);
}

// When we push the 'Y' key, this function will work.
void User_CBFunc_GoForward()
{
	Lwheelvelocity = -1.0;
	Rwheelvelocity = -1.0;
}

// When we push the 'H' key, this function will work.
void User_CBFunc_Stop()
{
	Lwheelvelocity = 0.0;
	Rwheelvelocity = 0.0;
}

// When we push the 'N' key, this function will work.
void User_CBFunc_GoBackward()
{
	Lwheelvelocity = 1.0;
	Rwheelvelocity = 1.0;
}

void User_CBFunc_TurnLeft()
{
	Lwheelvelocity = 1.0;
	Rwheelvelocity = -1.0;
}

void User_CBFunc_TurnRight()
{
	Lwheelvelocity = -1.0;
	Rwheelvelocity = 1.0;
}

void User_CBFunc_ActivateSpring()
{
	gSpring.Activate(!gSpring.IsActive());
}

void User_CBFunc_Impulse()
{
	gImpulse = dse3(0.0, 0.0, 0.0, 0.0, 0.0, 1000.0);
}


// When we push the 'P' key, this function will be register to loop
// function of srSimpleViewer. Then the simulation will run.
void User_Simulation_Go_One_Step()
{
	// Dynamics simulation step forward function.
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
}

// When we push the 'O' key, this function will be register to loop
// function of srSimpleViewer. Then the simulation will be stopped.
void User_Simulation_Pause()
{
	// do nothings
}

void User_CBFunc_Render(void* pvData)
{
	// you can draw whatever you want in OpenGL world
	srgMaterialColor BoxColor(0.5, 1.0, 0.2);
	BoxColor.PushAttrib();
		srgDrawBox_NV(5.0, 5.0, .01);
	BoxColor.PopAttrib();
}
