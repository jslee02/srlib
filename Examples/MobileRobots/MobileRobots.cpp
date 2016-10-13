//////////////////////////////////////////////////////////////////////////
//
// Mobile Robots Example
//
//////////////////////////////////////////////////////////////////////////

// Include necessary header files.
#include "Renderer/SimpleViewer.h"	// for rendering
#include "srg/srgGeometryDraw.h"	// for User rendering
#include "srDyn/srSpace.h"			// for dynamics

#include "Ground.h"
#include "MobileRobots.h"

// Get srSimpleViewer instance.
srSimpleViewer& gViewer = srSimpleViewer::GetInstance();	// Simple Viewer (singleton)

// Space
srSpace gSpace;

// Ground
Ground gGround;

// Mobile Robots
MobileRobot	robot1;
MobileRobot	robot2;

// Big plate
srSystem	gPlate;
srLink		gPlateLink;
srCollision	gPlateCollision;

// Obstacle
srSystem	gUphillSystem;		// system for uphill obstacle
srLink		gUphill;			// link for uphill obstacle
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

// If you want to control your robot, we need a contorl function.
// And this function must be registered at srSpace.
void User_CBFunc_ControlLoop();	// User control loop.

// Followings are callback functions for simulation control. We are going to connect
// these function with certain key.
void User_CBFunc_Pause_DYN(void); // Callback functions for User Key Control
void User_CBFunc_Run_DYN(void);   // Callback functions for User Key Control

// Callback function for user rendering
//   put your code to render what you want.
void User_CBFunc_Render(void* pvData);

// Callback function for user key function.
void User_CBFunc_KeyFunc(char key, void* pvData);


// array for saving the traces of robot1 & robot2
_array<Vec3>	trace1;
_array<Vec3>	trace2;

//////////////////////////////////////////////////////////////////////////
// main
//////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	// STEP 1: Viewer initialization
	// srSimpleViewer render scene using OpenGL GLUT.
	// ...OpenGL GLUT need argc and argv for initializing.
	gViewer.Init(&argc, argv, "Two Mobile Robots");

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
	SE3 T;

	// System 1: Create Ground
	gSpace.AddSystem(gGround.BuildGround());
	gGround.m_Ground->SetFriction(0.8);

	// System 2, 3: Create Mobile Robots
	gSpace.AddSystem(robot1.BuildRobot(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.3, 0.0, 0.2))));
	gSpace.AddSystem(robot2.BuildRobot(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.3, 0.0, 0.2))));
	robot1.m_LeftWheel.SetFriction(0.8);
	robot1.m_RightWheel.SetFriction(0.8);
	robot2.m_LeftMotor.SetActType(srJoint::PASSIVE);
	robot2.m_RightMotor.SetActType(srJoint::PASSIVE);

	// System 4: Big plate over two mobile robots
	gPlateLink.GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gPlateLink.GetGeomInfo().SetDimension(1.0, 0.5, 0.01);
	gPlateLink.GetGeomInfo().SetColor(0.8, 0.4, 0.3, 0.5);
	gPlateLink.UpdateInertia(200);
	gPlateCollision.GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gPlateCollision.GetGeomInfo().SetDimension(1.0, 0.5, 0.01);
	gPlateLink.AddCollision(&gPlateCollision);

	gPlateLink.SetFrame(SE3(Vec3(0.0, 0.0, 0.3)));
	gPlate.SetBaseLink(&gPlateLink);
	gPlate.SetBaseLinkType(srSystem::DYNAMIC);
	gSpace.AddSystem(&gPlate);

	// System 5: Uphill Obstacle
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

	// Set key function. glut windows will call 'User_CBFunc_KeyFunc' function
	// every time when key pressed.
	gViewer.SetUserKeyFunc(User_CBFunc_KeyFunc, NULL);
}

// Every simulation step, this function will work.
void User_CBFunc_ControlLoop()
{
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

// Set your own key binding here.
void User_CBFunc_KeyFunc(char key, void* pvData)
{
	switch(key) {
	// Initialize all state.
	case 'i':
	case 'I':
		gSpace._RestoreInitState();
		robot1.SetVelocity(0.0, 0.0);
		trace1.clear();
		trace2.clear();
		break;
	// Robot1 go forward.
	case 'w':
	case 'W':
		robot1.SetVelocity(-1.5, -1.5);
		break;
	// Robot1 go backward.
	case 's':
	case 'S':
		robot1.SetVelocity(1.5, 1.5);
		break;
	// Robot1 stop.
	case 'x':
	case 'X':
		robot1.SetVelocity(0.0, 0.0);
		break;
	// Robot1 turn left.
	case 'a':
	case 'A':
		robot1.SetVelocity(-1.0, 1.0);
		break;
	// Robot1 turn right.
	case 'd':
	case 'D':
		robot1.SetVelocity(1.0, -1.0);
		break;
	default:
		break;
	}
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

// User Rendering function.
// In the example below, the function saves the position of robot1 at every rendering time step,
// then draw a trace line of it.
void User_CBFunc_Render(void* pvData)
{
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);
		// Trace of Robot1
		glColor3f(0.1f, 0.8f, 0.1f);
		glBegin(GL_LINE_STRIP);
		for(int i = 0 ; i < trace1.get_size(); ++i)
		{
			glVertex3f(trace1[i][0], trace1[i][1], trace1[i][2]);
		}
		glEnd();
	
		// Trace of Robot2
		glColor3f(0.8f, 0.3f, 0.1f);
		glBegin(GL_LINE_STRIP);
		for(int i = 0 ; i < trace2.get_size(); ++i)
		{
			glVertex3f(trace2[i][0], trace2[i][1], trace2[i][2]);
		}
		glEnd();
	
	glEnable(GL_LIGHTING);
	glPopAttrib();
	trace1.add_tail(robot1.GetPosition());	//<- save current position of robot1.
	trace2.add_tail(robot2.GetPosition());	//<- save current position of robot2.
}

