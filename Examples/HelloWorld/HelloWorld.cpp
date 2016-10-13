#include "Renderer/SimpleViewer.h"
#include "srDyn/srSpace.h"

#include <time.h>
#include <math.h>  

// Simple Viewer
srSimpleViewer& gViewer = srSimpleViewer::GetInstance();

// World and Entities

#define EXAMPLE_NUM_OF_WORM 5
#define EXAMPLE_FENCE 22
#define EXAMPLE_FENCE_HEIGHT 5


srSpace gSpace;

srSystem gGroundSystem;
srLink	gGround;
srCollision gGroundCollision;

srSystem gGroundSystem_Addition[4];
srLink	gGroundLink_Addition[4];
srCollision gGroundCollision_Addition[4];



srSystem gSystem[EXAMPLE_NUM_OF_WORM];
//srLink gLink[EXAMPLE_NUM_OF_WORM][3];
//srCollision gCollision[EXAMPLE_NUM_OF_WORM][3];
//srRevoluteJoint gJoint[EXAMPLE_NUM_OF_WORM][2];

srLink gLink[EXAMPLE_NUM_OF_WORM][5];
srCollision gCollision[EXAMPLE_NUM_OF_WORM][4];
srRevoluteJoint gJoint[EXAMPLE_NUM_OF_WORM][4];
srBallJoint gBallJoint[EXAMPLE_NUM_OF_WORM][4];




double gAmplitude[EXAMPLE_NUM_OF_WORM];
double gFrequency[EXAMPLE_NUM_OF_WORM];

bool g_bRunSimulation = false;

// Modeling and Control Functions
void User_Modeling();
void User_SimulationSetting();

// Callback functions for simulation and control
void User_CBFunc_SimulationLoop();
void User_CBFunc_ControlLoop();

// Callback functions for User Key Control
void User_CBFunc_Pause_DYN(void);
void User_CBFunc_Run_DYN(void);

int main(int argc, char **argv)
{
	// STEP 1: Viewer initialization
	// srSimpleViewer render scene using OpenGL GLUT.
	// ...OpenGL GLUT need argc and argv for initializing.
	gViewer.Init(&argc, argv);

	// STEP 2: Robot Modeling
	User_Modeling();
	
	// STEP 3: Simulation settings
	User_SimulationSetting();

	// STEP 4: Run window view.
	gViewer.Run();

	return 0;
}

inline float rand_zero_to_max(float max)
{
	return ( (float)( (rand())/(RAND_MAX+1.0) * max ) );
};

void User_Modeling()
{
	int i;
	unsigned seed = (unsigned)time(NULL);
	srand(seed);


	///- Ground
	// Geometry information
	gGround.GetGeomInfo().SetShape(srGeometryInfo::PLANE);
	// Inertia update
	gGround.UpdateInertia();
	// Collision entity
	gGroundCollision.GetGeomInfo().SetShape(srGeometryInfo::PLANE);
	gGroundCollision.SetLocalFrame(SE3());
	// Add collision entity
	gGround.AddCollision(&gGroundCollision);
	// Make system.
	gGroundSystem.SetBaseLink(&gGround);
	gGroundSystem.SetBaseLinkType(srSystem::FIXED);
	// Add to Space
	gSpace.AddSystem(&gGroundSystem);


	// 1
	// Geometry information
	gGroundLink_Addition[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundLink_Addition[0].GetGeomInfo().SetDimension(EXAMPLE_FENCE, 1.0, EXAMPLE_FENCE_HEIGHT);
	gGroundLink_Addition[0].GetGeomInfo().SetColor(0.5f, 0.5f, 0.5f);
	gGroundLink_Addition[0].SetFrame(SE3(Vec3(0.0, EXAMPLE_FENCE/2, EXAMPLE_FENCE_HEIGHT/2)));
	// Inertia update
	gGroundLink_Addition[0].UpdateInertia();
	// Collision entity
	gGroundCollision_Addition[0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundCollision_Addition[0].GetGeomInfo().SetDimension(EXAMPLE_FENCE, 1.0, EXAMPLE_FENCE_HEIGHT);
	gGroundCollision_Addition[0].SetLocalFrame(SE3());
	// Add collision entity
	gGroundLink_Addition[0].AddCollision(&gGroundCollision_Addition[0]);
	// Make system.
	gGroundSystem_Addition[0].SetBaseLink(&gGroundLink_Addition[0]);
	gGroundSystem_Addition[0].SetBaseLinkType(srSystem::FIXED);
	// Add to Space
	gSpace.AddSystem(&gGroundSystem_Addition[0]);

	// 2
	// Geometry information
	gGroundLink_Addition[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundLink_Addition[1].GetGeomInfo().SetDimension(EXAMPLE_FENCE, 1.0, EXAMPLE_FENCE_HEIGHT);
	gGroundLink_Addition[1].GetGeomInfo().SetColor(0.5f, 0.5f, 0.5f);
	gGroundLink_Addition[1].SetFrame(SE3(Vec3(0.0, -EXAMPLE_FENCE/2, EXAMPLE_FENCE_HEIGHT/2)));
	// Inertia update
	gGroundLink_Addition[1].UpdateInertia();
	// Collision entity
	gGroundCollision_Addition[1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundCollision_Addition[1].GetGeomInfo().SetDimension(EXAMPLE_FENCE, 1.0, EXAMPLE_FENCE_HEIGHT);
	gGroundCollision_Addition[1].SetLocalFrame(SE3());
	// Add collision entity
	gGroundLink_Addition[1].AddCollision(&gGroundCollision_Addition[1]);
	// Make system.
	gGroundSystem_Addition[1].SetBaseLink(&gGroundLink_Addition[1]);
	gGroundSystem_Addition[1].SetBaseLinkType(srSystem::FIXED);
	// Add to Space
	gSpace.AddSystem(&gGroundSystem_Addition[1]);

	// 3
	// Geometry information
	gGroundLink_Addition[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundLink_Addition[2].GetGeomInfo().SetDimension(1.0, EXAMPLE_FENCE, EXAMPLE_FENCE_HEIGHT);
	gGroundLink_Addition[2].GetGeomInfo().SetColor(0.5f, 0.5f, 0.5f);
	gGroundLink_Addition[2].SetFrame(SE3(Vec3(EXAMPLE_FENCE/2, 0.0, EXAMPLE_FENCE_HEIGHT/2)));
	// Inertia update
	gGroundLink_Addition[0].UpdateInertia();
	// Collision entity
	gGroundCollision_Addition[2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundCollision_Addition[2].GetGeomInfo().SetDimension(1.0, EXAMPLE_FENCE, EXAMPLE_FENCE_HEIGHT);
	gGroundCollision_Addition[2].SetLocalFrame(SE3());
	// Add collision entity
	gGroundLink_Addition[2].AddCollision(&gGroundCollision_Addition[2]);
	// Make system.
	gGroundSystem_Addition[2].SetBaseLink(&gGroundLink_Addition[2]);
	gGroundSystem_Addition[2].SetBaseLinkType(srSystem::FIXED);
	// Add to Space
	gSpace.AddSystem(&gGroundSystem_Addition[2]);

	// 4
	// Geometry information
	gGroundLink_Addition[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundLink_Addition[3].GetGeomInfo().SetDimension(1.0, EXAMPLE_FENCE, EXAMPLE_FENCE_HEIGHT);
	gGroundLink_Addition[3].GetGeomInfo().SetColor(0.5f, 0.5f, 0.5f);
	gGroundLink_Addition[3].SetFrame(SE3(Vec3(-EXAMPLE_FENCE/2, 0.0, EXAMPLE_FENCE_HEIGHT/2)));
	// Inertia update
	gGroundLink_Addition[3].UpdateInertia();
	// Collision entity
	gGroundCollision_Addition[3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
	gGroundCollision_Addition[3].GetGeomInfo().SetDimension(1.0, EXAMPLE_FENCE, EXAMPLE_FENCE_HEIGHT);
	gGroundCollision_Addition[3].SetLocalFrame(SE3());
	// Add collision entity
	gGroundLink_Addition[3].AddCollision(&gGroundCollision_Addition[3]);
	// Make system.
	gGroundSystem_Addition[3].SetBaseLink(&gGroundLink_Addition[3]);
	gGroundSystem_Addition[3].SetBaseLinkType(srSystem::FIXED);
	// Add to Space
	gSpace.AddSystem(&gGroundSystem_Addition[3]);




	for (i = 0 ; i < EXAMPLE_NUM_OF_WORM ; i++)
	{
		///- Mass1
		// Geometry information
		gLink[i][0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gLink[i][0].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);

		// Inertia update
		gLink[i][0].UpdateInertia(500);

		// Collision entity
		gCollision[i][0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[i][0].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);
		gCollision[i][0].SetLocalFrame(SE3());
		// Add collision entity
		gLink[i][0].AddCollision(&gCollision[i][0]);
		gLink[i][0].SetFrame(EulerZYX(Vec3(0.0,0.0,0.0),Vec3(0.0, 0.0, 7 + i*7)));


		/////- Mass1
		//// Geometry information
		//gLink[i][0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		//gLink[i][0].GetGeomInfo().SetDimension(3.0, 2.0, 1.0);
		//gLink[i][0].GetGeomInfo().SetColor(rand_zero_to_max(1.0f), rand_zero_to_max(1.0f), rand_zero_to_max(1.0f));
		//// Inertia update
		//gLink[i][0].UpdateInertia();

		//// Collision entity
		//gCollision[i][0].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		//gCollision[i][0].GetGeomInfo().SetDimension(3.0, 2.0, 1.0);
		//gCollision[i][0].SetLocalFrame(SE3());
		//// Add collision entity
		//gLink[i][0].AddCollision(&gCollision[i][0]);
		//// Initial position and orientation.
		//gLink[i][0].SetFrame(SE3(Vec3(0.0, 0.0, 14 + i*12)));


		///- Mass2
		// Geometry information
		gLink[i][1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gLink[i][1].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);

		// Inertia update
		gLink[i][1].UpdateInertia();

		// Collision entity
		gCollision[i][1].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[i][1].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);
		gCollision[i][1].SetLocalFrame(SE3());
		// Add collision entity
		gLink[i][1].AddCollision(&gCollision[i][1]);

		///- Mass3
		// Geometry information
		gLink[i][2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gLink[i][2].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);

		// Inertia update
		gLink[i][2].UpdateInertia();

		// Collision entity
		gCollision[i][2].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[i][2].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);
		gCollision[i][2].SetLocalFrame(SE3());
		// Add collision entity
		gLink[i][2].AddCollision(&gCollision[i][2]);




		///- Mass4
		// Geometry information
		gLink[i][3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gLink[i][3].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);

		// Inertia update
		gLink[i][3].UpdateInertia();

		// Collision entity
		gCollision[i][3].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gCollision[i][3].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);
		gCollision[i][3].SetLocalFrame(SE3());
		// Add collision entity
		gLink[i][3].AddCollision(&gCollision[i][3]);


		///- Mass5
		// Geometry information
		gLink[i][4].GetGeomInfo().SetShape(srGeometryInfo::BOX);
		gLink[i][4].GetGeomInfo().SetDimension(1.0, 1.0, 3.0);

		// Inertia update
		gLink[i][4].UpdateInertia(500);



		///- Joint1
		// Actuator type
		gBallJoint[i][0].SetActType(srJoint::PASSIVE);
		// Parent and child links
		gBallJoint[i][0].SetParentLink(&gLink[i][0]);
		gBallJoint[i][0].SetChildLink(&gLink[i][1]);
		gBallJoint[i][0].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, -2.5)));
		gBallJoint[i][0].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 2.5)));



		///// - Joint1
		//gJoint[i][0].SetActType(srJoint::PASSIVE);
		//// Parent and child links
		//gJoint[i][0].SetParentLink(&gLink[i][0]);
		//gJoint[i][0].SetChildLink(&gLink[i][1]);
		//gJoint[i][0].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI/2, 0.0),Vec3(0.0, 0.0, -2.5)));
		//gJoint[i][0].SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI/2, 0.0),Vec3(0.0, 0.0, 2.5)));

		//gJoint[i][0].MakePositionLimit(true);
		//gJoint[i][0].SetPositionLimit(-360, 360);


		/////- Joint1
		//// Actuator type
		//gJoint[i][0].SetActType(srJoint::PASSIVE);
		//// Parent and child links
		//gJoint[i][0].SetParentLink(&gLink[i][0]);
		//gJoint[i][0].SetChildLink(&gLink[i][1]);
		//gJoint[i][0].SetParentLinkFrame(EulerZYX(Vec3(SR_PI/2, SR_PI/2, 0.0),Vec3(0.0, 0.0, -0.5)));
		//gJoint[i][0].SetChildLinkFrame(EulerZYX(Vec3(SR_PI/2, SR_PI/2, 0.0),Vec3(0.0, 0.0, 2.5)));

		//gJoint[i][0].MakePositionLimit(true);
		//gJoint[i][0].SetPositionLimit(-30, 30);

		//- Joint2
		// Actuator type
		gJoint[i][1].SetActType(srJoint::VELOCITY);
		//gJoint[i][1].SetActType(srJoint::PASSIVE);
		// Parent and child links
		gJoint[i][1].SetParentLink(&gLink[i][1]);
		gJoint[i][1].SetChildLink(&gLink[i][2]);
		gJoint[i][1].SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI/2, 0.0),Vec3(0.0, 0.0, -2.5)));
		gJoint[i][1].SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI/2, 0.0),Vec3(0.0, 0.0, 2.5)));

		//gJoint[i][1].MakePositionLimit(false);
		gJoint[i][1].MakePositionLimit(true);
		gJoint[i][1].SetPositionLimit(-360, 360);
		gJoint[i][1].SetTorqueLimit(-10000000, 10000000);


		///- Joint3
		// Actuator type
		gBallJoint[i][2].SetActType(srJoint::PASSIVE);
		// Parent and child links
		gBallJoint[i][2].SetParentLink(&gLink[i][2]);
		gBallJoint[i][2].SetChildLink(&gLink[i][3]);
		gBallJoint[i][2].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, -2.5)));
		gBallJoint[i][2].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 2.5)));

		//gJoint[i][2].MakePositionLimit(true);
		//gJoint[i][2].SetPositionLimit(-360, 360);



		///- Joint4
		// Actuator type
		//gBallJoint[i][3].SetActType(srJoint::PASSIVE);
		//// Parent and child links
		//gBallJoint[i][3].SetParentLink(&gLink[i][3]);
		//gBallJoint[i][3].SetChildLink(&gLink[i][4]);
		//gBallJoint[i][3].SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, -2.5)));
		//gBallJoint[i][3].SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 2.5)));

		//gJoint[i][3].MakePositionLimit(true);
		//gJoint[i][3].SetPositionLimit(-360, 360);


		//gJoint[i][0].m_State.m_rValue[0] = SR_PI/2;
		//gJoint[i][1].m_State.m_rValue[0] = SR_PI/2;
		//gJoint[i][2].m_State.m_rValue[0] = SR_PI/2;
		//gJoint[i][3].m_State.m_rValue[0] = SR_PI/2;

		gBallJoint[i][0].m_State.m_SO3Pos =  EulerZYX(Vec3(0.0, 0.0, SR_PI/2)).GetOrientation();
		gBallJoint[i][2].m_State.m_SO3Pos =  EulerZYX(Vec3(0.0, 0.0, SR_PI/2)).GetOrientation();
		gBallJoint[i][3].m_State.m_SO3Pos =  EulerZYX(Vec3(0.0, 0.0, SR_PI/2)).GetOrientation();

		//gSystem[i].MakeClosedLoop(&gLink[i][0], &gLink[i][4]);

		// Make system.
		gSystem[i].SetBaseLink(&gLink[i][0]);
		gSystem[i].SetBaseLinkType(srSystem::DYNAMIC);

		gSystem[i].SetSelfCollision(true);

		gSpace.AddSystem(&gSystem[i]);
	}

	for (i = 0 ; i < EXAMPLE_NUM_OF_WORM ; i++)
	{
		//gAmplitude[i] = 30.0 + rand_zero_to_max(90.0);
		//gFrequency[i] = 2.0 + rand_zero_to_max(9.0);
		gAmplitude[i] = 15.0 + rand_zero_to_max(30.0);
		gFrequency[i] = 2.0 + rand_zero_to_max(1.0);
	}

	// Space setting
	gSpace.SetTimestep(0.001);
	gSpace.SetGravity(0.0,0.0,-9.8);
	gSpace.SetNumberofSubstepForRendering(50);
}

void User_SimulationSetting()
{

	// Initialize for dynamics simulation.
	gSpace.DYN_MODE_PRESTEP();

	// Set user control loop function.
	gSpace.SET_USER_CONTROL_FUNCTION(User_CBFunc_ControlLoop);

	// Set target space
	gViewer.SetTarget(&gSpace);

	// Set Idle loop function.
	gViewer.SetLoopFunc(User_CBFunc_SimulationLoop);

	// Key mapping
	gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'P');
	gViewer.SetKeyFunc(User_CBFunc_Run_DYN, 'p');

	gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'O');
	gViewer.SetKeyFunc(User_CBFunc_Pause_DYN, 'o');
}

void User_CBFunc_SimulationLoop()
{
	if (g_bRunSimulation)
		gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
}

void User_CBFunc_ControlLoop()
{
	int i;
	double desired_pos, velocity_command;
	for (i = 0 ; i < EXAMPLE_NUM_OF_WORM ; i++ )
	{
		desired_pos = gAmplitude[i] * SR_RADIAN * sin ( gSpace.GetSimulationTime() * gFrequency[i] );
		velocity_command =  30 * ( desired_pos - gJoint[i][1].m_State.m_rValue[0]);
		gJoint[i][1].m_State.m_rCommand = velocity_command;

		real temp = gJoint[i][1].m_State.m_rCommand;// = velocity_command;
	}
}

void User_CBFunc_Pause_DYN(void)
{
	g_bRunSimulation = false;
}

void User_CBFunc_Run_DYN(void)
{
	g_bRunSimulation = true;
}
