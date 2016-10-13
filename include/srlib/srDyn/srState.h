#ifndef SRLIB_STATE
#define SRLIB_STATE

#include "LieGroup/_array.h"
#include "LieGroup/LieGroup.h"
#include "srDyn/srObject.h"

class srJoint;

class srState
{
public:
	srState();

	/*!
		Position = Position + timestep * Velocity
	*/
	virtual void UpdatePosition(real& timestep) = 0;
	/*!
		Velocity = Velocity + timestep * Acceleration
	*/
	virtual void UpdateVelocity(real& timestep) = 0;
	/*!
		Velocity = Velocity + Delta Velocity
	*/
	virtual void UpdateVelWithDelVel() = 0;
	/*!
		Acceleration = Acceleration + fps * Delta Velocity
	*/
	virtual void UpdateAccWithDelVel(real& fps) = 0;
	/*!
		Force = Force + fps * Impulse
	*/
	virtual void UpdateTorqueWithImp(real& fps) = 0;

	/*!
		Impulse = zero
	*/
	virtual void ResetConstraintImpulse() = 0;
	/*!
		Command = zero
	*/
	virtual void ResetCommand() = 0;
	/*!
		Velocity = zero
	*/
	virtual void ResetVel() = 0;
	/*!
		Acceleration = zero
	*/
	virtual void ResetAcc() = 0;


	// Not used yet. (for closed chain)
	/*!
		Position = Position + timestep * Position Error Velocity
	*/
	virtual	void UpdatePosWithPosErrVel(real & timestep) = 0;
	/*!
		Position Error Velocity = Position Error Velocity + Delta Velocity
	*/
	virtual	void UpdatePosErrVelWithDelVel() = 0;
	/*!
		Position Error Velocity = zero
	*/
	virtual void ResetPosErrVel() = 0;
	/*!
		Position = Position + timestep * ( Velocity + Position Error Velocity )
	*/
	virtual void UpdatePosWithCorrection(real &) = 0;


	virtual void ClearHistory() = 0;
	virtual void PushState() = 0;
	virtual void PopState(int ) = 0;
	virtual void BackupInitState() = 0;
	virtual void RestoreInitState() = 0;
};

//////////////////////////////////////////////////////////////////////////
// Weld Joint State
class srWeldState : public srState
{
public:
	srWeldState() {};

	void UpdatePosition(real& ) {};
	void UpdateVelocity(real& ) {};
	void UpdateVelWithDelVel() {};
	void UpdateAccWithDelVel(real& ) {};
	void UpdateTorqueWithImp(real& ) {};

	void ResetConstraintImpulse() {};
	void ResetCommand() {};
	void ResetVel() {};
	void ResetAcc() {};

	void UpdatePosWithPosErrVel(real &) {};
	void UpdatePosErrVelWithDelVel() {};
	void ResetPosErrVel() {};
	void UpdatePosWithCorrection(real &) {};

	void ClearHistory() {};
	void PushState() {};
	void PopState(int ) {};
	void BackupInitState() {};
	void RestoreInitState() {};

	const srWeldState & operator = (const srWeldState& ) { return *this; };
};


//////////////////////////////////////////////////////////////////////////
// Revolute Joint State
class srRevoluteState : public srState
{
	float	m_InitValue[4];
	_array<float>	m_History;

public:
	/*!
		Index is for states which are position, velocity, acceleration and force in order.
		
		i.g) For revolute joint state.
		m_rValue[0] : Angle of revolute joint.
		m_rValue[1] : Velocity of revolute joint angle.
		m_rValue[2] : Acceleration of revolute joint angle.
		m_rValue[3] : Torque of revolute joint.
	*/
	real m_rValue[4];
	/*!
		User command input. Meaning of this depends on actuator type.

		i.g) For revolute joint.
		[ Actuator Type : Effect (Meaning) ]
		PASSIVE : Not used (Do not effect on simulation).
		TORQUE : Torque of revolute joint.
		VELOCITY : Desired velocity of revolute joint angle.
		HYBRID : Acceleration of revolute joint angle.
	*/
	real m_rCommand;

	real m_rPosErrVel;
	real m_rDelVel;
	real m_rImp;

public:
	srRevoluteState();
	srRevoluteState(srRevoluteState& );

	void UpdatePosition(real& );
	void UpdateVelocity(real& );
	void UpdateVelWithDelVel();
	void UpdateAccWithDelVel(real& );
	void UpdateTorqueWithImp(real& );

	void ResetConstraintImpulse();
	void ResetCommand();
	void ResetVel();
	void ResetAcc();

	void UpdatePosWithPosErrVel(real&);
	void UpdatePosErrVelWithDelVel();
	void ResetPosErrVel();
	void UpdatePosWithCorrection(real&);

	void ClearHistory();
	void PushState();
	void PopState(int );
	void BackupInitState();
	void RestoreInitState();

	const srRevoluteState & operator = (const srRevoluteState& );
};

//////////////////////////////////////////////////////////////////////////
// Prismatic Joint State
typedef srRevoluteState srPrismaticState;

//////////////////////////////////////////////////////////////////////////
// Universal Joint State
class srUniversalState : public srState
{
	struct _state {
		float val[2];
	};

	float	m_InitValue[2][4];
	_array<_state>	m_History;

public:
	/*!
		First index is for axes of universal joint.
		Second index is for states which are position, velocity, acceleration and force in order.

		
		i.g) For universal joint state.
		m_rValue[0][0] : First angle of universal joint.
		m_rValue[0][1] : Velocity of universal joint first angle.
		m_rValue[0][2] : Acceleration of universal joint first angle.
		m_rValue[0][3] : Torque of universal joint first axis.

		m_rValue[1][0] : Second angle of universal joint.
		m_rValue[1][1] : Velocity of universal joint second angle.
		m_rValue[1][2] : Acceleration of universal joint second angle.
		m_rValue[1][3] : Torque of universal joint second axis.

	*/
	real	m_rValue[2][4];	
	/*!
		User command inputs. Meaning of this depends on actuator type.
		First index is for axes of universal joint.

		i.g) For universal joint.
		[ Actuator Type : Effect (Meaning) ]
		PASSIVE : Not used (Do not effect on simulation).
		TORQUE : Torque of revolute joint.
		VELOCITY : Desired velocity of revolute joint angle.
		HYBRID : Acceleration of revolute joint angle.

		m_rCommand[0] : Command on first axis of universal joint.
		m_rCommand[1] : Command on second axis of universal joint.
	*/
	real	m_rCommand[2];		


	real	m_rPosErrVel[2];
	real	m_rDelVel[2];
	real	m_rImp[2];

public:
	srUniversalState();
	srUniversalState(srUniversalState& );
	
	void UpdatePosition(real& );
	void UpdateVelocity(real& );
	void UpdateVelWithDelVel();
	void UpdateAccWithDelVel(real& );
	void UpdateTorqueWithImp(real& );

	void ResetConstraintImpulse();
	void ResetCommand();
	void ResetVel();
	void ResetAcc();

	void UpdatePosWithPosErrVel(real &);
	void UpdatePosErrVelWithDelVel();
	void ResetPosErrVel();
	void UpdatePosWithCorrection(real &);

	void ClearHistory();
	void PushState();
	void PopState(int );
	void BackupInitState();
	void RestoreInitState();

	const srUniversalState & operator = (const srUniversalState& );
};

//////////////////////////////////////////////////////////////////////////
// Ball Joint State
class srBallState : public srState
{
	SO3		m_InitPos;
	Vec3	m_InitVel;
	Vec3	m_InitAcc;
	Vec3	m_InitTorque;

	_array<SO3>	m_History;

public:
	SO3		m_SO3Pos;		// ball joint orientation
	Vec3	m_Vel;			// so3
	Vec3	m_Acc;			// so3
	Vec3	m_Torque;		// dso3
	InvVec3	m_EulerAngle;

	Vec3	m_Command;

	// DYN
	Vec3	m_PosErrVel;
	Vec3	m_DelVel;
	Vec3	m_Imp;

	//Limit
	bool	m_bRollPitchLimit;
	bool	m_bYawLimit;
	real	m_RollPitchLimit;
	real	m_YawLimit[2];
	real	m_TorqueLimit;

public:
			srBallState();
			srBallState(srBallState& );

	void UpdatePosition(real& );
	void UpdateVelocity(real& );
	void UpdateVelWithDelVel();
	void UpdateAccWithDelVel(real& );
	void UpdateTorqueWithImp(real& );

	void UpdatePosWithPosErrVel(real &);
	void UpdatePosErrVelWithDelVel();
	void ResetPosErrVel();
	void UpdatePosWithCorrection(real &);

	void ResetConstraintImpulse();
	void ResetCommand();
	void ResetVel();
	void ResetAcc();

	void ClearHistory();
	void PushState();
	void PopState(int );
	void BackupInitState();
	void RestoreInitState();

	const srBallState & operator = (const srBallState& );
};

#endif
