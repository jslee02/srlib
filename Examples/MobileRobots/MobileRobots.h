#ifndef EXAMPLE_MOBILEROBOT
#define EXAMPLE_MOBILEROBOT

#include "srDyn/srSpace.h"

class MobileRobot : public srSystem
{
public:
	// main body of mobile robot
	srLink			m_Body;	
	srCollision		m_BodyCollision;

	// left wheel
	srLink			m_LeftWheel;
	srCollision		m_LeftWheelCollision;
	srRevoluteJoint	m_LeftMotor;

	// right wheel
	srLink			m_RightWheel;
	srCollision		m_RightWheelCollision;
	srRevoluteJoint	m_RightMotor;

	// caster
	srLink			m_Caster;
	srCollision 	m_CasterCollision;
	srWeldJoint 	m_CasterFixation;

	// sensors
	srIRSensor		m_IR;
	srRangeFinder	m_RF;

public:
	void		SetVelocity(real l, real r) {
					m_LeftMotor.m_State.m_rCommand = l;
					m_RightMotor.m_State.m_rCommand = r;
				};

	srSystem*	BuildRobot(SE3 T = SE3(Vec3(0.0, 0.0, 0.2))) {
					m_Body.GetGeomInfo().SetShape(srGeometryInfo::BOX);
					m_Body.GetGeomInfo().SetDimension(0.4, 0.3, 0.2);
					m_Body.GetGeomInfo().SetColor(0.5, 0.5, 0.8);
					m_Body.UpdateInertia(10);
					m_BodyCollision.GetGeomInfo().SetShape(srGeometryInfo::BOX);
					m_BodyCollision.GetGeomInfo().SetDimension(0.4, 0.3, 0.2);
					m_BodyCollision.SetLocalFrame(SE3());
					m_Body.AddCollision(&m_BodyCollision);

					m_IR.SetRange(1.0, 0.1);
					m_IR.SetLocalFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(-0.2, 0.0, -0.07)));
					m_Body.AddSensor(&m_IR);

					m_RF.SetRange(2.0, 0.1);
					m_RF.SetSpread(120);	m_RF.SetResolution(5);
					m_RF.SetLocalFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.2, 0.0, 0.0)));
					m_Body.AddSensor(&m_RF);

					m_LeftWheel.GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
					m_LeftWheel.GetGeomInfo().SetDimension(0.15, 0.05);		// Diameter, depth.
					m_LeftWheel.GetGeomInfo().SetColor(0.1, 0.1, 0.1);
					m_LeftWheel.UpdateInertia(10);
					//	m_LeftWheel.SetFriction(1.5);
					m_LeftWheelCollision.GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
					m_LeftWheelCollision.GetGeomInfo().SetDimension(0.15, 0.05);	// Diameter
					m_LeftWheelCollision.SetLocalFrame(SE3());
					m_LeftWheel.AddCollision(&m_LeftWheelCollision);

					m_RightWheel.GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
					m_RightWheel.GetGeomInfo().SetDimension(0.15, 0.05);	// Diameter, depth.
					m_RightWheel.GetGeomInfo().SetColor(0.1, 0.1, 0.1);
					m_RightWheel.UpdateInertia(10);
					//	m_RightWheel.SetFriction(1.5);
					m_RightWheelCollision.GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
					m_RightWheelCollision.GetGeomInfo().SetDimension(0.15, 0.05);	// Diameter
					m_RightWheelCollision.SetLocalFrame(SE3());
					m_RightWheel.AddCollision(&m_RightWheelCollision);

					m_Caster.GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
					m_Caster.GetGeomInfo().SetDimension(0.1);				// Diameter
					m_Caster.UpdateInertia(10);
					m_Caster.SetFriction(0.0);	// Attach rear wheel with weld joint and make it frictionless.
					m_CasterCollision.GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
					m_CasterCollision.GetGeomInfo().SetDimension(0.1);
					m_CasterCollision.SetLocalFrame(SE3());
					m_Caster.AddCollision(&m_CasterCollision);


					m_LeftMotor.SetActType(srJoint::VELOCITY);
					m_LeftMotor.MakePositionLimit(false);
					m_LeftMotor.SetParentLink(&m_Body);
					m_LeftMotor.SetChildLink(&m_LeftWheel);
					m_LeftMotor.SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI/2),Vec3(0.1, -0.2, -0.1)));
					m_LeftMotor.SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 0.0)));

					m_RightMotor.SetActType(srJoint::VELOCITY);
					m_RightMotor.MakePositionLimit(false);
					m_RightMotor.SetParentLink(&m_Body);
					m_RightMotor.SetChildLink(&m_RightWheel);
					m_RightMotor.SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI/2),Vec3(0.1, 0.2, -0.1)));
					m_RightMotor.SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 0.0)));

					m_CasterFixation.SetParentLink(&m_Body);
					m_CasterFixation.SetChildLink(&m_Caster);
					m_CasterFixation.SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(-0.2, 0.0, -0.125)));
					m_CasterFixation.SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0),Vec3(0.0, 0.0, 0.0)));


					m_Body.SetFrame(T);
					this->SetBaseLink(&m_Body);
					this->SetBaseLinkType(srSystem::DYNAMIC);
					this->SetSelfCollision(false);

					return this;
				};
	Vec3		GetPosition() {
					return m_Body.GetPosition();
				}
};

#endif	//EXAMPLE_MOBILEROBOT
