#ifndef SRLIB_IR_RANGE_FINDER
#define SRLIB_IR_RANGE_FINDER

#include "srDyn/srSensor.h"
#include "LieGroup/_array.h"

#define	MAX_SPOT	1080 + 1

/*!
	\class srRangeFinder
	\brief Class represents IR scanner sensor
*/
class srRangeFinder : public srSensor
{
public:
	/*!
	Maximum detection range. This should be specified by user.
	This must be greater than or equal to m_MinRange.
	Default is 1.0.
	*/
	real	m_MaxRange;
	/*!
	Minimum detection range. This should be specified by user.
	This value must be between zero and m_MaxRange.
	Default is 0.0.
	*/
	real	m_MinRange;
	/*!
		Detection spread angle in degree. This should be specified by user.
		This must be greater than zero and lower than 360.
		Default is 180.
	*/
	int		m_Spread;
	/*!
		Angle between adjacent spots in degree. This should be specified by user.
		This must be greater than zero and lower than spread angle.
		Default is 1.0.
	*/
	real	m_Resolution;

public:
	/*!
	Get maximum detection range.
	*/
	real	GetMaxRange();
	/*!
	Get minimum detection range.
	*/
	real	GetMinRange();
	/*!
	Set maximum detection range.
	*/
	void	SetMaxRange(real );
	/*!
	Set minimum detection range.
	*/
	void	SetMinRange(real );
	/*!
	Set maximum and minimum detection range.
	*/
	void	SetRange(real max, real min = 0.0);
	/*!
		Get resolution.
	*/
	real	GetResolution();
	/*!
		Set resolution.
	*/
	void	SetResolution(real );
	/*!
		Get spread angle.
	*/
	int		GetSpread();
	/*!
		Set spread angle.
	*/
	void	SetSpread(int );


//////////////////////////////////////////////////////////////////////////
public:
	/*!
		Constructor.
	*/
	srRangeFinder();

	/*!
	List of collision entities that sensor can detect.
	*/
	_array<srCollision*>	m_Objects;

	/*!
		Numbers of IR rays. (m_Spread / m_Resolution + 1)
	*/
	int		m_NumSpots;		

	/*!
		Detected values of sensor.
	*/
	real	m_DetectedValue[MAX_SPOT];

	/*!
		Half of spread angle in radian.
	*/
	real	m_SpreadRad;
	/*!
		Resolution in radian
	*/
	real	m_ResRad;

public:
	/*!
		Get numbers of IR rays
	*/
	int		GetNumSpots();
	/*!
		Reset detected value of sensor.
	*/
	void	ResetSensor();
	/*!
		Get Detected values of sensor.
	*/
	real*	GetDetectedValue();

	/*!
		Add collision entity to detection list.
	*/
	void	AddObject(srCollision* );
	/*!
		Remove collision entity to detection list.
	*/
	void	RemoveObject(srCollision* );
	/*!
		Reset detection list.
	*/
	void	ClearObject();

	/*!
		Initialize sensor. This function is called once before simulation.
	*/
	void	Initialize();
	/*!
		Sensor operate.
	*/
	void	Detect();

protected:
	void	_DetectOneSpot(int &idx, Vec3 &direction, Vec3 &position);
};

#endif

