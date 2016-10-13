#include "srDyn/srConstraint.h"

//***********************************************//
// Compile Option
#define MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR		4	// 현재 3 or 4 만 지원
#define MAX_NUM_OF_CONTACTJACOBIAN					MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR*3

//--- Variables
//--
#define CONTACT_THRESHOLD							0.02
#define FRICTIONCOEFF_EPS							0.0001
#define RESTITUTIONCOEFF_EPS						0.0001

//-- Dependent Variables
#define CONTACT_THRESHOLD_SQR						CONTACT_THRESHOLD*CONTACT_THRESHOLD


//**********************************************************************//
// Contact Constraint
struct MassPair
{
	srLink			*pLeftMass;
	srLink			*pRightMass;
	bool			bCollidable;
};

struct Contact
{
	Vec3	globalpoint;
	real	Penetration;
	Vec3	NormalVec;

	Vec3	localpoint_Left;
	Vec3	localpoint_Right;

	bool	bActive;
	int		lifetime;

	real	lambda[3];
	// warm starting을 위한 변수들
};
typedef Contact* ContactPtr;

class ContactConstraint;

class CollisionPair
{
public:
	srCollision		*pLeftCol;
	srCollision		*pRightCol;

	ContactConstraint *pContactConstraint;

	bool		PRESTEP_Find_NarrowPhase_Algorithm();
	bool		PRESTEP_Find_RoughCheck__Algorithm();

	bool		RoughCheck____________________________________MARK7();
	int			NarrowPhase_Algorithm_________________________MARK7();

protected:
	bool	(CollisionPair::*m_pfn_RoughCheck_Algoritm)();

	bool	_PlaneOthers________RoughCheck_TEMP();
	bool	_OthersPlane________RoughCheck_TEMP();
	bool	_OthersOthers_______RoughCheck_TEMP();


	// Under construction
	/*
	bool	_BoxBox_____________RoughCheck();
	bool	_BoxSphere__________RoughCheck();
	bool	_BoxCylinder________RoughCheck();
	bool	_BoxCapsule_________RoughCheck();

	bool	_SphereBox__________RoughCheck();
	bool	_SphereSphere_______RoughCheck();
	bool	_SphereCylinder_____RoughCheck();
	bool	_SphereCapsule______RoughCheck();

	bool	_CylinderBox________RoughCheck();
	bool	_CylinderSphere_____RoughCheck();
	bool	_CylinderCylinder___RoughCheck();
	bool	_CylinderCapsule____RoughCheck();

	bool	_CapsuleBox_________RoughCheck();
	bool	_CapsuleSphere______RoughCheck();
	bool	_CapsuleCylinder____RoughCheck();
	bool	_CapsuleCapsule_____RoughCheck();

	bool	_PlaneBox___________RoughCheck();
	bool	_PlaneSphere________RoughCheck();
	bool	_PlaneCylinder______RoughCheck();
	bool	_PlaneCapsule_______RoughCheck();

	bool	_BoxPlane___________RoughCheck();
	bool	_SpherePlane________RoughCheck();
	bool	_CylinderPlane______RoughCheck();
	bool	_CapsulePlane_______RoughCheck();
	*/



	int		(CollisionPair::*m_pfn_NarrowPhase_Algoritm)();

	int		_BoxBox_____________MARK8();
	int		_BoxSphere__________MARK8();
	int		_BoxCylinder________MARK8();
	int		_BoxCapsule_________MARK8();

	int		_SphereBox__________MARK8();
	int		_SphereSphere_______MARK8();
	int		_SphereCylinder_____MARK8();
	int		_SphereCapsule______MARK8();

	int		_CylinderBox________MARK8();
	int		_CylinderSphere_____MARK8();
	int		_CylinderCylinder___MARK8();
	int		_CylinderCapsule____MARK8();

	int		_CapsuleBox_________MARK8();
	int		_CapsuleSphere______MARK8();
	int		_CapsuleCylinder____MARK8();
	int		_CapsuleCapsule_____MARK8();

	int		_PlaneBox___________MARK8();
	int		_PlaneSphere________MARK8();
	int		_PlaneCylinder______MARK8();
	int		_PlaneCapsule_______MARK8();

	int		_BoxPlane___________MARK8();
	int		_SpherePlane________MARK8();
	int		_CylinderPlane______MARK8();
	int		_CapsulePlane_______MARK8();
};

class ContactConstraint : public Constraint
{
public:
	ContactConstraint()
	{
		nd = 0;
		type1 = false;
	}

	~ContactConstraint()
	{
	}

	// static memebers
	static void SetErp(real _erp);
	static real erp_contact;

	static void SetAllowedPenetration(real _allowedpenetration);
	static real allowedpenetration;

	static void SetBouncingThreshold(real _bouncingthreshold);
	static real bouncing_threshold;

	static void SetMaximumErpVelocity(real _maximum_erp_velocity);
	static real maximum_erp_velocity;

	static void	SetMaximumBouncingVelocity(real _maximum_bouncing_velocity);
	static real maximum_bouncing_velocity;


	// member variables

	//=== PRESTEP ===//
	srLink			*pLeftMass;
	srLink			*pRightMass;

	srSystem			*pLSystem;
	srSystem			*pRSystem;

	bool			bFriction;
	real			rFriction_coeff;
	bool			bBounce;
	real			rRestitution_coeff;

	//=== RUNTIME ===//
	int				nContactPts;
	bool			bActive;

	ContactPtr		pContactPts[MAX_NUM_OF_CONTACTPOINT_PER_MASSPAIR];
	dse3			JacobianLeft[MAX_NUM_OF_CONTACTJACOBIAN];
	dse3			JacobianRight[MAX_NUM_OF_CONTACTJACOBIAN];


	// member functions

	/// pre-step
	static	real	_PRESTEP_Calculate_RestitutionCoeff________MARK7(real leftmat, real rightmat);
	static	real	_PRESTEP_Calculate_FrictionCoeff___________MARK7(real leftmat, real rightmat);

	void			_PRESTEP_Set_Virual_Function_______________MARK7();
	void			_ALLOC_pContactPts();
	void			_FREE_pContactPts();

	void			PRESTEP_Set_MassPair_______________________MARK9(MassPair* pmasspair);

	bool			PRESTEP_Test_Collision_____________________MARK7();


	/// runtime
	void			Filter_InvalidContactPoints_And_Refresh____MARK7(); 
	void			Filter_InactiveContactPoints_______________MARK7();
	void			Prepare_for_Solver_________________________MARK7();

	void			HowAboutThisPoint(Vec3& point , Vec3& normal , real& penetration);

	int				FindNearestContactPoint(Vec3 & localpoint_Left); // Left local point 와의 차이를 비교
	void			InsertContactPointAt(Vec3 & point, Vec3 & normal, real & penetration, Vec3& localpoint_Left, int insertIdx);
	int				SortContactPoints(Vec3& localpoint_Left, real & penetration);
	void			RemoveContactPoint(int _index);
	void			TangentPlane(const Vec3& n, Vec3& p, Vec3& q);




	// union-find  --Function Pointer 
	void		UF_Unite();
	void		(ContactConstraint::*m_pfn_unite)();
	void		_unite_do_something();
	void		_unite_do_nothing();


	void	GetRelVelocity(real * sjari);
	void	(ContactConstraint::*m_pfn_getrelvelocity)(real * sjari);
	void	_getrelvelocity_LeftMove(real * sjari);
	void	_getrelvelocity_RightMove(real * sjari);
	void	_getrelvelocity_BothMove(real * sjari);

	// VIRTUAL FUNCTIONS
	void	GetInformation(ConstraintInfo * info);

	// impulse test --Function Pointer

	void	ApplyImpulse(int _idx);
	void	(ContactConstraint::*m_pfn_applyimpulse)(int _idx);
	void	(ContactConstraint::*m_pfn_applyimpulse_Right)(int _idx);
	void	(ContactConstraint::*m_pfn_applyimpulse_Left)(int _idx);
	void	_applyimpulse_FS_Right(int _idx);
	void	_applyimpulse_FS_Left(int _idx);
	void	_applyimpulse_FS_Both(int _idx);
	void	_applyimpulse_L_Right(int _idx);
	void	_applyimpulse_L_Left(int _idx);
	void	_applyimpulse_Both(int _idx);

	void	GetDelVelocity(real * sjari);
	void	(ContactConstraint::*m_pfn_getdelvelocity)(real * sjari);
	void	_getdelvelocity_LeftMove(real *sjari);
	void	_getdelvelocity_RightMove(real *sjari);
	void	_getdelvelocity_BothMove(real *sjari);
	void	__getdelvelocity_Leftside(real *sjari);
	void	__getdelvelocity_Rightside(real *sjari);
	void	__getdelvelocity_Bothsides(real *sjari);
	void	__getdelvelocity_None(real *sjari);

	void	Excite();
	void	(ContactConstraint::*m_pfn_excite)();
	void	_excite_LeftSystem();
	void	_excite_RightSystem();
	void	_excite_BothSystem();

	void	UnExcite();
	void	(ContactConstraint::*m_pfn_unexcite)();
	void	_unexcite_LeftSystem();
	void	_unexcite_RightSystem();
	void	_unexcite_BothSystem();

	void	SetImpulse(real * _lambda);

	srSystem*	UF_Find_Constraint();
	srSystem*	(ContactConstraint::*m_pfn_find_constraint)();
	srSystem*	_find_return_leftID();
	srSystem*	_find_return_rightID();

	friend	class srDYN;
};
//**********************************************************************//

