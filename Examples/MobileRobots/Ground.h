#ifndef EXAMPLE_GROUND
#define EXAMPLE_GROUND

#include "srDyn/srSpace.h"

class Ground : public srSystem
{
public:
	srLink*		m_Ground;
	srCollision*m_Plane;

public:
				 Ground(): m_Ground(NULL), m_Plane(NULL) {};
				~Ground() { delete m_Ground; delete m_Plane; };
	srSystem*	 BuildGround() {
					if(m_Ground == NULL)	m_Ground = new srLink;
					if(m_Plane == NULL)		m_Plane = new srCollision;
					m_Ground->GetGeomInfo().SetShape(srGeometryInfo::PLANE);
					m_Ground->UpdateInertia();
					m_Plane->GetGeomInfo().SetShape(srGeometryInfo::PLANE);
					m_Plane->SetLocalFrame(SE3());
					m_Ground->AddCollision(m_Plane);
					this->SetBaseLink(m_Ground);
					this->SetBaseLinkType(FIXED);
					return this;
				 };
};
#endif //EXAMPLE_GROUND
