// Robot Kit - hexapod.c

// Copyright 2010 Alistair Buxton <a.j.buxton@gmail.com>

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "Ogre.h"
#include <btBulletDynamicsCommon.h>

using namespace Ogre;

#include "parts.h"
#include "hexapod.h"

Hexapod::Hexapod(SceneManager *s, btDynamicsWorld *dw) {

//	t = new Hip(s, s->getRootSceneNode()->createChildSceneNode(), dw, RIGHT);
//	t->node->setPosition(0,0.2,0.0);

	t = new Torso(s, s->getRootSceneNode()->createChildSceneNode(), dw);
	t->node->setPosition(0,0.2,0.0);
	t->node->yaw(Degree(180));
	t->node->roll(Degree(5));

	t->CreateBody();

	for(int n=0;n<3;n++) {
		Quaternion qa,qb;
		Vector3 va,vb;

		lh[n] = new Hip(s, s->getRootSceneNode()->createChildSceneNode(), dw, LEFT);
		t->Connect(n, lh[n], 0);

		lf[n] = new Femur(s, s->getRootSceneNode()->createChildSceneNode(), dw, LEFT);
		lh[n]->Connect(1, lf[n], 0, true);

		lt[n] = new Tibia(s, s->getRootSceneNode()->createChildSceneNode(), dw, LEFT);
		lf[n]->Connect(1, lt[n], 0);

		rh[n] = new Hip(s, s->getRootSceneNode()->createChildSceneNode(), dw, RIGHT);
		t->Connect(n+3, rh[n], 0);

		rf[n] = new Femur(s, s->getRootSceneNode()->createChildSceneNode(), dw, RIGHT);
		rh[n]->Connect(1, rf[n], 0, true);

		rt[n] = new Tibia(s, s->getRootSceneNode()->createChildSceneNode(), dw, RIGHT);
		rf[n]->Connect(1, rt[n], 0);
	}


}

void Hexapod::think(btScalar timestep) {

	// this function is called before each physics update
	// connect angular motors to servo network lib here

	// this simple walk motion is done by setting the hinge limits
	// angular motors should be used instead

	static btScalar time = 1.0;
	static btScalar speed = 5.0;
	time += timestep;

	#define LIMITS 0.5, 0.3, 1.0

	btScalar sine = sin(time*speed);
	btScalar a1 = 0.2*(sine);

	lf[1]->hinge->setLimit(-a1,-a1, LIMITS);
	rf[0]->hinge->setLimit( a1, a1, LIMITS);
	rf[2]->hinge->setLimit( a1, a1, LIMITS);

	rf[1]->hinge->setLimit(-a1,-a1, LIMITS);
	lf[0]->hinge->setLimit( a1, a1, LIMITS);
	lf[2]->hinge->setLimit( a1, a1, LIMITS);

	btScalar cosine = cos(time*speed);
	btScalar a2 = -0.3*(cosine);

	lh[1]->hinge->setLimit(-a2,-a2, LIMITS);
	rh[0]->hinge->setLimit( a2, a2, LIMITS);
	rh[2]->hinge->setLimit( a2, a2, LIMITS);

	rh[1]->hinge->setLimit(-a2,-a2, LIMITS);
	lh[0]->hinge->setLimit( a2, a2, LIMITS);
	lh[2]->hinge->setLimit( a2, a2, LIMITS);

        lt[0]->hinge->setLimit(0.2,0.2);
        lt[1]->hinge->setLimit(0.2,0.2);
        lt[2]->hinge->setLimit(0.2,0.2);

        rt[0]->hinge->setLimit(-0.2,-0.2);
        rt[1]->hinge->setLimit(-0.2,-0.2);
        rt[2]->hinge->setLimit(-0.2,-0.2);


}

SceneNode *Hexapod::GetSceneNode(void) {
	return t->node;
}

Hexapod::~Hexapod() { ; }
