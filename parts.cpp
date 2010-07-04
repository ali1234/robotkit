// Robot Kit - parts.c

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

#include <istream>

#include "Ogre.h"
#include <btBulletDynamicsCommon.h>

using namespace Ogre;

#include "BtOgrePG.h"
#include "BtOgreGP.h"
#include "BtOgreExtras.h"


#include "parts.h"

// parts definitions
// todo: define bullet collision shapes
// and dynamics objects


Part::Part(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw) : dynamicsWorld(dw) {

	node = parent->createChildSceneNode();
	//node->scale(0.01,0.01,0.01); // meshes are made in cm units so scale them
	number++;
	printf("NUMBER: %d\n", number);

	shape = new btSphereShape(0.01);
	mass = 0.05;

}

void Part::NodeAlign(SceneNode *a, SceneNode *b, SceneNode *c, SceneNode *d) {
	// b is a child of a
	// d is a child of c
	// transform c relative to a such that b and d 
	// have same global position and orientation

//	printf("NodeAlign\n");
	Vector3 vb = b->getPosition();
	Vector3 vd = d->getPosition();
	Quaternion qb = b->getOrientation();
	Quaternion qd = d->getOrientation();

/*	printf("%f, %f, %f\n", vb.x, vb.y, vb.z);
	printf("%f, %f, %f\n", vd.x, vd.y, vd.z);

	printf("%f, %f, %f, %f\n", qb.x, qb.y, qb.z, qb.w);
	printf("%f, %f, %f, %f\n", qd.x, qd.y, qd.z, qd.w);
*/

	Quaternion qdiff = (qb * (qd.Inverse()));
	Vector3 vdiff = (vb - (qdiff*vd));

	c->setOrientation(qdiff);	
	c->setPosition(vdiff);
}

btHingeConstraint *Part::Connect(int c1, Part *p2, int c2, bool reversed) {

        // translate the child part (p2) to the rest position
        // and add a bullet angular motor between them

	// note: we don't reuse node align here because we must use the derived
	// position of joint nodes, not the current position which may have been 
	// moved by previous iterations of construction.

        Part *p1 = this;

        Quaternion qo,qa,qb,qdiff,qfinal;
        Vector3 vo,va,vb,vdiff,vfinal;
        
        qo = p1->node->getOrientation();
        vo = p1->node->getPosition();
        qa = p1->GetJointOrientation(c1);
        va = p1->GetJointPosition(c1);
        qb = p2->GetJointOrientation(c2);
        vb = p2->GetJointPosition(c2);

        qdiff = (qa)*(qb.Inverse());

        vdiff = (va) - (qdiff*vb);

        vfinal = vo + (qo*vdiff);
        qfinal = qo*qdiff;      

        p2->node->setPosition(vfinal);
        p2->node->setOrientation(qfinal);

        p2->CreateBody();

        btVector3 v1(va.x, va.y, va.z);
        btVector3 v2(vb.x, vb.y, vb.z);


        btQuaternion q1(qa.x, qa.y, qa.z, qa.w);
        btQuaternion q2(qb.x, qb.y, qb.z, qb.w);

        btTransform t1(q1, v1);
        btTransform t2(q2, v2);

        // make a reversed constraint (swap a & b)
        // we use this because some parts have two servos
        // and some have none. if we didn't reverse then
        // some servos would turn the wrong way
	
        btHingeConstraint *h = new btHingeConstraint(*p2->body, *p1->body, t2, t1, !reversed);

        h->enableMotor(false);
        h->setLimit(0,0);
//      h->setParam(BT_CONSTRAINT_CFM, 100000000);
//      h->setParam(BT_CONSTRAINT_STOP_CFM, 100000000);
//      h->setParam(BT_CONSTRAINT_STOP_ERP, 0.00001);

//      h->enableMotor(true);
//      h->setMaxMotorImpulse(100.0);
//      h->setMotorTarget(0.0, 1.0);
        dynamicsWorld->addConstraint(h, true);  
        p2->hinge = h;
        return h;       
}

void Part::CreateBody(void) {


        //Calculate inertia.
        
        btVector3 inertia;
        shape->calculateLocalInertia(mass, inertia);

        //Create BtOgre MotionState (connects Ogre and Bullet).
        BtOgre::RigidBodyState *state = new BtOgre::RigidBodyState(node);

        //Create the Body.
        body = new btRigidBody(mass, state, shape, inertia);
        dynamicsWorld->addRigidBody(body);

}


void Part::PlaceJoint(SceneNode *jn) {

        jointq.push_back(jn->_getDerivedOrientation());
        jointv.push_back(jn->_getDerivedPosition());
        
}

Quaternion Part::GetJointOrientation(int n) {
        return jointq[n];
}

Vector3 Part::GetJointPosition(int n) {
        return jointv[n];
}


void Part::ReadJoints(std::string filename) {

	int num_joints = 0;
	float d[4];

	std::ifstream f(("media/"+filename).c_str(), std::ios::in);

	f >> num_joints;
	f >> thickness;
	//printf("NUM JOINTS: %d\n", num_joints);
	for(int jj=0;jj<num_joints;jj++) {
		SceneNode *n = node->createChildSceneNode();
		f >> d[0]; f >> d[1]; f >> d[2];
	//	printf("%f, %f, %f\n", d[0], d[1], d[2]);
		n->setPosition(d[0], d[1], d[2]);
		f >> d[0]; f >> d[1]; f >> d[2]; f >> d[3];
		n->setOrientation(d[0], d[1], d[2], d[3]);
		fitting_nodes.push_back(n);
	}

}

Part::~Part() { ; }

int Part::number = 0;

Plate::Plate(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, const char* name) : Part(s, parent, dw) {

	Entity *e = s->createEntity("Plate."+std::string(name)+"."+StringConverter::toString(number), "Plate."+std::string(name)+".mesh");
	node->attachObject(e);

	ReadJoints("Plate."+std::string(name)+".dat");
}

ServoHorn::ServoHorn(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw) : Part(s, parent, dw) {

	Entity *e = s->createEntity("Servo.Horn."+StringConverter::toString(number), "Servo.Horn.mesh");
	node->attachObject(e);

	ReadJoints("Servo.Horn.dat");
}

void ServoHorn::Fix(Plate* p, int joint, int orientation) {
	SceneNode *a,*b,*c,*d;
	a = p->node;
	b = p->fitting_nodes[joint];
	c = node;
	d = fitting_nodes[0];

	d->setPosition(d->getPosition()+Vector3(0,0,-p->thickness*0.5));

	if(orientation == 1)
		d->roll(Degree(180));

	NodeAlign(a,b,c,d);
}

Servo::Servo(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw) : Part(s, parent, dw) {

	Entity *e = s->createEntity("Servo.Body."+StringConverter::toString(number), "Servo.Body.mesh");
	node->attachObject(e);

	ReadJoints("Servo.Body.dat");

	sh = new ServoHorn(s, node, dw);
	NodeAlign(node, fitting_nodes[4], sh->node, sh->fitting_nodes[0]); 
}

void Servo::FixFlushAbove(Plate* p, int joint, int orientation) {
	SceneNode *a,*b,*c,*d;
	a = p->node;
	b = p->fitting_nodes[joint];
	c = node;
	d = fitting_nodes[0];

	d->setPosition(d->getPosition()+Vector3(0,0,p->thickness*0.5));

	if(orientation == 1)
		d->roll(Degree(180));

	NodeAlign(a,b,c,d);
}

void Servo::FixFlushBelow(Plate* p, int joint, int orientation) {
	SceneNode *a,*b,*c,*d;
	a = p->node;
	b = p->fitting_nodes[joint];
	c = node;
	d = fitting_nodes[1];

	d->setPosition(d->getPosition()+Vector3(0,0,-p->thickness*0.5));

	if(orientation == 1)
		d->roll(Degree(180));

	NodeAlign(a,b,c,d);
}



void Servo::FixSlotFar(Plate* p, int joint, int orientation) {
	SceneNode *a,*b,*c,*d;
	a = p->node;
	b = p->fitting_nodes[joint];
	c = node;
	d = fitting_nodes[2];

	d->setPosition(d->getPosition()+Vector3(-p->thickness*0.5,0,0));

	if(orientation == 1)
		d->roll(Degree(180));

	NodeAlign(a,b,c,d);
}

void Servo::FixSlotNear(Plate* p, int joint, int orientation) {
	SceneNode *a,*b,*c,*d;
	a = p->node;
	b = p->fitting_nodes[joint];
	c = node;
	d = fitting_nodes[3];

	d->setPosition(d->getPosition()+Vector3(p->thickness*0.5,0,0));

	if(orientation == 1)
		d->roll(Degree(180));

	NodeAlign(a,b,c,d);
}

Torso::Torso(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw) : Part(s, parent, dw) {

	node->scale(0.01,0.01,0.01);
	tp[0] = new Plate(s, node, dw, "Torso");
	tp[0]->node->setPosition(0,3.195,0);
	tp[1] = new Plate(s, node, dw, "Torso");
	tp[1]->node->setPosition(0,-3.195,0);

	for(int jj=0;jj<6;jj++){
		sh[jj] = new ServoHorn(s, tp[0]->node, dw);
		sh[jj]->Fix(tp[0], jj, 0);
		PlaceJoint(sh[jj]->node);
	}

	// TODO: derive the bt collision shape from the component parts
        btCompoundShape *cs = new btCompoundShape();
        btBoxShape *box = new btBoxShape(btVector3(0.06,0.002,0.1));

        cs->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,-0.03195,0)), box);
        cs->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0.03195,0)), box);

        shape = cs;


}

Femur::Femur(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, bool right) : Part(s, parent, dw) {

	node->scale(0.01,0.01,0.01);
	plate = new Plate(s, node, dw, "Femur");
	//translate to centre of gravity when we can calculate it
	//plate->node->setPosition(0,-3.195,0);

	hip = new ServoHorn(s, plate->node, dw);
	tibia = new ServoHorn(s, plate->node, dw);

	if(right) {
		hip->Fix(plate, 0, 1);
		tibia->Fix(plate, 1, 1);
	} else {
		hip->Fix(plate, 0, 0);
		tibia->Fix(plate, 1, 0);
	}
	PlaceJoint(hip->node);
	PlaceJoint(tibia->node);
}

Tibia::Tibia(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, bool right) : Part(s, parent, dw) {

	node->scale(0.01,0.01,0.01);
	plate = new Plate(s, node, dw, "Tibia");
	//translate to centre of gravity when we can calculate it
	//plate->node->setPosition(0,-3.195,0);

	femur = new Servo(s, plate->node, dw);

        btCompoundShape *cs = new btCompoundShape();
        btBoxShape *box = new btBoxShape(btVector3(0.01,0.002,0.10));

        cs->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0,0,0.03)), box);

        shape = cs;


	if(right) {
		femur->FixFlushAbove(plate, 0, 1);
	} else {
		femur->FixFlushAbove(plate, 0, 0);
	}
	PlaceJoint(femur->sh->node);
}

Hip::Hip(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, bool right) : Part(s, parent, dw) {

	node->scale(0.01,0.01,0.01);
	plates[0] = new Plate(s, node, dw, "Hip");
	plates[0]->node->setPosition(0,2.23,0);

	plates[1] = new Plate(s, node, dw, "Hip");
	plates[1]->node->setPosition(0,-2.23,0);

	btCompoundShape *cs = new btCompoundShape();
        btBoxShape *box = new btBoxShape(btVector3(0.016,0.002,0.026));

        cs->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(-0.011,-0.0223,-0.006)), box);
        cs->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(-0.011,0.0223,-0.006)), box);

        box = new btBoxShape(btVector3(0.013,0.002,0.01));

        cs->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0.018,-0.0223,0.01)), box);
        cs->addChildShape(btTransform(btQuaternion(0,0,0,1),btVector3(0.018,0.0223,0.01)), box);

        shape = cs;



	if(right) {
		femur = new Servo(s, plates[1]->node, dw);
		torso = new Servo(s, plates[1]->node, dw);
		torso->FixFlushBelow(plates[1], 0, 1);
		femur->FixSlotFar(plates[1], 1, 1);
	} else {
		femur = new Servo(s, plates[0]->node, dw);
		torso = new Servo(s, plates[0]->node, dw);
		torso->FixFlushBelow(plates[0], 0, 0);
		femur->FixSlotFar(plates[0], 1, 0);
	}
	PlaceJoint(torso->sh->node);
	PlaceJoint(femur->sh->node);
	
}

