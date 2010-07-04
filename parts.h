class Part {
	public:
		Part(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw);
		~Part();
	
		SceneNode *node;

		btDynamicsWorld *dynamicsWorld;
		btCollisionShape *shape;
		float thickness;
		btScalar mass;
		std::vector <SceneNode *>fitting_nodes;

                std::vector <Quaternion>jointq;
                std::vector <Vector3>jointv;

		btRigidBody *body;
		btHingeConstraint *hinge;
                void CreateBody(void);

                btHingeConstraint *Connect(int s1, Part *p2, int s2, bool reversed=false);
                Quaternion GetJointOrientation(int n);
                Vector3 GetJointPosition(int n);

	protected:
		static int number;
		void ReadJoints(std::string filename);
		void PlaceJoint(SceneNode *jn);
		void NodeAlign(SceneNode *a, SceneNode *b, SceneNode *c, SceneNode *d);

};

class Plate : public Part {
	public:
		Plate(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, const char* type);

};

class ServoHorn : public Part {
	public:
		ServoHorn(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw);
		void Fix(Plate *p, int joint, int orientation);
};

class Servo : public Part {
	public:
		Servo(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw);
		void FixFlushAbove(Plate *p, int joint, int orientation);
		void FixFlushBelow(Plate *p, int joint, int orientation);
		void FixSlotFar(Plate *p, int joint, int orientation);
		void FixSlotNear(Plate *p, int joint, int orientation);
		ServoHorn *sh;
};

class Torso : public Part {
	public:
		Torso(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw);

	protected:
		Plate *tp[2];
		ServoHorn *sh[6];

};

#define LEFT false
#define RIGHT true

class Femur : public Part {
	public:
		Femur(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, bool right=false);

	protected:
		Plate *plate;
		ServoHorn *hip, *tibia;

};

class Tibia : public Part {
	public:
		Tibia(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, bool right=false);

	protected:
		Plate *plate;
		Servo *femur;

};

class Hip : public Part {
	public:
		Hip(SceneManager *s, SceneNode *parent, btDynamicsWorld *dw, bool right=false);

	protected:
		Plate *plates[2];
		Servo *torso, *femur;

};
