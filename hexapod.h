class Hexapod {
	public:
		Hexapod(SceneManager *s, btDynamicsWorld *dw);
		~Hexapod();
		void think(btScalar timestep);
		SceneNode *GetSceneNode(void);

	protected:
		Part *t, *lh[3], *rh[3], *lf[3], *rf[3], *lt[3], *rt[3];
		void ConnectParts(Part *p1, int c1, Part *p2, int c2);
};


