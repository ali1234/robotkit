HEADERS=parts.h hexapod.h
OFILES=main.o parts.o hexapod.o BtOgre.o
CXXFLAGS=-g -Wall -I/usr/include/OGRE -I/usr/local/include/bullet -Ibtogre/include -L/usr/local/lib
EXENAME=hexapod

all: $(EXENAME)

BtOgre.o:
	$(CXX) -c btogre/BtOgre.cpp $(CXXFLAGS)

$(EXENAME): $(OFILES)
	$(CXX) -o $(EXENAME) $(OFILES) -lOgreMain -lOIS -lLinearMath -lBulletDynamics -lBulletCollision

$(OFILES): $(HEADERS)

clean:
	rm -f $(EXENAME) $(OFILES)

run: $(EXENAME)
	cd bin ; ../$(EXENAME) ; cd ..

rungdb: $(EXENAME)
	cd bin ; gdb ../$(EXENAME) ; cd ..
