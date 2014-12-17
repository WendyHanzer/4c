# Linux
CC=g++
LIBS= -lglut -lGLEW -lGL -lassimp -I/usr/include/bullet -lBulletDynamics -lBulletSoftBody -lBulletCollision -lLinearMath `Magick++-config --cppflags --cxxflags --ldflags --libs`


# For Macs uncomment the next line and comment out the previous one
#CC=clang++
#LIBS=  -framework OpenGL -framework GLUT -framework Cocoa -lGLEW -stdlib=libc++
# Assuming you want to use a recent compiler

# Compiler flags
CXXFLAGS= -g -Wall -std=c++0x

all: ../bin/physics

../bin/physics : ../src/main.cpp
	$(CC) $(CXXFLAGS) ../src/main.cpp -o ../bin/physics $(LIBS) $(DEFS)

#assimp2: ../bin/physics
#DEFS=-DASSIMP_2
#../bin/physics : ../src/main.cpp
#	$(CC) $(CXXFLAGS) ../src/main.cpp -o ../bin/physics $(LIBS) $(DEFS)

clean:
	rm -rf *~ 
	rm *.o ../bin/physics

