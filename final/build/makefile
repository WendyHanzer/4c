# Linux
CC=g++
LIBS= -lglut -lGLEW -lGL -lassimp -I/usr/include/bullet -lBulletDynamics -lBulletSoftBody -lBulletCollision -lLinearMath -lsfml-audio -lsfml-system `Magick++-config --cppflags --cxxflags --ldflags --libs`


# For Macs uncomment the next line and comment out the previous one
#CC=clang++
#LIBS=  -framework OpenGL -framework GLUT -framework Cocoa -lGLEW -stdlib=libc++
# Assuming you want to use a recent compiler

# Compiler flags
CXXFLAGS= -g -Wall -std=c++0x

all: ../bin/angryPotato

../bin/angryPotato : ../src/main.cpp
	$(CC) $(CXXFLAGS) ../src/main.cpp -o ../bin/angryPotato $(LIBS) $(DEFS)

#assimp2: ../bin/angryPotato
#DEFS=-DASSIMP_2
#../bin/angryPotato : ../src/main.cpp
#	$(CC) $(CXXFLAGS) ../src/main.cpp -o ../bin/angryPotato $(LIBS) $(DEFS)

clean:
	rm -rf *~ 
	rm *.o ../bin/angryPotato

