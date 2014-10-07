# Linux
CC=g++
LIBS= -lglut -lGLEW -lGL -lassimp `Magick++-config --cppflags --cxxflags --ldflags --libs`


# For Macs uncomment the next line and comment out the previous one
#CC=clang++
#LIBS=  -framework OpenGL -framework GLUT -framework Cocoa -lGLEW -stdlib=libc++
# Assuming you want to use a recent compiler

# Compiler flags
CXXFLAGS= -g -Wall -std=c++0x

all: ../bin/Assign6

../bin/Assign6 : ../src/main.cpp
	$(CC) $(CXXFLAGS) ../src/main.cpp -o ../bin/Assign6 $(LIBS) $(DEFS)

clean:
	rm -rf *~ 
	rm *.o ../bin/Assign5

