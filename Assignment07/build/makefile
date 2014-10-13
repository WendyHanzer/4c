# Linux
CC=g++
LIBS= -lglut -lGLEW -lGL -lassimp `Magick++-config --cppflags --cxxflags --ldflags --libs`

# Compiler flags
CXXFLAGS= -g -Wall -std=c++0x `Magick++-config --cppflags --cxxflags --ldflags --libs`

all: ../bin/Assign7

../bin/Assign7 : modelClasses.o ../src/main.o
	$(CC) $(CXXFLAGS)  -o ../bin/Assign7 ../src/main.cpp modelClasses.o $(LIBS) $(DEFS)

modelClasses.o: ../src/modelClasses.cpp ../src/modelClasses.h
	$(CC) $(CXXFLAGS) -c ../src/modelClasses.cpp

clean:
	rm -rf *~ 
	rm *.o ../bin/Assign7

