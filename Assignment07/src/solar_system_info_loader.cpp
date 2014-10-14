/*
	This will be in main in one of it's own functions
*/
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

struct bodyData
{
	char name [50];
    float selfSpin;
    float revolution;
    float axisTilt;
    float radius;
    float revolutionRadius;
    float revolutionTilt;
};


void readIn(const char* fileName);

int main()
{
    const char* c = "../bin/solar_system.txt";
    readIn(c);
    return 0;
}

void readIn(const char* fileName)
{
	ifstream file;
	int fileLength = 0;
    char readObj [100];
    float readValue = 0.0;
    
    bodyData planets [5];
	
	
	file.open (fileName);
	
	if (file.is_open())
	{
		//get the stuff
        for(int i = 0; i < 5; i++)
        {
            file >> readObj;

            //if readObj is the sun it has different variables
            if(strcmp (readObj, "Sun") == 0)
			{
				//assign name to obj
                file >> readObj;
                file >> readValue;
                planets[i].selfSpin = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].axisTilt = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].radius = readValue;
            }

            else if(strcmp (readObj, "Moon") == 0)	//check each name of moon
			{
				//mark that this is a moon in obj bool
                file >> readObj;
                file >> readValue;
                planets[i].selfSpin = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].revolution = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].axisTilt = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].radius = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].revolutionRadius = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].revolutionTilt = readValue;
            }

            else 
			{
				//this is a planet
				//assign planet name to obj
                file >> readObj;
                file >> readValue;
                planets[i].selfSpin = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].revolution = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].axisTilt = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].radius = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].revolutionRadius = readValue;
                file >> readObj;
                file >> readValue;
                planets[i].revolutionTilt = readValue;
            }
      
        }
        file.close();
        
        for (int i = 0; i < 5; i++)
        {
        	cout << i<< ": \n" << planets[i].selfSpin << planets[i].revolution << planets[i].axisTilt << planets[i].radius << planets[i].revolutionRadius << planets[i].revolutionTilt << endl;
        }
	}
	
	else
	{
		cout << "Error opening " << fileName << endl;
	}
	
}
