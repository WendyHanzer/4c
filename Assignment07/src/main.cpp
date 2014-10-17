#define GLM_FORCE_RADIANS

#include <GL/glew.h> // glew must be included before the main gl libs
#include <GL/glut.h> // doing otherwise causes compiler shouting
#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <chrono>
#include <vector>
#include <math.h>
#include <Magick++.h>

#include "modelClasses.h"

#ifdef ASSIMP_2
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>

#else
#include <assimp/Importer.hpp>  //Asset Importer
#include <assimp/scene.h>		//Asset Importer scene graph aiScene object
#include <assimp/color4.h>
#include <assimp/postprocess.h>
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp> //Makes passing matrices to shaders easier

using namespace std;
	
//--Evil Global variables
//Just for this example!
int w = 1200, h = 600;// Window size
GLuint program;// The GLSL program handle
//GLuint vbo_geometry;// VBO handle for our geometry
Object *OBJ;
double isRotate = 0.0;
//uniform locations
GLint loc_mvpmat;// Location of the modelviewprojection matrix in the shader

//attribute locations
GLint loc_position;
GLint loc_tex;

//transform matrices

glm::mat4 model;//obj->world each object should have its own model matrix
glm::mat4 view;//world->eye position of the camera
glm::mat4 projection;//eye->clip lens of the camera
glm::mat4 mvp;//premultiplied modelviewprojection

//------------------------New Globals-----------------------------
vector<Object> indepPlanets;
vector<Object> depPlanets;

float timeScale = 0.1;
glm::vec3 cameraLookAt = glm::vec3(0.0, 0.0, 0.0);
glm::vec3 cameraPosition = glm::vec3(0.0, 60.0, -20.0);
int lookAtIndex = 0;
//----------------------------------------------------------------

//read in planet info
void readInPlanets(const char* fileName);

//--GLUT Callbacks
void render();
void update();
void reshape(int n_w, int n_h);
void keyboard(unsigned char key, int x_pos, int y_pos);
void mouse(int button, int state, int x, int y);
void Rotation_menu(int id);
void top_menu(int id);

//--Resource management
bool initialize(int argc, char **argv);
void cleanUp();
//Object *modelLoader(char *objName);

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

const char *shaderloader(char *input);

float lerp(float start, float end, float time);

//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("Assignment07- Solar System");
	
    // Now that the window is created the GL context is fully set up
    // Because of that we can now initialize GLEW to prepare work with shaders
    GLenum status = glewInit();
    if( status != GLEW_OK)
    {
        std::cerr << "[F] GLEW NOT INITIALIZED: ";
        std::cerr << glewGetErrorString(status) << std::endl;
        return -1;
    }

    // Set all of the callbacks to GLUT that we need
    glutDisplayFunc(render);// Called when its time to display
    glutReshapeFunc(reshape);// Called if the window is resized
    glutIdleFunc(update);// Called if there is nothing else to do
    glutKeyboardFunc(keyboard);// Called if there is keyboard input
	glutMouseFunc(mouse);//Called if there is mouse input
	//glutCreateMenu(Menu);
	int sub_menu = glutCreateMenu(Rotation_menu);
	glutAddMenuEntry("Start CCW Rotation", 1);
	glutAddMenuEntry("Start CLW Rotation", 2);
	glutAddMenuEntry("Stop Rotation", 3);
	glutCreateMenu(top_menu);
	glutAddSubMenu("Rotation options", sub_menu);
	glutAddMenuEntry("Quit", 2);
	//sub_menu = glut
	glutAttachMenu(GLUT_RIGHT_BUTTON);
	//srand(getDT());
    // Initialize all of our resources(shaders, geometry)
    bool init = initialize(argc, argv);
    if(init)
    {
        t1 = std::chrono::high_resolution_clock::now();
        glutMainLoop();
    }

    // Clean up after ourselves
    cleanUp();
    return 0;
}

//--Implementations
void render() 
{
    //--Render the scene

    //clear the screen
    glClearColor(0.0, 0.0, 1.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //enable the shader program
    glUseProgram(program);
    glEnableVertexAttribArray(loc_position);
    glEnableVertexAttribArray(loc_tex);

    glVertexAttribPointer( loc_position,//location of attribute
                           3,//number of elements
                           GL_FLOAT,//type
                           GL_FALSE,//normalized?
                           sizeof(Vertex),//stride
                           0);//offset

    glVertexAttribPointer( loc_tex,
                           2,
                           GL_FLOAT,
                           GL_FALSE,
                           sizeof(Vertex),
                           (void*)offsetof(Vertex,texuv));
    
    for (unsigned int i = 0; i< indepPlanets.size(); i++)
        {
        mvp = indepPlanets[i].render(projection*view);
        glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mvp));
        //itterate meshes
        for (int meshindex =0; meshindex< indepPlanets[i].numMesh; meshindex++)
            {
            indepPlanets[i].bind(meshindex);
            glDrawArrays(GL_TRIANGLES, 0, indepPlanets[i].mesh[meshindex].NumVert);            
            }
        }
            
     

    // redner each depPlanet
    for (unsigned int j = 0; j < depPlanets.size(); j++)
        {
         // generate MVP
         mvp = projection * view * depPlanets[j].modelMatrix;
         glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mvp));
         // bind geometry and texture
         depPlanets[j].bind(0);

         // draw 
         glDrawArrays(GL_TRIANGLES, 0, depPlanets[j].mesh[0].NumVert);//mode, starting index, count
         
        }

    //clean up
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_tex);
                      
    
    //swap the buffers
    glutSwapBuffers();
}

void update()
{
    // get DT
    float dt = getDT();
    dt *= timeScale;
    float lerpTime = 0.45;

    // update indep bodies
    for (unsigned int i = 0; i < indepPlanets.size(); i++)
        {
         indepPlanets[i].tick(dt);
        }

    // update dep bodies
    for (unsigned int j = 0; j < depPlanets.size(); j++)
        {
         depPlanets[j].tick(dt);
        }
    
    glm::vec3 planetPos = indepPlanets[lookAtIndex].getPosition();

    cameraLookAt.x = lerp(cameraLookAt.x, planetPos.x, lerpTime);
    cameraLookAt.y = lerp(cameraLookAt.y, planetPos.y, lerpTime);
    cameraLookAt.z = lerp(cameraLookAt.z, planetPos.z, lerpTime);

    planetPos.x += 0;
    planetPos.y += 60;
    planetPos.z += 60;

    cameraPosition.x = lerp(cameraPosition.x, planetPos.x, lerpTime);
    cameraPosition.y = lerp(cameraPosition.y, planetPos.y, lerpTime);
    cameraPosition.z = lerp(cameraPosition.z, planetPos.z, lerpTime);

    view = glm::lookAt( cameraPosition, //Eye Position
                        cameraLookAt, //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Z is up

    glutPostRedisplay();//call the display callback
}


void reshape(int n_w, int n_h)
{
    w = n_w;
    h = n_h;
    //Change the viewport to be correct
    glViewport( 0, 0, w, h);
    //Update the projection matrix as well
    //See the init function for an explaination
    projection = glm::perspective(45.0f, float(w)/float(h), 0.01f, 100.0f);

}

void keyboard(unsigned char key, int x_pos, int y_pos)
{
    // Handle keyboard input
    if((key == 27)||(key == 'q')||(key == 'Q'))//ESC
        {
         exit(0);
        }
    else if (key == 'r' || key == 'R')
        {
         if (timeScale < 3.0)
            {
             timeScale += 0.1;
            }
        }
    else if (key == 'e' || key == 'E')
        {
         if (timeScale > -3.0)
            {
             timeScale -= 0.1;
            }
        }
    else if (key >= '0' && key <= '9')
        {
         int num = key - '0';
         if(indepPlanets.size() >= (unsigned int)num)
            {
             lookAtIndex = num;
            }
         else
            {
             lookAtIndex = 0;
            }
        }
    else if (key == ' ')
        {
         timeScale = 0.0;
        }
}

void mouse(int button, int state, int x, int y){
	//Mouse handler
	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);//what does right button do?
	}

void Rotation_menu(int id){
	switch(id)
		{
		case 1: //ccw rotate
			isRotate = M_PI/4;
			break;
		case 2: //clw rotate
			isRotate =-M_PI/4;
			break;
		case 3: //stop rotate
			isRotate = 0.0;
			break;
		}
	glutPostRedisplay();
	}
	
void top_menu(int id){
	switch (id)
		{
		case 1:
			Rotation_menu(id);
			break;
		case 2:
			exit(0);
			break;
		}
	glutPostRedisplay();
	}
	
bool initialize(int argc, char **argv)
{
    // get object data
    readInPlanets(argv[3]);

    char* pathName = new char[256];

    // load model data for each object
    for(unsigned int i = 0; i < indepPlanets.size(); i++)
        {
         sprintf(pathName, "texture/%s.obj", indepPlanets[i].name);
         indepPlanets[i].load(pathName);
        }
    for(unsigned int j = 0; j < depPlanets.size(); j++)
        {
         sprintf(pathName, "texture/%s.obj", depPlanets[j].name);
         depPlanets[j].load(pathName);
        }
        
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    //Shader Sources
    //TODO: make shader into a class so you can init and make stored shaders
    // Note the added uniform!
    const char *vs = shaderloader(argv[1]);
    const char *fs = shaderloader(argv[2]);
    //compile the shaders
    GLint shader_status;

    // Vertex shader first
    glShaderSource(vertex_shader, 1, &vs, NULL);
    glCompileShader(vertex_shader);
    //check the compile status
    char buffer[512];
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &shader_status);
    if(!shader_status)
    {
        glGetShaderInfoLog(vertex_shader, 512, NULL, buffer);
        std::cerr<<buffer<<std::endl;
        std::cerr << "[F] FAILED TO COMPILE VERTEX SHADER!" << std::endl;
        return false;
    }

    // Now the Fragment shader
    glShaderSource(fragment_shader, 1, &fs, NULL);
    glCompileShader(fragment_shader);
    //check the compile status
    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &shader_status);
    if(!shader_status)
    {
    	glGetShaderInfoLog(fragment_shader, 512, NULL, buffer);
    	std::cerr << buffer << std::endl;
        std::cerr << "[F] FAILED TO COMPILE FRAGMENT SHADER!" << std::endl;
        return false;
    } //sed to store and transform them. 
    
    //Now we link the to shader objects into a program
    //This program is what is run on the GPU
    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    //check if everything linked ok
    glGetProgramiv(program, GL_LINK_STATUS, &shader_status);
    if(!shader_status)
    {
        std::cerr << "[F] THE SHADER PROGRAM FAILED TO LINK" << std::endl;
        return false;
    }

    //Now we set the locations of the attributes and uniforms
    //this allows us to access them easily while rendering
    loc_position = glGetAttribLocation(program, /*sed to store and transform them.*/  const_cast<const char*>("v_position"));
    if(loc_position == -1)
    {	
    	std::cerr << glGetError() << std::endl;
        std::cerr << "[F] POSITION NOT FOUND" << std::endl;
        return false;
    }

    loc_tex = glGetAttribLocation(program,
                    const_cast<const char*>("v_tex"));
    if(loc_tex == -1)
    {
        std::cerr << "[F] V_tex NOT FOUND" << std::endl;
        return false;
    }

    loc_mvpmat = glGetUniformLocation(program,
                    const_cast<const char*>("mvpMatrix"));
    if(loc_mvpmat == -1)
    {
        std::cerr << "[F] MVPMATRIX NOT FOUND" << std::endl;
        return false;
    }
    //--Init the view and projection matrices
    //  if you will be having a moving camera the view matrix will need to more dynamic
    //  ...Like you should update it before you render more dynamic 
    //  for this project having them static will be fine
    view = glm::lookAt( glm::vec3(0.0, 60.0, -60.0), //Eye Position
                        glm::vec3(0.0, 0.0, 0.0), //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Z is up

    projection = glm::perspective( 45.0f, //the FoV typically 90 degrees is good which is what this is set to
                                   float(w)/float(h), //Aspect Ratio, so Circles stay Circular
                                   0.00001f, //Distance to the near plane, normally a small value like this
                                   1000000.0f); //Distance to the far plane, 

    //enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    //and its done
    return true;
}

void cleanUp()
{
    // Clean up, Clean up
    glDeleteProgram(program);
    //glDeleteBuffers(1, &vbo_geometry);
    //glDeleteBuffers(1, &vbo_geometry2);
}

//returns the time delta
float getDT()
{
    float ret;
    t2 = std::chrono::high_resolution_clock::now();
    ret = std::chrono::duration_cast< std::chrono::duration<float> >(t2-t1).count();
    t1 = std::chrono::high_resolution_clock::now();
    return ret;
}
//Loads shader definations in from files
const char *shaderloader(char *input){
	
	FILE* infile = fopen(input, "rb");   //Open file
  	if(fseek(infile, 0, SEEK_END) == -1) return NULL;
  	long size = ftell(infile);			//get file size
  	if(fseek(infile, 0, SEEK_SET) == -1) return NULL;
  	char *shader = (char*) malloc( (size_t) size +1  ); 
 
  	size = fread(shader, 1, (size_t)size, infile); //read from file into shader
  	if(ferror(infile)) {
    	free(shader);
    	return NULL;
  		}
 
  	fclose(infile);
  	shader[size] = '\0';
  	return shader;
	}

void readInPlanets(const char* fileName)
{
	ifstream file;
    char readObj [100];
    float readValue = 0.0;
    int readInt = 0;
    int currentPlanet = 0;
    int numBodies;
    
    //data in object is private, make functions or put in public??
    Object* planet;
	
	
	file.open (fileName);
	
	if (file.is_open())
	{
     // get number of bodies to read in
     file >> readObj;
     file >> numBodies;

		//get the stuff
        for(int i = 0; i < numBodies; i++)
        {
            planet = new Object;

        	//this is the planet/moon name
            file >> readObj;

            //if readObj is the sun it has different variables
            if(strcmp (readObj, "sun") == 0)
			{
                strcpy(planet->name, readObj);
                file >> readObj;
                file >> readValue;
                planet->planetData.selfSpin = readValue;
                file >> readObj;
                file >> readValue;
                planet->planetData.axisTilt = readValue;
                file >> readObj;
                file >> readValue;
                planet->planetData.radius = readValue;
                planet->planetData.revolution = 0;
                planet->planetData.revolutionTilt = 0;
                planet->planetData.revolutionRadius = 0;
                planet->planetData.isMoon = 0;
                planet->planetData.parent = NULL;
                indepPlanets.push_back(*planet);
                currentPlanet++;
                cout<<"Added "<<planet->name<<" to indep list"<<endl;
            }

            else 
			{
                strcpy(planet->name, readObj);
				//this is a planet or moon
				//assign planet name to obj
                file >> readObj;
                file >> readInt;

                planet->planetData.isMoon = readInt;
                //this is a planet
                if (readInt == 0)
                {                	
		            file >> readObj;
		            file >> readValue;
                	planet->planetData.selfSpin = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.revolution = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.axisTilt = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.radius = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.revolutionRadius = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.revolutionTilt = readValue;
                    planet->planetData.parent = NULL;
                    
                	indepPlanets.push_back(*planet);
                	cout<<"Added "<<planet->name<<" to indep list"<<endl;
                	currentPlanet++;

                }
                else
                {	
                	file >> readObj;
		            file >> readValue;
		            planet->planetData.selfSpin = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.revolution = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.axisTilt = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.radius = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.revolutionRadius = readValue;
		            file >> readObj;
		            file >> readValue;
		            planet->planetData.revolutionTilt = readValue;
                    planet->planetData.parent = &(indepPlanets[currentPlanet-1]);
                    cout<<"Added "<<planet->name<<" to dep list with parent "<<(planet->planetData.parent->name)<<endl;
		            depPlanets.push_back(*planet);

                }
            }
      
        }
	}
	
	else
	{
		cout << "Error opening " << fileName << endl;
	}
	
    file.close();
}

float lerp(float start, float end, float time)
    {
     return ((1-time)*start) + (time*end);
    }


