#define GLM_FORCE_RADIANS

#include <GL/glew.h> // glew must be included before the main gl libs
#include <GL/glut.h> // doing otherwise causes compiler shouting
#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <chrono>
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
int w = 640, h = 480;// Window size
GLuint program;// The GLSL program handle
GLuint vbo_geometry;// VBO handle for our geometry
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
Object *modelLoader(char *objName);

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

const char *shaderloader(char *input);

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
	srand(getDT());
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
    glClearColor(0.0, 0.0, 0.2, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //premultiply the matrix for this example
    mvp = projection * view * model;
	
    //enable the shader program
    glUseProgram(program);
	
    //upload the matrix to the shader
    glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mvp));
	
    //set up the Vertex Buffer Object so it can be drawn
    glEnableVertexAttribArray(loc_position);
    glEnableVertexAttribArray(loc_tex);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_geometry);
    
    //set pointers into the vbo for each of the attributes(position and color)
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
	//draw first obj
	OBJ[0].Texs->Bind(GL_TEXTURE0);
    glDrawArrays(GL_TRIANGLES, 0, OBJ[0].NumVert);//mode, starting index, count
	
    //clean up
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_tex);
                           
    //swap the buffers
    glutSwapBuffers();
}

void update()
{
    //total time
    static float angle = 0.0;
    static float turn  = 0.0;
    float dt = getDT();// if you have anything moving, use dt.
    angle += dt * M_PI/2; //move through 90 degrees a second
    turn  += dt * isRotate;
    
    model = glm::translate(glm::mat4(1.0f), glm::vec3(4.0 * sin(angle), 0.0, 4.0 * cos(angle)));
    model = glm::rotate(model,turn , glm::vec3(0.0,1.0,0.0));
    // Update the state of the scene
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
        exit(0);
    else if ((key == 'm')||(key == 'M')) //menu
    	Rotation_menu(1);
    else if ((key == 'a')||(key == 'A'))
    	Rotation_menu(2);
    else if ((key == 'z')||(key == 'Z'))
    	Rotation_menu(3);
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
    // Initialize basic geometry and shaders for this example
	OBJ = modelLoader(argv[3]);
	//
    // Create a Vertex Buffer object to store this vertex info on the GPU*/
    glGenBuffers(1, &vbo_geometry);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_geometry);
    glBufferData(GL_ARRAY_BUFFER, OBJ[0].NumVert*24, OBJ[0].Geo, GL_STATIC_DRAW);
    //--Geometry done*/

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
    //Now we link the 2 shader objects into a program
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
    view = glm::lookAt( glm::vec3(0.0, 10.0, -16.0), //Eye Position
                        glm::vec3(0.0, 0.0, 0.0), //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Z is up

    projection = glm::perspective( 45.0f, //the FoV typically 90 degrees is good which is what this is set to
                                   float(w)/float(h), //Aspect Ratio, so Circles stay Circular
                                   0.01f, //Distance to the near plane, normally a small value like this
                                   100.0f); //Distance to the far plane, 

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
    glDeleteBuffers(1, &vbo_geometry);
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

Object *modelLoader(char *objName)
	{ 
	Object *output;
	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(objName, aiProcess_Triangulate);
	output = new Object[scene->mNumMeshes];
	output[0].numMesh = scene->mNumMeshes;
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
	for(unsigned int meshindex =0; meshindex < scene->mNumMeshes; meshindex++){
		output[meshindex].Geo = new Vertex[scene->mMeshes[meshindex]->mNumVertices];
	    output[meshindex].NumVert = scene->mMeshes[meshindex]->mNumVertices;
	    const aiMaterial* mat = scene->mMaterials[scene->mMeshes[meshindex]->mMaterialIndex];
	    if (mat->GetTextureCount(aiTextureType_DIFFUSE) > 0){
	    	aiString Path;
	    	if (mat->GetTexture(aiTextureType_DIFFUSE, 0, &Path, NULL, NULL, NULL, NULL, NULL) == AI_SUCCESS) {
	    		std::string fullpath = Path.data;
	    		output[meshindex].Texs = new Texture(GL_TEXTURE_2D, fullpath.c_str());
	    		if (output[meshindex].Texs->Load())
	    			cout<<"Success"<<endl;
	    		}
	    	}
	    for (unsigned int Index = 0; 
	    	Index < scene->mMeshes[meshindex]->mNumVertices; 
	    	Index++){
	    	const aiVector3D* pTexCoords = scene->mMeshes[meshindex]-> HasTextureCoords(0) ? &(scene->mMeshes[meshindex]->mTextureCoords[0][Index]): &Zero3D;
		    output[meshindex].Geo[Index]
		    				  ={{scene->mMeshes[meshindex]->mVertices[Index].x,
		                     	 scene->mMeshes[meshindex]->mVertices[Index].y,
		                         scene->mMeshes[meshindex]->mVertices[Index].z}
		                       ,{pTexCoords->x, -(pTexCoords->y)}}; //add texture
		   	/*cout<<output[meshindex].Geo[Index].position[0]<<" : "
		   		<<output[meshindex].Geo[Index].position[1]<<" : "
		   		<<output[meshindex].Geo[Index].position[2]<<endl;
		*/}
	}
	return(output);
	}


