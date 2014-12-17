#include <GL/glew.h> // glew must be included before the main gl libs
#include <GL/glut.h> // doing otherwise causes compiler shouting
#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <chrono>
#include <math.h>
#include <Magick++.h>

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
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp> //Makes passing matrices to shaders easier

#include <btBulletDynamicsCommon.h>
using namespace std;

//--Data types
//This object will define the attributes of a vertex(position, color, etc...)
struct Vertex //https://github.com/ccoulton/cs480coulton.git
{
    GLfloat position[3];
    GLfloat texuv[2];
};

class Texture{  //CLASS FROM OGLDEV_TEXTURE
	public:
		Texture(GLenum TextureTarget, const std::string& FileName);
		bool Load();
		void Bind(GLenum TextureUnit);
	private:
		std::string mfileName;
		GLenum mtextureTarget;
		GLuint mtextureObj;
		Magick::Image mimage;
		Magick::Blob mblob;
	};
	
Texture::Texture(GLenum TextureTarget, const std::string& FileName)
	{
	mtextureTarget = TextureTarget;
	mfileName = FileName;
	}

bool Texture::Load(){
	try{
		mimage.read(mfileName);
		mimage.write(&mblob, "RGBA");
		}
	catch (Magick::Error& Error){
		cout<<"Didn't load texture"<<mfileName<<Error.what()<<endl;
		return false;
		}
	glGenTextures(1, &mtextureObj);
	glBindTexture(mtextureTarget, mtextureObj);
	glTexImage2D(mtextureTarget, 0, GL_RGBA, mimage.columns(), mimage.rows(), 0, GL_RGBA, GL_UNSIGNED_BYTE, mblob.data());
	glTexParameterf(mtextureTarget, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(mtextureTarget, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(mtextureTarget, 0);
	return true;
	}

void Texture::Bind(GLenum TextureUnit){
	glActiveTexture(TextureUnit);
	glBindTexture(mtextureTarget, mtextureObj);
	} 
	
struct Object
{
	int numMesh;
    Vertex *Geo;
    char *name;
    unsigned int NumVert;
    Texture *Texs;
    GLuint vbo_Geo;
};
//--Evil Global variables
//Just for this example!
int w = 640, h = 480;// Window size
GLuint program;// The GLSL program handle
//GLuint vbo_geometry;// VBO handle for our geometry
Object *OBJ;
Object *OBJ2;
Object *OBJ3;
Object *OBJ4;
double isRotate = 0.0;
//uniform locations
GLint loc_mvpmat;// Location of the modelviewprojection matrix in the shader

//attribute locations
GLint loc_position;
GLint loc_tex;

//transform matrices

glm::mat4 model;//obj->world each object should have its own model matrix
glm::mat4 model2;
glm::mat4 model3;
glm::mat4 model4;
glm::mat4 view;//world->eye position of the camera
glm::mat4 projection;//eye->clip lens of the camera
glm::mat4 mvp;//premultiplied modelviewprojection

// motion flags
bool applyForward = false;
bool applyLeft = false;
bool applyBack = false;
bool applyRight = false;


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
Object *modelLoader(const char *objName);

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

//bullet globals
btDiscreteDynamicsWorld* dynamicsWorld;
btRigidBody* sphereRigidBody;
btRigidBody* cubeRigidBody;
btRigidBody* cylinderRigidBody;

const char *shaderloader(char *input);

//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("Assignment_08 - Physics!");
	
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
	
	//Create bullet stuff
/////////////
    //broadphase reduces the amount of collision detection calculations
        btBroadphaseInterface* broadphase = new btDbvtBroadphase();
		//initialize for actual collision detection
        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
		//takes into account gravity, collisions and other forces to do math
        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
		//initialize the dynamics world with all above info
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		//set the world gravity, this is (x, y, z), y is up
        dynamicsWorld->setGravity(btVector3(0, -10, 0));

		//create ground collision shape plane at y = 1 with an offset of 1 unit
        btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
        btCollisionShape* sideOne = new btStaticPlaneShape(btVector3(-1, 0, 0), 1);
        btCollisionShape* sideTwo = new btStaticPlaneShape(btVector3(1, 0, 0), 1);
        btCollisionShape* sideThree = new btStaticPlaneShape(btVector3(0, 0, 1), 1);
        btCollisionShape* sideFour = new btStaticPlaneShape(btVector3(0, 0, -1), 1);

		//create sphere collision shape body (radius in meters)
        btCollisionShape* sphere = new btSphereShape(1);
        //create sphere collision shape body (radius in meters)
        btCollisionShape* box = new btBoxShape(btVector3(0.5,0.5,0.5));
        //create sphere collision shape body (radius in meters)
        btCollisionShape* cylinder = new btCylinderShape(btVector3(1.0,1.0,1.0));
        
	//***create ground planes and rigid bodies for the board
		//create the ground with a Quaternion(0,0,0,1) the origin and position -1 to the origin
        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
        //pass rigid body info to create; for the ground parameters (mass-infinite, default motion state, shape, inertia-zero)
        // note:  if you want to create many objects with the same rigid body info, only need to do this once
        btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
        //create the rigid body for the base of the board
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
        //add the rigid body to the world
        dynamicsWorld->addRigidBody(groundRigidBody);
        
        //create the rigid body for side 1 of the board
        btDefaultMotionState* sideOneMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(5.3, 0, 0)));
        btRigidBody::btRigidBodyConstructionInfo
                sideOneRigidBodyCI(0, sideOneMotionState, sideOne, btVector3(0, 0, 0));
        btRigidBody* sideOneRigidBody = new btRigidBody(sideOneRigidBodyCI);
        
        dynamicsWorld->addRigidBody(sideOneRigidBody);
        //create the rigid body for side 2 of the board
        btDefaultMotionState* sideTwoMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(-5.3, 0, 0)));
        btRigidBody::btRigidBodyConstructionInfo
                sideTwoRigidBodyCI(0, sideTwoMotionState, sideTwo, btVector3(0, 0, 0));
        btRigidBody* sideTwoRigidBody = new btRigidBody(sideTwoRigidBodyCI);
        
        dynamicsWorld->addRigidBody(sideTwoRigidBody);
        //create the rigid body for side 3 of the board
        btDefaultMotionState* sideThreeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -5.4)));
        btRigidBody::btRigidBodyConstructionInfo
                sideThreeRigidBodyCI(0, sideThreeMotionState, sideThree, btVector3(0, 0, 0));
        btRigidBody* sideThreeRigidBody = new btRigidBody(sideThreeRigidBodyCI);
        
        dynamicsWorld->addRigidBody(sideThreeRigidBody);
        //create the rigid body for side 4 of the board
        btDefaultMotionState* sideFourMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 5.4)));
        btRigidBody::btRigidBodyConstructionInfo
                sideFourRigidBodyCI(0, sideFourMotionState, sideFour, btVector3(0, 0, 0));
        btRigidBody* sideFourRigidBody = new btRigidBody(sideFourRigidBodyCI);
        
        dynamicsWorld->addRigidBody(sideFourRigidBody);

	//***create the sphere rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,50)
        btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(2, 1, 0)));
        //set the mass of the sphere
        btScalar mass = 1;
        //calculate the falling inertia using built in function starting from no movement
        btVector3 fallInertia(0, 0, 0);
        sphere->calculateLocalInertia(mass, fallInertia);
        //pass rigid body info to create; for the sphere parameters (mass-1, default motion state, shape, inertia-calculated)
        btRigidBody::btRigidBodyConstructionInfo sphereRigidBodyCI(mass, fallMotionState, sphere, fallInertia);
        //create the rigid body
        sphereRigidBody = new btRigidBody(sphereRigidBodyCI);
        dynamicsWorld->addRigidBody(sphereRigidBody);
        
	//***create the cube rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,50)
        btDefaultMotionState* cubeMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1, 0)));
        //set the mass of the box
        mass = 0;
        //calculate the cube inertia using built in function starting from no movement
        btVector3 cubeInertia(0, 0, 0);
        box->calculateLocalInertia(mass, cubeInertia);
        //pass rigid body info to create; for the parameters (mass-1, default 				motion state, shape, inertia-calculated)
        btRigidBody::btRigidBodyConstructionInfo cubeRigidBodyCI(mass, cubeMotionState, sphere, fallInertia);
        //create the rigid body
        cubeRigidBody = new btRigidBody(cubeRigidBodyCI);
        cubeRigidBody->setCollisionFlags(cubeRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		cubeRigidBody->setActivationState(DISABLE_DEACTIVATION);
        dynamicsWorld->addRigidBody(cubeRigidBody);
        
	//***create the cylinder rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,z)
        btDefaultMotionState* cylinderMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(1, 20, 1)));
        //set the mass of the cylinder
        mass = 1;
        //calculate the cylinder inertia using built in function starting from no movement
        btVector3 cylinderInertia(0, 0, 0);
        cylinder->calculateLocalInertia(mass, cylinderInertia);
        //pass rigid body info to create; for the parameters (mass, default 				motion state, shape, inertia)
        btRigidBody::btRigidBodyConstructionInfo cylinderRigidBodyCI(mass, cylinderMotionState, cylinder, fallInertia);
        //create the rigid body
        cylinderRigidBody = new btRigidBody(cylinderRigidBodyCI);
        dynamicsWorld->addRigidBody(cylinderRigidBody);
/////////////
	
    // Initialize all of our resources(shaders, geometry)
    bool init = initialize(argc, argv);
    if(init)
    {
        t1 = std::chrono::high_resolution_clock::now();
        glutMainLoop();
    }

    // Clean up after ourselves 
    // Clean up Bullet
        dynamicsWorld->removeRigidBody(groundRigidBody);
        delete groundRigidBody->getMotionState();
        delete groundRigidBody;
        
        dynamicsWorld->removeRigidBody(sphereRigidBody);
        delete sphereRigidBody->getMotionState();
        delete sphereRigidBody;
        
        dynamicsWorld->removeRigidBody(cubeRigidBody);
        delete cubeRigidBody->getMotionState();
        delete cubeRigidBody;
        
        dynamicsWorld->removeRigidBody(cylinderRigidBody);
        delete cylinderRigidBody->getMotionState();
        delete cylinderRigidBody;
        
        dynamicsWorld->removeRigidBody(sideOneRigidBody);
        delete sideOneRigidBody->getMotionState();
        delete sideOneRigidBody;

        dynamicsWorld->removeRigidBody(sideTwoRigidBody);
        delete sideTwoRigidBody->getMotionState();
        delete sideTwoRigidBody;
      
        dynamicsWorld->removeRigidBody(sideThreeRigidBody);
        delete sideThreeRigidBody->getMotionState();
        delete sideThreeRigidBody;
        
        dynamicsWorld->removeRigidBody(sideFourRigidBody);
        delete sideFourRigidBody->getMotionState();
        delete sideFourRigidBody;

        delete sphere;
        delete box;
        delete cylinder;
        delete groundShape;
        delete sideOne;
        delete sideTwo;
        delete sideThree;
        delete sideFour;
        delete dynamicsWorld;
        delete solver;
        delete collisionConfiguration;
        delete dispatcher;
        delete broadphase;
        
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
    for(int meshindex =0; meshindex< OBJ[0].numMesh; meshindex++){
		glBindBuffer(GL_ARRAY_BUFFER, OBJ[meshindex].vbo_Geo);
		
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
		//draw sphere obj
		OBJ[meshindex].Texs->Bind(GL_TEXTURE0);
		glDrawArrays(GL_TRIANGLES, 0, OBJ[meshindex].NumVert);//mode, starting index, count
	}
	
    mvp = projection * view * model2;
    glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mvp));
	for(int meshindex =0; meshindex< OBJ2[0].numMesh; meshindex++){
		glBindBuffer(GL_ARRAY_BUFFER, OBJ2[meshindex].vbo_Geo);
		
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
		//draw box obj
		OBJ2[meshindex].Texs->Bind(GL_TEXTURE0);
		glDrawArrays(GL_TRIANGLES, 0, OBJ2[meshindex].NumVert);//mode, starting index, count
	}
	
    mvp = projection * view * model3;
    glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mvp));
	for(int meshindex =0; meshindex< OBJ3[0].numMesh; meshindex++){
		glBindBuffer(GL_ARRAY_BUFFER, OBJ3[meshindex].vbo_Geo);
		
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
		//draw cylinder obj
		OBJ3[meshindex].Texs->Bind(GL_TEXTURE0);
		glDrawArrays(GL_TRIANGLES, 0, OBJ3[meshindex].NumVert);//mode, starting index, count
	}
	
    mvp = projection * view * model4;
    glUniformMatrix4fv(loc_mvpmat, 1, GL_FALSE, glm::value_ptr(mvp));
	for(int meshindex =0; meshindex< OBJ4[0].numMesh; meshindex++){
		glBindBuffer(GL_ARRAY_BUFFER, OBJ4[meshindex].vbo_Geo);
		
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
		//draw board obj
		OBJ4[meshindex].Texs->Bind(GL_TEXTURE0);
		glDrawArrays(GL_TRIANGLES, 0, OBJ4[meshindex].NumVert);//mode, starting index, count
	}
    //clean up
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_tex);
                           
    //swap the buffers
    glutSwapBuffers();
}

void update()
{
    float dt = getDT();
    float force = 10.0;
    

    // check if force needs to be added
    if(applyForward)
    {
        sphereRigidBody->applyCentralImpulse(btVector3(0.0,0.0,force));
        applyForward = false;
    }
    if(applyBack)
    {
        sphereRigidBody->applyCentralImpulse(btVector3(0.0,0.0,-force));
        applyBack = false;
    }
    if(applyLeft)
    {
        sphereRigidBody->applyCentralImpulse(btVector3(force,0.0,0.0));
        applyLeft = false;
    }
    if(applyRight)
    {
        sphereRigidBody->applyCentralImpulse(btVector3(-force,0.0,0.0));
        applyRight = false;
    }

    
    dynamicsWorld->stepSimulation(dt, 1);

	//create a transform object, 
    //this holds all the position and orientation information in the dynamics world at this step
    btTransform trans;

    btScalar m[16];
    btScalar m2[16];
    btScalar m3[16];

    // get sphere model matrix
    sphereRigidBody->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(m);
    model = glm::make_mat4(m);

    // get cube model matrix
    cubeRigidBody->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(m2);
    model2 = glm::make_mat4(m2);

    // get cylinder model matrix
    cylinderRigidBody->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(m3);
    model3 = glm::make_mat4(m3);
    model3 = glm::scale( model3, glm::vec3(.7,.7,.7));

    // Update the state of the scene
    glutPostRedisplay();

    sphereRigidBody->clearForces();
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
    else if((key == 'w')||(key == 'W'))
    {
        applyForward = true;
    }
    else if((key == 'a')||(key == 'A'))
    {
        applyLeft = true;
    }
    else if((key == 's')||(key == 'S'))
    {
        applyBack = true;
    }
    else if((key == 'd')||(key == 'D'))
    {
        applyRight = true;
    }
/*
    else if ((key == 'm')||(key == 'M')) //menu
    	Rotation_menu(1);
    else if ((key == 'a')||(key == 'A'))
    	Rotation_menu(2);
    else if ((key == 'z')||(key == 'Z'))
    	Rotation_menu(3);
    else if ((key == 'f')||(key == 'F'))
    	sphereRigidBody->applyCentralForce(btVector3(0.5,0.0,0.0));	
*/

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
			isRotate = 45.0;
			break;
		case 2: //clw rotate
			isRotate =-45.0;
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
    const char* table = "table.obj";
	OBJ = modelLoader(argv[3]);
	OBJ2 = modelLoader(argv[4]);
	OBJ3 = modelLoader(argv[5]);
	OBJ4 = modelLoader(table);
	//

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
    view = glm::lookAt( glm::vec3(0.0, 8.0, -8.0), //Eye Position
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
    // Clean up,
    glDeleteProgram(program);
    glDeleteBuffers(1, &OBJ[0].vbo_Geo);
    glDeleteBuffers(1, &OBJ2[0].vbo_Geo);
    glDeleteBuffers(1, &OBJ3[0].vbo_Geo);
    glDeleteBuffers(1, &OBJ4[0].vbo_Geo);
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

Object *modelLoader(const char *objName)
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
		                       ,{pTexCoords->x, pTexCoords->y}}; //add texture
		   	/*cout<<output[meshindex].Geo[Index].position[0]<<" : "
		   		<<output[meshindex].Geo[Index].position[1]<<" : "
		   		<<output[meshindex].Geo[Index].position[2]<<endl;
		  
		*/}
    cout<<"attempting binding"<<endl;
	glGenBuffers(1, &output[meshindex].vbo_Geo);
    cout<<"buffers gen'd"<<endl;
    glBindBuffer(GL_ARRAY_BUFFER, output[meshindex].vbo_Geo);
    cout<<"bind buffer"<<endl;
    glBufferData(GL_ARRAY_BUFFER, output[meshindex].NumVert*24,
                     output[meshindex].Geo, GL_STATIC_DRAW);
    cout<<"bound"<<endl;
	}
    cout<<"loaded"<<endl;
	return(output);
	}
