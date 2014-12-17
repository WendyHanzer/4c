#include <GL/glew.h> // glew must be included before the main gl libs
#include <GL/freeglut.h> // doing otherwise causes compiler shouting
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


//This object will define the attributes of a vertex(position, color, etc...)
struct Vertex //https://github.com/ccoulton/cs480coulton.git
{
    GLfloat position[3];
    GLfloat texuv[2];
    GLfloat normal[3];
};

// texture class: container for texture info
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

// load texture from file
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

// object class: container for openGL object info
struct Object
{
	int numMesh;
    Vertex *Geo;
    char *name;
    unsigned int NumVert;
    Texture *Texs;
    GLuint vbo_Geo;
    void Render(glm::mat4 model, glm::mat4 view, glm::mat4 projection, GLuint shade)
        {
        glm::mat4 mvp = projection * view * model;
        glm::mat4 mv = view * model;
        //upload the matrix to the shader
        GLint mvp_loc = glGetUniformLocation(shade,
                                            const_cast<const char*>("mvpMatrix"));
        GLint vp_location  = glGetUniformLocation(shade,
                                            const_cast<const char*>("mvMatrix"));
        GLint loc_position = glGetAttribLocation(shade,
                                            const_cast<const char*>("v_position"));
        GLint loc_tex      = glGetAttribLocation(shade,
                                            const_cast<const char*>("v_tex"));
        GLint loc_normal =   glGetAttribLocation(shade,
                                            const_cast<const char*>("v_normal"));
        glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, glm::value_ptr(mvp));
        glUniformMatrix4fv(vp_location, 1, GL_FALSE, glm::value_ptr(mv));
	    glBindBuffer(GL_ARRAY_BUFFER, vbo_Geo);
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

        glVertexAttribPointer( loc_normal,
                               3,
                               GL_FLOAT,
                               GL_FALSE,
                               sizeof(Vertex),
                               (void*)offsetof(Vertex,normal));
	    //draw sphere obj
	    Texs->Bind(GL_TEXTURE0);
	    glDrawArrays(GL_TRIANGLES, 0, NumVert);//mode, starting index, count
        
        };
};
//--Evil Global variables
//Just for this example!
int w = 1100, h = 700;// Window size
GLuint program;// The GLSL program handle
//GLuint vbo_geometry;// VBO handle for our geometry

Object *OBJ;
Object *OBJ2;
Object *OBJ3;
//Object *OBJ4;
Object *rink;
double isRotate = 0.0;
//uniform locations
GLint loc_mvpmat;// Location of the modelviewprojection matrix in the shader
GLint loc_Ambient;
GLint loc_mvmat;


//attribute locations
GLint loc_position;
GLint loc_tex;
GLint loc_normal;

//transform matrices

glm::mat4 model;//obj->world each object should have its own model matrix
glm::mat4 model2;// puck
glm::mat4 model3;// player1
glm::mat4 model4;// player2
glm::mat4 view;//world->eye position of the camera
glm::mat4 projection;//eye->clip lens of the camera
glm::mat4 mvp;//premultiplied modelviewprojection

// motion flags
bool applyForward = false;
bool applyLeft = false;
bool applyBack = false;
bool applyRight = false;

bool  mouseApply = false;

// light indicators
bool ambientOn = true;
bool distantOn = true;
bool pointOn = true;
bool spotOn = true;

// player scores
int p1Score = 0;
int p2Score = 0;

// mouse position
int mX,mY;
bool paused = false;

// camera vectors
glm::vec3* camPos = new glm::vec3(0.0, 14.0, -20.0);
glm::vec3* lookPos = new glm::vec3(0.0, 0.0, 0.0);

glm::vec3 pPos;

//--GLUT Callbacks
void render();
void update();
void reshape(int n_w, int n_h);
void keyboard(unsigned char key, int x_pos, int y_pos);
void mouse(int button, int state, int x, int y);
void mouseMove(int x, int y);
void top_menu(int id);

//--Resource management
bool initialize(int argc, char **argv);
void cleanUp();
Object *modelLoader(const char *objName);
void resetBoard(bool goingUp);
void getTriMesh(Object *obj, btTriangleMesh *tetraMesh);

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

//--bullet globals
btDiscreteDynamicsWorld* dynamicsWorld;
btRigidBody* sphereRigidBody;
btRigidBody* cubeRigidBody;
btRigidBody* cylinderRigidBody;

//bullet collision masks
#define BIT(x) (1<<(x))
enum collisiontypes {
    COL_PADDLE = BIT(0), //<Collide with paddles
    COL_WALL = BIT(1), //<Collide with rink
    COL_CENTER = BIT(2), //<Collide with center plane
    COL_PUCK = BIT(3), //<Collide with puck
};

int puckCollidesWith = COL_WALL | COL_PADDLE | COL_PUCK;
int paddleCollidesWith = COL_WALL | COL_PADDLE | COL_PUCK | COL_CENTER;
int rinkCollidesWith = COL_PADDLE | COL_PUCK;
int centerCollidesWith = COL_PADDLE;

//--Shaderloader
const char *shaderloader(char *input);



//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("Assignment_09 - Air Hockey!!!");
	
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
    glutPassiveMotionFunc(mouseMove);
	glutCreateMenu(top_menu);
	glutAddMenuEntry("Pause", 1);
	glutAddMenuEntry("Resume", 2);
    glutAddMenuEntry("Reset", 3);
	glutAddMenuEntry("Quit", 4);
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
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

		//create ground collision shape plane at y = 1 with an offset of 1 unit
        //btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
        btCollisionShape* sideOne = new btStaticPlaneShape(btVector3(-1, 0, 0), 1);
        btCollisionShape* sideTwo = new btStaticPlaneShape(btVector3(1, 0, 0), 1);
        btCollisionShape* sideThree = new btStaticPlaneShape(btVector3(0, 0, 1), 1);
        btCollisionShape* sideFour = new btStaticPlaneShape(btVector3(0, 0, -1), 1);


///////////////////////////////////////
        // Initialize basic geometry
        rink = modelLoader("rink.obj");
	    OBJ = modelLoader("puck.obj");
	    OBJ2 = modelLoader("paddle.obj");
	    OBJ3 = modelLoader("paddle.obj");

        // init triangle meshes
        btTriangleMesh* tetraMesh = new btTriangleMesh();

        // get triangle meshe
        getTriMesh(rink, tetraMesh);       

        // create bullet colission shapes   
        btBvhTriangleMeshShape* rinkShape = new btBvhTriangleMeshShape(tetraMesh, false);
        btCollisionShape* centerShape = new btBoxShape(btVector3(10,10,0.01));
        btCollisionShape* puckShape = new btCylinderShape(btVector3(.5,.5,.5));
        btCollisionShape* P1Shape = new btCylinderShape(btVector3(1.2,1.2,1.2));
        btCollisionShape* P2Shape = new btCylinderShape(btVector3(1.2,1.2,1.2));
        
		//create the ground with a Quaternion(0,0,0,1) the origin and position -1 to the origin
        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
        //pass rigid body info to create; for the ground parameters (mass-infinite, default motion state, shape, inertia-zero)
        // note:  if you want to create many objects with the same rigid body info, only need to do this once
        btRigidBody::btRigidBodyConstructionInfo
                rinkRigidBodyCI(0, groundMotionState, rinkShape, btVector3(0, 0, 0));
        //create the rigid body for the base of the board
        btRigidBody* rinkRigidBody = new btRigidBody(rinkRigidBodyCI);
        //add the rigid body to the world
        dynamicsWorld->addRigidBody(rinkRigidBody, COL_WALL, rinkCollidesWith);

     
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
        btDefaultMotionState* sideThreeMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -10.4)));
        btRigidBody::btRigidBodyConstructionInfo
                sideThreeRigidBodyCI(0, sideThreeMotionState, sideThree, btVector3(0, 0, 0));
        btRigidBody* sideThreeRigidBody = new btRigidBody(sideThreeRigidBodyCI);
        
        dynamicsWorld->addRigidBody(sideThreeRigidBody);
        //create the rigid body for side 4 of the board
        btDefaultMotionState* sideFourMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 8.7)));
        btRigidBody::btRigidBodyConstructionInfo
                sideFourRigidBodyCI(0, sideFourMotionState, sideFour, btVector3(0, 0, 0));
        btRigidBody* sideFourRigidBody = new btRigidBody(sideFourRigidBodyCI);
        
        dynamicsWorld->addRigidBody(sideFourRigidBody);

	//***create the sphere rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,50)
        btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1, -1)));
        //set the mass of the sphere
        btScalar mass = .5;
        //calculate the falling inertia using built in function starting from no movement
        btVector3 fallInertia(0, 0, 0);
        puckShape->calculateLocalInertia(mass, fallInertia);
        //pass rigid body info to create; for the sphere parameters (mass-1, default motion state, shape, inertia-calculated)
        btRigidBody::btRigidBodyConstructionInfo sphereRigidBodyCI(mass, fallMotionState, puckShape, fallInertia);
        //create the rigid body
        sphereRigidBody = new btRigidBody(sphereRigidBodyCI);
        sphereRigidBody->forceActivationState(DISABLE_DEACTIVATION);
        sphereRigidBody->setFriction(0.01);
        sphereRigidBody->setRestitution(2.0);
        sphereRigidBody->setAngularFactor(btVector3(0,1,0));
        dynamicsWorld->addRigidBody(sphereRigidBody, COL_PUCK, puckCollidesWith);

	//***create the P1 rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,50)
        btDefaultMotionState* cubeMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 2, -6)));
        //set the mass of the box
        mass = 2;
        //calculate the cube inertia using built in function starting from no movement
        btVector3 cubeInertia(0, 0, 0);
        P1Shape->calculateLocalInertia(mass, cubeInertia);
        //pass rigid body info to create; for the parameters (mass-1, default 				motion state, shape, inertia-calculated)
        btRigidBody::btRigidBodyConstructionInfo cubeRigidBodyCI(mass, cubeMotionState, P1Shape, fallInertia);
        //create the rigid body
        cubeRigidBody = new btRigidBody(cubeRigidBodyCI);
        cubeRigidBody->forceActivationState(DISABLE_DEACTIVATION);
        cubeRigidBody->setFriction(0.01);
        cubeRigidBody->setRestitution(.5);
        cubeRigidBody->setAngularFactor(btVector3(0,1,0));
        dynamicsWorld->addRigidBody(cubeRigidBody, COL_PADDLE, paddleCollidesWith);

        
	//***create the P2 rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,z)
        btDefaultMotionState* cylinderMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 2, 3)));
        //set the mass of the cylinder
        mass = 2;
        //calculate the cylinder inertia using built in function starting from no movement
        btVector3 cylinderInertia(0, 0, 0);
        P2Shape->calculateLocalInertia(mass, cylinderInertia);
        //pass rigid body info to create; for the parameters (mass, default 				motion state, shape, inertia)
        btRigidBody::btRigidBodyConstructionInfo cylinderRigidBodyCI(mass, cylinderMotionState, P2Shape, fallInertia);
        //create the rigid body
        cylinderRigidBody = new btRigidBody(cylinderRigidBodyCI);
        cylinderRigidBody->forceActivationState(DISABLE_DEACTIVATION);
        cylinderRigidBody->setFriction(0.01);
        cylinderRigidBody->setRestitution(.5);
        cylinderRigidBody->setAngularFactor(btVector3(0,1,0));
        dynamicsWorld->addRigidBody(cylinderRigidBody, COL_PADDLE, paddleCollidesWith);
        
        //create center plane
		btDefaultMotionState* centerMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -.7)));
        btRigidBody::btRigidBodyConstructionInfo
                centerRigidBodyCI(0, centerMotionState, centerShape, btVector3(0, 0, 0));
        btRigidBody* centerRigidBody = new btRigidBody(centerRigidBodyCI);
        dynamicsWorld->addRigidBody(centerRigidBody, COL_CENTER, centerCollidesWith);
        
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
        dynamicsWorld->removeRigidBody(sphereRigidBody);
        delete sphereRigidBody->getMotionState();
        delete sphereRigidBody;
        
        dynamicsWorld->removeRigidBody(cubeRigidBody);
        delete cubeRigidBody->getMotionState();
        delete cubeRigidBody;
        
        dynamicsWorld->removeRigidBody(cylinderRigidBody);
        delete cylinderRigidBody->getMotionState();
        delete cylinderRigidBody;

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
   
    glUseProgram(program);
    //set up the Vertex Buffer Object so it can be drawn
    glEnableVertexAttribArray(loc_position);
    glEnableVertexAttribArray(loc_tex);
    glEnableVertexAttribArray(loc_normal);
    
    //render each ob
    //methodize me
    for(int meshindex =0; meshindex< OBJ[0].numMesh; meshindex++)
	    OBJ[meshindex].Render(model, view, projection, program);
	//End of Method!
	for(int meshindex =0; meshindex< OBJ2[0].numMesh; meshindex++)
		OBJ2[meshindex].Render(model2, view, projection, program);
	
	for(int meshindex =0; meshindex< OBJ3[0].numMesh; meshindex++)
		OBJ3[meshindex].Render(model3, view, projection, program);
	
	for(int meshindex =0; meshindex< rink[0].numMesh; meshindex++)
		rink[meshindex].Render(model4, view, projection, program);
    //clean up
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_tex);
    glDisableVertexAttribArray(loc_normal);

    // print score (2-D text)
    char p1ScoreTex[30];
    char p2ScoreTex[30];
    p1ScoreTex[0] = '\0';
    p2ScoreTex[0] = '\0';
    sprintf(p1ScoreTex, "P1 Score: %d", p1Score);
    sprintf(p2ScoreTex, "P2 Score: %d", p2Score);
    glColor3f(0.5f, 0, 0);
    glRasterPos2f(.6, .8);
    glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (unsigned char*)p1ScoreTex);
    //glColor3f(0.5f, 0, 0);
    glRasterPos2f(.6, .72);
    glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (unsigned char*)p2ScoreTex);

/* used from debugging
    glRasterPos2f(.6, .64);
    char xyz[100];
    sprintf(xyz, "%.2f %.2f %.2f", pPos.x, pPos.y, pPos.z);
    glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (unsigned char*)xyz);
*/                     
    //swap the buffers
    glutSwapBuffers();
}

void update()
{
    float dt = getDT();
    float force = 100.0;
    static int scoreCtr = 0;

    view = glm::lookAt( *camPos, //Eye Position
                        *lookPos, //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Z is up

    // do not update if paused
    if(paused)
        {
            return;
        }

float fX,fZ;

    // give paddle left force
    if(mouseApply)
    {
        if(mX < (w/3))
            {
                fX = force;
            }
        // give paddle right force
        else if (mX > (2*w/3))
            {
                fX = -force;
            }
        // apply no left/right force
        else
            {
                fX = 0;
            }

        // give paddle upward force
        if(mY < (h/3))
            {
                fZ = force;
            }
        // give paddle downward force
        else if (mY > (2*h/3))
            {
                fZ = -force;
            }
        // apply no up/down force
        else
            {
                fZ = 0;
            }
        // apply mouse force
        cylinderRigidBody->applyCentralImpulse(btVector3(fX,0.0,fZ));
        mouseApply = false;
    }

    // check if force needs to be added
    if(applyForward)
    {
        cubeRigidBody->applyCentralImpulse(btVector3(0.0,0.0,force));
        applyForward = false;
    }
    if(applyBack)
    {
        cubeRigidBody->applyCentralImpulse(btVector3(0.0,0.0,-force));
        applyBack = false;
    }
    if(applyLeft)
    {
        cubeRigidBody->applyCentralImpulse(btVector3(force,0.0,0.0));
        applyLeft = false;
    }
    if(applyRight)
    {
        cubeRigidBody->applyCentralImpulse(btVector3(-force,0.0,0.0));
        applyRight = false;
    }

    // cap velocities
    btVector3 velocity = cylinderRigidBody->getLinearVelocity();
    btScalar speed = velocity.length();
    if(speed > 8.0) {
        velocity *= 8.0/speed;
        cylinderRigidBody->setLinearVelocity(velocity);
    }

    btVector3 velocityC = cubeRigidBody->getLinearVelocity();
    btScalar speedC = velocityC.length();
    if(speedC > 8.0) {
        velocityC *= 8.0/speedC;
        cubeRigidBody->setLinearVelocity(velocityC);
    }

    // tick simulation
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

    // get z position of puck
    pPos = glm::vec3(model[3][0], model[3][1],model[3][2]);

    bool p1Collision = pPos.z < -8.5 && scoreCtr == 0;
    bool p2Collision = pPos.z > 7.3 && scoreCtr == 0;

    // add score to player1
    if(p1Collision)
        {
            resetBoard(true);
            p1Score++;
            scoreCtr = 8;
            applyForward = applyBack = applyLeft = applyRight = false;
        }
    // add score to player2
    if(p2Collision)
        {
            resetBoard(false);
            p2Score++;
            scoreCtr = 8;
            applyForward = applyBack = applyLeft = applyRight = false;
        }

    // buffer (to prevent multiple scores
    if(scoreCtr > 0)
        {
            scoreCtr--;
        }

    // Update the state of the scene
    glutPostRedisplay();

    // clear all previous forces
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
	
	GLfloat ambient_off[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat diffuse_off[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat specular_off[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat position_off[] = {0.0f, 0.0f, 0.0f, 1.0f};
	
	GLfloat ambient_d[] = {0.5f, 0.5f, 0.5f, 1.0f};
	GLfloat diffuse_d[] = {0.0f, 0.0f, 1.0f, 0.2f};
	GLfloat specular_d[] = {.5f, 1.0f, 1.0f, 0.2f};
	
	GLfloat ambient_p[] = {0.5f, 0.5f, 0.5f, 0.2f};
	GLfloat diffuse_p[] = {0.0f, 1.0f, 0.0f, 0.2f};
	GLfloat specular_p[] = {.5f, 1.0f, 1.0f, 0.2f};
	GLfloat position_p[] = {2.0f, 1.0f, 0.0f, 1.0f};
	
	GLfloat ambient_s[] = {0.5f, 0.5f, 0.5f, 0.2f};
	GLfloat diffuse_s[] = {1.0f, 0.0f, 0.0f, 0.2f};
	GLfloat specular_s[] = {1.0f, 0.0f, 0.0f, 0.2f};
	GLfloat position_s[] = {0.0f, 5.0f, 0.0f, 1.0f};
	
    static int camStepX = 0;
    static int camStepY = 0;

    // Handle keyboard input
    if((key == 27)||(key == 'q')||(key == 'Q'))//ESC
    {
        exit(0);
    }
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
    else if((key == 'z')||(key == 'Z'))
    {
        //p1Score = p2Score = 0;
        resetBoard(false);
    }
    // toggle pause
    else if(key == ' ')
    {
        if(paused)
            {
                paused = false;
            }
        else
            {
             paused = true;
            }
    }
    // i/j/k/l : camera movement
    else if((key == 'i')||(key == 'I'))
    {
        if(camStepY < 10)
            {
                camStepY++;
                camPos->z = -20 + (camStepY);
            }
    }
    else if((key == 'j')||(key == 'J'))
    {
        if(camStepX < 10)
            {
                camStepX++;
                camPos->x = (camStepX);
            }
    }
    else if((key == 'k')||(key == 'K'))
    {
        if(camStepY > -10)
            {
                camStepY--;
                camPos->z = -20 + (camStepY);
            }
    }
    else if((key == 'l')||(key == 'L'))
    {
        if(camStepX > -10)
            {
                camStepX--;
                camPos->x = (camStepX);
            }
    }
    // 1,2 : score control
    else if(key == '1')
    {
        p1Score++;
    }
    else if(key == '2')
    {
        p2Score++;
    }
    else if(key == '!')
    {
        p1Score--;
        if(p1Score < 0)
            {
                p1Score = 0;
            }
    }
    else if(key == '@')
    {
        p2Score--;
        if(p2Score < 0)
            {
                p2Score = 0;
            }
    }
    else if(key == 'b' || key == 'B')
        {
         // toggle ambient light
         if(ambientOn)
            {
            ambientOn = false;
			glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_off);
			glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_off);
			glLightfv(GL_LIGHT0, GL_SPECULAR, specular_off);
            cout << "diable ambient" << endl;
            }
         else
            {
			GLfloat ambient_on[] = {0.0f, 1.0f, 1.0f, 1.0f};
			glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_on);
            ambientOn = true;
            cout << "enable ambient" << endl;
            }
        }
    else if(key == 'x' || key == 'X')
        {
         // toggle distant light
         if(distantOn)
            {
            distantOn = false;
			glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_off);
			glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_off);
			glLightfv(GL_LIGHT1, GL_SPECULAR, specular_off);
            cout << "diable distant" << endl;
            }
         else
            {
             distantOn = true;
			glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_d);
			glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_d);
			glLightfv(GL_LIGHT1, GL_SPECULAR, specular_d);
            cout << "enable distant" << endl;
            }
        }
    else if(key == 'c' || key == 'C')
        {
         // toggle point light
         if(pointOn)
            {
            pointOn = false;
			glLightfv(GL_LIGHT2, GL_AMBIENT, ambient_off);
			glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse_off);
			glLightfv(GL_LIGHT2, GL_SPECULAR, specular_off);
			glLightfv(GL_LIGHT2, GL_POSITION, position_off);
            cout << "diable point" << endl;
            }
         else
            {
            pointOn = true;
			glLightfv(GL_LIGHT2, GL_AMBIENT, ambient_p);
			glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse_p);
			glLightfv(GL_LIGHT2, GL_SPECULAR, specular_p);
			glLightfv(GL_LIGHT2, GL_POSITION, position_p);
            cout << "enable point" << endl;
            }
        }
    else if(key == 'v' || key == 'V')
        {
         // toggle spot light
         if(spotOn)
            {
            spotOn = false;
			glLightfv(GL_LIGHT3, GL_AMBIENT, ambient_off);
			glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse_off);
			glLightfv(GL_LIGHT3, GL_SPECULAR, specular_off);
			glLightfv(GL_LIGHT3, GL_POSITION, position_off);
            cout << "diable spot" << endl;
            }
         else
            {
             spotOn = true;
			glLightfv(GL_LIGHT3, GL_AMBIENT, ambient_s);
			glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse_s);
			glLightfv(GL_LIGHT3, GL_SPECULAR, specular_s);
			glLightfv(GL_LIGHT3, GL_POSITION, position_s);
            cout << "enable spot" << endl;
            }
        }
}

void mouse(int button, int state, int x, int y){
	//Mouse handler
	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
	else if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);//what does right button do?
	}

void mouseMove(int x, int y){

    // get moved mouse position
    mX = x;
    mY = y;
    mouseApply = true;

}

// menu switch	
void top_menu(int id){
	switch (id)
		{
		case 1:
            paused = true;
			break;
		case 2:
			paused = false;
			break;
		case 3:
			resetBoard(true);
			break;
		case 4:
			exit(0);
			break;
		}
	glutPostRedisplay();
	}
	
bool initialize(int argc, char **argv)
{
	//--gl light sources
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	
	

	//ambient light
	GLfloat ambient_a[] = {0.0f, 1.0f, 1.0f, 1.0f};
	GLfloat diffuse_a[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat specular_a[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat position_a[] = {0.0f, 0.0f, 0.0f, 0.0f};
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_a);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_a);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular_a);
	glLightfv(GL_LIGHT0, GL_POSITION, position_a);
	
	//distant light
	GLfloat ambient_d[] = {0.5f, 0.5f, 0.5f, 1.0f};
	GLfloat diffuse_d[] = {0.0f, 0.0f, 1.0f, 0.2f};
	GLfloat specular_d[] = {.5f, 1.0f, 1.0f, 0.2f};
	GLfloat position_d[] = {0.0f, 5.0f, 2.0f, 0.0f};
		
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_d);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_d);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular_d);
	glLightfv(GL_LIGHT1, GL_POSITION, position_d);	
	
	//point light
	GLfloat ambient_p[] = {0.5f, 0.5f, 0.5f, 0.2f};
	GLfloat diffuse_p[] = {0.0f, 1.0f, 0.0f, 0.2f};
	GLfloat specular_p[] = {.5f, 1.0f, 1.0f, 0.2f};
	GLfloat position_p[] = {2.0f, 1.0f, 0.0f, 1.0f};	
	
	glLightfv(GL_LIGHT2, GL_AMBIENT, ambient_p);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuse_p);
	glLightfv(GL_LIGHT2, GL_SPECULAR, specular_p);
	glLightfv(GL_LIGHT2, GL_POSITION, position_p);
	
	//spot light
	GLfloat ambient_s[] = {0.5f, 0.5f, 0.5f, 0.2f};
	GLfloat diffuse_s[] = {1.0f, 0.0f, 0.0f, 0.2f};
	GLfloat specular_s[] = {1.0f, 0.0f, 0.0f, 0.2f};
	GLfloat position_s[] = {0.0f, 2.0f, 0.0f, 1.0f};
	GLfloat spot_direction[] = { 0.0, -1.0, 0.0 };
	
	glLightfv(GL_LIGHT3, GL_AMBIENT, ambient_s);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuse_s);
	glLightfv(GL_LIGHT3, GL_SPECULAR, specular_s);
	glLightfv(GL_LIGHT3, GL_POSITION, position_s);
	glLightf(GL_LIGHT3, GL_SPOT_CUTOFF, 5.0);
	glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT3, GL_SPOT_EXPONENT, 120.0);
	
	
    // get shaders
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

    //Shader Sources
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
        std::cerr << "[F] v_tex NOT FOUND" << std::endl;
        //return false;
    }


    loc_normal = glGetAttribLocation(program, 
            const_cast<const char*>("v_normal"));

    if(loc_normal == -1)
    {
        std::cerr << "[F] v_normal NOT FOUND" << std::endl;
        //return false;
    }

    loc_Ambient = glGetUniformLocation(program,
                    const_cast<const char*>("gLight"));
    if(loc_Ambient == -1)
        {
        int error = glGetError();
        if (error == 0)
            cout<<"no error"<<endl;
        else{
            cout<<error<<endl;
            cerr<<"[f] light not found"<<endl;
            }
        }

    loc_mvpmat = glGetUniformLocation(program,
                    const_cast<const char*>("mvpMatrix"));
    if(loc_mvpmat == -1)
    {
        std::cerr << "[F] MVPMATRIX NOT FOUND" << std::endl;
        return false;
    }
    
    loc_mvmat = glGetUniformLocation(program,
                    const_cast<const char*>("mvMatrix"));
    if(loc_mvmat == -1)
    {
        std::cerr << "[F] MVMATRIX NOT FOUND" << std::endl;
        //return false;
    }
    //--Init the view and projection matrices
    //  if you will be having a moving camera the view matrix will need to more dynamic
    //  ...Like you should update it before you render more dynamic 
    //  for this project having them static will be fine
    view = glm::lookAt( *camPos, //Eye Position
                        *lookPos, //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Z is up

    projection = glm::perspective( 45.0f, //the FoV typically 90 degrees is good which is what this is set to
                                   float(w)/float(h), //Aspect Ratio, so Circles stay Circular
                                   0.01f, //Distance to the near plane, normally a small value like this
                                   1000.0f); //Distance to the far plane, 

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
    glDeleteBuffers(1, &rink[0].vbo_Geo);
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

// laods model(s) for object
Object *modelLoader(const char *objName)
	{ 
	Object *output;
	Assimp::Importer importer;
	const aiScene *scene = importer.ReadFile(objName, aiProcess_Triangulate);
	output = new Object[scene->mNumMeshes];
	output[0].numMesh = scene->mNumMeshes;
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
    cout<<objName<<endl;
	for(unsigned int meshindex =0; meshindex < scene->mNumMeshes; meshindex++){
        cout<<"mesh#: "<<meshindex+1<<endl;
        if(scene->mMeshes[meshindex]->HasNormals())
            cout<<"has Normals"<<endl;
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
		                       ,{pTexCoords->x, -pTexCoords->y},//add texture
                                {scene->mMeshes[meshindex]->mNormals[Index].x,
                                 scene->mMeshes[meshindex]->mNormals[Index].y,
                                 scene->mMeshes[meshindex]->mNormals[Index].z}}; 
		}
    cout<<"attempting binding"<<endl;
	glGenBuffers(1, &output[meshindex].vbo_Geo);
    cout<<"buffers gen'd"<<endl;
    glBindBuffer(GL_ARRAY_BUFFER, output[meshindex].vbo_Geo);
    cout<<"bind buffer"<<endl;
    glBufferData(GL_ARRAY_BUFFER, output[meshindex].NumVert*sizeof(Vertex),
                     output[meshindex].Geo, GL_STATIC_DRAW);
    cout<<"bound"<<endl;
	}
    cout<<"loaded"<<endl;
	return(output);
	}

// resets the state of the board after a score
void resetBoard(bool topScored)
{
    // bring object back to start position
    btDefaultMotionState* mState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1.6, 3)));
    cylinderRigidBody->setMotionState(mState);

    mState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1.6, -1)));
    sphereRigidBody->setMotionState(mState);

    mState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 1.6, -6)));
    cubeRigidBody->setMotionState(mState);

    // remove all forces and velocities
    cylinderRigidBody->clearForces();
    cylinderRigidBody->setLinearVelocity (btVector3(0,0,0));
    cylinderRigidBody->setAngularVelocity (btVector3(0,0,0));
    sphereRigidBody->clearForces();
    sphereRigidBody->setLinearVelocity (btVector3(0,0,0));
    sphereRigidBody->setAngularVelocity (btVector3(0,0,0));
    cubeRigidBody->clearForces();
    cubeRigidBody->setLinearVelocity (btVector3(0,0,0));
    cubeRigidBody->setAngularVelocity (btVector3(0,0,0));
    applyForward = applyBack = applyLeft = applyRight = false;
    dynamicsWorld->clearForces();


}

// load triangle collision mesh for static object
void getTriMesh(Object *obj, btTriangleMesh *tetraMesh)
{
        // add set of triangles to mesh
        for(int i = 0; i < obj->numMesh;i++)
            {
                for(unsigned int j = 0; j < obj[i].NumVert; j += 3)
                    {
                        tetraMesh->addTriangle(btVector3(obj[i].Geo[j].position[0], 
                                                        obj[i].Geo[j].position[1], 
                                                        obj[i].Geo[j].position[2]), 
                                              btVector3(obj[i].Geo[j+1].position[0], 
                                                        obj[i].Geo[j+1].position[1], 
                                                        obj[i].Geo[j+1].position[2]),  
                                              btVector3(obj[i].Geo[j+2].position[0], 
                                                        obj[i].Geo[j+2].position[1], 
                                                        obj[i].Geo[j+2].position[2]), false);
                    }
            }

}
