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
//#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp> //Makes passing matrices to shaders easier

#include <btBulletDynamicsCommon.h>
using namespace std;

/**************************************
TODO:    															 y			x
    gravity class to handle rotating gravity					0\-	   7/4p0\2p
    															 \  \		\
    	2 angles theta and rho								 ----\----xz z--\--
    	theta is the direction in x, z							 \  /		\
    	radius = 9.8 z is noramally up so switch that to y     pi\-	  5/4p p\
    	
    	x = r cos (theta) * sin rho y = r cos rho z = r sin theta sin rho
    	since we want the direction to be down rho needs to start off at pi
    	gravity current direction=(r*cos(0)*sin(pi),r cos(pi),r sin(0)sin(pi))
		for init gravity
		
		gravity should interpolate to a theta, decreasing rho to say no less
		than  150 degrees or 135 degrees movement of about 30 or 45 degrees.
	
    rotating knobs
	
	skybox would be great... and correct lighing/shading
**************************************/
//This object will define the attributes of a vertex(position, color, etc...)
struct Vertex //https://github.com/ccoulton/cs480coulton.git
{
    GLfloat position[3];
    GLfloat texuv[2];
    GLfloat normal[3];
};

class CameraNode{
    public:
        CameraNode();
        bool init(double in_dist, double in_theta, 
                    glm::vec3 in_up, 
                    glm::vec3 lookAtWhat);
        bool update(double DT);
        void camlft();
        void camrgt();
        void camzin();
        void camzot();
        glm::mat4 getview();
        double getTheta();
    private:
        double theta;
        double distance;
        glm::vec3 *lookat;
        glm::vec3 *campos;
        glm::vec3 *up;
    };
    
CameraNode::CameraNode()
    {
    lookat = new glm::vec3(0,0,0);
    }

bool CameraNode::init(double in_dist, double in_theta, glm::vec3 in_up
                    , glm::vec3 lookAtWhat)
    {
    up = new glm::vec3(in_up[0],in_up[1],in_up[2]);
    *lookat = lookAtWhat;
    distance = in_dist;
    theta = in_theta;
    
    campos = new glm::vec3(distance*sin(theta), 15.0, distance*cos(theta));
    
    return(true);
    }

void CameraNode::camlft(){
    theta -= (2*M_PI)/360;
    }

void CameraNode::camrgt(){
    theta += (2*M_PI)/360;
    }

void CameraNode::camzin(){
    distance -= .3;
    }

void CameraNode::camzot(){
    distance += .3;
    }

glm::mat4 CameraNode::getview()
    {
    return(glm::lookAt(*campos, *lookat, *up));
    }
    
bool CameraNode::update(double DT)
    {
    //theta += (2*M_PI)/360;
    *campos = glm::vec3(distance*sin(theta), 15.0, distance*cos(theta));
    return(true);
    }
    
double CameraNode::getTheta()
    return (theta);

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
	glTexImage2D(mtextureTarget, 0, GL_RGBA, mimage.columns(), mimage.rows(), 0, 			GL_RGBA, GL_UNSIGNED_BYTE, mblob.data());
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


int w = 1100, h = 700;// Window size
GLuint program;// The GLSL program handle
//GLuint vbo_geometry;// VBO handle for our geometry

Object *ball;
Object *table;
Object *box;
Object *knobr;
Object *knobd;
//Object *skybox;

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

glm::mat4 ballmodel;
glm::mat4 tablemodel;
glm::mat4 boxmodel;
glm::mat4 knobright;
glm::mat4 knobdown;
//glm::mat4 skyboxmodel;
glm::mat4 view;//world->eye position of the camera
glm::mat4 projection;//eye->clip lens of the camera
glm::mat4 mvp;//premultiplied modelviewprojection
CameraNode camera;

// motion flags
bool applyForward = false;
bool applyLeft = false;
bool applyBack = false;
bool applyRight = false;

//gravity to change for moving the ball
btVector3 normalGravity = btVector3(0.0,-9.81,0.0);
btVector3 backGravity = btVector3(0.0,-6.93672,6.93672);
btVector3 frontGravity = btVector3(0.0,-6.93672,-6.93672);
btVector3 leftGravity = btVector3(-6.93672,-6.93672,0.0);
btVector3 rightGravity = btVector3(6.93672,-6.93672,0.0);
btVector3 frontLeftGravity = btVector3(-5.66381,-5.66381,-5.66381);
btVector3 frontRightGravity = btVector3(5.66381,-5.66381,-5.66381);
btVector3 backLeftGravity = btVector3(-5.66381,-5.66381,5.66381);
btVector3 backRightGravity = btVector3(5.66381,-5.66381,5.66381);


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
glm::vec3* camPos = new glm::vec3(0.0, 20.0, 15.0);
glm::vec3* lookPos = new glm::vec3(0.0, 0.0, 0.0);

glm::vec3 pPos;

//--GLUT Callbacks
void render();
void update();
void reshape(int n_w, int n_h);
void keyboard(unsigned char key, int x_pos, int y_pos);
void keyboardUp(unsigned char key, int x_pos, int y_pos);
void mouse(int button, int state, int x, int y);
void mouseMove(int x, int y);
void top_menu(int id);

//--Resource management
bool initialize(int argc, char **argv);
void cleanUp();
Object *modelLoader(const char *objName);
void resetBoard();
void getTriMesh(Object *obj, btTriangleMesh *tetraMesh);

//--Random time things
float getDT();
std::chrono::time_point<std::chrono::high_resolution_clock> t1,t2;

//--bullet globals
btDiscreteDynamicsWorld* dynamicsWorld;
btRigidBody* sphereRigidBody;


//bullet collision masks
#define BIT(x) (1<<(x))
enum collisiontypes {
    COL_BALL = BIT(0), //<Collide with paddles
    COL_WALL = BIT(1), //<Collide with rink
    COL_END = BIT(2), //<Collide with center plane
};

int ballCollidesWith = COL_WALL | COL_END;
int tableCollidesWith = COL_BALL;


//--Shaderloader
const char *shaderloader(char *input);

// linear interpolation
float lerp(float v0, float v1, float t);

float timer = 0.0f;
float currentScore = 0.0f;
float highScore = 0.0f;

float percentageX = 0.0f;
float percentageZ = 0.0f;

float xRotate = 0.0f;
float zRotate = 0.0f;

//--Main
int main(int argc, char **argv)
{
    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("Assignment 11 - Marble Labyrinth!!");
	
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
    glutKeyboardUpFunc(keyboardUp);
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

///////////////////////////////////////
        // Initialize basic geometry
        table = modelLoader("maze.obj");
	    ball = modelLoader("chrome.obj");
        box = modelLoader("edge.obj");

        knobr = modelLoader("knobright.obj");
        knobd = modelLoader("knobdown.obj");

        //skybox =  modelLoader("skybox.obj");

        // init triangle meshes
        btTriangleMesh* tetraMesh = new btTriangleMesh();

        // get triangle meshe
        getTriMesh(table, tetraMesh);       

        // create bullet colission shapes   
        btBvhTriangleMeshShape* tableShape = new btBvhTriangleMeshShape(tetraMesh, false);
        btCollisionShape* ballShape = new btSphereShape(.4);
        
		//create the ground with a Quaternion(0,0,0,1) the origin and position -1 to the origin
        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
        //pass rigid body info to create; for the ground parameters (mass-infinite, default motion state, shape, inertia-zero)
        // note:  if you want to create many objects with the same rigid body info, only need to do this once
        btRigidBody::btRigidBodyConstructionInfo
              tableRigidBodyCI(0, groundMotionState, tableShape, btVector3(0, 0, 0));
        //create the rigid body for the base of the board
        btRigidBody* tableRigidBody = new btRigidBody(tableRigidBodyCI);
        //add the rigid body to the world
        dynamicsWorld->addRigidBody(tableRigidBody, COL_WALL, tableCollidesWith);

	//***create the sphere rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,50)
        btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(2.8,0.6,-7.9)));
        //set the mass of the sphere
        btScalar mass = 50;
        //calculate the falling inertia using built in function starting from no movement
        btVector3 fallInertia(0, 0, 0);
        ballShape->calculateLocalInertia(mass, fallInertia);
        //pass rigid body info to create; for the sphere parameters (mass-1, default motion state, shape, inertia-calculated)
        btRigidBody::btRigidBodyConstructionInfo sphereRigidBodyCI(mass, fallMotionState, ballShape, fallInertia);
        //create the rigid body
        sphereRigidBody = new btRigidBody(sphereRigidBodyCI);
        sphereRigidBody->forceActivationState(DISABLE_DEACTIVATION);
        sphereRigidBody->setFriction(100);
        sphereRigidBody->setRestitution(2.0);
        //sphereRigidBody->setAngularFactor(btVector3(0,1,0));
        dynamicsWorld->addRigidBody(sphereRigidBody, COL_BALL, ballCollidesWith);

	
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
    glClearColor(0.1, 0.5, 0.5, 0.7);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   
    glUseProgram(program);
    //set up the Vertex Buffer Object so it can be drawn
    glEnableVertexAttribArray(loc_position);
    glEnableVertexAttribArray(loc_tex);
    glEnableVertexAttribArray(loc_normal);
    
    //render each ob
    //methodize me
    for(int meshindex = 0; meshindex< table[0].numMesh; meshindex++)
	    table[meshindex].Render(tablemodel, view, projection, program);
	for(int meshindex = 0; meshindex < ball[0].numMesh; meshindex++)
        ball[meshindex].Render(ballmodel, view, projection, program);
    for(int meshindex = 0; meshindex < box[0].numMesh; meshindex++)
        box[meshindex].Render(boxmodel, view, projection, program);

    for(int meshindex = 0; meshindex < knobr[0].numMesh; meshindex++)
        {   
        knobr[meshindex].Render(knobright, view, projection, program);
        knobd[meshindex].Render(knobdown, view, projection, program);
        }

    /*for(int meshindex = 0; meshindex < skybox[0].numMesh; meshindex++)
        skybox[meshindex].Render(skyboxmodel, view, projection, program);*/

    // disable vertex attributes
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_tex);
    glDisableVertexAttribArray(loc_normal);

    // print score (2-D text)
    char scoreTex[64];
    char highScoreTex[64];
    scoreTex[0] = '\0';
    highScoreTex[0] = '\0';
    sprintf(scoreTex,     "Score:      %d", (int)currentScore);
    sprintf(highScoreTex, "High Score: %d", (int)highScore);

    glColor3f(1.0f, 1.0f, 1.0f);
    glRasterPos2f(.6, .8);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)scoreTex);
    glColor3f(0.5f, 0.0f, 1.0f);
    glRasterPos2f(.6, .72);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)highScoreTex);

    // time
    glRasterPos2f(.6, .64);
    char timestr[100];
    sprintf(timestr, "Time:       %d", (int)timer);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)timestr);

    // show if paused
    if(paused)
        {
            glRasterPos2f(.6, .57);
            char pausestr[100];
            pausestr[0] = '\0';
            sprintf(pausestr, "PAUSED");
            glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)pausestr);
        }
    //swap the buffers
    glutSwapBuffers();
}

void update()
{
    float dt = getDT();

    // do not update if paused
    if(paused)
        {
            return;
        }

    timer += dt;
    static int scoreCtr = 0;

    // update camera position
    camera.update(dt);
    view = camera.getview();

    // calc score
    currentScore = 300 - (timer*4);

    // if time up
    if(timer > 80)
        {
            timer = 0;
            resetBoard();
            return;
        }

    // change light color depending on time left
    if(currentScore > 200)
        {
            // set lighting color to green
            GLfloat ambient_on[] = {0.0f, 1.0f, 0.0f, 1.0f};
			glLightfv(GL_LIGHT0, GL_DIFFUSE, ambient_on);

        }
    else if(currentScore <= 200 && currentScore > 100)
        {
            // set lighting color to yellow
            GLfloat ambient_on[] = {1.0f, 1.0f, 0.0f, 1.0f};
			glLightfv(GL_LIGHT0, GL_DIFFUSE, ambient_on);
        }
    else
        {
            // set lighting color to red
            GLfloat ambient_on[] = {1.0f, 0.0f, 0.0f, 1.0f};
			glLightfv(GL_LIGHT0, GL_DIFFUSE, ambient_on);
        }

    double theta = 0;
    double rho = M_PI - M_PI/60;
    //apply gravity changes to move the ball around
    // only  forward
    if(applyForward && !applyRight && !applyLeft && !applyBack)
        {
            theta = camera.getTheta() - M_PI;
            percentageX = lerp(percentageX, 0.0f, 0.03f);
            dynamicsWorld->setGravity(frontGravity);
        }
    // only left
    else if(!applyForward && !applyRight && applyLeft && !applyBack)
        {
            theta = camera.getTheta() - M_PI/2;
            percentageZ = lerp(percentageZ, 0.0f, 0.03f);
            dynamicsWorld->setGravity(leftGravity);
        }
    // only right
    else if(!applyForward && applyRight && !applyLeft && !applyBack)
        {
            theta = camera.getTheta() + M_PI/2;
            percentageZ = lerp(percentageZ, 0.0f, 0.03f);
            dynamicsWorld->setGravity(rightGravity);
        }
    // only back
    else if(!applyForward && !applyRight && !applyLeft && applyBack)
        {
            theta = camera.getTheta();
            percentageX = lerp(percentageX, 0.0f, 0.03f);
            dynamicsWorld->setGravity(backGravity);
        }
    // forward right
    else if(applyForward && applyRight && !applyLeft && !applyBack)
        {
            dynamicsWorld->setGravity(frontRightGravity);
        }
    // forward left
    else if(applyForward && !applyRight && applyLeft && !applyBack)
        {
            dynamicsWorld->setGravity(frontLeftGravity);
        }
    // back right
    else if(!applyForward && applyRight && !applyLeft && applyBack)
        {
            dynamicsWorld->setGravity(backRightGravity);
        }
    // back left
    else if(!applyForward && !applyRight && applyLeft && applyBack)
        {
            dynamicsWorld->setGravity(backLeftGravity);
        }
    // no lean
    else
        {
            rho = M_PI;
            dynamicsWorld->setGravity(normalGravity);

            // interpolate values back to normal
            percentageX = lerp(percentageX, 0.0f, 0.03f);
            percentageZ = lerp(percentageZ, 0.0f, 0.03f);
        }
    //applyForward = applyBack = applyLeft = applyRight = false;
    double radius = 9.8;
    btVector3 gravity = btVector3(radius*cos(theta)*sin(rho), 
                                  radius*cos(rho), 
                                  radius*sin(theta)sin(rho));
    dynamicsWorld->setGravity(gravity);

    // calc new gravity vector
    xRotate = percentageX * 100.0 * M_PI / 180.0;
    zRotate = percentageZ * 100.0 * M_PI / 180.0;

    // rotate board from lean
    tablemodel = glm::rotate(glm::mat4(1.0f), -zRotate, glm::vec3(1.0f, 0.0f, 0.0f));
    tablemodel = glm::rotate(tablemodel, xRotate, glm::vec3(0.0f, 0.0f, 1.0f));

    // rotate right knob
    knobright = glm::rotate(glm::mat4(1.0f), -zRotate, glm::vec3(1.0f, 0.0f, 0.0f));
    

    // rotate bottom knob
    knobdown = glm::rotate(glm::mat4(1.0f), xRotate, glm::vec3(0.0f, 0.0f, 1.0f));

    // tick simulation
    dynamicsWorld->stepSimulation(dt, 1);

	//create a transform object, 
    //this holds all the position and orientation information in the dynamics world at this step
    btTransform trans;

    btScalar m[16];

    // get sphere model matrix
    sphereRigidBody->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(m);
    ballmodel = glm::make_mat4(m);

    // roate ball from lean
    ballmodel = glm::rotate(ballmodel, -zRotate, glm::vec3(1.0f, 0.0f, 0.0f));
    ballmodel = glm::rotate(ballmodel, xRotate, glm::vec3(0.0f, 0.0f, 1.0f));
/*
    // cap velocities
    btVector3 velocity = sphereRigidBody->getLinearVelocity();
    btScalar speed = velocity.length();
    if(speed > 2.5) {
        velocity *= 2.5/speed;
        sphereRigidBody->setLinearVelocity(velocity);
    }
*/


    // get position of sphere
    pPos = glm::vec3(ballmodel[3][0], ballmodel[3][1], ballmodel[3][2]);

    // check for collisions
    bool pCollision = pPos.x > 8.1 && pPos.x < 8.5 &&
                       pPos.y > 0.38 && pPos.y < 0.9 &&
                       pPos.z > 1.6 && pPos.z < 1.9 &&
                       scoreCtr == 0;

    bool ballOut = pPos.y < -.5;


    // save score
    if(pCollision)
        {
            resetBoard();
            p1Score++;
            scoreCtr = 8;
            timer = 0;
            if(currentScore > highScore)
                {
                    highScore = currentScore;
                }
            applyForward = applyBack = applyLeft = applyRight = false;
        }
    // buffer (to prevent multiple scores)
    if(scoreCtr > 0)
        {
            scoreCtr--;
        }
    // ball falls through hole
    if(ballOut)
        {
            resetBoard();
            timer = 0;  
        }

    // Update the state of the scene
    glutPostRedisplay();
}


void reshape(int n_w, int n_h)
{
    w = n_w;
    h = n_h;
    //Change the viewport to be correct
    glViewport( 0, 0, w, h);
    //Update the projection matrix as well
    //See the init function for an explaination
    projection = glm::perspective(45.0f, float(w)/float(h), 0.01f, 10000.0f);

}

void keyboard(unsigned char key, int x_pos, int y_pos)
{
    float tiltAdd = 0.1;

    // Handle keyboard input
    if((key == 27)||(key == 'q')||(key == 'Q'))//ESC
    {
        exit(0);
    }
    else if((key == 'w')||(key == 'W'))
    {
        // lean board forward
        percentageZ += tiltAdd;
        if(percentageZ > 1.0f)
            {
                percentageZ = 1.0f;
            }
        applyForward = true;
    }
    else if((key == 'a')||(key == 'A'))
    {
        // lean board left
        percentageX += tiltAdd;
        if(percentageX > 1.0f)
            {
                percentageX = 1.0f;
            }
        applyLeft = true;
    }
    else if((key == 's')||(key == 'S'))
    {
        // lean board back
        percentageZ -= tiltAdd;
        if(percentageZ < -1.0f)
            {
                percentageZ = -1.0f;
            }
        applyBack = true;
    }
    else if((key == 'd')||(key == 'D'))
    {
        // lean board right
        percentageX -= tiltAdd;
        if(percentageX < -1.0f)
            {
                percentageX = -1.0f;
            }
        applyRight = true;
    }
    else if((key == 'z')||(key == 'Z'))
    {
        // reset game
        resetBoard();
        timer = 0.0f;
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
        camera.camzin();
    }
    else if((key == 'j')||(key == 'J'))
    {
        camera.camlft();
    }
    else if((key == 'k')||(key == 'K'))
    {
        camera.camzot();
    }
    else if((key == 'l')||(key == 'L'))
    {
        camera.camrgt();
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
}

void keyboardUp(unsigned char key, int x_pos, int y_pos)
{
    if(key == 'w' || key == 'W')
        {
            applyForward = false;
        }
    if(key == 'a' || key == 'A')
        {
            applyLeft = false;
        }
    if(key == 's' || key == 'S')
        {
            applyBack = false;
        }
    if(key == 'd' || key == 'D')
        {
            applyRight = false;
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
			resetBoard();
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
/*
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
	
	*/
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

    camera.init(20.0, 0, glm::vec3(0.0, 1.0, 0.0), *lookPos);
    
    view = glm::lookAt( *camPos, //Eye Position
                        *lookPos, //Focus point
                        glm::vec3(0.0, 1.0, 0.0)); //Positive Z is up*/

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
    glDeleteBuffers(1, &ball[0].vbo_Geo);
    glDeleteBuffers(1, &table[0].vbo_Geo);
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
void resetBoard()
{
    // remove tilt
    percentageX = percentageZ = 0.0f;

    // bring object back to start position
    btDefaultMotionState* mState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(2.8, 0.6, -7.9)));
    /*btDefaultMotionState* mState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(8.2, 0.4, 0)));*/

    sphereRigidBody->setMotionState(mState);
    sphereRigidBody->clearForces();
    sphereRigidBody->setLinearVelocity (btVector3(0,0,0));
    sphereRigidBody->setAngularVelocity (btVector3(0,0,0));
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

float lerp(float v0, float v1, float t) {
  return (1-t)*v0 + t*v1;
}
