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
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/type_ptr.hpp> //Makes passing matrices to shaders easier

#include <btBulletDynamicsCommon.h>

#include <SFML/Audio.hpp>

using namespace std;

/**************************************
TODO:    															 
    -multiple levels
    -multiple structures
    -game mechanics
    	~scoring % for each block knocked off platform
    	~you get (3) shots to get to x score for each level
**************************************/
//****** Program classes

// This object will define the attributes of a vertex(position, color, etc...)
struct Vertex 
{
    GLfloat position[3];
    GLfloat texuv[2];
    GLfloat normal[3];
};

// Camera movement and handling
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
        if(distance > -190.0)
            {
		        distance -= .6;
            }
		}

	void CameraNode::camzot(){
        if(distance < 190.0)
            {		
                distance += .6;
            }
		}

	glm::mat4 CameraNode::getview()
		{
		return(glm::lookAt(*campos, *lookat, *up));
		}
		
	bool CameraNode::update(double DT)
		{
		*campos = glm::vec3(distance*sin(theta), 15.0, distance*cos(theta));
		return(true);
		}
		
	double CameraNode::getTheta()
		{
		    return (theta);
		}
		
// texture class: container for texture info
//CLASS FROM OGLDEV_TEXTURE
class Texture{  
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
		
// object struct: container for openGL object info
struct Object
{
	int numMesh;
    Vertex *Geo;
    char *name;
    unsigned int NumVert;
    Texture *Texs;
    GLuint vbo_Geo;

    glm::mat4 modelMat;
    btRigidBody* rigidBody;

    void Render(glm::mat4 view, glm::mat4 projection, GLuint shade)
        {
        glm::mat4 mvp = projection * view * modelMat;
        glm::mat4 mv = view * modelMat;
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
		
//****** Program functions

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



//****** Globals

int w = 1100, h = 700;// Window size
GLuint program;// The GLSL program handle
//GLuint vbo_geometry;// VBO handle for our geometry

Object *potato;
Object *cannon;
Object *groundObj;
Object * sky;
//Object *box;
//Object *knobr;
//Object *knobd;
//Object *skybox;
vector<Object*> blocks;

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

/*glm::mat4 ballmodel;
glm::mat4 tablemodel;
glm::mat4 boxmodel;
glm::mat4 knobright;
glm::mat4 knobdown;*/
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

glm::vec3 trajectory;

float cannonPower = 1700.0;
bool cannonShot = false;

float angleX = 35.0*M_PI/180.0;
float angleY = 0.0;


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
glm::vec3* camPos = new glm::vec3(0.0, 30.0, 25.0);
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
bool getBlocks(vector<Object*> &blocks, const char* fName);
bool buildBlocks(vector<Object*> &blocks, const char* fName);
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

void resetPotato();

float timer = 0.0f;
int blocksKnockedOff = 0;
int shotsLeft = 3;
float bestPercentage = 0.0;
float currentPercentage = 0.0;

float percentageX = 0.0f;
float percentageZ = 0.0f;

float xRotate = 0.0f;
float zRotate = 0.0f;


sf::SoundBuffer bufferCannon;
sf::Sound SoundCannon;
sf::SoundBuffer bufferReload;
sf::Sound reloadSound;

//--Main
int main(int argc, char **argv)
{
   // play sound
    sf::Music music;
    if (!music.OpenFromFile("loop.wav"))
        return -1; // error
    music.SetLoop(true);
    music.SetVolume(33.0f);
    music.Play();

    if (!bufferCannon.LoadFromFile("cannon.wav"))
        return -1;

    SoundCannon.SetBuffer(bufferCannon);

    if (!bufferReload.LoadFromFile("reload.wav"))
        return -1;

    reloadSound.SetBuffer(bufferReload);



    // Initialize glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(w, h);
    // Name and create the Window
    glutCreateWindow("ANGRY POTATO!!");
	
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
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

    //create the rigid body for ground
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -3, 0)));
    btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
       
    dynamicsWorld->addRigidBody(groundRigidBody);
    
    

///////////////////////////////////////
        // Initialize basic geometry
        cannon = modelLoader("cannon.obj");
	    potato = modelLoader("potato.obj");
        groundObj = modelLoader("ground.obj");
        
        sky = modelLoader("chrome.obj");
        
        //rotate sky to have any tearing in the back of the sphere
        sky->modelMat = glm::scale(glm::mat4(1.0f), glm::vec3(500,500,500));
        sky->modelMat = glm::rotate(sky->modelMat, 180.0f, glm::vec3(0,1,0));      

        btCollisionShape* platform = new btBoxShape(btVector3(18.5,1,17.5));
        btCollisionShape* ballShape = new btSphereShape(.6);
        
		//***create the platform***
        btDefaultMotionState* unmovingMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 1)));
        //pass rigid body info to create; for the ground parameters (mass-infinite, default motion state, shape, inertia-zero)
        btRigidBody::btRigidBodyConstructionInfo platformCI(0, unmovingMotionState, platform, btVector3(0, 0, 0));
        //create the rigid body
        btRigidBody* platformRigidBody = new btRigidBody(platformCI);
        //add the rigid body to the world
        dynamicsWorld->addRigidBody(platformRigidBody);

	//***create the sphere rigid body***
		//set the default motion state to have origin(0,0,0,1) and be located at (x,y,50)
        btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, 8.0,50.0)));
        //set the mass of the sphere
        btScalar mass = 50;
        //calculate the falling inertia using built in function starting from no movement
        btVector3 fallInertia(0, 0, 0);
        ballShape->calculateLocalInertia(mass, fallInertia);
        //pass rigid body info to create; for the sphere parameters (mass-1, default motion state, shape, inertia-calculated)
        btRigidBody::btRigidBodyConstructionInfo sphereRigidBodyCI(mass, fallMotionState, ballShape, fallInertia);
        //create the rigid body
        potato->rigidBody = new btRigidBody(sphereRigidBodyCI);
        potato->rigidBody->forceActivationState(DISABLE_DEACTIVATION);
        //sphereRigidBody->setFriction(100);
        //sphereRigidBody->setRestitution(2.0);
        //sphereRigidBody->setAngularFactor(btVector3(0,1,0));
        dynamicsWorld->addRigidBody(potato->rigidBody);
        
        resetPotato();

        

	
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
        delete platformRigidBody;
      
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
    for(int meshindex = 0; meshindex< cannon[0].numMesh; meshindex++)
	    cannon[meshindex].Render(view, projection, program);
	for(int meshindex = 0; meshindex < potato[0].numMesh; meshindex++)
        potato[meshindex].Render(view, projection, program);
    for(int meshindex = 0; meshindex < groundObj[0].numMesh; meshindex++)
        groundObj[meshindex].Render(view, projection, program);
    for(int meshindex = 0; meshindex < sky[0].numMesh; meshindex++)
        sky[meshindex].Render(view, projection, program);

    /*for(int meshindex = 0; meshindex < knobr[0].numMesh; meshindex++)
        {   
        knobr[meshindex].Render(view, projection, program);
        knobd[meshindex].Render(view, projection, program);
        }*/

    for(unsigned int i = 0; i < blocks.size(); i++)
        {
            for(int j = 0; j < blocks[i][0].numMesh; j++)
                {
                    blocks[i][j].Render(view, projection, program);
                }
        }
    // disable vertex attributes
    glDisableVertexAttribArray(loc_position);
    glDisableVertexAttribArray(loc_tex);
    glDisableVertexAttribArray(loc_normal);

    char cannonState[128];
    char shotsLeftTxt[128];
    char blocksKnocked[128];
    char bestPer[128];

    if(!cannonShot)
    {
        sprintf(cannonState, "Cannon State:         Ready to Fire");
    }
    else
    {
        sprintf(cannonState, "Cannon State:         Reloading");
    }

    sprintf(shotsLeftTxt,    "Shots Left:           %d", shotsLeft);
    sprintf(bestPer,         "Best Percentage:      %.2f%%", bestPercentage);
    if((int)blocks.size() > 0)
    {
        sprintf(blocksKnocked,   "%% Blocks Knocked Off: %.2f%%", currentPercentage);
    }
    else
    {
        sprintf(blocksKnocked,   "%% Blocks Knocked Off: 0.00%%");
    }

    glRasterPos2f(.4, .90);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)cannonState);
    glRasterPos2f(.4, .82);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)shotsLeftTxt);
    glRasterPos2f(.4, .74);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)blocksKnocked);
    glRasterPos2f(.4, .66);
    glutBitmapString(GLUT_BITMAP_9_BY_15, (unsigned char*)bestPer);
    //swap the buffers
    glutSwapBuffers();
}

void update()
{
    float dt = getDT();

    // update camera position
    camera.update(dt);
    view = camera.getview();

    // do not update if paused
    if(paused)
        {
            return;
        }

    dynamicsWorld->stepSimulation(dt, 5, 1.0/240.0);

    unsigned int i;
    btScalar m[16];
    btTransform trans;

    blocksKnockedOff = 0;

    // get each blocks position from bullet
    for(i = 0; i < blocks.size(); i++)
        {
            blocks[i]->rigidBody->getMotionState()->getWorldTransform(trans);
            trans.getOpenGLMatrix(m);
            blocks[i]->modelMat = glm::make_mat4(m);

            if(!((blocks[i]->modelMat[3][0] > -16.5 && blocks[i]->modelMat[3][0] < 18.0) &&
                 (blocks[i]->modelMat[3][2] > -18.0 && blocks[i]->modelMat[3][2] < 18.0)))
                {
                    blocksKnockedOff++;
                }
        }

    currentPercentage = 100*blocksKnockedOff/(float)blocks.size();

    if(currentPercentage > bestPercentage)
        {
            bestPercentage = currentPercentage;
        }

    // get potato position from bullet
    potato->rigidBody->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(m);
    potato->modelMat = glm::make_mat4(m);

    cannon->modelMat = glm::translate(glm::mat4(1.0), glm::vec3(0.0,-1.6,74.0));

    cannon->modelMat = glm::rotate(cannon->modelMat, float(angleX*180.0/M_PI), glm::vec3(1.0,0,0));
    cannon->modelMat = glm::rotate(cannon->modelMat, float(angleY*180.0/M_PI), glm::vec3(0,1.0,0));

    trajectory = glm::rotateX(glm::vec3(0.0,0.0,-1.0), float(angleX*180.0/M_PI));
    trajectory = glm::rotateY(trajectory, float(angleY*180.0/M_PI));

    if(cannonShot)
        {
            //start timer
            timer += dt;
            view = glm::lookAt( glm::vec3(-50.0,5.5, 12.0), //Eye Position
                                glm::vec3(potato->modelMat[3][0], 
                                          potato->modelMat[3][1], 
                                          potato->modelMat[3][2]), //Focus point
                                glm::vec3(0.0, 1.0, 0.0)); //Positive Z is up*/
        }
    else
        {
            timer = 0.0f;
        }

    if(timer > 7.7)
        {
            resetPotato();
            cannonShot = false;
            reloadSound.Play();
            if(shotsLeft <= 0)
                {
                    //clear platform
                    getBlocks(blocks, "nothing");

                    // reset shot counter
                    shotsLeft = 3;
                    
                }
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
    float angleOffset = 0.005;

    // Handle keyboard input
    if((key == 27)||(key == 'q')||(key == 'Q'))//ESC
    {
        exit(0);
    }
    else if((key == 'w')||(key == 'W'))
    {
        angleX += angleOffset;

        if(angleX > 75.0*M_PI/180.0)
            {
             angleX = 75.0*M_PI/180.0;
            }
    }
    else if((key == 'a')||(key == 'a'))
    {
        angleY += angleOffset;

        if(angleY > 25.0*M_PI/180.0)
            {
             angleY = 25.0*M_PI/180.0;
            }
    }
    else if((key == 's')||(key == 'S'))
    {
        angleX -= angleOffset;

        if(angleX < 10.0*M_PI/180.0)
            {
             angleX = 10.0*M_PI/180.0;
            }
    }
    else if((key == 'd')||(key == 'D'))
    {
        angleY -= angleOffset;

        if(angleY < -25.0*M_PI/180.0)
            {
             angleY = -25.0*M_PI/180.0;
            }
    }
    else if((key == 'z')||(key == 'Z'))
    {
        // reset game
        resetPotato();
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
        shotsLeft = 3;
        currentPercentage = 0.0;
        angleX = 35.0*M_PI/180.0;
        angleY = 0.0;
        getBlocks(blocks, "easyStructure");
    }
    else if(key == '2')
    {
        shotsLeft = 3;
        currentPercentage = 0.0;
        angleX = 35.0*M_PI/180.0;
        angleY = 0.0;
        getBlocks(blocks, "structure2");
    }
    else if(key == '3')
    {
        shotsLeft = 3;
        currentPercentage = 0.0;
        angleX = 35.0*M_PI/180.0;
        angleY = 0.0;
        getBlocks(blocks, "structure3");
    }
    else if(key == '4')
    {
        shotsLeft = 3;
        currentPercentage = 0.0;
        angleX = 35.0*M_PI/180.0;
        angleY = 0.0;
        buildBlocks(blocks, "testinput.txt");
    }

    else if(key == 'e' || key == 'E')
    {
        if(!cannonShot)
            {
                trajectory = glm::normalize(trajectory);
                potato->rigidBody->applyCentralImpulse(btVector3(trajectory.x*cannonPower,
                                                             trajectory.y*cannonPower,
                                                             trajectory.z*cannonPower));
                cannonShot = true;
                shotsLeft--;
                SoundCannon.Play();
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
			resetPotato();
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
	
	

	//ambient light
	GLfloat ambient_a[] = {1.0f, 0.9f, 0.9f, 1.0f};
	GLfloat diffuse_a[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat specular_a[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat position_a[] = {10.0f, 0.0f, 0.0f, 0.0f};
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient_a);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_a);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specular_a);
	glLightfv(GL_LIGHT0, GL_POSITION, position_a);
	
	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_a);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse_a);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular_a);
	glLightfv(GL_LIGHT1, GL_POSITION, position_a);
	
	
	glLightfv(GL_LIGHT2, GL_AMBIENT, ambient_a);

    // get shaders
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);

    const char *vs;
    const char *fs;

    if(argc == 1)
        {
            char vsn[] = "shadervs.cg";
            char fsn[] = "shaderfs.cg";
            vs = shaderloader(vsn);
            fs = shaderloader(fsn);
        }
    else
        {
            vs = shaderloader(argv[1]);
            fs = shaderloader(argv[2]);
        }
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

    camera.init(125.0, 0, glm::vec3(0.0, 1.0, 0.0), *lookPos);
    
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

    
    cout<<"init done"<<endl;
    //and its done
    return true;
}

void cleanUp()
{
    // Clean up,
    glDeleteProgram(program);
    glDeleteBuffers(1, &potato[0].vbo_Geo);
    glDeleteBuffers(1, &cannon[0].vbo_Geo);
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

    output->modelMat = glm::mat4(1.0);
	return(output);
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

bool getBlocks(vector<Object*> &blocks, const char* fName)
{
    int i;
    unsigned int j;
    // if object already exist, then remove rigid bodies from dynamics world
    if(blocks.size() > 0)
        {
            for(j = 0; j < blocks.size(); j++)
                {
                    dynamicsWorld->removeRigidBody(blocks[j]->rigidBody);
                    delete blocks[j]->rigidBody->getMotionState();
                    delete blocks[j]->rigidBody;
                }
            // clear the vector
            blocks.clear();
        }


    int numBlocks, scan;

    float x,y,z,rx,ry,rz;

    Object* tempOBJ;

    int randomX = rand() % 16 - 8;
    int randomZ = rand() % 5 - 11;
    FILE* fp;
    fp = fopen(fName, "r");
    // get number of blocks in the file
    scan = fscanf(fp, "%d", &numBlocks);

    // for each block in the file
    for(i = 0; i < numBlocks; i++)
        {
            // make new object
            tempOBJ = modelLoader("jenga.obj");
            // get position and orientation
            scan = fscanf(fp, "%f %f %f %f %f %f", &x, &y, &z, &rx, &ry, &rz);
            if(scan == EOF)
                {
                    return false;
                }
            // create new fall motion state
            btDefaultMotionState* fallMotionState =
                    new btDefaultMotionState(btTransform(btQuaternion(ry*M_PI/180.0, 
                                                                      rx*M_PI/180.0,
                                                                      rz*M_PI/180.0),
                                                                      btVector3(x + randomX, 
                                                                                y, 
                                                                                z + randomZ)));
            //set the mass of the sphere
            btScalar mass = 10;
            // set the collision shape of block
            btCollisionShape* box = new btBoxShape(btVector3(1.5,0.3,0.5));
            //calculate the falling inertia using built in function 
            //starting from no movement
            btVector3 fallInertia(0, 0, 0);
            box->calculateLocalInertia(mass, fallInertia);
            //pass rigid body info to create; for the sphere parameters 
            //(mass-1, default motion state, shape, inertia-calculated)
            btRigidBody::btRigidBodyConstructionInfo blockRigidBodyCI(mass, 
                                                                fallMotionState, box,
                                                                         fallInertia);
            //create the rigid body
            tempOBJ->rigidBody = new btRigidBody(blockRigidBodyCI);
            tempOBJ->rigidBody->forceActivationState(DISABLE_DEACTIVATION);
            tempOBJ->rigidBody->setFriction(1.0);
            tempOBJ->rigidBody->setRestitution(.05);
            // add to dynamics world
            dynamicsWorld->addRigidBody(tempOBJ->rigidBody);
            // push new object into vector
            blocks.push_back(tempOBJ);
        }

    fclose(fp);
    
    return true;
}

bool buildBlocks(vector<Object*> &blocks, const char* fName)
    {
    unsigned int j;
    // if object already exist, then remove rigid bodies from dynamics world
    if(blocks.size() > 0)
        {
            for(j = 0; j < blocks.size(); j++)
                {
                    dynamicsWorld->removeRigidBody(blocks[j]->rigidBody);
                    delete blocks[j]->rigidBody->getMotionState();
                    delete blocks[j]->rigidBody;
                }
            // clear the vector
            blocks.clear();
        }
    int wdh[3];
    float rotation;
    float currRot = 0.0;
    double CoM[3];
    double xyz[3];
    double movevec[3] = {0.0, 0.0, 0.0};
    ifstream Build;
    Build.open(fName);
    Object* tempOBJ;
    glm::vec3 tempVect;
    //get the width depth and height of the structure to be built
    //the rotation of each level of the structure, and the Center of Mass
    Build>>wdh[0]>>wdh[1]>>wdh[2]>>rotation>>CoM[0]>>CoM[1]>>CoM[2];
    //calculate the start point.
    xyz[1] = CoM[1]; //the y should be the same
    xyz[0] = CoM[0] - (wdh[0]-1)*.5; //find x start point.
    xyz[2] = CoM[2] - (wdh[1]-1)*1.5; // find the z start point.
    glm::vec3 vectortoStart(xyz[0]-CoM[0], xyz[1], xyz[2]-CoM[2]);
    while (Build.good())
        {
        for(int height = 0; height < wdh[2]; height++){
            for(int depth = 0; depth < wdh[1]; depth++){
                for(int width = 0; width < wdh[0]; width++){
                    tempOBJ = modelLoader("jenga.obj");
                    btDefaultMotionState* fallMotionState =
                    new btDefaultMotionState(btTransform(btQuaternion		
                    						(currRot*M_PI/180.0,
                    						 0,
                    						 0),//btQuaternion
                                                 btVector3(xyz[0]+movevec[0], 
                           	                               xyz[1]+movevec[1], 
                                                         xyz[2]+movevec[2])
                                                         //btVector
                                                         )//btTransform
                                                         );//btDefaultmotion
                    //set the mass of the sphere
                    btScalar mass = 10;
                    // set the collision shape of block
                    btCollisionShape* box = new btBoxShape(btVector3(1.5,0.3,0.5));
                    //calculate the falling inertia using built in function 
                    //starting from no movement
                    btVector3 fallInertia(0, 0, 0);
                    box->calculateLocalInertia(mass, fallInertia);
                    //pass rigid body info to create; for the sphere parameters 
                    //(mass-1, default motion state, shape, inertia-calculated)
                    btRigidBody::btRigidBodyConstructionInfo blockRigidBodyCI(mass, 
                                                                fallMotionState, box,
                                                                         fallInertia);
                    //create the rigid body
                    tempOBJ->rigidBody = new btRigidBody(blockRigidBodyCI);
                    tempOBJ->rigidBody->forceActivationState(DISABLE_DEACTIVATION);
                    tempOBJ->rigidBody->setFriction(1.0);
                    tempOBJ->rigidBody->setRestitution(1.0);
                    // add to dynamics world
                    dynamicsWorld->addRigidBody(tempOBJ->rigidBody);
                    // push new object into vector
                    blocks.push_back(tempOBJ);
                    //place width number of jenga bricks at start point
                    movevec[2]-= 1.01*cos(currRot*M_PI/180.0);
                   	movevec[0]-= 1.01*sin(currRot*M_PI/180.0); 
                    }
                movevec[2] = 0.0;
                movevec[0] -= 3.1*cos(currRot*M_PI/180.0); //move an amount in depth
               	movevec[2] -= 3.1*sin(currRot*M_PI/180.0); //
            //move back from starting point one depth of jenga brick
                }
            currRot += rotation;
            cout<<currRot<<endl;
            if (int(currRot) >= 180){
            	currRot -= 180.0;
            	}
            //rotate starting vector to current direction
            tempVect = glm::rotate(vectortoStart, currRot, glm::vec3(0,1,0));
            //rotate for roation
            /*cout<<"height: "<<height<<"currentrot: "<<currRot<<endl;
       cout<<vectortoStart.x<<":"<<vectortoStart.y<<":"<<vectortoStart.z<<endl;
            cout<<tempVect.x<<":"<<tempVect.y<<":"<<tempVect.z<<endl;*/
            xyz[0] = tempVect.x*pow(-1, height+1) + CoM[0]*pow(-1,height);
            xyz[2] = tempVect.z*pow(-1, height+1) + CoM[2]*pow(-1,height);
            movevec[0] = 0.0;
            movevec[2] = 0.0;
            movevec[1] += .6;
            }
        currRot = 0.0;
        movevec[0]= movevec[1] = movevec[2] = 0.0;
        Build>>wdh[0]>>wdh[1]>>wdh[2]>>rotation>>CoM[0]>>CoM[1]>>CoM[2];
        //calculate the start point.
    	xyz[1] = CoM[1]; //the y should be the same
    	xyz[0] = CoM[0] - (wdh[0]-1)*.5; //find x start point.
    	xyz[2] = CoM[2] - (wdh[1]-1)*1.5; // find the z start point.
        }
    Build.close();
    return true;
    }
 
void resetPotato()
{
    btDefaultMotionState* mState =
            new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0, 0.2, 75.0)));
    potato->rigidBody->setMotionState(mState);
    potato->rigidBody->clearForces();
    potato->rigidBody->setLinearVelocity (btVector3(0,0,0));
    potato->rigidBody->setAngularVelocity (btVector3(0,0,0)); 
    dynamicsWorld->clearForces();
}










