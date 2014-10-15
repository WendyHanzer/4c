#ifndef MODELCLASSES_H
#define MODELCLASSES_H
#define GLM_FORCE_RADIANS
#include <GL/glew.h> 
#include <GL/glut.h>
#include <Magick++.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <vector>

using namespace std;

class Object;

struct Vertex
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

struct meshData
{
    unsigned int NumVert;
    Vertex *Geo;
    GLuint bufferName;
    Texture *Texs;
};

struct bodyData
{
    float selfSpin;
    float revolution;
    float axisTilt; // self lean
    float radius; // of planet
    float revolutionRadius; // orbit raduis
    float revolutionTilt; // tilt of orbit
    bool isMoon;
    int parentInd;
};

class Object
{
    public:
        Object();

        bool bind(int index);
        bool bind();
        void render();
        bool load(std::string objName); //load obj model and texture info
        void tick(float dt);
        
        char *name;
        bodyData planetData;
        vector<Object> *moons;

        // model
        meshData *mesh;
        glm::mat4 modelMatrix;
        int numMesh;

};

#endif
