#ifndef _TEST_APP
#define _TEST_APP


#include "ofMain.h"
#include "ofxNetwork.h"
#include "ofxOpenCv.h"
#include "Vector.h"
#include "ofxOpenNI.h"
#include "ofxVectorMath.h"
#include "NetworkThread.h"
class NetworkThread;


enum BlockType  {	GRASS = 2, COBBLE = 4, LAVA = 10,
	LAVA2 = 11, STONE = 1, DIRT = 3,
	LOGS = 5, WATER = 8, LEAVES = 18,
	SNOW = 78, NONE = 0, TREE = 17 };

struct Block {
	BlockType type;
	int textureRef;
	bool textured;
	
};




class testApp : public ofBaseApp {
	
public:
	void setup();
	void update();
	void draw();
	void stop();
	
	void keyPressed  (int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
	void drawBlock(int x, int y, int z, int wx, int wy, int wz, Block *bType);
	void trim(string& str);
	void processShit(const string& str);
	bool testVisibility(int x, int y, int z);
	
	/** Fbo init and stuff */
	void initFrameBufferTexture();
	void initFrameBufferDepthBuffer();
	void initFrameBuffer();
	
	void bindFbo();
	void unbindFbo();
	void drawFbo();
	
	GLuint mDepthID;
	GLuint mFBOID;
	GLuint mTextureID;	
	
	
	float clickX, clickY, rotXAmt, rotYAmt, rotX, rotY;
	
	//net stuff
	//ofxUDPManager udpConnectionRx;	
	//string rxMessage;
	
	NetworkThread* netThread;
	
	
	//map data
	//Block blocks [30][30][30] ;
	int mapWidth, mapHeight, mapDepth;
	bool mapLocked;
	vector<vector<vector<Block> > > array3D;
	
	
	//camera stuff
	bool bDraw;	
	ofxOpenNIContext context;
	ofxDepthGenerator depth;
	ofxUserGenerator user;
	
	ofxImageGenerator image;
	
	ofxCvColorImage convert;
	ofxCvGrayscaleImage gray;
	
	ofxCvColorImage kinectImage;
	ofxCvGrayscaleImage kinectDepthImage;
	ofxCvGrayscaleImage finalMaskImage;
	ofxCvGrayscaleImage sceneDepthImage;
	ofxCvColorImage finalImage;
	ofxCvColorImage sceneImage;
	
	ofVideoGrabber grabber;
	
	//temp pixel buffer for depth
	unsigned char * pixelBuf;
	unsigned char * colorPixelBuf;
	unsigned char * finalImageBuf;
	unsigned short * sceneDepthBuf;
	unsigned short * kinectDepthBuf;
	unsigned char * finalBuf;	
	unsigned char * sceneBuffer;
	
	ofxVec3f whitePoint;
	
	ofImage grassImage;
	ofImage textures[9];
	ofTexture grassTexture;
	
	int mx, my;
	
	
	float mapScale;
	bool guiDraw;	
	ofxPoint2f offset;
	float scVal;
	
	//tracking stuff
	//ARToolKitPlus::TrackerSingleMarker *tracker;
	ofColor sceneWhiteLevel;

	
};



#endif
