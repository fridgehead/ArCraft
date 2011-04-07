#ifndef _TEST_APP
#define _TEST_APP


#include "ofMain.h"
#include "ofxNetwork.h"
#include "Vector.h"
#include "Poco/RegularExpression.h"
using Poco::RegularExpression;

enum BlockType  {	GRASS = 2, COBBLE = 4, LAVA = 10,
	LAVA2 = 11, STONE = 1, DIRT = 3,
	LOGS = 5, WATER = 8, LEAVES = 18,
	SNOW = 78, NONE = 0 };

struct Block {
	BlockType type;
	
};




class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
	
		void drawBlock(int x, int y, int z, int wx, int wy, int wz);
		void trim(string& str);
		void processShit(const string& str);
		float clickX, clickY, rotXAmt, rotYAmt, rotX, rotY;
	
		ofxUDPManager udpConnectionRx;
	
		string rxMessage;
		//Block blocks [30][30][30] ;
		vector<vector<vector<Block> > > array3D;
	
};



#endif
