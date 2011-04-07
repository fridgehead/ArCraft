#include "testApp.h"
#include "ARToolKitPlus/TrackerSingleMarkerImpl.h"

ARToolKitPlus::TrackerSingleMarker *tracker;
static const int       width = 640, height = 480, bpp = 1;
static   size_t        numPixels = width*height*bpp;
static    size_t        numBytesRead;
static   const char    *fName = "data/markerboard_480-499.raw";
static   const char    *tagName = "data/patt.hiro";

static    unsigned char *cameraBuffer = new unsigned char[numPixels];
static bool useBCH = true;

//--------------------------------------------------------------
void testApp::setup(){
	//tracker = new ARToolKitPlus::TrackerSingleMarkerImpl<16,16,64, ARToolKitPlus::PIXEL_FORMAT_RGB565>(640,480);
	grabber.initGrabber(640, 480);
	convert.allocate(640, 480);
	gray.allocate(640, 480);
	bDraw = false;
	
	tracker = new ARToolKitPlus::TrackerSingleMarkerImpl<6,6,6, 1, 8>(width,height);
	
	const char* description = tracker->getDescription();
	printf("ARToolKitPlus compile-time information:\n%s\n\n", description);
	
    // set a logger so we can output error messages
    
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);	
	
    if( !tracker->init( (const char *)ofToDataPath("LogitechPro4000.dat").c_str(), 1.0f, 1000.0f) )            // load std. ARToolKit camera file
	{
		printf("ERROR: init() failed\n");
		delete cameraBuffer;
		delete tracker;
		return;
	}
	
    // define size of the marker
    tracker->setPatternWidth(80);
	
	// the marker in the BCH test image has a thin border...
    tracker->setBorderWidth(useBCH ? 0.125f : 0.250f);
	
    // set a threshold. alternatively we could also activate automatic thresholding
    tracker->setThreshold(150);
	
    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
	
    // RPP is more robust than ARToolKit's standard pose estimator
    tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
	
    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(useBCH ? ARToolKitPlus::MARKER_ID_BCH : ARToolKitPlus::MARKER_ID_SIMPLE);
   
	
	udpConnectionRx.Create();
	udpConnectionRx.Bind(19802); //incomming data on my port # ...
	udpConnectionRx.SetNonBlocking(true);
	
	//fill our 3d vector with 20x20x20
	array3D.resize(20);
	for (int i = 0; i < 20; ++i) {
		array3D[i].resize(20);
		
		for (int j = 0; j < 20; ++j)
			array3D[i][j].resize(20);
	}
	Block b;
	
	for(int x=0; x < 20; x++){
		for(int y=0; y < 20; y++){
			for(int z=0; z < 20; z++){
				b.type = NONE;
				array3D[x][y][z] = b;
			}
		}
	}
	b.type = GRASS;
	array3D[5][5][5] = b;
	rotXAmt = 0;
	rotYAmt = 0;
	
	
}


//--------------------------------------------------------------
// starting:<-150><64><-124><2><2><2>
/* slice:<y:0><z:0><0:1><0:1>
 *
 */

void testApp::update(){
	grabber.grabFrame();
	if(grabber.isFrameNew()){
		
		//convert our camera frame to grayscale
		convert.setFromPixels(grabber.getPixels(), 640, 480);
		gray = convert;
		
		//find the marker and get back the confidence
		int markerId = tracker->calc(gray.getPixels());
		float conf = (float)tracker->getConfidence();
		
		if( conf > 0.0 ){
			bDraw = true;
		}else bDraw = false;
		
		printf("\n\nFound marker %d  (confidence %d%%)\n\nPose-Matrix:\n  ", markerId, (int(conf*100.0f)));
		
		//prints out the matrix - useful for debugging?
		for(int i=0; i<16; i++)
			printf("%.2f  %s", tracker->getModelViewMatrix()[i], (i%4==3)?"\n  " : "");
		
	}
	
	
	char udpMessage[100000];
	udpConnectionRx.Receive(udpMessage,100000);
	string s;
	s.assign(udpMessage);
	trim(s);
	if(s != ""){
		//cout<<"udpMessage-"<<s<<"-"<<endl;
		
		//process this shit
		processShit(s);
		
	}
	
	rotX += rotXAmt;
	rotY += rotYAmt;
	
	
}

void testApp::drawBlock(int x, int y, int z, int wx, int wy, int wz){
	glBegin(GL_QUADS);
	
	glVertex3f( x - wx,  y, z) ;
	glVertex3f( x,  y,  z);
	glVertex3f( x, y - wy,  z);
	glVertex3f( x - wx, y - wy, z);
	
	glVertex3f( x,  y,  z);
	glVertex3f( x,  y, z - wz);
	glVertex3f( x, y - wy, z - wz);
	glVertex3f( x, y - wy,  z);
	
	glVertex3f( x,  y, z - wz);
	glVertex3f(x - wx,  y, z - wz);
	glVertex3f(x - wx, y - wy, z - wz);
	glVertex3f( x, y - wy, z - wz);
	
	glVertex3f(x - wx,  y, z - wz);
	glVertex3f(x - wx,  y,  z);
	glVertex3f(x - wx, y - wy,  z);
	glVertex3f(x - wx, y - wy, z - wz);
	
	glVertex3f(x - wx,  y, z - wz);
	glVertex3f( x,  y, z - wz);
	glVertex3f( x,  y,  z);
	glVertex3f(x - wx,  y,  z );
	
	glVertex3f(x - wx, y - wy, z - wz);
	glVertex3f( x, y - wy, z - wz);
	glVertex3f( x, y - wy,  z);
	glVertex3f(x - wx, y - wy,  z);
	glEnd();
}


//--------------------------------------------------------------
void testApp::draw(){
	ofClear(0,0,0);
	ofSetColor(255,255,255);
	grabber.draw(0, 0);

	
	if(!bDraw){
		ofTranslate( ofGetWidth() / 2, ofGetHeight() / 2, -100);
	
		ofRotateX(rotX);
		ofRotateY(rotY);
	} else {
		glViewport(0, 0, 640, 480 );
		glMatrixMode( GL_PROJECTION );
		glLoadMatrixf(tracker->getProjectionMatrix());
		glMatrixMode( GL_MODELVIEW );
		glLoadMatrixf(tracker->getModelViewMatrix());
		
		//ofSetColor(0xFFFFFF);
		
	}		
	for(int x=0; x < 20; x++){
		for(int y=0; y < 20; y++){
			for(int z=0; z < 20; z++){
				Block b = array3D[x][y][z];
				if(b.type != NONE){
					switch (b.type ){
						case GRASS:
							
							ofSetColor(0,200,0);
							break;
						case COBBLE:
							ofSetColor(128,128,128);
							break;
						case LAVA:
							ofSetColor(255,0,0);
							break;
						case LAVA2:
							ofSetColor(255,0,0);
							break;
						case STONE:
							ofSetColor(200,200,200);
							break;
					}				
					drawBlock(x * 10,y * 10,z * 10, 10, 10,10);
				}
			}
		}
	}

}


//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){
	rotXAmt = ofMap(y - clickY, 0, 500, 0, 0.4f);
	rotYAmt = ofMap(x - clickX, 0, 500, 0, 0.4f);
	
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	clickX = x;
	clickY = y;
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	rotYAmt = 0;
	rotXAmt = 0;
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

void testApp::trim(string& str)
{
	string::size_type pos = str.find_last_not_of(' ');
	if(pos != string::npos) {
		str.erase(pos + 1);
		pos = str.find_first_not_of(' ');
		if(pos != string::npos) str.erase(0, pos);
	}
	else str.erase(str.begin(), str.end());
}


void testApp::processShit(const string& s){
	if(s.substr(0, 6) == "slice:"){					//<([a-z0-9]*):([0-9]*)>
		//cout << "slice: " << s << endl;
		//std::regex pattern("<([a-z0-9]*):([0-9]*)>");		
		RegularExpression re("<([a-z0-9]*):([0-9]*)>");
		RegularExpression::MatchVec results;
		string result;
		string first, second;
		int currentY, currentZ;
		int curCount = 0;
		//re.match(s, 0, matches);
		
		//vector<string> results;
		int num = re.match(s, 0, results);
		
		while(num != 0){
			
			
			first = s.substr(results[1].offset, results[1].length);
			second = s.substr(results[2].offset, results[2].length);	

			
			
			if(first == "y"){
				currentY = atoi(second.c_str());

			} else if (first == "z"){
				currentZ = atoi(second.c_str());
				//clear that row
				for(int x = 0; x < 20; x++){
					array3D[x][currentY][currentZ].type = NONE;
				}	
				curCount = 0;
			} else {
				int type = atoi(first.c_str());
				//cout << type << "," << endl;
				array3D[curCount][currentY][currentZ].type = (BlockType)type;
				curCount++;
			}
			
			
			num = re.match(s,results[0].offset + results[0].length , results);

		}
		

		
		
	} else if(s.substr(0, 4) == "del:"){			//<([-]?[0-9]*):([-]?[0-9]*):([-]?[0-9]*)>
	} else if(s.substr(0, 4) == "add:"){			//<([0-9]*)><([-]?[0-9]*):([-]?[0-9]*):([-]?[0-9]*)>
	} else if(s.substr(0, 7) == "player:"){			//<([0-9]*)><([-]?[0-9\\.]*):([-]?[0-9\\.]*):([-]?[0-9\\.]*)>
	} else if(s.substr(0, 9) == "starting:"){		//<([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)>
		
		
		
		
	}

}





