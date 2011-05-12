#include "testApp.h"
#include "ARToolKitPlus/TrackerMultiMarkerImpl.h"

static const int       width = 640, height = 480, bpp = 1;
static   size_t        numPixels = width*height*bpp;
static    size_t        numBytesRead;
static   const char    *fName = "data/markerboard_480-499.raw";
static   const char    *tagName = "data/patt.hiro";
ARToolKitPlus::TrackerMultiMarker *tracker;


//#define KINECT 0


//--------------------------------------------------------------
void testApp::setup(){
	ofDisableArbTex();
	ofSetTextureWrap();
	//configure FBO
	initFrameBuffer();
	
	//camera config
#ifdef KINECT
	
	context.setupUsingXMLFile();
	image.setup(&context);
	depth.setup(&context);
	xn::DepthGenerator& depthGen = depth.getXnDepthGenerator();
	xn::ImageGenerator& imageGen = image.getXnImageGenerator();
	
	
	XnStatus ret = xnSetViewPoint(depthGen, imageGen);
	cout << "Using kinect" << endl;

#else
	grabber.initGrabber(640, 480);
	cout << "Using grabber" << endl;
	
#endif
	
	
	convert.allocate(640, 480);				//conversion of camera image to grayscale
	gray.allocate(640, 480);				//grayscale tracking image
	kinectImage.allocate(640, 480);			//image from kinect
	kinectDepthImage.allocate(640, 480);	//Depth image from kinect
	finalMaskImage.allocate(640, 480);;		//final composition mask
	sceneDepthImage.allocate(640, 480);;	//scenes depthmap image
	finalImage.allocate(640, 480);
	sceneImage.allocate(640,480);
	
	pixelBuf = new unsigned char[640*480];			//temp buffer for kinect depth strea
	colorPixelBuf = new unsigned char[640*480*3];	//temp buffer for kinect image
	sceneDepthBuf = new unsigned short[640 * 480];	//depth buffer from our rendered scene
	kinectDepthBuf = new unsigned short[640 * 480];	//camera image buffer
	finalBuf = new unsigned char[640 * 480];		//final mask buffer
	finalImageBuf = new unsigned char[640 * 480 * 3];	//final Image buffer
	sceneBuffer = new unsigned char[640 * 480 * 3];		//copy of the scene in the FBO	
	
	bDraw = false;
	
	//tracker = new ARToolKitPlus::TrackerSingleMarkerImpl<6,6,6, 1, 8>(width,height);
	tracker = new ARToolKitPlus::TrackerMultiMarkerImpl<6,6,6, 1, 8>(width,height);
	
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);	
	
    if( !tracker->init( (const char *)ofToDataPath("LogitechPro4000.dat").c_str(), (const char *)ofToDataPath("markerboard_480-499.cfg").c_str(), 1.0f, 1000.0f) )            // load std. ARToolKit camera file
	{
		printf("ERROR: init() failed\n");
		delete tracker;
		return;
	}
	// the marker in the BCH test image has a thin border...
    tracker->setBorderWidth(0.125f);	
    // set a threshold. alternatively we could also activate automatic thresholding
    tracker->setThreshold(150);	
    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
	
    // RPP is more robust than ARToolKit's standard pose estimator
    tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
	
    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
	/*
	udpConnectionRx.Create();
	udpConnectionRx.Bind(19802); //incomming data on my port # ...
	udpConnectionRx.SetNonBlocking(true);
	*/
	netThread = new NetworkThread(this);
	netThread->start();
	
	//load textures
	ofDisableArbTex();
	ofSetTextureWrap();
	textures[0].loadImage("grass.png");
	textures[1].loadImage("cobble.png");	
	textures[2].loadImage("dirt.png");	
	textures[3].loadImage("lava.png");	
	textures[4].loadImage("rock.png");	
	textures[5].loadImage("sand.png");	
	textures[6].loadImage("snow.png");	
	textures[7].loadImage("tree.png");
	textures[8].loadImage("leaves.png");
	
	
	mapWidth = 20;
	mapHeight = 20;
	mapDepth = 20;
	mapLocked = false;
	
	//fill our 3d vector with 20x20x20
	mapLocked = true;
	array3D.resize(mapWidth);
	for (int i = 0; i < mapWidth; ++i) {
		array3D[i].resize(mapHeight);
		
		for (int j = 0; j < mapHeight; ++j)
			array3D[i][j].resize(mapDepth);
	}
	Block b;
	
	for(int x=0; x < mapWidth; x++){
		for(int y=0; y < mapHeight; y++){
			for(int z=0; z < mapDepth; z++){
				b.type = NONE;
				b.textured = false;

				array3D[x][y][z] = b;
			}
		}
	}
	b.type = GRASS;
	b.textured = true;
	b.textureRef = 0;
	array3D[5][5][5] = b;
	rotXAmt = 0;
	rotYAmt = 0;
	
	//fbo.allocate(640,480, GL_RGBA, 1);
	
	
	guiDraw = true;
	mapScale = 1.0f;
	offset.x = -100.0f;
	offset.y = 100.0f;
	
	scVal = 1.0f;
	
	sceneWhiteLevel = ofColor(255,255,255);
}


//--------------------------------------------------------------
// starting:<-150><64><-124><2><2><2>
/* slice:<y:0><z:0><0:1><0:1>
 *
 */

void testApp::stop(){
	netThread->stop();
	delete netThread;
	delete tracker;

#ifdef KINECT
	
#else
	grabber.close();
#endif
}

void testApp::update(){
#ifdef KINECT
	
	
	context.update();
	xn::DepthGenerator& depthGen = depth.getXnDepthGenerator();
	xn::Context& xnCont = context.getXnContext();
	xn::ImageGenerator& imageGen = image.getXnImageGenerator();
	const XnDepthPixel* pDepthMap = depthGen.GetDepthMap();
	const XnRGB24Pixel* pImageMap = imageGen.GetRGB24ImageMap();
	int ct=0;
	//get kinect data into open cv
	for (int y=0; y<XN_VGA_Y_RES ; y++){
		for(int x=0;x<XN_VGA_X_RES ;x++){
			
			colorPixelBuf[ct] = pImageMap[y * XN_VGA_X_RES + x].nRed;
			colorPixelBuf[ct + 1] =  pImageMap[y * XN_VGA_X_RES + x].nGreen;
			colorPixelBuf[ct + 2] =  pImageMap[y * XN_VGA_X_RES + x].nBlue;
			
			pixelBuf[y * XN_VGA_X_RES + x] = pDepthMap[y * XN_VGA_X_RES + x] / 8;
			kinectDepthBuf[y * XN_VGA_X_RES + x] = pDepthMap[y * XN_VGA_X_RES +  x];
			ct += 3;
		}
    }
	kinectImage.setFromPixels(colorPixelBuf, 640, 480);
	kinectDepthImage.setFromPixels(pixelBuf, 640, 480);
#else
	grabber.grabFrame();
	kinectImage.setFromPixels(grabber.getPixels(), 640, 480);
	
#endif
	
	
	
	//convert our camera frame to grayscale
	convert.setFromPixels(kinectImage.getPixels(), 640, 480);
	gray = convert;
	
	//find the marker and get back the confidence
	int markerId = tracker->calc(gray.getPixels());
//	float conf = (float)tracker->getConfidence();
	int conf = tracker->getNumDetectedMarkers();
	//cout << conf << endl;
	if( conf > 0 ){
		bDraw = true;
	}else bDraw = false;
		
	/*
	
	char udpMessage[100000];
	udpConnectionRx.Receive(udpMessage,100000);
	string s;
	s.assign(udpMessage);
	trim(s);
	if(s != ""){
		//cout<<"udpMessage-"<<s<<"-"<<endl;
		
		//process this shit
		processShit(s);
		
	}*/
	
	rotX += rotXAmt;
	rotY += rotYAmt;
	
	
	
	
}

void testApp::drawBlock(int x, int y, int z, int wx, int wy, int wz, Block *bType){
	//fbo.begin();
	glPushMatrix();
	glTranslatef(x,y,z);
	glScalef(wx,wy,wz);
	//Block block = *bType;
	if(bType->textured){
		textures[bType->textureRef].bind();
	}
	//grassImage.getTextureReference().bind();
	
	glBegin(GL_QUADS);
	/*      This is the top face*/
	glTexCoord2f(0.0,0.0); 	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0,1.0);	glVertex3f(0.0f, 0.0f, -1.0f);
	glTexCoord2f(1.0,1.0);	glVertex3f(-1.0f, 0.0f, -1.0f);
	glTexCoord2f(1.0,0.0);	glVertex3f(-1.0f, 0.0f, 0.0f);
	
	/*      This is the front face*/
	glTexCoord2f(0.0,0.0);	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0,1.0);	glVertex3f(-1.0f, 0.0f, 0.0f);
	glTexCoord2f(1.0,1.0);	glVertex3f(-1.0f, -1.0f, 0.0f);
	glTexCoord2f(1.0,0.0);	glVertex3f(0.0f, -1.0f, 0.0f);
	
	/*      This is the right face*/
	glTexCoord2f(0.0,0.0);	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0,1.0);	glVertex3f(0.0f, -1.0f, 0.0f);
	glTexCoord2f(1.0,1.0);	glVertex3f(0.0f, -1.0f, -1.0f);
	glTexCoord2f(1.0,0.0);	glVertex3f(0.0f, 0.0f, -1.0f);
	
	/*      This is the left face*/
	glTexCoord2f(0.0,0.0);	glVertex3f(-1.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0,1.0);	glVertex3f(-1.0f, 0.0f, -1.0f);
	glTexCoord2f(1.0,1.0);	glVertex3f(-1.0f, -1.0f, -1.0f);
	glTexCoord2f(1.0,0.0);	glVertex3f(-1.0f, -1.0f, 0.0f);
	
	/*      This is the bottom face*/
	glTexCoord2f(0.0,0.0);	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0,1.0);	glVertex3f(0.0f, -1.0f, -1.0f);
	glTexCoord2f(1.0,1.0);	glVertex3f(-1.0f, -1.0f, -1.0f);
	glTexCoord2f(1.0,0.0);	glVertex3f(-1.0f, -1.0f, 0.0f);
	
	/*      This is the back face*/
	glTexCoord2f(0.0,0.0);	glVertex3f(0.0f, 0.0f, 0.0f);
	glTexCoord2f(0.0,1.0);	glVertex3f(-1.0f, 0.0f, -1.0f);
	glTexCoord2f(1.0,1.0);	glVertex3f(-1.0f, -1.0f, -1.0f);
	glTexCoord2f(1.0,0.0);	glVertex3f(0.0f, -1.0f, -1.0f);
	
	glEnd();
	if(bType->textured){
		textures[bType->textureRef].unbind();
	}
	//grassImage.getTextureReference().unbind();
	
	
	glPopMatrix();
	
	
}


//--------------------------------------------------------------
void testApp::draw(){
	glEnable (GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClearDepth(1.0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);
	
	glPushMatrix();
	
	if(bDraw){
		bindFbo();
		glMatrixMode( GL_PROJECTION );
		glPushMatrix();
		glLoadMatrixf(tracker->getProjectionMatrix());
		glMatrixMode( GL_MODELVIEW );		
		glPushMatrix();
		glLoadMatrixf(tracker->getModelViewMatrix());
				
		

		glTranslatef(offset.x,offset.y,0);
		glRotatef(90, 1, 0, 0);
		glScalef(mapScale, mapScale, mapScale);
		
		//x
		glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3f(0,0,0);
		glVertex3f(300,0,0);
		glEnd();
		
		//y
		glBegin(GL_LINES);
		glColor3f(0, 1, 0);
		glVertex3f(0,0,0);
		glVertex3f(0,300,0);
		glEnd();
		
		//z
		glBegin(GL_LINES);
		glColor3f(0, 0, 1);
		glVertex3f(0,0,0);
		glVertex3f(0,0,300);
		glEnd();
		glColor3f(1,1,1);
		
		for(int x=0; x < mapWidth; x++){
			for(int y=0; y < mapHeight; y++){
				for(int z=0; z < mapDepth; z++){
					Block b = array3D[x][y][z];
					if(b.type != NONE && testVisibility(x,y,z)){
						
						
						drawBlock(x * 10,y * 10,z * 10, 10, 10,10, &b);
					}
				}
			}
		}
		//get our depth buffer data
		glReadPixels(0, 0, 640, 480, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, sceneDepthBuf);
		//glReadPixels(0, 0, 640, 480, GL_RGB, GL_UNSIGNED_BYTE, sceneBuffer);
		glMatrixMode( GL_PROJECTION );
		glPopMatrix();
		glMatrixMode( GL_MODELVIEW );		
		glPopMatrix();
		unbindFbo();
		
	}
	glPopMatrix();
	
	glDisable (GL_DEPTH_TEST);
	ofEnableAlphaBlending();
	kinectImage.draw(0, 0);	
	
	if(bDraw){
		ofSetColor(sceneWhiteLevel);
		drawFbo();
		ofSetColor(255,255,255);
				
	}	
	if(guiDraw){
		
		kinectDepthImage.draw(0,0,160,120);	
		
		sceneDepthImage.draw(0,120,160,120);
		finalMaskImage.draw(0,240, 160,120);
		ofDrawBitmapString(ofToString(scVal), 300,20);

	}	
	
	
	
	
	
	
#ifdef KINECT
	int ct = 0;
	for(int p = 0; p < 640 * 480 ; p++){
		unsigned short kinectVal = kinectDepthBuf[p];
		unsigned short sceneVal = USHRT_MAX - sceneDepthBuf[p];
		
		if(kinectVal > sceneVal){
			finalBuf[p] = 255;
		} else {
			finalBuf[p] = 0;
		}
		
	}
	//finalImage.setFromPixels(finalImageBuf, 640, 480);
	//glColor3f(1, 1, 1);
	finalMaskImage.setFromPixels(finalBuf, 640, 480);

#endif

	
}


//--------------------------------------------------------------
void testApp::keyPressed(int key){
	if(key == 'w'){
		scVal += 5;
	} else if(key == 's'){
		scVal -= 5;
	} else if (key == 'g'){
		guiDraw = !guiDraw;
	}
	
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
	
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
	mx = x;
	my = y;
	
	//offset.x = -mx;
	//offset.y = my;
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
	cout << mx << " " << my << endl;
	unsigned char * pix;
	pix = kinectImage.getPixels();
	unsigned char r = pix[x + y * 640];
	unsigned char g = pix[x + y * 640 + 1];
	unsigned char b = pix[x + y * 640 + 2];
	
	cout << "r: " << r << " g: " << g << " b: " << b << endl;
	sceneWhiteLevel = ofColor(r,g,b);
	
	
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
		
		char * results;
		char * cstr;
		
		string first, second;
		int currentY, currentZ;
		currentY = 0;
		currentZ = 0;
		int curCount = 0;
		cstr = new char [s.size()+1];
		strcpy (cstr, s.c_str());
		results = strtok(cstr, "<>");
		if(results == "slice"){
			results = strtok(NULL, "<>");			
		}
		
		
		while (results != NULL){
			
			/*			
			 first = s.substr(results[1].offset, results[1].length);
			 second = s.substr(results[2].offset, results[2].length);	
			 */
			string res;
			res.assign(results);
			int splitPoint = res.find(":");
			if(splitPoint != string::npos && splitPoint != res.length()){
				first = res.substr(0,splitPoint);
				
				second = res.substr(splitPoint + 1, s.length() - splitPoint);
				//cout  << "first: " << first ;
				//cout  << " second: " << second << endl;
				
				if(first == "y"){
					currentY = atoi(second.c_str());
					
				} else if (first == "z"){
					currentZ = atoi(second.c_str());
					//clear that row
					for(int x = 0; x < 20; x++){
						array3D[x][currentY][currentZ].type = NONE;
					}	
				//	cout << "HOT DOGGETY" << endl;
					curCount = 0;
				} else {
					int type = atoi(first.c_str());
					//cout << "type: " << type << "," << endl;
				//	cout << "added block id: " << type << " at x: " << curCount << " " << currentY << " " << currentZ << endl;
					array3D[curCount][currentY][currentZ].type = (BlockType)type;
					array3D[curCount][currentY][currentZ].textured = false;						
					
					switch ((BlockType)type ){
						case GRASS:
							array3D[curCount][currentY][currentZ].textured = true;						
							array3D[curCount][currentY][currentZ].textureRef = 0;
							break;
						case COBBLE:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 1;
							break;
						case LAVA:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 3;
							break;
						case LAVA2:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 3;
							break;
						case STONE:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 4;
							break;
						case DIRT:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 2;
							break;
						case LOGS:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 7;
							break;
						case LEAVES:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 8;
							break;
						case TREE:
							array3D[curCount][currentY][currentZ].textured = true;
							array3D[curCount][currentY][currentZ].textureRef = 7;
							break;
					}
					curCount++;
				}
				
			} 
		
			
			
			
			results = strtok(NULL, "<>");
			
		}
		
		
		
		
	} else if(s.substr(0, 4) == "del:"){			//<([-]?[0-9]*):([-]?[0-9]*):([-]?[0-9]*)>
	} else if(s.substr(0, 4) == "add:"){			//<([0-9]*)><([-]?[0-9]*):([-]?[0-9]*):([-]?[0-9]*)>
	} else if(s.substr(0, 7) == "player:"){			//<([0-9]*)><([-]?[0-9\\.]*):([-]?[0-9\\.]*):([-]?[0-9\\.]*)>
		//get the player pos, this should happen every 50ms
		
		
	} else if(s.substr(0, 9) == "starting:"){		//<([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)><([-]?[0-9]*)>
		
		
		
		
	}
	
}

//void testApp::resizeArray(int wx, int wy, int wz){

bool testApp::testVisibility(int x, int y, int z){
	if(x <= 0 || x >= mapWidth - 1 ||  y >= mapHeight - 1 || z <= 0 || z >= mapDepth - 1){
		return true;
	} else if(y <= 0){
		return false;
	} else {
		BlockType blockTypes[2] = {NONE, SNOW};
		for(int i = 0; i < 2; i++){
			
			
			if(array3D[x - 1][y][z].type == blockTypes[i] || array3D[x + 1][y][z].type  == blockTypes[i]){
				return true;
			}
			if(array3D[x][y - 1][z].type == blockTypes[i] || array3D[x][y + 1][z].type == blockTypes[i]){
				return true;
			}
			if(array3D[x][y][z - 1].type == blockTypes[i] || array3D[x][y][z + 1].type == blockTypes[i]){
				return true;
			}
		}
	}
	return false;
	
	
}

//FBO config

void testApp::bindFbo(){
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mFBOID);	
	glPushAttrib(GL_VIEWPORT_BIT | GL_ENABLE_BIT);
	glEnable (GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	

}

void testApp::unbindFbo(){
	glPopAttrib();
	
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
}	

void testApp::drawFbo(){
	glEnable(GL_TEXTURE_2D);
		
	//bind texture
	glBindTexture(GL_TEXTURE_2D, mTextureID);
	glGenerateMipmapEXT(GL_TEXTURE_2D);
	
	//ofSetColor(255, 255, 255, 0);	
	//draw quad
	
	//ofSetColor(255, 255, 255);
	glPushMatrix();
	glTranslated(0, 0, 0);
	
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1, 1);
	glVertex3f(640, 0, 1);
	glTexCoord2f(1, 0);
	glVertex3f(640, 480, 1);
	glTexCoord2f(0, 0);
	glVertex3f(0, 480, 1);
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);
	glPopMatrix();
}	

void testApp::initFrameBufferDepthBuffer() {  
	
	glGenRenderbuffersEXT(1, &mDepthID); // Generate one render buffer and store the ID in mDepthID  
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, mDepthID); // Bind the mDepthID render buffer  
	
	glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, 640, 480); // Set the render buffer storage to be a depth component, with a width and height of the window  
	
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, mDepthID); // Set the render buffer of this buffer to the depth buffer  
	
	glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0); // Unbind the render buffer  
}  

void testApp::initFrameBufferTexture() {  
	glGenTextures(1, &mTextureID); // Generate one texture  
	glBindTexture(GL_TEXTURE_2D, mTextureID); // Bind the texture mFBOID  
	
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 640, 480, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL); // Create a standard texture with the width and height of our window  
	
	// Setup the basic texture parameters  
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);  
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);  
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);  
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);  
	
	// Unbind the texture  
	glBindTexture(GL_TEXTURE_2D, 0);  
}  

void testApp::initFrameBuffer() {  
	initFrameBufferDepthBuffer(); // Initialize our frame buffer depth buffer  
	
	initFrameBufferTexture(); // Initialize our frame buffer texture  
	
	glGenFramebuffersEXT(1, &mFBOID); // Generate one frame buffer and store the ID in fbo  
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, mFBOID); // Bind our frame buffer  
	
	glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, mTextureID, 0); // Attach the texture mFBOID to the color buffer in our frame buffer  
	
	glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, mDepthID); // Attach the depth buffer mDepthID to our frame buffer  
	
	GLenum status = glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT); // Check that status of our generated frame buffer  
	
	if (status != GL_FRAMEBUFFER_COMPLETE_EXT) // If the frame buffer does not report back as complete  
	{  
		std::cout << "Couldn't create frame buffer" << std::endl; // Output an error to the console  
		//exit(0); // Exit the application  
	}  else {
		cout << "BITCHIN " <<endl;
	}
	
	glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); // Unbind our frame buffer  
}  


