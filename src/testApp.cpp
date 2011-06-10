#include "testApp.h"
#include "ARToolKitPlus/TrackerMultiMarkerImpl.h"

static const int       width = 640	, height = 480, bpp = 1;
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
	//ofSetLogLevel(OF_LOG_VERBOSE);
	//grabber.listDevices();
	//grabber.setDeviceID(7);
	if(grabber.initGrabber(640, 480)){
		
	
		cout << "Using grabber" << endl;
		
	} else {
		cout << "MASSIVE FAIL" <<endl;
		
	}
	
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
	tracker = new ARToolKitPlus::TrackerMultiMarkerImpl<6,6,6, 1, 64>(width,height);
	
	tracker->setPixelFormat(ARToolKitPlus::PIXEL_FORMAT_LUM);	
	//markerboard_480-499.cfg
    if( !tracker->init( (const char *)ofToDataPath("bluedot.cfg").c_str(), (const char *)ofToDataPath("conf.cfg").c_str(), 1.0f, 1000.0f) )            // load std. ARToolKit camera file
	{
		printf("ERROR: init() failed\n");
		delete tracker;
		return;
	}
	// the marker in the BCH test image has a thin border...
    tracker->setBorderWidth(0.125f);	
    // set a threshold. alternatively we could also activate automatic thresholding
    //tracker->setThreshold(150);	
	tracker->activateAutoThreshold(true);
    // let's use lookup-table undistortion for high-speed
    // note: LUT only works with images up to 1024x1024
    tracker->setUndistortionMode(ARToolKitPlus::UNDIST_LUT);
	
    // RPP is more robust than ARToolKit's standard pose estimator
    tracker->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
	
	tracker->setImageProcessingMode(ARToolKitPlus::IMAGE_FULL_RES);
	
    // switch to simple ID based markers
    // use the tool in tools/IdPatGen to generate markers
    tracker->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
	
	
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
	array3D.resize(0);
	
	
	//create block face data and clear map 
		
	
	vList[0] = ofxVec3f(0.0f, 0.0f, -1.0f);
	vList[1] = ofxVec3f(-1.0f, 0.0f, -1.0f);
	vList[2] = ofxVec3f(-1.0f, 0.0f, 0.0f);
	vList[3] = ofxVec3f(0.0f, 0.0f, 0.0f);
	vList[4] = ofxVec3f(-1.0f, -1.0f, 0.0f);
	vList[5] = ofxVec3f(0.0f, -1.0f, 0.0f);
	vList[6] = ofxVec3f(0.0f, -1.0f, -1.0f);
	vList[7] = ofxVec3f(-1.0f, -1.0f, -1.0f);
	
	//vertex indices for faces
	const static int faceVals[6][4] = {
		{2, 1, 0, 3}, //top
		{5, 4, 2, 3}, //front
		{0, 6, 5, 3},//right
		{4, 7, 1, 2},//left
		{5, 6, 7, 5},//bottom
		{0, 1, 7, 6} //back
	};
		
	
	array3D.resize(0);
	//add some test blocks to play with when offline
	baseBlock.type = GRASS;
	baseBlock.textured = true;
	baseBlock.textureRef = 0;
	baseBlock.visMask = 0;
	baseBlock.position = ofxVec3f(0,0,0);
	baseBlock.transparent = false;
	for(int a = 0; a < 6; a++){
		for (int c = 0; c < 4; c++){
			baseBlock.faceList[a][c] = faceVals[a][c];
		}
	}
	for(int i = 0; i < 6; i++){
		calculateNormal(&baseBlock, i);
	}
	array3D.push_back(baseBlock);	
	baseBlock.position = ofxVec3f(1,0,0);
	array3D.push_back(baseBlock);	
	baseBlock.position = ofxVec3f(2,0,0);
	array3D.push_back(baseBlock);	
	baseBlock.position = ofxVec3f(3,0,0);
	array3D.push_back(baseBlock);	
	baseBlock.position = ofxVec3f(1,1,0);
	array3D.push_back(baseBlock);	
	baseBlock.position = ofxVec3f(1,0,1);

	startDrawPointer = 0;


	
	//testBlock = b;
	
	//run the face visibility and normal calculations
	updateVisibility();
	
	//dont draw the gui
	guiDraw = false;
	
	//scale and offset for map
	mapScale = 1.0f;
	offset.x = -100.0f;
	offset.y = 100.0f;
	
	scVal = 1.0f;
	
	//used as light colour (when lights eventually work)
	sceneWhiteLevel = ofColor(255,255,255);
	
	#ifdef SYPHON
	//start up syphon and set framerate
	mainOutputSyphonServer.setName("Minecraft");
	#endif
	
	ofSetFrameRate(60);
	
	//try and set up some lights
	
	light light = {
		{0,50,0,1},  //position (the final 1 means the light is positional)
		{1,1,1,1},    //diffuse
		{0,0,0,1},    //specular
		{0,0,0,1}     //ambient
    };
	
	light0 = light;
	light1 = light;
	light1.ambient[0] = 0.5f;
	light1.ambient[1] = 0.5f;
	light1.ambient[2] = 0.5f;
	light1.ambient[3] = 1.0f;
	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,0);  //sets lighting to one-sided
	glLightfv(GL_LIGHT0, GL_POSITION, light0.pos);
	doLights();
	
	
	//turn on backface culling
	glEnable(GL_CULL_FACE);
	lastTextureRef = -1;
	
}

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
	if(grabber.isFrameNew()){
		kinectImage.setFromPixels(grabber.getPixels(), 640, 480);
	}
	
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
		lastDetectTime = ofGetElapsedTimeMillis();
	}else {
		bDraw = false;
	}
		
		
	
	
}

void testApp::drawBlock(int x, int y, int z, int wx, int wy, int wz, Block *bType){
	
	glPushMatrix();
	glTranslatef(x,y,z);
	glScalef(wx,wy,wz);
	float mcolor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	//hmmmmm
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, mcolor);
	glMaterialfv(GL_FRONT, GL_SHININESS, mcolor);
	glFrontFace(GL_CW);
	
	if (lastTextureRef != bType->textureRef && bType->textured){
		textures[lastTextureRef].unbind();
		lastTextureRef = bType->textureRef;
		
	
		textures[bType->textureRef].bind();
	}

	glBegin(GL_QUADS);	
	for (int f = 0; f < 6; f++){
		int visFace = 1 << f;
		if(bType->visMask & visFace){
			ofxVec3f verts[4];
			for (int i = 0; i < 4; i++){
				verts[i] = vList[ bType->faceList[f][i] ];
			}
			ofxVec3f norm = bType->normals[f];
			glNormal3f(norm.x, norm.y, norm.z);
			glTexCoord2f(0.0,0.0); 	glVertex3f(verts[0].x, verts[0].y, verts[0].z);
			glTexCoord2f(0.0,1.0);	glVertex3f(verts[1].x, verts[1].y, verts[1].z);
			glTexCoord2f(1.0,1.0);	glVertex3f(verts[2].x, verts[2].y, verts[2].z);
			glTexCoord2f(1.0,0.0);	glVertex3f(verts[3].x, verts[3].y, verts[3].z);
		}
	}
	glEnd();
	

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
	bindFbo();
	glDisable(GL_DEPTH_TEST);
	
	kinectImage.draw(0, 0,800,600);
	glEnable (GL_DEPTH_TEST);
	
	//if(bDraw){
	
	
	//bindFbo();
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
	
	doLights();
	
	lastTextureRef = array3D[startDrawPointer].textureRef;
	textures[lastTextureRef].bind();
	
	for (int i = startDrawPointer; i < array3D.size(); i++){
		Block* b = &array3D[i];
		if(b->visMask != 0){				
			
			drawBlock(b->position.x * 10, b->position.y * 10,b->position.z * 10, 10, 10,10, b);
			
		}
		b = NULL;
		delete b;
	}
	textures[lastTextureRef].unbind();
	
	glMatrixMode( GL_PROJECTION );
	glPopMatrix();
	glMatrixMode( GL_MODELVIEW );		
	glPopMatrix();
	
	unbindFbo();
	glPopMatrix();
	
    glDisable(GL_LIGHTING);
	glDisable (GL_DEPTH_TEST);
	
	drawFbo();
		
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
	
#ifdef SYPHON
	mainOutputSyphonServer.publishFail(&thisWillNeverWork);
#endif
	
}

void testApp::doLights(){	
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0.diffuse);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light0.ambient);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light0.specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light0.pos);
	glLightf(GL_LIGHT0, GL_CONSTANT_ATTENUATION, 0.07f);
	//glLightf(GL_LIGHT0, GL_LINEAR_ATTENUATION, 0.2f);
	glLightf(GL_LIGHT0, GL_QUADRATIC_ATTENUATION, 0.0f);
	
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, light1.ambient);
	
}


//--------------------------------------------------------------
void testApp::keyPressed(int key){
	if(key == 'w'){
		scVal += 0.01;
	} else if(key == 's'){
		scVal -= 0.01;
	} else if (key == 'g'){
		guiDraw = !guiDraw;
	}
	cout << ofToString(scVal) << endl;
	mapScale = scVal;
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
	
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
	mx = x;
	my = y;
	
	light0.pos[0] = -mx;
	light0.pos[1] =  my;
	light0.pos[2] = 0;
	
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
	
	cout << "r: " << (int)r << " g: " << (int)g << " b: " << (int)b << endl;
	sceneWhiteLevel = ofColor(r,g,b);
	
	
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	
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
	char * results;
	char * cstr;
	//cout << "mess: " << s << endl;

	cstr = new char [s.size()+1];
	strcpy (cstr, s.c_str());
	results = strtok(cstr, ",");

	if(results[0]  == 's'){	
		//cout << "mess: " << results[0] <<endl;
		int currentY, currentZ;
		currentY = 0;
		currentZ = 0;
		int curCount = 0;
		int ct = 0;

		
		
		while (results != NULL){
			
			if(ct == 1){
				currentY = atoi(results);
				ct++;
			} else if (ct == 2){
				currentZ = atoi(results);

				
				curCount = 0;
				ct++;
			} else {
				int type = atoi(results);
				if((BlockType)type != NONE){
					setBlock(curCount,currentY,currentZ, type);
				}	
				curCount++;
				ct++;
			}
			results = strtok(NULL, ",");

			
		}
		
		
	//	updateVisibility();

		
	} else if(results[0]  == 'a'){
		int ct = 0;
		int currentX = -1;
		int currentY = -1;
		int currentZ = -1;
		int type = 0;
		while (results != NULL){
			
			if(ct == 1){
				type = atoi(results);
				cout << "add block id: " << type << endl;
				ct++;
			} else if (ct == 2){
				currentX = atoi(results);
				
				ct++;
			} else if (ct == 3){
				currentY = atoi(results);
				
				ct++;
			}else if (ct == 4){
				currentZ = atoi(results);
					
				ct++;
			} else {
				ct++;
			}
			results = strtok(NULL, ",");
		}
		if(currentX >= 0 && currentX < mapWidth && currentY >= 0 && currentY < mapHeight && currentZ >= 0 && currentZ < mapDepth){
			setBlock(currentX, currentY, currentZ, type);
			cout << "added" << endl;
		}
		//updateVisibility();

	} else if(results[0]  == 'd'){
		int ct = 0;
		int currentX = -1;
		int currentY = -1;
		int currentZ = -1;
		int type = 0;
		while (results != NULL){
			
			if (ct == 1){
				currentX = atoi(results);
				
				ct++;
			} else if (ct == 2){
				currentY = atoi(results);
				
				ct++;
			}else if (ct == 3){
				currentZ = atoi(results);
				
				ct++;
			} else {
				ct++;
			}
			results = strtok(NULL, ",");
		}
		if(currentX >= 0 && currentX < mapWidth && currentY >= 0 && currentY < mapHeight && currentZ >= 0 && currentZ < mapDepth){
			deleteBlock(currentX, currentY, currentZ);
			cout << "delete" << endl;
		}
		//updateVisibility();
	} else if(results[0]  == 'p'){
			int ct = 0;
			float currentX = -1;
			float currentY = -1;
			float currentZ = -1;
			int playerId = 0;
			while (results != NULL){
				
				if (ct == 1){
					//player id
					
					playerId = atoi(results);
					//cout << "player id: " << playerId << endl;
					
					ct++;
				} else if (ct == 2){
					//x
					p.xPos = strtod(results,NULL);
					//cout << currentX << endl;

					
					ct++;
				}else if (ct == 3){
					//y
					p.yPos = strtod(results,NULL);
					ct++;
				} else if (ct == 4){
					//z
					
					p.zPos = strtod(results,NULL);
					ct++;
				} else {
					
					ct++;
				}
				results = strtok(NULL, ",");
				
			}
			//cout << "pos" << endl;
		p.lastUpdateTime = ofGetElapsedTimeMillis();
		
	} else if(results[0]  == 'i'){
		//reinit map
		cout << "resetting map" << endl;
		array3D.resize(0);
		
	} else if(results[0]  == 'e'){
		cout << "visibiltiy test" << endl;
		updateVisibility();
	}
//	delete cstr;
//	delete results;
}
	
void testApp::deleteBlock(int x, int y, int z){
	Block comp;
	comp.position = ofxVec3f(x,y,z);
	for (int i = 0; i < array3D.size(); i++){
		
		if(array3D[i] == comp){
			//array3D.erase(i);
			break;
		}
	}
}

void testApp::setBlock(int x, int y, int z, int type){
	
	
	Block b;
	b = baseBlock;	
	
	//Block* b = &array3D[x][y][z];
	b.type = (BlockType)type;
	b.textured = false;
	b.position = ofxVec3f(x,y,z);
		
	switch ((BlockType)type ){
		case GRASS:
			b.textured = true;						
			b.textureRef = 0;
			break;
		case COBBLE:
			b.textured = true;
			b.textureRef = 1;
			break;
		case LAVA:
			b.textured = true;
			b.textureRef = 3;
			break;
		case LAVA2:
			b.textured = true;
			b.textureRef = 3;
			break;
		case STONE:
			b.textured = true;
			b.textureRef = 4;
			break;
		case DIRT:
			b.textured = true;
			b.textureRef = 2;
			break;
		case LOGS:
			b.textured = true;
			b.textureRef = 7;
			break;
		case LEAVES:
			b.textured = true;
			b.textureRef = 8;
			b.transparent = true;
			break;
		case TREE:
			b.textured = true;
			b.textureRef = 7;
			break;
		case SAND:
			b.textured = true;
			b.textureRef = 5;
			break;
	}
	
	
	array3D.push_back(b);
	
}

/*
 *		update visibility of faces and their normals
 */
void testApp::updateVisibility(){
	int x, y, z;
	int vmask = 0;
	for (int i = 0; i < array3D.size(); i++){
		Block* block = &array3D[i];
		x = block->position.x;
		y = block->position.y;
		z = block->position.z;
		vmask = 0;
		//for each block find out if there is one next to it
		
		if(!testBlock(x - 1, y, z)){
			vmask |= VIS_LEFT;	
		} 
		if(!testBlock(x + 1, y, z)){
			vmask |= VIS_RIGHT;				
			
		} 
		if(!testBlock(x, y - 1, z) ){
			vmask |= VIS_BOTTOM;
			
		} 
		if (!testBlock(x, y + 1, z)){
			vmask |= VIS_TOP;
		} 
		if(!testBlock(x, y, z - 1)){
			vmask |= VIS_BACK;
			
		}
		if (!testBlock(x, y, z + 1)){
			vmask |= VIS_FRONT;
			
		}
	
		block->visMask = vmask;
																			  
	}
	
	//sort the array by vmask, remove all values that are 0
	std::sort(array3D.begin(), array3D.end());
	
	
	//find the first 0 value and set the start pointer to it
	Block* b;
	for (int i = 0; i < array3D.size(); i++){
		b = &array3D[i];
		if(b->visMask != NONE){
			startDrawPointer = i;
			cout << "dp: " << startDrawPointer << endl;
			break;
		}
	}
	
	
	b = NULL;
	delete b;
	
}

// true = block present
// false = no block, set vis of this face
bool testApp::testBlock(int x, int y, int z){
	Block comp;
	comp.position = ofxVec3f(x,y,z);
	Block* b;
	for (int i = 0; i < array3D.size(); i++){
		b = &array3D[i];
		if(b->position == comp.position){
			
			b = NULL;
			delete b;
			return true;	

		}
	}
	b = NULL;
	delete b;
	return false;
}

void testApp::calculateNormal(Block* b, int faceId){
	
	
	ofxVec3f V = vList[b->faceList[faceId][3]] - vList[b->faceList[faceId][1]];
  	ofxVec3f U = vList[b->faceList[faceId][2]] - vList[b->faceList[faceId][1]];
	b->normals[faceId].x = (U.y * V.z) - (U.z * V.y);
	b->normals[faceId].y = (U.y * V.x) - (U.x * V.z);
	b->normals[faceId].z = (U.x * V.y) - (U.y * V.x);
	
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
	//glGenerateMipmapEXT(GL_TEXTURE_2D);
	
	//ofSetColor(255, 255, 255, 0);	
	//draw quad
	
	//ofSetColor(255, 255, 255);
	glPushMatrix();
	glTranslated(0, 0, 0);
	
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1, 1);
	glVertex3f(800, 0, 1);
	glTexCoord2f(1, 0);
	glVertex3f(800, 600, 1);
	glTexCoord2f(0, 0);
	glVertex3f(0, 600, 1);
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
	
#ifdef SYPHON
	thisWillNeverWork.bAllocated = true;
	thisWillNeverWork.textureID = mTextureID;
	thisWillNeverWork.width = 640;
	thisWillNeverWork.height = 480;
	thisWillNeverWork.bFlipTexture = true;
	thisWillNeverWork.textureTarget = GL_TEXTURE_2D;
#endif
	
}  


