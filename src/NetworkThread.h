/*
 *  NetworkThread.h
 *  ArTest
 *
 *  Created by Tom Wyatt on 12/05/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef _THREADED_OBJECT
#define _THREADED_OBJECT

#include "ofMain.h"
#include "ofxThread.h"
#include "ofxNetwork.h"
#include "testApp.h"
class testApp;


class NetworkThread : public ofxThread{
public:	
	NetworkThread(testApp* parentIn);		
	void trim(string& str);	
	void start();
	void stop();
	void threadedFunction();
		
private:
	testApp* parent;
	//net stuff
	ofxUDPManager udpConnectionRx;	
	string rxMessage;
	string stringBuf;
	char udpMessage[100000];
	
	
	
};

#endif
