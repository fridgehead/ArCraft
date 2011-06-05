/*
 *  NetworkThread.cpp
 *  ArTest
 *
 *  Created by Tom Wyatt on 12/05/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "NetworkThread.h"

NetworkThread::NetworkThread(testApp *parentIn){
	cout << "created net thread " << endl;
	parent = parentIn;
	udpConnectionRx.Create();
	udpConnectionRx.Bind(19802); //incomming data on my port # ...
	udpConnectionRx.SetNonBlocking(true);
}

void NetworkThread::start(){
	startThread(true, false);
}

void NetworkThread::stop(){
	udpConnectionRx.Close();
	stopThread();
}

void NetworkThread::threadedFunction(){
	while(isThreadRunning() != 0){
		
		udpConnectionRx.Receive(udpMessage,100000);
		stringBuf.assign(udpMessage);
		trim(stringBuf);
		if(stringBuf != ""){
			lock();
			//cout<<"!!!   udpMessage-"<<stringBuf<<"-"<<endl;
			
			//process this shit
			if(parent){
				parent->processShit(stringBuf);
			} else {
				ofLog(OF_LOG_WARNING, "null parent pointer in network receive");
			}
			unlock();
		}
		
	}	
}



void NetworkThread::trim(string& str)
{
	string::size_type pos = str.find_last_not_of(' ');
	if(pos != string::npos) {
		str.erase(pos + 1);
		pos = str.find_first_not_of(' ');
		if(pos != string::npos) str.erase(0, pos);
	}
	else 
	{
		str.erase(str.begin(), str.end());
	}
}

