#include "ofApp.h"

int pw = 640;
int ph = 480;

//--------------------------------------------------------------
void ofApp::setup() {
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();
	kinect.initInfraredSource();
	kinect.initBodySource();
	kinect.initBodyIndexSource();

	ofSetWindowShape(pw * 2, ph * 2);

}

//--------------------------------------------------------------
void ofApp::update(){
	kinect.update();

	//--
	//Getting joint positions (skeleton tracking)
	//--
	//
	{
		auto bodies = kinect.getBodySource()->getBodies();
		for (auto body : bodies) {
			for (auto joint : body.joints) {
				//now do something with the joints
			}
		}
	}
	//
	//--



	//--
	//Getting bones (connected joints)
	//--
	//
	{
		// Note that for this we need a reference of which joints are connected to each other.
		// We call this the 'boneAtlas', and you can ask for a reference to this atlas whenever you like
		auto bodies = kinect.getBodySource()->getBodies();
		auto boneAtlas = ofxKinectForWindows2::Data::Body::getBonesAtlas();

		for (auto body : bodies) {
			for (auto bone : boneAtlas) {
				auto firstJointInBone = body.joints[bone.first];
				auto secondJointInBone = body.joints[bone.second];

				//now do something with the joints  
			}
		}
	}
	//
	//--
}

//--------------------------------------------------------------
void ofApp::draw(){

	kinect.getDepthSource()->draw(0, 0, pw, ph);  // note that the depth texture is RAW so may appear dark
	
	// Color is at 1920x1080 instead of 512x424 so we should fix aspect ratio
	float colorHeight = pw * (kinect.getColorSource()->getHeight() / kinect.getColorSource()->getWidth());
	float colorTop = (ph - colorHeight) / 2.0;

	kinect.getColorSource()->draw(pw, 0 + colorTop, pw, colorHeight);
	kinect.getBodySource()->drawProjected(pw, 0 + colorTop, pw, colorHeight);
	
	kinect.getInfraredSource()->draw(0, ph, pw, ph);
	
	kinect.getBodyIndexSource()->draw(pw, ph, pw, ph);
	kinect.getBodySource()->drawProjected(pw, ph, pw, ph, ofxKFW2::ProjectionCoordinates::DepthCamera);

	ofDrawBitmapString(ofGetFrameRate(), 5, 15);

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

	pw = w / 2;
	ph = h / 2;


}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
