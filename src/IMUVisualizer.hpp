#pragma once

#include <string>
#include <cmath>

#include <MessageBroker.h>
#include <Vector3.pb.h>

class IMUVisualizer : public ofBaseApp{
private:
    ofVec3f                         _currentAHRS, _currentAccelerometer, _currentGyroscope, _currentMagnetometer;
    ofEasyCam                       easyCam;
    deque<ofVec3f>                  pathVertices;
    ofMesh                          pathLines;
    MaxBotMessages::MessageBroker   _messageBroker;

	void drawIMU(ofColor pathColor, ofColor heightColor) {
        // draw the path of the box
        ofSetLineWidth(2);
        ofSetColor(pathColor);
        //pathLines.draw();
        // draw a line connecting the box to the grid
        //ofSetColor(heightColor);
        //ofLine(current.x, current.y, current.z, current.x, 0, current.z);
        // translate and rotate to the current position and orientation
        //ofTranslate(current.x, current.y, current.z);
        rotateTo(_currentAHRS);
        ofSetColor(255);
        ofDrawBox(64,32,128);
        ofDrawAxis(32);
    }

    void rotateTo(ofVec3f normal) {
        auto x = normal.x;
        auto y = normal.y;
        auto z = normal.z;
        ofRotate(z, 0, normal.y, 0);
        ofRotate(y, normal.x, 0, 0);
        ofRotate(x, 0, 0, -normal.z);
    }

    void drawLabels()
    {
        int sx = 20, sy = 20;
        ofDrawBitmapString("accelerometer",sx,sy);
        ofDrawBitmapString("x:" + to_string(_currentAccelerometer.x),sx,sy+20);
        ofDrawBitmapString("y:" + to_string(_currentAccelerometer.y),sx,sy+40);
        ofDrawBitmapString("z:" + to_string(_currentAccelerometer.z),sx,sy+60);
        sx = 150;
        ofDrawBitmapString("gyroscope",sx,sy);
        ofDrawBitmapString("x:" + to_string(_currentGyroscope.x),sx,sy+20);
        ofDrawBitmapString("y:" + to_string(_currentGyroscope.y),sx,sy+40);
        ofDrawBitmapString("z:" + to_string(_currentGyroscope.z),sx,sy+60);
        sx = 320;
        ofDrawBitmapString("magnetometer",sx,sy);
        ofDrawBitmapString("x:" + to_string(_currentMagnetometer.x),sx,sy+20);
        ofDrawBitmapString("y:" + to_string(_currentMagnetometer.y),sx,sy+40);
        ofDrawBitmapString("z:" + to_string(_currentMagnetometer.z),sx,sy+60);
        sx = 450;
        ofDrawBitmapString("AHRS",sx,sy);
        ofDrawBitmapString("x:" + to_string(_currentAHRS.x),sx,sy+20);
        ofDrawBitmapString("y:" + to_string(_currentAHRS.y),sx,sy+40);
        ofDrawBitmapString("z:" + to_string(_currentAHRS.z),sx,sy+60);
    }

    double rad_to_deg(double r) {
        if (r >= 0)
            return r * 180/M_PI;
        else
            return (360 + r * 180 / M_PI);
    }

    ofVec3f rad_to_deg(MaxBotMessages::Vector3 v) {
         return ofVec3f(rad_to_deg(v.x()), rad_to_deg(v.y()), rad_to_deg(v.z()));
    }

    ofVec3f translate(MaxBotMessages::Vector3 v) {
         return ofVec3f(v.x(), v.y(), v.z());
    }

    void UpdateAHRS(const std::string& message) {
        MaxBotMessages::Vector3Stamped v;
        v.ParseFromString(message);
        _currentAHRS = rad_to_deg(v.vector());
    }

    void UpdateGyroscope(const std::string& message) {
        MaxBotMessages::Vector3Stamped v;
        v.ParseFromString(message);
        _currentGyroscope = rad_to_deg(v.vector());
    }

    void UpdateAccelerometer(const std::string& message) {
        MaxBotMessages::Vector3Stamped v;
        v.ParseFromString(message);
        _currentAccelerometer = translate(v.vector());
    }

    void UpdateMagnetometer(const std::string& message) {
        MaxBotMessages::Vector3Stamped v;
        v.ParseFromString(message);
        _currentMagnetometer = translate(v.vector());
    }

public:

    IMUVisualizer() : _messageBroker("AHRS",1)  {
        _messageBroker.Subscribe("AHRS", [&](std::string s){ UpdateAHRS(s); });
        _messageBroker.Subscribe("GYRO", [&](std::string s){ UpdateGyroscope(s); });
        _messageBroker.Subscribe("ACEL", [&](std::string s){ UpdateAccelerometer(s); });
        _messageBroker.Subscribe("MAGN", [&](std::string s){ UpdateMagnetometer(s); });
    }

    void setup() {
        ofSetVerticalSync(true);
        pathLines.setMode(OF_PRIMITIVE_LINE_STRIP);
    }

    void update(){
        _messageBroker.DoWork();
    }

	void draw() {
	    ofColor cyan = ofColor::fromHex(0x00abec);
        ofColor magenta = ofColor::fromHex(0xec008c);
        ofColor yellow = ofColor::fromHex(0xffee00);
        ofColor gray = ofColor::fromHex(0x313141);
        ofBackgroundGradient(gray * .6, gray * .4);
        ofNoFill();
        easyCam.begin();
        ofRotateX(15);
        ofSetColor(0);
        ofDrawGrid(500, 10, false, false, true, false);
        drawIMU(cyan, yellow);
        easyCam.end();
        drawLabels();
	}

    void keyPressed(int key) {};
    void keyReleased(int key) {};
    void mouseMoved(int x, int y) {};
    void mouseDragged(int x, int y, int button) {};
    void mousePressed(int x, int y, int button) {};
    void mouseReleased(int x, int y, int button) {};
    void windowResized(int w, int h) {};
    void dragEvent(ofDragInfo dragInfo) {};
    void gotMessage(ofMessage msg) {};
};
