#pragma once

#include <string>
#include <cmath>

#include <MessageBroker.h>
#include <Vector.pb.h>

class IMUVisualizer : public ofBaseApp{
private:
    ofVec3f _currentAccelerometer, _currentGyroscope, _currentMagnetometer, _currentAHRS;
    ofEasyCam easyCam;
    deque<ofVec3f> pathVertices;
    ofMesh pathLines;
    MaxBotMessages::MessageBroker _messageBroker;

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
        ofRotateY(_currentAHRS.y);
        ofRotateX(_currentAHRS.x);
        ofRotateZ(_currentAHRS.z);
        ofSetColor(255);
        ofDrawBox(64,32,128);
        ofDrawAxis(32);
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
        sx = 440;
        ofDrawBitmapString("AHRS",sx,sy);
        ofDrawBitmapString("  yaw:" + to_string(_currentAHRS.y),sx,sy+20);
        ofDrawBitmapString("pitch:" + to_string(_currentAHRS.x),sx,sy+40);
        ofDrawBitmapString(" roll:" + to_string(_currentAHRS.z),sx,sy+60);
    }

    double rad_to_deg(double r) {
        return r * 180/M_PI;
    }

    ofVec3f rad_to_deg(MaxBotMessages::Vector3 v) {
         return ofVec3f(rad_to_deg(v.x()), rad_to_deg(v.y()), rad_to_deg(v.z()));
    }

    ofQuaternion rad_to_deg(MaxBotMessages::Quaternion q) {
         return ofQuaternion(rad_to_deg(q.z()), rad_to_deg(q.x()), rad_to_deg(q.y()), rad_to_deg(q.w()));
    }

    ofVec3f translate(MaxBotMessages::Vector3 v) {
         return ofVec3f(v.x(), v.y(), v.z());
    }

    void UpdateAHRS(const std::string& message) {
        MaxBotMessages::QuaternionStamped qs;
        qs.ParseFromString(message);
        auto q = qs.mutable_quaternion();
        double sqw = q->w() * q->w();
        double sqx = q->x() * q->x();
        double sqy = q->y() * q->y();
        double sqz = q->z() * q->z();
        _currentAHRS.y = rad_to_deg(atan2(2.0 * (q->x() * q->y() + q->z() * q->w()), (sqx - sqy - sqz + sqw)));   //yaw
        _currentAHRS.x = rad_to_deg(asin(-2.0 * (q->x() * q->z() - q->y() * q->w())));                            //pitch
        _currentAHRS.z = rad_to_deg(atan2(2.0 * (q->y() * q->z() + q->x() * q->w()), (-sqx - sqy + sqz + sqw)));  //roll
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

    IMUVisualizer() : _messageBroker(1)  {
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
        //ofColor magenta = ofColor::fromHex(0xec008c);
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
