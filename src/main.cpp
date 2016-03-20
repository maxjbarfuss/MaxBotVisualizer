#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#include <ofMain.h>

#include "IMUVisualizer.hpp"

int main()
{
    ofSetupOpenGL(800, 600, OF_GAME_MODE);
	ofRunApp( new IMUVisualizer() );
}
