#include "ofMain.h"
#include "ofApp.h"

// Main entry point
int main() {
    ofSetupOpenGL(1024, 768, OF_WINDOW); // Set up the OpenGL context
    ofRunApp(new ofApp());              // Start the application
}
