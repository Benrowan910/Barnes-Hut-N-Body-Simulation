#include "ofApp.h"


float lastTime = 0.0f;
float fps = 0.0f;

// Insert body into quadtree node
void Node::insert(Body* b) {
    if (!children[0]) {
        if (!body) {
            body = b;
            totalMass = b->mass;
            centerOfMass = b->position;
        }
        else {
            subdivide();
            Body* existingBody = body;
            body = nullptr;

            // Reinsert existing body
            bool insertedExisting = false;
            for (int i = 0; i < 4; ++i) {
                if (children[i]->contains(*existingBody)) {
                    children[i]->insert(existingBody);
                    insertedExisting = true;
                    break;
                }
            }

            // Insert new body
            bool insertedNew = false;
            for (int i = 0; i < 4; ++i) {
                if (children[i]->contains(*b)) {
                    children[i]->insert(b);
                    insertedNew = true;
                    break;
                }
            }

            // Update mass and COM from children
            totalMass = 0;
            Vec2 tempCOM(0, 0);
            for (int i = 0; i < 4; ++i) {
                totalMass += children[i]->totalMass;
                tempCOM += children[i]->centerOfMass * children[i]->totalMass;
            }
            centerOfMass = tempCOM / totalMass;
        }
    }
    else {
        // Insert into appropriate child
        for (int i = 0; i < 4; ++i) {
            if (children[i]->contains(*b)) {
                children[i]->insert(b);
                break;
            }
        }

        // Update mass and COM from children
        totalMass = 0;
        Vec2 tempCOM(0, 0);
        for (int i = 0; i < 4; ++i) {
            totalMass += children[i]->totalMass;
            tempCOM += children[i]->centerOfMass * children[i]->totalMass;
        }
        centerOfMass = tempCOM / totalMass;
    }
}

// Subdivide quadtree node into four children
void Node::subdivide() {
    float halfSize = size / 2;
    float quarterSize = size / 4;
    children[0] = new Node(center.x - quarterSize, center.y - quarterSize, halfSize);
    children[1] = new Node(center.x + quarterSize, center.y - quarterSize, halfSize);
    children[2] = new Node(center.x - quarterSize, center.y + quarterSize, halfSize);
    children[3] = new Node(center.x + quarterSize, center.y + quarterSize, halfSize);
}

void Node::calculateForce(Body& b, float theta, float G) {
    if (totalMass == 0 || &b == body) return;

    Vec2 dir = centerOfMass - b.position;
    float distance = dir.magnitude();

    // Softening to prevent singularity
    float softening = 10.0f;
    distance = sqrt(distance * distance + softening * softening);

    if (children[0]) {
        // Use Barnes-Hut criterion
        if (size / distance < theta) {
            float forceMag = (G * totalMass * b.mass) / (distance * distance + softening);
            b.force += dir.normalize() * forceMag;
        }
        else {
            // Recurse
            for (int i = 0; i < 4; ++i) {
                if (children[i]) children[i]->calculateForce(b, theta, G);
            }
        }
    }
    else {
        // Direct calculation for leaves
        if (body != &b) {
            float forceMag = (G * totalMass * b.mass) / (distance * distance + softening);
            b.force += dir.normalize() * forceMag;
        }
    }
}


//Calculate Desntiy
void Node::calculateDensity(Body& b, float radius) {
    if (totalMass == 0 || &b == body) return;

    Vec2 direction = centerOfMass - b.position;
    float distance = direction.magnitude();

    if (distance <= radius || !children[0]) {
        // Add mass contribution to density
        b.density += totalMass / (distance * distance + 1e-4f); // Add smoothing term to avoid singularities
    }
    else {
        // Recurse on children
        for (int i = 0; i < 4; ++i) {
            if (children[i]) children[i]->calculateDensity(b, radius);
        }
    }
}

void Node::draw() {
    // Convert normalized coordinates to screen space
    float x = center.x * ofGetWidth();
    float y = center.y * ofGetHeight();
    float w = size * ofGetWidth();

    ofNoFill();
    ofSetColor(100, 100, 100, 50);
    ofDrawRectangle(x - w / 2, y - w / 2, w, w);

    // Draw children
    for (int i = 0; i < 4; ++i) {
        if (children[i]) children[i]->draw();
    }
}

//Logic for handling collisions
void ofApp::handleCollisions(std::vector<Body>& bodies, float minDistance) {
    spatialHash.clear();
    for (auto& b : bodies) spatialHash.insert(&b);

    for (size_t i = 0; i < bodies.size(); ++i) {
        Body& a = bodies[i];
        auto nearby = spatialHash.getNearbyBodies(a.position);

        for (Body* ptr : nearby) {
            if (ptr <= &a) continue; // Avoid duplicate checks
            Body& b = *ptr;

            Vec2 delta = b.position - a.position;
            float dist = delta.magnitude();
            if (dist < minDistance && dist > 0) {
                // Collision response
                Vec2 normal = delta.normalize();
                float overlap = minDistance - dist;
                a.position -= normal * overlap * 0.5f;
                b.position += normal * overlap * 0.5f;

                // Elastic collision
                Vec2 relVel = b.velocity - a.velocity;
                float velAlongNormal = relVel.dot(normal);
                if (velAlongNormal > 0) continue;

                float restitution = 0.8f;
                float impulseMag = -(1 + restitution) * velAlongNormal;
                impulseMag /= 1 / a.mass + 1 / b.mass;

                a.velocity -= normal * (impulseMag / a.mass);
                b.velocity += normal * (impulseMag / b.mass);
            }
        }
    }
}

ofApp::ofApp() : spatialHash(0.05f)  // Initialize spatialHash with cell size
, quadtreeRoot(nullptr)
, theta(0.5f)
, timeStep(0.001f)
, fps(0.0f)
, lastTime(0.0f)
{
}

// Setup simulation
void ofApp::setup() {

    gui.setup("Controls");
    gui.add(gSlider.setup("Gravity", 6.27e-11, 6.27e-11, 2.0));
    gui.add(timeStepSlider.setup("Time Step", 0.0001, 0.00001, 0.001));

    lastTime = ofGetElapsedTimef();
    fps = 0.0f;
    ofSetBackgroundColor(0, 0, 0);
    theta = .5f;
    timeStep = 0.0001f;



    Vec2 center(0.5f, 0.5f);
    for (int i = 0; i < 2000; ++i) {
        float angle = ofRandom(TWO_PI);
        float radius = ofRandom(0.1f, 0.4f);
        Vec2 pos = center + Vec2(cos(angle), sin(angle)) * radius;
        float mass = ofRandom(1.0f, 2.0f);
        bodies.emplace_back(pos.x, pos.y, mass);

        // Orbital velocity
        Vec2 tangent(-(pos.y - center.y), pos.x - center.x);
        bodies.back().velocity = tangent.normalize() * sqrt(1000.0f * mass / radius);
    }

    theta = 0.5f; // Barnes-Hut parameter
    timeStep = 0.001f;
}

void ofApp::update() {

    float currentTime = ofGetElapsedTimef();
    float currentG = gSlider;
    float currentTimeStep = timeStepSlider;
    float deltaTime = currentTime - lastTime;
    fps = 0.9f * fps + 0.1f * (1.0f / deltaTime); // Smoothed FPS
    lastTime = currentTime;


    // Delete previous quadtree
    delete quadtreeRoot;
    quadtreeRoot = new Node(0.5f, 0.5f, 1.0f);

    // Update spatial hash
    spatialHash.clear();
    for (auto& b : bodies) {
        spatialHash.insert(&b);
    }

    // Build quadtree
    for (auto& b : bodies) {
        quadtreeRoot->insert(&b);
    }

    // Calculate forces and update
    maxForce = 0.0f;
    for (auto& b : bodies) {
        b.force = Vec2(0, 0);
        quadtreeRoot->calculateForce(b, theta, currentG);
        maxForce = std::max(maxForce, b.force.magnitude());
    }

    // Update positions
    for (auto& b : bodies) {
        Vec2 acceleration = b.force * (1.0f / b.mass);
        b.velocity += acceleration * currentTimeStep;
        b.position += b.velocity * currentTimeStep;

        // Boundary wrap
        b.position.x = fmod(b.position.x + 1.0f, 1.0f);
        b.position.y = fmod(b.position.y + 1.0f, 1.0f);
    }

    handleCollisions(bodies, 0.005f);
}

void ofApp::draw() {
    ofBackground(0);

    // Draw quadtree
    if (quadtreeRoot) {
        quadtreeRoot->draw();
    }

    // Draw bodies
    for (const auto& b : bodies) {
        // Map force to color
        float forceMag = b.force.magnitude();
        float hue = ofMap(forceMag, 0, maxForce, 180, 360, true);
        ofColor color;
        color.setHsb(hue, 255, 255);
        ofSetColor(color);

        // Draw particle
        float x = b.position.x * ofGetWidth();
        float y = b.position.y * ofGetHeight();
        ofDrawCircle(x, y, 2);
    }

    // Draw FPS
    ofSetColor(255);
    ofDrawBitmapString("FPS: " + ofToString(fps, 1), 10, 20);    
    //ofDrawBitmapString("Current G: " + ofToString(gSlider, 1), 10, 40);
    //ofDrawBitmapString("Current Time Step: " + ofToString(timeStepSlider, 1), 10, 60);

    gui.draw();
}