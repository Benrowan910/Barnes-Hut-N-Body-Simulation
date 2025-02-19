#include "ofApp.h"


float lastTime = 0.0f;
float fps = 0.0f;

// Insert body into quadtree node
void Node::insert(Body* b) {
    if (!body && totalMass == 0) {
        // Empty node
        body = b;
        totalMass = b->mass;
        centerOfMass = b->position;
        return;
    }

    if (body) {
        // Subdivide and reinsert existing body
        subdivide();
        for (int i = 0; i < 4; ++i) {
            if (children[i]->contains(*body)) {
                children[i]->insert(body);
                break;
            }
        }
        body = nullptr; // Node now represents multiple bodies
    }

    // Insert the new body
    for (int i = 0; i < 4; ++i) {
        if (children[i]->contains(*b)) {
            children[i]->insert(b);
            break;
        }
    }

    // Update total mass and center of mass
    totalMass += b->mass;
    centerOfMass = (centerOfMass * (totalMass - b->mass) + b->position * b->mass) / totalMass;
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

// Calculate gravitational force acting on a body using Barnes-Hut approximation
void Node::calculateForce(Body& b, float theta) {
    if (totalMass == 0 || &b == body) return;

    Vec2 direction = centerOfMass - b.position;
    float distance = direction.magnitude();

    // Avoid extremely close distances causing instability (softening term)
    //float softeningFactor = 1e-4f;
    //distance = std::max(distance, softeningFactor);

    // Check if the node can be approximated as a single mass
    if (size / distance < theta || !children[0]) {
        // Gravitational constant (adjusted for the scale of the simulation)
        float G = 6.7e-11f; // Adjust this value based on the scale of your simulation

        // Apply the inverse square law of gravitation
        float forceMagnitude = (G * totalMass * b.mass) / (distance * distance);

        // Normalize direction and apply force
        b.force = b.force + direction.normalize() * forceMagnitude;
    }
    else {
        // Recurse on children nodes for more precise force calculation
        for (int i = 0; i < 4; ++i) {
            if (children[i]) {
                children[i]->calculateForce(b, theta);
            }
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

//Logic for handling collisions
void ofApp::handleCollisions(std::vector<Body>& bodies, float minDistance) {
    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            Vec2 direction = bodies[j].position - bodies[i].position;
            float distance = direction.magnitude();

            if (distance < minDistance) {
                // Normalize direction
                Vec2 normalizedDirection = direction.normalize();

                // Adjust positions to prevent overlap
                float overlap = minDistance - distance;
                bodies[i].position -= normalizedDirection * (overlap / 2);
                bodies[j].position += normalizedDirection * (overlap / 2);

                // Adjust velocities (simple elastic collision)
                float relativeVelocity = (bodies[j].velocity - bodies[i].velocity).dot(normalizedDirection);
                if (relativeVelocity < 0) { // Only adjust if particles are moving toward each other
                    Vec2 impulse = normalizedDirection * relativeVelocity;
                    bodies[i].velocity += impulse;
                    bodies[j].velocity -= impulse;
                }
            }
        }
    }
}

// Setup simulation
void ofApp::setup() {
    ofSetBackgroundColor(0, 0, 0);
    theta = .01f;
    timeStep = 0.01f;

    // Create random bodies
    for (int i = 0; i < 100; ++i) {
        float x = ofRandom(0.1f, 0.9f);
        float y = ofRandom(0.1f, 0.9f);
        float mass = ofRandom(10.0f, 10000.0f);
        bodies.emplace_back(x, y, mass);

        Vec2 center(0.5f, 0.5f);
        Vec2 direction = Vec2(x, y) - center;
        float distance = direction.magnitude();
        float velocityMagnitude = std::sqrt(6.67430e-11 * 100.0f / distance);
        Vec2 tangent(-direction.y, direction.x);
        //bodies.back().velocity = tangent.normalize() * velocityMagnitude;
        bodies.back().velocity = Vec2(ofRandom(-0.01f, 0.01f), ofRandom(-0.01f, 0.01f));
    }
}

// Update simulation
void ofApp::update() {

    float currentTime = ofGetElapsedTimef();
    fps = 1.0f / (currentTime - lastTime);
    lastTime = currentTime;

    Node root(0.5f, 0.5f, 1.0f);

    // Insert bodies into quadtree
    for (auto& b : bodies) {
        root.insert(&b);
    }

    // Calculate forces and update positions
    for (auto& b : bodies) {
        b.force = Vec2(0, 0);
        b.density = 0;

        root.calculateDensity(b, 0.0005f);
        root.calculateForce(b, theta);

        //float densityFactor = 0.000001f;
        //b.force = b.force + b.position.normalize() * (b.density * densityFactor);

        Vec2 acceleration = b.force * (1.0f / b.mass);
        b.velocity = b.velocity + acceleration * timeStep;
        b.position = b.position + b.velocity * timeStep;

        // Wrap around edges (optional)
        if (b.position.x < 0) b.position.x += 1;
        if (b.position.x > 1) b.position.x -= 1;
        if (b.position.y < 0) b.position.y += 1;
        if (b.position.y > 1) b.position.y -= 1;
    }
    handleCollisions(bodies, 0.01f);
}

// Draw simulation
void ofApp::draw() {
    ofSetColor(255, 255, 255);
    ofDrawBitmapString("FPS: " + std::to_string(fps), 10, 20);

    for (const auto& b : bodies) {
        float x = b.position.x * ofGetWidth();
        float y = b.position.y * ofGetHeight();
        float radius = 0.5f;

        //ofDrawLine(b.position.x * ofGetWidth(), b.position.y * ofGetHeight(),
            //(b.position + b.force * 0.1f).x * ofGetWidth(), (b.position + b.force * 0.1f).y * ofGetHeight());

        float brightness = ofClamp(b.density * 1000.0f, 0, 255); // Adjust scale
        ofSetColor(brightness, brightness, 255);
        ofDrawCircle(x, y, radius);
    }
}