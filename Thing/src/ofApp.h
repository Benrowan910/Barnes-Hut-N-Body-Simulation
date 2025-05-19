#pragma once
#include "ofMain.h"
#include "ofxGui.h"
#include <vector>

extern "C" {
    void* __std_find_trivial_1 = nullptr;
    void* __std_find_trivial_2 = nullptr;
}

// 2D vector utility struct
struct Vec2 {
    float x, y;
    Vec2(float x = 0, float y = 0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& other) const { return Vec2(x + other.x, y + other.y); }
    Vec2 operator-(const Vec2& other) const { return Vec2(x - other.x, y - other.y); }
    Vec2 operator*(float scalar) const { return Vec2(x * scalar, y * scalar); }
    Vec2 operator/(float scalar) const { return Vec2(x / scalar, y / scalar); } // Added division operator
    Vec2& operator+=(const Vec2& v) {
        x += v.x;
        y += v.y;
        return *this;
    }
    Vec2& operator-=(const Vec2& v) {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    float magnitude() const { return std::sqrt(x * x + y * y); }
    Vec2 normalize() const { float mag = magnitude(); return mag == 0 ? Vec2(0, 0) : Vec2(x / mag, y / mag); }

    // Dot product
    float dot(const Vec2& v) const {
        return x * v.x + y * v.y;
    }
};

// Struct for celestial bodies
struct Body {
    Vec2 position, velocity, force;
    float mass, density;
    Body(float x, float y, float mass) : position(x, y), velocity(0, 0), force(0, 0), mass(mass), density(0) {}
};

// QuadTree Node for Barnes-Hut simulation
struct Node {
    Vec2 center;        // Center of the node
    float size;         // Size of the node (width/height)
    float totalMass;    // Total mass in this region
    Vec2 centerOfMass;  // Center of mass in this region
    Body* body;         // Single body if leaf
    Node* children[4];  // Four quadrants

    Node(float x, float y, float size)
        : center(x, y), size(size), totalMass(0), centerOfMass(0, 0), body(nullptr) {
        for (int i = 0; i < 4; ++i) children[i] = nullptr;
    }

    ~Node() {
        for (int i = 0; i < 4; ++i) {
            delete children[i];
        }
    }

    // Reset function to clear the node and prepare it for reuse
    void reset(float x, float y, float size) {
        center = Vec2(x, y);
        this->size = size;
        totalMass = 0;
        centerOfMass = Vec2(0, 0);
        body = nullptr;
        for (int i = 0; i < 4; ++i) {
            children[i] = nullptr; // Children will be reallocated as needed
        }
    }

    bool contains(const Body& b) const {
        return b.position.x >= center.x - size / 2 && b.position.x <= center.x + size / 2 &&
            b.position.y >= center.y - size / 2 && b.position.y <= center.y + size / 2;
    }

    void insert(Body* b);
    void subdivide();
    void calculateForce(Body& b, float theta, float G);
    void calculateDensity(Body& b, float radius);
    void draw();

};

// A hash function for 2D integer coordinates
struct GridKey {
    int x, y;
    bool operator==(const GridKey& other) const { return x == other.x && y == other.y; }
};

// Hash function for unordered_map
struct GridKeyHasher {
    std::size_t operator()(const GridKey& key) const {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
    }
};

class SpatialHash {
public:
    float cellSize; // Size of each grid cell
    std::unordered_map<GridKey, std::vector<Body*>, GridKeyHasher> grid;

    SpatialHash(float cellSize) : cellSize(cellSize) {}
    SpatialHash(const SpatialHash&) = delete;  // Disable copying
    SpatialHash& operator=(const SpatialHash&) = delete;

    // Hash a position to grid coordinates
    GridKey hash(const Vec2& position) const {
        return { static_cast<int>(std::floor(position.x / cellSize)),
                 static_cast<int>(std::floor(position.y / cellSize)) };
    }

    // Clear the grid
    void clear() { grid.clear(); }

    // Add a body to the grid
    void insert(Body* body) {
        GridKey key = hash(body->position);
        grid[key].push_back(body);
    }

    // Retrieve nearby bodies
    std::vector<Body*> getNearbyBodies(const Vec2& position) const {
        std::vector<Body*> nearbyBodies;
        GridKey center = hash(position);

        // Check the 3x3 neighborhood around the cell
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                GridKey neighbor = { center.x + dx, center.y + dy };
                auto it = grid.find(neighbor);
                if (it != grid.end()) {
                    nearbyBodies.insert(nearbyBodies.end(), it->second.begin(), it->second.end());
                }
            }
        }
        return nearbyBodies;
    }
};

class ofApp : public ofBaseApp {
public:

    ofApp();

    std::vector<Body> bodies;
    float theta;       // Approximation threshold
    float timeStep;    // Time step for simulation
    float lastTime;
    float fps;

    Node* quadtreeRoot = nullptr;
    SpatialHash spatialHash;
    float maxForce = 0.0f;

    ofxPanel gui;
    ofxFloatSlider gSlider;
    ofxFloatSlider timeStepSlider;

    void setup();
    void update();
    void draw();
    void handleCollisions(std::vector<Body>& bodies, float minDistance);
};



