#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

using namespace std;

// Define a struct to hold a vertex's x, y, and z coordinates
struct Vertex {
    float x;
    float y;
    float z;
};

// Define a struct to hold a sphere's center coordinates and radius
struct Sphere {
    Vertex center;
    float radius;
};

// Calculate the Euclidean distance between two vertices
float distance(Vertex v1, Vertex v2) {
    float dx = v1.x - v2.x;
    float dy = v1.y - v2.y;
    float dz = v1.z - v2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

// Check if three vertices form a sphere
bool isSphere(Vertex v1, Vertex v2, Vertex v3, float threshold) {
    // Calculate the distances between the vertices
    float d12 = distance(v1, v2);
    float d13 = distance(v1, v3);
    float d23 = distance(v2, v3);
    // Calculate the sphere's center and radius
    float a = d12*d12 + d13*d13 - d23*d23;
    float b = d12*d12 + d23*d23 - d13*d13;
    float c = d13*d13 + d23*d23 - d12*d12;
    float x = (a*v1.x + b*v2.x + c*v3.x) / (2*(a+b+c));
    float y = (a*v1.y + b*v2.y + c*v3.y) / (2*(a+b+c));
    float z = (a*v1.z + b*v2.z + c*v3.z) / (2*(a+b+c));
    float r = distance(Vertex{x, y, z}, v1);
    // Check if the vertices form a sphere
    for (Vertex v : {v1, v2, v3}) {
        if (distance(v, Vertex{x, y, z}) > r + threshold) {
            return false;
        }
    }
    return true;
}

int main() {
    // Open the OBJ file
    ifstream objFile("/home/liam/Downloads/aaa/untitled.obj");
    if (!objFile) {
        cerr << "Error: could not open OBJ file" << endl;
        return 1;
    }
    // Define vectors to hold the vertices and sphere centers
    vector<Vertex> vertices;
    vector<Vertex> orbCenters;
    // Define a threshold for identifying spheres
    float threshold = 0.1f;
    // Parse the OBJ file
    while (!objFile.eof()) {
        string line;
        getline(objFile, line);
        if (line.substr(0, 2) == "v ") {
            // Extract the vertex coordinates from the line
            float x, y, z;
            sscanf(line.c_str(), "v %f %f %f", &x, &y, &z);
            Vertex v = {x, y, z};
            // Check if the vertex is part of a sphere
            for (int i = 0; i < vertices.size(); i++) {
                for (int j = i + 1; j < vertices.size(); j++) {
                    for (int k = j + 1; k < vertices.size(); k++) {
                        if (isSphere(vertices[i], vertices[j], vertices[k], threshold)) {
                            orbCenters.push_back(Vertex{x, y, z});
                            break;
                        }
                    }
                }
            }
            // Add the vertex to the list of vertices
            vertices.push_back(v);
        }
    }
    // Print the coordinates of the orb centers
    for (Vertex c: orbCenters) {
        cout << "Orb center: (" << c.x << ", " << c.y << ", " << c.z << ")" << endl;
    }
    return 0;
}
