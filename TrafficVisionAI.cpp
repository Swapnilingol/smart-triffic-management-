/**
 * Traffic Vision AI: Intelligent Management System
 * 
 * ==========================================
 * COMPARISON OF APPROACHES SECTION
 * ==========================================
 * 1. Greedy vs. Graph-Based:
 *    - Greedy Approach (Real-time signal switching):
 *      Used for immediate, localized decision-making. At a single intersection, checking
 *      the queue length (density) of 4 lanes and picking the maximum is O(1) or O(L) where L=4.
 *      It provides instant reactiveness which is critical for real-time traffic signal switching.
 *    - Graph-Based Approach (Routing & Global Planning):
 *      Representing the entire city map as nodes (intersections) and edges (roads) is utilized
 *      for long-term routing (like an Ambulance's Green Wave across multiple intersections).
 *      A graph search (like Dijkstra's or Backtracking) is too slow for real-time light switching
 *      at a single node but is perfect for pre-calculating the best path across the city.
 * 
 * 2. Time Complexity calculations:
 *    - Wait Time Calculation (Dynamic Programming style): By keeping a running sum of wait times,
 *      we update the wait time for N cars in a lane in O(N) rather than recalculating from start O(N^2).
 *    - Searching/Routing: A simple DFS/Backtracking on a graph with V vertices is O(V + E).
 *      If we used a balanced binary search tree or priority queue for wait time queries, search 
 *      could be in O(log N) time instead of O(N).
 */

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cstdlib>
#include <ctime>
#include <algorithm>

// NOTE: Uncomment the following line and configure your compiler to use SFML if available.
// #include <SFML/Graphics.hpp> 
// If SFML is unavailable, the logic will still compile and run in a text-based simulated loop.

// ---------------------------------------------------------
//  ALGORITHM 1: DYNAMIC PROGRAMMING / TIME COMPLEXITY
// ---------------------------------------------------------
// Represents a single Vehicle
class Vehicle {
public:
    int id;
    bool isAmbulance;
    int expectedWaitTime; // Calculated using DP concepts (prefix sums)

    Vehicle(int _id, bool _isAmb) : id(_id), isAmbulance(_isAmb), expectedWaitTime(0) {}
};

// Represents a Traffic Lane
class Lane {
private:
    std::string name;
    std::queue<Vehicle> vehicles;
    bool ambulancePresent;
    int starvationScore;

public:
    Lane(std::string n) : name(n), ambulancePresent(false), starvationScore(0) {}

    void incrementStarvation() { if (!vehicles.empty()) starvationScore++; }
    void resetStarvation() { starvationScore = 0; }
    int getStarvation() const { return starvationScore; }

    void addVehicle(Vehicle v) {
        if (v.isAmbulance) ambulancePresent = true;
        vehicles.push(v);
        calculateWaitTimes(); // DP optimization step
    }

    void removeVehicle() {
        if (!vehicles.empty()) {
            if (vehicles.front().isAmbulance) {
                // If the ambulance passes, check if there are others
                ambulancePresent = false; 
            }
            vehicles.pop();
            
            // Re-check for ambulance in remaining queue
            std::queue<Vehicle> temp = vehicles;
            ambulancePresent = false;
            while(!temp.empty()) {
                if(temp.front().isAmbulance) ambulancePresent = true;
                temp.pop();
            }
            calculateWaitTimes();
        }
    }

    // Dynamic Programming Concept - Prefix Sums for wait time.
    // Instead of recalculating every vehicle's wait time naively O(n^2),
    // we use the time of the previous vehicle + constant processing time. Time: O(n).
    void calculateWaitTimes() {
        if(vehicles.empty()) return;
        
        std::queue<Vehicle> tempQueue;
        int currentWait = 0;
        int processingTimePerVehicle = 2; // basic static unit
        
        while(!vehicles.empty()) {
            Vehicle v = vehicles.front();
            vehicles.pop();
            v.expectedWaitTime = currentWait;
            currentWait += processingTimePerVehicle; // DP: use previous computation
            tempQueue.push(v);
        }
        vehicles = tempQueue; // restore queue
    }

    int getDensity() const {
        return vehicles.size();
    }

    bool hasAmbulance() const {
        return ambulancePresent;
    }

    std::string getName() const {
        return name;
    }
};

// ---------------------------------------------------------
//  ALGORITHM 2: GREEDY ALGORITHM & AMBULANCE OVERRIDE
// ---------------------------------------------------------
enum LightState { RED, GREEN, YELLOW };

class TrafficLight {
public:
    LightState state;
    TrafficLight() : state(RED) {}
};

class Controller {
private:
    std::vector<Lane> lanes;
    std::vector<TrafficLight> lights;
    int currentGreenIndex;
    int greenStartDensity; // Tracks how many cars were present when green was given

public:
    Controller() {
        lanes.push_back(Lane("North"));
        lanes.push_back(Lane("South"));
        lanes.push_back(Lane("East"));
        lanes.push_back(Lane("West"));
        lights.resize(4);
        currentGreenIndex = 0;
        greenStartDensity = 0;
        lights[0].state = GREEN;
    }

    Lane& getLane(int index) { return lanes[index]; }

    void addRandomTraffic() {
        for(int i = 0; i < 4; i++) {
            int numCars = rand() % 3;
            for(int c = 0; c < numCars; c++) {
                // 10% chance for an ambulance
                bool isAmb = (rand() % 100) < 10; 
                lanes[i].addVehicle(Vehicle(rand() % 1000, isAmb));
            }
        }
    }

    // Main real-time switching logic
    void updateSignals() {
        // AMBULANCE DETECTION OVERRIDE (always bypasses drainage rule)
        for (int i = 0; i < 4; i++) {
            if (lanes[i].hasAmbulance()) {
                std::cout << "\n[!] AMBULANCE DETECTED IN " << lanes[i].getName() << " LANE! OVERRIDING SYSTEM.\n";
                setGreenLight(i);
                return;
            }
        }

        // --- 50% DRAINAGE RULE ---
        // If the current green lane had a large initial density, do not switch
        // until at least half of those original cars have passed through.
        int currentDensity = lanes[currentGreenIndex].getDensity();
        int halfThreshold = greenStartDensity / 2; // 50% of initial count must clear
        if (currentDensity > halfThreshold && greenStartDensity > 1) {
            std::cout << "[Hold] " << lanes[currentGreenIndex].getName()
                      << " is draining: " << currentDensity << " remaining (need <= "
                      << halfThreshold << " to release).\n";
            return; // Lock green here until threshold is met
        }

        // Manage Starvation & Find current green lane
        for (int i = 0; i < 4; i++) {
            if (i == currentGreenIndex) {
                lanes[i].resetStarvation();
            } else {
                lanes[i].incrementStarvation();
            }
        }

        // GREEDY APPROACH: Pick the lane with the highest density
        int maxDensity = -1;
        int maxLaneIndex = currentGreenIndex;

        // Anti ping-pong momentum: Give the currently active lane a +3 priority bonus
        // so it doesn't instantly flicker to another lane when they are tied in length.
        int bias = (lanes[currentGreenIndex].getDensity() > 0) ? 3 : 0;
        
        for (int i = 0; i < 4; i++) {
            // Calculate effective density: base density + starvation age bonus + momentum bias
            int effectiveDensity = lanes[i].getDensity();
            effectiveDensity += (lanes[i].getStarvation() / 2); // Starving lanes slowly gain priority

            if (i == currentGreenIndex) effectiveDensity += bias;

            if (effectiveDensity > maxDensity) {
                maxDensity = effectiveDensity;
                maxLaneIndex = i;
            }
        }

        // Switch to the most dense lane to minimize overall wait time greedily
        if (maxLaneIndex != currentGreenIndex) {
            std::cout << "Greedy Logic: Switching to " << lanes[maxLaneIndex].getName() 
                      << " because it has highest effective density (" << maxDensity << ").\n";
            setGreenLight(maxLaneIndex);
        } else {
            std::cout << lanes[currentGreenIndex].getName() << " continues green (Density: " << maxDensity << ").\n";
        }
    }

    void setGreenLight(int index) {
        for (int i = 0; i < 4; i++) lights[i].state = RED;
        lights[index].state = GREEN;
        currentGreenIndex = index;
        // Capture starting density so the 50% drainage rule can track progress
        greenStartDensity = lanes[index].getDensity();
    }

    void letVehiclesPass() {
        // Decrease queue for the green light lane
        if (lanes[currentGreenIndex].getDensity() > 0) {
            std::cout << " - Vehicle passed from " << lanes[currentGreenIndex].getName() << " lane.\n";
            lanes[currentGreenIndex].removeVehicle();
        }
    }

    void printDashboard() {
        std::cout << "\n=== Traffic Vision AI Dashboard ===\n";
        for (int i = 0; i < 4; i++) {
            std::cout << lanes[i].getName() << " Lane \t| Cars: " << lanes[i].getDensity();
            if (lanes[i].hasAmbulance()) std::cout << " [AMBULANCE]";
            std::cout << " \t| Light: " << (lights[i].state == GREEN ? "[GREEN]" : "[RED]") << "\n";
        }
        std::cout << "===================================\n";
    }
};

// ---------------------------------------------------------
//  ALGORITHM 3: GRAPH CONCEPTS & BACKTRACKING
// ---------------------------------------------------------
// Representing intersections as Nodes and roads as Edges.
// Backtracking/Branch & Bound to find a clear path for Ambulance (Green Wave).

#define V 4 // Suppose a mini map of 4 intersections

class CityMapGraph {
    int adjMatrix[V][V];
public:
    CityMapGraph() {
        for(int i=0; i<V; i++)
            for(int j=0; j<V; j++)
                adjMatrix[i][j] = 0;
        
        // Define simple roads (edges) with weights (traffic density)
        // 0-1, 0-2, 1-3, 2-3
        addEdge(0, 1, 5);
        addEdge(0, 2, 2);
        addEdge(1, 3, 10); // Heavy traffic
        addEdge(2, 3, 3);
    }

    void addEdge(int u, int v, int weight) {
        adjMatrix[u][v] = weight;
        adjMatrix[v][u] = weight; 
    }

    // Backtracking based dfs to find a path
    void findGreenWavePathUtil(int u, int dest, std::vector<bool>& visited, std::vector<int>& path, 
                               int currentCost, int& bestCost, std::vector<int>& bestPath) {
        visited[u] = true;
        path.push_back(u);

        if (u == dest) {
            // Branch & Bound concept: update if this path cost is strictly better
            if (currentCost < bestCost) {
                bestCost = currentCost;
                bestPath = path;
            }
        } else {
            for (int i = 0; i < V; ++i) {
                if (adjMatrix[u][i] && !visited[i]) {
                    // Branch & Bound: Only explore if current cost + edge < bestCost
                    if (currentCost + adjMatrix[u][i] < bestCost) {
                        findGreenWavePathUtil(i, dest, visited, path, currentCost + adjMatrix[u][i], bestCost, bestPath);
                    }
                }
            }
        }

        // Backtrack
        path.pop_back();
        visited[u] = false;
    }

    void calculateAmbulanceRoute(int start, int dest) {
        std::vector<bool> visited(V, false);
        std::vector<int> path;
        std::vector<int> bestPath;
        int bestCost = 999999; // Infinity

        findGreenWavePathUtil(start, dest, visited, path, 0, bestCost, bestPath);

        std::cout << "\n[Graph/Backtracking] Optimal Green Wave Path for Ambulance from " 
                  << start << " to " << dest << ":\n -> Route: ";
        for(int node : bestPath) std::cout << node << " ";
        std::cout << "\n -> Total Traffic Cost: " << bestCost << "\n";
    }
};

// ---------------------------------------------------------
//  MAIN SIMULATION LOOP (WITH SFML STUBS)
// ---------------------------------------------------------
int main() {
    srand((unsigned)time(0));
    Controller c;
    CityMapGraph g;

    std::cout << "Starting Traffic Vision AI System...\n";

    // Graph concept demonstration before the main simulation loop
    g.calculateAmbulanceRoute(0, 3); // Find path from Intersection 0 to 3

    // ==========================================
    // GUI LOOP (Simulated in terminal below)
    // If you have SFML, you would initialize here:
    // sf::RenderWindow window(sf::VideoMode(800, 800), "Traffic Vision AI");
    // while (window.isOpen()) { 
    //    ... SFML event loops and render calls map over this logic ... 
    // }
    // ==========================================

    for (int tick = 0; tick < 10; tick++) { // Simulating 10 ticks
        std::cout << "\n--- TICK " << tick << " ---\n";
        
        // 1. Randomly arrive traffic
        c.addRandomTraffic();

        // 2. Decide logic (Greedy / Override)
        c.updateSignals();

        // 3. Output state
        c.printDashboard();

        // 4. Vehicles move
        c.letVehiclesPass();
        c.letVehiclesPass(); // double rate for green lane passing
    }

    std::cout << "\nSimulation Complete. System shutting down.\n";
    return 0;
}
