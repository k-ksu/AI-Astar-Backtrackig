#include <iostream>
#include <algorithm>
#include <math.h>
#include <vector>
#include <functional>
#include <string>
#include <map>
#include <set>

class AStar {
public:
    using uint = unsigned int;

    // Structure to represent 2D coordinates
    struct Vec2i {
        int x, y;

        // Overloading addition operator for vector addition
        Vec2i operator+(const Vec2i& other) const {
            return { x + other.x, y + other.y };
        }

        // Overloading equality operator for coordinate comparison
        bool operator==(const Vec2i& coordinates_) const {
            return (x == coordinates_.x && y == coordinates_.y);
        }

        // Overloading less-than operator for sorting and set usage
        bool operator<(const Vec2i& other) const {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }
    };

    // Structure to represent a node in the A* search
    struct Node {
        Vec2i coordinates; // Node's coordinates
        Node* parent; // Pointer to the parent node
        uint G; // Cost from the start node
        uint H; // Heuristic cost to the target

        Node(Vec2i coordinates_, Node* parent_)
                : coordinates(coordinates_), parent(parent_), G(0), H(0) {}

        // Function to calculate the total score of the node
        uint getScore() {
            return G + H;
        }
    };

    using NodeSet = std::vector<Node*>; // Type alias for a set of nodes
    using CoordinateList = std::vector<Vec2i>; // Type alias for a list of coordinates

    // Class for heuristic functions
    class Heuristic {
    public:
        // Manhattan distance heuristic
        static uint manhattan(Vec2i source_, Vec2i target_) {
            auto delta = getDelta(source_, target_);
            return static_cast<uint>(10 * (delta.x + delta.y));
        }

        // Euclidean distance heuristic
        static uint euclidean(Vec2i source_, Vec2i target_) {
            auto delta = getDelta(source_, target_);
            return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
        }

    private:
        // Calculate the difference between two coordinates
        static Vec2i getDelta(Vec2i source_, Vec2i target_) {
            return { abs(source_.x - target_.x), abs(source_.y - target_.y) };
        }
    };

    // Class for pathfinding generator
    class Generator {
    public:
        Generator() : radius(1) {  // Initialize the radius with a default value
            setDiagonalMovement(false); // Disable diagonal movement by default
            setHeuristic(&Heuristic::manhattan); // Set default heuristic
            // Define movement directions (4 or 8 directions)
            direction = {
                    { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
                    { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
            };
        }

        // Set the size of the world
        void setWorldSize(Vec2i worldSize_) {
            worldSize = worldSize_;
        }

        // Enable or disable diagonal movement
        void setDiagonalMovement(bool enable_) {
            directions = (enable_ ? 8 : 4); // Set number of directions based on input
        }

        // Set the heuristic function
        void setHeuristic(uint(*heuristic_)(Vec2i, Vec2i)) {
            heuristic = heuristic_;
        }

        // Method to set the radius around the character
        void setRadius(int r) {
            radius = r;
        }

        // Add an obstacle to the world
        void addCollision(Vec2i coordinates_, const std::string& type = "") {
            obstacles[coordinates_] = type; // Store obstacle type

            // Define affected zones based on obstacle type
            if (type == "A") {  // Agent type
                for (const auto& dir : direction) {
                    obstacles[coordinates_ + dir] = "P"; // Mark surrounding cells as dangerous
                }
            } else if (type == "S") {  // Sentinel type
                static std::vector<Vec2i> crossDirection = { {0,1}, {1,0}, {0,-1}, {-1,0} };
                for (const auto& dir : crossDirection) {
                    obstacles[coordinates_ + dir] = "P"; // Mark cross cells as dangerous
                }
            }
        }

        // Clear agents from the world
        void clearAgents() {
            for (auto it = obstacles.begin(); it != obstacles.end(); ) {
                if (it->second == "A" || it->second == "P") {
                    it = obstacles.erase(it); // Remove agents and dangerous zones
                } else {
                    ++it;
                }
            }
        }

        // Find a path from source to target using A* algorithm
        CoordinateList findPath(Vec2i source_, Vec2i target_) {
            Node* current = nullptr; // Current node
            NodeSet openSet, closedSet; // Open and closed sets for A*
            openSet.reserve(100);
            closedSet.reserve(100);
            openSet.push_back(new Node(source_, nullptr)); // Start node

            // Main loop for A* algorithm
            while (!openSet.empty()) {
                auto current_it = openSet.begin();
                current = *current_it;

                // Find the node with the lowest score in openSet
                for (auto it = openSet.begin(); it != openSet.end(); it++) {
                    auto node = *it;
                    if (node->getScore() <= current->getScore()) {
                        current = node;
                        current_it = it;
                    }
                }

                // Check if the target has been reached
                if (current->coordinates == target_) {
                    break; // Exit if target is found
                }

                closedSet.push_back(current); // Move current node to closedSet
                openSet.erase(current_it); // Remove current node from openSet

                // Handle obstacle input around the current cell
                handleUserObstacleInput(current->coordinates);

                // Explore all possible movements from current node
                for (uint i = 0; i < directions; ++i) {
                    Vec2i newCoordinates(current->coordinates + direction[i]);

                    // Check for out-of-bounds
                    if (newCoordinates.x < 0 || newCoordinates.x >= worldSize.x ||
                        newCoordinates.y < 0 || newCoordinates.y >= worldSize.y) {
                        continue;  // Skip if out of bounds
                    }

                    // Ignore cells with collisions, but allow the target
                    if (detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates)) {
                        continue;
                    }

                    // Handle special cases for radius = 2
                    if (radius == 2){
                        bool hasPath = false;
                        for (uint j = 0; j < directions; ++j) {
                            Vec2i adjacentCoordinates = newCoordinates + direction[j];

                            // Check boundaries for adjacent cells
                            if (adjacentCoordinates.x < 0 || adjacentCoordinates.x >= worldSize.x ||
                                adjacentCoordinates.y < 0 || adjacentCoordinates.y >= worldSize.y ||
                                adjacentCoordinates == current->coordinates) {  // Ignore the cell we came from
                                continue;
                            }

                            // If adjacent cell is passable, mark hasPath true
                            if (!detectCollision(adjacentCoordinates) && !isDangerousZone(adjacentCoordinates)) {
                                hasPath = true;
                                break;
                            }
                        }
                        // If no paths are available, mark the cell as dangerous
                        if (!hasPath) {
                            obstacles[newCoordinates] = "P";
                            continue;
                        }
                    }

                    // Check for dangerous zones, ignoring the target
                    if (newCoordinates != target_ && isDangerousZone(newCoordinates)) {
                        std::cout << "Neo dies at (" << newCoordinates.x << ", " << newCoordinates.y << ")!\n";
                        return {};  // Return empty path if Neo enters a dangerous zone
                    }

                    uint totalCost = current->G + ((i < 4) ? 1 : 14); // Calculate cost to move to the new node
                    Node* successor = findNodeOnList(openSet, newCoordinates);
                    if (successor == nullptr) {
                        // Create a new node if it doesn't exist in openSet
                        successor = new Node(newCoordinates, current);
                        successor->G = totalCost; // Set G cost
                        successor->H = heuristic(successor->coordinates, target_); // Calculate H cost
                        openSet.push_back(successor); // Add new node to openSet
                    }
                    else if (totalCost < successor->G) {
                        // Update node if a cheaper path is found
                        successor->parent = current;
                        successor->G = totalCost; // Update G cost
                    }
                }
            }

            // Construct the path from target to source
            CoordinateList path;
            while (current != nullptr) {
                path.insert(path.begin(), current->coordinates); // Insert at the beginning for correct order
                current = current->parent; // Move to the parent node
            }

            // Release memory for nodes in openSet and closedSet
            releaseNodes(openSet);
            releaseNodes(closedSet);
            return path; // Return the final path
        }

        // Method to calculate the length of the path
        uint calculatePathLength(const CoordinateList& path) {
            uint length = 0;
            for (size_t i = 1; i < path.size(); ++i) {
                length += (path[i].x == path[i-1].x || path[i].y == path[i-1].y) ? 1 : 1;  // Length calculation
            }
            return length; // Return total length
        }

    private:
        int radius;  // Radius around the character
        std::map<Vec2i, std::string> obstacles; // Map to store obstacles
        Vec2i worldSize; // Size of the world
        std::vector<Vec2i> direction; // Possible movement directions
        uint directions; // Number of directions
        uint(*heuristic)(Vec2i, Vec2i); // Pointer to heuristic function

        // Find a node in the given node set
        Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_) {
            for (auto node : nodes_) {
                if (node->coordinates == coordinates_) {
                    return node; // Return node if found
                }
            }
            return nullptr; // Return null if not found
        }

        // Release memory for nodes in a set
        void releaseNodes(NodeSet& nodes_) {
            for (auto it = nodes_.begin(); it != nodes_.end();) {
                delete *it; // Delete node
                it = nodes_.erase(it); // Remove node from list
            }
        }

        // Check for collisions at a specific coordinate
        bool detectCollision(Vec2i coordinates_) {
            // Check if the coordinates are out of bounds
            if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
                coordinates_.y < 0 || coordinates_.y >= worldSize.y) {
                return true; // Return true for out-of-bounds
            }
            return obstacles.find(coordinates_) != obstacles.end(); // Check if obstacle exists
        }

        // Check if a coordinate is in a dangerous zone
        bool isDangerousZone(Vec2i coordinates_) {
            auto it = obstacles.find(coordinates_);
            return (it != obstacles.end() && it->second == "P"); // Return true if it's marked dangerous
        }

        // Handle user input for obstacles around a coordinate
        void handleUserObstacleInput(Vec2i newCoordinates) {
            std::cout << "m " << newCoordinates.x << " " << newCoordinates.y << "\n"; // Prompt for input

            int obstacleCount;
            std::cin >> obstacleCount; // Read number of obstacles

            // Read details for each obstacle
            for (int i = 0; i < obstacleCount; ++i) {
                int x, y;
                std::string type;
                std::cin >> x >> y >> type; // Read coordinates and type

                if (type == "K") {
                    continue;  // Skip if type is K
                }

                if (type != "B") {
                    addCollision({ x, y }, type); // Add collision if type is not B
                }

                if (type == "B") {
                    clearAgents();  // Clear all agents from the map
                    // Increase radius for Sentinels if they exist
                    for (auto& [coord, obstacle_type] : obstacles) {
                        if (obstacle_type == "S") {
                            addCollision(coord, "S");
                        }
                    }
                }
            }
        }
    };
};

int main() {
    AStar::Generator generator; // Create an instance of the A* pathfinding generator
    generator.setWorldSize({ 10, 10 }); // Set the world size to 10x10
    generator.setHeuristic(AStar::Heuristic::euclidean); // Set the heuristic to Euclidean
    generator.setDiagonalMovement(false); // Disable diagonal movement

    int radius;
    std::cin >> radius; // Read radius from user input
    generator.setRadius(radius);  // Set the radius in the generator

    int targetX, targetY;
    std::cin >> targetX >> targetY; // Read target coordinates
    AStar::Vec2i target = { targetX, targetY }; // Create target coordinate

    // Find the path from (0, 0) to the target
    auto path = generator.findPath({ 0, 0 }, target);

    // Check if a valid path was found
    if (path.empty() || path.back() != target) {
        std::cout << "e -1\n"; // Output error if no valid path exists
    } else {
        // Output the final coordinates and length of the path
        std::cout << "m " << target.x << " " << target.y << "\n";
        unsigned int pathLength = generator.calculatePathLength(path); // Calculate path length
        std::cout << "e " << pathLength << "\n"; // Output the path length
    }

    return 0; // Exit the program
}
