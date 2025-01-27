#include <iostream>
#include <vector>
#include <limits>

using namespace std;

// Global variables for the radius, amount of obstacles, destination coordinates, and path lengths
int radius;
int amount;
int destX, destY; // Coordinates of the destination point
int minPath = INT_MAX; // Minimum path length found
int curPath = 0; // Current path length

const int SIZE = 9; // Size of the grid
// 2D vectors for tracking visited cells and dangerous cells
vector<vector<bool>> visited(SIZE, vector<bool>(SIZE, false));
vector<vector<bool>> danger(SIZE, vector<bool>(SIZE, false));
// 2D vector for storing minimum distances
vector<vector<int>> minDist(SIZE, vector<int>(SIZE, INT_MAX));

// Direction arrays for movement: Up, Right, Down, Left
int dx[] = {-1, 0, 1, 0};
int dy[] = {0, 1, 0, -1};

// Function to mark a cell as dangerous
void addDanger(int x, int y) {
    danger[x][y] = 1; // Set the corresponding cell in the danger matrix
}

// Recursive function to search for the path using memoization
void search(int x, int y) {
    // If the current path length is not better than the already known distance, return
    if (curPath >= minDist[y][x]) {
        return;
    }

    // Update the minimum distance for the current cell
    minDist[y][x] = curPath;

    // Output the current cell being processed
    cout << "m " << x << " " << y << endl;
    cin >> amount; // Read the number of obstacles around the current cell
    for (int i = 0; i < amount; i++) {
        int x, y;
        std::string type;
        cin >> x >> y >> type; // Read obstacle details

        if (type == "K" || type == "B") {
            continue;  // Skip iteration if the type is K or B
        }

        addDanger(y, x); // Mark the cell as dangerous
    }

    // Check if the destination has been reached
    if (x == destX && y == destY) {
        minPath = min(minPath, curPath); // Update the minimum path length
        return; // Exit if the target is reached
    }

    // Explore all possible directions
    for (int i = 0; i < 4; i++) {
        int xx = x + dx[i]; // New x-coordinate
        int yy = y + dy[i]; // New y-coordinate

        // Check for boundaries and already visited or dangerous cells
        if (xx < 0 || yy < 0 || xx >= SIZE || yy >= SIZE || visited[yy][xx] || danger[yy][xx]) {
            continue; // Skip this cell if it's out of bounds or already visited
        }

        visited[yy][xx] = true; // Mark the new cell as visited
        curPath++; // Increment the current path length
        search(xx, yy); // Recursively call search for the next cell

        // Output indicating return to the previous cell
        cout << "m " << x << " " << y << endl;
        cin >> amount; // Read the number of obstacles again
        for (int i = 0; i < amount; i++) {
            int x, y;
            std::string type;
            cin >> x >> y >> type; // Read obstacle details again
        }

        curPath--; // Decrement the current path length
        visited[yy][xx] = false; // Unmark the cell as visited for backtracking
    }
}

int main() {
    cin >> radius; // Input the radius for the search
    cin >> destX >> destY; // Input the destination coordinates

    visited[0][0] = true; // Mark the starting cell as visited
    search(0, 0); // Start the search from the initial position (0,0)

    // Check if a path was found and output the result
    if (minPath == INT_MAX) {
        cout << "e -1" << endl; // Output -1 if no path was found
    } else {
        cout << "e " << minPath << endl; // Output the minimum path length
    }

    return 0; // Exit the program
}
