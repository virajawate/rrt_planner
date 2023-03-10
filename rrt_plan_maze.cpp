#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>

using namespace std;

const int ROWS = 10;
const int COLS = 12;
const int OBSTACLE =1;
const int FREE = 0;
const int STEP_SIZE  = 1;
const double MAX_ITERATIONS = 10100;

struct Node
{
    int x,y;
    Node* parent;
};

class RRT
{
    private:
        int matrix[ROWS][COLS];
        vector<Node*> nodes;
        Node* start;
        Node* goal;
        int iterations;
        bool pathFound;
        bool goalFound = false;

        bool isFree(int x, int y)
        {
            if(x<0 || x >= ROWS || y<0 || y >= COLS || matrix[x][y] == OBSTACLE)
            {
                return false;
            }

            return matrix[x][y] == FREE;
        }

        float distance(Node* n1, Node* n2)
        {
            int dx = n1->x - n2->x;
            int dy = n1->y - n2->y;
            return sqrt(dx*dx + dy*dy);
        }

        Node* randomNode()
        {
            Node* node = new Node();
            node->x = rand() % ROWS;
            node->y = rand() % COLS;
            node->parent = nullptr;
            return node;
        }

        Node* nearestNode(Node* node)
        {
            Node* nearest = nodes[0];
            float minDistance = distance(node, nearest);

            for(int i=1; i<nodes.size(); i++)
            {
                float d = distance(node, nodes[i]);
                if(d<minDistance)
                {
                    nearest = nodes[i];
                    minDistance = d;
                }
            }

            return nearest;
        }

        Node* newNode(Node* nearest, Node* random)
        {
            Node* node = new Node();
            float d = distance(nearest, random);
            if(d<=STEP_SIZE)
            {
                node->x = random->x;
                node->y = random->y;
            }
            else
            {
                float theta = atan2(random->y - nearest->y, random->x - nearest->x);
                node->x = nearest->x + STEP_SIZE * cos(theta);
                node->y = nearest->y + STEP_SIZE * sin(theta);
            }
            node->parent = nearest;
            return node;
        }

        bool isCloseToGoal(Node* node)
        {
            return distance(node, goal) <= STEP_SIZE;
        }

        bool isPathCollisionFree(Node* n1, Node* n2)
        {
            int dx = abs(n2->x - n1->x);
            int dy = abs(n2->y - n1->y);

            int sx = n1->x < n2->x ? 1 : -1;
            int sy = n1->y < n2->y ? 1 : -1;

            int err = dx - dy;
            
            int x = n1->x;
            int y = n1->y;

            while(x != n2->x || y != n2->y)
            {
             
                int e2 = 2*err;

                if(e2 > -dy)
                {
                    err -= dy;
                    x += sx;
                }

                if(e2 < dx)
                {
                    err += dx;
                    y += sy;
                }
            }

            return true;
        }

        void findPath()
        {
            Node* current = goal;
            while (current != NULL && current != start)
            {
                if (current->x <= 0 || current->x >= ROWS || current->y <= 0 || current->y >= COLS)
                {
                    // current is outside the bounds of the matrix
                    break;
                }
                current = current->parent;
                matrix[current->x][current->y] = 2;
            }
            if (current == start)
            {
                matrix[start->x][start->y] = 2;
            }
        }




    
    public:
        RRT()
        {
            srand(time(NULL));

            int matrix[ROWS][COLS] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                                      {0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1},
                                      {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
                                      {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1},
                                      {1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1},
                                      {0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0},
                                      {1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1},
                                      {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
                                      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

            start = new Node();
            start->x = 2;
            start->y = 2;
            start->parent = nullptr;

            goal = new Node();
            goal->x = 6;
            goal->y = 7;
            goal->parent = nullptr;

            nodes.push_back(start);

            iterations = 0;
            pathFound = false;
        }

        bool goalreach(Node* cell)
        {
            
            if(cell==goal)
            {
                return true;
            }
            return false;
        }

        void generateRRT()
        {
            // do
            // {
            Node* random = randomNode();
            Node* nearest = nearestNode(random);
            Node* node = newNode(nearest, random);

            if(isFree(node->x, node->y) && isPathCollisionFree(nearest, node))
            {
                nodes.push_back(node);

                if(isCloseToGoal(node))
                {
                    pathFound = true;
                    findPath();
                }
                
                // Add the new node to the matrix
                matrix[node->x][node->y] = 3;
                
                // Add the path to the matrix
                vector<Node*> path;
                Node* current = node;
                while (current != start)
                {
                    path.push_back(current);
                    current = current->parent;
                }
                path.push_back(start);
                
                for (int i = path.size() - 1; i >= 1; i--)
                {
                    Node* n1 = path[i];
                    Node* n2 = path[i-1];
                    if (isPathCollisionFree(n1, n2))
                    {
                        matrix[n1->x][n1->y] = 2;
                        matrix[n2->x][n2->y] = 2;
                        cout<<nodes[i]->x<<','<<nodes[i]->y<<endl;
                    }
                    else
                    {
                        continue;
                    }

                }
                for(int i=0; i<nodes.size(); i++)
                    goalFound = goalreach(nodes[i]);                    
            }
            // }while(!goalFound);
        }


        int printMatrix()
        {
            for(int i=0; i<=ROWS; i++)
            {
                for(int j=0; j<=COLS; j++)
                {
                    cout<<matrix[i][j]<<" ";
                }
                cout<<endl;
            }
            cout<<endl;
            return 0;
        }

        void printNodes()
        {
            for (int i = 0; i < nodes.size(); i++)
            {
                cout << "Node " << i << ": (" << nodes[i]->x << ", " << nodes[i]->y << ")" << endl;
            }
        }

};

int main()
{
    RRT rrt = RRT();
    rrt.printMatrix();
    cout<<endl;
    rrt.generateRRT();
    rrt.printMatrix();
    rrt.printNodes();
    return 0;
}