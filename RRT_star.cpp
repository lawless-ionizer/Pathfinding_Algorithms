#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <algorithm>

struct Point {
    double X, Y;
    Point(double x = 0, double y = 0) : X(x), Y(y) {}
};

struct Node {
    sf::Vector2f point;
    Node* parent;
    double cost;
    Node(sf::Vector2f point, Node* parent = nullptr, double cost = 0.0) 
        : point(point), parent(parent), cost(cost) {}
};

struct Obstacle {
    sf::FloatRect rect;
    Obstacle(float left, float top, float width, float height) : rect(left, top, width, height) {}
};

double distance(const sf::Vector2f& a, const sf::Vector2f& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

sf::Vector2f getRandomPoint(double x_max, double y_max) {
    return sf::Vector2f(static_cast<double>(rand()) / RAND_MAX * x_max, 
                 static_cast<double>(rand()) / RAND_MAX * y_max);
}

Node* nearest(const std::vector<Node*>& nodes, const sf::Vector2f& point) {
    Node* nearestNode = nullptr;
    double minDist = std::numeric_limits<double>::max();
    for (auto node : nodes) {
        double dist = distance(node->point, point);
        if (dist < minDist) {
            minDist = dist;
            nearestNode = node;
        }
    }
    return nearestNode;
}

sf::Vector2f steer(const sf::Vector2f& from, const sf::Vector2f& to, double maxStepSize) {
    double dist = distance(from, to);
    if (dist <= maxStepSize) {
        return to;
    }
    double theta = std::atan2(to.y - from.y, to.x - from.x);
    return sf::Vector2f(from.x + maxStepSize * std::cos(theta), 
                 from.y + maxStepSize * std::sin(theta));
}

int orientation(const sf::Vector2f& l, const sf::Vector2f& m, const sf::Vector2f& n)
{
    double crossP = (m.y - l.y)*(n.x - l.x) - (m.x - l.x)*(n.y - l.x);
    
    if(crossP == 0)
        return 0;
    return (crossP > 0)? 1 : -1;
}

bool lineIntersect(const sf::Vector2f& p1, const sf::Vector2f& p2, const sf::ConvexShape& obstacle) {
    int n = obstacle.getPointCount();

    for(int i = 0; i < n; i++)
    {
        int direc = orientation(p1,p2,obstacle.getPoint(i)), ogDirec;
        if(i == 0)
            ogDirec = direc;
        
        if(ogDirec != direc)
            return true;
    }

    return false;

    // // Check if line intersects any of the rectangle's sides
    // auto intersects = [&](float x3, float y3, float x4, float y4) {
    //     float denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    //     if (denom == 0.0f) return false; // Parallel lines

    //     float ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    //     float ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    //     return (ua >= 0.0f && ua <= 1.0f) && (ub >= 0.0f && ub <= 1.0f);
    // };

    // return intersects(minX, minY, maxX, minY) || // Top side
    //        intersects(minX, maxY, maxX, maxY) || // Bottom side
    //        intersects(minX, minY, minX, maxY) || // Left side
    //        intersects(maxX, minY, maxX, maxY);   // Right side
}

bool obstacleFree(const sf::Vector2f& from, const sf::Vector2f& to, const std::vector<sf::ConvexShape>& obstacles) {
    for (const auto& obstacle : obstacles)
    {
        if (lineIntersect(from, to, obstacle))
            return false;
    }
    return true;
}

std::vector<Node*> findNearNodes(const std::vector<Node*>& nodes, const sf::Vector2f& point, double radius) {
    std::vector<Node*> nearNodes;
    for (auto node : nodes) {
        if (distance(node->point, point) <= radius) {
            nearNodes.push_back(node);
        }
    }
    return nearNodes;
}

Node* chooseParent(const std::vector<Node*>& nearNodes, Node* nearestNode, const sf::Vector2f& newPoint) {
    Node* minCostNode = nearestNode;
    double minCost = nearestNode->cost + distance(nearestNode->point, newPoint);
    for (auto node : nearNodes) {
        double cost = node->cost + distance(node->point, newPoint);
        if (cost < minCost) {
            minCostNode = node;
            minCost = cost;
        }
    }
    return minCostNode;
}

void rewire(std::vector<Node*>& nodes, Node* newNode, double radius, const std::vector<sf::ConvexShape>& obstacles) {
    for (auto node : nodes) {
        if (node == newNode->parent) continue;
        if (distance(node->point, newNode->point) <= radius) {
            double newCost = newNode->cost + distance(newNode->point, node->point);
            if (newCost < node->cost && obstacleFree(newNode->point, node->point, obstacles)) {
                node->parent = newNode;
                node->cost = newCost;
            }
        }
    }
}

std::vector<sf::Vector2f> getPath(Node* goalNode) {
    std::vector<sf::Vector2f> path;
    Node* currentNode = goalNode;
    while (currentNode != nullptr) {
        path.push_back(currentNode->point);
        currentNode = currentNode->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void drawTree(sf::RenderWindow& window, const std::vector<Node*>& nodes) {
    for (const auto& node : nodes) {
        if (node->parent) {
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(node->point.x, node->point.y)),
                sf::Vertex(sf::Vector2f(node->parent->point.x, node->parent->point.y))
            };
            window.draw(line, 2, sf::Lines);
        }
    }
}

void drawPath(sf::RenderWindow& window, const std::vector<sf::Vector2f>& path) {
    for (size_t i = 1; i < path.size(); ++i) {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(path[i-1].x, path[i-1].y), sf::Color::Red),
            sf::Vertex(sf::Vector2f(path[i].x, path[i].y), sf::Color::Red)
        };
        window.draw(line, 2, sf::Lines);
    }
}

void drawObstacles(sf::RenderWindow& window, std::vector<sf::ConvexShape>& obstacles) {
    for (int i = 0; i < obstacles.size(); i++)
    {
        obstacles[i].setFillColor(sf::Color::Black);
        window.draw(obstacles[i]);
    }
}

bool intersection(const sf::ConvexShape polygon, int p, int q, int r, int s)
{
    sf::Vector2f a, b, c, d;
    a = polygon.getPoint(p);
    b = polygon.getPoint(q);
    c = polygon.getPoint(r);
    d = polygon.getPoint(s);

    int o1 = orientation(a, b, c);
    int o2 = orientation(a, b, d);
    int o3 = orientation(c, d, a);
    int o4 = orientation(c, d, b);

    if((o1 != o2) && (o3 != o4))
        return true;
    else return false;
}

bool validPolygon(sf::ConvexShape polygon)
{
    int n = polygon.getPointCount();

    for(int i = 0; i < n; i++)
    {
        for(int j = i+1; j < n; j++)
        {
            int iNext = (i+1)%n;
            int jNext = (j+1)%n;

            if((iNext != j) && (jNext != i) && intersection(polygon,i,iNext,j,jNext))
                return false;
        }
    }

    return true;
}

int main() {
    srand(static_cast<unsigned int>(time(0)));
    double x_max = 800.0, y_max = 600.0;
    double maxStepSize = 10.0;
    double radius = 20.0;
    int maxIterations = 1000;
    int obs, n, k, i, x, y;
    sf::Vector2f pt;
    sf::ConvexShape polygon;

    sf::Vector2f start(50, 50);
    sf::Vector2f goal(750, 550);

    std::vector<sf::ConvexShape> obstacles;

    std::cout << "Enter number of obstacles in space: ";
    std::cin >> n;

    for(int i = 0; i < n; i++)
    {
        std::cout << "Number of vertices in " << i + 1 << "th obstacle: ";
        std::cin >> k;
        polygon.setPointCount(k);

        for(int j = 0; j < k; j++)
        {
            std::cin >> x >> y;
            polygon.setPoint(j, sf::Vector2f(x,y));
        }

        if(k > 3)
        {
            if(validPolygon(polygon))
                obstacles.push_back(polygon);
            else
            {
                std::cout << "Invalid polygon. Re-enter vertices of " << i << "th polygon." << std::endl;
                i--;
            }
        }
    }

    // std::vector<Obstacle> obstacles = {
    //     Obstacle(200, 150, 100, 300),
    //     Obstacle(400, 200, 200, 100),
    //     Obstacle(600, 400, 100, 100)
    // };
    
    std::vector<Node*> nodes;
    nodes.push_back(new Node(start));

    Node* goalNode = nullptr;

    sf::RenderWindow window(sf::VideoMode(x_max, y_max), "RRT* Pathfinding");

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        for (int i = 0; i < 5; ++i) { // Perform several iterations per frame
            sf::Vector2f randomPoint = getRandomPoint(x_max, y_max);
            Node* nearestNode = nearest(nodes, randomPoint);
            sf::Vector2f newPoint = steer(nearestNode->point, randomPoint, maxStepSize);
            if (!obstacleFree(nearestNode->point, newPoint, obstacles)) continue;

            std::vector<Node*> nearNodes = findNearNodes(nodes, newPoint, radius);
            Node* newParent = chooseParent(nearNodes, nearestNode, newPoint);
            Node* newNode = new Node(newPoint, newParent, newParent->cost + distance(newParent->point, newPoint));
            nodes.push_back(newNode);

            rewire(nodes, newNode, radius, obstacles);

            if (distance(newNode->point, goal) < maxStepSize) {
                goalNode = new Node(goal, newNode, newNode->cost + distance(newNode->point, goal));
                nodes.push_back(goalNode);
                break;
            }
        }

        window.clear(sf::Color::White);
        drawTree(window, nodes);
        if (goalNode != nullptr) {
            std::vector<sf::Vector2f> path = getPath(goalNode);
            drawPath(window, path);
        }
        drawObstacles(window, obstacles);
        window.display();
    }

    for (auto node : nodes) {
        delete node;
    }

    return 0;
}
