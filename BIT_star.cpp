#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <SFML/Graphics.hpp>

struct Node {
    float x, y;
    float cost;
    Node* parent;

    Node(float x, float y) : x(x), y(y), cost(std::numeric_limits<float>::infinity()), parent(nullptr) {}
};

struct Obstacle {
    float x, y, width, height;

    Obstacle(float x, float y, float width, float height) : x(x), y(y), width(width), height(height) {}
};

float distance(Node* a, Node* b) {
    return std::sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

bool isCollision(Node* a, Node* b, const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        float minX = std::min(a->x, b->x);
        float maxX = std::max(a->x, b->x);
        float minY = std::min(a->y, b->y);
        float maxY = std::max(a->y, b->y);

        if (maxX > obstacle.x && minX < obstacle.x + obstacle.width &&
            maxY > obstacle.y && minY < obstacle.y + obstacle.height) {
            return true;
        }
    }
    return false;
}

std::vector<Node*> generateSamples(int num_samples, float width, float height) {
    std::vector<Node*> samples;
    for (int i = 0; i < num_samples; ++i) {
        samples.push_back(new Node(rand() % int(width), rand() % int(height)));
    }
    return samples;
}

Node* nearestNeighbor(Node* query, std::vector<Node*>& nodes) {
    Node* nearest = nullptr;
    float min_dist = std::numeric_limits<float>::infinity();
    for (auto node : nodes) {
        float dist = distance(query, node);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }
    return nearest;
}

void bitStar(std::vector<Node*>& samples, Node* start, Node* goal, const std::vector<Obstacle>& obstacles) {
    std::priority_queue<std::pair<float, Node*>, std::vector<std::pair<float, Node*>>, std::greater<>> queue;
    start->cost = 0;
    queue.push({distance(start, goal), start});
    
    while (!queue.empty()) {
        Node* current = queue.top().second;
        queue.pop();

        if (current == goal) {
            break;
        }

        for (auto sample : samples) {
            if (!isCollision(current, sample, obstacles)) {
                float new_cost = current->cost + distance(current, sample);
                if (new_cost < sample->cost) {
                    sample->cost = new_cost;
                    sample->parent = current;
                    queue.push({new_cost + distance(sample, goal), sample});
                }
            }
        }
    }
}

std::vector<Node*> reconstructPath(Node* goal) {
    std::vector<Node*> path;
    Node* current = goal;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int main() {
    float width = 800, height = 600;
    Node* start = new Node(50, 50);
    Node* goal = new Node(750, 550);
    int num_samples = 100;
    std::vector<Node*> samples = generateSamples(num_samples, width, height);

    std::vector<Obstacle> obstacles = {
        {300, 200, 100, 100},
        {500, 400, 150, 50}
    };

    samples.push_back(start);
    samples.push_back(goal);

    bitStar(samples, start, goal, obstacles);

    std::vector<Node*> path = reconstructPath(goal);

    sf::RenderWindow window(sf::VideoMode(width, height), "BIT* Pathfinding with Obstacles");
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        window.clear();

        sf::CircleShape startShape(5);
        startShape.setFillColor(sf::Color::Green);
        startShape.setPosition(start->x - 5, start->y - 5);
        window.draw(startShape);

        sf::CircleShape goalShape(5);
        goalShape.setFillColor(sf::Color::Red);
        goalShape.setPosition(goal->x - 5, goal->y - 5);
        window.draw(goalShape);

        for (const auto& obstacle : obstacles) {
            sf::RectangleShape obstacleShape(sf::Vector2f(obstacle.width, obstacle.height));
            obstacleShape.setFillColor(sf::Color::Blue);
            obstacleShape.setPosition(obstacle.x, obstacle.y);
            window.draw(obstacleShape);
        }

        for (auto sample : samples) {
            sf::CircleShape sampleShape(2);
            sampleShape.setFillColor(sf::Color::White);
            sampleShape.setPosition(sample->x - 2, sample->y - 2);
            window.draw(sampleShape);
        }

        for (size_t i = 0; i < path.size() - 1; ++i) {
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(path[i]->x, path[i]->y)),
                sf::Vertex(sf::Vector2f(path[i + 1]->x, path[i + 1]->y))
            };
            window.draw(line, 2, sf::Lines);
        }

        window.display();
    }

    for (auto sample : samples) {
        delete sample;
    }

    return 0;
}
