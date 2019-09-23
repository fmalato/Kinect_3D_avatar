//
// Created by fredd on 22/09/2019.
//

#include "Polygon.h"

Polygon::Polygon(std::vector<std::vector<float>> vertices, int numSides, std::vector<float> center, std::vector<float> color) {

    for(auto itr = vertices.begin(); itr != vertices.end(); itr++) {
        this->vertices.push_back(*itr);
    }
    this->numSides = numSides;
    this->center = center;
    this->color = color;

}

const std::vector<std::vector<float>> &Polygon::getVertices() const {
    return vertices;
}

void Polygon::setVertices(const std::vector<std::vector<float>> &vertices) {
    Polygon::vertices = vertices;
}

const std::vector<float> &Polygon::getColor() const {
    return color;
}

void Polygon::setColor(const std::vector<float> &color) {
    Polygon::color = color;
}

const std::vector<float> &Polygon::getCenter() const {
    return center;
}

void Polygon::setCenter(const std::vector<float> &center) {
    Polygon::center = center;
}

int Polygon::getNumSides() const {
    return numSides;
}

void Polygon::setNumSides(int numSides) {
    Polygon::numSides = numSides;
}
