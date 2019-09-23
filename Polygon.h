//
// Created by fredd on 22/09/2019.
//

#ifndef INC_3D_AVATAR_POLYGON_H
#define INC_3D_AVATAR_POLYGON_H


#include "Joint.h"

class Polygon {

private:

    std::vector<std::vector<float>> vertices;
    std::vector<float> color;
    std::vector<float> center;
    int numSides;

public:

    Polygon(std::vector<std::vector<float>> vertices, int numSides, std::vector<float> center, std::vector<float> color);

    const std::vector<std::vector<float>> &getVertices() const;

    void setVertices(const std::vector<std::vector<float>> &vertices);

    const std::vector<float> &getColor() const;

    void setColor(const std::vector<float> &color);

    const std::vector<float> &getCenter() const;

    void setCenter(const std::vector<float> &center);

    int getNumSides() const;

    void setNumSides(int numSides);

};


#endif //INC_3D_AVATAR_POLYGON_H
