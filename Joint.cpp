//
// Created by fredd on 18/09/2019.
//

#include "Joint.h"

float Joint::getX() const {
    return x;
}

float Joint::getY() const {
    return y;
}

void Joint::setY(float y) {
    Joint::y = y;
}

float Joint::getZ() const {
    return z;
}

void Joint::setZ(float z) {
    Joint::z = z;
}

void Joint::setX(float x) {
    Joint::x = x;
}
