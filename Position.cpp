//
// Created by fredd on 18/09/2019.
//

#include "Position.h"

const std::vector<Joint *> &Position::getJoints() const {
    return joints;
}

void Position::setJoints(const std::vector<Joint *> &joints) {
    Position::joints = joints;
}
