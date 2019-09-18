//
// Created by fredd on 18/09/2019.
//

#ifndef INC_3D_AVATAR_POSITION_H
#define INC_3D_AVATAR_POSITION_H


#include <vector>
#include "Joint.h"

class Position {

private:

    std::vector<Joint*> joints;

public:

    Position() {

        // stuff...?

    }

    void add(Joint* j) {
        this->joints.push_back(j);
    }

    const std::vector<Joint *> &getJoints() const;

    void setJoints(const std::vector<Joint *> &joints);

};


#endif //INC_3D_AVATAR_POSITION_H
