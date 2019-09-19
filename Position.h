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

    explicit Position(std::vector<Joint*> joints) {

        for(auto itr = joints.begin(); itr != joints.end(); itr++) {
            this->joints.push_back(*itr);
        }

    }

    void add(Joint* j) {
        this->joints.push_back(j);
    }

    const std::vector<Joint *> &getJoints() const;

    void setJoints(const std::vector<Joint *> &joints);

};


#endif //INC_3D_AVATAR_POSITION_H
