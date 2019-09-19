//
// Created by fredd on 18/09/2019.
//

#ifndef INC_3D_AVATAR_JOINT_H
#define INC_3D_AVATAR_JOINT_H

#include <vector>

class Joint {

private:
    float x;
    float y;
    float z;

public:

    Joint(float x, float y, float z) {

        this->x = x;
        this->y = y;
        this->z = z;

    }
    float getX() const;

    void setX(float x);

    float getY() const;

    void setY(float y);

    float getZ() const;

    void setZ(float z);

};


#endif //INC_3D_AVATAR_JOINT_H
