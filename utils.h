#include <vector>
#include <algorithm>
#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <fstream>
#include <sstream>

#include "stb_image.h"

#include "Shader.h"
#include "Camera.h"
#include "Position.h"

#ifndef PI
#define PI 3.141592653
#endif

extern Camera camera;
extern float fov;
extern const unsigned int WIN_WIDTH;
extern const unsigned int WIN_HEIGHT;

extern Position* lastKnownPos;
extern GLUquadricObj* quadric;
/*std::vector<std::vector<double>> getJointPositions(std::string fileName) {

    std::fstream fin;
    fin.open(fileName, std::ios::in);
    std::vector<std::string> row;
    std::string line, word, temp;
    std::vector<std::vector<double>> positions;

    while(fin >> temp) {

        row.clear();
        getline(fin, line);
        std::stringstream s(line);
        while(getline(s, word, ' ')) {

            row.push_back(word);

        }

        for(int i = 0; i < row.size() / 3; i += 3) {

            positions.push_back({std::stod(row[i]), std::stod(row[i + 1]), std::stod(row[i + 2])});

        }

    }

    return positions;

}*/

void drawCylinder(float pHeight, std::vector<float> center1, std::vector<float> center2, float bRadius, float tRadius,
                  std::vector<float> color) {

    const GLfloat* projection = glm::value_ptr(glm::perspective(glm::radians(fov), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 100.0f));
    const GLfloat* view = glm::value_ptr(camera.GetViewMatrix());

    glm::vec3 z = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 diff = glm::vec3(center1[0] - center2[0], center1[1] - center2[1], center1[2] - center2[2]);
    glm::vec3 cross = glm::cross(z, diff);

    double angle = 180 / PI * acos(glm::dot(z, diff) / glm::length(diff));

    glUseProgram(0);

    glPushMatrix();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projection);

    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor3f(color[0], color[1], color[2]);
    glTranslated(center2[0] - 12.25f, (center2[1]) + 0.0f, (center2[2]) - 12.25f);
    glRotated(angle, cross.x, cross.y, cross.z);
    gluQuadricOrientation(quadric, GLU_OUTSIDE);
    gluCylinder(quadric, bRadius, tRadius, pHeight, 32, 32);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

}

void drawSkeleton(Shader* shader, std::vector<Position*> allInterPos, int skeletonFrame, std::vector<float> colorRGB) {

    shader->use();

    // since the allInterPos vector contains >> positions than positions, the animation is more fluid even without
    // re-assignment
    std::vector<Joint*> joints = allInterPos[skeletonFrame]->getJoints();

    float skeletonVertices[joints.size() * 6];
    int current = 0;

    // This kind of offsets were the fastest way to get a constant translation in the middle of the grid
    for(int i = 0; i < joints.size() * 6; i += 6) {
        skeletonVertices[i] = joints[current]->getX() + 6;
        skeletonVertices[i + 1] = joints[current]->getY() + (float)2.5;
        skeletonVertices[i + 2] = joints[current]->getZ() + 2;
        skeletonVertices[i + 3] = colorRGB[0];
        skeletonVertices[i + 4] = colorRGB[1];
        skeletonVertices[i + 5] = colorRGB[2];
        current++;
    }

    unsigned int skeletonIndices[] = {

            3, 2,        // HEAD - NECK
            2, 20,       // NECK - SPINE

            20, 4,       // SPINE - SHOULDER_LEFT
            4, 5,        // SHOULDER_LEFT - ELBOW__LEFT
            5, 6,        // ELBOW_LEFT - WRIST_LEFT
            6, 22,       // WRIST_LEFT - THUMB_LEFT
            6, 7,        // WRIST_LEFT - HAND_LEFT
            7, 21,       // HAND_LEFT - HANF_TIP_LEFT

            20, 8,       // SPINE - SHOULDER_RIGHT
            8, 9,        // SHOULDER_RIGHT - ELBOW_RIGHT
            9, 10,       // ELBOW_RIGHT - WRIST_RIGHT
            10, 24,      // WRIST_RIGHT - THUMB_RIGHT
            10, 11,      // WRIST_RIGHT - HAND_RIGHT
            11, 23,      // HAND_RIGHT - HAND_TIP_RIGHT

            20, 1,       // SPINE - SPINE_MID
            1, 0,        // SPINE_MID - SPINE_BASE

            0, 12,       // SPINE_BASE - HIP_LEFT
            12, 13,      // HIP_LEFT - KNEE_LEFT
            13, 14,      // KNEE_LEFT - ANKLE_LEFT
            14, 15,      // ANKLE_LEFT - FOOT_LEFT

            0, 16,       // SPINE_BASE - HIP_RIGHT
            16, 17,      // HIP_RIGHT - KNEE_RIGHT
            17, 18,      // KNEE_RIGHT - ANKLE_RIGHT
            18, 19,      // ANKLE_RIGHT - FOOT_RIGHT

    };

    unsigned int skeletonVAO, skeletonVBO, skeletonEBO;
    glGenVertexArrays(1, &skeletonVAO);
    glBindVertexArray(skeletonVAO);

    glGenBuffers(1, &skeletonVBO);
    glBindBuffer(GL_ARRAY_BUFFER, skeletonVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skeletonVertices), skeletonVertices, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &skeletonEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, skeletonEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(skeletonIndices), skeletonIndices, GL_DYNAMIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glLineWidth(20.0f);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(-12.5f, 0.0f, -12.5f));
    shader->setMat4("modelSkeleton", model);

    glDrawElements(GL_LINES, sizeof(skeletonVertices), GL_UNSIGNED_INT, nullptr);

}

void drawSkeletonRealtime(Shader* shader, std::vector<Joint*> joints, std::vector<float> colorRGB) {

    shader->use();

    // This kind of offsets were the fastest way to get a constant translation in the middle of the grid
    float skeletonVertices[joints.size() * 6];
    int current = 0;

    // This kind of offsets were the fastest way to get a constant translation in the middle of the grid
    for(int i = 0; i < joints.size() * 6; i += 6) {
        skeletonVertices[i] = joints[current]->getX() + 6;
        skeletonVertices[i + 1] = joints[current]->getY() + (float)2.5;
        skeletonVertices[i + 2] = joints[current]->getZ() + 2;
        skeletonVertices[i + 3] = colorRGB[0];
        skeletonVertices[i + 4] = colorRGB[1];
        skeletonVertices[i + 5] = colorRGB[2];
        current++;
    }

    unsigned int skeletonIndices[] = {

            3, 2,        // HEAD - NECK
            2, 20,       // NECK - SPINE

            20, 4,       // SPINE - SHOULDER_LEFT
            4, 5,        // SHOULDER_LEFT - ELBOW__LEFT
            5, 6,        // ELBOW_LEFT - WRIST_LEFT
            6, 22,       // WRIST_LEFT - THUMB_LEFT
            6, 7,        // WRIST_LEFT - HAND_LEFT
            7, 21,       // HAND_LEFT - HANF_TIP_LEFT

            20, 8,       // SPINE - SHOULDER_RIGHT
            8, 9,        // SHOULDER_RIGHT - ELBOW_RIGHT
            9, 10,       // ELBOW_RIGHT - WRIST_RIGHT
            10, 24,      // WRIST_RIGHT - THUMB_RIGHT
            10, 11,      // WRIST_RIGHT - HAND_RIGHT
            11, 23,      // HAND_RIGHT - HAND_TIP_RIGHT

            20, 1,       // SPINE - SPINE_MID
            1, 0,        // SPINE_MID - SPINE_BASE

            0, 12,       // SPINE_BASE - HIP_LEFT
            12, 13,      // HIP_LEFT - KNEE_LEFT
            13, 14,      // KNEE_LEFT - ANKLE_LEFT
            14, 15,      // ANKLE_LEFT - FOOT_LEFT

            0, 16,       // SPINE_BASE - HIP_RIGHT
            16, 17,      // HIP_RIGHT - KNEE_RIGHT
            17, 18,      // KNEE_RIGHT - ANKLE_RIGHT
            18, 19,      // ANKLE_RIGHT - FOOT_RIGHT

    };

    unsigned int skeletonVAO, skeletonVBO, skeletonEBO;
    glGenVertexArrays(1, &skeletonVAO);
    glBindVertexArray(skeletonVAO);

    glGenBuffers(1, &skeletonVBO);
    glBindBuffer(GL_ARRAY_BUFFER, skeletonVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skeletonVertices), skeletonVertices, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &skeletonEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, skeletonEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(skeletonIndices), skeletonIndices, GL_DYNAMIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glLineWidth(7.0f);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(-12.5f, 0.0f, -12.5f));
    shader->setMat4("modelSkeleton", model);

    glDrawElements(GL_LINES, sizeof(skeletonVertices), GL_UNSIGNED_INT, nullptr);

}

void drawCoordSystem(Shader* shader, unsigned int coordVAO, unsigned int coordEBO, int numVertices) {

    shader->use();

    glLineWidth(5.0f);

    glBindVertexArray(coordVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, coordEBO);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glDrawElements(GL_LINES, numVertices, GL_UNSIGNED_INT, nullptr);

}

std::vector<Position*> getJointPositions(std::string fileName) {

    std::fstream fin;
    fin.open(fileName, std::ios::in);
    std::vector<std::string> row;
    std::string line, word, temp;
    std::vector<Position*> positions;
    int numRow = 0;

    while(fin >> temp) {

        // Useful data are at row 1, other data that may be useful are in the next two rows
        if( numRow == 1 ) {
            row.clear();
            line = temp;
            std::stringstream s(line);
            while(std::getline(s, word, ',')) {
                row.push_back(word);
            }

            if (!s && word.empty())
            {
                row.push_back("");
            }

            for(int i = 0; i < row.size(); i += 75) {
                auto* position = new Position();
                for(int j = 0; j < 75; j += 3) {
                    // Doubling to make the skeleton bigger and hence more visible
                    position->add(new Joint(2 * std::stof(row[i + j]),
                                            2 * std::stof(row[i + j + 1]),
                                            2 * std::stof(row[i + j + 2])));
                }
                positions.push_back(position);
            }

        }
        numRow++;

    }

    return positions;

}

std::vector<Position*> getJointPositionsRealtime(std::string fileName) {

    std::fstream fin;
    fin.open(fileName, std::ios::in);
    std::vector<std::string> row;
    std::string line, word, temp;
    std::vector<Position*> positions;
    int numRow = 0;

    while(fin >> temp) {

        // Useful data are at row 1, other data that may be useful are in the next two rows

        if( numRow == 1 ) {
            row.clear();
            line = temp;
            std::stringstream s(line);
            while(std::getline(s, word, ',')) {
                row.push_back(word);
            }

            if (!s && word.empty())
            {
                row.push_back("");
            }

            for(int i = 0; i < row.size(); i += 75) {
                auto position = new Position();
                for(int j = 0; j < 75; j += 3) {
                    // Doubling to make the skeleton bigger and hence more visible
                    try {
                        position->add(new Joint(2 * (float) std::stod(row[i + j]),
                                                2 * (float) std::stod(row[i + j + 1]),
                                                2 * (float) std::stod(row[i + j + 2])));
                    } catch(std::exception e) {
                        e.what();
                    }
                }
                positions.push_back(position);
                if(position->getJointsSize() != 0) {
                    lastKnownPos = position;
                }
            }

        }
        numRow++;

    }

    if(positions.empty()) {
        if(!lastKnownPos) {
            std::vector<Joint*> joints;
            for(int i = 0; i < 25; i++) {
                joints.push_back(new Joint(3, 3, 3));
            }
            positions.push_back(new Position(joints));
            return positions;
        }
        positions.push_back(lastKnownPos);
    }

    return positions;

}

std::vector<Position*> interpolate(Position* start, Position* end, int times) {

    std::vector<Joint*> interPos;
    std::vector<Position*> result;
    std::vector<Joint*> startJoints = start->getJoints();
    std::vector<Joint*> endJoints = end->getJoints();

    // loop on the joints
    for(int i = 0; i < 25; i++) {
        // loop on the number of interpolations
        for (int j = 1; j < times; j++) {
            interPos.push_back(new Joint(startJoints[i]->getX() + (endJoints[i]->getX() - startJoints[i]->getX()) * j / times,
                                         startJoints[i]->getY() + (endJoints[i]->getY() - startJoints[i]->getY()) * j / times,
                                         startJoints[i]->getZ() + (endJoints[i]->getZ() - startJoints[i]->getZ()) * j / times));
        }
    }

    std::vector<Joint*> currentPosJoints;
    for(int k = 0; k < times - 1; k++) {
        for (auto itr = std::next(interPos.begin(), k); itr < interPos.end(); itr += times - 1) {
            currentPosJoints.push_back(*itr);
        }
        result.push_back(new Position(currentPosJoints));
        currentPosJoints.clear();
    }

    return result;
}

void drawGrid(Shader* shader, unsigned int gridVAO, unsigned int gridEBO, int numVertices) {

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLineWidth(3.0f);

    shader->use();

    glm::mat4 projection = glm::perspective(glm::radians(fov), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 100.0f);
    shader->setMat4("projection", projection);

    glm::mat4 view = camera.GetViewMatrix();
    shader->setMat4("view", view);

    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gridEBO);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for(int i = 0; i < 50; i++) {
        for(int j = 0; j < 50; j++) {
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, glm::vec3(-0.25 * i, 0.0f, -0.25f * j));
            shader->setMat4("model", model);
            glDrawElements(GL_LINE_LOOP, numVertices, GL_UNSIGNED_INT, nullptr);
        }
    }
}

void drawSphere(std::vector<GLfloat> color, std::vector<GLdouble> position, float radius) {

    const GLfloat* projection = glm::value_ptr(glm::perspective(glm::radians(fov), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 100.0f));
    const GLfloat* view = glm::value_ptr(camera.GetViewMatrix());

    glUseProgram(0);

    glPushMatrix();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projection);

    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor3f(color[0], color[1], color[2]);

    glTranslated(position[0] - 12.25f, position[1], position[2] - 12.25f);
    glutSolidSphere(radius, 50, 50);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

}

void drawCube(std::vector<GLfloat> color, std::vector<GLdouble> position, float side) {

    const GLfloat* projection = glm::value_ptr(glm::perspective(glm::radians(fov), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 100.0f));
    const GLfloat* view = glm::value_ptr(camera.GetViewMatrix());

    glUseProgram(0);

    glPushMatrix();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projection);

    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(view);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glColor3f(color[0], color[1], color[2]);

    glTranslated(position[0] - 12.25f, position[1], position[2] - 12.25f);
    glutSolidCube(side);
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

}