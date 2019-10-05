#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <future>
#include <list>
#include <ctime>
#include <iterator>
#include <unistd.h>

#include "stb_image.h"

#include "Shader.h"
#include "Camera.h"
#include "Position.h"
#include "utils.h"

#define PI 3.141592653

void framebufferSizeCallback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouseCallback(GLFWwindow* window, double posX, double posY);
void scrollCallback(GLFWwindow* window, double offsetX, double offsetY);

// drawing functions
void drawGrid(Shader* shader, unsigned int gridVAO, unsigned int gridEBO, int numVertices);
void drawCoordSystem(Shader* shader, unsigned int coordVAO, unsigned int coordEBO, int numVertices);
void drawSkeleton(Shader* shader, std::vector<Position*> allInterPos, int skeletonFrame, std::vector<float> colorRGB);
void drawSkeletonRealtime(Shader* shader, std::vector<Joint*> joints, std::vector<float> colorRGB);
void drawSphere(std::vector<GLfloat> color, std::vector<GLdouble> position, float radius);
void drawCube(std::vector<GLfloat> color, std::vector<GLdouble> position, float side);
void drawCylinder(float pHeight, std::vector<float> center1, std::vector<float> center2, float bRadius, float tRadius,
        std::vector<float> color);

// data management functions
std::vector<Position*> getJointPositions(std::string fileName);
std::vector<Position*> getJointPositionsRealtime(std::string fileName);
std::vector<Position*> interpolate(Position* start, Position* end, int times);

// Window settings
const unsigned int WIN_WIDTH = 1920;
const unsigned int WIN_HEIGHT = 1080;

// Camera handling
Camera camera(glm::vec3(0.0f, 5.0f, -14.0f));
double lastX = (float)WIN_WIDTH / 2;
double lastY = (float)WIN_HEIGHT / 2;
bool firstMouse = true;
float fov = 45.0f;

// Time handling
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// a one-position buffer in case the MatLab KinectJointsRealtime.csv file is empty, used to make the skeleton stable.
Position* lastKnownPos;
int skeletonFrame = 0;
GLUquadricObj* quadric = gluNewQuadric();
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

// a flag to decide whether to get realtime data or not
bool realtime = false;

int main(int argcp, char **argv) {

    std::vector<Position*>allInterPos;
    std::vector<Joint*> joints;
    if(!realtime) {
        std::vector<Position *> positions = getJointPositions("../KinectJoints.csv");

        // Smoothing the animation
        int times = 10;
        int currentPos = 0;
        for(int i = 0; i < positions.size() - 1; i++) {
            std::cout << "interpolating between positions " << currentPos << " and " << currentPos + 1 << std::endl;
            std::vector<Position*> interPositions = interpolate(positions[i], positions[i + 1], times);
            for(auto itr = interPositions.begin(); itr != interPositions.end(); itr++) {
                allInterPos.push_back(*itr);
            }
            currentPos++;
        }

        std::cout << "Current number of frames: " << allInterPos.size() << std::endl;

        // since the allInterPos vector contains >> positions than positions, the animation is more fluid even without
        // re-assignment
        joints = allInterPos[skeletonFrame]->getJoints();
    }

    std::vector<Joint*> startingPos;
    for(int i = 0; i < 25; i++) {
        startingPos.push_back(new Joint(3, 3, 3));
    }
    lastKnownPos = new Position(startingPos);

    glutInit(&argcp, argv);
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "Kinect 3D avatar", nullptr, nullptr);

    if(window == nullptr) {
        std::cout << "Failed to create window." << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetCursorPosCallback(window, mouseCallback);
    glfwSetScrollCallback(window, scrollCallback);

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD." << std::endl;
        return -2;
    }

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glEnable(GL_DEPTH_TEST);

    Shader shader("../Shaders/vertexShader.vert", "../Shaders/fragmentShader.frag");

    float currentFrame;

    float gridVertices[] = {

            // position          // color
            0.0f, 0.0f, 0.0f,    1.0f, 1.0f, 1.0f,
            0.25f, 0.0f, 0.0f,    1.0f, 1.0f, 1.0f,
            0.25f, 0.0f, 0.25f,    1.0f, 1.0f, 1.0f,
            0.0f, 0.0f, 0.25f,    1.0f, 1.0f, 1.0f

    };

    unsigned int gridIndices[] = {

            0, 1, 2, 3

    };

    float coordSystemVertices[] = {

            -1.0f, 0.0f, 0.0f,    0.0f, 0.0f, 1.0f,    // X-axis -- BLUE
            4.0f, 0.0f, 0.0f,    0.0f, 0.0f, 1.0f,

            0.0f, -1.0f, 0.0f,    1.0f, 0.0f, 0.0f,    // Y-axis -- RED
            0.0f, 4.0f, 0.0f,    1.0f, 0.0f, 0.0f,

            0.0f, 0.0f, -1.0f,    0.0f, 1.0f, 0.0f,    // Z-axis -- GREEN
            0.0f, 0.0f, 4.0f,    0.0f, 1.0f, 0.0f,

            3.75f, -0.2f, 0.0f,    0.0f, 0.0f, 1.0f,    // X-arrow
            3.75f, 0.2, 0.0f,    0.0f, 0.0f, 1.0f,

            0.1f, 3.75f, -0.1f,    1.0f, 0.0f, 0.0f,    // Y-arrow
            -0.1f, 3.75f, 0.1f,    1.0f, 0.0f, 0.0f,

            0.0f, -0.2f, 3.75f,    0.0f, 1.0f, 0.0f,    // Z-arrow
            0.0f, 0.2f, 3.75f,    0.0f, 1.0f, 0.0f,

    };

    unsigned int coordIndices[] = {

            0, 1,
            2, 3,
            4, 5,
            1, 7,
            1, 6,
            3, 8,
            3, 9,
            5, 10,
            5, 11

    };

    // --------------------- GRID ------------------------------

    unsigned int gridVAO, gridVBO, gridEBO;
    glGenVertexArrays(1, &gridVAO);
    glBindVertexArray(gridVAO);

    glGenBuffers(1, &gridVBO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gridVertices), gridVertices, GL_STATIC_DRAW);

    glGenBuffers(1, &gridEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gridEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(gridIndices), gridIndices, GL_STATIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // ---------------------- COORDINATE SYSTEM ------------------------

    unsigned int coordVAO, coordVBO, coordEBO;
    glGenVertexArrays(1, &coordVAO);
    glBindVertexArray(coordVAO);

    glGenBuffers(1, &coordVBO);
    glBindBuffer(GL_ARRAY_BUFFER, coordVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(coordSystemVertices), coordSystemVertices, GL_STATIC_DRAW);

    glGenBuffers(1, &coordEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, coordEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(coordIndices), coordIndices, GL_STATIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glPointSize(15.0f);

    double timerStart, timerEnd;
    float distance, bRadius, tRadius;
    std::vector<float> color;

    glm::vec3 pos = camera.Position;

    while(!glfwWindowShouldClose(window)) {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // NON REALTIME ANIMATION
        if(!realtime) {
            timerStart = glfwGetTime();
            joints = allInterPos[skeletonFrame % allInterPos.size()]->getJoints();
        }


        // REALTIME ANIMATION! To use it, you will need:
        // https://it.mathworks.com/matlabcentral/fileexchange/53439-kinect-2-interface-for-matlab
        // Just copy the videoDemo.m and videoDemoWithWindows.m scripts inside the project's folder and run one of them
        else {
            std::vector<Position*> positionsRealtime = getJointPositionsRealtime("../KinectJointsRealtime.csv");
            try {
                joints = positionsRealtime[0]->getJoints();
            }catch(std::exception &e) {
                std::cout << "Exception caught: " << e.what() << std::endl;
            }
        }

        currentFrame = (float) glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        drawGrid(&shader, gridVAO, gridEBO, sizeof(gridIndices));

        // Since we need a bunch of different spheres (depending on the joint we're using), we need to use this bad
        // if-else statment...
        for(int i = 0; i < joints.size(); i++) {
            if(i != 1 && i != 2 && i != 3 && i != 22 && i != 24 && i != 11 && i != 7 && i != 21 && i != 23 && i != 6 && i != 10) {
                drawSphere({0.1, 0.1, 0.7},
                           {joints[i]->getX() + 6, joints[i]->getY() + (float) 2.5, joints[i]->getZ() + 2}, 0.15);
            }
            else if(i == 3) {
                drawSphere({0.1, 0.1, 0.7},
                           {joints[i]->getX() + 6, joints[i]->getY() + (float) 2.5, joints[i]->getZ() + 2}, 0.3);
            }
            else if(i == 22 || i == 24 || i == 11 || i == 7 || i == 21 || i == 23 || i == 6 || i == 10) {
                drawSphere({0.1, 0.1, 0.7},
                           {joints[i]->getX() + 6, joints[i]->getY() + (float) 2.5, joints[i]->getZ() + 2}, 0.075);
            }
            else if(i == 0 || i == 1) {
                drawSphere({0.0, 1.0, 0.0},
                           {joints[i]->getX() + 6, joints[i]->getY() + (float) 2.5, joints[i]->getZ() + 2}, 0.1);
            }
            else {
                drawSphere({1.0, 1.0, 0.0},
                           {joints[i]->getX() + 6, joints[i]->getY() + (float) 2.5, joints[i]->getZ() + 2}, 0.1);
            }
        }

        // ... and the same goes for the cylinders
        for(int j = 0; j < 48; j += 2) {
            distance = (float)sqrt(pow(joints[skeletonIndices[j + 1]]->getX() - joints[skeletonIndices[j]]->getX(), 2) +
                    pow(joints[skeletonIndices[j + 1]]->getY() - joints[skeletonIndices[j]]->getY(), 2) +
                    pow(joints[skeletonIndices[j + 1]]->getZ() - joints[skeletonIndices[j]]->getZ(), 2));
            if(skeletonIndices[j + 1] == 6 || skeletonIndices[j + 1] == 10) {
                bRadius = 0.05f;
                tRadius = 0.1f;
            }
            else if(skeletonIndices[j + 1] == 11 || skeletonIndices[j + 1] == 24 || skeletonIndices[j + 1] == 23 ||
                    skeletonIndices[j + 1] == 7 || skeletonIndices[j + 1] == 21 || skeletonIndices[j + 1] == 22) {
                bRadius = 0.05f;
                tRadius = 0.05f;
            }
            else if(skeletonIndices[j + 1] == 17 || skeletonIndices[j + 1] == 13) {
                bRadius = 0.115f;
                tRadius = 0.15f;
            }
            else if(skeletonIndices[j + 1] == 14 || skeletonIndices[j + 1] == 18) {
                bRadius = 0.09f;
                tRadius = 0.115f;
            }
            else{
                bRadius = 0.1f;
                tRadius = 0.1f;
            }
            if(skeletonIndices[j + 1] == 4 || skeletonIndices[j + 1] == 8 || skeletonIndices[j + 1] == 12 ||
                    skeletonIndices[j + 1] == 16 || skeletonIndices[j + 1] == 1 || skeletonIndices[j + 1] == 0) {
                color = {0.0f, 1.0f, 0.0f};
            }
            else {
                color = {1.0f, 1.0f, 0.0f};
            }
            drawCylinder(distance, joints[skeletonIndices[j]]->getCoordinates(), joints[skeletonIndices[j + 1]]->getCoordinates(),
                    bRadius, tRadius, color);
            distance = (float)sqrt(pow(joints[8]->getX() - joints[16]->getX(), 2) +
                                  pow(joints[8]->getY() - joints[16]->getY(), 2) +
                                  pow(joints[8]->getZ() - joints[16]->getZ(), 2));
            drawCylinder(distance, joints[8]->getCoordinates(), joints[16]->getCoordinates(), 0.1f, 0.1f, {0.0f, 1.0f, 0.0f});
            distance = (float)sqrt(pow(joints[4]->getX() - joints[12]->getX(), 2) +
                                   pow(joints[4]->getY() - joints[12]->getY(), 2) +
                                   pow(joints[4]->getZ() - joints[12]->getZ(), 2));
            drawCylinder(distance, joints[4]->getCoordinates(), joints[12]->getCoordinates(), 0.1f, 0.1f, {0.0f, 1.0f, 0.0f});
        }

        drawCoordSystem(&shader, coordVAO, coordEBO, sizeof(coordIndices));

        // Draw the skeleton in the correct way
        if(!realtime) {
            drawSkeleton(&shader, allInterPos, skeletonFrame % allInterPos.size(), {1.0f, 0.0f, 0.0f});
        }
        else {
            drawSkeletonRealtime(&shader, joints, {1.0f, 0.0f, 0.0f});
        }

        glfwSwapBuffers(window);
        glfwPollEvents();

        // Time handling: this way, it should move at around 60 FPS
        if(!realtime) {
            timerEnd = glfwGetTime();
            if (timerEnd - timerStart < 1 / (double)60) {
                sleep(1 - (timerEnd - timerStart));
                skeletonFrame++;
            }

        }

    }

    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteBuffers(1, &gridEBO);
    glDeleteVertexArrays(1, &coordVAO);
    glDeleteBuffers(1, &coordVBO);
    glDeleteBuffers(1, &coordEBO);

    glfwTerminate();
    return 0;

}

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {

    glViewport(0, 0, width, height);

}

void mouseCallback(GLFWwindow* window, double posX, double posY) {

    if(firstMouse) {
        lastX = posY;
        lastY = posY;
        firstMouse = false;
    }

    double offsetX = posX - lastX;
    double offsetY = lastY - posY;
    lastX = posX;
    lastY = posY;

    camera.ProcessMouseMovement((float)offsetX, (float)offsetY);

}
void scrollCallback(GLFWwindow* window, double offsetX, double offsetY) {

    camera.ProcessMouseScroll((float)offsetY);

}

void processInput(GLFWwindow* window) {

    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        camera.ProcessKeyboard(FORWARD, deltaTime);
    }
    if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        camera.ProcessKeyboard(LEFT, deltaTime);
    }
    if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    }
    if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        camera.ProcessKeyboard(RIGHT, deltaTime);
    }

}
