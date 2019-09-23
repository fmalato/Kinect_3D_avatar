#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

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
// #include "Polygon.h"

#define PI 3.141592653

void framebufferSizeCallback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouseCallback(GLFWwindow* window, double posX, double posY);
void scrollCallback(GLFWwindow* window, double offsetX, double offsetY);

// drawing functions
void drawGrid(Shader* shader, unsigned int gridVAO, unsigned int gridEBO, int numVertices);
void drawCube(Shader* shader, std::vector<float> center, float sideLength, std::vector<float> colorRGB);
void drawCoordSystem(Shader* shader, unsigned int coordVAO, unsigned int coordEBO, int numVertices);
void drawSkeleton(Shader* shader, std::vector<Position*> allInterPos, int skeletonFrame, std::vector<float> colorRGB);
void drawSkeletonRealtime(Shader* shader, std::vector<Joint*> joints, std::vector<float> colorRGB);
/*Polygon* drawPolygon2D(Shader* shader, float radius, int numSides,  bool rotation, std::vector<float> rotationAxis,
        float rotationAngle, std::vector<float> center, std::vector<float> colorRGB);*/
void drawSphere(Shader* shader, float radius, std::vector<float> center, std::vector<float> colorRGB);
//void linkTwoPolygons(Shader* shader, Polygon* p1, Polygon* p2, int numSides);

// data management functions
std::vector<Position*> getJointPositions(std::string fileName);
std::vector<Position*> getJointPositionsRealtime(std::string fileName);
std::vector<Position*> interpolate(Position* start, Position* end, int times);

// Window settings
const unsigned int WIN_WIDTH = 1920;
const unsigned int WIN_HEIGHT = 1080;

// Camera handling
Camera camera(glm::vec3(0.0f, 5.0f, 3.0f));
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

// a flag to decide whether to get realtime data or not
bool realtime = false;

int main(int argcp, char **argv) {

    std::vector<Position*>allInterPos;
    std::vector<Joint*> joints;
    if(!realtime) {
        std::vector<Position *> positions = getJointPositions("../KinectJoints.csv");

        // Smoothing the animation
        int times = 8;
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
    Shader polygonShader("../Shaders/polygonVertShader.vert", "../Shaders/polygonFragShader.frag");

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

    shader.use();

    double timerStart, timerEnd;

    while(!glfwWindowShouldClose(window)) {

        // NON REALTIME ANIMATION
        /** If you don't want a realtime animation, uncomment this and comment the 3* parts **/
        if(!realtime) {
            timerStart = glfwGetTime();
            joints = allInterPos[skeletonFrame % allInterPos.size()]->getJoints();
        }
        /** Comment this far **/

        // REALTIME ANIMATION! To use it, you will need:
        // https://it.mathworks.com/matlabcentral/fileexchange/53439-kinect-2-interface-for-matlab
        // Just copy the videoDemo.m and videoDemoWithWindows.m scripts inside the project's folder and run one of them
        /*** If you want a realtime animation, uncomment this and comment the 2* parts ***/
        else {
            std::vector<Position*> positionsRealtime = getJointPositionsRealtime("../KinectJointsRealtime.csv");
            try {
                joints = positionsRealtime[0]->getJoints();
            }catch(std::exception &e) {
                std::cout << "Exception caught: " << e.what() << std::endl;
            }
        }
        /*** Comment this far ***/

        currentFrame = (float) glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        drawGrid(&shader, gridVAO, gridEBO, sizeof(gridIndices));
        for(int i = 0; i < joints.size(); i++) {
            drawCube(&shader, {joints[i]->getX() + 6, joints[i]->getY() + (float) 2.5, joints[i]->getZ() + 2}, 0.10, {1.0, 1.0, 1.0});
            // drawSphere(&polygonShader, 0.1f, {joints[i]->getX() + 6, joints[i]->getY() + (float) 2.5, joints[i]->getZ() + 2}, {0.0f, 0.0f, 1.0f});
        }
        drawCoordSystem(&shader, coordVAO, coordEBO, sizeof(coordIndices));
        if(!realtime) {
            drawSkeleton(&shader, allInterPos, skeletonFrame % allInterPos.size(), {1.0f, 0.0f, 0.0f});
        }
        else {
            drawSkeletonRealtime(&shader, joints, {1.0f, 0.0f, 0.0f});
        }

        /*drawPolygon2D(&shader, 0.15, 20, false, {0.0f, 0.0f, 0.0f}, 0.0f,
                {joints[11]->getX() + 6, joints[11]->getY() + (float) 2.5, joints[11]->getZ() + 2}, {0.0f, 1.0f, 0.0f});
        drawPolygon2D(&shader, 0.15, 20, false, {0.0f, 0.0f, 0.0f}, 0.0f,
                {joints[7]->getX() + 6, joints[7]->getY() + (float) 2.5, joints[7]->getZ() + 2}, {0.0f, 1.0f, 0.0f});*/
        // drawSphere(&polygonShader, 1.0, {1.0f, 1.0f, 1.0f}, {0.0f, 0.0f, 1.0f});

        glColor3f(1.0f, 0.0f, 0.0f);
        glPushMatrix();
        glTranslated(2.0, 2.0, 2.0);
        glutSolidSphere(1, 50, 50);
        glPopMatrix();

        glfwSwapBuffers(window);
        glfwPollEvents();

        /** If you don't want a realtime animation, uncomment this and comment the 3* parts **/
        if(!realtime) {
            timerEnd = glfwGetTime();
            if (timerEnd - timerStart < 1 / (double) 60) {
                sleep(1 - (timerEnd - timerStart));
                skeletonFrame++;
            }
            else {
                skeletonFrame++;
            }
        }
        /*** Comment this far ***/

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

void drawCube(Shader* shader, std::vector<float> center, float sideLength, std::vector<float> colorRGB) {

    shader->use();

    GLfloat centerX = center[0];
    GLfloat centerY = center[1];
    GLfloat centerZ = center[2];

    GLfloat halfSide = sideLength * (GLfloat)0.5;

    GLfloat colorR = colorRGB[0];
    GLfloat colorG = colorRGB[1];
    GLfloat colorB = colorRGB[2];


    float cubeVertices[] {

            // position                                                    // color
            centerX - halfSide, centerY - halfSide, centerZ + halfSide,    colorR, colorG, colorB,
            centerX + halfSide, centerY - halfSide, centerZ + halfSide,    colorR, colorG, colorB,
            centerX + halfSide, centerY + halfSide, centerZ + halfSide,    colorR, colorG, colorB,
            centerX - halfSide, centerY + halfSide, centerZ + halfSide,    colorR, colorG, colorB,

            centerX - halfSide, centerY - halfSide, centerZ - halfSide,    colorR, colorG, colorB,
            centerX + halfSide, centerY - halfSide, centerZ - halfSide,    colorR, colorG, colorB,
            centerX + halfSide, centerY + halfSide, centerZ - halfSide,    colorR, colorG, colorB,
            centerX - halfSide, centerY + halfSide, centerZ - halfSide,    colorR, colorG, colorB,

    };

    unsigned int cubeIndices[] = {

            0, 1, 2,    // front face
            0, 2, 3,
            4, 5, 6,    // back face
            4, 6, 7,
            2, 7, 3,    // upper face
            2, 6, 7,
            0, 1, 4,    // lower face
            1, 5, 4,
            4, 3, 7,    // left face
            4, 0, 3,
            1, 5, 2,    // right face
            5, 6, 2

    };

    unsigned int cubeVAO, cubeVBO, cubeEBO;

    glGenVertexArrays(1, &cubeVAO);
    glBindVertexArray(cubeVAO);

    glGenBuffers(1, &cubeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &cubeEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_DYNAMIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glLineWidth(4.0f);

    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glDrawElements(GL_TRIANGLES, sizeof(cubeVertices), GL_UNSIGNED_INT, nullptr);

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

    glLineWidth(7.0f);

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

/*Polygon* drawPolygon2D(Shader* shader, float radius, int numSides, bool rotation, std::vector<float> rotationAxis,
        float rotationAngle, std::vector<float> center, std::vector<float> colorRGB) {

    shader->use();

    float vertices[numSides * 6];
    int indices[numSides];
    int anglePart = 0;
    std::vector<std::vector<float>> polygonVertices;

    for(int i = 0; i < numSides * 6; i += 6) {
        vertices[i] = center[0] + (float)(radius * cos(anglePart * 2 * PI / (float)numSides));
        vertices[i + 1] = center[1] + (float)(radius * sin(anglePart * 2 * PI / (float)numSides));
        vertices[i + 2] = center[2];
        vertices[i + 3] = colorRGB[0];
        vertices[i + 4] = colorRGB[1];
        vertices[i + 5] = colorRGB[2];
        if(i % 6 == 0) {
            anglePart++;
        }
        polygonVertices.push_back({vertices[i], vertices[i + 1], vertices[i + 2]});
    }

    for(int i = 0; i < numSides; i++) {
        indices[i] = i;
    }

    unsigned int cubeVAO, cubeVBO, cubeEBO;

    glGenVertexArrays(1, &cubeVAO);
    glBindVertexArray(cubeVAO);

    glGenBuffers(1, &cubeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &cubeEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_DYNAMIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glLineWidth(4.0f);

    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glm::mat4 projection = glm::perspective(glm::radians(fov), (float)WIN_WIDTH / (float)WIN_HEIGHT, 0.1f, 100.0f);
    shader->setMat4("projection", projection);

    glm::mat4 view = camera.GetViewMatrix();
    shader->setMat4("view", view);

    glm::mat4 model = glm::mat4(1.0f);
    if(rotation) {
        // Rotating around the object's axis
        model = glm::translate(model, glm::vec3(-12.25f, 0.0f, -12.25f));
        model = glm::translate(model, glm::vec3(center[0], center[1], center[2]));
        model = glm::rotate(model, glm::radians(rotationAngle), glm::vec3(rotationAxis[0], rotationAxis[1], rotationAxis[2]));
        model = glm::translate(model, glm::vec3(-center[0], -center[1], -center[2]));
        shader->setMat4("modelPolygon", model);
    }
    else {
        model = glm::translate(model, glm::vec3(-12.25f, 0.0f, -12.25f));
        shader->setMat4("modelPolygon", model);
    }

    glDrawElements(GL_LINE_LOOP, sizeof(vertices), GL_UNSIGNED_INT, nullptr);

    return new Polygon(polygonVertices, numSides, {center[0], center[1], center[2]}, {colorRGB[0], colorRGB[1], colorRGB[2]});

}*/

/*void drawSphere(Shader* shader, float radius, std::vector<float> center, std::vector<float> colorRGB) {

    int numRings = 12;
    int numSides = 20;
    float theta = 360 / numRings;
    std::vector<Polygon*> polygons;
    for(int i = 0; i < numRings; i++) {
       polygons.push_back(drawPolygon2D(shader, radius, numSides, true, {1.0f, 0.0f, 0.0f}, i * theta, center, colorRGB));
    }

    //for(int i = 0; i < numSides; i++) {
    //    linkTwoPolygons(shader, polygons[i], polygons[(i + 1) % numSides], numSides);
    //}

}*/

// Polygons MUST have the same number of sides
/*void linkTwoPolygons(Shader* shader, Polygon* p1, Polygon* p2, int numSides) {

    std::vector<std::vector<float>> v1 = p1->getVertices();
    std::vector<std::vector<float>> v2 = p2->getVertices();
    std::vector<float> c1 = p1->getColor();
    std::vector<float> c2 = p2->getColor();
    float verticesPol[numSides * 12];
    int indicesPol[numSides * 2];

    for(int i = 0; i < numSides * 12; i += 12) {
        verticesPol[i] = v1[i][0];
        verticesPol[i + 1] = v1[i][1];
        verticesPol[i + 2] = v1[i][2];
        verticesPol[i + 3] = c1[0];
        verticesPol[i + 4] = c1[1];
        verticesPol[i + 5] = c1[2];
        verticesPol[i + 6] = v2[i][0];
        verticesPol[i + 7] = v2[i][1];
        verticesPol[i + 8] = v2[i][2];
        verticesPol[i + 9] = c2[0];
        verticesPol[i + 10] = c2[1];
        verticesPol[i + 11] = c2[2];
    }

    for(int i = 0; i < numSides * 2; i += 2) {
        if(i == 0) {
            indicesPol[0] = 0;
            indicesPol[1] = 1;
        }
        else {
            indicesPol[i] = indicesPol[i];
            indicesPol[i + 1] = indicesPol[i] + 1;
        }
    }

    unsigned int VAO, VBO, EBO;

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verticesPol), verticesPol, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indicesPol), indicesPol, GL_DYNAMIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glLineWidth(4.0f);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glDrawElements(GL_LINE_LOOP, sizeof(verticesPol), GL_UNSIGNED_INT, nullptr);

}*/

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
                auto* position = new Position();
                for(int j = 0; j < 75; j += 3) {
                    // Doubling to make the skeleton bigger and hence more visible
                    position->add(new Joint(2 * (float)std::stod(row[i + j]),
                                            2 * (float)std::stod(row[i + j + 1]),
                                            2 * (float)std::stod(row[i + j + 2])));
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