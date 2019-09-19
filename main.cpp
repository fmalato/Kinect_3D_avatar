#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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

#include "stb_image.h"

#include "Shader.h"
#include "Camera.h"
#include "Position.h"

void framebufferSizeCallback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouseCallback(GLFWwindow* window, double posX, double posY);
void scrollCallback(GLFWwindow* window, double offsetX, double offsetY);

// drawing functions
void drawGrid(Shader* shader, unsigned int gridVAO, unsigned int gridEBO, int numVertices);
void drawCube(Shader* shader, unsigned int cubeVAO, unsigned int cubeEBO, int numVertices);
void drawCoordSystem(Shader* shader, unsigned int coordVAO, unsigned int coordEBO, int numVertices);
void drawSkeleton(Shader* shader, unsigned int skeletonVAO, unsigned int skeletonEBO, int numVertices);

// data management functions
std::vector<Position*> getJointPositions(std::string fileName);
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

int main() {

    std::vector<Position*> positions = getJointPositions("../KinectJoints.csv");

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

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
    Shader skeletonShader("../Shaders/skeletonVertShader.vert", "../Shaders/skeletonFragShader.frag");
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

    float cubeVertices[] {

        // position          // color
        6.0f, 0.0f, 6.0f,    0.0f, 0.5f, 0.0f,
        5.0f, 0.0f, 6.0f,    0.0f, 0.5f, 0.0f,
        5.0f, 1.0f, 6.0f,    0.0f, 0.5f, 0.0f,
        6.0f, 1.0f, 6.0f,    0.0f, 0.5f, 0.0f,

        6.0f, 0.0f, 5.0f,    0.0f, 0.5f, 0.0f,
        5.0f, 0.0f, 5.0f,    0.0f, 0.5f, 0.0f,
        5.0f, 1.0f, 5.0f,    0.0f, 0.5f, 0.0f,
        6.0f, 1.0f, 5.0f,    0.0f, 0.5f, 0.0f,

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

    // Smoothing the animation
    int times = 2;
    int currentPos = 0;
    for(auto posItr = positions.begin(); posItr < positions.end() - 1; posItr++) {
        std::cout << "interpolating between positions " << currentPos << " and " << currentPos + 1 << std::endl;
        std::vector<Position*> interPositions = interpolate(*posItr, *(posItr++), times);
        for(auto itr2 = interPositions.begin(); itr2 != interPositions.end(); itr2++) {
            posItr = positions.insert(posItr, *itr2);
        }
        currentPos++;
    }

    int skeletonFrame = 0;
    std::vector<Joint*> joints = positions[skeletonFrame]->getJoints();

    // This kind of offsets were the fastest way to get a constant translation in the middle of the grid
    float skeletonVertices[] = {

            joints[0]->getX() + 6, joints[0]->getY() + (float)2.5, joints[0]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[1]->getX() + 6, joints[1]->getY() + (float)2.5, joints[1]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[2]->getX() + 6, joints[2]->getY() + (float)2.5, joints[2]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[3]->getX() + 6, joints[3]->getY() + (float)2.5, joints[3]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[4]->getX() + 6, joints[4]->getY() + (float)2.5, joints[4]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[5]->getX() + 6, joints[5]->getY() + (float)2.5, joints[5]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[6]->getX() + 6, joints[6]->getY() + (float)2.5, joints[6]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[7]->getX() + 6, joints[7]->getY() + (float)2.5, joints[7]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[8]->getX() + 6, joints[8]->getY() + (float)2.5, joints[8]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[9]->getX() + 6, joints[9]->getY() + (float)2.5, joints[9]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[10]->getX() + 6, joints[10]->getY() + (float)2.5, joints[10]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[11]->getX() + 6, joints[11]->getY() + (float)2.5, joints[11]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[12]->getX() + 6, joints[12]->getY() + (float)2.5, joints[12]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[13]->getX() + 6, joints[13]->getY() + (float)2.5, joints[13]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[14]->getX() + 6, joints[14]->getY() + (float)2.5, joints[14]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[15]->getX() + 6, joints[15]->getY() + (float)2.5, joints[15]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[16]->getX() + 6, joints[16]->getY() + (float)2.5, joints[16]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[17]->getX() + 6, joints[17]->getY() + (float)2.5, joints[17]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[18]->getX() + 6, joints[18]->getY() + (float)2.5, joints[18]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[19]->getX() + 6, joints[19]->getY() + (float)2.5, joints[19]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[20]->getX() + 6, joints[20]->getY() + (float)2.5, joints[20]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[21]->getX() + 6, joints[21]->getY() + (float)2.5, joints[21]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[22]->getX() + 6, joints[22]->getY() + (float)2.5, joints[22]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[23]->getX() + 6, joints[23]->getY() + (float)2.5, joints[23]->getZ() + 2,    1.0f, 0.0f, 1.0f,
            joints[24]->getX() + 6, joints[24]->getY() + (float)2.5, joints[24]->getZ() + 2,    1.0f, 0.0f, 1.0f,

    };

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

    // --------------------- CUBE ------------------------------

    unsigned int cubeVAO, cubeVBO, cubeEBO;

    glGenVertexArrays(1, &cubeVAO);
    glBindVertexArray(cubeVAO);

    glGenBuffers(1, &cubeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    glGenBuffers(1, &cubeEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_STATIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

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

    // ---------------------- SKELETON -----------------------------------

    unsigned int skeletonVAO, skeletonVBO, skeletonEBO;
    glGenVertexArrays(1, &skeletonVAO);
    glBindVertexArray(skeletonVAO);

    glGenBuffers(1, &skeletonVBO);
    glBindBuffer(GL_ARRAY_BUFFER, skeletonVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(skeletonVertices), skeletonVertices, GL_STATIC_DRAW);

    glGenBuffers(1, &skeletonEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, skeletonEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(skeletonIndices), skeletonIndices, GL_STATIC_DRAW);

    // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_TRUE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // color
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    shader.use();

    float duration = 0;

    while(!glfwWindowShouldClose(window)) {

        // Animation...?
        joints = positions[skeletonFrame % positions.size()]->getJoints();
        float skeletonVertices2[] = {

                joints[0]->getX() + 6, joints[0]->getY() + (float)2.5, joints[0]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[1]->getX() + 6, joints[1]->getY() + (float)2.5, joints[1]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[2]->getX() + 6, joints[2]->getY() + (float)2.5, joints[2]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[3]->getX() + 6, joints[3]->getY() + (float)2.5, joints[3]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[4]->getX() + 6, joints[4]->getY() + (float)2.5, joints[4]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[5]->getX() + 6, joints[5]->getY() + (float)2.5, joints[5]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[6]->getX() + 6, joints[6]->getY() + (float)2.5, joints[6]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[7]->getX() + 6, joints[7]->getY() + (float)2.5, joints[7]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[8]->getX() + 6, joints[8]->getY() + (float)2.5, joints[8]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[9]->getX() + 6, joints[9]->getY() + (float)2.5, joints[9]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[10]->getX() + 6, joints[10]->getY() + (float)2.5, joints[10]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[11]->getX() + 6, joints[11]->getY() + (float)2.5, joints[11]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[12]->getX() + 6, joints[12]->getY() + (float)2.5, joints[12]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[13]->getX() + 6, joints[13]->getY() + (float)2.5, joints[13]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[14]->getX() + 6, joints[14]->getY() + (float)2.5, joints[14]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[15]->getX() + 6, joints[15]->getY() + (float)2.5, joints[15]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[16]->getX() + 6, joints[16]->getY() + (float)2.5, joints[16]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[17]->getX() + 6, joints[17]->getY() + (float)2.5, joints[17]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[18]->getX() + 6, joints[18]->getY() + (float)2.5, joints[18]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[19]->getX() + 6, joints[19]->getY() + (float)2.5, joints[19]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[20]->getX() + 6, joints[20]->getY() + (float)2.5, joints[20]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[21]->getX() + 6, joints[21]->getY() + (float)2.5, joints[21]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[22]->getX() + 6, joints[22]->getY() + (float)2.5, joints[22]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[23]->getX() + 6, joints[23]->getY() + (float)2.5, joints[23]->getZ() + 2,    1.0f, 0.0f, 1.0f,
                joints[24]->getX() + 6, joints[24]->getY() + (float)2.5, joints[24]->getZ() + 2,    1.0f, 0.0f, 1.0f,

        };
        glBufferData(GL_ARRAY_BUFFER, sizeof(skeletonVertices2), skeletonVertices2, GL_STATIC_DRAW);

        currentFrame = (float)glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        drawGrid(&shader, gridVAO, gridEBO, sizeof(gridIndices));
        // drawCube(&shader, cubeVAO, cubeEBO, sizeof(cubeIndices));
        drawCoordSystem(&shader, coordVAO, coordEBO, sizeof(coordIndices));
        drawSkeleton(&shader, skeletonVAO, skeletonEBO, sizeof(skeletonIndices));

        glfwSwapBuffers(window);
        glfwPollEvents();

        duration += 1 / (float)8;
        if(duration > 1) {
            duration = 0;
            skeletonFrame++;
        }
    }

    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteBuffers(1, &gridEBO);
    glDeleteVertexArrays(1, &cubeVAO);
    glDeleteBuffers(1, &cubeVBO);
    glDeleteBuffers(1, &cubeEBO);
    glDeleteVertexArrays(1, &coordVAO);
    glDeleteBuffers(1, &coordVBO);
    glDeleteBuffers(1, &coordEBO);
    glDeleteVertexArrays(1, &skeletonVAO);
    glDeleteBuffers(1, &skeletonVBO);
    glDeleteBuffers(1, &skeletonEBO);

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

    camera.ProcessMouseScroll(offsetY);

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

void drawCube(Shader* shader, unsigned int cubeVAO, unsigned int cubeEBO, int numVertices) {

    glLineWidth(4.0f);

    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glDrawElements(GL_TRIANGLES, numVertices, GL_UNSIGNED_INT, nullptr);

}

void drawSkeleton(Shader* shader, unsigned int skeletonVAO, unsigned int skeletonEBO, int numVertices) {

    glLineWidth(7.0f);

    glBindVertexArray(skeletonVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, skeletonEBO);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    /*glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(-0.25 * i, 0.0f, -0.25f * j));
    shader->setMat4("modelSkeleton", model);*/

    glDrawElements(GL_LINES, numVertices, GL_UNSIGNED_INT, nullptr);
    
}

void drawCoordSystem(Shader* shader, unsigned int coordVAO, unsigned int coordEBO, int numVertices) {

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

std::vector<Position*> interpolate(Position* start, Position* end, int times) {

    std::vector<Joint*> interPos;
    std::vector<Position*> result;
    std::vector<Joint*> startJoints = start->getJoints();
    std::vector<Joint*> endJoints = end->getJoints();

    // loop on the joints
    for(int i = 0; i < 25; i++) {
        // loop on the number of interpolations
        for (int j = 1; j < times; j++) {
            interPos.push_back(new Joint((endJoints[i]->getX() + startJoints[i]->getX()) * j / times,
                                       (endJoints[i]->getY() + startJoints[i]->getY()) * j / times,
                                       (endJoints[i]->getZ() + startJoints[i]->getZ()) * j / times));
        }
    }
    std::vector<Joint*> currentPosJoints;
    for(int k = 0; k < times - 1; k++) {
        for (auto itr = interPos.begin() + k; itr != interPos.end(); itr++) {
            for(int l = 0; l < times - 2; l++) {
                itr++;
            }
            currentPosJoints.push_back(*itr);
        }
        result.push_back(new Position(currentPosJoints));
    }

    return result;
}