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

#include <Kinect.h>

#include "stb_image.h"

#include "Shader.h"
#include "Camera.h"
#include "utils.h"

void framebufferSizeCallback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouseCallback(GLFWwindow* window, double posX, double posY);
void scrollCallback(GLFWwindow* window, double offsetX, double offsetY);

// drawing functions
void drawGrid(Shader* shader, unsigned int gridVAO, unsigned int gridEBO, int numVertices);
void drawCube(Shader* shader, unsigned int cubeVAO, unsigned int cubeEBO, int numVertices);
void drawCoordSystem(Shader* shader, unsigned int coordVAO, unsigned int coordEBO, int numVertices);
void drawSkeleton(Shader* shader, unsigned int skeletonVAO, unsigned int skeletonEBO, int numVertices);
void drawKinectData();

// Kinect functions
bool initKinect();
void getBodyData(IMultiSourceFrame* frame);

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

// Kinect handling
bool tracked;
Joint joints[JointType_Count];
IKinectSensor* kinect;             // Kinect sensor
IMultiSourceFrameReader* reader;   // Kinect data source
ICoordinateMapper* mapper;         // Converts between depth, color, and 3d coordinates

int main() {

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
    float currentFrame;

    initKinect();

    float gridVertices[] = {

            // position             // color
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

    float skeletonVertices[] = {

        1.5f, 0.0f, 1.75f,    1.0f, 0.0f, 1.0f,
        1.5f, 0.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // RIGHT FOOT

        2.5f, 0.0f, 1.75f,    1.0f, 0.0f, 1.0f,
        2.5f, 0.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // LEFT FOOT

        1.5f, 2.0f, 1.2f,    1.0f, 0.0f, 1.0f,    // RIGHT KNEE
        2.5f, 2.0f, 1.2f,    1.0f, 0.0f, 1.0f,    // LEFT KNEE

        1.5f, 4.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // RIGHT HIP
        2.5f, 4.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // LEFT HIP

        2.0f, 5.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // PELVIS

        2.0f, 7.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // NECK

        2.0f, 8.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // TOP HEAD

        1.0f, 7.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // RIGHT SHOULDER
        3.0f, 7.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // LEFT SHOULDER

        0.5f, 5.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // RIGHT ELBOW
        3.5f, 5.0f, 1.0f,    1.0f, 0.0f, 1.0f,    // LEFT ELBOW

        0.5f, 3.0f, 1.2f,    1.0f, 0.0f, 1.0f,    // RIGHT WRIST
        3.5f, 3.0f, 1.2f,    1.0f, 0.0f, 1.0f,    // LEFT WRIST

        0.6f, 2.75f, 1.27f,    1.0f, 0.0f, 1.0f,    // RIGHT THUMB
        3.4f, 2.75f, 1.27f,    1.0f, 0.0f, 1.0f,    // LEFT THUMB

        0.375f, 2.5f, 1.27f,    1.0f, 0.0f, 1.0f,    // RIGHT THUMB
        3.625f, 2.5f, 1.27f,    1.0f, 0.0f, 1.0f,    // LEFT THUMB

    };

    unsigned int skeletonIndices[] = {

            0, 1,
            2, 3,
            1, 4,
            3, 5,
            4, 6,
            5, 7,
            6, 8,
            7, 8,
            8, 9,
            9, 10,
            9, 11,
            9, 12,
            11, 13,
            13, 15,
            15, 17,
            15, 19,
            12, 14,
            14, 16,
            16, 18,
            16, 20,
            17, 19,
            18, 20

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

    while(!glfwWindowShouldClose(window)) {

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

    glDrawElements(GL_LINES, numVertices, GL_UNSIGNED_INT, nullptr);
    
}

void drawCoordSystem(Shader* shader, unsigned int coordVAO, unsigned int coordEBO, int numVertices) {

    glLineWidth(5.0f);

    glBindVertexArray(coordVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, coordEBO);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glDrawElements(GL_LINES, numVertices, GL_UNSIGNED_INT, nullptr);

}

void drawKinectData() {
    // ...
    if (tracked) {
        // Draw some arms
        const CameraSpacePoint& lh = joints[JointType_WristLeft].Position;
        const CameraSpacePoint& le = joints[JointType_ElbowLeft].Position;;
        const CameraSpacePoint& ls = joints[JointType_ShoulderLeft].Position;;
        const CameraSpacePoint& rh = joints[JointType_WristRight].Position;;
        const CameraSpacePoint& re = joints[JointType_ElbowRight].Position;;
        const CameraSpacePoint& rs = joints[JointType_ShoulderRight].Position;;
        glBegin(GL_LINES);
        glColor3f(1.f, 0.f, 0.f);
        // lower left arm
        glVertex3f(lh.X, lh.Y, lh.Z);
        glVertex3f(le.X, le.Y, le.Z);
        // upper left arm
        glVertex3f(le.X, le.Y, le.Z);
        glVertex3f(ls.X, ls.Y, ls.Z);
        // lower right arm
        glVertex3f(rh.X, rh.Y, rh.Z);
        glVertex3f(re.X, re.Y, re.Z);
        // upper right arm
        glVertex3f(re.X, re.Y, re.Z);
        glVertex3f(rs.X, rs.Y, rs.Z);
        glEnd();
    }
}

bool initKinect() {

    if(!GetDefaultKinectSensor(&kinect)) {
        std::cout << "Failed to get Kinect sensor." << std::endl;
        return false;
    }
    if(kinect) {
        kinect->get_CoordinateMapper(&mapper);
        kinect->Open();
        kinect->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color |
                                           FrameSourceTypes::FrameSourceTypes_Depth |
                                           FrameSourceTypes::FrameSourceTypes_Body, &reader);
        std::cout << "Kinect sensor initialized." << std::endl;
        return reader;
    }
    else {
        std::cout << "Couldn't open Kinect sensor." << std::endl;
        return false;
    }

}

void getBodyData(IMultiSourceFrame* frame) {

    IBodyFrame* bodyframe;
    IBodyFrameReference* frameref = nullptr;
    frame->get_BodyFrameReference(&frameref);
    frameref->AcquireFrame(&bodyframe);
    if (frameref) frameref->Release();

    if (!bodyframe) return;

    IBody* body[BODY_COUNT];
    bodyframe->GetAndRefreshBodyData(BODY_COUNT, body);
    for (int i = 0; i < BODY_COUNT; i++) {
        body[i]->get_IsTracked(&tracked);
        if (tracked) {
            body[i]->GetJoints(JointType_Count, joints);
            break;
        }
    }

    if (bodyframe) bodyframe->Release();

}
