#include "camera.h"
#include <iostream>



/**
 * @brief Camera::getViewMatrix
 *        A function which returns a view matrix based on the camera's local axes and position.
 *        You may not use GLM's lookAt function for this.
 * @return
 */
glm::mat4 Camera::getViewMatrix() const {

    // Translation matrix
    glm::mat4 viewMatrix_T(1, 0, 0, -cameraPosWS.x,
                           0, 1, 0, -cameraPosWS.y,
                           0, 0, 1, -cameraPosWS.z,
                           0, 0, 1, 0);
    // col major, we need transpose it into the form we used to.
    viewMatrix_T = glm::transpose(viewMatrix_T);

    // Orientation matrix, rotation
    glm::mat4 viewMatrix_R(rightDir,
                           upDir,
                           forwardDir,
                           glm::vec4(0, 0, 0, 1));
    viewMatrix_R = glm::transpose(viewMatrix_R);

    return viewMatrix_R * viewMatrix_T;
}

glm::mat4 Camera::getProjectionMatrix() const {
    float verticalRadiansFOV = glm::radians(verticalDegreeFOV);
    float scaleY = 1 / glm::tan(verticalRadiansFOV / 2);
    float scaleX = scaleY * aspectRation;
    float sceneDepthZ = farClipPlane - nearClipPlane;
    float p = farClipPlane / sceneDepthZ;
    float q = -farClipPlane * nearClipPlane / sceneDepthZ;
    return glm::transpose(glm::mat4(scaleX, 0, 0, 0,
                                    0, scaleY, 0, 0,
                                    0, 0, p, q,
                                    0, 0, 1, 0));
}

// Camera move a stride in z(forward) direction
void Camera::zDirMove(float stride) {
    cameraPosWS += stride * forwardDir;
}

// Camera move a stride in x(right) direction
void Camera::xDirMove(float stride) {
    cameraPosWS += stride * rightDir;
}

// Camera move a stride in y(up) direction
void Camera::yDirMove(float stride) {
    cameraPosWS += stride * upDir;
}

void Camera::zDirRotationDegree(float degree) {
    float radians = glm::radians(degree);
    auto rotateMatrix = glm::rotate(glm::mat4(), radians, glm::vec3(this->forwardDir));
    // Rotate the right and up directions around the forward axis
    rightDir = rotateMatrix * rightDir;
    upDir = rotateMatrix * upDir;
}

void Camera::xDirRotationDegree(float degree) {
    float radians = glm::radians(degree);
    auto rotateMatrix = glm::rotate(glm::mat4(), radians, glm::vec3(this->rightDir));
    // Rotate the forward and up directions around the right axis
    forwardDir = rotateMatrix * forwardDir;
    upDir = rotateMatrix * upDir;
}

void Camera::yDirRotationDegree(float degree) {
    float radians = glm::radians(degree);
    auto rotateMatrix = glm::rotate(glm::mat4(), radians, glm::vec3(this->upDir));
    // Rotate the forward and right directions around the up axis
    forwardDir = rotateMatrix * forwardDir;
    rightDir = rotateMatrix * rightDir;
}

glm::vec4 Camera::mainCameraPositionWS() const {
    return cameraPosWS;
}

glm::vec4 Camera::getForwardDir() const {
    return forwardDir;
}

