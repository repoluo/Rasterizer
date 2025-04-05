#include "camera.h"
#include <iostream>


/*
Additionally, your camera should implement the following functions:
此外，您的相机应实现以下功能：

A function which returns a view matrix based on the camera's local axes and position. You may not use GLM's lookAt function for this.
一个返回基于相机本地轴的视图矩阵的函数 和位置。您可以不使用GLM的 lookAt 函数。
A function which returns a perspective projection matrix based on the camera's clipping planes, aspect ratio, and field of view. You may not use GLM's persp function for this. Note that the example renders were made without converting the FOV from degrees to radians; if you do this your renders will still be correct, but they won't match the examples.
返回基于的透视投影矩阵的函数 相机的裁剪平面、长宽比和视野。你可能不会 使用GLM的 persp 函数。请注意，示例渲染已经完成 没有将视场从角度转换为弧度；如果你这样做你的渲染 仍然是正确的，但它们与例子不符。
Three functions that translate the camera along each of its local axes, both forward and backward. The amount of translation should be determined by an input to the function.
三个功能可以使相机沿着每个局部轴移动， 向前和向后。翻译量应由 函数的输入。
Three functions that rotate the camera about each of its local axes. Note that these functions should only alter the orientation of the camera; its position should not change. The amount of rotation should be determined by an input to the function.
三个功能，旋转相机的每一个本地轴。请注意, 这些功能只能改变相机的方向；其位置 不应该改变。旋转的量应该由输入来决定 这个函数。
*/
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

