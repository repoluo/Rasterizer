#ifndef CAMERA_H
#define CAMERA_H
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
    // A vector that represents the camera's "forward" direction, i.e. its Z axis. Default value of <0, 0, -1, 0>.
    glm::vec4 forwardDir = {0, 0, -1, 0};
    // A vector that represents the camera's "right" direction, i.e. its X axis. Default value of <1, 0, 0, 0>.
    glm::vec4 rightDir = {1, 0, 0, 0};
    // A vector that represents the camera's "up" direction, i.e. its Y axis. Default value of <0, 1, 0, 0>.
    glm::vec4 upDir = {0, 1, 0, 0};
    // A value for the camera's vertical field of view. Default value of 45 degrees.
    float verticalDegreeFOV = 45.f;
    // A vector that represents the camera's position in world space. Default value of <0, 0, 10, 1>.
    glm::vec4 cameraPosWS = {0, 0, 13, 1};
    // A floating point number representing the camera's near clip plane. Default value of 0.01.
    float nearClipPlane = 0.01f;
    // A floating point number representing the camera's far clip plane. Default value of 100.0.
    float farClipPlane = 100.f;
    // A floating point number representing the camera's aspect ratio. Default value of 1.0.
    float aspectRation = 1.0;

public:
    Camera() {}
    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;
    void zDirMove(float);
    void xDirMove(float);
    void yDirMove(float);
    void zDirRotationDegree(float);
    void xDirRotationDegree(float);
    void yDirRotationDegree(float);
    glm::vec4 mainCameraPositionWS() const;
    glm::vec4 getForwardDir() const;
};



#endif // CAMERA_H
