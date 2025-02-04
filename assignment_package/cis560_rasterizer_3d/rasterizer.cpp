#include "rasterizer.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>

Rasterizer::Rasterizer(const std::vector<Polygon>& polygons)
    : m_polygons(polygons)
{}

QImage Rasterizer::renderScene() {
    QImage result(512, 512, QImage::Format_RGB32);
    // Fill the image with black pixels.
    // Note that qRgb creates a QColor,
    // and takes in values [0, 255] rather than [0, 1].
    result.fill(qRgb(0.f, 0.f, 0.f));
    // TODO: Complete the various components of code that make up this function.
    // It should return the rasterized image of the current scene.

    // General outline of this function:
    // A) Iterate through each Polygon and then each Triangle within that Polygon.
    //    Make sure you use const references when iterating
    // A) (3D milestone only)

    // Make liberal use of helper functions; writing your rasterizer as one
    // long RenderScene function will make it (a) hard to debug and
    // (b) hard to write without copy-pasting. Also, Adam will be sad when
    // he reads your code.

    // Also! As per the style requirements for this assignment, make sure you
    // use std::arrays to store things like your line segments, Triangles, and
    // vertex coordinates. This lets you easily use loops to perform operations
    // on your scene components, rather than copy-pasting operations three times
    // each!
    return result;
}

float Rasterizer::computeTriangleArea(const glm::vec2 &v1,
                                      const glm::vec2 &v2,
                                      const glm::vec2 &v3) const {
    // TODO: Calculate the 2D area of the triangle
    //       composed of the three input vertex positions
    return 0;
}

BarycentricWeights Rasterizer::computeBarycentricWeights(const glm::vec2 &fragPos,
                                                         const glm::vec2 &v1,
                                                         const glm::vec2 &v2,
                                                         const glm::vec2 &v3) const {
    // TODO: Compute the barycentric interpolation weights
    //       for input fragment position fragPos relative to the three
    //       input triangle vertex positions v1, v2, and v3.
    //       The weights should sum to 1.0.
    return BarycentricWeights(0,0,0);
}

float Rasterizer::perspectiveCorrectInterpolateZ(float z1, float z2, float z3,
                                                 const BarycentricWeights &baryWeights) const {
    // TODO: Use the perspective-correct Z interpolation formula
    //       to calculate the camera-space Z coordinate of the fragment
    //       whose barycentric weights have been passed in.
    return 0.f;
}

template<class T>
T Rasterizer::perspectiveCorrectInterpolateAttrib(const T &v1Attrib,
                                                  const T &v2Attrib,
                                                  const T &v3Attrib,
                                                  float z1, float z2, float z3,
                                                  float fragmentZcoord,
                                                  const BarycentricWeights &baryWeights) const {
    // TODO: Use the perspective-correct vertex attribute interpolation
    //       formula to interpolate the three input vertex attributes based
    //       on the input barycentric weights and input vertex and fragment Z coordinates.
    // You will use this function to interpolate UV coordinates and vertex normals,
    // at the very least.

    // Example invocation of this function:
    // glm::vec2 uv = perspectiveCorrectInterpolateAttrib<glm::vec2>(uv1, uv2, uv3,
    //                                                               z1, z2, z3,
    //                                                               fragZ,
    //                                                               baryWeights);
}

void Rasterizer::clearScene() {
    m_polygons.clear();
}
