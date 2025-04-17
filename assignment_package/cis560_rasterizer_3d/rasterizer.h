#pragma once
#include <polygon.h>
#include <QImage>
#include <vector>
#include "camera.h"

const float SCREENSIZE = 666.f;

enum class ShadingMode {
    BlinnPhong, // Default, rendering mode
    Wireframes, // Line rendering
};

class Rasterizer {
private:
    //This is the set of Polygons loaded from a JSON scene file
    std::vector<Polygon> m_polygons;
    std::vector<float> z_buffer;
    Camera camera;
    int width = SCREENSIZE;
    int height = SCREENSIZE;
    ShadingMode shadingMode = ShadingMode::BlinnPhong;
    // std::vector<Vertex> fram_buffer;

public:
    Rasterizer(const std::vector<Polygon>& polygons);
    QImage renderScene();
    void clearScene();

    void renderPolygon(const Polygon&, QImage&);
    void renderTriangular(const std::array<Segment, 3>&, QImage&, const QImage* const);
    void setShadingMode(ShadingMode);
    void renderTriangularBlinnPhong(const std::array<Segment, 3>&, QImage&, const QImage* const);
    void renderTriangularWireFrames(const std::array<Segment, 3>&, QImage&, const QImage* const);
    std::vector<std::pair<glm::vec2, glm::vec3>> bresenhamLine(Segment);

    Vertex interpolate(const glm::vec2& fragPos,
                       const Vertex& v1,
                       const Vertex& v2,
                       const Vertex& v3,
                       const QImage* const texture) const;

    std::vector<std::array<Segment, 3>> getTriangularSegment(const Polygon&) const;
    std::vector<std::pair<float, float>> getLeftAndRightIntersectionX(const std::array<Segment, 3>&, const glm::vec4 &) const;
    glm::vec4 getBoundingBox(const std::array<Segment, 3>&) const;
    void clip(glm::vec4&, float, float) const;

    Vertex& worldSpaceToCameraSpace(Vertex&);
    Vertex& cameraSpaceToScreenSpace(Vertex&);
    Vertex& screenSpaceToPixelSpace(Vertex&);
    Vertex pixelSpaceToWorldSpace(Vertex);

    Camera& getCamera();
    int getWidth() const;
    int getHeight() const;

    glm::vec3 reflectionColor(Vertex&, glm::vec3&);

    float computeTriangleArea(const glm::vec2 &v1,
                              const glm::vec2 &v2,
                              const glm::vec2 &v3) const;

    BarycentricWeights computeBarycentricWeights(const glm::vec2 &fragPos,
                                                 const glm::vec2 &v1,
                                                 const glm::vec2 &v2,
                                                 const glm::vec2 &v3) const;

    float perspectiveCorrectInterpolateZ(float z1,
                                         float z2,
                                         float z3,
                                         const BarycentricWeights &baryWeights) const;
    // template<class T>
    // T perspectiveCorrectInterpolateAttrib(const T &v1Attrib,
    //                                       const T &v2Attrib,
    //                                       const T &v3Attrib,
    //                                       float z1, float z2, float z3,
    //                                       float fragmentZcoord,
    //                                       const BarycentricWeights &baryWeights) const;
};


