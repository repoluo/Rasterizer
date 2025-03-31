#pragma once
#include <polygon.h>
#include <QImage>
#include <vector>

const float SCREENSIZE = 512.f;

class Rasterizer {
private:
    //This is the set of Polygons loaded from a JSON scene file
    std::vector<Polygon> m_polygons;
    std::vector<float> z_buffer;
    // std::vector<Vertex> fram_buffer;
public:
    Rasterizer(const std::vector<Polygon>& polygons);
    QImage renderScene();
    void clearScene();

    void renderPolygon(const Polygon&, QImage&);
    void renderTriangular(const std::array<Segment, 3>&, QImage&);

    Vertex interpolate(const glm::vec2& fragPos,
                             const Vertex& v1,
                             const Vertex& v2,
                             const Vertex& v3);

    std::vector<std::array<Segment, 3>> getTriangularSegment(const Polygon&);
    std::vector<std::pair<float, float>> getLeftAndRightIntersectionX(const std::array<Segment, 3>&, const glm::vec4 &);
    glm::vec4 getBoundingBox(const std::array<Segment, 3>&);
    void clip(glm::vec4&, float, float);


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
    template<class T>
    T perspectiveCorrectInterpolateAttrib(const T &v1Attrib,
                                          const T &v2Attrib,
                                          const T &v3Attrib,
                                          float z1, float z2, float z3,
                                          float fragmentZcoord,
                                          const BarycentricWeights &baryWeights) const;
};

