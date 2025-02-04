#pragma once
#include <polygon.h>
#include <QImage>
#include <vector>

class Rasterizer {
private:
    //This is the set of Polygons loaded from a JSON scene file
    std::vector<Polygon> m_polygons;
public:
    Rasterizer(const std::vector<Polygon>& polygons);
    QImage renderScene();
    void clearScene();

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
