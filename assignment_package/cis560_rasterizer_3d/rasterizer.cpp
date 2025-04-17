#include "rasterizer.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <iostream>
#include <algorithm>

const QRgb BACKGROUND_COLOR = qRgb(127.f, 127.f, 127.f);

using std::min;

Rasterizer::Rasterizer(const std::vector<Polygon>& polygons)
    : m_polygons(polygons)
{}

/**
 * @brief Rasterizer::renderScene
 *        Traverse the polygons to render the scene.
 * @return A QImage represent a scene.
 */
QImage Rasterizer::renderScene() {
    float W =  SCREENSIZE, H = SCREENSIZE;
    QImage result(W, H, QImage::Format_RGB32);
    z_buffer.resize(W * H);
    std::fill(z_buffer.begin(), z_buffer.end(), std::numeric_limits<float>::infinity());

    // Fill the image with black pixels.
    // Note that qRgb creates a QColor,
    // and takes in values [0, 255] rather than [0, 1].
    result.fill(BACKGROUND_COLOR);


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
    for (Polygon polygon : m_polygons) {
        for (Vertex& v : polygon.m_verts) {
            worldSpaceToCameraSpace(v);
            cameraSpaceToScreenSpace(v);
            screenSpaceToPixelSpace(v);
        }
        renderPolygon(polygon, result);
    }

    return result;
}

void Rasterizer::clearScene() {
    m_polygons.clear();
}

/**
 * @brief Rasterizer::renderPolygon
 *        Assemble triangular with segment, and the render through each triangular
 * @param polygon
 * @param result : QImage
 */
void Rasterizer::renderPolygon(const Polygon &polygon, QImage& result) {
    //assemble triangular with segment
    auto triWithSegment = getTriangularSegment(polygon);
    for (auto& tri : triWithSegment) {
        renderTriangular(tri, result, polygon.mp_texture);
    }
}

/**
 * @brief Rasterizer::interpolate
 * @param fragPos
 * @param v1
 * @param v2
 * @param v3
 * @return an frag that interpolate by fragPos, v1, v2, v3
 */
Vertex Rasterizer::interpolate(const glm::vec2 &fragPos,
                               const Vertex &v1,
                               const Vertex &v2,
                               const Vertex &v3,
                               const QImage* const texture = nullptr) const{
    BarycentricWeights w = computeBarycentricWeights(fragPos, glm::vec2(v1.m_pos), glm::vec2(v2.m_pos), glm::vec2(v3.m_pos));
    glm::vec4 pos = w.s1 * v1.m_pos + w.s2 * v2.m_pos + w.s3 * v3.m_pos;
    glm::vec4 normal = w.s1 * v1.m_normal + w.s2 * v2.m_normal + w.s3 * v3.m_normal;
    glm::vec2 uv = w.s1 * v1.m_uv + w.s2 * v2.m_uv + w.s3 * v3.m_uv;
    glm::vec3 color;
    if (texture) {
        color = GetImageColor(uv, texture);
    } else {
        color = glm::vec3(255, 0, 255);
    }
    color.r = std::min(color.r, 255.f);
    color.g = std::min(color.g, 255.f);
    color.b = std::min(color.b, 255.f);
    Vertex frag(pos, color, normal, uv);
    return frag;
}

/**
 * @brief Rasterizer::renderTriangular
 *        Rendering a triangular, first get bounding box, the clip bounding box,
 *        and then find the left and right bound of intersection, finally,
 *        use barycentric weight interpolate the set color for each fragment.
 * @param tri
 * @param result : QImage
 * @param texture : QImage
 */
void Rasterizer::renderTriangular(const std::array<Segment, 3> &tri, QImage &result, const QImage* const texture) {
    switch (shadingMode) {
    case ShadingMode::BlinnPhong:
        renderTriangularBlinnPhong(tri, result, texture);
        break;
    case ShadingMode::Wireframes:
        renderTriangularWireFrames(tri, result, texture);
        break;
    }
}

/**
 * @brief Rasterizer::setShadingMode
 *        Set shading mode.
 * @param shadingMode_
 */
void Rasterizer::setShadingMode(ShadingMode shadingMode_) {
    shadingMode = shadingMode_;
}


/**
 * @brief Rasterizer::renderTriangularBlinnPhong
 *        Rendering a triangular, first get bounding box, the clip bounding box,
 *        and then find the left and right bound of intersection, finally,
 *        use barycentric weight interpolate the set color for each fragment.
 * @param tri
 * @param result
 * @param texture
 */
void Rasterizer::renderTriangularBlinnPhong(const std::array<Segment, 3> &tri, QImage &result, const QImage * const texture){
    const Vertex& v1 = tri[0].getP1();
    const Vertex& v2 = tri[0].getP2();
    const Vertex& v3 = tri[1].getP2();
    glm::vec2 v1_screen(v1.m_pos.x, v1.m_pos.y);
    glm::vec2 v2_screen(v2.m_pos.x, v2.m_pos.y);
    glm::vec2 v3_screen(v3.m_pos.x, v3.m_pos.y);
    float cross = (v2_screen.x - v1_screen.x) * (v3_screen.y - v1_screen.y)
                  - (v2_screen.y - v1_screen.y) * (v3_screen.x - v1_screen.x);
    if (cross >= 0.0f) {
        return;
    }
    float W = (float)result.width();
    float H = (float)result.height();
    glm::vec4 boundingBox = getBoundingBox(tri);
    clip(boundingBox, W, H);
    float yLower = boundingBox[2];
    auto intervals = getLeftAndRightIntersectionX(tri, boundingBox);
    for (unsigned int i = 0; i < intervals.size(); i++) {
        float y = std::ceil(i + yLower);
        for (unsigned int x = std::ceil(intervals[i].first); x < std::ceil(intervals[i].second); x++) {
            glm::vec2 fragPos(x, y);
            Vertex frag = interpolate(fragPos, v1, v2, v3, texture);
            if (frag.m_pos.z < z_buffer[x + W * y] && frag.m_pos.z > 0 && frag.m_pos.z < 100) {
                frag.m_color = reflectionColor(frag, frag.m_color);
                QColor color(frag.m_color.r, frag.m_color.g, frag.m_color.b);
                // std::cout << frag.m_color.r << frag.m_color.g << frag.m_color.b << '\n';
                result.setPixelColor(x, y, color);
                z_buffer[x + W * y] = frag.m_pos.z;
            }
        }
    }
}

/**
 * @brief Rasterizer::renderTriangularWireFrames
 *        Rendering a triangular, in form of line rendering. Use Bresenham algorithm to draw a segment.
 * @param tri
 * @param result
 * @param texture
 */
void Rasterizer::renderTriangularWireFrames(const std::array<Segment, 3> &tri, QImage &result, const QImage * const texture) {
    const Vertex& v1 = tri[0].getP1();
    const Vertex& v2 = tri[0].getP2();
    const Vertex& v3 = tri[1].getP2();
    glm::vec2 v1_screen(v1.m_pos.x, v1.m_pos.y);
    glm::vec2 v2_screen(v2.m_pos.x, v2.m_pos.y);
    glm::vec2 v3_screen(v3.m_pos.x, v3.m_pos.y);

    Segment seg1 = tri[0];
    Segment seg2 = tri[1];
    Segment seg3 = tri[2];
    std::vector<std::pair<glm::vec2, glm::vec3>> line1 = bresenhamLine(seg1);
    std::vector<std::pair<glm::vec2, glm::vec3>> line2 = bresenhamLine(seg2);
    std::vector<std::pair<glm::vec2, glm::vec3>> line3 = bresenhamLine(seg3);
    for (auto& [point, color] : line1) {
        result.setPixelColor(point.x, point.y, QColor(color.r, color.g, color.b));
    }
    for (auto& [point, color] : line2) {
        result.setPixelColor(point.x, point.y, QColor(color.r, color.g, color.b));
    }
    for (auto& [point, color] : line3) {
        result.setPixelColor(point.x, point.y, QColor(color.r, color.g, color.b));
    }
}

/**
 * @brief Rasterizer::bresenhamLine
 *        Implementation of Bresenham algorithm.
 * @param seg : input segment contain two point, a gradient and some function.
 * @return a vector of a series of points need to be drawn in screen.
 */
std::vector<std::pair<glm::vec2, glm::vec3>> Rasterizer::bresenhamLine(Segment seg) {
    int x1 = seg.getP1().m_pos.x;
    int y1 = seg.getP1().m_pos.y;
    int x2 = seg.getP2().m_pos.x;
    int y2 = seg.getP2().m_pos.y;
    bool isSteep = std::abs(seg.getGradient()) > 1; // whether segment is steep, (gradient > 1)
    // if is steep, swap the role of x and y
    if (isSteep) {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    // draw from left to right
    if (x1 > x2) {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    std::vector<std::pair<glm::vec2, glm::vec3>> segConstructedWithAllPoints;
    glm::vec3 color(255, 255, 255);

    int dx = x2 - x1;
    int dy = std::abs(y2 - y1);
    int error = dx / 2;
    int yStep = y2 > y1 ? 1 : -1;
    int y = y1;

    for (int x = x1; x < x2; x++) {
        if (isSteep) {
            if (x > 0 && x < height && y > 0 && y < width) {
                segConstructedWithAllPoints.emplace_back(std::make_pair(glm::vec2(y, x), color));
            }
        } else {
            if (x > 0 && x < width && y > 0 && y < height) {
                segConstructedWithAllPoints.emplace_back(std::make_pair(glm::vec2(x, y), color));
            }
        }
        error -= 1 * dy;
        if (error < 0) {
            y += yStep;
            error += 1 * dx;
        }
    }

    return segConstructedWithAllPoints;
}

/**
 * @brief Rasterizer::getTriangularSegment
 * @param polygon
 * @return The triangular of a polygon, which is constructed with three segment
 */
std::vector<std::array<Segment, 3>> Rasterizer::getTriangularSegment(const Polygon& polygon) const {
    std::vector<std::array<Segment, 3>> triWithSegment;
    for (const Triangle& tri : polygon.m_tris) {
        // find the vertex index of triangular in polygon
        auto p1 = tri.m_indices[0];
        auto p2 = tri.m_indices[1];
        auto p3 = tri.m_indices[2];
        // construct segments for each triangular, s1(p1, p2), s2(p1, p3), s3(p2, p3)
        Segment s1(polygon.VertAt(p1), polygon.VertAt(p2));
        Segment s2(polygon.VertAt(p1), polygon.VertAt((p3)));
        Segment s3(polygon.VertAt(p2), polygon.VertAt(p3));
        triWithSegment.push_back({s1, s2, s3});
    }
    return triWithSegment;
}

/**
 * @brief Rasterizer::getLeftAndRightIntersectionX
 *        Traverse y in bounding box, find the left and right bound of intersection.
 * @param segments : represent a triangular
 * @param boundingBox
 * @return a lists of interval where to be add color
 */
std::vector<std::pair<float, float>> Rasterizer::getLeftAndRightIntersectionX(const std::array<Segment, 3> &segments,
                                                                              const glm::vec4 &boundingBox) const {
    std::vector<std::pair<float, float>> intersectionBounds;
    float yLower = boundingBox[2];
    float yUpper = boundingBox[3];
    for (int y = std::ceil(yLower); y < std::ceil(yUpper); y++) {
        float xLeft = SCREENSIZE;
        float xRight = 0.f;
        for (const Segment& segment : segments) {
            float temp = -1.f;
            bool intersection = segment.getIntersection(y, &temp);
            if (intersection) {
                xLeft = std::min(xLeft, temp);;
                xRight = std::max(xRight, temp);
            }
        }
        xLeft = std::max(xLeft, 0.f);
        xRight = std::min(xRight, SCREENSIZE);
        // std::cout << xLeft << ' ' << xRight << '\n'; // for debug
        intersectionBounds.push_back({xLeft, xRight});
    }
    return intersectionBounds;
}

/**
 * @brief Rasterizer::getBoundingBox
 * @param triWithSegment
 * @return (x1, x2, y1, y2)
 */
glm::vec4 Rasterizer::getBoundingBox(const std::array<Segment, 3> &triWithSegment) const {
    glm::vec2 p1 = triWithSegment[0].getP1Coordinate();
    glm::vec2 p2 = triWithSegment[0].getP2Coordinate();
    glm::vec2 p3 = triWithSegment[1].getP2Coordinate();
    float xLeft = std::min({p1[0], p2[0], p3[0]});
    float xRight = std::max({p1[0], p2[0], p3[0]});
    float yLower = std::min({p1[1], p2[1], p3[1]});
    float yUpper = std::max({p1[1], p2[1], p3[1]});
    return glm::vec4(xLeft, xRight, yLower, yUpper);
}

/**
 * @brief Rasterizer::clip
 *        Clip the part that can not been seen in boundingbox.
 * @param boundingBox
 * @param W
 * @param H
 */
void Rasterizer::clip(glm::vec4 &boundingBox, float W, float H) const {
    boundingBox[0] = std::max(boundingBox[0], 0.f);
    boundingBox[1] = std::min(boundingBox[1], W);
    boundingBox[2] = std::max(boundingBox[2], 0.f);
    boundingBox[3] = std::min(boundingBox[3], H);
}

Vertex &Rasterizer::worldSpaceToCameraSpace(Vertex &v) {
    v.m_pos = camera.getViewMatrix() * v.m_pos;
    return v;
}

Vertex &Rasterizer::cameraSpaceToScreenSpace(Vertex &v) {
    v.m_pos = camera.getProjectionMatrix() * v.m_pos;
    v.m_pos /= v.m_pos.w == 0 ? 1 : v.m_pos.w; // Transform it into homogenious coordinate
    return v;
}

Vertex &Rasterizer::screenSpaceToPixelSpace(Vertex &v) {
    float W = getWidth();
    float H = getHeight();
    v.m_pos.x = (v.m_pos.x + 1) * W / 2;
    v.m_pos.y = (1 - v.m_pos.y) * H / 2;
    return v;
}

Vertex Rasterizer::pixelSpaceToWorldSpace(Vertex v) {
    float W = getWidth();
    float H = getHeight();
    // Pixel to screen
    v.m_pos.x = v.m_pos.x * 2 / W - 1;
    v.m_pos.y = 1 - v.m_pos.y * 2 / H;
    // Unhomogenious
    v.m_pos *= v.m_pos.w == 0 ? 1 : v.m_pos.w;
    // Inverse projection matrix
    v.m_pos = glm::inverse(camera.getProjectionMatrix()) * v.m_pos;
    // Inverse view matrix
    v.m_pos = glm::inverse(camera.getViewMatrix()) * v.m_pos;
    return v;
}

Camera &Rasterizer::getCamera() {
    return camera;
}

int Rasterizer::getWidth() const {
    return width;
}

int Rasterizer::getHeight() const {
    return height;
}

/**
 * @brief Rasterizer::reflectionColor
 * @param v
 * @param color_
 * @return color which apply blinn-phong shading model
 */
glm::vec3 Rasterizer::reflectionColor(Vertex &v, glm::vec3& color_) {
    glm::vec3 normal = glm::normalize(glm::vec3(v.m_normal));

    glm::vec3 lightPos = glm::vec3(100, 100, 50);
    glm::vec3 fragPos = glm::vec3(pixelSpaceToWorldSpace(v).m_pos);
    glm::vec3 lightDir = glm::normalize(lightPos - fragPos);
    glm::vec3 viewDir = glm::normalize(glm::vec3(-camera.getForwardDir()));
    glm::vec3 halfDir = glm::normalize(lightDir + viewDir);

    float diffuse = std::max(0.f, glm::dot(normal, lightDir));
    float ambient = 0.5f;
    float specular = std::pow(std::max(0.f, glm::dot(normal, halfDir)), 200);

    glm::vec3 ambientColor = ambient * color_;
    glm::vec3 diffuseColor = diffuse * color_;
    glm::vec3 specularColor = specular * glm::vec3(255.0f);

    glm::vec3 finalColor = ambientColor + diffuseColor + specularColor;

    finalColor.r = std::min(finalColor.r, 255.f);
    finalColor.g = std::min(finalColor.g, 255.f);
    finalColor.b = std::min(finalColor.b, 255.f);

    return finalColor;
}


float Rasterizer::computeTriangleArea(const glm::vec2 &v1,
                                      const glm::vec2 &v2,
                                      const glm::vec2 &v3) const {
    // TODO: Calculate the 2D area of the triangle
    //       composed of the three input vertex positions
    glm::vec3 p1p2(v2 - v1, 0.f);
    glm::vec3 p1p3(v3 - v1, 0.f);
    float area = glm::length(glm::cross(p1p2, p1p3)) * 0.5;
    return area;
}

BarycentricWeights Rasterizer::computeBarycentricWeights(const glm::vec2 &fragPos,
                                                         const glm::vec2 &v1,
                                                         const glm::vec2 &v2,
                                                         const glm::vec2 &v3) const {
    // TODO: Compute the barycentric interpolation weights
    //       for input fragment position fragPos relative to the three
    //       input triangle vertex positions v1, v2, and v3.
    //       The weights should sum to 1.0.
    float totalArea = computeTriangleArea(v1, v2, v3);

    if (totalArea == 0) {
        return BarycentricWeights(0, 0, 0);
    }

    float area1 = computeTriangleArea(fragPos, v2, v3);
    float area2 = computeTriangleArea(fragPos, v1, v3);
    float area3 = computeTriangleArea(fragPos, v1, v2);

    float w1 = area1 / totalArea;
    float w2 = area2 / totalArea;
    float w3 = area3 / totalArea;

    return BarycentricWeights(w1, w2, w3);
}

float Rasterizer::perspectiveCorrectInterpolateZ(float z1, float z2, float z3,
                                                 const BarycentricWeights &baryWeights) const {
    // TODO: Use the perspective-correct Z interpolation formula
    //       to calculate the camera-space Z coordinate of the fragment
    //       whose barycentric weights have been passed in.
    float reciprocalZ = baryWeights.s1 / z1 + baryWeights.s2 / z2 + baryWeights.s3 / z3;
    float z = 1.f / reciprocalZ;
    return z;
}

// template<class T>
// T Rasterizer::perspectiveCorrectInterpolateAttrib(const T &v1Attrib,
//                                                   const T &v2Attrib,
//                                                   const T &v3Attrib,
//                                                   float z1, float z2, float z3,
//                                                   float fragmentZcoord,
//                                                   const BarycentricWeights &baryWeights) const {
//     // TODO: Use the perspective-correct vertex attribute interpolation
//     //       formula to interpolate the three input vertex attributes based
//     //       on the input barycentric weights and input vertex and fragment Z coordinates.
//     // You will use this function to interpolate UV coordinates and vertex normals,
//     // at the very least.

//     // Example invocation of this function:
//     // glm::vec2 uv = perspectiveCorrectInterpolateAttrib<glm::vec2>(uv1, uv2, uv3,
//     //                                                               z1, z2, z3,
//     //                                                               fragZ,
//     //                                                               baryWeights);
// }

