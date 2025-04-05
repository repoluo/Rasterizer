#include "polygon.h"
#include <glm/gtx/transform.hpp>
#include <limits>


BarycentricWeights::BarycentricWeights(float s1, float s2, float s3)
    : s1(s1), s2(s2), s3(s3)
{}

Segment::Segment(float x1, float y1, float x2, float y2) :
    p1(Vertex(x1, y1)), p2(Vertex(x2, y2)) {
    if (x1 == x2) {
        gradient = std::numeric_limits<float>::infinity();
    } else {
        gradient = (y1 - y2) / (x1 - x2);
    }
}

Segment::Segment(const Vertex& v1, const Vertex& v2) : p1(v1), p2(v2) {
    float x1 = v1.m_pos[0];
    float y1 = v1.m_pos[1];
    float x2 = v2.m_pos[0];
    float y2 = v2.m_pos[1];
    if (x1 == x2) {
        gradient = std::numeric_limits<float>::infinity();
    } else {
        gradient = (y1 - y2) / (x1 - x2);
    }
}

/**
 * @brief Segment::getIntersection
 *        judge whether the segment has a intersection with a horizon line y
 *        if yes, change the float pointer x pointed value to x value of intersection point
 * @param y
 * @param x
 * @return whether the segment has a intersection with a horizon line y
 */
bool Segment::getIntersection(float y, float* x) const {
    bool result = false;
    if (y >= std::min(p1.m_pos[1], p2.m_pos[1]) && y <= std::max(p2.m_pos[1], p1.m_pos[1])) {
        result = true;
    }
    if (gradient == 0) {
        result = false;
    }
    if (result) {
        if (gradient == std::numeric_limits<float>::infinity()) {
            (*x) = p1.m_pos[0];
        } else {
            (*x) = ((1.f / gradient) * (y - p1.m_pos[1])) + p1.m_pos[0];
        }
    }
    return result;
}

/**
 * @brief Return the coordinate of all the point
 * @return (x1, y1, x2, y2)
 */
glm::vec4 Segment::getCoordinate() const {
    return glm::vec4(this->p1.m_pos[0], this->p1.m_pos[1], this->p2.m_pos[0], this->p2.m_pos[1]);
}

/**
 * @brief Segment::getP1Coordinate
 * @return (x1, y1)
 */
glm::vec2 Segment::getP1Coordinate() const {
    return glm::vec2(this->p1.m_pos[0], this->p1.m_pos[1]);
}

/**
 * @brief Segment::getP2Coordinate
 * @return (x2, y2)
 */
glm::vec2 Segment::getP2Coordinate() const {
    return glm::vec2(this->p2.m_pos[0], this->p2.m_pos[1]);
}

/**
 * @brief Polygon::Triangulate
 *        Triangulate a polygon and populate the list of triangles index into m_tris.
 */
void Polygon::Triangulate()
{
    //TODO: Populate list of triangles
    for (unsigned int i = 0; i < m_verts.size() - 2; i++) {
        Triangle tri = {0, i + 1, i + 2};
        m_tris.push_back(tri);
    }
}

// Creates a polygon from the input list of vertex positions and colors
Polygon::Polygon(const QString& name, const std::vector<glm::vec4>& pos, const std::vector<glm::vec3>& col)
    : m_tris(), m_verts(), m_name(name), mp_texture(nullptr), mp_normalMap(nullptr)
{
    for(unsigned int i = 0; i < pos.size(); i++)
    {
        m_verts.push_back(Vertex(pos[i], col[i], glm::vec4(), glm::vec2()));
    }
    Triangulate();
}

// Creates a regular polygon with a number of sides indicated by the "sides" input integer.
// All of its vertices are of color "color", and the polygon is centered at "pos".
// It is rotated about its center by "rot" degrees, and is scaled from its center by "scale" units
Polygon::Polygon(const QString& name, int sides, glm::vec3 color, glm::vec4 pos, float rot, glm::vec4 scale)
    : m_tris(), m_verts(), m_name(name), mp_texture(nullptr), mp_normalMap(nullptr)
{
    glm::vec4 v(0.f, 1.f, 0.f, 1.f);
    float angle = 360.f / sides;
    for(int i = 0; i < sides; i++)
    {
        glm::vec4 vert_pos = glm::translate(glm::vec3(pos.x, pos.y, pos.z))
                           * glm::rotate(rot, glm::vec3(0.f, 0.f, 1.f))
                           * glm::scale(glm::vec3(scale.x, scale.y, scale.z))
                           * glm::rotate(i * angle, glm::vec3(0.f, 0.f, 1.f))
                           * v;
        m_verts.push_back(Vertex(vert_pos, color, glm::vec4(), glm::vec2()));
    }

    Triangulate();
}

Polygon::Polygon(const QString &name)
    : m_tris(), m_verts(), m_name(name), mp_texture(nullptr), mp_normalMap(nullptr)
{}

Polygon::Polygon()
    : m_tris(), m_verts(), m_name("Polygon"), mp_texture(nullptr), mp_normalMap(nullptr)
{}

Polygon::Polygon(const Polygon& p)
    : m_tris(p.m_tris), m_verts(p.m_verts), m_name(p.m_name), mp_texture(nullptr), mp_normalMap(nullptr)
{
    if(p.mp_texture != nullptr)
    {
        mp_texture = new QImage(*p.mp_texture);
    }
    if(p.mp_normalMap != nullptr)
    {
        mp_normalMap = new QImage(*p.mp_normalMap);
    }
}

Polygon::~Polygon()
{
    delete mp_texture;
}

void Polygon::SetTexture(QImage* i)
{
    mp_texture = i;
}

void Polygon::SetNormalMap(QImage* i)
{
    mp_normalMap = i;
}

void Polygon::AddTriangle(const Triangle& t)
{
    m_tris.push_back(t);
}

void Polygon::AddVertex(const Vertex& v)
{
    m_verts.push_back(v);
}

void Polygon::ClearTriangles()
{
    m_tris.clear();
}

Triangle& Polygon::TriAt(unsigned int i)
{
    return m_tris[i];
}

Triangle Polygon::TriAt(unsigned int i) const
{
    return m_tris[i];
}

Vertex &Polygon::VertAt(unsigned int i)
{
    return m_verts[i];
}

Vertex Polygon::VertAt(unsigned int i) const
{
    return m_verts[i];
}

glm::vec3 GetImageColor(const glm::vec2 &uv_coord, const QImage* const image)
{
    if(image)
    {
        int X = glm::min(image->width() * uv_coord.x, image->width() - 1.0f);
        int Y = glm::min(image->height() * (1.0f - uv_coord.y), image->height() - 1.0f);
        QColor color = image->pixel(X, Y);
        return glm::vec3(color.red(), color.green(), color.blue());
    }
    return glm::vec3(255.f, 255.f, 255.f);
}
