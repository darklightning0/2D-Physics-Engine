
#ifndef UTILITY_HPP
#define UTILITY_HPP

#include "Object.hpp"

using namespace sf;

inline Vector change(Vector2f vector);

inline float distance(Vector centerA, Vector centerB);

inline Vector findArithmeticMean(std::vector<Vector> vertices);

inline int findClosestPointOnPolygon(Vector circleCenter, std::vector<Vector> vertices);

inline std::vector<Vector> getVerticles(sf::RectangleShape* rectangle);

inline void projectVertices(std::vector<Vector> vertices, Vector axis, float& min, float& max);

inline void projectCircle(Vector center, float radius, Vector axis, float& min, float& max);

inline bool intersectCircles(sf::CircleShape* circleA, sf::CircleShape* circleB, Vector& normal, float& depth);

inline bool intersectPolygons(std::vector<Vector> verticesA, std::vector<Vector> verticesB, Vector& normal, float& depth);

inline bool intersectCirclePolygons(Vector circleCenter, float circleRadius, Vector polygonCenter, std::vector<Vector> vertices, Vector& normal, float& depth);

inline bool intersectCirclePolygons(Vector circleCenter, float circleRadius, std::vector<Vector> vertices, Vector& normal, float& depth);

inline bool collide(Object* obj1, Object* obj2, Vector& normal, float& depth);

inline void resolveCollision(Object* shape1, Object* shape2, Vector normal, float depth);

#endif