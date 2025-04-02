
#ifndef UTILITY_HPP
#define UTILITY_HPP

#include "Manifold.hpp"

using namespace sf;

inline Vector change(Vector2f vector);

inline float distance(Vector centerA, Vector centerB);

inline float distanceSquared(Vector centerA, Vector centerB);

inline Vector findArithmeticMean(std::vector<Vector> vertices);

inline int findClosestPointOnPolygon(Vector circleCenter, std::vector<Vector> vertices);

inline std::vector<Vector> getVertices(sf::RectangleShape* rectangle);

inline void projectVertices(std::vector<Vector> vertices, Vector axis, float& min, float& max);

inline void projectCircle(Vector center, float radius, Vector axis, float& min, float& max);

inline void pointSegmentDistance(Vector point, Vector s1, Vector s2, float& distanceSq, Vector& contact);

inline void findContactPoint(Vector centerA, float radiusA, Vector centerB, Vector& contactPoint);

inline void findContactPoint(Vector circleCenter, float circleRadius, Vector polygonCenter, std::vector<Vector> vertices, Vector& contactPoint);

inline void findContactPoints(Object obj1, Object obj2, Vector& contact1, Vector& contact2, int& contactCount);

inline void findContactPoints(std::vector<Vector> verticesA, std::vector<Vector> verticesB, Vector& contact1, Vector& contact2, int& contactCount);

inline bool intersectAABBs(AABB a, AABB b);

inline bool intersectCircles(sf::CircleShape* circleA, sf::CircleShape* circleB, Vector& normal, float& depth);

inline bool intersectPolygons(Vector centerA, std::vector<Vector> verticesA, Vector centerB, std::vector<Vector> verticesB, Vector& normal, float& depth);

inline bool intersectCirclePolygons(Vector circleCenter, float circleRadius, Vector polygonCenter, std::vector<Vector> vertices, Vector& normal, float& depth);

inline bool collide(Object* obj1, Object* obj2, Vector& normal, float& depth);

inline void resolveCollision(Manifold contact);

#endif