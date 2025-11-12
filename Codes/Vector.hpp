
#ifndef VECTOR_HPP
#define VECTOR_HPP

#include <iostream>
#include <limits>
#include <SFML/Graphics.hpp>

class Vector{

    public:

        float x;
        float y;

        Vector(float x = 0, float y = 0) : x(x), y(y){}

        Vector operator-() const{

            return Vector(-x, -y);

        }

        Vector operator+(const Vector& other) const{

            return Vector(x + other.x, y + other.y);

        } 

        Vector operator+(float scalar) const{

            return Vector(x + scalar, y + scalar);

        }

        friend Vector operator+(float scalar, const Vector& vec){

            return Vector(vec.x + scalar, vec.y + scalar);

        }

        Vector operator-(const Vector& other) const{

            return Vector(x - other.x, y - other.y);

        }

        Vector operator*(const Vector& other) const{

            return Vector(x * other.x, y * other.y);

        }

        Vector operator*(float scalar) const{

            return Vector(x * scalar, y * scalar);

        }

        friend Vector operator*(float scalar, const Vector& vec){

            return Vector(vec.x * scalar, vec.y * scalar);

        }

        Vector operator/(float scalar) const{

            return Vector{x / scalar, y / scalar};

        }

        bool operator==(const Vector& other) const{

            return (x == other.x && y == other.y);

        }

        bool operator!=(const Vector& other) const{

            return (x != other.x || y != other.y);
            
        }

        friend std::ostream& operator<<(std::ostream& os, const Vector& vec){

            os << vec.x << ", " << vec.y;
            return os;
            
        }

        static float radToAngle(float rad){

            return rad * 180 / M_PI;

        }

        static float angleToRad(float angle){

            return angle * M_PI / 180;

        }

        static bool nearlyEqual(float a, float b, float threshold = 0.0005f){

            return std::fabs(a - b) <= threshold;

        }
    
        static bool nearlyEqual(const Vector& v1, const Vector& v2, float threshold = 0.0005f){

            return nearlyEqual(v1.x, v2.x, threshold) && nearlyEqual(v1.y, v2.y, threshold);

        }

        float magnitude() const{

            return sqrt(x * x + y * y);

        }

        float magnitudeSquared() const{

            return x * x + y * y;

        }

        Vector normalize() const{

            float mag = magnitude();
            return mag > 0 ? Vector(x / mag, y / mag) : Vector(0,0);

        }

        float static dot(Vector a, Vector b){

            return a.x * b.x + a.y * b.y;

        }

        float static cross(Vector a, Vector b){

            return a.x * b.y - a.y * b.x;

        }

        Vector min(const Vector& other) const{

            if(this->magnitude() > other.magnitude()){

                return other;

            }else{

                return *this;

            }

        }

        Vector max(const Vector& other) const{

            if(this->magnitude() < other.magnitude()){

                return other;

            }else{

                return *this;

            }

        }

        static Vector Zero(){

            return Vector(0.f, 0.f);

        }

};

#endif