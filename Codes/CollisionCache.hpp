#ifndef COLLISION_CACHE_HPP
#define COLLISION_CACHE_HPP

#include <cstddef>
#include <cstdint>
#include "Vector.hpp"

class Object;

struct PairKey{
    Object* a;
    Object* b;

    static PairKey make(Object* obj1, Object* obj2){
        if(obj1 <= obj2){
            return {obj1, obj2};
        }
        return {obj2, obj1};
    }

    bool operator==(const PairKey& other) const{
        return a == other.a && b == other.b;
    }
};

struct PairKeyHash{
    std::size_t operator()(const PairKey& key) const noexcept{
        auto ha = reinterpret_cast<std::uintptr_t>(key.a);
        auto hb = reinterpret_cast<std::uintptr_t>(key.b);
        return (ha >> 3) ^ (hb << 1);
    }
};

struct CachedManifold{
    Vector normal = Vector::Zero();
    float depth = 0.f;
    Vector contact1 = Vector::Zero();
    Vector contact2 = Vector::Zero();
    int contactCount = 0;
    int framesLeft = 0;
    bool active = false;
};

#endif
