#ifndef BROADPHASE_HPP
#define BROADPHASE_HPP

#include <vector>
#include <algorithm>
#include "Object.hpp"
#include "AABB.hpp"

struct SweepEntry{
    Object* obj;
    float minX;
    float maxX;
    float minY;
    float maxY;
};

inline void generatePotentialPairs(const std::vector<Object*>& objects, std::vector<std::pair<Object*, Object*>>& outPairs){

    std::vector<SweepEntry> entries;
    entries.reserve(objects.size());

    for(Object* obj : objects){

        AABB aabb = obj->getAABB();
        entries.push_back({obj, aabb.min.x, aabb.max.x, aabb.min.y, aabb.max.y});

    }

    std::sort(entries.begin(), entries.end(), [](const SweepEntry& lhs, const SweepEntry& rhs){
        return lhs.minX < rhs.minX;
    });

    outPairs.clear();

    for(size_t i = 0; i < entries.size(); ++i){

        const SweepEntry& entryA = entries[i];

        for(size_t j = i + 1; j < entries.size(); ++j){

            const SweepEntry& entryB = entries[j];

            if(entryB.minX > entryA.maxX){
                break;
            }

            if(entryB.maxY < entryA.minY || entryB.minY > entryA.maxY){
                continue;
            }

            Object* obj1 = entryA.obj;
            Object* obj2 = entryB.obj;

            if(obj1 == obj2) continue;

            outPairs.emplace_back(obj1, obj2);

        }

    }

}

#endif
