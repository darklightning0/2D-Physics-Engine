#ifndef JOINT_HPP
#define JOINT_HPP

#include "Object.hpp"
#include <cmath>
#include <algorithm>

inline Vector rotateLocalToWorld(const Vector& local, float angle){
    float c = std::cos(angle);
    float s = std::sin(angle);
    return Vector(c * local.x - s * local.y, s * local.x + c * local.y);
}

inline Vector rotateWorldToLocal(const Vector& world, float angle){
    float c = std::cos(angle);
    float s = std::sin(angle);
    return Vector(c * world.x + s * world.y, -s * world.x + c * world.y);
}

struct DistanceJoint{
    Object* objA;
    Object* objB;
    Vector localAnchorA;
    Vector localAnchorB;
    float restLength;
    float frequencyHz;
    float dampingRatio;
    float impulse;
    bool enabled;

    Vector rA;
    Vector rB;
    Vector axis;
    float effectiveMass;
    float bias;
    float gamma;

    DistanceJoint(Object* a, Object* b, Vector anchorA, Vector anchorB, float length, float frequencyHz = 2.f, float dampingRatio = 0.7f)
        : objA(a), objB(b), localAnchorA(anchorA), localAnchorB(anchorB), restLength(length),
          frequencyHz(frequencyHz), dampingRatio(dampingRatio), impulse(0.f), enabled(true),
          rA(Vector::Zero()), rB(Vector::Zero()), axis(Vector::Zero()), effectiveMass(0.f), bias(0.f), gamma(0.f){}
};

inline void distanceJointPreStep(DistanceJoint& joint, float dt){

    if(!joint.enabled || !joint.objA || !joint.objB) return;

    joint.rA = rotateLocalToWorld(joint.localAnchorA, joint.objA->angle);
    joint.rB = rotateLocalToWorld(joint.localAnchorB, joint.objB->angle);

    Vector worldAnchorA = joint.objA->position + joint.rA;
    Vector worldAnchorB = joint.objB->position + joint.rB;

    Vector delta = worldAnchorB - worldAnchorA;
    float distance = std::max(delta.magnitude(), 0.0001f);
    joint.axis = delta * (1.f / distance);

    float crA = Vector::cross(joint.rA, joint.axis);
    float crB = Vector::cross(joint.rB, joint.axis);

    float invMass = joint.objA->invMass + joint.objB->invMass + (crA * crA) * joint.objA->invMoI + (crB * crB) * joint.objB->invMoI;

    joint.gamma = 0.f;
    joint.bias = 0.f;

    if(joint.frequencyHz > 0.f){

        float mass = invMass > 0.f ? 1.f / invMass : 0.f;
        float omega = 2.f * static_cast<float>(M_PI) * joint.frequencyHz;
        float d = 2.f * mass * joint.dampingRatio * omega;
        float k = mass * omega * omega;

        joint.gamma = dt * (d + dt * k);
        joint.gamma = joint.gamma > 0.f ? 1.f / joint.gamma : 0.f;
        joint.bias = (distance - joint.restLength) * dt * k * joint.gamma;
        joint.effectiveMass = invMass + joint.gamma > 0.f ? 1.f / (invMass + joint.gamma) : 0.f;

    }else{

        joint.effectiveMass = invMass > 0.f ? 1.f / invMass : 0.f;
        joint.bias = -(distance - joint.restLength) * 0.1f / std::max(dt, 0.0001f);

    }

    Vector impulseVec = joint.axis * joint.impulse;
    if(joint.objA->invMass > 0.f){
        joint.objA->linearVelocity = joint.objA->linearVelocity - impulseVec * joint.objA->invMass;
        joint.objA->angularVelocity = joint.objA->angularVelocity - Vector::cross(joint.rA, impulseVec) * joint.objA->invMoI;
    }

    if(joint.objB->invMass > 0.f){
        joint.objB->linearVelocity = joint.objB->linearVelocity + impulseVec * joint.objB->invMass;
        joint.objB->angularVelocity = joint.objB->angularVelocity + Vector::cross(joint.rB, impulseVec) * joint.objB->invMoI;
    }
}

inline void distanceJointApplyImpulse(DistanceJoint& joint){

    if(!joint.enabled || joint.effectiveMass == 0.f) return;

    Vector velA = joint.objA->linearVelocity + Vector(-joint.rA.y, joint.rA.x) * joint.objA->angularVelocity;
    Vector velB = joint.objB->linearVelocity + Vector(-joint.rB.y, joint.rB.x) * joint.objB->angularVelocity;

    Vector relativeVel = velB - velA;
    float relAlongAxis = Vector::dot(relativeVel, joint.axis);

    float lambda = -(relAlongAxis + joint.bias + joint.gamma * joint.impulse) * joint.effectiveMass;
    float prevImpulse = joint.impulse;
    joint.impulse += lambda;

    const float maxImpulse = 1000.f;
    joint.impulse = std::clamp(joint.impulse, -maxImpulse, maxImpulse);
    lambda = joint.impulse - prevImpulse;

    Vector impulseVec = joint.axis * lambda;

    if(joint.objA->invMass > 0.f){
        joint.objA->linearVelocity = joint.objA->linearVelocity - impulseVec * joint.objA->invMass;
        joint.objA->angularVelocity = joint.objA->angularVelocity - Vector::cross(joint.rA, impulseVec) * joint.objA->invMoI;
    }

    if(joint.objB->invMass > 0.f){
        joint.objB->linearVelocity = joint.objB->linearVelocity + impulseVec * joint.objB->invMass;
        joint.objB->angularVelocity = joint.objB->angularVelocity + Vector::cross(joint.rB, impulseVec) * joint.objB->invMoI;
    }
}

struct SpringJoint{
    Object* objA;
    Object* objB;
    Vector localAnchorA;
    Vector localAnchorB;
    float restLength;
    float stiffness;
    float damping;

    SpringJoint(Object* a, Object* b, const Vector& anchorA, const Vector& anchorB, float restLength, float stiffness, float damping)
        : objA(a), objB(b), localAnchorA(anchorA), localAnchorB(anchorB), restLength(restLength), stiffness(stiffness), damping(damping){}
};

inline void applySpringForces(SpringJoint& spring, float dt){

    if(!spring.objA || !spring.objB) return;

    Vector rA = rotateLocalToWorld(spring.localAnchorA, spring.objA->angle);
    Vector rB = rotateLocalToWorld(spring.localAnchorB, spring.objB->angle);

    Vector worldA = spring.objA->position + rA;
    Vector worldB = spring.objB->position + rB;

    Vector delta = worldB - worldA;
    float length = delta.magnitude();
    if(length < 1e-5f) return;

    Vector dir = delta * (1.f / length);

    Vector velA = spring.objA->linearVelocity + Vector(-rA.y, rA.x) * spring.objA->angularVelocity;
    Vector velB = spring.objB->linearVelocity + Vector(-rB.y, rB.x) * spring.objB->angularVelocity;
    float relVel = Vector::dot(velB - velA, dir);

    float displacement = length - spring.restLength;
    float forceMag = -spring.stiffness * displacement - spring.damping * relVel;
    Vector impulse = dir * (forceMag * dt);

    if(spring.objA->invMass > 0.f){
        spring.objA->linearVelocity = spring.objA->linearVelocity - impulse * spring.objA->invMass;
        spring.objA->angularVelocity = spring.objA->angularVelocity - Vector::cross(rA, impulse) * spring.objA->invMoI;
    }

    if(spring.objB->invMass > 0.f){
        spring.objB->linearVelocity = spring.objB->linearVelocity + impulse * spring.objB->invMass;
        spring.objB->angularVelocity = spring.objB->angularVelocity + Vector::cross(rB, impulse) * spring.objB->invMoI;
    }
}

#endif
