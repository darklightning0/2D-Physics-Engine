#ifndef JOINT_HPP
#define JOINT_HPP

#include "Object.hpp"
#include <algorithm>
#include <cmath>

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

inline float det2x2(float a, float b, float c, float d){
    return a * d - b * c;
}

inline void invert2x2(float a, float b, float c, float d, float& outA, float& outB, float& outC, float& outD){
    float det = det2x2(a, b, c, d);
    if(std::fabs(det) < 1e-6f){
        outA = outD = 0.f;
        outB = outC = 0.f;
        return;
    }
    float invDet = 1.f / det;
    outA =  d * invDet;
    outB = -b * invDet;
    outC = -c * invDet;
    outD =  a * invDet;
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

    // cached per-step data
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

struct RevoluteJoint{

    Object* objA;
    Object* objB;
    Vector localAnchorA;
    Vector localAnchorB;
    float biasFactor;
    float softness;
    bool enabled;

    Vector rA;
    Vector rB;
    float mass00, mass01, mass10, mass11;
    Vector bias;
    Vector impulse;

    RevoluteJoint(Object* a, Object* b, Vector anchorA, Vector anchorB, float biasFactor = 0.2f, float softness = 0.f)
        : objA(a), objB(b), localAnchorA(anchorA), localAnchorB(anchorB),
          biasFactor(biasFactor), softness(softness), enabled(true),
          rA(Vector::Zero()), rB(Vector::Zero()),
          mass00(0.f), mass01(0.f), mass10(0.f), mass11(0.f),
          bias(Vector::Zero()), impulse(Vector::Zero()){}
};

inline void revoluteJointPreStep(RevoluteJoint& joint, float dt){

    if(!joint.enabled || !joint.objA || !joint.objB) return;

    joint.rA = rotateLocalToWorld(joint.localAnchorA, joint.objA->angle);
    joint.rB = rotateLocalToWorld(joint.localAnchorB, joint.objB->angle);

    float mA = joint.objA->invMass;
    float mB = joint.objB->invMass;
    float iA = joint.objA->invMoI;
    float iB = joint.objB->invMoI;

    float k11 = mA + mB + joint.rA.y * joint.rA.y * iA + joint.rB.y * joint.rB.y * iB;
    float k12 = -joint.rA.x * joint.rA.y * iA - joint.rB.x * joint.rB.y * iB;
    float k22 = mA + mB + joint.rA.x * joint.rA.x * iA + joint.rB.x * joint.rB.x * iB;

    if(joint.softness > 0.f){
        float sTerm = joint.softness / std::max(dt, 0.0001f);
        k11 += sTerm;
        k22 += sTerm;
    }

    invert2x2(k11, k12, k12, k22, joint.mass00, joint.mass01, joint.mass10, joint.mass11);

    Vector worldA = joint.objA->position + joint.rA;
    Vector worldB = joint.objB->position + joint.rB;
    Vector C = worldB - worldA;
    float beta = joint.biasFactor / std::max(dt, 0.0001f);
    joint.bias = C * beta;

    Vector P = joint.impulse;

    if(joint.objA->invMass > 0.f){
        joint.objA->linearVelocity = joint.objA->linearVelocity - P * joint.objA->invMass;
        joint.objA->angularVelocity = joint.objA->angularVelocity - Vector::cross(joint.rA, P) * joint.objA->invMoI;
    }

    if(joint.objB->invMass > 0.f){
        joint.objB->linearVelocity = joint.objB->linearVelocity + P * joint.objB->invMass;
        joint.objB->angularVelocity = joint.objB->angularVelocity + Vector::cross(joint.rB, P) * joint.objB->invMoI;
    }
}

inline void revoluteJointApplyImpulse(RevoluteJoint& joint){

    if(!joint.enabled) return;

    Vector velA = joint.objA->linearVelocity + Vector(-joint.rA.y, joint.rA.x) * joint.objA->angularVelocity;
    Vector velB = joint.objB->linearVelocity + Vector(-joint.rB.y, joint.rB.x) * joint.objB->angularVelocity;
    Vector relVel = velB - velA + joint.bias;

    Vector lambda;
    lambda.x = -(joint.mass00 * relVel.x + joint.mass01 * relVel.y);
    lambda.y = -(joint.mass10 * relVel.x + joint.mass11 * relVel.y);

    joint.impulse = joint.impulse + lambda;

    if(joint.objA->invMass > 0.f){
        joint.objA->linearVelocity = joint.objA->linearVelocity - lambda * joint.objA->invMass;
        joint.objA->angularVelocity = joint.objA->angularVelocity - Vector::cross(joint.rA, lambda) * joint.objA->invMoI;
    }

    if(joint.objB->invMass > 0.f){
        joint.objB->linearVelocity = joint.objB->linearVelocity + lambda * joint.objB->invMass;
        joint.objB->angularVelocity = joint.objB->angularVelocity + Vector::cross(joint.rB, lambda) * joint.objB->invMoI;
    }
}

#endif
