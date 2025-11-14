#ifndef MATERIAL_HPP
#define MATERIAL_HPP

#include <string>

enum class Material{
    Default,
    Wood,
    Steel,
    Rubber,
    Ice,
    Mud
};

struct MaterialPairProperties{
    float staticFriction;
    float dynamicFriction;
    float rollingFriction;
};

inline float getMaterialRestitution(Material material){

    switch(material){
        case Material::Wood:    return 0.35f;
        case Material::Steel:   return 0.6f;
        case Material::Rubber:  return 0.85f;
        case Material::Ice:     return 0.25f;
        case Material::Mud:     return 0.15f;
        case Material::Default:
        default:                return 0.5f;
    }

}

inline MaterialPairProperties getMaterialPairProperties(Material a, Material b){

    const float frictionMatrix[6][6][3] = {
        // Default row
        {{0.6f, 0.45f, 0.02f}, {0.55f, 0.4f, 0.03f}, {0.4f, 0.25f, 0.015f}, {0.98f, 0.82f, 0.06f}, {0.12f, 0.06f, 0.001f}, {0.78f, 0.62f, 0.04f}},
        // Wood row
        {{0.55f, 0.4f, 0.03f}, {0.65f, 0.5f, 0.04f}, {0.45f, 0.3f, 0.02f}, {0.85f, 0.65f, 0.05f}, {0.18f, 0.08f, 0.005f}, {0.72f, 0.55f, 0.045f}},
        // Steel row
        {{0.4f, 0.25f, 0.015f}, {0.45f, 0.3f, 0.02f}, {0.7f, 0.5f, 0.04f}, {0.72f, 0.55f, 0.045f}, {0.07f, 0.035f, 0.003f}, {0.5f, 0.35f, 0.025f}},
        // Rubber row
        {{0.98f, 0.82f, 0.08f}, {0.85f, 0.65f, 0.07f}, {0.72f, 0.55f, 0.05f}, {1.05f, 0.95f, 0.1f}, {0.22f, 0.12f, 0.01f}, {0.9f, 0.72f, 0.09f}},
        // Ice row
        {{0.12f, 0.06f, 0.002f}, {0.18f, 0.08f, 0.0025f}, {0.07f, 0.035f, 0.001f}, {0.22f, 0.12f, 0.003f}, {0.03f, 0.02f, 0.0005f}, {0.1f, 0.05f, 0.0015f}},
        // Mud row
        {{0.78f, 0.62f, 0.05f}, {0.72f, 0.55f, 0.055f}, {0.5f, 0.35f, 0.03f}, {0.9f, 0.72f, 0.07f}, {0.1f, 0.05f, 0.006f}, {0.98f, 0.82f, 0.09f}}
    };

    int i = static_cast<int>(a);
    int j = static_cast<int>(b);

    MaterialPairProperties props;
    props.staticFriction = frictionMatrix[i][j][0];
    props.dynamicFriction = frictionMatrix[i][j][1];
    props.rollingFriction = frictionMatrix[i][j][2];
    return props;
}

inline const char* materialToString(Material material){
    switch(material){
        case Material::Wood: return "Wood";
        case Material::Steel: return "Steel";
        case Material::Rubber: return "Rubber";
        case Material::Ice: return "Ice";
        case Material::Mud: return "Mud";
        case Material::Default:
        default: return "Default";
    }
}

inline Material materialFromString(const std::string& name){
    if(name == "Wood") return Material::Wood;
    if(name == "Steel") return Material::Steel;
    if(name == "Rubber") return Material::Rubber;
    if(name == "Ice") return Material::Ice;
    if(name == "Mud") return Material::Mud;
    return Material::Default;
}

#endif
