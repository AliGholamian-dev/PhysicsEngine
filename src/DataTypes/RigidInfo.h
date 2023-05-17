#pragma once


struct RigidInfo final {
    double mass;
    double area;
    double density;
    double restitution;
    double rotationalInertia;
    double staticFriction;
    double dynamicFriction;
};

struct InverseRigidInfo final {
    double inverseMass;
    double inverseArea;
    double inverseDensity;
    double inverseRestitution;
    double inverseRotationalInertia;
    double inverseStaticFriction;
    double inverseDynamicFriction;
};