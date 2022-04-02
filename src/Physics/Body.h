#ifndef BODY_H
#define BODY_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include "Vec2.h"
#include "Shape.h"

struct Body {
    // Linear motion
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;

    // Angular motion
    float rotation;
    float angularVelocity;
    float angularAcceleration;
    
    // Forces and torque
    Vec2 sumForces;
    float sumTorque;

    // Mass and Moment of Inertia
    float mass;
    float invMass;
    float I;
    float invI;

    // Coefficient of restitution (elasticity)
    float restitution;

    // Coefficient of friction
    float friction;

    // Pointer to the shape/geometry of this rigid body
    Shape* shape = nullptr;

    // Pointer to an SDL texture
    SDL_Texture* texture = nullptr;

    Body(const Shape& shape, float x, float y, float mass);
    ~Body();

    bool IsStatic() const;

    void AddForce(const Vec2& force);
    void AddTorque(float torque);
    void ClearForces();
    void ClearTorque();

    void SetTexture(const char* textureFileName);

    Vec2 LocalSpaceToWorldSpace(const Vec2& point) const;
    Vec2 WorldSpaceToLocalSpace(const Vec2& point) const;

    void ApplyImpulseLinear(const Vec2& j);
    void ApplyImpulseAngular(const float j);
    void ApplyImpulseAtPoint(const Vec2& j, const Vec2& r);

    void IntegrateLinear(float dt);
    void IntegrateAngular(float dt);

    void IntegrateForces(const float dt);
    void IntegrateVelocities(const float dt);
};

#endif
