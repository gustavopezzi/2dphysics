#include "../Graphics.h"
#include "Body.h"
#include <math.h>
#include <iostream>

Body::Body(const Shape& shape, float x, float y, float mass) {
    this->shape = shape.Clone();
    this->position = Vec2(x, y);
    this->velocity = Vec2(0, 0);
    this->acceleration = Vec2(0, 0);
    this->rotation = 0.0;
    this->angularVelocity = 0.0;
    this->angularAcceleration = 0.0;
    this->sumForces = Vec2(0, 0);
    this->sumTorque = 0.0;
    this->restitution = 0.6;
    this->friction = 0.7;
    this->mass = mass;
    if (mass != 0.0) {
        this->invMass = 1.0 / mass;
    } else {
        this->invMass = 0.0;
    }
    I = shape.GetMomentOfInertia() * mass;
    if (I != 0.0) {
        this->invI = 1.0 / I;
    } else {
        this->invI = 0.0;
    }
    this->shape->UpdateVertices(rotation, position);
    std::cout << "Body constructor called!" << std::endl;
}

Body::~Body() {
    delete shape;
    SDL_DestroyTexture(texture);
    std::cout << "Body destructor called!" << std::endl;
}

void Body::SetTexture(const char* textureFileName) {
    SDL_Surface* surface = IMG_Load(textureFileName);
    if (surface) {
        texture = SDL_CreateTextureFromSurface(Graphics::renderer, surface);
        SDL_FreeSurface(surface);
    }
}

bool Body::IsStatic() const {
    const float epsilon = 0.005f;
    return fabs(invMass - 0.0) < epsilon;
}

void Body::AddForce(const Vec2& force) {
    sumForces += force;
}

void Body::AddTorque(float torque) {
    sumTorque += torque;
}

void Body::ClearForces() {
    sumForces = Vec2(0.0, 0.0);
}

void Body::ClearTorque() {
    sumTorque = 0.0;
}

Vec2 Body::LocalSpaceToWorldSpace(const Vec2& point) const {
    Vec2 rotated = point.Rotate(rotation);
	return rotated + position;
}

Vec2 Body::WorldSpaceToLocalSpace(const Vec2& point) const {
    float translatedX = point.x - position.x;
    float translatedY = point.y - position.y;
    float rotatedX = cos(-rotation) * translatedX - sin(-rotation) * translatedY;
    float rotatedY = cos(-rotation) * translatedY + sin(-rotation) * translatedX;
	return Vec2(rotatedX, rotatedY);
}

void Body::ApplyImpulseLinear(const Vec2& j) {
    if (IsStatic())
        return;
    velocity += j * invMass;
}

void Body::ApplyImpulseAngular(const float j) {
    if (IsStatic())
        return;
    angularVelocity += j * invI;
}

void Body::ApplyImpulseAtPoint(const Vec2& j, const Vec2& r) {
    if (IsStatic())
        return;
    velocity += j * invMass;
    angularVelocity += r.Cross(j) * invI;
}

void Body::IntegrateForces(const float dt) {
    if (IsStatic())
        return;

    // Find the acceleration based on the forces that are being applied and the mass
    acceleration = sumForces * invMass;

    // Integrate the acceleration to find the new velocity
    velocity += acceleration * dt;

    // Find the angular acceleration based on the torque that is being applied and the moment of inertia
    angularAcceleration = sumTorque * invI;

    // Integrate the angular acceleration to find the new angular velocity
    angularVelocity += angularAcceleration * dt;

    // Clear all the forces and torque acting on the object before the next physics step
    ClearForces();
    ClearTorque();
}

void Body::IntegrateVelocities(const float dt) {
    if (IsStatic())
        return;

    // Integrate the velocity to find the new position
    position += velocity * dt;

    // Integrate the angular velocity to find the new rotation angle
    rotation += angularVelocity * dt;

    // Update the vertices to adjust them to the new position/rotation
    shape->UpdateVertices(rotation, position);
}
