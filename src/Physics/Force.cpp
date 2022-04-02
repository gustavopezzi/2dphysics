#include "./Force.h"
#include <algorithm>

Vec2 Force::GenerateDragForce(const Body& body, float k) {
    Vec2 dragForce = Vec2(0, 0);
    if (body.velocity.MagnitudeSquared() > 0) {
        // Calculate the drag direction (inverse of velocity unit vector)
        Vec2 dragDirection = body.velocity.UnitVector() * -1.0;

        // Calculate the drag magnitude, k * |v|^2
        float dragMagnitude = k * body.velocity.MagnitudeSquared();

        // Generate the final drag force with direction and magnitude
        dragForce = dragDirection * dragMagnitude;
    }
    return dragForce;
}

Vec2 Force::GenerateFrictionForce(const Body& body, float k) {
    // Calculate the friction direction (inverse of velocity unit vector)
    Vec2 frictionDirection = body.velocity.UnitVector() * -1.0;

    // Calculate the friction magnitude (just k, for now)
    float frictionMagnitude = k;

    // Calculate the final resulting friction force vector
    Vec2 frictionForce = frictionDirection * frictionMagnitude;
    return frictionForce;
}

Vec2 Force::GenerateGravitationalForce(const Body& a, const Body& b, float G, float minDistance, float maxDistance) {
    // Calculate the distance between the two objects
    Vec2 d = (b.position - a.position);

    float distanceSquared = d.MagnitudeSquared();

    // Clamp the values of the distance (to allow for some insteresting visual effects)
    distanceSquared = std::clamp(distanceSquared, minDistance, maxDistance);

    // Calculate the direction of the attraction force
    Vec2 attractionDirection = d.UnitVector();

    // Calculate the strength of the attraction force
    float attractionMagnitude = G * (a.mass * b.mass) / distanceSquared;

    // Calculate the final resulting attraction force vector
    Vec2 attractionForce = attractionDirection * attractionMagnitude;
    return attractionForce;
}

Vec2 Force::GenerateSpringForce(const Body& body, Vec2 anchor, float restLength, float k) {
    // Calculate the distance between the anchor and the object
    Vec2 d = body.position - anchor;

    // Find the spring displacement considering the rest length
    float displacement = d.Magnitude() - restLength;

    // Calculate the direction of the spring force
    Vec2 springDirection = d.UnitVector();

    // Calculate the magnitude of the spring force
    float springMagnitude = -k * displacement;
    
    // Calculate the final resulting spring force vector
    Vec2 springForce = springDirection * springMagnitude;
    return springForce;
}
