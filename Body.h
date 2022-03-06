#ifndef BODY
#define BODY

#include <iostream>
#include "header.h"
#include "raylib.h"
#include "raymath.h"

using namespace std;


class Body
{
private:
    double distanceToBody(Body b)
    {
        return sqrt(
            (b.pos.x - pos.x) * (b.pos.x - pos.x) + 
            (b.pos.y - pos.y) * (b.pos.y - pos.y)
        );
    }

    bool isColliding(Body b)
    {
        return distanceToBody(b) < radius + b.radius;
    }

public:
    Vector2 pos;
    Vector2 acc;
    Vector2 vel;
    float radius;
    float mass;

    Body(float _x, float _y, float _radius)
    {
        pos.x = _x;
        pos.y = _y;
        radius = _radius;
        mass = 1;

        vel.x = GetRandomValue(-10, 10);
        vel.y = GetRandomValue(-10, 10);
        vel = Vector2Normalize(vel);
        vel = Vector2Scale(vel, 3);
    }

    void display()
    {
        DrawCircle(pos.x, pos.y, radius, RED);
    }

    void updatePosition()
    {
        pos = Vector2Add(pos, vel);
        
        if(pos.x-radius < 0) {
            pos.x = radius;
            vel.x *= -1;
        }

        if(pos.x+radius > mapBoundsLength) {
            pos.x = mapBoundsLength-radius;
            vel.x *= -1;
        }

        if(pos.y-radius < 0) {
            pos.y = radius;
            vel.y *= -1;
        }

        if(pos.y+radius > mapBoundsLength) {
            pos.y = mapBoundsLength-radius;
            vel.y *= -1;
        }
    }

    void resolveCollision(Body* b)
    {
        double dist = distanceToBody(*b);
        double sum_rads = radius + b->radius;

        // if it is not colliding with the body, return
        if(dist > sum_rads)
            return;

        // if it is colliding, calculate direction and distance to resolve overlap
        double angle = atan2(b->pos.y - pos.y, b->pos.x - pos.x);
        double dist_to_move = sum_rads - dist;


        // bouncing effect
        Vector2 tangent_vector;
        tangent_vector.y = -( b->pos.x - pos.x );
        tangent_vector.x = b->pos.y - pos.y;

        tangent_vector = Vector2Normalize(tangent_vector);

        Vector2 relative_velocity;
        relative_velocity.x = b->vel.x - vel.x; 
        relative_velocity.y = b->vel.y - vel.y;

        float length = Vector2DotProduct(relative_velocity, tangent_vector);

        Vector2 velocity_component_on_tangent;
        velocity_component_on_tangent = Vector2Scale(tangent_vector, length);

        Vector2 velocity_component_perpendicular_to_tangent;
        velocity_component_perpendicular_to_tangent = Vector2Subtract(velocity_component_on_tangent, relative_velocity);

        vel.x -= velocity_component_perpendicular_to_tangent.x;
        vel.y -= velocity_component_perpendicular_to_tangent.y;

        b->vel.x += velocity_component_perpendicular_to_tangent.x;
        b->vel.y += velocity_component_perpendicular_to_tangent.y;

        // apply resolution to b coords
        b->pos.x += cos(angle) * dist_to_move;
        b->pos.y += sin(angle) * dist_to_move;
    }
};

#endif