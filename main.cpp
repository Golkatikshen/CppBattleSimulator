#include <iostream>
#include <string>
#include <vector>
#include "header.h"
#include "raylib.h"
#include "Body.h"

using namespace std;

// Definitions
const int n_bodies = 1000;
vector<Body*> bodies;

Camera2D camera;
const int camera_speed = 10;
int camera_H_mov = 0;
int camera_V_mov = 0;

// Prototypes
void checkInput();
void addNBodies();
void updateCamera();
void updateWorld();
void drawWorld();


int main(void)
{
    // Initialization
    //--------------------------------------------------------------------------------------
    InitWindow(screenWidth, screenHeight, "C++ Battle Simulator");
    SetTargetFPS(60);                            // Set our game to run at 60 frames-per-second
    camera.offset = {0.0f, 0.0f};
    camera.target = {-100.0f, -100.0f};
    camera.rotation = 0.0f;
    camera.zoom = 1.0f;
    //--------------------------------------------------------------------------------------


    // Main game loop
    while (!WindowShouldClose())                // Detect window close button or ESC key
    {
        // Update
        //----------------------------------------------------------------------------------
        checkInput();
        updateCamera();
        updateWorld();
        //----------------------------------------------------------------------------------


        // Draw
        //----------------------------------------------------------------------------------
        BeginDrawing();
            BeginMode2D(camera);
                ClearBackground(BLACK);
                drawWorld();
            EndMode2D();

            DrawFPS(10, 10);
            DrawText(("N Bodies: " + to_string((int)bodies.size())).c_str(), 10, 30, 20, DARKGREEN);
        EndDrawing();
        //----------------------------------------------------------------------------------
    }


    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}

void checkInput()
{

    // start
    if(IsKeyPressed(KeyboardKey::KEY_X)) {
        addNBodies();
    }

    if(GetMouseWheelMove() != 0) {
        camera.zoom *= 1.0f-(-GetMouseWheelMove()*0.1f);
    }

    if(IsKeyPressed(KeyboardKey::KEY_D)) {
        camera_H_mov += 1;
    }

    if(IsKeyReleased(KeyboardKey::KEY_D)) {
        camera_H_mov -= 1;
    }

    if(IsKeyPressed(KeyboardKey::KEY_A)) {
        camera_H_mov -= 1;
    }

    if(IsKeyReleased(KeyboardKey::KEY_A)) {
        camera_H_mov += 1;
    }

    if(IsKeyPressed(KeyboardKey::KEY_S)) {
        camera_V_mov += 1;
    }

    if(IsKeyReleased(KeyboardKey::KEY_S)) {
        camera_V_mov -= 1;
    }

    if(IsKeyPressed(KeyboardKey::KEY_W)) {
        camera_V_mov -= 1;
    }

    if(IsKeyReleased(KeyboardKey::KEY_W)) {
        camera_V_mov += 1;
    }
}

void updateCamera()
{
    camera.target.x += camera_H_mov * camera_speed;
    camera.target.y += camera_V_mov * camera_speed;
}

void addNBodies()
{
    for(int i=0; i<n_bodies; i++) {
        bodies.push_back(new Body(GetRandomValue(0, screenWidth), GetRandomValue(0, screenHeight), 5));
    }
}

void updateWorld()
{
    for(int i=0; i<(int)bodies.size(); i++) {
        bodies[i]->updatePosition();
    }

    for(int i=0; i<(int)bodies.size()-1; i++) {
        for(int j=i+1; j<(int)bodies.size(); j++) {
            bodies[i]->resolveCollision(bodies[j]);
        }
    }
}

void drawWorld()
{
    for(int i=0; i<(int)bodies.size(); i++) {
        bodies[i]->display();
    }
}