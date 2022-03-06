#ifndef CAR
#define CAR

#include <iostream>
#include <random>
#include <Eigen/Dense>
#include "raylib.h"
#include "rlgl.h"
#include "Track.h"

using namespace std;
typedef Eigen::MatrixXd matrix;

default_random_engine generator;
uniform_real_distribution<double> distribution(-1, 1);

class Track;

class Car
{
private:
    bool running;
    float vel;
    float dir;
    float x;
    float y;
    const int in = 5;
    const int n1 = 3;
    const int n2 = 3;
    const int out = 2;
    
    
    void initializeMatrices()
    {
        h1 = Eigen::MatrixXd::Random(in, n1);
        h2 = Eigen::MatrixXd::Random(n1, n2);
        h3 = Eigen::MatrixXd::Random(n2, out);
    }

    double fastSigmoid(double x)
    {
        return x / (1 + abs(x));
    }

    template <typename Derived>
    std::string get_shape(const Eigen::EigenBase<Derived>& x)
    {
        std::ostringstream oss;
        oss  << "(" << x.rows() << ", " << x.cols() << ")";
        return oss.str();
    }


public:
    Ray rays[5];
    matrix h1;
    matrix h2;
    matrix h3;
    float score;

    Car(int _x, int _y, int _dir)
    {
        running = true;
        vel = 0;
        dir = _dir;
        x = _x;
        y = _y;
        score = 0;

        initializeMatrices();
    }

    void setPosDir(int _x, int _y, int _dir)
    {
        running = true;
        vel = 0;
        dir = _dir;
        x = _x;
        y = _y;
        score = 0;
    }

    void display(bool ray_on)
    {
        rlPushMatrix();
        rlTranslatef(x, y, 0);
        rlRotatef(dir, 0, 0, 1);
        DrawRectanglePro({-10, -5, 20, 10}, {0, 0}, 0, RED);
        rlPopMatrix();

        if(ray_on) {
            for(int i=0; i<5; i++)
                DrawRay(rays[i], GREEN);
        }
    }

    void displayHit(Track* track)
    {
        for(int i=0; i<5; i++) {
            float min_dist = -1;
            Vector3 min_pos = {0,0,0};

            for(int j=0; j<(int)track->walls.size(); j++) {
                RayCollision rc = GetRayCollisionQuad(rays[i], track->walls[j].p1, track->walls[j].p2, track->walls[j].p3, track->walls[j].p4);
                if(rc.hit){
                    if(rc.distance < min_dist || min_dist == -1) {
                        min_dist = rc.distance;
                        min_pos = rc.point;
                    }
                }
                
            }

            DrawCircle(min_pos.x, min_pos.y, 10, BLACK);
        }
    }


    void update(Track* track)
    {
        if(!running)
            return;

        if(checkWallCollisionOrOutOfTrack(track)) {
            running = false;
            return;
        }
        
        // network forward prop
        // calculation ray collision for inputs
        Eigen::VectorXd inputs = Eigen::VectorXd::Zero(5);
        for(int i=0; i<in; i++) {
            RayCollision rc = track->rayCheckOnTrack(rays[i]);
            float distance = track->points_dist*2;
            if(rc.hit) 
                distance = min(distance, rc.distance);
                
            inputs(i) = distance;
        }

        Eigen::VectorXd C1 = h1.transpose() * inputs;
        for(int i=0; i<n1; i++) {
            C1(i) = fastSigmoid(C1(i));
        }

        Eigen::VectorXd C2 = h2.transpose() * C1;
        for(int i=0; i<n2; i++) {
            C2(i) = fastSigmoid(C2(i));
        }
        
        Eigen::VectorXd O = h3.transpose() * C2;
        for(int i=0; i<out; i++) {
            O(i) = fastSigmoid(O(i));
        }

        vel += O(0);
        vel = min(5.0f, vel);
        vel = max(1.0f, vel);
        dir += 10*O(1);


        // update params
        x += vel*cos(dir*DEG2RAD);
        y += vel*sin(dir*DEG2RAD);

        for(int i=0; i<5; i++) {
            rays[i].position = {x, y};
            rays[i].direction = {cos((-70+(35*i)+dir)*DEG2RAD), sin((-70+(35*i)+dir)*DEG2RAD)};
        }
    }


    void crossOver(Car* p1, Car* p2)
    {
        for(int i=0; i<in*n1; i++){
            if(GetRandomValue(0, 1)) {
                h1(i) = p1->h1(i);
            }
            else {
                h1(i) = p2->h1(i);
            }
        }

        for(int i=0; i<n1*n2; i++){
            if(GetRandomValue(0, 1)) {
                h2(i) = p1->h2(i);
            }
            else {
                h2(i) = p2->h2(i);
            }
        }

        for(int i=0; i<n2*out; i++){
            if(GetRandomValue(0, 1)) {
                h3(i) = p1->h3(i);
            }
            else {
                h3(i) = p2->h3(i);
            }
        }
    }

    void mutation()
    {
        for(int i=0; i<in*n1; i++){
            if(!GetRandomValue(0, 15)) {
                h1(i) += distribution(generator);
            }
        }

        for(int i=0; i<n1*n2; i++){
            if(!GetRandomValue(0, 15)) {
                h2(i) += distribution(generator);
            }
        }

        for(int i=0; i<n2*out; i++){
            if(!GetRandomValue(0, 15)) {
                h3(i) += distribution(generator);
            }
        }
    }


    bool checkWallCollisionOrOutOfTrack(Track* track) {
        if(track->checkFinishLineCollision({x, y})) {
            track->n_car_arrived ++;
            score += 1000000/track->n_car_arrived;
            return true;
        }

        return track->checkWallCollision({x, y}) || track->distToCenter({x, y}) > track->points_dist;
    }

    int closestSegment(Track* track) {
        return track->closestSegment({x, y});
    }

    void calcPathAdvancementScore(Track* track)
    {
        int cs = closestSegment(track);
        score += 1000*(cs+1);

        score += Vector2Distance(track->points[cs], {x, y});
    }

    bool mouseOver()
    {
        return (Vector2Distance({x, y}, {(float)GetMouseX(), (float)GetMouseY()}) < 10);
    }

    void printInfos(Track* track)
    {
        cout << "score: " << score << endl;
        for(int i=0; i<in; i++) {
            RayCollision rc = track->rayCheckOnTrack(rays[i]);
            cout << "ray: " << (i+1) << ", hit: " << rc.hit << endl;
            cout << "ray: " << (i+1) << ", dist: " << rc.distance << endl;
        }
    }
};

#endif