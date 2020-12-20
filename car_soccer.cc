#include <stdlib.h> // for rand
#include "car_soccer.h"
#include "config.h"


CarSoccer::CarSoccer() : GraphicsApp(1024,768, "Car Soccer") {
    // Define a search path for finding data files (images and shaders)
    searchPath_.push_back(".");
    searchPath_.push_back("./data");
    searchPath_.push_back(DATA_DIR_INSTALL);
    searchPath_.push_back(DATA_DIR_BUILD);
}


CarSoccer::~CarSoccer() {
}


Vector2 CarSoccer::joystick_direction() {
    Vector2 dir;
    if (IsKeyDown(GLFW_KEY_LEFT))
        dir[0]--;
    if (IsKeyDown(GLFW_KEY_RIGHT))
        dir[0]++;
    if (IsKeyDown(GLFW_KEY_UP))
        dir[1]++;
    if (IsKeyDown(GLFW_KEY_DOWN))
        dir[1]--;
    return dir;
}


void CarSoccer::LaunchBall() {
    ball_.Reset();
    float x, y, z;

    // set random velocity constrained by the boundaries of the playing field, and offset by the radius of the ball
    x = -40 + ball_.radius() + static_cast <float> (rand()) 
        /( static_cast <float> (RAND_MAX/((40 - ball_.radius()) - (-40 + ball_.radius()))));
    y = ball_.radius() + static_cast <float> (rand()) 
        /( static_cast <float> (RAND_MAX/((35 - ball_.radius()) - ball_.radius())));
    z = -50 + ball_.radius() + static_cast <float> (rand()) 
        /( static_cast <float> (RAND_MAX/((50 - ball_.radius()) - (-50 + ball_.radius()))));
    Vector3 v(x,y,z);
    ball_.set_velocity(v);
}


void CarSoccer::OnSpecialKeyDown(int key, int scancode, int modifiers) {
    if (key == GLFW_KEY_SPACE) {
        LaunchBall();
    }
}


void CarSoccer::DetectBoundaryContact() {
    // check if intersection/touching ground
    if (ball_.position().y() <= ball_.radius() || ball_.position().y() >= 35.0 - ball_.radius()) { // if ball is touching or intersecting the ground
        while (ball_.position().y() > 35.0 - ball_.radius()) {
            ball_.set_position(Point3(
                ball_.position().x(),
                ball_.position().y() - 0.01,
                ball_.position().z()
            ));
        }
        while (ball_.position().y() < ball_.radius()) {
            ball_.set_position(Point3(
                ball_.position().x(),
                ball_.position().y() + 0.01,
                ball_.position().z()
            ));
        }
        ball_.set_velocity(Vector3(  // update its velocity from collision and friction
            ball_.velocity().x() * 0.75,
            ball_.velocity().y() * (-0.75),
            ball_.velocity().z() * 0.75
        ));
    }
    // check if ball is intersecting/touching left and right sides of boundaries
    if (ball_.position().x() >= 40.0 - ball_.radius() || ball_.position().x() <= -40.0 + ball_.radius()) {
        while (ball_.position().x() > 40.0 - ball_.radius()) {
            ball_.set_position(Point3(
                ball_.position().x() - 0.01,
                ball_.position().y(),
                ball_.position().z()
            ));
        }
        while (ball_.position().x() < -40.0 + ball_.radius()) {
            ball_.set_position(Point3(
                ball_.position().x() + 0.01,
                ball_.position().y(),
                ball_.position().z()
            ));
        }
        ball_.set_velocity(Vector3(  // update its velocity from collision and friction
            ball_.velocity().x() * (-0.75),
            ball_.velocity().y() * 0.75,
            ball_.velocity().z() * 0.75
        ));
    }
    //check if ball is intersecting/touching near and far side of boundaries
    if (ball_.position().z() >= 50.0 - ball_.radius() || ball_.position().z() <= -50.0 + ball_.radius()) {
        while (ball_.position().z() > 50.0 - ball_.radius()) {
            ball_.set_position(Point3(
                ball_.position().x(),
                ball_.position().y(),
                ball_.position().z() - 0.01
            ));
        }
        while (ball_.position().z() < -50.0 + ball_.radius()) {
            ball_.set_position(Point3(
                ball_.position().x(),
                ball_.position().y(),
                ball_.position().z() + 0.01
            ));
        }
        ball_.set_velocity(Vector3(  // update its velocity from collision and friction
            ball_.velocity().x() * 0.75,
            ball_.velocity().y() * 0.75,
            ball_.velocity().z() * (-0.75)
        ));
    }
}


void CarSoccer::KeepCarInBounds() {
    // check if car is intersecting/touching left and right sides of boundaries
    if (car_.position().x() >= 40.0 - car_.collision_radius() || car_.position().x() <= -40.0 + car_.collision_radius()) {
        while (car_.position().x() > 40.0 - car_.collision_radius()) {
            car_.set_position(Point3(
                car_.position().x() - 0.01,
                car_.position().y(),
                car_.position().z()
            ));
        }
        while (car_.position().x() < -40.0 + car_.collision_radius()) {
            car_.set_position(Point3(
                car_.position().x() + 0.01,
                car_.position().y(),
                car_.position().z()
            ));
        }
    }
    //check if car is intersecting/touching near and far side of boundaries
    if (car_.position().z() >= 50.0 - car_.collision_radius() || car_.position().z() <= -50.0 + car_.collision_radius()) {
        while (car_.position().z() > 50.0 - car_.collision_radius()) {
            car_.set_position(Point3(
                car_.position().x(),
                car_.position().y(),
                car_.position().z() - 0.01
            ));
        }
        while (car_.position().z() < -50.0 + car_.collision_radius()) {
            car_.set_position(Point3(
                car_.position().x(),
                car_.position().y(),
                car_.position().z() + 0.01
            ));
        }
    }
}


void CarSoccer::UpdateCar(float timeStep) {
    float angle;
    Vector2 direction = joystick_direction();

    // calculate new speed
    float newSpeed = car_.speed() + (0.12 * direction[1] - 0.99 * car_.speed()) * timeStep;
    car_.set_speed(newSpeed);
    double turnRate = 20.0;

    Point3 pos = car_.position();

    // get angle of car
    if (direction[0] == -1) {
        car_.set_angle(car_.angle() + (turnRate * car_.speed() * timeStep));
    } 
    if (direction[0] == 1) {
        car_.set_angle(car_.angle() - (turnRate * car_.speed() * timeStep));
    }
    Vector3 car_vector(0,0,0);
    if (direction == Vector2(0,0)) {
        car_.set_speed(0);
    }

    // set position of car according to speed and angle
    car_vector[0] = cos(car_.angle() + 3.14159 / 2) * car_.speed();
    car_vector[2] = -sin(car_.angle() + 3.14159 / 2) * car_.speed();
    car_.set_position(car_.position() + car_vector * 2);
}


double CarSoccer::GetDistance() {
    return sqrt(
        pow(ball_.position().x() - car_.position().x(), 2) +
        pow(ball_.position().y() - car_.position().y(), 2) +
        pow(ball_.position().z() - car_.position().z(), 2)
    );
}


void CarSoccer::DetectCarBallContact(float timeStep) {
    double distance = GetDistance();
    double radiusSum = double(ball_.radius() + car_.collision_radius());

    if (distance <= radiusSum) {
        double x, y, z;
        Vector3 collisionNormal = Vector3::Normalize(ball_.position() - car_.position());

        // move ball position along collisionNormal so it's no longer intersecting
        while (distance < radiusSum) {
            ball_.set_position(ball_.position() + collisionNormal * timeStep);
            distance = GetDistance();
        }

        // compute relative velocity of ball (ball.velocity - car.velocity)
        Vector3 carVelocity = Vector3(0,0,0);
        carVelocity[0] = cos(car_.angle() + 3.14159 / 2) * car_.speed() * 150;
        carVelocity[2] = -sin(car_.angle() + 3.14159 / 2) * car_.speed() * 150;
        Vector3 relativeVelocity = ball_.velocity() - carVelocity;

        // reflect relative velocity about the collision normal 
        relativeVelocity = Vector3(relativeVelocity - 2 * (Vector3::Dot(relativeVelocity, collisionNormal)) * collisionNormal);

        // set new velocity of ball 
        ball_.set_velocity(carVelocity + relativeVelocity);
    } 
}


void CarSoccer::DetectGoal() {
    if (ball_.position().y() <= 10 && 
        ball_.position().x() <= 10.0 && ball_.position().x() >= -10.0 &&
        (ball_.position().z() >= 47 || ball_.position().z() <= -47)) {
            std::cout << "GOALLLLLLLLLLLLLLL !!!!!!!!!!!!!" << std::endl;
            car_.Reset();
            ball_.Reset();
        }

}


void CarSoccer::UpdateSimulation(float timeStep) {
    // take into consideration gravitational acceleration
    Vector3 gravity(0, -9.81 * timeStep, 0);
    Vector3 new_velocity = gravity + ball_.velocity();
    ball_.set_velocity(new_velocity);

    // update position of ball based on velocity
    ball_.set_position(ball_.position() + ball_.velocity() * timeStep);

    UpdateCar(timeStep);

    KeepCarInBounds();

    DetectBoundaryContact();

    DetectCarBallContact(timeStep);

    DetectGoal();
}


void CarSoccer::InitOpenGL() {
    // Set up the camera in a good position to see the entire field
    projMatrix_ = Matrix4::Perspective(60, aspect_ratio(), 1, 1000);
    modelMatrix_ = Matrix4::LookAt(Point3(0,60,70), Point3(0,0,10), Vector3(0,1,0));
 
    // Set a background color for the screen
    glClearColor(0.8,0.8,0.8, 1);
    
    // Load some image files we'll use
    fieldTex_.InitFromFile(Platform::FindFile("pitch.png", searchPath_));
    crowdTex_.InitFromFile(Platform::FindFile("crowd.png", searchPath_));
}


void CarSoccer::DrawNets(bool near) {
    Matrix4 m_loop;
    std::vector<Point3> loop;
    float halfNetWidth = 10.0;
    float endLineToMidFieldDistance;
    int r = 1;
    int g = 1;
    int b = 1;

    if (near) {
        endLineToMidFieldDistance = 50;
        g = 255;
    } else {
        endLineToMidFieldDistance = -50;
        r = 255;
    }

    Color netColor(r,g,b);
    
    // horizonal lines in net
    for (float i = 10.0; i >= 0.0; i--) {
        loop.push_back(Point3((-1.0) * halfNetWidth, i, endLineToMidFieldDistance));
        loop.push_back(Point3(halfNetWidth, i, endLineToMidFieldDistance));
    }
    
    // vertical lines in net
    for (float i = -10.0; i <= 10.0; i++) {
        loop.push_back(Point3(i, 0.0, endLineToMidFieldDistance));
        loop.push_back(Point3(i, 10.0, endLineToMidFieldDistance));
    }

    quickShapes_.DrawLines(modelMatrix_ * m_loop, viewMatrix_, projMatrix_, netColor, loop, QuickShapes::LinesType::LINES, 0.1);
}


void CarSoccer::DrawBoundaries() {
    Matrix4 m_loop;
    std::vector<Point3> loop;
    
    // draw nearside short line
    loop.push_back(Point3(-40.0, 0.0, 50.0));
    loop.push_back(Point3(40.0, 0.0, 50.0));

    // draw farside short line
    loop.push_back(Point3(-40.0, 0.0, -50.0));
    loop.push_back(Point3(40.0, 0.0, -50.0));

    // draw left boundary
    loop.push_back(Point3(-40.0, 0.0, -50.0));
    loop.push_back(Point3(-40.0, 0.0, 50.0));

    // draw right boundary
    loop.push_back(Point3(40.0, 0.0, -50.0));
    loop.push_back(Point3(40.0, 0.0, 50.0));

    // now we draw ceiling part of the boundaries

    // draw nearside short line
    loop.push_back(Point3(-40.0, 35.0, 50.0));
    loop.push_back(Point3(40.0, 35.0, 50.0));

    // draw farside short line
    loop.push_back(Point3(-40.0, 35.0, -50.0));
    loop.push_back(Point3(40.0, 35.0, -50.0));

    // draw left boundary
    loop.push_back(Point3(-40.0, 35.0, -50.0));
    loop.push_back(Point3(-40.0, 35.0, 50.0));

    // draw right boundary
    loop.push_back(Point3(40.0, 35.0, -50.0));
    loop.push_back(Point3(40.0, 35.0, 50.0));

    // draw near-left corner
    loop.push_back(Point3(-40.0, 0.0, 50));
    loop.push_back(Point3(-40.0, 35.0, 50));

    // draw near-right corner
    loop.push_back(Point3(40.0, 0.0, 50));
    loop.push_back(Point3(40.0, 35.0, 50));

    // draw far-right corner
    loop.push_back(Point3(40.0, 0.0, -50));
    loop.push_back(Point3(40.0, 35.0, -50));

    // draw far-left corner
    loop.push_back(Point3(-40.0, 0.0, -50));
    loop.push_back(Point3(-40.0, 35.0, -50));
    quickShapes_.DrawLines(modelMatrix_ * m_loop, viewMatrix_, projMatrix_, Color(1,1,255), loop, QuickShapes::LinesType::LINES, 0.1);
}


void CarSoccer::DrawUsingOpenGL() {
    // Draw the crowd as a fullscreen background image
    quickShapes_.DrawFullscreenTexture(Color(1,1,1), crowdTex_);
    
    // Draw the field with the field texture on it.
    Color col(16.0/255.0, 46.0/255.0, 9.0/255.0);
    Matrix4 M = Matrix4::Translation(Vector3(0,-0.201,0)) * Matrix4::Scale(Vector3(50, 1, 60));
    quickShapes_.DrawSquare(modelMatrix_ * M, viewMatrix_, projMatrix_, col);
    M = Matrix4::Translation(Vector3(0,-0.2,0)) * Matrix4::Scale(Vector3(40, 1, 50));
    quickShapes_.DrawSquare(modelMatrix_ * M, viewMatrix_, projMatrix_, Color(1,1,1), fieldTex_);
    
    // Draw the car
    Color carcol(0.8, 0.2, 0.2);
    Matrix4 Mcar =
        Matrix4::Translation(car_.position() - Point3(0,0,0)) *
        Matrix4::RotationY(car_.angle()) *
        Matrix4::Scale(car_.size()) *
        Matrix4::Scale(Vector3(0.5,0.5,0.5));
    quickShapes_.DrawCube(modelMatrix_ * Mcar, viewMatrix_, projMatrix_, carcol);
    
    // Draw the ball
    Color ballcol(1,1,1);
    Matrix4 Mball =
        Matrix4::Translation(ball_.position() - Point3(0,0,0)) *
        Matrix4::Scale(Vector3(ball_.radius(), ball_.radius(), ball_.radius()));
    quickShapes_.DrawSphere(modelMatrix_ * Mball, viewMatrix_, projMatrix_, ballcol);
    
    // Draw the ball's shadow -- this is a bit of a hack, scaling Y by zero
    // flattens the sphere into a pancake, which we then draw just a bit
    // above the ground plane.
    Color shadowcol(0.2,0.4,0.15);
    Matrix4 Mshadow =
        Matrix4::Translation(Vector3(ball_.position()[0], -0.1, ball_.position()[2])) *
        Matrix4::Scale(Vector3(ball_.radius(), 0, ball_.radius())) *
        Matrix4::RotationX(90);
    quickShapes_.DrawSphere(modelMatrix_ * Mshadow, viewMatrix_, projMatrix_, shadowcol);
    
    // Draw the goal nets
    DrawNets(true); // draws nearside net
    DrawNets(false); // draws farside net
    
    // Draw boundary of playing area
    DrawBoundaries();

    UpdateSimulation(0.01);
}