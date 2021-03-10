// File:          my_controller.cpp
// Date:
// Description: Q-learning controller for Khepera1
// Author: Wahdan Hasan, Rama Al Sbeinaty
// Modifications: idk what this means lol but we made it

#pragma region include statements
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Supervisor.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#pragma endregion

#define M_PI 3.14159265358979323846

using namespace webots;


template<typename t>
struct Array2D
{
    t** array;
    int row_count;
    int column_count;
};

#pragma region enums
/* When rotating Clockwise 
    Right is 0 to -90
    Behind is -90 to -180, then switches to 180
    Left is 180 to 90
    Front is 90 to 0

*/

/* 
    Ive changed this so that clockwise rotation
    is represented from 360 to 0 degrees.
*/


enum DirectionAngle
{
    TopRight = 315,
    MiddleRight = 270,
    BottomRight = 225,
    BottomCenter = 180,
    BottomLeft = 135,
    MiddleLeft = 90,
    TopLeft = 45,
    TopCenter = 360,
};

enum TurnDirection
{
    Clockwise = 1,
    Anti_Clockwise = -1,
};
#pragma endregion

#pragma region function prototypes
void CreateAdjacenyMatrix(Array2D<float>*, int, int);
void RotateRobot(InertialUnit*, DirectionAngle, TurnDirection, TurnDirection);
TurnDirection GetTurnDirection(float, float);
float RadianToDegree(double);
bool CheckIfMovingDiagonally();
void Move();
void SetUp();
void CleanUp();
void StopMove();
#pragma endregion

/* Maybe change these to defines instead, more performant */
#pragma region global constants
const int NUMBER_OF_TILES_PER_METER = 2;
const float SIZE_OF_TILE = 0.5f;
const int NUMBER_OF_DIVISIONS = 1;
const float MOTOR_DEFAULT_SPEED = 10.0f;
const float WHEEL_RADIUS = 0.028f;
const float DISTANCE_BETWEEN_WHEELS = 0.052f;
#pragma endregion

#pragma region global variables
Supervisor* robot;
//Robot* robot;
Motor* left_motor;
Motor* right_motor;

PositionSensor* left_sensor;
PositionSensor* right_sensor;
DirectionAngle previous_direction;

float distance_to_travel_linear;
float distance_to_travel_diagonal;
std::chrono::duration<float, std::ratio<1>> time_to_travel_for_linearly;
std::chrono::duration<float, std::ratio<1>> time_to_travel_for_diagonally;

float linear_velocity;
float rotation_rate;
float turning_duration;
int timeStep;
bool should_rotate = true;
bool should_move = true;
bool rotation_first_iteration;
bool move_delay_thread_started;
int xyz = 0;
#pragma endregion


int main(int argc, char **argv)
{
    using namespace std::chrono;

#pragma region setup

    SetUp();

    /* Robot uses supervisor so we can obtain world info as well */
    robot = new Supervisor();

    /* Gets a reference to the floor/arena */
    Node* n = robot->getFromDef("Arena");
    
    /* Get map size in meters */
    int map_row_size = (n->getField("floorSize")->getSFVec2f())[0];
    int map_column_size = (n->getField("floorSize")->getSFVec2f())[1];

    /* Change representation from meters to tiles */
    map_row_size *= NUMBER_OF_TILES_PER_METER;
    map_column_size *= NUMBER_OF_TILES_PER_METER;

    /* Subdivide the tiles */
    map_row_size *= NUMBER_OF_DIVISIONS;
    map_column_size *= NUMBER_OF_DIVISIONS;

    /* Create q-table and movement arrays */
    Array2D<float> adjacency_matrix;
    adjacency_matrix.row_count = map_row_size * map_column_size;
    adjacency_matrix.column_count = 9;

    Array2D<float> q_table;
    q_table.row_count = map_row_size * map_column_size;
    q_table.column_count = 9;

    /* Instantiate 2D adjacency matrix and q-table */
    adjacency_matrix.array = new float*[adjacency_matrix.row_count];
    for (int i = 0; i < adjacency_matrix.row_count; i++)
        adjacency_matrix.array[i] = new float[adjacency_matrix.column_count];

    q_table.array = new float* [q_table.row_count];
    for (int i = 0; i < q_table.row_count; i++)
        q_table.array[i] = new float[q_table.column_count];

    /* Createse adjacency matrix */
    //CreateAdjacenyMatrix(&adjacency_matrix, map_row_size, map_column_size);

#pragma endregion

    Node* rob = robot->getFromDef("Khepera1");

    timeStep = 8;  

    /* Get both motors */
    left_motor = robot->getMotor("left wheel motor");
    right_motor = robot->getMotor("right wheel motor");

    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);
    left_motor->setVelocity(10.0);
    right_motor->setVelocity(-10.0);        
    left_motor->setVelocity(0.0);
    right_motor->setVelocity(0.0);    

    high_resolution_clock::time_point time_at_movement_initiation;
    high_resolution_clock::time_point time_now;
    duration<double> time_elapsed;

    bool is_moving = false;

    float start_time = robot->getTime();


    InertialUnit* imu = robot->getInertialUnit("inertial unit");
    imu->enable(timeStep);

    previous_direction = TopCenter;

    float robot_forward_angle;
    TurnDirection turn_direction;
    TurnDirection opposite_turn_direction;
    DirectionAngle turn_to;

    should_rotate = true;
    move_delay_thread_started = false;
    rotation_first_iteration = true;

    left_motor->setVelocity(10.0);
    right_motor->setVelocity(10.0);

    while (robot->step(timeStep) != -1)
    {
        //left_motor->setVelocity(10.0);
        //right_motor->setVelocity(10.0);
        //continue;

        std::cout << "Velocity L: " << left_motor->getVelocity() << std::endl;
        std::cout << "Velocity R: " << right_motor->getVelocity() << std::endl;
        /* Decide where to turn here */
        turn_to = TopLeft;

        switch (xyz)
        {
        case 1:
            turn_to = TopLeft;
            break;
        case 2:
            turn_to = TopRight;
            break;
        case 3:
            turn_to = MiddleRight;
            break;
        case 4:
            turn_to = MiddleLeft;
            break;
        case 5:
            turn_to = TopCenter;
            break;
        case 6:
            turn_to = BottomCenter;
            break;
        }

        if (should_rotate)
        {
            robot_forward_angle = RadianToDegree(imu->getRollPitchYaw()[2]);
            turn_direction = GetTurnDirection(robot_forward_angle, turn_to);

            if (rotation_first_iteration)
            {
                opposite_turn_direction = (TurnDirection)(turn_direction * -1);
                rotation_first_iteration = false;
            }

            RotateRobot(imu, turn_to, turn_direction, opposite_turn_direction);
            continue;
        }

        /* Should set variable should_rotate to true at the end of the move */
        if (should_move)
        {
            Move();

            should_move = false;
            /* Wait for x amount of time before setting should_rotate to true again */
        }
        else
        {
            if (!move_delay_thread_started)
            {
                std::thread(StopMove).detach();
                move_delay_thread_started = true;
            }
        }

    }

    CleanUp();
    return 0;
}

/* Converts radians to degrees, also converts the -180 to 180 range to 360 to 0 */
float RadianToDegree(double radian)
{
    /* Converts the angle from Radians to Degrees */
    float angle = (radian * 180) / M_PI;

    /* Converts the angle from the range -180 to 180, to 360 to 0 (Clockwise) */
    if (angle < 0.0f) angle = 360 - (angle * -1);

    /* Incase sensor is facing straight forward, we need to cover the edge case */
    if (angle < 0.0f) return 360.0f; 

    return angle;
}

/* Rotates the robot to face the direction provided */
void RotateRobot(InertialUnit* imu, DirectionAngle robot_forward_angle_delta, TurnDirection turn_direction, TurnDirection opposite_turn_direciton)
{
    /* Returns if the current facing direction is the direction to rotate towards */
    if (previous_direction == robot_forward_angle_delta)
    {
        should_rotate = false;
        return;
    }

    /* If the direction to move towards flips sides, it means that the robot has reached the threshold of
       or surpassed how much it should turn. This stops the rotation */
    if(turn_direction == opposite_turn_direciton)
    {
        left_motor->setVelocity(0.0f);
        right_motor->setVelocity(0.0f);

        previous_direction = robot_forward_angle_delta;
        should_rotate = false;
        return;
    }

    /* Begins rotating either clockwise or anti-clockwise */
    left_motor->setVelocity(turn_direction * MOTOR_DEFAULT_SPEED);
    right_motor->setVelocity(turn_direction * -MOTOR_DEFAULT_SPEED);

    //if (turn_direction == Clockwise) std::cout << "I should turn clockwise: " << std::endl;
    //else std::cout << "I should turn Anti-clockwise: " << std::endl;
}

/* Returns the optimal direction to rotate in to reach the target position */
TurnDirection GetTurnDirection(float angle_a, float angle_b)
{
    float angle_diff = angle_a - angle_b;

    if (angle_diff > 180) return Anti_Clockwise;
    else if (angle_diff < -180) return Clockwise;
    else if (angle_a > angle_b) return Clockwise;
    else if (angle_a < angle_b) return Anti_Clockwise;
}

void Move()
{
    left_motor->setVelocity(MOTOR_DEFAULT_SPEED);
    right_motor->setVelocity(MOTOR_DEFAULT_SPEED);
}

void StopMove()
{
    using namespace std::chrono;

    std::chrono::duration<float, std::ratio<1>> sleep_for;

    if (CheckIfMovingDiagonally())
        sleep_for = time_to_travel_for_diagonally;
    else
        sleep_for = time_to_travel_for_linearly;


    high_resolution_clock::time_point time_at_movement_initiation = high_resolution_clock::now();
    std::this_thread::sleep_for(sleep_for);
    high_resolution_clock::time_point time_now = high_resolution_clock::now();

    left_motor->setVelocity(0.0f);
    right_motor->setVelocity(0.0f);

    move_delay_thread_started = false;
    should_move = true;
    should_rotate = true;
    rotation_first_iteration = true;
    xyz++;

    duration<double> time_span = duration_cast<duration<double>>(time_now - time_at_movement_initiation);

    std::cout << "Done Moving.. I was supposed to move for " << time_span.count() << " seconds" << std::endl;
}

bool CheckIfMovingDiagonally()
{
    switch (previous_direction)
    {
    case TopLeft:
    case TopRight:
    case BottomLeft:
    case BottomRight:
        return true;
    default:
        return false;
    }
}

/* Currently unused; to be removed if it remains so */
void CreateAdjacenyMatrix(Array2D<float>* table, int map_row_count, int map_column_count)
{
    float** arr = table->array;
    int row_count = table->row_count;
    int column_count = table->column_count;

    /* 
        Below is the representation of the possible moves for the robot,
        with 1 being possible, 0 being where the robot is, which is
        not possible:

        1 1 1
        1 0 1
        1 1 1
    */

    int from_state_number = 0;
    int action_number = 0;
    for (int i = 0; i < map_row_count; i++)
    {
        for (int j = 0; j < map_column_count; j++)
        {
            /* Check top-left */
            if ((i - 1) != -1 && (j - 1) != -1)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number++;

            /* Check top-center */
            if ((i - 1) != -1)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number++;

            /* Check top-right */
            if ((i - 1) != -1 && (j + 1) != map_column_count)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number++;

            /* Check middle-left */
            if ((j - 1) != -1)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number++;

            /* Set middle-center */
            arr[from_state_number][action_number] = -1;

            action_number++;

            /* Check middle-right */
            if ((j + 1) != map_column_count)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number++;

            /* Check bottom-left */
            if ((i + 1) != map_row_count && (j - 1) != -1)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number++;

            /* Check bottom-center */
            if ((i + 1) != map_row_count)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number++;

            /* Check bottom-right */
            if ((i + 1) != map_row_count && (j + 1) != map_column_count)
                arr[from_state_number][action_number] = 1;
            else
                arr[from_state_number][action_number] = -1;

            action_number = 0;
            from_state_number++;
        }
    }

    for (int i = 0; i < row_count; i++)
    {
        for (int j = 0; j < column_count; j++)
        {
            std::cout << arr[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void SetUp()
{
    //distance_to_travel_linear = (SIZE_OF_TILE/NUMBER_OF_DIVISIONS)/2;
    distance_to_travel_linear = 0.5f;
    distance_to_travel_diagonal = sqrt(distance_to_travel_linear * distance_to_travel_linear * 2);
    time_to_travel_for_linearly = std::chrono::duration<float, std::ratio<1>>(distance_to_travel_linear / MOTOR_DEFAULT_SPEED);
    time_to_travel_for_diagonally = std::chrono::duration<float, std::ratio<1>>(distance_to_travel_diagonal / MOTOR_DEFAULT_SPEED);

    std::cout << "TIME TO MOVE LINEARLY " << distance_to_travel_linear / MOTOR_DEFAULT_SPEED << " FOR " << distance_to_travel_linear << " METERS" << std::endl;
}

void CleanUp()
{
    delete robot;
    delete left_motor;
    delete right_motor;
    delete left_sensor;
    delete right_sensor;
}

