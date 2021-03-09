// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
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
#define M_PI 3.14159265358979323846

using namespace webots;


template<typename t>
struct Array2D
{
    t** array;
    int row_count;
    int column_count;
};

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
    TopMiddle = 360,
};

enum TurnDirection
{
    Clockwise = 1,
    Anti_Clockwise = -1,
};

void CreateAdjacenyMatrix(Array2D<float>*, int, int);
void rotateRobot(InertialUnit*, DirectionAngle, TurnDirection, TurnDirection);
TurnDirection GetTurnDirection(float, float);
float RadianToDegree(double);
void Move();
void SetUp();
void CleanUp();

const int NUMBER_OF_TILES_PER_METER = 2;
const float SIZE_OF_TILE = 0.5f;
const int NUMBER_OF_DIVISIONS = 2;
const float MOTOR_DEFAULT_SPEED = 10.0f;
const float WHEEL_RADIUS = 0.028f;
const float DISTANCE_BETWEEN_WHEELS = 0.052f;

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
float time_to_travel_for_linearly;
float time_to_travel_for_diagonally;

float linear_velocity;
float rotation_rate;
float turning_duration;
int timeStep;
bool should_rotate = true;
bool should_move = true;
bool rotation_first_iteration;
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

    previous_direction = TopMiddle;

    float robot_forward_angle;
    TurnDirection turn_direction;
    TurnDirection opposite_turn_direction;
    DirectionAngle turn_to;
    should_rotate = true;
    
    rotation_first_iteration = true;
    while (robot->step(timeStep) != -1)
    {
        /* Decide where to turn here */
        turn_to = TopLeft;

        if (should_rotate)
        {
            robot_forward_angle = RadianToDegree(imu->getRollPitchYaw()[2]);
            turn_direction = GetTurnDirection(robot_forward_angle, turn_to);

            if (rotation_first_iteration)
            {
                opposite_turn_direction = (TurnDirection)(turn_direction * -1);
                rotation_first_iteration = false;
            }

            rotateRobot(imu, turn_to, turn_direction, opposite_turn_direction);
            continue;
        }

        /* Should set variable should_rotate to true at the end of the move */
        if (should_move)
        {

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
void rotateRobot(InertialUnit* imu, DirectionAngle robot_forward_angle_delta, TurnDirection turn_direction, TurnDirection opposite_turn_direciton)
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

    if (turn_direction == Clockwise) std::cout << "I should turn clockwise: " << std::endl;
    else std::cout << "I should turn Anti-clockwise: " << std::endl;
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
    distance_to_travel_linear = (SIZE_OF_TILE/NUMBER_OF_DIVISIONS)/2;
    distance_to_travel_diagonal = sqrt(distance_to_travel_linear * distance_to_travel_linear * 2);
    time_to_travel_for_linearly = distance_to_travel_linear / MOTOR_DEFAULT_SPEED;
    time_to_travel_for_diagonally = distance_to_travel_diagonal / MOTOR_DEFAULT_SPEED;
}

void CleanUp()
{
    delete robot;
    delete left_motor;
    delete right_motor;
    delete left_sensor;
    delete right_sensor;
}

