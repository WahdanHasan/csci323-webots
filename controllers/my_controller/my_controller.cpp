// File:          my_controller.cpp
// Date:
// Description: Q-learning controller for Khepera1
// Author: Wahdan Hasan, Rama Al Sbeinaty
// Modifications: idk what this means lol but we made it

#pragma region include statements
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Lidar.hpp>
#include <webots/GPS.hpp>
#include <iostream>
#include <time.h>
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

#pragma region Enums
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

#pragma region Function Prototypes
void CreateAdjacenyMatrix(Array2D<float>*, int, int);
void RotateRobot(InertialUnit*, DirectionAngle, TurnDirection, TurnDirection);
const double* GetGpsCoordinateToWatch(DirectionAngle facing_direction);
bool CheckIfReachedTarget(bool, const double*, double);
TurnDirection GetTurnDirection(float, float);
float RadianToDegree(double);
bool CheckIfMovingDiagonally();
bool CheckIfMovementIsPositiveVE(DirectionAngle);
void UpdatePosition(DirectionAngle direction, int* position);
void Move();
void SetUp();
void CleanUp();
void StopMove();
void ResetDecisionVariables();
DirectionAngle ChooseRandomMove();
DirectionAngle ExploitEnvironment(Array2D<float>*, int* position);
#pragma endregion

/* Maybe change these to defines instead, more performant */
#pragma region Global Constants
const int NUMBER_OF_TILES_PER_METER = 2;
const float SIZE_OF_TILE = 0.5f;
const int NUMBER_OF_SUBDIVISIONS = 5;
const double MOTOR_DEFAULT_SPEED = 10.0;
const double MOTOR_STOP_SPEED = 0.0;
const float WHEEL_RADIUS = 0.028f;
const float DISTANCE_BETWEEN_WHEELS = 0.052f;
const float BATTERY_LOST_PER_MOVE = 0.05f;
const int AMOUNT_OF_MOVES = 9;
const double DISTANCE_SENSOR_THRESHOLD = 1000.0;
const double MINIMUM_BATTERY_LEVEL = 0.0;
const double BATTERY_START_LEVEL = 100.00;
#pragma endregion

#pragma region Global Variable Declarations
Supervisor* robot;
GPS* gps;
Motor* left_motor;
Motor* right_motor;

PositionSensor* left_sensor;
PositionSensor* right_sensor;
Lidar* lidar;
DirectionAngle previous_direction;

double distance_to_travel_linear;

int timeStep;
bool should_rotate;
bool should_move;
bool rotation_first_iteration;
int xyz;
#pragma endregion


int main(int argc, char **argv)
{
 #pragma region Local Variable Declarations
    TurnDirection turn_direction;
    TurnDirection opposite_turn_direction;
    DirectionAngle turn_to;

    InertialUnit* imu;

    float robot_forward_angle;
    bool should_stop_moving;
    bool stop_move_first_iteration;
    bool is_movement_positive_ve;
    const double* gps_axis_to_watch = NULL;
    double target_distance;
    double exploration_rate;
    int current_position[2];
    int new_position[2];
    double laser_sensor_distance;
    float battery;
    int current_episode;
    int current_step_count;
    const int laser_sensor_count = 1;
#pragma endregion


#pragma region Setup

    SetUp();

    /* Robot uses supervisor so we can obtain world info as well */
    robot = new Supervisor();

    /* Gets a reference to the floor/arena */
    Node* n = robot->getFromDef("Arena");
    
    /* Get map size in meters */
    double map_row_size = (n->getField("floorSize")->getSFVec2f())[0];
    double map_column_size = (n->getField("floorSize")->getSFVec2f())[1];

    /* Change representation from meters to tiles */
    map_row_size *= NUMBER_OF_TILES_PER_METER;
    map_column_size *= NUMBER_OF_TILES_PER_METER;

    /* Subdivide the tiles */
    map_row_size *= NUMBER_OF_SUBDIVISIONS;
    map_column_size *= NUMBER_OF_SUBDIVISIONS;

    /* Create q-table */
    Array2D<float> q_table;
    q_table.row_count = map_row_size * map_column_size;
    q_table.column_count = 8;

    /* Instantiate q-table */
    q_table.array = new float* [q_table.row_count];
    for (int i = 0; i < q_table.row_count; i++)
        q_table.array[i] = new float[q_table.column_count];

    for (int i = 0; i < q_table.row_count; i++)
        for (int j = 0; j < q_table.column_count; j++)
            q_table.array[i][j] = BATTERY_LOST_PER_MOVE;

    //for (int i = 0; i < q_table.row_count; i++)
    //{
    //    for (int j = 0; j < q_table.column_count; j++)
    //    {
    //        std::cout << q_table.array[i][j] << " ";
    //    }
    //    std::cout << std::endl;
    //}

    //return 0;

    timeStep = robot->getBasicTimeStep();  
#pragma endregion


#pragma region Pre-Loop Things
    /* Get both motors */
    left_motor = robot->getMotor("left wheel motor");
    right_motor = robot->getMotor("right wheel motor");

    /* Reseting values on start */
    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);      
    left_motor->setVelocity(MOTOR_STOP_SPEED);
    right_motor->setVelocity(MOTOR_STOP_SPEED);

    /* Get and enable the GPS and Inertial Unit */
    imu = robot->getInertialUnit("inertial unit");
    gps = robot->getGPS("gps");
    //lidar = robot->getLidar("lidar");
    imu->enable(timeStep);
    gps->enable(timeStep);
    //lidar->enable(timeStep);

    DistanceSensor** laser_sensors = new DistanceSensor*[laser_sensor_count];
    laser_sensors[0] = robot->getDistanceSensor("distance sensor1");
    //laser_sensors[1] = robot->getDistanceSensor("distance sensor2");
    //laser_sensors[2] = robot->getDistanceSensor("distance sensor3");
    //laser_sensors[3] = robot->getDistanceSensor("distance sensor4");
    //laser_sensors[4] = robot->getDistanceSensor("distance sensor5");
    laser_sensors[0]->enable(timeStep);
    //laser_sensors[1]->enable(timeStep);
    //laser_sensors[2]->enable(timeStep);
    //laser_sensors[3]->enable(timeStep);
    //laser_sensors[4]->enable(timeStep);

    /*rewards table*/
    const int number_of_episodes = 1000;
    int rewards_all_episodes[number_of_episodes];
    int max_number_of_steps = 1000;

    battery = BATTERY_START_LEVEL;
    //int current_reward = 0;

    //int number_of_iter = 0;

    current_episode = 0;

    /* Variable initial values */
    previous_direction = TopCenter;
    xyz = 0;

    should_rotate = true;
    rotation_first_iteration = true;
    should_rotate = true;
    should_move = true;

    current_position[0] = 0;
    current_position[1] = 0;

    exploration_rate = 100.0;

    turn_to = TopCenter;
    current_step_count = 0;
#pragma endregion


    //ds2, ds3
    //return 0;
    srand(time(0));
    int eee = 0;
    int ppp = 0;

    while ((robot->step(timeStep) != -1) && (current_episode < number_of_episodes)) //step(timestep) is a simulation step and doesnt correspond to seconds in real-time.
    {
        eee++;
        //std::cout << laser_sensor->getValue() << std::endl;
        new_position[0] = current_position[0];
        new_position[1] = current_position[1];

        /* Decide where to turn here */
        if (rotation_first_iteration)
        {
            int explore_probability = rand() % 1;
            //if (explore_probability >= exploration_rate)
                turn_to = ChooseRandomMove();
            //else
                // turn_to = ExploitEnvironment(state);

            UpdatePosition(turn_to, new_position);
        }

        if (should_rotate)
        {
            robot_forward_angle = RadianToDegree(imu->getRollPitchYaw()[2]);
            turn_direction = GetTurnDirection(robot_forward_angle, turn_to);
            if (rotation_first_iteration)
            {
                //std::cout << "Rotating.." << std::endl;
                opposite_turn_direction = (TurnDirection)(turn_direction * -1);
                rotation_first_iteration = false;
            }

            RotateRobot(imu, turn_to, turn_direction, opposite_turn_direction);
            continue;
        }

        /* Q-Learning Algorithm */
        
        if (should_move) 
        {
            laser_sensor_distance = 0;
            for(int k = 0 ; k < laser_sensor_count; k++)
                laser_sensor_distance += laser_sensors[k]->getValue();
            //std::cout << "DISTANCE: " << laser_sensor_distance << " " << (DISTANCE_SENSOR_THRESHOLD * laser_sensor_count) << std::endl;
            if (laser_sensor_distance != DISTANCE_SENSOR_THRESHOLD * laser_sensor_count)
            {
                std::cout << "WHOOPS WALLLLL " << laser_sensor_distance << std::endl;
                q_table.array[new_position[0]][new_position[1]] = -50.00f;
                current_step_count++;
                ResetDecisionVariables();
                continue;
            }
        }

        //ds2_distance = 0;
        //for (int k = 0; k < 4; k++)
        //{
        //    ds2_distance += ds_sensors[k]->getValue();
        //}
        //std::cout << "DISTANCEEE: " << ds2_distance << std::endl;
        /* Should set variable should_rotate to true at the end of the move */
        if (should_move)
        {
            Move();

            should_move = false;

            stop_move_first_iteration = true;
        }
        else
        {

            if (stop_move_first_iteration)
            {
                gps_axis_to_watch = GetGpsCoordinateToWatch(turn_to);
                is_movement_positive_ve = CheckIfMovementIsPositiveVE(turn_to);

                if (is_movement_positive_ve) target_distance = *gps_axis_to_watch + distance_to_travel_linear;
                else                         target_distance = *gps_axis_to_watch - distance_to_travel_linear;
                //if (is_movement_positive_ve) target_distance = ((*gps_axis_to_watch + distance_to_travel_linear)/ distance_to_travel_linear)*distance_to_travel_linear;
                //else                         target_distance = ((*gps_axis_to_watch - distance_to_travel_linear) / distance_to_travel_linear) * distance_to_travel_linear;
               
                //if (is_movement_positive_ve) target_distance = floorf((*gps_axis_to_watch + distance_to_travel_linear) * 100000)/100000;
                //else                         target_distance = floorf((*gps_axis_to_watch - distance_to_travel_linear) * 100000)/100000;
                stop_move_first_iteration = false;
                ppp++;
            }
            //std::cout << "hi";
            //if ((std::fmod(target_distance, 0.01) > 0.008 && std::fmod(target_distance, 0.01) <= 0.01) || (std::fmod(target_distance, 0.01) < 0.002 && std::fmod(target_distance, 0.01) >= 0.000999))
            //    std::cout << "TOO INACCURATE " << target_distance << " " << std::fmod(target_distance, 0.01) << std::endl;
            //std::cout << "Distance: " << ds2->getValue() << std::endl;
            if(eee%500 == 0) 
            std::cout << "Target: " << target_distance << ", Current: " << *gps_axis_to_watch << " " << ppp << std::endl;

            should_stop_moving = CheckIfReachedTarget(is_movement_positive_ve, gps_axis_to_watch, target_distance);

            if (should_stop_moving)
            {
                StopMove();
                battery -= BATTERY_LOST_PER_MOVE;
                //std::cout << battery << std::endl;
                ResetDecisionVariables();
            }

        }

        if (current_step_count == max_number_of_steps || battery == MINIMUM_BATTERY_LEVEL)
        {
            current_step_count = 0;
            battery = BATTERY_START_LEVEL;
            //ResetEnvironment; /* Move to next episode */
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
        //std::cout << "Done rotating..." << std::endl;
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

bool CheckIfReachedTarget(bool is_movement_positive_ve, const double* gps_axis_to_watch, double target_distance)
{
    if (is_movement_positive_ve)
    {
        if (*gps_axis_to_watch >= target_distance) return true;
        else return false;
    }
    else
    {
        if (*gps_axis_to_watch <= target_distance) return true;
        else return false;
    }
}

bool CheckIfMovementIsPositiveVE(DirectionAngle direction)
{
    switch (direction)
    {
    case TopRight:
    case MiddleRight:
    case BottomRight:
    case BottomCenter:
        return true;
    default:
        return false;
    }
}

void ResetDecisionVariables()
{
    should_move = true;
    should_rotate = true;
    rotation_first_iteration = true;
    xyz++;
}

void Move()
{
    left_motor->setVelocity(MOTOR_DEFAULT_SPEED);
    right_motor->setVelocity(MOTOR_DEFAULT_SPEED);
}

void StopMove()
{
    left_motor->setVelocity(MOTOR_STOP_SPEED);
    right_motor->setVelocity(MOTOR_STOP_SPEED);
}

const double* GetGpsCoordinateToWatch(DirectionAngle facing_direction)
{
    /* While diagonally/linearly, we can simply check the x-axis to determine if
       we've reached our destination. There exist 2 exceptions (TopCenter/BottomCenter)
       which need to be watched on the z-axis instead.
    */
    switch (facing_direction)
    {
    case TopCenter:
    case BottomCenter:
        return &(gps->getValues()[2]);
    default:
        return &(gps->getValues()[0]);
    }
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

DirectionAngle ChooseRandomMove()
{
    int move_index = rand() % AMOUNT_OF_MOVES;
    
    //std::cout << "Chose: " << move_index << std::endl;

    switch (move_index)
    {
    case 1:
        return TopLeft;
    case 2:
        return TopCenter;
    case 3:
        return TopRight;
    case 4:
        return MiddleLeft;
    case 5:
        return MiddleRight;
    case 6:
        return BottomLeft;
    case 7:
        return BottomCenter;
    case 8:
        return BottomRight;
    default:
        return TopCenter;
    }
}

DirectionAngle ExploitEnvironment(Array2D<float>* q_table, int* position)
{
    int best_index = 0;
    for (int i = 0; i < q_table->column_count; i++)
    {
        if (q_table->array[position[0]][i] > q_table->array[position[0]][best_index]) best_index = i;
    }


    switch (best_index)
    {
    case 1:
        return TopLeft;
    case 2:
        return TopCenter;
    case 3:
        return TopRight;
    case 4:
        return MiddleLeft;
    case 5:
        return MiddleRight;
    case 6:
        return BottomLeft;
    case 7:
        return BottomCenter;
    case 8:
        return BottomRight;
    default:
        return TopCenter;
    }
}

void UpdatePosition(DirectionAngle direction, int* position)
{
    //std::cout << "DirectionAngle: " << direction << std::endl;

    switch (direction)
    {
    case TopLeft:
        position[0] -= 1;
        position[1] -= 1;
        break;
    case TopCenter:
        position[0] -= 1;
        break;
    case TopRight:
        position[0] -= 1;
        position[1] += 1;
        break;
    case MiddleLeft:
        position[1] -= 1;
        break;
    case MiddleRight:
        position[1] += 1;
        break;
    case BottomLeft:
        position[0] += 1;
        position[1] -= 1;
        break;
    case BottomCenter:
        position[0] += 1;
        break;
    case BottomRight:
        position[0] += 1;
        position[1] += 1;
        break;
    }
}

void SetUp()
{
    distance_to_travel_linear = SIZE_OF_TILE/NUMBER_OF_SUBDIVISIONS;
}

void CleanUp()
{
    delete robot;
    delete left_motor;
    delete right_motor;
    delete left_sensor;
    delete right_sensor;
}

