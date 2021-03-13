// File:          my_controller.cpp
// Date:
// Description: Q-learning controller for Khepera1
// Author: Wahdan Hasan, Rama Al Sbeinaty
// Modifications: idk what this means lol but we made it

#pragma region include statements
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/InertialUnit.hpp>
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
    TopCenter = 0,
    INVALID,
};
//enum DirectionAngle
//{
//    TopRight = 135,
//    MiddleRight = 90,
//    BottomRight = 45,
//    BottomCenter = 360,
//    BottomLeft = 315,
//    MiddleLeft = 270,
//    TopLeft = 225,
//    TopCenter = 180,
//};

enum TurnDirection
{
    Clockwise = 1,
    Anti_Clockwise = -1,
};
#pragma endregion

#pragma region Function Prototypes
void CreateAdjacenyMatrix(Array2D<float>*, int, int);
void RotateRobot(InertialUnit*, DirectionAngle, TurnDirection, TurnDirection);
const double* GetGpsCoordinateToWatch(DirectionAngle);
bool CheckIfReachedTarget(bool, const double*, double);
TurnDirection GetTurnDirection(float, float);
float RadianTo360Degree(double);
bool CheckIfMovementIsPositiveVE(DirectionAngle);
void UpdatePosition(DirectionAngle, int*);
void Move();
void SetUp();
void CleanUp();
void StopMove();
bool IsFacingObstacle(DistanceSensor**, int laser_sensor_count);
void SetGoal(DirectionAngle, double*, double*, double*);
DirectionAngle DecideMove(double, Array2D<float>*, int*, int*);
void ResetDecisionVariables();
DirectionAngle ChooseRandomMove();
DirectionAngle ExploitEnvironment(Array2D<float>*, int*);
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
const double EXPLORATION_RATE_DECAY_RATE = 0.01;
#pragma endregion

#pragma region Global Variable Declarations
Supervisor* robot;
GPS* gps;
Motor* left_motor;
Motor* right_motor;

DirectionAngle previous_direction;

double distance_to_travel_linear;

int timeStep;
bool should_rotate;
bool should_move;
bool stop_move_first_iteration;
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
    bool is_movement_positive_ve;
    const double* robot_gps_axis_to_watch = NULL;
    double target_distance;
    double exploration_rate;
    int current_position[2];
    int new_position[2];
    double laser_sensor_distance;
    float battery;
    int current_episode;
    int current_step_count;
    const int laser_sensor_count = 3;
    double explore_probability;
    int axis_to_watch;
    double s1;
    double s2;
    double n1;
    double n2;
    double x_coordinate_new;
    double z_coordinate_new;
    double x_coordinate_old;
    double z_coordinate_old;
#pragma endregion


#pragma region Setup

    SetUp();

    /* Robot uses supervisor so we can obtain world info as well */
    robot = new Supervisor();

    /* Gets a reference to the floor/arena */
    Node* n = robot->getFromDef("Arena");
    
    /* Get map size in meters */
    double arena_row_size = (n->getField("floorSize")->getSFVec2f())[0];
    double arena_column_size = (n->getField("floorSize")->getSFVec2f())[1];

    double tile_row_count = NUMBER_OF_TILES_PER_METER * arena_row_size;
    double tile_column_count = NUMBER_OF_TILES_PER_METER * arena_column_size;

    double map_row_size;
    double map_column_size;

    /* Initialize map row and column size */
    map_row_size = (NUMBER_OF_SUBDIVISIONS * tile_row_count) - (tile_row_count - 1);
    map_column_size = (NUMBER_OF_SUBDIVISIONS * tile_column_count) - (tile_column_count - 1);

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
            q_table.array[i][j] = -BATTERY_LOST_PER_MOVE;

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
    imu->enable(timeStep);
    gps->enable(timeStep);

    DistanceSensor** laser_sensors = new DistanceSensor*[laser_sensor_count];
    laser_sensors[0] = robot->getDistanceSensor("distance sensor1");
    laser_sensors[1] = robot->getDistanceSensor("distance sensor2");
    laser_sensors[2] = robot->getDistanceSensor("distance sensor3");

    laser_sensors[0]->enable(timeStep);
    laser_sensors[1]->enable(timeStep);
    laser_sensors[2]->enable(timeStep);


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
    stop_move_first_iteration = true;

    current_position[0] = (int) map_row_size / 2;
    current_position[1] = (int) map_column_size / 2;

    exploration_rate = 100.0;

    turn_to = TopCenter;
    current_step_count = 0;


    srand(time(0));

    std::cout << "Map Size: " << map_row_size << ", " << map_column_size << std::endl;
    std::cout << "Rover Starting: " << current_position[0] << ", " << current_position[1] << std::endl;

    x_coordinate_new = 0.0;
    z_coordinate_new = 0.0;
    x_coordinate_old = 0.0;
    z_coordinate_old = 0.0;

#pragma endregion




    while ((robot->step(timeStep) != -1) && (current_episode < number_of_episodes)) //step(timestep) is a simulation step and doesnt correspond to seconds in real-time.
    {

#pragma region Rotation
        /* Decide where to rotate here based on exploration and exploitation */
        if (rotation_first_iteration)
        {
            x_coordinate_new = x_coordinate_old;
            z_coordinate_new = z_coordinate_old;

            new_position[0] = current_position[0];
            new_position[1] = current_position[1];

            turn_to = DecideMove(exploration_rate, &q_table, current_position, new_position);

            if (turn_to == INVALID)
            {
                ResetDecisionVariables();
                continue;
            }
        }

        /* Begin rotation here */
        if (should_rotate)
        {
            robot_gps_axis_to_watch = GetGpsCoordinateToWatch(turn_to);

            robot_forward_angle = RadianTo360Degree(imu->getRollPitchYaw()[2]);

            if (stop_move_first_iteration)
            {
                //if (robot_forward_angle <= 0.001)
                //{
                //    std::cout << " SSSSSSSSSSSSSSSHEIT " << std::endl;
                //    ResetDecisionVariables();
                //    continue;
                //}

                SetGoal(turn_to, &x_coordinate_new, &z_coordinate_new, &target_distance);
                
                stop_move_first_iteration = false;
                          
                if (rotation_first_iteration)
                {
                    s1 = gps->getValues()[0] * -1;
                    s2 = gps->getValues()[2] * -1;

                    std::cout << "Coords: " << "(" << x_coordinate_old << ", " << z_coordinate_old << ") (" << gps->getValues()[0] << ", " << gps->getValues()[2] << ")" << std::endl;

                    n1 = x_coordinate_new;
                    n2 = z_coordinate_new;

                    n1 += s1;
                    n2 += s2;
                }



                std::string s;
                switch (turn_to)
                {
                case TopLeft:
                    s = "TOP LEFT";
                    break;
                case TopCenter:
                    s = "TOP CENTER";
                    break;
                case TopRight:
                    s = "TOP RIGHT";
                    break;
                case MiddleLeft:
                    s = "MIDDLE LEFT";
                    break;
                case MiddleRight:
                    s = "MIDDLE RIGHT";
                    break;
                case BottomLeft:
                    s = "BOTTOM LEFT";
                    break;
                case BottomCenter:
                    s = "BOTTOM CENTER";
                    break;
                case BottomRight:
                    s = "BOTTOM RIGHT";
                    break;
                }
                std::cout << "(" << x_coordinate_new << ", " << z_coordinate_new << ") (" << n1 << ", " << n2 << "): " << RadianTo360Degree(((std::atan2(n2, n1) - 1.5708) * -1)) << " " << robot_forward_angle << " " << s  << " " << target_distance << std::endl;

            }

            turn_direction = GetTurnDirection(robot_forward_angle, RadianTo360Degree(((std::atan2(n2, n1) - 1.5708) * -1)));
            if (rotation_first_iteration)
            {
                opposite_turn_direction = (TurnDirection)(turn_direction * -1);
                rotation_first_iteration = false;
            }

            RotateRobot(imu, turn_to, turn_direction, opposite_turn_direction);
            continue;
        }
#pragma endregion


#pragma region Check Move Decision


        /*                                        Q-Learning Algorithm                                      */
        
        if (should_move) 
        {
            bool is_facing_wall = IsFacingObstacle(laser_sensors, laser_sensor_count);

            if (is_facing_wall)
            {
                //UpdateNewPositionScore(q_table.array, new_position);

                //q_table.array[new_position[0]][new_position[1]] = -50.00f;

                std::cout << "WHOOPS WALL " << std::endl;
                current_step_count++;

                ResetDecisionVariables();
                continue;
            }

        }

#pragma endregion


#pragma region Movement
        /*                                                  MOVEMENT                                                                */    

        /* Should set variable should_rotate to true at the end of the move */
        if (should_move)
        {
            Move();
            is_movement_positive_ve = CheckIfMovementIsPositiveVE(turn_to);

            should_move = false;

        }
        else
        {

            // std::cout << is_movement_positive_ve << " " << "Target: " << target_distance << ", Current: " << *robot_gps_axis_to_watch << std::endl;

            
            should_stop_moving = CheckIfReachedTarget(is_movement_positive_ve, robot_gps_axis_to_watch, target_distance);

            if (should_stop_moving)
            {
                current_position[0] = new_position[0];
                current_position[1] = new_position[1];
                x_coordinate_old = x_coordinate_new;
                z_coordinate_old = z_coordinate_new;
                StopMove();
                battery -= BATTERY_LOST_PER_MOVE;
                ResetDecisionVariables();
            }

        }
#pragma endregion


        /* Move to next episode if max steps reached or battery runs out */
        if (current_step_count == max_number_of_steps || battery == MINIMUM_BATTERY_LEVEL)
        {
            current_step_count = 0;
            battery = BATTERY_START_LEVEL;
            
            ResetDecisionVariables();

            //ResetEnvironment; /* Move to next episode */
        }
    }


    CleanUp();
    return 0;
}

bool IsFacingObstacle(DistanceSensor** laser_sensors, int laser_sensor_count)
{
    double laser_sensor_distance = 0;
    for(int k = 0 ; k < laser_sensor_count; k++)
        laser_sensor_distance += laser_sensors[k]->getValue();

    if (laser_sensor_distance != DISTANCE_SENSOR_THRESHOLD * laser_sensor_count) return true;

    return false;
}

void SetGoal(DirectionAngle turn_to, double* x_coordinate_new, double* z_coordinate_new, double* target_distance)
{
    switch (turn_to)
    {
    case TopLeft:
        *x_coordinate_new -= distance_to_travel_linear;
        *z_coordinate_new -= distance_to_travel_linear;
        *target_distance = *x_coordinate_new;
        break;
    case TopCenter:
        *z_coordinate_new -= distance_to_travel_linear;
        *target_distance = *z_coordinate_new;
        break;
    case TopRight:
        *x_coordinate_new += distance_to_travel_linear;
        *z_coordinate_new -= distance_to_travel_linear;
        *target_distance = *x_coordinate_new;
        break;
    case MiddleLeft:
        *x_coordinate_new -= distance_to_travel_linear;
        *target_distance = *x_coordinate_new;
        break;
    case MiddleRight:
        *x_coordinate_new += distance_to_travel_linear;
        *target_distance = *x_coordinate_new;
        break;
    case BottomLeft:
        *x_coordinate_new -= distance_to_travel_linear;
        *z_coordinate_new += distance_to_travel_linear;
        *target_distance = *x_coordinate_new;
        break;
    case BottomCenter:
        *z_coordinate_new += distance_to_travel_linear;
        *target_distance = *z_coordinate_new;
        break;
    case BottomRight:
        *x_coordinate_new += distance_to_travel_linear;
        *z_coordinate_new += distance_to_travel_linear;
        *target_distance = *x_coordinate_new;
        break;
    }
}

DirectionAngle DecideMove(double exploration_rate, Array2D<float>*q_table, int* current_position, int* new_position)
{
    DirectionAngle turn_to;
    int explore_probability = rand() % 100;

    if (explore_probability <= exploration_rate)
        turn_to = ChooseRandomMove();
    else
        turn_to = ExploitEnvironment(q_table, current_position);

    //turn_to = TopCenter;

    UpdatePosition(turn_to, new_position);

    return turn_to;
}

/* Converts radians to degrees, also converts the -180 to 180 range to 360 to 0 */
float RadianTo360Degree(double radian)
{
    /* Converts the angle from Radians to Degrees */
    float angle = (radian * 180) / M_PI;

    /* Converts the angle from the range -180 to 180, to 360 to 0 (Clockwise) */
    if (angle < 0.0f) angle = 360 - (angle * -1);

    /* Incase sensor is facing straight forward, we need to cover the edge case */
    //if (angle < 0.0f) return 360.0f; 

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

    //if (angle_diff > 180) return Anti_Clockwise;
    //else if (angle_diff < -180) return Clockwise;
    //else if (angle_a > angle_b) return Clockwise;
    //else if (angle_a < angle_b) return Anti_Clockwise;

    if (angle_diff > 180) return Clockwise;
    else if (angle_diff < -180) return Anti_Clockwise;
    else if (angle_a > angle_b) return Anti_Clockwise;
    else if (angle_a < angle_b) return Clockwise;
}

bool CheckIfReachedTarget(bool is_movement_positive_ve, const double* robot_gps_axis_to_watch, double target_distance)
{
    if (is_movement_positive_ve)
    {
        if (*robot_gps_axis_to_watch >= target_distance) return true;
        else                                             return false;
    }
    else
    {
        if (*robot_gps_axis_to_watch <= target_distance) return true;
        else                                             return false;
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
    stop_move_first_iteration = true;
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
    // (row_size * i) + j
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
        return INVALID;
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
    }
}

void UpdatePosition(DirectionAngle direction, int* position)
{
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
}

