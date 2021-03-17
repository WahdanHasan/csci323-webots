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
#include <thread>
#include <chrono>
#include <fstream>
#pragma endregion

#define M_PI 3.14159265358979323846

using namespace webots;

#pragma region Classes and Structs
template<typename t>
struct Array2D
{
    t** array;
    int row_count;
    int column_count;
};
#pragma endregion

#pragma region Enums

enum DirectionAngle
{
    TopLeft,
    TopCenter,
    TopRight,
    MiddleLeft,
    MiddleRight,
    BottomLeft,
    BottomCenter,
    BottomRight,
};



enum TurnDirection
{
    Clockwise = 1,
    Anti_Clockwise = -1,
};
#pragma endregion

#pragma region Function Prototypes
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
void SetGoalPosition(DirectionAngle, double*, double*, double*);
DirectionAngle DecideMove(double, Array2D<double>*, int*, int*);
void ResetDecisionVariables();
DirectionAngle ChooseRandomMove();
DirectionAngle GetStateBestDirectionAngle(Array2D<double>*, int*);
void UpdateNewPositionScore(Array2D<double>*, DirectionAngle, int*, int*, double*, double);
int GetDirectionIndex(DirectionAngle);
void ResetCoins();
void ResetEnvironment();
bool CheckForCoin(double, double);
void HideCoin(double, double);
bool LoadState(Array2D<double>*);
void SaveState(Array2D<double>*);
bool IsCoinPickedUp(double, double);
#pragma endregion

#pragma region Global Constants
const int NUMBER_OF_TILES_PER_METER = 2;
const int NUMBER_OF_SUBDIVISIONS = 5;
const double SIZE_OF_TILE = 0.5;
const double MOTOR_DEFAULT_SPEED = 10.0;
const double MOTOR_STOP_SPEED = 0.0;
const double DISTANCE_SENSOR_THRESHOLD = 1000.0;
const double COIN_TRANSPARENCY_DEFAULT = 0.0;
const double COIN_TRANSPARENCY_PICKED = 1.0;
const double ROBOT_DEFAULT_POSITION[] = { 0.0, 0.0, 0.0 };
const double ROBOT_DEFAULT_ROTATION[] = { 0.0, 1.0, 0.0, 0.0 };
const double INDICATOR_EXPLORING_COLOR[] = { 0.0, 1.0, 0.0 };
const double INDICATOR_EXPLOITING_COLOR[] = { 1.0, 0.0, 0.0 };
#pragma endregion

#pragma region Q-Learning Constants
const int AMOUNT_OF_MOVES = 8;
const double BOARD_BASE_SCORE = 0.0;
const double MINIMUM_BATTERY_LEVEL = 0.0;
const double BATTERY_START_LEVEL = 100.00;
const double BATTERY_LOST_PER_MOVE = 1.0;
const double REWARD_OBSTRUCTION = -100.00;
const double REWARD_EMPTY = 0.0;
const double REWARD_COIN = 100;
const double LEARNING_RATE = 0.1;
const double DISCOUNT_RATE = 0.99;
const double MINIMUM_EXPLORATION_RATE = 1.0;
const double MAX_EXPLORATION_RATE = 100.0;
const double EXPLORATION_DECAY_RATE = -0.001;
const double MAX_SCORE = REWARD_COIN;
#pragma endregion

#pragma region Global Variable Declarations
Supervisor* environment;
GPS* gps;
Motor* left_motor;
Motor* right_motor;
Node** coins;
Field* indicator;
DirectionAngle previous_direction;

double distance_to_travel_linear;

int amount_of_coin;
int timeStep;
int amount_of_coin_picked_up = 0;
bool should_rotate;
bool should_move;
bool stop_move_first_iteration;
bool rotation_first_iteration;
bool first_step = false;
double map_row_size;
double map_column_size;
double** coin_positions;
#pragma endregion


int found_coin_at_index = 1;

int main(int argc, char **argv)
{
    static double epsilon = 0.001;


#pragma region Local Variable Declarations
    DirectionAngle turn_to;
    TurnDirection turn_direction;
    TurnDirection opposite_turn_direction = Clockwise;

    const double* robot_gps_axis_to_watch = NULL;
    const int laser_sensor_count = 3;
    double robot_forward_angle;
    double target_distance;
    double laser_sensor_distance;
    double gps_x;
    double gps_z;
    double temp_robot_new_x;
    double temp_robot_new_z;
    double x_coordinate_new;
    double z_coordinate_new;
    double x_coordinate_old;
    double z_coordinate_old;
    double arena_row_size;
    double arena_column_size;
    double tile_row_count;
    double tile_column_count;
    double average;
    float robot_current_facing_angle;
    bool should_stop_moving;
    bool is_on_coin = false;
    bool is_path_obstructed;
    bool is_movement_positive_ve;
    int axis_to_watch;
#pragma endregion


#pragma region Local Q-Learning Variable Declarations
    InertialUnit* imu;
    Node* arena;
    Array2D<double> q_table;
    const int number_of_episodes = 10;
    double battery;
    double exploration_rate;
    double explore_probability;
    double current_episode_reward;
    double rewards_all_episodes[number_of_episodes];
    int start_position_row;
    int start_position_column;
    int current_position[2];
    int new_position[2];
    int current_episode = 0;
#pragma endregion


#pragma region Setup

    /* Calculate the distance to travel based on the size of the tile and the subdivisions per tile */
    distance_to_travel_linear = SIZE_OF_TILE / NUMBER_OF_SUBDIVISIONS;

    /* Robot uses supervisor so we can obtain world info as well */
    environment = new Supervisor();
    timeStep = environment->getBasicTimeStep();

    /* Gets a reference to the floor/arena */
    arena = environment->getFromDef("Arena");
    
    /* Get map size in meters */
    arena_row_size = (arena->getField("floorSize")->getSFVec2f())[0];
    arena_column_size = (arena->getField("floorSize")->getSFVec2f())[1];

    tile_row_count = NUMBER_OF_TILES_PER_METER * arena_row_size;
    tile_column_count = NUMBER_OF_TILES_PER_METER * arena_column_size;

    map_row_size = (tile_row_count * NUMBER_OF_SUBDIVISIONS) + 1;
    map_column_size = (tile_column_count * NUMBER_OF_SUBDIVISIONS) + 1;
    
    /* Initialize q-table */
    q_table.row_count = map_row_size * map_column_size;
    q_table.column_count = AMOUNT_OF_MOVES;

    /* Instantiate q-table */
    q_table.array = new double* [q_table.row_count];
    for (int i = 0; i < q_table.row_count; i++)
        q_table.array[i] = new double[q_table.column_count];
    if (!LoadState(&q_table))
    {
        for (int i = 0; i < q_table.row_count; i++)
            for (int j = 0; j < q_table.column_count; j++)
                q_table.array[i][j] = BOARD_BASE_SCORE;
    }

    /* Get the amount of coin */
    amount_of_coin = environment->getFromDef("CoinContainer")->getField("children")->getCount();

    /* Initialize the coin position array */
    coin_positions = new double* [amount_of_coin];
    for (int i = 0; i < amount_of_coin; i++)
    {
        coin_positions[i] = new double[2];
    }
    
    /* Get coin locations */

    for (int i = 0; i < amount_of_coin; i++)
    {
        coin_positions[i][0] = environment->getFromDef("CoinContainer")->getField("children")->getMFNode(i)->getField("translation")->getSFVec3f()[0];
        coin_positions[i][1] = environment->getFromDef("CoinContainer")->getField("children")->getMFNode(i)->getField("translation")->getSFVec3f()[2];
    }

#pragma endregion


#pragma region Pre-Loop Things
    /* Get both motors */
    left_motor = environment->getMotor("left wheel motor");
    right_motor = environment->getMotor("right wheel motor");

    /* Reseting values on start */
    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);      
    left_motor->setVelocity(MOTOR_STOP_SPEED);
    right_motor->setVelocity(MOTOR_STOP_SPEED);

    /* Get and enable the GPS and Inertial Unit */
    imu = environment->getInertialUnit("inertial unit");
    gps = environment->getGPS("gps");
    imu->enable(timeStep);
    gps->enable(timeStep);

    /* Get and enable sensors */
    DistanceSensor** laser_sensors = new DistanceSensor*[laser_sensor_count];
    laser_sensors[0] = environment->getDistanceSensor("distance sensor1");
    laser_sensors[1] = environment->getDistanceSensor("distance sensor2");
    laser_sensors[2] = environment->getDistanceSensor("distance sensor3");

    laser_sensors[0]->enable(timeStep);
    laser_sensors[1]->enable(timeStep);
    laser_sensors[2]->enable(timeStep);

    /* Q-Learning variables */
    battery = BATTERY_START_LEVEL;
    current_episode = 0;
    exploration_rate = 100.0;

    start_position_row = (int)map_row_size / 2;
    start_position_column = (int)map_column_size / 2;

    current_position[0] = start_position_row;
    current_position[1] = start_position_column;

    /* Get Indicator reference */
    indicator = environment->getFromDef("Indicator")->getField("children")->getMFNode(0)->getField("appearance")->getSFNode()->getField("baseColor");

    /* Variable initial values */
    x_coordinate_new = 0.0;
    z_coordinate_new = 0.0;
    x_coordinate_old = 0.0;
    z_coordinate_old = 0.0;

    //previous_direction = TopCenter;
    //turn_to = previous_direction;

    should_rotate = true;
    rotation_first_iteration = true;
    should_move = true;
    stop_move_first_iteration = true;

    for (int i = 0; i < number_of_episodes; i++)
    {
        rewards_all_episodes[i] = 0.0;
    }

    srand(time(0));

    std::cout << "Map Size: " << map_row_size << ", " << map_column_size << std::endl;
    std::cout << "Rover Starting: " << current_position[0] << ", " << current_position[1] << std::endl;
    
    amount_of_coin = 2;

    std::cout << "Amount of coins: " << amount_of_coin << std::endl;
    std::cout << "Amount of episodes: " << number_of_episodes << std::endl;
    std::cout << "Amount of steps per episodes: " << BATTERY_START_LEVEL/BATTERY_LOST_PER_MOVE << std::endl;

#pragma endregion

    ResetCoins();

    while ((environment->step(timeStep) != -1) && (current_episode < number_of_episodes)) //step(timestep) is a simulation step and doesnt correspond to seconds in real-time.
    {


#pragma region If End of Episode
        /* Move to next episode if max steps reached or battery runs out */
        if ( (battery <= MINIMUM_BATTERY_LEVEL && should_move) || amount_of_coin_picked_up == amount_of_coin)
        {
            exploration_rate = MINIMUM_EXPLORATION_RATE + (MAX_EXPLORATION_RATE - MINIMUM_EXPLORATION_RATE) * exp(EXPLORATION_DECAY_RATE * current_episode);

            battery = BATTERY_START_LEVEL;

            rewards_all_episodes[current_episode] = current_episode_reward;
            current_episode++;

            average = 0.0;

            for (int i = 0; i < current_episode; i++)
            {
                average += rewards_all_episodes[i];
            }

            average /= current_episode;

            std::cout << "Average for episode: " << average << std::endl;
            std::cout << "Exploration rate is now at: " << exploration_rate << std::endl;
            std::cout << "Current episode number: " << current_episode << std::endl;

            current_episode_reward = 0.0;
            current_position[0] = start_position_row;
            current_position[1] = start_position_column;

            x_coordinate_old = 0.0;
            z_coordinate_old = 0.0;

            amount_of_coin_picked_up = 0;

            std::cout << "next.."<< std::endl;
            ResetEnvironment(); /* Move to next episode */
        }

#pragma endregion


#pragma region Rotation
        /* Decide where to rotate here based on exploration and exploitation */
        if (rotation_first_iteration)
        {
            x_coordinate_new = x_coordinate_old;
            z_coordinate_new = z_coordinate_old;

            new_position[0] = current_position[0];
            new_position[1] = current_position[1];

            turn_to = DecideMove(exploration_rate, &q_table, current_position, new_position);
        }

        /* Begin rotation here */
        if (should_rotate)
        {
            robot_gps_axis_to_watch = GetGpsCoordinateToWatch(turn_to);

            robot_forward_angle = RadianTo360Degree(imu->getRollPitchYaw()[2]);
            if (stop_move_first_iteration)
            {
                SetGoalPosition(turn_to, &x_coordinate_new, &z_coordinate_new, &target_distance);

                stop_move_first_iteration = false;

                if (rotation_first_iteration)
                {
                    gps_x = gps->getValues()[0] * -1;
                    gps_z = gps->getValues()[2] * -1;

                    temp_robot_new_x = x_coordinate_new;
                    temp_robot_new_z = z_coordinate_new;

                    temp_robot_new_x += gps_x;
                    temp_robot_new_z += gps_z;
                }

            }
            robot_current_facing_angle = RadianTo360Degree(((std::atan2(temp_robot_new_z, temp_robot_new_x) - 1.5708) * -1));

            turn_direction = GetTurnDirection(robot_forward_angle, robot_current_facing_angle);

            if (rotation_first_iteration)
            {
                opposite_turn_direction = (TurnDirection)(turn_direction * -1);
                rotation_first_iteration = false;
            }

            RotateRobot(imu, turn_to, turn_direction, opposite_turn_direction);
            continue;
        }
#pragma endregion


#pragma region Obstacle Detection


        /*                                        Q-Learning Algorithm                                      */

        if (should_move)
        {
            battery -= BATTERY_LOST_PER_MOVE;

            std::cout << "Battery left: " << battery << std::endl;

            is_path_obstructed = IsFacingObstacle(laser_sensors, laser_sensor_count);

            if (is_path_obstructed)
            {
                UpdateNewPositionScore(&q_table, turn_to, current_position, new_position, &current_episode_reward, REWARD_OBSTRUCTION);

                ResetDecisionVariables();
                continue;
            }



        }
#pragma endregion


#pragma region Movement
        /*                                                  MOVEMENT                                                                */

        if (should_move)
        {
            first_step = false;
            Move();
            is_movement_positive_ve = CheckIfMovementIsPositiveVE(turn_to);

            should_move = false;
        }
        else
        {
            should_stop_moving = CheckIfReachedTarget(is_movement_positive_ve, robot_gps_axis_to_watch, target_distance);

            if (should_stop_moving)
            {
                is_on_coin = CheckForCoin(x_coordinate_new, z_coordinate_new);

                if (is_on_coin)
                {
                    //exploration_rate = MAX_EXPLORATION_RATE;
                    HideCoin(x_coordinate_new, z_coordinate_new);
                    UpdateNewPositionScore(&q_table, turn_to, current_position, new_position, &current_episode_reward, REWARD_COIN * found_coin_at_index);
                }
                else
                {
                    UpdateNewPositionScore(&q_table, turn_to, current_position, new_position, &current_episode_reward, REWARD_EMPTY);
                }

                current_position[0] = new_position[0];
                current_position[1] = new_position[1];
                x_coordinate_old = x_coordinate_new;
                z_coordinate_old = z_coordinate_new;

                StopMove();
                ResetDecisionVariables();
            }

        }
#pragma endregion


    }

    SaveState(&q_table);

    CleanUp();
    return 0;
}

/* Checks if the position has a coin on it by comparing x and z coordinates */
bool CheckForCoin(double x_coordinate, double z_coordinate)
{
    static double epsilon = 0.001;
    
    for (int i = 0; i < amount_of_coin; i++)
    {
        //(coin_positions[i][0]) == x_coordinate && (coin_positions[i][1]) == z_coordinate
        if ( (abs(coin_positions[i][0] - x_coordinate) <= epsilon * abs(coin_positions[i][0])) && (abs(coin_positions[i][1] - z_coordinate) <= epsilon * abs(coin_positions[i][1])))
            if (IsCoinPickedUp(x_coordinate, z_coordinate))
            {
                found_coin_at_index = i+1;
                return true;
            }
    }
    return false;
}

/* Checks if a coin has been picked up */
bool IsCoinPickedUp(double x_coordinate, double z_coordinate)
{
    static double epsilon = 0.001;
    for (int i = 0; i < amount_of_coin; i++)
    {
        if ((abs(coin_positions[i][0] - x_coordinate) <= epsilon * abs(coin_positions[i][0])) && (abs(coin_positions[i][1] - z_coordinate) <= epsilon * abs(coin_positions[i][1])))
        {
            double coin_transparency = environment->getFromDef("CoinContainer")->getField("children")->getMFNode(i)->getField("children")->
                getMFNode(0)->getField("appearance")->getSFNode()->getField("transparency")->getSFFloat();
            if ((coin_transparency - COIN_TRANSPARENCY_PICKED) <= epsilon * abs(coin_transparency))
            {
                return true;
            }
        }
    }
    
    return false;
}

/* Hides the coin that matches the x and z coordinates provided */
void HideCoin(double x_coordinate, double z_coordinate)
{
    static double epsilon = 0.001;

    double coin_x;
    double coin_z;
    for (int i = 0; i < amount_of_coin; i++)
    {
        if ((abs(coin_positions[i][0] - x_coordinate) <= epsilon * abs(coin_positions[i][0])) && (abs(coin_positions[i][1] - z_coordinate) <= epsilon * abs(coin_positions[i][1])))
        {
            environment->getFromDef("CoinContainer")->getField("children")->getMFNode(i)->getField("children")->
                getMFNode(0)->getField("appearance")->getSFNode()->getField("transparency")->setSFFloat(COIN_TRANSPARENCY_PICKED);
            amount_of_coin_picked_up++;
            return;
        }
    }
}

/* Resets the environment to its default state */
void ResetEnvironment()
{
    /* Reset Robot */
    environment->getFromDef("Khepera")->getField("translation")->setSFVec3f(ROBOT_DEFAULT_POSITION);
    environment->getFromDef("Khepera")->getField("rotation")->setSFRotation(ROBOT_DEFAULT_ROTATION);


    /* Reset Coin visibility */
    ResetCoins();

    ResetDecisionVariables();

    first_step = true;
    previous_direction = TopCenter;
    amount_of_coin_picked_up = 0;
}

/* Resets the transparency of all of the coins */
void ResetCoins()
{
    for (int i = 0 ; i < amount_of_coin ; i++)
    {
        environment->getFromDef("CoinContainer")->getField("children")->getMFNode(i)->getField("children")->
            getMFNode(0)->getField("appearance")->getSFNode()->getField("transparency")->setSFFloat(COIN_TRANSPARENCY_DEFAULT);
        //environment->getFromDef("CoinContainer")->getField("children")->getMFNode(i)->getField("children")->
        //    getMFNode(0)->getField("appearance")->getSFNode()->getField("baseColor")->setSFColor(DIRT_DEFAULT_COLOR);
    }
}

/* Updates the q-value for the q-table and the current episode reward */
void UpdateNewPositionScore(Array2D<double>* q_table, DirectionAngle direction, int* current_position, int* new_position, double* current_episode_reward, double reward)
{

    int action = GetDirectionIndex(direction);
    int state = (map_row_size * current_position[0]) + current_position[1];
    int new_state = (map_row_size * new_position[0]) + new_position[1];

    *current_episode_reward += reward;

    DirectionAngle best_direction = GetStateBestDirectionAngle(q_table, new_position);

    int new_pos_best_action_index = GetDirectionIndex(best_direction);

    //q_table->array[state][action] = (1 - LEARNING_RATE) * q_table->array[state][action] 
    //    + LEARNING_RATE * (reward + DISCOUNT_RATE * q_table->array[state][best_action_index]);

    //q_table->array[state][action] = q_table->array[state][action] + 
    //    (LEARNING_RATE * (reward + (DISCOUNT_RATE * q_table->array[new_state][new_pos_best_action_index]) - q_table->array[state][action]));

    q_table->array[state][action] = (1 - LEARNING_RATE) * q_table->array[state][action] + 
    (LEARNING_RATE * (reward + (DISCOUNT_RATE * q_table->array[new_state][new_pos_best_action_index])));

    std::string s;
    switch (action)
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

    std::cout << "State-Action (" << state << ", " << action << ") is updated to " << q_table->array[state][action] << " points." << std::endl;
}

/* Returns the index position of a direction */
int GetDirectionIndex(DirectionAngle direction)
{
    switch (direction)
    {
    case TopLeft:
        return 0;
    case TopCenter:
        return 1;
    case TopRight:
        return 2;
    case MiddleLeft:
        return 3;
    case MiddleRight:
        return 4;
    case BottomLeft:
        return 5;
    case BottomCenter:
        return 6;
    case BottomRight:
        return 7;
    }
}

/* Checks if the next move is obstructed */
bool IsFacingObstacle(DistanceSensor** laser_sensors, int laser_sensor_count)
{
    static double epsilon = 0.001;
    //abs(laser_sensor_distance - (DISTANCE_SENSOR_THRESHOLD * laser_sensor_count)) <= epsilon * abs(laser_sensor_distance) 

    double laser_sensor_distance = 0;
    for(int k = 0 ; k < laser_sensor_count; k++)
        laser_sensor_distance += laser_sensors[k]->getValue();

    //if (laser_sensor_distance != DISTANCE_SENSOR_THRESHOLD * laser_sensor_count) return true;
    if (abs(laser_sensor_distance - (DISTANCE_SENSOR_THRESHOLD * laser_sensor_count)) > epsilon * abs(laser_sensor_distance)) return true;

    return false;
}

/* Sets the new goal position for the robot based on the direction provided */
void SetGoalPosition(DirectionAngle turn_to, double* x_coordinate_new, double* z_coordinate_new, double* target_distance)
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

/* Updates the (i, j) values of the robot in matrix space */
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

/* Decides the next move based on exploration-exploitation */
DirectionAngle DecideMove(double exploration_rate, Array2D<double>*q_table, int* current_position, int* new_position)
{
    DirectionAngle turn_to;
    int explore_probability = rand() % 101;

    //if (explore_probability <= exploration_rate)
    //{
    //    turn_to = ChooseRandomMove();
    //    indicator->setSFColor(INDICATOR_EXPLORING_COLOR);
    //}
    //else
    //{
        turn_to = GetStateBestDirectionAngle(q_table, current_position);
        indicator->setSFColor(INDICATOR_EXPLOITING_COLOR);
    //}

    if (first_step)
        turn_to = TopCenter;
    
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

    return angle;
}

/* Rotates the robot to face the direction provided */
void RotateRobot(InertialUnit* imu, DirectionAngle robot_forward_angle_delta, TurnDirection turn_direction, TurnDirection opposite_turn_direciton)
{
    ///* Returns if the current facing direction is the direction to rotate towards */
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
        //std::cout << "Done Rotating.." << std::endl;
        return;
    }


    /* Begins rotating either clockwise or anti-clockwise */
    left_motor->setVelocity(turn_direction * MOTOR_DEFAULT_SPEED);
    right_motor->setVelocity(turn_direction * -MOTOR_DEFAULT_SPEED);
}

/* Returns the optimal direction to rotate in to reach the target position */
TurnDirection GetTurnDirection(float angle_a, float angle_b)
{
    float angle_diff = angle_a - angle_b;

    if (angle_diff > 180) return Clockwise;
    else if (angle_diff < -180) return Anti_Clockwise;
    else if (angle_a > angle_b) return Anti_Clockwise;
    else if (angle_a < angle_b) return Clockwise;
}

/* Checks if the webot has reached the target position */
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

/* Checks whether the direction the webot has chosen to travel is positive VE or not */
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

/* Resets all variables related to rotation and movement only */
void ResetDecisionVariables()
{
    should_move = true;
    should_rotate = true;
    rotation_first_iteration = true;
    stop_move_first_iteration = true;
}

/* Sets the webot to move forward based on its local axis */
void Move()
{
    left_motor->setVelocity(MOTOR_DEFAULT_SPEED);
    right_motor->setVelocity(MOTOR_DEFAULT_SPEED);
}

/* Stops the webot's movement */
void StopMove()
{
    left_motor->setVelocity(MOTOR_STOP_SPEED);
    right_motor->setVelocity(MOTOR_STOP_SPEED);
}

/* Returns the coordinate to be observed based on the chosen direction to move in */
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

/* Chooses a random direction to move in */
DirectionAngle ChooseRandomMove()
{
    int move_index = rand() % AMOUNT_OF_MOVES;
    
    //if (move_index == 0)
    //    move_index = 1;
    //else if (move_index == 1)
    //    move_index = 3;
    //else if (move_index == 2)
    //    move_index = 4;
    //else if (move_index == 3)
    //    move_index = 6;

    switch (move_index)
    {
    case 0:
        return TopLeft;
    case 1:
        return TopCenter;
    case 2:
        return TopRight;
    case 3:
        return MiddleLeft;
    case 4:
        return MiddleRight;
    case 5:
        return BottomLeft;
    case 6:
        return BottomCenter;
    case 7:
        return BottomRight;
    }
}

/* Chooses the best action based on the current state */
DirectionAngle GetStateBestDirectionAngle(Array2D<double>* q_table, int* position)
{
    int best_index = 1;
    int state = (map_row_size * position[0]) + position[1];
    for (int j = 0; j < q_table->column_count; j++)
    {
        if (q_table->array[state][j] > q_table->array[state][best_index]) best_index = j;
    }

    //if ((q_table->array[state][1] > q_table->array[state][best_index]))
    //    best_index = 1;
    //if ((q_table->array[state][3] > q_table->array[state][best_index]))
    //    best_index = 3;
    //if ((q_table->array[state][4] > q_table->array[state][best_index]))
    //    best_index = 4;
    //if ((q_table->array[state][6] > q_table->array[state][best_index]))
    //    best_index = 6;

    switch (best_index)
    {
    case 0:
        return TopLeft;
    case 1:
        return TopCenter;
    case 2:
        return TopRight;
    case 3:
        return MiddleLeft;
    case 4:
        return MiddleRight;
    case 5:
        return BottomLeft;
    case 6:
        return BottomCenter;
    case 7:
        return BottomRight;
    }
}

/* Saves the current q_table */
void SaveState(Array2D<double>* q_table)
{
    std::fstream out("Q-Table.txt", std::ios::out);

    int states = q_table->row_count;
    int actions = q_table->column_count;

    out << states << " " << actions << "\n";

    for (int i = 0; i < states; i++)
    {
        for (int j = 0; j < actions; j++)
        {
            out << q_table->array[i][j] << " ";
        }
        out << "\n";
    }

    out.close();

    std::cout << "Successfully saved the current state of the q_table.. " << std::endl;
}

/* Loads the previous run q_table if exists */
bool LoadState(Array2D<double>* q_table)
{
    std::fstream in("Q-Table.txt", std::ios::in);

    if (!in)
    {
        std::cout << "No previous q_table found.. " << std::endl << "Creating new q_table.." << std::endl;
        return false;
    }

    int states;
    int actions;

    in >> states;
    in >> actions;

    if (q_table->row_count != states || q_table->column_count != actions)
    {
        std::cout << "The current state-action (" << q_table->row_count << ", " << q_table->column_count <<
            ") does not match the loaded table state-action (" << states << ", " << actions <<
            ").. The program will create a new q_table and overwrite the old one upon successful exit.." << std::endl;
        std::cout << "Kindly stop the execution of the program before a successful run has completed to prevent data overwritting.. " << std::endl;
        return false;
    }

    for (int i = 0; i < states; i++)
    {
        for (int j = 0; j < actions; j++)
        {
            in >> q_table->array[i][j];
        }
    }

    in.close();

    std::cout << "Successfully loaded previous q_table information.. " << std::endl;
    return true;
}

/* Cleans up all heap initialized memory */
void CleanUp()
{
    delete environment;
    delete left_motor;
    delete right_motor;
}

