// LOI BONG -> HSV

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <cassert>

#include <basic_math_and_OBS.h>

#include <webots/connector.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/gps.h>
#include <webots/led.h>
#include <vector> 
#include <cstdlib>  
#include <ctime>
//-------------USEFUL-FUNCTION-----------

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

//-------------PHYSICAL-LIMIT-STAT------

#define MAX_SPEED 10  // RADIAN/SECOND, needed to be TUNE_SECOND
#define HALF_LENGTH_LEG 0.265
#define RADIUS_WHEEL 0.0985
#define FRICTION 0.2
//-------------ROBOT-RELATED--------

#define TEAM_A "SPN"
#define TEAM_B "NON"
#define INFO_MODE 0
#define ALL_DEBUG_MODE 0
#define SENSOR_MODE 0
#define DEBUG_WEBOT_API 1
#define FIELD_VECTOR_DEBUG_MODE 0
#define SPEC_DEBUG_MODE (robot_encrypted_id == 503)

#define ACTION_DEBUG_MODE 0
#define WIRELESS_DEBUG_MODE 0

using namespace std;  
string ROBOT_NAME;
int ROBOT_TEAM;
int ROBOT_ID;
int robot_encrypted_id;
double TIME_STEP; 

//-------------STATE

int current_state = 0, old_state = -1;
double time_elapsed = 0;
bool action_done_return = 0;

//-------------MOTION_PLANNED--------
double ACTIVE_RANGE = 0.4; // 2*sqrt(2)*HALF_LENGTH_LEG

double Vmax = RADIUS_WHEEL * MAX_SPEED; // Radius of wheel * rad / second
double Wmax = (double)RADIUS_WHEEL * MAX_SPEED / HALF_LENGTH_LEG;

//-----------OBJECT_NAME--------------
WbNodeRef ball_node, goal_node;
double goal_position_x[3] = {0}, goal_position_y[3] = {0}; 

//-----------SENSOR_NAME----------------
    // ir sensor
WbDeviceTag ir_sensor_0, ir_sensor_1, ir_sensor_2, ir_sensor_3;
    // distance_sensor
WbDeviceTag sonar_sensor_0, sonar_sensor_1, sonar_sensor_2, sonar_sensor_3, sonar_sensor_4, sonar_sensor_5, sonar_sensor_6, sonar_sensor_7, sonar_sensor_8, sonar_sensor_9, sonar_sensor_10, sonar_sensor_11, sonar_sensor_12, sonar_sensor_13, sonar_sensor_14;
    // motor
WbDeviceTag left_wheel, right_wheel;
    // linear motor
WbDeviceTag springer;
    // connector 
WbDeviceTag magnetic_sensor;
WbNodeRef magnetic_node;
    // radio
WbDeviceTag emitter, receiver;
    // gps
WbDeviceTag gps;
    // bumper
WbDeviceTag bumper_sensor;
    
//----------------------------------------
//-----------SENSOR_VARIABLE--------------
    // motor left and right rotation per minute
double left_speed = 0.0, right_speed = 0.0;
double velo_vec = 0.0, rot_vec = 0.0;
    // distance sensor value, with touch ~= 1023 and farthest is 0
double sensor_value[15];
    // ir sensor value, with white and black range
double ir_value[4], old_ir_value[4];
int tile_value[4] = {-1, -1, -1, -1}, tile_value_old[4] = {-1, -1, -1, -1};
  //  TRAPPED
int STUCKED_TIME = 0, bumper_value;
//     // ROBOT GPS Value
double gps_value[3];  
//     // ROBOT DIR Value
double robot_dir_x, robot_dir_y, robot_dir_z;
double current_robot_dir = -1, robot_angle_velo = 0;  
//     // FORMATION MODE
// int formation_mode = 0;

//-----------MATH_FUNCTION-------------------------



void check_speed(double *speed) 
{ 
    *speed = MIN(*speed, MAX_SPEED);
    *speed = MAX(*speed, -MAX_SPEED);
    // *speed /= TUNE_SECOND;
}

// accumulate velocity with inverse kinematics
void add_vector(double *velo_vec, double *rot_vec, double velocity, double rotational)
{
      // if (INFO_MODE) printf("BEFORE velocity %f rotational %f\n", velocity, rotational);

    double new_u = (*velo_vec) * cos(*rot_vec) + (velocity) * cos(rotational);
    double new_v = (*velo_vec) * sin(*rot_vec) + (velocity) * sin(rotational);
    (*velo_vec) = length_vector(new_u, new_v);
    (*rot_vec) = atan2(new_v, new_u);
      // if (INFO_MODE) printf("AFTER velocity %f rotational %f\n", *velo_vec, *rot_vec);
}

// calculate direction vector to robot_dir_x and robot_dir_y, with THIS robot gps and unit_vector
// NOTE: for_some_reason, dis code buggy when used to calculate formation virtual node. so only used to get leader direction vector
void new_position(double *robot_dir_x, double *robot_dir_y, double *robot_dir_z, double unit_vector_x, double unit_vector_y, double unit_vector_z)
{
    const double *R_Matrix = wb_supervisor_node_get_orientation(wb_supervisor_node_get_self());

    *robot_dir_x = R_Matrix[0] * unit_vector_x + R_Matrix[1] * unit_vector_y + R_Matrix[2] * unit_vector_z;
    *robot_dir_y = R_Matrix[3] * unit_vector_x + R_Matrix[4] * unit_vector_y + R_Matrix[5] * unit_vector_z;
    *robot_dir_z = R_Matrix[6] * unit_vector_x + R_Matrix[7] * unit_vector_y + R_Matrix[8] * unit_vector_z;
}

//-----------------------------------------

#define ROBOTS 7*2  // number of robots

double ball_position[3] = {0, 0, 0.2};
double ball_predict_pos[3] = {0, 0, 0.2}; 
double old_ball_position[3] = {0, 0, 0.2};
double ball_moving_direction[2] = {0, 0};
double ball_velo;
//  robots vari
const char *robot_name[ROBOTS] = {"NON_GK", "NON_CB", "NON_LB", "NON_RB", "NON_ST", "NON_LW", "NON_RW", 
                                  "SPN_GK", "SPN_CB", "SPN_LB", "SPN_RB", "SPN_ST", "SPN_LW", "SPN_RW"};
int map_id[12] = {-1, 0, 1, -1, 3, 2, -1, -1, -1, 4, 5, 6};
inline int robot_decrypt(int en_id) { return (en_id >= 500) ? map_id[en_id - 500] : (map_id[en_id] + 7); }

int command[ROBOTS] = {0};
double param_main[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};
double param_sub[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};

WbNodeRef player_def[ROBOTS];
WbNodeRef NAN_DEF;
double player_position[ROBOTS][3];
bool missing_player[ROBOTS] = {0};

int i, j;

// -------------------sensor- value

#define MIN_DISTANCE_RANGE 1023

int num_robot_neighbor_estimated = 0;
int num_robot_neighbor_estimated_old = -1;
// int num_robot_inrange_estimated = 0;
double min_distance = -1;

double min_ir_value = 999, max_ir_value = -1;

double IR_CHANGE_THRESHOLD = 0.02;
double WHITE_THRESHOLD = 0.031;
bool HALT_SIGNAL[4] = {0, 0, 0, 0};

// ----------------player_state
#define EMITTER_INFO 1
int springer_counter = 0;
//----------------player_vector

#define N_VECTOR 3 
double field_vector_magnetude[N_VECTOR] = {0};
// double temp_vector_magnetude[N_VECTOR];
double field_vector_direction[N_VECTOR] = {0};
double final_mangetude = 0, final_direction = 0;
//----------------motion_PID
double delta_speed = 0.000001, small_speed = 0.4;
// NOTE: DIS could be F(dist) => 2 radius and gradient
double CHANGE_DIR_RANGE = 0.2;
double K_p1 = 0.2, K_p2 = 0.1;
double K_d1 = 5, K_d2 = 8;  
double K_ball = 0.4;  
double K_velo = 1, K_angle = 2.3;
//-------------------ball_state
int ball_hold_counter = 0;
int ball_release_counter = 0;
double distance_query = -1;
int ball_possesion = 0, old_ball_possesion = -1; // 0-none, 1-hold, 2-shooted
// -----------------


void init_player_ball_position()
{
  NAN_DEF = wb_supervisor_node_get_from_def("NAN");


  ball_node = wb_supervisor_node_get_from_def("BALL");
  wb_supervisor_node_enable_pose_tracking(ball_node, TIME_STEP, NULL);

  for (i = 0; i < ROBOTS; i++) {
    player_def[i] = wb_supervisor_node_get_from_def(robot_name[i]);
    // if (isnan(player_def[i])) missing_player[i] = 1;
    if (player_def[i] == NAN_DEF) missing_player[i] = 1;

    else wb_supervisor_node_enable_pose_tracking(player_def[i], TIME_STEP, NULL);
  }

  const double *tempPost = wb_supervisor_node_get_position(goal_node);

  goal_position_x[0] = tempPost[0]; goal_position_y[0] = tempPost[1];
  goal_position_x[1] = tempPost[0]; goal_position_y[1] = tempPost[1] - 0.6;
  goal_position_x[2] = tempPost[0]; goal_position_y[2] = tempPost[1] + 0.6;

  set_up_goal(goal_position_x[0], goal_position_y[0]);
  // if (ALL_DEBUG_MODE)
  //   printf("nani %f %f %f %f \n", goal_position_y[1], goal_position_y[2], goal_position_y[0], goal_position_x[0]);
}

void update_sensor_value()
{
    //bumper_value = wb_touch_sensor_get_value(bumper_sensor);
    // NOTE reverse value
    sensor_value[8] = (wb_distance_sensor_get_value(sonar_sensor_8));
    sensor_value[9] = (wb_distance_sensor_get_value(sonar_sensor_9));
    sensor_value[10] = (wb_distance_sensor_get_value(sonar_sensor_10));
    sensor_value[11] = (wb_distance_sensor_get_value(sonar_sensor_11));
    sensor_value[12] = (wb_distance_sensor_get_value(sonar_sensor_12));
    sensor_value[13] = (wb_distance_sensor_get_value(sonar_sensor_13));
    sensor_value[14] = (wb_distance_sensor_get_value(sonar_sensor_14));

    sensor_value[6] = (wb_distance_sensor_get_value(sonar_sensor_6));
    sensor_value[7] = (wb_distance_sensor_get_value(sonar_sensor_7));
    sensor_value[5] = (wb_distance_sensor_get_value(sonar_sensor_5));
    sensor_value[4] = (wb_distance_sensor_get_value(sonar_sensor_4));
    sensor_value[3] = (wb_distance_sensor_get_value(sonar_sensor_3));
    sensor_value[2] = (wb_distance_sensor_get_value(sonar_sensor_2));
    sensor_value[1] = (wb_distance_sensor_get_value(sonar_sensor_1));
    sensor_value[0] = (wb_distance_sensor_get_value(sonar_sensor_0));
    
    // ir_value[3] = (wb_distance_sensor_get_value(ir_sensor_3));
    // ir_value[2] = (wb_distance_sensor_get_value(ir_sensor_2));
    // ir_value[1] = (wb_distance_sensor_get_value(ir_sensor_1));
    // ir_value[0] = (wb_distance_sensor_get_value(ir_sensor_0));

    const double *gps_value_temp = wb_gps_get_values(gps);
    gps_value[0] = gps_value_temp[0];
    gps_value[1] = gps_value_temp[1];
    gps_value[2] = gps_value_temp[2];


    if (INFO_MODE) printf("gps %f %f \n", gps_value[0], gps_value[2]);

    // if (SENSOR_MODE) printf("sensor %f %f %f \n", sensor_value[7], sensor_value[6], sensor_value[8]);
}

void get_player_ball_position()
{
  new_position(&robot_dir_x, &robot_dir_y, &robot_dir_z, 0, 1, 0);
  robot_angle_velo = fabs(current_robot_dir - atan2(robot_dir_y, robot_dir_x)); 
  current_robot_dir = atan2(robot_dir_y, robot_dir_x);  

  if (INFO_MODE) printf("robot dir %f %f %f \n", robot_dir_x, robot_dir_y, robot_dir_z);
  if (INFO_MODE) printf("robot dir in deg %f \n", current_robot_dir); 

  const double *tempBall = wb_supervisor_node_get_position(ball_node);
  if (INFO_MODE) printf("ball position %f %f %f \n", tempBall[0], tempBall[1], tempBall[2]);
  for (j = 0; j < 3; j++) old_ball_position[j] = ball_position[j], ball_position[j] = tempBall[j];

  ball_moving_direction[0] = ball_position[0] - old_ball_position[0];
  ball_moving_direction[1] = ball_position[1] - old_ball_position[1];
  // if (SPEC_DEBUG_MODE) printf("ball direction %f  \n", atan2(ball_moving_direction[1], ball_moving_direction[0]));

  tempBall = wb_supervisor_node_get_velocity(ball_node);
  if (INFO_MODE) printf("ball velocity %f x %f y %f z \n", tempBall[0], tempBall[1], tempBall[2]);
  ball_velo = fabs(sqrt(tempBall[0] * tempBall[0] + tempBall[1] * tempBall[1]));

  for (i = 0; i < ROBOTS; i++) {
    if (missing_player[i]) continue;
    tempBall = wb_supervisor_node_get_position(player_def[i]);
    // transmit value
    for (j = 0; j < 3; j++)
      player_position[i][j] = tempBall[j];
    if (INFO_MODE) printf("PLAYER %d is at %f %f %f\n", i, player_position[i][0], player_position[i][1], player_position[i][2]);
    // robot_rotation_field[i] = wb_supervisor_node_get_field(node, "rotation");
    // robot_rotation[i] = wb_supervisor_field_get_sf_rotation(robot_rotation_field[i]);
  }
}

void get_related_info()
{
  num_robot_neighbor_estimated = 0;
  min_distance = -1;
  for (int x = 0; x < 8; x++)
  {
    num_robot_neighbor_estimated += (sensor_value[x] * 4 >= MIN_DISTANCE_RANGE);
  // COULD ENCODE ORDER OF OBSTACLE
  // COULD BE USEFUL FOR ORIENT
    if (sensor_value[x] > min_distance) min_distance = sensor_value[x];
    if (INFO_MODE) printf(" , %d : %f, ", x, sensor_value[x]);
  }
  if (INFO_MODE) printf("\n total_obstalce % d min distance %f \n", num_robot_neighbor_estimated,  min_distance);
  

  // quite slow reacting incase bug
  if (INFO_MODE) printf(" IR VALUE ir_n %f ir_w %f ir_s %f ir_e %f \n", ir_value[2], ir_value[3], ir_value[0], ir_value[1]);
  for (int x = 0; x < 4; x++)
    // tile_value[x] = (ir_value[x] > 0.034);
    {
      if (HALT_SIGNAL[x]) 
      {
        tile_value[x] = -1;
        continue;
      }
      if ((max_ir_value < 0 ) || old_ir_value[x] == 0) // the first update
      {
        max_ir_value = MAX(max_ir_value, ir_value[x]);
        min_ir_value = MIN(min_ir_value, ir_value[x]);
        tile_value[x] = (ir_value[x] > WHITE_THRESHOLD);
      }
      else
      {
        double diff_check = fabs(ir_value[x] - old_ir_value[x]);
        if (diff_check >= IR_CHANGE_THRESHOLD)
          tile_value[x] = (ir_value[x] > WHITE_THRESHOLD);
        else
     {
          if ((ir_value[x] > WHITE_THRESHOLD && old_ir_value[x] < WHITE_THRESHOLD) || ((ir_value[x] < WHITE_THRESHOLD && old_ir_value[x] > WHITE_THRESHOLD)))
          {
            // WHITE THRESHOLD IS NOT GOOD ENOUGH, HALT SENSOR
            HALT_SIGNAL[x] = 1;
      printf("\n WARNING THIS IS NOT A TEST, %s, SENSOR_ID %d HALTING DUE TO WRONG THRESHOLD with %f and %f",ROBOT_NAME.c_str(), x, ir_value[x], old_ir_value[x]);
          }
        }

      }
    }
    // BIG FALSE HERE, NEED TO CHECK WHEN COLLISION
    // SHOULD INCLUDE RANGE  with -1
    // CHECK FOR VARIANCE, WHEN COLLIDED, IR_VALUE GO UP TO 0.0283
  for (int x = 0; x < 4; x++)
    old_ir_value[x] = ir_value[x];
  // if (INFO_MODE) printf("gps %f %f %f \n", gps_value[0], gps_value[1], gps_value[2]);
}

void init_robot()
{
  const char* TEMP_ROBOT_NAME = wb_robot_get_name();
  ROBOT_NAME = TEMP_ROBOT_NAME;

  char team_name[3];  
  memcpy( team_name, ROBOT_NAME.c_str(), 3 ); // first 3 character  
  team_name[3] = '\0';
  if (strcmp(team_name,TEAM_A) == 0) 
  {
    ROBOT_TEAM = 0;
    goal_node = wb_supervisor_node_get_from_def("BLUE_GOAL");
  }
  else if (strcmp(team_name,TEAM_B) == 0) 
  {
    ROBOT_TEAM = 500;
    goal_node = wb_supervisor_node_get_from_def("YELLOW_GOAL");
  }
  else ROBOT_TEAM = -1;
  ROBOT_ID = (ROBOT_NAME[4]-'0')*100 + (ROBOT_NAME[5] - '0')*10 + (ROBOT_NAME[6] - '0');

  robot_encrypted_id = ROBOT_TEAM + ROBOT_ID;

  if (INFO_MODE) printf("WEBOTS ROBOT ID %d %d %s \n", ROBOT_TEAM, ROBOT_ID, TEMP_ROBOT_NAME);
  srand(ROBOT_ID + ROBOT_TEAM);
  const int temp = wb_robot_get_basic_time_step();
  TIME_STEP = temp;
}

void init_sensor_actuator()
{
  // get devices
  // bumper_sensor = wb_robot_get_device("base_cover_link");
  left_wheel = wb_robot_get_device("wheel_left_joint");
  right_wheel = wb_robot_get_device("wheel_right_joint");
  wb_motor_set_position(left_wheel, INFINITY);
  wb_motor_set_position(right_wheel, INFINITY);
  wb_motor_set_velocity(left_wheel, 0);
  wb_motor_set_velocity(right_wheel, 0);

  springer = wb_robot_get_device("springer");
  wb_motor_set_velocity(springer, 10);

  magnetic_sensor = wb_robot_get_device("connector");
  wb_connector_enable_presence(magnetic_sensor, TIME_STEP);

  magnetic_node = wb_supervisor_node_get_from_device(magnetic_sensor);
  WbFieldRef tens_field = wb_supervisor_node_get_field(magnetic_node, "tensileStrength");
  WbFieldRef sher_field = wb_supervisor_node_get_field(magnetic_node, "shearStrength");
  WbFieldRef tol_field = wb_supervisor_node_get_field(magnetic_node, "distanceTolerance");

  wb_supervisor_field_set_sf_float(tens_field, 23);
  wb_supervisor_field_set_sf_float(sher_field, 15);
  wb_supervisor_field_set_sf_float(tol_field, 0.065);

  string robot_bumper = string(robot_name[robot_decrypt(robot_encrypted_id)]) + ".Springer.BUMPER";
  WbNodeRef bumper= wb_supervisor_node_get_from_def(robot_bumper.c_str());
  WbFieldRef mat_field = wb_supervisor_node_get_field(bumper, "contactMaterial");
  wb_supervisor_field_set_sf_string(mat_field, "rubber");


  emitter = wb_robot_get_device("emitter");
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  sonar_sensor_0 = wb_robot_get_device("base_sonar_225deg");
  sonar_sensor_1 = wb_robot_get_device("base_sonar_270deg");
  sonar_sensor_2 = wb_robot_get_device("base_sonar_285deg");
  sonar_sensor_3 = wb_robot_get_device("base_sonar_300deg");
  sonar_sensor_4 = wb_robot_get_device("base_sonar_315deg");
  sonar_sensor_5 = wb_robot_get_device("base_sonar_330deg");
  sonar_sensor_6 = wb_robot_get_device("base_sonar_345deg");

  sonar_sensor_7 = wb_robot_get_device("base_sonar_0deg");
  sonar_sensor_8 = wb_robot_get_device("base_sonar_15deg");
  sonar_sensor_9 = wb_robot_get_device("base_sonar_30deg");
  sonar_sensor_10 = wb_robot_get_device("base_sonar_45deg");
  sonar_sensor_11 = wb_robot_get_device("base_sonar_60deg");
  sonar_sensor_12 = wb_robot_get_device("base_sonar_75deg");
  sonar_sensor_13 = wb_robot_get_device("base_sonar_90deg");
  sonar_sensor_14 = wb_robot_get_device("base_sonar_135deg");
 
  ir_sensor_0  = wb_robot_get_device("ir_sonar_0");
  ir_sensor_1  = wb_robot_get_device("ir_sonar_1");
  ir_sensor_2  = wb_robot_get_device("ir_sonar_2");
  ir_sensor_3  = wb_robot_get_device("ir_sonar_3");

  wb_distance_sensor_enable(sonar_sensor_0, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_1, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_2, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_3, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_4, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_5, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_6, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_7, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_8, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_9, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_10, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_11, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_12, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_13, TIME_STEP);
  wb_distance_sensor_enable(sonar_sensor_14, TIME_STEP);
 
  // wb_distance_sensor_enable(ir_sensor_0, TIME_STEP);
  // wb_distance_sensor_enable(ir_sensor_1, TIME_STEP);
  // wb_distance_sensor_enable(ir_sensor_2, TIME_STEP);
  // wb_distance_sensor_enable(ir_sensor_3, TIME_STEP);

  //wb_touch_sensor_enable(bumper_sensor, TIME_STEP);

  // led = wb_robot_get_device("led_leader");
  // wb_led_set(led, 1);
}

void emitter_publish()
{
    int data_send[3] = {robot_encrypted_id, current_state, ball_possesion};

    if ((current_state != old_state) || (ball_possesion != old_ball_possesion))
    { 
      if (EMITTER_INFO && SPEC_DEBUG_MODE) printf("Published once with id %d and %d and %d \n", data_send[0], data_send[1], data_send[2]);
      wb_emitter_send(emitter, data_send, sizeof(int) * 3);
    }

    old_state = current_state;
    old_ball_possesion = ball_possesion;
}

void update_wireless_receiver()
{
  while (wb_receiver_get_queue_length(receiver) > 0) {
      const float *message = (float*)wb_receiver_get_data(receiver);
      const int data_length = wb_receiver_get_data_size(receiver) /sizeof(float);

      if (WIRELESS_DEBUG_MODE) {    
        printf("PLAYER COMMAND %d %d %.2f \n", data_length, wb_receiver_get_data_size(receiver), *(message));
        for (i = 0; i < data_length/4; i++)
          printf("robot_id %.2f command %.2f param_main %.2f param_sub %.2f", *(message + i * 4), *(message + i * 4 + 1), *(message + i * 4 + 2),  *(message + i * 4 + 3));
        printf("\n");
      }    
      for (i = 0; i < data_length/4; i++)
        command[int(*(message + i * 4))] = int(*(message + i * 4 + 1)),
        param_main[int(*(message + i * 4))] = *(message + i * 4 + 2),
        param_sub[int(*(message + i * 4))] = *(message + i * 4 + 3);
      wb_receiver_next_packet(receiver);
   }
   current_state = command[ robot_decrypt(robot_encrypted_id) ];
   // if (SPEC_DEBUG_MODE) printf("GET CURRENT STATE %d and id %d\n", current_state,  robot_decrypt(robot_encrypted_id) );
}


void check_counter_springer()
{
  if (springer_counter-- < 0)
  {
    wb_motor_set_position(springer, 0.1);
    springer_counter = 0;
  }
}

#define FORCE2DISTANCE 0.01435
#define MAX_FORCE 0.39

void on_springer(int on_timer)  
{ 
  springer_counter = on_timer;  
  double applied_force = FORCE2DISTANCE*(distance_query-1.74)+0.18; 
  // applied_force = 0.2+((double) rand() / (RAND_MAX))*0.45; 
  // applied_force = 0.15;  
  // if (distance_query < 18 * ACTIVE_RANGE) applied_force -= 0.05; 
  // if (distance_query < 9 * ACTIVE_RANGE) applied_force -= 0.05;  
  // std::cout << " APPLIED FORCE " << applied_force << '\n'; 
  applied_force = MIN(applied_force, MAX_FORCE);  
  wb_motor_set_position(springer, applied_force); 
} 
void move_to_position(double target_x, double target_y, double *return_magnitude, double *return_direction, bool use_ball_pos = 0)
{
  double reflect_dir_x = target_x - gps_value[0];
  double reflect_dir_y = target_y - gps_value[1];

  if (use_ball_pos){
    reflect_dir_x = target_x - ball_position[0];
    reflect_dir_y = target_y - ball_position[1];
  }

  double reflect_dir_len = length_vector(reflect_dir_x, reflect_dir_y );
  distance_query = reflect_dir_len;

  *return_magnitude = 0;
  if (reflect_dir_len < CHANGE_DIR_RANGE)
    *return_magnitude = small_speed/2;
  else
  if (reflect_dir_len < 3*ACTIVE_RANGE)
    *return_magnitude = small_speed;
  else
  if (reflect_dir_len < 6*ACTIVE_RANGE)
    *return_magnitude = 0.2 + K_p1 * reflect_dir_len;
  else
  if (reflect_dir_len < 10 * ACTIVE_RANGE)
    *return_magnitude = Vmax - (10 * ACTIVE_RANGE - reflect_dir_len) * K_p2;
  else
    *return_magnitude = Vmax;
  
  double Vec_Dir = atan2(reflect_dir_y, reflect_dir_x); 
  double Robot_Dir = current_robot_dir; 
  *return_direction = angle_difference(Vec_Dir, Robot_Dir); 
  if (fabs(*return_direction) < 0.03) *return_direction = 0; 
}

void move_to_object(WbNodeRef track_object, double *return_magnitude, double *return_direction)
{  
  const double *temp_post = wb_supervisor_node_get_position(track_object);
  if (INFO_MODE) printf("track_object position %f %f %f \n", temp_post[0], temp_post[1], temp_post[2]);
  move_to_position(temp_post[0], temp_post[1], return_magnitude, return_direction);
}

void rotate_to_position(double target_x, double target_y, double *return_magnitude, double *return_direction)
{
  move_to_position(target_x, target_y, return_magnitude, return_direction);
  *return_magnitude = 0.001;
}

void ball_rotate_to_position(double target_x, double target_y, double *return_magnitude, double *return_direction)
{
  move_to_position(target_x, target_y, return_magnitude, return_direction, 1);
  *return_magnitude = 0.001;
}

void rotate_to_object(WbNodeRef track_object, double *return_magnitude, double *return_direction)
{
  move_to_object(track_object, return_magnitude, return_direction);
  *return_magnitude = 0.001;
}

double activate_function(double value) { return 0.001+value+0.5*value*value+value*value*value/6; }

void normalize_vector()
{
  double denomi = 0;
  for (i = 0; i < N_VECTOR; i++)
    denomi = denomi + activate_function(field_vector_magnetude[i]);
  for (i = 0; i < N_VECTOR; i++)
    field_vector_magnetude[i] = Vmax*activate_function(field_vector_magnetude[i])/denomi;
}
    
double loss_angle_velo(double a)  
{return 1 - tanh(fabs(a));} 
//{return 2*(1+-1/(1+exp(-fabs(a))) );} 
//{return fabs(a);} 
// {return MAX(0, (fabs(a) < 1.47 ? cos(a) : 1 - tanh(fabs(a))) );} 
double loss_angle_rotate()  
// { return MIN(K_angle, K_d2 * robot_angle_velo); }  
{return MAX(0.5,  1 - K_d1 * robot_angle_velo);}  
// {return 1;}
double F_angle(double a)  
// {return (a) * (K_angle - loss_angle_rotate());}  
{return (a) * (K_angle*loss_angle_rotate()) + cos(a)*sin(a) * K_angle/2.5; }  
// {return (a)* K_angle*loss_angle_rotate();} 
// {return (a)*K_angle;}  
// { return (a);} 


void apply_velocity(double *left_speed, double *right_speed, double *velocity, double *rotational)  
{ 
  *velocity = MIN( (*velocity), Vmax);  
  *velocity = MAX( (*velocity), -Vmax); 
  *rotational = MIN(  (*rotational), Wmax); 
  *rotational = MAX(  (*rotational), -Wmax);  
  if (SPEC_DEBUG_MODE) printf("brain command speed %f and velo %f\n", (*velocity), (*rotational));
  if (SPEC_DEBUG_MODE) printf("final speed velocity %f final speed angle %f \n",  (*velocity) / RADIUS_WHEEL * loss_angle_velo( *rotational ) * K_velo, HALF_LENGTH_LEG * F_angle(*rotational)/ RADIUS_WHEEL);
  if (SPEC_DEBUG_MODE) printf("with loss_velo %f loss_rotate %f and robot_speed_angle %f \n", loss_angle_velo( *rotational ), loss_angle_rotate(), robot_angle_velo );//F_angle(*rotational));  
  double new_left = ( (*velocity) / RADIUS_WHEEL * loss_angle_velo( *rotational ) * K_velo - HALF_LENGTH_LEG * F_angle(*rotational)/ RADIUS_WHEEL); 
  double new_right = ( (*velocity) / RADIUS_WHEEL * loss_angle_velo( *rotational ) * K_velo + HALF_LENGTH_LEG * F_angle(*rotational)/ RADIUS_WHEEL);  
  *left_speed += new_left; //*left_speed +  
  *right_speed += new_right; //*right_speed +   
  // if (SPEC_DEBUG_MODE) printf("sup_left %f sup_right %f new_left %f new_right %f\n", new_left, new_right, *left_speed, *right_speed);  
  *velocity = 0; *rotational = 0; 
}
  
void calculate_final_velo( double *final_mangetude, double *final_direction)  
{ 
  if (SPEC_DEBUG_MODE && FIELD_VECTOR_DEBUG_MODE){
    for (int i = 0; i < 3; i++){
      cout << "COMPONENT VEC " << i << " speed " << field_vector_magnetude[i] << " angle " << field_vector_direction[i] << '\n';
    }
  }
  // could have 2 ways: total magnetude or sum magnetude  
  // either way 
  double temp_sum_mag = 0, temp_sum_dir = 0;  
  for (i = 0; i < N_VECTOR; i++)  
    temp_sum_mag = temp_sum_mag + field_vector_magnetude[i],  
    temp_sum_dir = temp_sum_dir + fabs(field_vector_direction[i]);  
  // if (SPEC_DEBUG_MODE) printf("BEFORE sum %f normal %f %f \n",temp_sum, field_vector_magnetude[0], field_vector_magnetude[1]) && FIELD_VECTOR_DEBUG_MODE;  

  if ((temp_sum_mag > Vmax) || (temp_sum_dir > Wmax)) 
    normalize_vector(); 

  if (SPEC_DEBUG_MODE && FIELD_VECTOR_DEBUG_MODE){
    for (int i = 0; i < 3; i++){
      cout << "AFTER NORMAL COMPONENT VEC " << i << " speed " << field_vector_magnetude[i] << " angle " << field_vector_direction[i] << '\n';
    }
  }

  // if (SPEC_DEBUG_MODE) printf("AFTER sum %f normal %f %f \n",temp_sum_mag, field_vector_magnetude[0], field_vector_magnetude[1]);  
 // BIG QUESTION MARK HERE  
  double final_u = 0, final_v = 0;  
  for (i = 0; i < N_VECTOR; i++)  
  { 
    final_u = final_u + field_vector_magnetude[i] * cos(field_vector_direction[i]); 
    final_v = final_v + field_vector_magnetude[i] * sin(field_vector_direction[i]); 
      
    // apply_velocity(&left_speed, &right_speed, &field_vector_magnetude[i], &field_vector_direction[i]); 
    field_vector_magnetude[i] = 0;  
    field_vector_direction[i] = 0;  
  } 
  *final_mangetude = length_vector(final_u, final_v); 
  *final_direction = atan2(final_v, final_u); 
}

double F_dynamic_error(double ball_goal_distance){
  // ball_goal small -> error bigger
  if (ball_goal_distance < 3*ACTIVE_RANGE)
    return 0.11;
  else if (ball_goal_distance < 9*ACTIVE_RANGE)
    return 0.061;
  else if (ball_goal_distance < 18*ACTIVE_RANGE)
    return 0.031;
  else
    return 0.015;
}

void shoot_ball()
{
    if (wb_connector_get_presence(magnetic_sensor) == 1)
    {
      // wb_connector_lock(magnetic_sensor);
      if (ball_hold_counter++ == 10) { /* 20*0.32 seconds elapsed */
        wb_connector_unlock(magnetic_sensor);
        on_springer(5);
        ball_release_counter = -2;
        ball_hold_counter = 0;

        // current_state = 0;
        ball_possesion = 2;
      }
    }
}

void idle()
{
  for (i = 0; i < N_VECTOR; i++)
    field_vector_direction[i] = 0, field_vector_magnetude[0] = 0;
}

void predicted()  
{ 
  // ball_velo; ball_moving_direction, ball_possesion; => ball_final_pos  
  double friction = TIME_STEP/(1000*FRICTION); // NEED  TO BE CALCULATE BY HAND 
  double delta_ball_dir = length_vector(ball_moving_direction[0], ball_moving_direction[1]); 
  double travel_distance = ( (ball_velo - 0.55)/0.13 + 1.74  )/delta_ball_dir;  
  // printf("BALL TRAVEL_DIST %f and BALL_VELO %f\n", travel_distance, ball_velo);  
  // if (fabs(ball_velo) < 0.0001) travel_distance = 0.0001;  
  if (isnan(travel_distance)) travel_distance = 0.0001;   
  ball_predict_pos[0] =  ball_position[0] + ball_moving_direction[0]*travel_distance; 
  ball_predict_pos[1] =  ball_position[1] + ball_moving_direction[1]*travel_distance; 
  // if (SPEC_DEBUG_MODE) printf("TF %f BALL_VELO %f BALL FUTURE %f %f \n", ball_velo/friction, ball_velo, ball_predict_pos[0], ball_predict_pos[1]); 
} 

void move_to_ball() 
{ 
  double temp_vector[2] = { gps_value[0] - ball_position[0] , gps_value[1] - ball_position[1]}; 
  // if (SPEC_DEBUG_MODE) printf("DIFF CHECK BALL %f\n", angle_difference( atan2( temp_vector[1], temp_vector[0]), atan2(ball_moving_direction[1], ball_moving_direction[0]); 
  bool ENABLE_PREDICT = 0;
  if (fabs(angle_difference( atan2( temp_vector[1], temp_vector[0] ), atan2(ball_moving_direction[1], ball_moving_direction[0]) )) > degToRad(80) && ball_velo > 0.8 && ENABLE_PREDICT && length_vector(temp_vector[0], temp_vector[1]) > 2*ACTIVE_RANGE) 
  { 
    predicted();
    move_to_position(ball_predict_pos[0], ball_predict_pos[1], &field_vector_magnetude[0], &field_vector_direction[0]); 
    // if (SPEC_DEBUG_MODE) printf("PREDICTED BALL TO VECTOR 0\n"); 
  } 
  else  
  { 
    move_to_object(ball_node, &field_vector_magnetude[0], &field_vector_direction[0]);  
    // if (abs(field_vector_direction[0])*2 > M_PI) field_vector_magnetude[0] = MAX(field_vector_magnetude[0] - ball_velo * K_ball, 0.0001);  
    // if (SPEC_DEBUG_MODE) printf("STRAIGHT BALL TO VECTOR 0\n");  

  } 
  // field_vector_magnetude[1] = ball_velo*0.5,  
  // field_vector_direction[1] = angle_difference( atan2(ball_moving_direction[1], ball_moving_direction[0]), current_robot_dir); 
  if (STUCKED_TIME <= 15){
    field_vector_magnetude[2] = 0;
    field_vector_direction[2] = 0;
  }
}


// APPROXIMATE PASS
void pass_ball(int robot_id) // according name field
{
  if (wb_connector_get_presence(magnetic_sensor) == 0){
    move_to_ball();
  }
  else
  {
    if (ball_release_counter++ > 0) wb_connector_lock(magnetic_sensor);
    ball_rotate_to_position(player_position[robot_id][0], player_position[robot_id][1], &field_vector_magnetude[0], &field_vector_direction[0]);
    distance_query += 0.3;
    distance_query *= 1.3;
    double error_angle = F_dynamic_error(distance_query);
    if (fabs(field_vector_direction[2]) + fabs(field_vector_magnetude[2]) > 1.2) error_angle *= 3;
    // special calc.
    // double delta_angle = current_robot_dir - atan2( ball_position[1] - gps_value[1], ball_position[0] - gps_value[0] );
    // field_vector_direction[0] += delta_angle;
    // cout << "distance " << distance_query << " delta_a " << delta_angle << " permited " << error_angle << " now " << fabs(field_vector_direction[0]) << '\n';
    // cout << "POD "<< field_vector_direction[2] << " stuck " << STUCKED_TIME << " pod velo " << field_vector_magnetude[2] << '\n';

    if (fabs(field_vector_direction[0]) <= error_angle) shoot_ball();
    // if (fabs(field_vector_direction[2]) + fabs(field_vector_magnetude[2]) > 1.2) shoot_ball();
  }
}

void dribble(double mode) 
{
  Point mid_goal = get_goal(robot_decrypt(robot_encrypted_id), -1);
  if (mode == -1){// dont care 'bout ball though
    field_vector_magnetude[2] *= 2;
    move_to_position(mid_goal.first, mid_goal.second , &field_vector_magnetude[0], &field_vector_direction[0]);
  }
  else { // need to keep ball
    field_vector_magnetude[2] *= mode;  
    field_vector_direction[2] *= mode;  

    if (wb_connector_get_presence(magnetic_sensor) == 0)
      move_to_ball();
    else{
      if (ball_release_counter++ > 0) wb_connector_lock(magnetic_sensor);
      move_to_position(mid_goal.first, mid_goal.second , &field_vector_magnetude[0], &field_vector_direction[0]);
      // if ( fabs(field_vector_direction[2]) + fabs(field_vector_magnetude[2]) > 2.1 ) shoot_ball();
    }
  }
}

void field_shoot(double x, double y)
{
  if (wb_connector_get_presence(magnetic_sensor) == 0){
    move_to_ball();
  }
  else
  {
    if (ball_release_counter++ > 0) wb_connector_lock(magnetic_sensor);
    rotate_to_position(x, y, &field_vector_magnetude[0], &field_vector_direction[0]);
    distance_query += 0.3;
    distance_query *= 1.3;
    double error_angle = F_dynamic_error(distance_query);
    if (fabs(field_vector_direction[2]) + fabs(field_vector_magnetude[2]) > 1.2) error_angle *= 3;

    if (fabs(field_vector_direction[0]) <= error_angle ) shoot_ball();
  }

}

void goal_shoot(int goal_id)
{
  distance_query = 1;
  if (wb_connector_get_presence(magnetic_sensor) == 0)
  {
    move_to_ball();
  }
  else
  {
    if (ball_release_counter++ > 0) wb_connector_lock(magnetic_sensor);
    Point goal = get_goal(robot_decrypt(robot_encrypted_id), goal_id);
    ball_rotate_to_position(goal.first, goal.second, &field_vector_magnetude[0], &field_vector_direction[0]);

    // for (j = 1; j < 3; j++)
    // {
    //   double temp_magnetude, temp_direction;
    //   rotate_to_position(goal_position_x[j], goal_position_y[j], &temp_magnetude, &temp_direction);
    //   if (fabs(temp_direction) < fabs(field_vector_direction[0])) field_vector_direction[0] = temp_direction;
    // }
    double error_angle = F_dynamic_error(distance_query)*0.2;
    distance_query = 100000;
    if (fabs(field_vector_direction[0]) <= error_angle ) shoot_ball();
  }

}

void force_shoot(){
    // dangerous, carefull to use
    field_vector_magnetude[1] = 0;
    field_vector_direction[1] = 0;
}

void pressing(double x, double y)
{
  move_to_position(x, y, &field_vector_magnetude[0], &field_vector_direction[0]);
}

void capture(double x, double y)
{
  move_to_position(x, y, &field_vector_magnetude[0], &field_vector_direction[0]);
  if (length_dist_vector(x, y, gps_value[0], gps_value[1]) < 0.8) rotate_to_object(ball_node, &field_vector_magnetude[0], &field_vector_direction[0]);
  if (STUCKED_TIME <= 5){
    field_vector_magnetude[2] = 0;
    field_vector_direction[2] = 0;
  }
}

void tackle(int opp_player_id)
{
  assert(opp_player_id >= 0);
  if (missing_player[opp_player_id])
    assert(False);
  else
    move_to_object(player_def[opp_player_id], &field_vector_magnetude[0], &field_vector_direction[0]);
  
  if (STUCKED_TIME <= 15){
    field_vector_magnetude[2] = 0;
    field_vector_direction[2] = 0;
  }
}

int main() {

  wb_robot_init();

  init_robot();
  init_player_ball_position();
  init_sensor_actuator();
  
  wb_robot_step(TIME_STEP);
  while (wb_robot_step(TIME_STEP) != -1) {

    // left_speed = left_speed / 10;
    // right_speed = right_speed / 10;
    left_speed = 0;
    right_speed = 0;

    if (INFO_MODE) printf("ROBOT %d\n", ROBOT_ID);
    update_sensor_value();
    update_wireless_receiver();
    get_player_ball_position();
    // predicted();  
    get_related_info();

    check_counter_springer();

    // if (INFO_MODE) printf("ir 0 %f bumper %d", ir_value[0], bumper_value);
  
    // if (INFO_MODE) left_speed = 0, right_speed = 0;

    // move_to_object(ball_node, &field_vector_magnetude[0], &field_vector_direction[0]);
  // 0 - idle
  // 1 - pass (*player)
  // 2 - goal (*choose_dir)
  // 3 - pressing (*field) 
  // 4 - capture (*field) omit pod
  // 5 - tackle (*player)
  // 6 - dribble -> goal
  // 7 - shoot (*field)
    ball_possesion = wb_connector_get_presence(magnetic_sensor);

    // double field_vector_magnetude[2] = 0, field_vector_direction[2] = 0;
    POD(&field_vector_magnetude[2], &field_vector_direction[2], sensor_value, 7, &STUCKED_TIME);

    if (SPEC_DEBUG_MODE) printf("TEST POD with robot %d have stucked %d velo %f and dir %f \n", robot_decrypt(robot_encrypted_id), STUCKED_TIME, field_vector_magnetude[2], field_vector_direction[2]);
    switch(current_state) {
      case 0:
        idle();
        break;
      case 1:
        pass_ball(int(param_main[robot_decrypt(robot_encrypted_id)]));
        break;
      case 2:
        goal_shoot(int(param_main[robot_decrypt(robot_encrypted_id)]));
        break;
      case 8:
        move_to_ball();
        break;
      case 9:
        field_shoot(param_main[robot_decrypt(robot_encrypted_id)], param_sub[robot_decrypt(robot_encrypted_id)]);
        break;
      case 18:
        rotate_to_object(ball_node, &field_vector_magnetude[0], &field_vector_direction[0]);
        break;
      case 4:
        capture(param_main[robot_decrypt(robot_encrypted_id)], param_sub[robot_decrypt(robot_encrypted_id)]);
        break;
      case 14:
        capture(param_main[robot_decrypt(robot_encrypted_id)], param_sub[robot_decrypt(robot_encrypted_id)]);
        break;
      case 3:
        pressing(param_main[robot_decrypt(robot_encrypted_id)], param_sub[robot_decrypt(robot_encrypted_id)]);
        break;
      case 13:
        pressing(param_main[robot_decrypt(robot_encrypted_id)], param_sub[robot_decrypt(robot_encrypted_id)]);
        break;
      case 5:
        tackle(int(param_main[robot_decrypt(robot_encrypted_id)]));
        break;
      case 6:
        dribble(int(param_main[robot_decrypt(robot_encrypted_id)]));
        break;
      case 10:
        force_shoot();
        break;
      default:
        ;
    }

// MAY CHANGE 7 in here
    emitter_publish();
    if (STUCKED_TIME < 15){
      field_vector_magnetude[2] /= 2;
    }
    else if (STUCKED_TIME >= 15 && STUCKED_TIME < 40) {
      field_vector_direction[0] /= (STUCKED_TIME/10);
      field_vector_magnetude[0] /= (STUCKED_TIME/10);
    }
    else if (STUCKED_TIME > 40) {
      field_vector_direction[0] = delta_speed;
      field_vector_magnetude[0] = delta_speed;
    }
    // if (robot_encrypted_id == 501) cout << "WHO " << robot_encrypted_id << " POD "<< field_vector_direction[2] << " stuck " << STUCKED_TIME << " pod velo " << field_vector_magnetude[2] << '\n';
    // if (robot_encrypted_id == 501) cout << "WHO " << robot_encrypted_id << " MAIN "<< field_vector_direction[0] <<  " main velo " << field_vector_magnetude[0] << '\n';
    // if (robot_encrypted_id == 501) cout << "WHO " << robot_encrypted_id << " SUB "<< field_vector_direction[1] <<  " sub velo " << field_vector_magnetude[1] << '\n';

    calculate_final_velo(&final_mangetude, &final_direction);
    apply_velocity(&left_speed, &right_speed, &final_mangetude, &final_direction);

    // if (robot_encrypted_id == 501) cout << "WHO " << robot_encrypted_id << " FINAL "<< final_direction <<  " final velo " << final_mangetude << '\n';

    if (STUCKED_TIME > 60) {
      left_speed = -6;
      right_speed = -6;
    }
    // if (SPEC_DEBUG_MODE) left_speed = MAX_SPEED, right_speed = MAX_SPEED;

    wb_motor_set_velocity(right_wheel, -left_speed);
    wb_motor_set_velocity(left_wheel, -right_speed);

  }

  wb_robot_cleanup();

  return 0;
}
