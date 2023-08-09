 #include <iomanip>
#include <ctime>
#include <fstream>
#include <stdio.h>
  #include <iostream>
#include <string.h>
#include <vector>
#include <webots/display.h>
#include <webots/keyboard.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <basic_strategy.h>

#define INFO_MODE 0
#define DECENTRALIZED 1
#define DEBUG_MODE 0
#define CHECK_COMMAND 1
#define DEBUG_WEBOT_API 0
#define WIRELESS_DEBUG_MODE 0
#define PAPER_STAT_MODE 1
#define GUARD_CHECK_MODE 1
#define UI_ON 1

#define ROBOTS 7*2  // number of robots
#define TEAM_A "SPN"
#define TEAM_B "NON"

#define KEYBOARD_V 86 // cv2
#define KEYBOARD_D 68 // double check
#define KEYBOARD_C 67 // calibrate
#define KEYBOARD_R 82 // record
#define KEYBOARD_P 80 // stop record
#define KEYBOARD_S 83 // capture once

int CURRENT_BRAIN_LEVEL = 1;//1->Naive, 3->ADVANCED

#define FIXED_FLOAT(x) std::fixed <<std::setprecision(2)<<(x)

using namespace std;

//  player vari
const char *robot_name[ROBOTS] = {"NON_GK", "NON_CB", "NON_LB", "NON_RB", "NON_ST", "NON_LW", "NON_RW",
                                  "SPN_GK", "SPN_CB", "SPN_LB", "SPN_RB", "SPN_ST", "SPN_LW", "SPN_RW"};
int map_id[12] = {-1, 0, 1, -1, 3, 2, -1, -1, -1, 4, 5, 6};
                //   gk  cb     rb lb             st lw rw

WbNodeRef player_def[ROBOTS];
WbNodeRef NAN_DEF;
double player_position[ROBOTS][3], player_rotation[ROBOTS][4];
double player_initial_position[ROBOTS][3], player_initial_rotation[ROBOTS][4];

int player_state[ROBOTS], player_ball[ROBOTS];
float player_param_main[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};
float player_param_sub[ROBOTS] = {-1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000,-1000,-1000,-1000, -1000, -1000};


float old_player_param_main[ROBOTS];
int old_player_state[ROBOTS];
int non_last_ball = -1, spn_last_ball = -1;
int player_last_touch = -1, cached_player_last_touch = -1;
bool ball_check_hold = 1;

bool missing_player[ROBOTS] = {0};

//  ball and robot pose
WbNodeRef ball_node;
WbNodeRef goal_node[2];
double ball_reset_timer = 0;
double ball_velo = 0;
double ball_position[3] = {0, 0, 0.2};
double old_ball_position[3] = {0, 0, 0.2};
double ball_moving_direction[2] = {0, 0};
 
double ball_initial_position[3] = {0, 0, 0.2};
double goal_position_x[2][3] = {0}, goal_position_y[2][3] = {0}; 
double GOAL_X_LIMIT = 1000;

int TIME_STEP;

// referee vari
char time_string[64];
int i, j;
int score[2] = {0, 0};
const double TIME_FULL_MATCH = 240;
double match_time = TIME_FULL_MATCH;  // a match lasts for 3 minutes

// referee sensor
WbDeviceTag non_emitter, non_receiver, spn_emitter, spn_receiver;
  // keyboard supported variable
int input = 0;
bool key_pressed = false;

// player return value

  // initial must be -1
int color_sen[ROBOTS][4];
int neigh_sen[ROBOTS];

void restart_all();

//----------------------OUTPUT

std::string get_file_timestamp(int mode = 0)
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  if (mode == 0) strftime(buffer,sizeof(buffer),"_%d-%m-%Y-%H-%M-%S",timeinfo);
  else if (mode==1) strftime(buffer,sizeof(buffer),"_%d-%m-%Y",timeinfo);
  else assert(false);
  return std::string(buffer);
}

ofstream MyFile;

void clear_file(string file_name){
  std::ofstream ofs;
  ofs.open(file_name, std::ofstream::out | std::ofstream::trunc);
  ofs.close();
}

std::string get_pre_set(){
  string pre_set;
  int num_robot = 0;
  for (int i = 0; i < ROBOTS; i++)
    if (!missing_player[i]) num_robot++;
  if (num_robot == 4) pre_set = "A_" ;
  else if (num_robot == 10) pre_set = "B_";
  else if (num_robot == ROBOTS) pre_set = "C_";
  else assert(false);

  if (CURRENT_BRAIN_LEVEL == 1)  pre_set = pre_set + "NVE_";
  else if (CURRENT_BRAIN_LEVEL == 3)  pre_set = pre_set + "ADV_";
  else assert(false);
  return pre_set;
}

void write_to_file(string record_file_name, string data)
{
    string suffix = get_file_timestamp(1);
    string prefix = get_pre_set();
    MyFile.open(prefix+record_file_name+suffix+".txt", ios::app);
    MyFile << data;
    // for (i = 0; i < ROBOTS; i++) {
    //   if (missing_player[i]) continue;
    //   MyFile << robot_name[i] << " pos ";
    //   for (j = 0; j < 3; j++)
    //     MyFile <<  player_position[i][j] << " ";

    //   MyFile << " rot ";
    //   for (j = 0; j < 4; j++)
    //     MyFile <<  player_rotation[i][j] << " ";

    //   MyFile << '\n';
    // }
    MyFile.close();
}
//------------------------- support function -------------------------

static void set_scores(int a, int b, double pos, int lid, int rid) {
  if (!UI_ON) return;
  char c_score[16];
  // convert int to c_score char
  sprintf(c_score, "%d", a);
  wb_supervisor_set_label(lid, c_score, 0.92, pos, 0.1, 0xffff00, 0.0, "Arial");  // spn c_score
  sprintf(c_score, "%d", b);
  wb_supervisor_set_label(rid, c_score, 0.22, pos, 0.1, 0x0000ff, 0.0, "Arial");  // non score
}

static void set_percent(double a, double b, double pos, int lid, int rid) {
  if (!UI_ON) return;
  char c_score[16];
  // convert int to c_score char
  sprintf(c_score, "%.02f %%", a*100);
  wb_supervisor_set_label(lid, c_score, 0.92, pos, 0.1, 0xffff00, 0.0, "Arial");  // spn percent
  sprintf(c_score, "%.02f %%", b*100);
  wb_supervisor_set_label(rid, c_score, 0.22, pos, 0.1, 0x0000ff, 0.0, "Arial");  // non percent
}

static void show_command(){
  if (!UI_ON) return;
  string list_command = "";
  for (int i = 0; i < ROBOTS; i++){
    if (missing_player[i]) continue;
    list_command = list_command + robot_name[i] + " " + std::to_string(player_state[i]) + "    "; 
  }
  // cout << list_command << '\n';
  wb_supervisor_set_label(37, list_command.c_str(), 0.02, 0.8, 0.1, 0xff0000, 0.0, "Arial");  // non percent
}


inline int robot_decrypt(int en_id) { return (en_id >= 500) ? map_id[en_id - 500] : (map_id[en_id] + 7); }
//------------------------init function--------------------------

void init_robot()
{
  const int temp = wb_robot_get_basic_time_step();
  TIME_STEP = temp;

  for (int x = 0; x < ROBOTS; x++)
  {
    neigh_sen[x] = -1;
    for (int y = 0; y < 4; y++)
      color_sen[x][y] = -1;
  }

  // const char* TEMP_ROBOT_NAME = wb_robot_get_name();
  // ROBOT_NAME = TEMP_ROBOT_NAME;

  // char team_name[3];
  // memcpy( team_name, TEMP_ROBOT_NAME, 3 ); // first 3 character
  // if (strcmp(team_name,TEAM_A) == 0) 
  // {
  //   ROBOT_TEAM = 0;
  //   goal_node = wb_supervisor_node_get_from_def("BLUE_GOAL");
  // }
  // else if (strcmp(team_name,TEAM_B) == 0) 
  // {
  //   ROBOT_TEAM = 500;
  //   goal_node = wb_supervisor_node_get_from_def("YELLOW_GOAL");
  // }
  // else ROBOT_TEAM = -1;
  goal_node[0] = wb_supervisor_node_get_from_def("BLUE_GOAL");
  goal_node[1] = wb_supervisor_node_get_from_def("YELLOW_GOAL");

}

void init_sensor_actuator()
{
  spn_emitter = wb_robot_get_device("spn_emitter");
  non_emitter = wb_robot_get_device("non_emitter");

  spn_receiver = wb_robot_get_device("spn_receiver");
  non_receiver = wb_robot_get_device("non_receiver");

  wb_receiver_enable(spn_receiver, TIME_STEP);
  wb_receiver_enable(non_receiver, TIME_STEP);

  wb_keyboard_enable(TIME_STEP);
}

void init_player_ball_position()
{
  NAN_DEF = wb_supervisor_node_get_from_def("NAN");

  ball_node = wb_supervisor_node_get_from_def("BALL");
  wb_supervisor_node_enable_pose_tracking(ball_node, TIME_STEP, NULL);

  const double *tempPost = wb_supervisor_node_get_position( goal_node[0]);
  GOAL_X_LIMIT = (tempPost[0]);

  for (i = 0; i < ROBOTS; i++) {
    player_def[i] = wb_supervisor_node_get_from_def(robot_name[i]);
    // if (isnan(player_def[i])) missing_player[i] = 1;
    if (player_def[i] == NAN_DEF) missing_player[i] = 1;

    else wb_supervisor_node_enable_pose_tracking(player_def[i], TIME_STEP, NULL);
  }

  for (i = 0; i < 2; i++){
    const double *tempPost = wb_supervisor_node_get_position(goal_node[i]);

    goal_position_x[i][0] = tempPost[0]; goal_position_y[i][0] = tempPost[1];
    goal_position_x[i][1] = tempPost[0]; goal_position_y[i][1] = tempPost[1] - 0.6;
    goal_position_x[i][2] = tempPost[0]; goal_position_y[i][2] = tempPost[1] + 0.6;

    set_up_goal(abs(tempPost[0]), tempPost[1]);
  }
  build_grid_field();
}


//----------------------timer update function-----------------

void update_robot_pose()
{
  const double *tempBall = wb_supervisor_node_get_position(ball_node);
  for (j = 0; j < 3; j++) old_ball_position[j] = ball_position[j], ball_position[j] = tempBall[j];
  ball_moving_direction[0] = ball_position[0] - old_ball_position[0];
  ball_moving_direction[1] = ball_position[1] - old_ball_position[1];

  tempBall = wb_supervisor_node_get_velocity(ball_node);
  ball_velo = fabs(sqrt(tempBall[0] * tempBall[0] + tempBall[1] * tempBall[1]));

  for (i = 0; i < ROBOTS; i++) {
    if (missing_player[i]) continue;
    tempBall = wb_supervisor_node_get_position(player_def[i]);
    for (j = 0; j < 3; j++)
      player_position[i][j] = tempBall[j];
    if (INFO_MODE) printf("PLAYER %d is at %f %f %f\n", i, player_position[i][0], player_position[i][1], player_position[i][2]);
    WbFieldRef temp = wb_supervisor_node_get_field(player_def[i], "rotation");
    tempBall = wb_supervisor_field_get_sf_rotation(temp);

    for (j = 0; j < 4; j++)
      player_rotation[i][j] = tempBall[j];
  }
}

void reset_receiver(){
  while (wb_receiver_get_queue_length(non_receiver) > 0) {
      // const int *message = (int const*)wb_receiver_get_data(non_receiver);
      wb_receiver_next_packet(non_receiver);
  }
  while (wb_receiver_get_queue_length(spn_receiver) > 0) {
      // const int *message = (int const*)wb_receiver_get_data(spn_receiver);
      wb_receiver_next_packet(spn_receiver);
  }
}

void update_wireless_receiver()
{
  while (wb_receiver_get_queue_length(non_receiver) > 0) {
      const int *message = (int const*)wb_receiver_get_data(non_receiver);
      // const int data_length = wb_receiver_get_data_size(non_receiver) /sizeof(int);

      int true_id = robot_decrypt(*message);
      if (WIRELESS_DEBUG_MODE) 
          cout << "RECEIVED: robot_id " << *message << " name " << robot_name[true_id] << " state " <<  *(message+1) << " ball_" << *(message+2) << '\n';   
      player_state[true_id] = *(message + 1);
      player_ball[true_id] = *(message + 2);

      if (player_ball[true_id] == 2) non_last_ball = true_id;
      if (player_ball[true_id] > 0) player_last_touch = true_id;

      wb_receiver_next_packet(non_receiver);
   }

  while (wb_receiver_get_queue_length(spn_receiver) > 0) {
      const int *message = (int const*)wb_receiver_get_data(spn_receiver);
      // const int data_length = wb_receiver_get_data_size(spn_receiver) /sizeof(int);

      int true_id = robot_decrypt(*message);
      if (WIRELESS_DEBUG_MODE) 
          cout << "RECEIVED: robot_id " << *message << " name " << robot_name[true_id] << " state " <<  *(message+1) << " ball_" << *(message+2) << '\n';   
      player_state[true_id] = *(message + 1);
      player_ball[true_id] = *(message + 2);

      if (player_ball[true_id] == 2) spn_last_ball = true_id;
      if (player_ball[true_id] > 0) player_last_touch = true_id;

      wb_receiver_next_packet(spn_receiver);
   }
}

void update_goal()
{
    if (ball_reset_timer == 0) {
      if (ball_position[0] > GOAL_X_LIMIT) {  // ball in the blue goal
        set_scores(++score[0], score[1], 0.01, 0, 1);
        ball_reset_timer = 3;                            // wait for 3 seconds before reseting the ball
      } else if (ball_position[0] < -GOAL_X_LIMIT) {  // ball in the yellow goal
        set_scores(score[0], ++score[1], 0.01, 0, 1);
        ball_reset_timer = 3;  // wait for 3 seconds before reseting the ball
      }
    } else {
      ball_reset_timer -= (double)TIME_STEP / 1000.0;
      if (ball_reset_timer <= 0) {
        ball_reset_timer = 0;
        restart_all();
      }
    }
}

// bool isolate_debug = 0;
// Point save_point_1, save_point_2;
//------------------- coaching function-------------
void command_decen(unsigned int time_step_now){



    for (int player = 0; player < ROBOTS; player++)
    {
      if (missing_player[player]) continue;
      try{
        if (COMMAND_DEBUG_MODE) 
          // if (player != 0 and player != 6) continue;
          if (player != 0) continue;
        Command_Pack get_command = change_mode(time_step_now, CURRENT_BRAIN_LEVEL, missing_player, player_position, player_ball, player_state, ball_position, player, Point{ball_moving_direction[0], ball_moving_direction[1]}, ball_velo);

        player_state[player] = get_command.player_state;
        player_param_main[player] = get_command.sub_param_0;
        player_param_sub[player] = get_command.sub_param_1;


        // if (player_ball[player]== 2) {
        //   isolate_debug = 1;
        //   save_point_1={ball_position[0],ball_position[1]};
        // }
        // if (isolate_debug){
        //   if (ball_velo > 0.34)cout << ball_velo << " ball velo " << '\n';
        //   if (ball_velo > 0.31 && ball_velo < 0.34) {
        //     save_point_2 = {ball_position[0],ball_position[1]};
        //     cout << length_dist_point(save_point_2, save_point_1) << " length ball \n";
        //   }
        //   player_state[player] = 0;
        // }

// FLOGGING BEHAVIOR

      } catch(...){
        cout << " player " << player << " ASSERTION \n";
      }
    }

    // if (non_last_ball == -1)
    //   player_state[robot_decrypt(501)] = 1,
    //   player_param_main[robot_decrypt(501)] = robot_decrypt(511);
    // else if (non_last_ball == robot_decrypt(501))
    //   player_state[robot_decrypt(511)] = 1,
    //   player_param_main[robot_decrypt(511)] = robot_decrypt(509);
    // else if (non_last_ball == robot_decrypt(511))
    //   player_state[robot_decrypt(509)] = 2,
    //   player_param_main[robot_decrypt(509)] = 0;

}

void transmit_coach()
{
    vector<int> changed;
    for (i = 0; i < 7; i++){
      if (missing_player[i]) continue;
      if (player_state[i] != old_player_state[i] or player_param_main[i] != old_player_param_main[i]) changed.push_back(i);
    }
    float data_send[changed.size() * 4] = {0};
    for (i = 0; i < int(changed.size()); i++){
      int c_id = changed[i];
      data_send[i*4] = c_id, data_send[i*4+1] = player_state[c_id], data_send[i*4+2] = player_param_main[c_id], data_send[i*4+3] = player_param_sub[c_id];

      if (CHECK_COMMAND){
        if (GUARD_CHECK_MODE && player_state[c_id] != 13) continue;
        string data_out = "ROBOT-> " + std::to_string(c_id) + " COMMAND->"+std::to_string(player_state[c_id]) + "\n";
        if (PAPER_STAT_MODE && player_state[c_id] == old_player_state[c_id]) continue;
        write_to_file("FLOG", data_out);
        // cout << " COACH_PACK->rob: " << c_id << " state " << player_state[c_id] << " param_m " << FIXED_FLOAT(player_param_main[c_id]) << " param_s " << FIXED_FLOAT(player_param_sub[c_id]) << '\n';
      }
    }
    if (changed.size()) wb_emitter_send(non_emitter, data_send, sizeof(float) * changed.size() * 4);


    changed = vector<int>();
    for (i = 7; i < ROBOTS; i++){
      if (missing_player[i]) continue;
      if (player_state[i] != old_player_state[i] or player_param_main[i] != old_player_param_main[i]) changed.push_back(i);
    }
    float _data_send[changed.size() * 4] = {0};
    for (i = 0; i < int(changed.size()); i++){
      int c_id = changed[i];
      _data_send[i*4] = c_id, _data_send[i*4+1] = player_state[c_id], _data_send[i*4+2] = player_param_main[c_id], _data_send[i*4+3] = player_param_sub[c_id];

      if (CHECK_COMMAND){
        if (GUARD_CHECK_MODE && player_state[c_id] != 13) continue;
        string data_out = "ROBOT-> " + std::to_string(c_id) + " COMMAND->"+std::to_string(player_state[c_id]) + "\n";
        if (PAPER_STAT_MODE && player_state[c_id] == old_player_state[c_id]) continue;
        write_to_file("FLOG", data_out);
        // cout << " COACH_PACK->rob: " << changed[i] << " state " << player_state[changed[i]] << " param_m " << FIXED_FLOAT(player_param_main[changed[i]]) << " param_s " << FIXED_FLOAT(player_param_sub[changed[i]]) << '\n';
      }
    }
    if (changed.size()) wb_emitter_send(spn_emitter, _data_send, sizeof(float) * changed.size()*4);

    for (i = 0; i < ROBOTS; i++)
      old_player_param_main[i] = player_param_main[i],
      old_player_state[i] = player_state[i];
}

void update_keyboard()
{
    const int key = wb_keyboard_get_key();
    if (key >= 0 && !key_pressed) {
      key_pressed = true;
      input = key;
      cout << "just press " << input << '\n';
    } else if (key == -1 && key_pressed) {
      key_pressed = false;
      switch (input){
        case KEYBOARD_C:
          break;
        default:
          ;
      }
    }
}

//----------------------referee function

double time_possession[2] = {1, 1};
double ball_attack_time[2] = {0, 0}; // total time in opponent field
double dead_ball_cached_time = 0;

int pass_attempt[2] = {0, 0}; int pass_cached = 0;
int pass_success[2] = {0, 0};

int shoot_attempt[2] = {0, 0};

void update_ball_possession()
{
  if (ball_position[0] >= 0) ball_attack_time[0]++; // in non field
  else ball_attack_time[1]++; // in spn field

  ball_check_hold = 1;
  // cout << "PLAYER BALL: " << '\n';
  for (i = 0; i < ROBOTS; i++)
  {
    // cout << player_ball[i] << " ";
    if (player_ball[i] > 0) ball_check_hold = 0;
    int _team = (i >= 7) ? 0 : 1;

    if (player_ball[i] == 2){
      // cout << "WTH " << player_state[i] << '\n';
      if (player_state[i] == 9) // PASS
        pass_attempt[_team]++;
      else if (player_state[i] == 2) // SHOOT
        shoot_attempt[_team]++;
      else cout << "CQQJZ\n";

      string StartMarker = std::string("Start_of_a_")+(player_state[i] == 9 ? "Pass" : "Shoot") + " from " + std::to_string(i) +" \n"; 
      if (pass_cached == 0) write_to_file("FLOG", StartMarker);
      if (player_state[i] == 9) pass_cached = 1;
      else if (player_state[i] == 2) pass_cached = -1;
    }
  }

  if (player_last_touch == -1) return;
  // cout << "now ball_last is " << player_last_touch << " with returned " << player_ball[player_last_touch] << " ball_check_hold " << ball_check_hold << '\n';
  // cout << " time possesion yellow " << time_possession[0] << " blue " << time_possession[1] << '\n';
  if (ball_check_hold == 1) dead_ball_cached_time += 1, cached_player_last_touch = player_last_touch;
  else
  {
    if (player_last_touch >= 7 and player_last_touch < 14) time_possession[0] += 1;
    else if (player_last_touch >= 0 and player_last_touch < 7) time_possession[1] += 1;

    if (cached_player_last_touch >= 7 && player_last_touch >= 7) {
      time_possession[0] += dead_ball_cached_time;
      if (cached_player_last_touch != player_last_touch && cached_player_last_touch != -1) {
        if (pass_cached == 1) 
          pass_success[0] += pass_cached,
          write_to_file("FLOG", std::string("A_good_pass_to ")+ std::to_string(player_last_touch)+"\n");
        else if (pass_cached == -1)
          write_to_file("FLOG", std::string("A_bad_shot_to ")+ std::to_string(player_last_touch)+"\n");
        pass_cached = 0;
        // cout << "spn " << player_last_touch << ' ' << cached_player_last_touch << '\n';
      }
    } else
    if (cached_player_last_touch < 7 && player_last_touch < 7) {
      time_possession[1] += dead_ball_cached_time;
      if (cached_player_last_touch != player_last_touch && cached_player_last_touch != -1) {
        if (pass_cached == 1) 
          pass_success[1] += pass_cached,
          write_to_file("FLOG", std::string("A_good_pass_to ")+ std::to_string(player_last_touch)+"\n");
        else if (pass_cached == -1)
          write_to_file("FLOG", std::string("A_bad_shot_to ")+ std::to_string(player_last_touch)+"\n");
        // cout << "non " << player_last_touch << ' ' << cached_player_last_touch << '\n';
        pass_cached = 0;
      }
    }
    else if (cached_player_last_touch != -1){
      pass_cached = 0;
      string ConsiderMarker = std::string("Switch_Possesion from ") + std::to_string(cached_player_last_touch) + " to " + std::to_string(player_last_touch) + "\n";
      write_to_file("FLOG", ConsiderMarker);
    }
    dead_ball_cached_time = 0;
    cached_player_last_touch = -1;
  }
}

// write_to_file("out_capture.txt", "frame " + to_string(counter_video_frame)+"\n");

void update_time_label()
{
    // Adds TIME_STEP ms to the time
    match_time -= (double)TIME_STEP / 1000;
    if (match_time < 0) {
      match_time = TIME_FULL_MATCH;  // restart
      score[0] = 0; score[1] = 0;
      set_scores(0, 0, 0.01, 0, 1);
      restart_all();
      // time_possession[0] = 1, time_possession[1] = 1;
      // dead_ball_cached_time = 0;
    }
    sprintf(time_string, "%02d:%02d", (int)(match_time / 60), (int)match_time % 60);
    wb_supervisor_set_label(86, time_string, 0.45, 0.01, 0.2, 0x6DFF33, 0.0, "Arial");  // black
}

unsigned int time_step_counter = 1;

void restart_all()
{
  string Header = std::string("TIME ") + get_file_timestamp() + " STEP " + std::to_string(time_step_counter) + "\n";
  string Possesion = "Possesion " + std::to_string(time_possession[0] / (time_possession[0] + time_possession[1])) + ' ' +  std::to_string(time_possession[1] / (time_possession[0] + time_possession[1])) + '\n';
  string PAttempt = "PassAttempt " + std::to_string(pass_attempt[0]) + " " + std::to_string(pass_attempt[1]) + " Success " + std::to_string(pass_success[0]) + " " + std::to_string(pass_success[1]) + "\n";
  string SAttempt = "ShootAttempt " + std::to_string(shoot_attempt[0]) + " " + std::to_string(shoot_attempt[1]) + "\n";
  string Ball_AtkRate = "BallAtkTime " + std::to_string(ball_attack_time[0]) + " " + std::to_string(ball_attack_time[1]) + "\n";

  string Goal_R = "Goal " + std::to_string(score[0]) + ' ' + std::to_string(score[1]) + '\n';

  string EndRoll = "EndOfReport\n";

  write_to_file("FLOG", Header + Possesion + PAttempt + SAttempt + Ball_AtkRate + Goal_R+EndRoll);
  wb_supervisor_simulation_reset();
  for (int i = 0; i < ROBOTS; i++)
    if (!missing_player[i]) wb_supervisor_node_restart_controller(player_def[i]);
  wb_supervisor_node_restart_controller(wb_supervisor_node_get_self());
  // wb_supervisor_world_reload();

  // double ball_velocity[6] = {0, 0, 0, 0, 0, 0};
  // wb_supervisor_node_set_velocity(ball_node, ball_velocity);
  // wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(ball_node, "translation"), ball_initial_position);

  // for (i = 0; i < ROBOTS; i++) {
  //   if (missing_player[i]) continue;
  //   wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(player_def[i], "translation"), player_initial_position[i]);
  //   wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(player_def[i], "rotation"), player_initial_rotation[i]);
  
  //   old_player_param_main[i] = -1;
  //   old_player_state[i] = -1;
  //   player_state[i] = 0;
  //   player_param_main[i] = -1000, player_param_sub[i] = -1000; player_ball[i] = 0;
  // }
  // non_last_ball = -1, spn_last_ball = -1;
  // player_last_touch = -1, cached_player_last_touch = -1;

  // reset_receiver();
}

void get_initial_pose()
{

  for (i = 0; i < ROBOTS; i++){
    for (j = 0; j < 3; j++)
      player_initial_position[i][j] = player_position[i][j];
    for (j = 0; j < 4; j++)
      player_initial_rotation[i][j] = player_rotation[i][j];
  }
  for (j = 0; j < 3; j++)
    ball_initial_position[j] = ball_position[j];
}

void little_reroll(){
  for (int i = 0; i < ROBOTS; i++){
    if (missing_player[i]) continue;
    srand(i+time(0));
    int choice = rand()%13;
    Point new_pose = get_move_to_dir(Point{player_initial_position[i][0], player_initial_position[i][1]}, choice);
    player_initial_position[i][0] = new_pose.first;
    player_initial_position[i][1] = new_pose.second;
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(player_def[i], "translation"), player_initial_position[i]);
  } 
}

int main(int argc, char **argv) {



  wb_robot_init();

  init_robot();

  init_player_ball_position();

  init_sensor_actuator();

  update_robot_pose();

  get_initial_pose();

  little_reroll();

  set_scores(0, 0, 0.01, 0, 1);

  wb_supervisor_set_label(20, "Score", 0.05, 0.01, 0.1, 0xfff0ff, 0.0, "Arial");
  wb_supervisor_set_label(21, "Posse", 0.05, 0.1, 0.1, 0xfff0ff, 0.0, "Arial");
  wb_supervisor_set_label(22, "P/Attem", 0.05, 0.2, 0.1, 0xfff0ff, 0.0, "Arial");
  wb_supervisor_set_label(23, "P/Succ", 0.05, 0.3, 0.1, 0xfff0ff, 0.0, "Arial");
  wb_supervisor_set_label(24, "S/Attem", 0.05, 0.4, 0.1, 0xfff0ff, 0.0, "Arial");

  // clear_file("out.txt");

  write_to_file("FLOG", "GAME_START\n");
  while (wb_robot_step(TIME_STEP) != -1) {

      time_step_counter += 1;
      update_robot_pose();
      update_wireless_receiver();
      // update_keyboard();
      update_ball_possession();

      set_percent(time_possession[0] / (time_possession[0] + time_possession[1]), time_possession[1] / (time_possession[0] + time_possession[1]), 0.1, 69, 70);
      set_scores(pass_attempt[0], pass_attempt[1], 0.2, 9, 10);
      set_scores(pass_success[0], pass_success[1], 0.3, 12, 13);
      set_scores(shoot_attempt[0], shoot_attempt[1], 0.4, 14, 15);

      show_command();
      // if (DECENTRALIZED)

      command_decen(time_step_counter);
      transmit_coach();
      update_time_label();
      update_goal();
  }

  wb_robot_cleanup();
  return 0;
}