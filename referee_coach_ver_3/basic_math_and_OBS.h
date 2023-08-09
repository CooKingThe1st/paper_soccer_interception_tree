#ifndef BASIC_MATH_AND_OBS   /* Include guard */
#define BASIC_MATH_AND_OBS

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#define SMALL_ENOUGH 0.00001

using namespace std;
#include <math.h>
#include <algorithm>    // std::swap
#include <vector>       // std::vector
#include <iostream>     // std::cout

const int NUM_GOAL_OPTION = 15;
const int MAX_DIR_ID = 21;

struct complex { double r,i; };
struct Point {double first, second;};

double pre_goal_position_x = 0;
double pre_goal_position_y[NUM_GOAL_OPTION] = {0};

double dir_velo_x[MAX_DIR_ID] = {0, 0  ,    0, 0.5,   0.5,  0.5, -0.5, -0.5, -0.5 , 1, 1, -1, -1,  2, 2, -2, -2, 4, -4, 0,  0};
double dir_velo_y[MAX_DIR_ID] = {0, 0.5, -0.5,   0,   0.5, -0.5,    0, -0.5,  0.5 , 1, -1, 1, -1, -2, 2, -2,  2, 0,  0, 4, -4};

// double dir_velo_x[MAX_DIR_ID] = {0, 0  , 0   , 0.5,  0.5, 0.5, -0.5 };// , 2, 2, -2, -2};
// double dir_velo_y[MAX_DIR_ID] = {0, 0.5, -0.5,   0,   0.5, -0.5, 0  };// , -2, 2, -2, 2};

Point get_goal(int id, int goal_id)
{
  double pos_y = (goal_id == -1) ? pre_goal_position_y[0] : pre_goal_position_y[goal_id];
  double pos_x = (id >= 7) ? pre_goal_position_x : -pre_goal_position_x;
  if (goal_id == -1) pos_x += (id >= 7) ? 0.5 : -0.5;
  return Point{pos_x, pos_y};
}

#define HIGH_BOUND_X 12.5 // org 13.25
#define HIGH_BOUND_Y 8 // org 8.83

double bound_x(double val){ return MAX(-HIGH_BOUND_X, MIN(HIGH_BOUND_X, val) );}
double bound_y(double val){ return MAX(-HIGH_BOUND_Y, MIN(HIGH_BOUND_Y, val) );}

const double GRID_SIZE = 0.5;
const int GRIDIFY_FIELD = int(2*HIGH_BOUND_Y/GRID_SIZE+1)*int(2*HIGH_BOUND_X/GRID_SIZE+1);
double grid_x[GRIDIFY_FIELD];
double grid_y[GRIDIFY_FIELD];

void build_grid_field(){
  int max_id_x = int(2*HIGH_BOUND_X/GRID_SIZE+1);
  int max_id_y = int(2*HIGH_BOUND_Y/GRID_SIZE+1);

  for (int i = 0; i < max_id_x; i++)
    for (int j = 0; j < max_id_y; j++){
      int id = i*max_id_y + j;
      grid_x[id] = i * GRID_SIZE - HIGH_BOUND_X;
      grid_y[id] = j * GRID_SIZE - HIGH_BOUND_Y;
    }
}

Point get_move_to_dir(Point Now, int dir){
  return Point{ bound_x( Now.first + dir_velo_x[dir] ),  bound_y (Now.second + dir_velo_y[dir] ) };
}

void set_up_goal(double init_x, double init_y){
  pre_goal_position_x = abs(init_x);
  pre_goal_position_y[0] = abs(init_y);
  pre_goal_position_y[1] = abs(init_y) - 0.3;
  pre_goal_position_y[2] = abs(init_y) + 0.3;
  pre_goal_position_y[3] = abs(init_y) - 0.5;
  pre_goal_position_y[4] = abs(init_y) + 0.5;
  pre_goal_position_y[5] = abs(init_y) - 0.7;
  pre_goal_position_y[6] = abs(init_y) + 0.7;
  pre_goal_position_y[7] = abs(init_y) - 0.9;
  pre_goal_position_y[8] = abs(init_y) + 0.9;
  pre_goal_position_y[9] = abs(init_y) - 1.1;
  pre_goal_position_y[10] = abs(init_y) + 1.1;
  pre_goal_position_y[11] = abs(init_y) - 1.25;
  pre_goal_position_y[12] = abs(init_y) + 1.25;
  pre_goal_position_y[13] = abs(init_y) - 1.3;
  pre_goal_position_y[14] = abs(init_y) + 1.3;
// 0, 0.5, -0.5, 0.1, -0.1, 0.15, -0.15, -0.2, 0.2, -0.25, 0.25, -0.3, 0.3, -0.35, 0.35
}

double length_vector(double x, double y) { return sqrt(x * x + y * y); }

double length_dist_vector(double x, double y, double ref_x, double ref_y) { return length_vector(x - ref_x, y - ref_y); }
double length_dist_point(Point A, Point B) { return length_vector(A.first - B.first, A.second - B.second); }

bool check_occupied(double x, double y, double ref_x, double ref_y, double range) { return length_dist_vector(x, y,ref_x,ref_y)  <= range; }
bool close_enough(Point A, Point B, double THRESHOLD) { return length_dist_point(A, B) <= THRESHOLD; }

double angle_bet(double xa, double ya, double xb, double yb){ return acos((xa * xb + ya * yb) / (sqrt(xa * xa + ya * ya) * sqrt(xb * xb + yb * yb))); }

double angle_difference(double target, double origin)
{
  double return_value = target - origin;

  double mirror = (target > 0) ? -(3.141592 - target + 3.141592) : (3.141592 + 3.141592 + target);
  if (fabs(mirror - origin) < fabs(return_value)) return_value = mirror - origin;
  return return_value;
}

int MAX_MASK_DIR = 4;

int get_mask_dir(double angle){
  double r_angle = radToDeg(angle);
  if (r_angle > 180) r_angle -= 360;
  if (r_angle < -180) r_angle += 360;
  if (r_angle >= 0 and r_angle < 90) return 0;
  else if (r_angle >= 90 and r_angle <= 180) return 1;
  else if (r_angle <= 0 and r_angle > -90) return 2;
  else if (r_angle <= -90 and r_angle >= -180) return 3;
  else return -1000;
}

double get_angle_ccw(double orient_0, double orient_1, double id_0, double id_1){
  double dot = orient_0*orient_1 + id_0*id_1;      // dot product between [orient_0, id_0] and [orient_1, id_1]
  double det = orient_0*id_1 - id_0*orient_1;      // determinant
  double angle = atan2(det, dot);
  return angle;
}

double get_dist_point_line(double real_point_0, double real_point_1, double coe_0, double coe_1){
  return fabs((coe_0*real_point_0)-real_point_1+coe_1)/sqrt((coe_0*coe_0)+1);
}

double dist_line(double pct1X, double pct1Y, double a, double b, double c)
{
     return abs(a * pct1X + b * pct1Y + c) / sqrt(a * a + b * b);
}

void get_line(double x1, double y1, double x2, double y2, double &a, double &b, double &c)
{
       // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y) 
       a = y1 - y2; // Note: this was incorrectly "y2 - y1" in the original answer
       b = x2 - x1;
       c = x1 * y2 - x2 * y1;
}

Point solve_quadratic(double a, double b, double c)
{
  if (a < 0) {a = -a; b = -b; c = -c;}
  Point result={0, 0};

  if(a<0.000001)    // ==0
  {
    if(fabs(b)>0.000001)  // !=0
      result.first=result.second=-c/b;
    else
      if(fabs(c)>0.00001) result.first=result.second = NAN;
    return result;
  }
  double delta=b*b-4*a*c;
  if(delta>=0.0)
  {
    result.first  =(-b-sqrt(delta))/2/a;
    result.second =(-b+sqrt(delta))/2/a;
  }
  else
    result.first=result.second = NAN;
  if (not isnan(result.first) && result.second < result.first) std::swap(result.first, result.second);
  return result;
}

#define INF 2147483647
Point _INF_ = {INF, INF};
Point _NAN_ = {NAN, NAN};

pair<double, Point> cone_get_line(Point Start, Point Sink, double this_speed, Point Trepasser, double trep_speed, bool NO_LIMIT = 0)
{
  // SIMPLE LINE FUNCTION 3D of BALL from 1 -> 2: with (pos A, pos B, v Ball Average)
  // x = pct1X + t * (pct2X - pct1X)
  // y = pct1Y + t * (pct2Y - pct1Y)
  // z = 0     + t * tFinal
  // tFinal = length_vector(pct2X - pct1X, pct2Y - pct1Y) / v Ball Average
  // t = 0..1

  // SIMPLE OPPONENT FUNCTION: with (pos Opp, v Opp)
  // (x - oppX)^2 + (y - oppY)^2 = (z*vOpp)^2 
  // z >= 0

  // SIMPLE EQUATION SOLVE with (pos A, pos B, v Ball Average, pos Opp, v Opp)
  // (pct1X - oppX + t * veA)^2 + (pct1Y - oppY + t * veB)^2 = (t*tFinal*vOpp)^2
  // t^2*(tFinal^2*vOpp^2) = (veA^2+veB^2)*t^2+(deltaX^2+deltaY^2)+2*(veA*deltaX+veB*deltaY)*t
  // => quadratic equation
  // A = veA^2+veB^2-tFinal^2*vOpp^2
  // B = 2*(veA*pct1X+veB*pct1Y)
  // C = (pct1X^2+pct1Y^2)

  double veA = Sink.first - Start.first;
  double veB = Sink.second - Start.second;
  double tFinal = length_vector(veA, veB) / this_speed;

  // cout << "length_vector " << length_vector(veA, veB)  << " tFinal " << tFinal << '\n';
  double deltaX = Start.first - Trepasser.first;
  double deltaY = Start.second - Trepasser.second;

  double A = veA*veA+veB*veB - tFinal*tFinal*trep_speed*trep_speed;
  double B = 2*(veA*deltaX + veB*deltaY);
  double C = deltaX*deltaX + deltaY*deltaY;

  Point t_root = solve_quadratic(A, B, C);

  // cout << "A " << A << " B " << B << " C " << C << '\n';
  // cout << " root_t " << t_root.first << ' ' << t_root.second << '\n';

  double multiplier = (NO_LIMIT) ? INF : 1;
  if (isnan(t_root.first)) return make_pair(NAN, _NAN_);
  else
  {
    if (0 <= t_root.first && t_root.first <= multiplier) return make_pair(t_root.first * tFinal, Point{Start.first + t_root.first * veA, Start.second+ t_root.first * veB} );
    else 
    if (0 <= t_root.second && t_root.second <= multiplier) return make_pair(t_root.second * tFinal, Point{Start.first + t_root.second * veA, Start.second+ t_root.second * veB});
    else return make_pair(NAN, _NAN_);
  }
}

double K_obs_1 = 0, K_obs_2 = 0.001;
int TOTAL_SEN = 15;
double MAX_DIST_RANGE = 1023;

void POD(double *velo_vec, double *rot_vec, double *sensor_value, int bias_dir, int* STUCKED_TIME)
{
    double pod[TOTAL_SEN];
        // Polar Ostacle Distribution Window Value
    int window_size = 4;

    double safe_distance = MAX_DIST_RANGE*9/10;

    double min_pod = INFINITY; 
    int min_pod_dir = -1;
    double max_pod = -INFINITY;
    int max_pod_dir = -1;

    double critical_distance = MAX_DIST_RANGE;

    // bool free_mode = 1;
    for (int i = 0; i < TOTAL_SEN; i++) // calculate pod[i]
    {
      pod[i] = 0;
      double sum_wei = (window_size + 1) * (window_size + 1);
      for (int j = -window_size; j <= window_size; j++)
      {
        if ((i + j < 0) || (i + j >= TOTAL_SEN)) continue;
        double Wei = (window_size - abs(j) + 1) / sum_wei; // weight function
        double F = (1 - MIN(safe_distance, MAX_DIST_RANGE - sensor_value[i + j])/ safe_distance) * (1 - MIN(safe_distance, MAX_DIST_RANGE - sensor_value[i + j])/ safe_distance) ;
        pod[i] = pod[i] + Wei * F;
      }
    }
    for (int i = 0; i < TOTAL_SEN; i++) // calculate pod[i]
    {
      if ((pod[i] < min_pod) || ( fabs(pod[i] - min_pod) * 10 <= min_pod && abs(i - bias_dir) < abs(min_pod_dir - bias_dir))){
        min_pod = pod[i];
        min_pod_dir = i;
      }
      if (pod[i] > max_pod){
        max_pod = MAX(max_pod, pod[i]);
        max_pod_dir = i;
      }
      if ( abs(i - bias_dir) <= 2 ) critical_distance = MIN(critical_distance, MAX_DIST_RANGE - sensor_value[i]);
      // printf(" at direction %d sensor value %f and pod %f with current min_pod_dir %d\n", i, sensor_value[i], pod[i], min_pod_dir);
    }
    // if (DEBUG_MODE) printf("dir and pod %f %f %f %f %f %f %f %f %f %f %f %f %f \n", pod[0], pod[1], pod[2], pod[3], pod[4], pod[5], pod[6], pod[7], pod[8], pod[9], pod[10], pod[11], pod[12]);
    // if (DEBUG_MODE) printf("choosing minpod %f go_dir %d max_pod %f crit_dis %f and free %d\n", min_pod, min_pod_dir, max_pod, critical_distance, free_mode);

    critical_distance = MAX(critical_distance, 0.000001); // avoid NAN
    // printf("critical_distance is %f with best_dir is %d with pod is %f \n", critical_distance, min_pod_dir, min_pod);
    // // to left is positive NOTE
    // // wmax is larger than 1 so no need to minimize
    // if (critical_distance > safe_distance)
    //   fix_velocity(&left_speed, &right_speed, K_obs_1 * (1 / critical_distance - 1 / safe_distance) / (critical_distance * critical_distance) - K_obs_2 * (critical_distance - safe_distance), (6 - min_pod_dir) * 0.26179);
    // else

    if (critical_distance < safe_distance) *STUCKED_TIME = (*STUCKED_TIME) + 1;
    else *STUCKED_TIME = 0;

    *rot_vec = (TOTAL_SEN / 2 - min_pod_dir) * 0.26179;    // else

    if (*STUCKED_TIME < 5)
     // *velo_vec =  K_obs_1 * (1 / critical_distance - 1 / MAX_DIST_RANGE) / (critical_distance * critical_distance) - K_obs_2 * (critical_distance - MAX_DIST_RANGE);
    {
      if (critical_distance < safe_distance) *velo_vec = 0.4;
      // else *velo_vec = 0.1;
    }
    // if (critical_distance > 0.02)
      // fix_velocity(&left_speed, &right_speed, K_dist * max_pod * cos( (6 - min_pod_dir) * 0.26179 ), (6 - min_pod_dir) * 0.26179);
    else
    {
      *velo_vec = 0.9;
      for (int i = 0; i < TOTAL_SEN; i++) // calculate pod[i]
      {
        if (fabs(pod[i] - min_pod) * 10 <= min_pod && abs(i - max_pod_dir) >= abs(min_pod_dir - max_pod_dir)){
          *rot_vec = (TOTAL_SEN / 2 - i) * 0.26179;    // else
        }
      }
    }

}

#endif // BASIC_MATH_AND_OBS