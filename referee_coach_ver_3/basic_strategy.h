#ifndef BASIC_STRATEGY   /* Include guard */
#define BASIC_STRATEGY


//MODE, RISK FUNC
//  THRESHOLD, REWARD FUNC


#include <cassert>  
#include "basic_math_and_OBS.h"
#define ROBOTS 7*2  // number of robots
#define HALF_LENGTH_LEG 0.265
#define TACKLE_DEBUG_MODE 0
#define ATTACK_DEBUG_MODE 0
#define ROLE_DEBUG_MODE 0
#define COMMAND_DEBUG_MODE 0
#define STRATEGY_DEBUG_MODE 0
#define PAPER_ATTAK_MODE 0

int BRAIN_LEVEL_0= 0;  //"BRAINDEAD"
int BRAIN_LEVEL_1= 1;  //"SIMPLE"
int BRAIN_LEVEL_2= 2; //"DEFENSE"
int BRAIN_LEVEL_3= 3;  //"THROUGH"

int BRAIN_LEVEL_OTHER = -10;

#define GREEDY 1
#define BIAS_BALL 0

double THRESHOLD_DIST = 0.3; 
double THRESHOLD_RISK = 100;
int THRESHOLD_NUM_RISK = 3;
double THRESHOLD_SHOOT_RISK = 90;
double THRESHOLD_SHOOT_REWARD = 40;
double THRESHOLD_SHOOT_DIST = HIGH_BOUND_Y*0.6;

struct Command_Pack {
	int player_id;
	int player_state;
	double sub_param_0, sub_param_1;
};

void log_to_file(string record_file_name, string data)
{
	ofstream MyLog;
    MyLog.open(record_file_name+".txt", ios::app);
    MyLog << data;
    MyLog.close();
}

int get_team_possesion(int *player_ball)
{
	int check_non = 0, check_spn = 0;
	for (int i = 0; i < 7; i++) 
		if (player_ball[i] != 0) {
			if (check_non == 0) check_non = -1;
			else check_non = -3;
		}
	for (int i = 7; i < ROBOTS; i++)
		if (player_ball[i] != 0) {
			if (check_spn == 0) check_spn = 2;
			else check_spn = 4;
		}

	if (check_spn != 0 && check_non != 0) return 404; // in dispute
	else if (check_spn != 0) return check_spn;
	else if (check_non != 0) return check_non;
	else return 0;
}

int get_team(int player_id){ return (player_id < 7) ? -1 : 1;}

//quite useless because of boundary
bool clear_line(Point Start, Point Sink, vector<int> white_list, bool *missing, double player_pos[][3] , double multiplier = 1)
{
	bool arr_white_list[ROBOTS] = {0};
	for (auto itr : white_list) arr_white_list[itr] = 1;

	for (int better_cand = 0; better_cand < ROBOTS; better_cand++){
		if (missing[better_cand]) continue;
		if (arr_white_list[better_cand]) continue;

		double temp_a, temp_b, temp_c;
		get_line(Start.first, Start.second, Sink.first, Sink.second, temp_a, temp_b, temp_c);


		if (dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_a, temp_b, temp_c ) > THRESHOLD_DIST * multiplier) continue;

		double c_at_A = -(temp_b * Start.first - temp_a * Start.second);
		double c_at_B = -(temp_b * Sink.first - temp_a * Sink.second);

		if (dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_b, -temp_a, c_at_A ) + \
			dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_b, -temp_a, c_at_B ) > length_dist_point(Start, Sink))
			continue;

		// if (arr_white_list[11] && arr_white_list[4]) {
		// 	cout << "ANDDDDDDD U " << better_cand << '\n';
		// 	cout << "UHMM (" << Start.first << ','<< Start.second << ") and (" << Sink.first << ',' << Sink.second << ") and a,b,c " <<  temp_a << ' ' << temp_b << ' ' << temp_c << '\n';
		// 	cout << "1st dist " << dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_a, temp_b, temp_c ) << '\n';
		// 	cout << "A dist " << dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_b, -temp_a, c_at_A ) << '\n';
		// 	cout << "B dist " << dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_b, -temp_a, c_at_B ) << '\n';
		// 	cout << " AB dist " << length_dist_point(Start, Sink) << '\n';
		// }

		return false;
	}
	return true;
}

Point first_collide(Point Start, Point Sink, bool *missing, double player_pos[][3], double multiplier)
{
	int index_collide = -1;
	double dist_collide = 10000;

	double temp_a, temp_b, temp_c;
	get_line(Start.first, Start.second, Sink.first, Sink.second, temp_a, temp_b, temp_c);

	for (int better_cand = 0; better_cand < ROBOTS; better_cand++){
		if (missing[better_cand]) continue;

		if (dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_a, temp_b, temp_c ) > THRESHOLD_DIST * multiplier) continue;

		double c_at_A = -(temp_b * Start.first - temp_a * Start.second);
		double c_at_B = -(temp_b * Sink.first - temp_a * Sink.second);

		if (dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_b, -temp_a, c_at_A ) + \
			dist_line( player_pos[better_cand][0], player_pos[better_cand][1], temp_b, -temp_a, c_at_B ) > length_dist_point(Start, Sink))
			continue;
		if ( length_dist_vector(player_pos[better_cand][0], player_pos[better_cand][1], Start.first, Start.second) <= dist_collide){
			dist_collide = length_dist_vector(player_pos[better_cand][0], player_pos[better_cand][1], Start.first, Start.second);
			index_collide = better_cand;
		}
	}
	assert(index_collide != -1);
// THIS NOT NEARLY CORRECT
	return Point{player_pos[index_collide][0], player_pos[index_collide][1]};
}


struct TackleNode
{
	int carrier;
	Point Start, Sink;
	double speed; 
	double time_limit;

	int color; // 1 our team, -1 opp team, 0 colorless

	void get_own_time(){
	  double veA = Sink.first - Start.first;
	  double veB = Sink.second - Start.second;
	  time_limit = length_vector(veA, veB) / speed;
	}
};

#define IDLE_COMMAND Command_Pack{this_id, 0, -1000, -1000}

#define THROUGH_MODE 1
#define SHOOT_MODE 1
#define PASS_MODE 1.2
#define RIVAL_ONLY 1

double V_ROBOT = 0.75;
double V_BALL = 0.9;

struct TackleTree{
	vector <vector<int> > ListAdj ; 
	int numNode = 0;
	double MODE;
	std::vector< TackleNode> ListNode;
	TackleNode _ROOT_;

	int FirstLayer = 0, SecondLayer = 0;

	double risk_point = -1000;
	int cover_plan[ROBOTS];

	void _root(double MODE, TackleNode ROOT){
		this->MODE = MODE;
		this->numNode = 1;

		this->ListAdj.clear();
		this->ListAdj.push_back(std::vector<int>());

		this->_ROOT_ = ROOT;
		this->_ROOT_.get_own_time();

		this->ListNode.clear();
		this->ListNode.push_back(this->_ROOT_);

		fill(begin(this->cover_plan), end(this->cover_plan), -1);
	}

	void edge_tree(int DaddyID, TackleNode Child){
		this->ListNode.push_back(Child);
		this->ListAdj.push_back(std::vector<int>());
		this->ListAdj[DaddyID].push_back(this->numNode);
		this->numNode++;
	}

	pair<double, Point> straight_move(Point A, Point B, double V){
  		  double veA = B.first - A.first;
		  double veB = B.second - A.second;
		  double time_limit = length_vector(veA, veB) / V;
		  return make_pair(time_limit, B);
	}

	// ONLY GET CUT
	int get_num_risk(){
		int result = 0;
		for (int i = 1; i < this->FirstLayer; i++)
			if (ListNode[i].time_limit <= _ROOT_.time_limit && (ListNode[ i ].color != 1)) result += 1;
		return result;
	}

	void build_first_layer(bool *missing, double player_pos[][3]){
		// 1st LAYER
		for (int other_player = 0; other_player < ROBOTS; other_player++){
			if (missing[other_player]) continue;
			if (other_player == _ROOT_.carrier) continue;
			if (get_team(_ROOT_.carrier) * get_team(other_player) < 0){ // || MODE == THROUGH_MODE){

				Point oppStart = Point{player_pos[other_player][0], player_pos[other_player][1]};
				
				pair<double, Point> temp_result = cone_get_line(_ROOT_.Start, _ROOT_.Sink, _ROOT_.speed, oppStart, V_ROBOT, 1);
				pair<double, Point> move_to_goal = straight_move(oppStart, _ROOT_.Sink, V_ROBOT * this->MODE);
				// if (other_player == 11) {
				// 	cout << "CONE (" << _ROOT_.Start.first << ','<< _ROOT_.Start.second << ") and Sink (" << _ROOT_.Sink.first << "," <<_ROOT_.Sink.second << ") and V " << _ROOT_.speed << " V_opp " << V_ROBOT << '\n'; 
				// 	cout << "WTF im here " << temp_result.first << ' ' << clear_line(oppStart, temp_result.second, vector<int>{other_player, _ROOT_.carrier}, missing, player_pos) << '\n';
				// 	return;
				// }

				bool choose_A = true, choose_B = true;
				if (isnan(temp_result.first) || not clear_line(oppStart, temp_result.second, vector<int>{other_player, _ROOT_.carrier}, missing, player_pos, 3)) choose_A = false; // never connect line
				if (not clear_line(oppStart, move_to_goal.second, vector<int>{other_player, _ROOT_.carrier}, missing, player_pos, 3) ) choose_B = false; 

				if 		(not choose_A	&& not choose_B) continue;
				else if (choose_A 		&& not choose_B) ; 
				else if (not choose_A 	&& choose_B) 	 temp_result = move_to_goal; 
				else if (choose_A 		&& choose_B){
					if (BIAS_BALL) { if ( fabs(temp_result.first - move_to_goal.first) < 2 ) temp_result = move_to_goal; } // I MADE UP
					else if (GREEDY) { if (temp_result.first > move_to_goal.first) temp_result = move_to_goal;}
				} 

				// if (TACKLE_DEBUG_MODE){
				// 	printf("1st LAYER of ROOT %d create Node %d with time_cut %f and point cut %.2f %.2f\n", _ROOT_.carrier, other_player, temp_result.first, temp_result.second.first, temp_result.second.second);
				// 	// printf("Clear line check %d \n", clear_line(oppStart, temp_result.second, vector<int>{other_player, _ROOT_.carrier}, missing, player_pos));
				// }

				int _color = get_team(_ROOT_.carrier) * get_team(other_player); 
				TackleNode oppNode = TackleNode{other_player, oppStart, temp_result.second, V_ROBOT * this->MODE, temp_result.first, _color};
				edge_tree(0, oppNode);
			}
		}
		this->FirstLayer = this->numNode;
	}

	void build_second_layer(bool *missing, double player_pos[][3], int pass_receiver = -1){
		// 2nd LAYER, ONLY FOR CONFLICTING
		for (int first_gen_node = 1; first_gen_node < this->FirstLayer; first_gen_node++){
			if (ListNode[ first_gen_node ].color == 1) continue;
			// ONLY TAKE POINT
			if (ListNode[ first_gen_node ].time_limit > _ROOT_.time_limit) {
				continue;
			}
			// BUILD A NEW LAYER FOR COUNTER 
			for (int second_gen = 0; second_gen < ROBOTS; second_gen++){
				if (missing[second_gen]) continue;
				if (get_team( ListNode[first_gen_node] .carrier) * get_team(second_gen) > 0) continue;
				if (second_gen == _ROOT_.carrier) continue;
				if (second_gen == pass_receiver) continue;
				// CREATE NODE

				Point guardStart = Point{player_pos[second_gen][0], player_pos[second_gen][1]};

				// ABIDE LARGER TIME COUNTER WITH NO_LIMIT
				pair<double, Point> temp_result = cone_get_line(ListNode[first_gen_node].Start, ListNode[first_gen_node].Sink, ListNode[first_gen_node].speed, guardStart, V_ROBOT, 0);
				if (isnan(temp_result.first)) continue; // never connect line

	// MAYBE HERE NOT NEED TO BE BLOCKED, OR CHECK BLOCKED WITH ONLY OPP
				if  (not clear_line(guardStart, temp_result.second, vector<int>{second_gen, _ROOT_.carrier, ListNode[first_gen_node] .carrier}, missing, player_pos, 3) ) continue; // have better choice
				
				int _color = get_team(ListNode[first_gen_node].carrier) * get_team(second_gen); 
				TackleNode guardNode = TackleNode{second_gen, guardStart, temp_result.second, V_ROBOT, temp_result.first, _color};
				edge_tree(first_gen_node, guardNode);
			}
		}

		this -> SecondLayer = this -> numNode;
	}

	#define T_MAX_ROTATE 2.3 // second
	#define T_MAX_TRAVEL 12 //  second

	double F_T(double t){
		if (t * 3 >= T_MAX_TRAVEL){
			return t/3;
		}
		else return 0;
	}
// PLACEHOLDER
	double f_risk_future(double T_org, double T_counter) {
		if (T_org <= T_counter)
		{
			// double T_prep = T_MAX_ROTATE + F_T(T_org);
			// if (T_counter >= T_org + T_prep) return 0;
			// else return 100 * (1 - (T_counter - T_org) / T_prep);
			return 100*(2-T_counter/T_org);
		}
		else {
			return 200 * (T_org/T_counter);
		}
	}
// PLACEHOLDER
	double f_risk_cover(double T_org, double T_counter, double T_guard){
		assert(T_counter < T_org);
		if (T_guard <= T_counter)
		{
			return 30*(T_guard / T_org) + 30 * (T_guard/T_counter) + 30 * (T_counter / T_org) + f_risk_only(T_org);
		}
		else {
			return 150*(T_guard/T_counter + T_org/T_counter);
		}	// T_guard matter, but still need to count 4 case
	}

// PLACEHOLDER
	double f_risk_only(double T_org){
		if (T_org * 2 < T_MAX_TRAVEL) return 100*(T_org/T_MAX_TRAVEL);
		else return 150*(T_org/T_MAX_TRAVEL);
	}

	// STRAIGHT PASS  AND  SHOOT PASS
	double get_point_risk(){
		this->risk_point = -1000;
		// std::vector< pair<int, double> > ;
		std::vector<int > troublesome;
		for (int first_gen_node = 1; first_gen_node < this->FirstLayer; first_gen_node++){

			// FILTER ONLY TREP
			if (ListNode[ first_gen_node ].color == 1) continue;

			// if (TACKLE_DEBUG_MODE) cout << "erhm 1st layer of " << _ROOT_.carrier << " and node " << ListNode[ first_gen_node ].carrier << " time " << ListNode[ first_gen_node ].time_limit << " RISK " << f_risk_future(_ROOT_.time_limit, ListNode[ first_gen_node ].time_limit) << '\n'; 

			if (ListNode[ first_gen_node ].time_limit >= _ROOT_.time_limit) 
				this-> risk_point = MAX(this->risk_point, f_risk_future(_ROOT_.time_limit, ListNode[ first_gen_node ].time_limit) );
			else 
				troublesome.push_back(first_gen_node);
		}
		// need to alocate MAXIMUM 2 TREPASSER
		// ALLLLOOCCAAAATTTTINGGGGGG

		// TEST WITH MAXIMUM 2 TREPASSER FIRST
		// BECAREFUL OF TREPASSER WITH NO COVER
		// IF EXIST ONE AND ONLY TREPASSER, COULD TRY BOTH TACKLE AND GET_BALL ???? if TREP > TIME SO FINE
		
		assert(troublesome.size() < THRESHOLD_NUM_RISK);

		fill(begin(this->cover_plan), end(this->cover_plan), -1);
		// if (TACKLE_DEBUG_MODE) {
			// cout << " who root " << _ROOT_.carrier << " with time " << _ROOT_.time_limit << " trouble size " << troublesome.size() << '\n';
		// }

		if (troublesome.size() == 0) return MAX(this->risk_point, f_risk_only(_ROOT_.time_limit));
		else if (troublesome.size() == 1){
			int u = troublesome[0];
			double optimum_arrangment = f_risk_future( _ROOT_.time_limit, ListNode[u].time_limit );
			int best_guard = -1;
			if (TACKLE_DEBUG_MODE){
				printf("uhh root %d guard %d with adj %lld \n", _ROOT_.carrier, ListNode[u].carrier, ListAdj[u].size());
			}
			for (std::vector<int>::size_type jid = 0; jid < this->ListAdj[u].size(); jid++)
				if (optimum_arrangment > f_risk_cover(_ROOT_.time_limit, ListNode[u].time_limit, ListNode[  ListAdj[u][jid] ].time_limit))
				{
					optimum_arrangment = f_risk_cover(_ROOT_.time_limit, ListNode[u].time_limit, ListNode[  ListAdj[u][jid] ].time_limit);
					best_guard = ListAdj[u][jid]; //ListNode[  ListAdj[u][jid] ].carrier;
				}
			if (best_guard >= 0) this->cover_plan[ ListNode[best_guard].carrier ] = best_guard;
			if (TACKLE_DEBUG_MODE){
				cout << " 1TROUBLE -> " << ListNode[u].carrier << ' ' << ListNode[u].time_limit << " and guard with node " << best_guard << " robot "<< ListNode[best_guard].carrier << ' ' << ListNode[best_guard].time_limit << " and risk " << optimum_arrangment << '\n';
				cout << " 1TROUBLE TREP AT (" << ListNode[u].Sink.first << ',' << ListNode[u].Sink.second << ") \n";
				cout << " 1TROUBLE GUARD AT (" << ListNode[best_guard].Sink.first << ',' << ListNode[best_guard].Sink.second << ") \n";
			}
			this->risk_point = MAX(this->risk_point, optimum_arrangment);
		}
		else if (troublesome.size() == 2){
			int u = troublesome[0];
			int v = troublesome[1];
			double optimum_arrangment = MAX(f_risk_future( _ROOT_.time_limit, ListNode[v].time_limit ), f_risk_future( _ROOT_.time_limit, ListNode[u].time_limit ));
			pair<int, int> best_guard = make_pair(-1, -1);

			for (std::vector<int>::size_type jid = 0; jid < this->ListAdj[u].size(); jid++)
				for (std::vector<int>::size_type kid = 0; kid < this->ListAdj[v].size(); kid++){
					if ( ListNode[ ListAdj[u][jid] ].carrier == ListNode[ ListAdj[v][kid] ].carrier ) continue;
					double current_arrange = MAX(f_risk_cover(_ROOT_.time_limit, ListNode[u].time_limit, ListNode[  ListAdj[u][jid] ].time_limit), \
							f_risk_cover(_ROOT_.time_limit, ListNode[v].time_limit, ListNode[  ListAdj[v][kid] ].time_limit) );
					if (optimum_arrangment > current_arrange){
						optimum_arrangment = current_arrange;
						best_guard = make_pair( ListAdj[u][jid],  ListAdj[v][kid]  );
					}
				}
			if (TACKLE_DEBUG_MODE){
				cout << " 2TROUBLE -> " << ListNode[u].carrier << ' ' << ListNode[u].time_limit << " -> "  << ListNode[v].carrier << ' ' << ListNode[v].time_limit ;
				cout << " and guard with robot " << ListNode[best_guard.first].carrier << ' ' << ListNode[best_guard.first].time_limit << " and robot " <<  ListNode[best_guard.second].carrier << ' ' << ListNode[best_guard.second].time_limit << " and risk " << optimum_arrangment<< '\n';
			}
			if (best_guard.first >= 0 && best_guard.second >= 0) this->cover_plan[ ListNode[ best_guard.first].carrier ] = best_guard.first, this->cover_plan[ ListNode[ best_guard.second].carrier ] = best_guard.second;
			this->risk_point = MAX(this->risk_point, optimum_arrangment);
		}
		else {
			assert(False);
		}

		return this->risk_point;
	}

};

// void back_up()
// {
// 	// MOVE TO 
// }

#define UNIT_X_LENGTH 1.13

#define K_PSEUDO_REWARD 5 // reward from -0.5 -> 4.5 so -2.5 -> 22.5 on risk
#define K_SHOOT_REWARD 0.8
#define THRESHOLD_ME_PASS_RISK 80
#define THRESHOLD_ME_PASS_REWARD 110
#define MINIMUM_REACTION_TIME 1.5
#define THRESHOLD_MINIMUM_PASS_DIST MINIMUM_REACTION_TIME*V_BALL*2 //(HALF_LENGTH_LEG*2)*6

double K_shoot_dist = 20, K_ball_dist = 8;
double K_angle_goal = 35;

string get_third_field(Point query_pos, int this_id){
	double ref_goal_x = get_goal(this_id, 0).first;
	int line_x = (int) round( fabs(query_pos.first - ref_goal_x) / UNIT_X_LENGTH);
	// cout << "WTF " << line_x << ' ' << ref_goal_x << " and ? " << fabs(query_pos.first - ref_goal_x) << " and div " <<  fabs(query_pos.first - ref_goal_x) / UNIT_X_LENGTH << '\n';
	if (line_x <= 4) return "ATK";
	else if (line_x <= 9) return "MID";
	else return "DEF";
}

TackleTree create_tree_1dep(int TREE_MODE, int query_id, Point query_start_pos, Point query_dest_pos, double root_speed,  bool *missing, double player_pos[][3]){
	TackleTree c_tree;
	c_tree._root(TREE_MODE, TackleNode{query_id, query_start_pos, query_dest_pos, root_speed, INF, 1});
	c_tree.build_first_layer(missing, player_pos);
	return c_tree;
}

// Using query_id as mask and team indicate; while using query_pos as calculate
pair<int, pair<double, int> > get_direct_shoot_risk(Point query_pos, bool *missing, double player_pos[][3], int query_id){
	int best_id_goal = -1;
	double best_goal_risk = 10000;
	int chance_create = 0;

	for (int i = 0; i < NUM_GOAL_OPTION; i++){
		TackleTree me_goal_tree = create_tree_1dep(SHOOT_MODE, query_id, query_pos, get_goal(query_id, i), V_BALL*3, missing, player_pos);
		if (me_goal_tree.get_num_risk() >= THRESHOLD_NUM_RISK) continue;
		double this_risk = me_goal_tree.get_point_risk();
		// if (ATTACK_DEBUG_MODE) cout << " 		BALL HOLDER CHECK GOAL risk " << me_goal_tree.get_num_risk() << " id " << i << " risk " << this_risk << '\n';

		if (this_risk > THRESHOLD_SHOOT_RISK) continue;
		chance_create++;
		if (best_goal_risk > this_risk) best_goal_risk = this_risk, best_id_goal = i;
	}
	return make_pair(chance_create, make_pair(best_goal_risk, best_id_goal));
}

double get_ball_distance_reward(Point ball_pos, Point query_pos){
	double diff_pos = length_dist_point(ball_pos, query_pos) - THRESHOLD_MINIMUM_PASS_DIST; // 0->2*sqrt(2)*bound_y-thresh
	if (diff_pos >= 0) return MIN(+K_ball_dist*fabs(diff_pos / THRESHOLD_MINIMUM_PASS_DIST), 35); // 0-> 20
	else return -60 - 10*(THRESHOLD_MINIMUM_PASS_DIST / length_dist_point(ball_pos, query_pos)); // 0->1
}

double get_goal_distance_reward(int query_id, Point query_pos){
	double diff_pos = length_dist_point(query_pos, get_goal(query_id, 0)) - THRESHOLD_SHOOT_DIST; // 0->2*sqrt(2)*bound_y-thresh
	if (diff_pos >= 0) return MAX(-30,-K_shoot_dist*fabs(diff_pos / THRESHOLD_SHOOT_DIST)); //0->3
	else return MIN(+K_shoot_dist*fabs(diff_pos)/THRESHOLD_SHOOT_DIST, 40); // 0->1
}

double get_combine_shoot_reward(Point query_pos, bool *missing, double player_pos[][3], int query_id){
	double total_reward = get_goal_distance_reward(query_id, query_pos);

	pair<int, pair<double, int>> get_risk = get_direct_shoot_risk(query_pos, missing, player_pos, query_id);

	if (get_risk.first > 0) total_reward += K_SHOOT_REWARD *(THRESHOLD_SHOOT_RISK - get_risk.second.first);
	else total_reward -= 15; // no shoot available

	if (get_risk.first > 1) total_reward += K_angle_goal*(get_risk.first/NUM_GOAL_OPTION); // 0->1

	return total_reward;
}

// COULD INCLUDE ACTUAL SITUATION LATER
float get_offball_reward(Point query_pos, bool *missing, double player_pos[][3], double *ball_pos, int query_id, double multiplier){
	// for all_other robot;
	double reward = MAX_MASK_DIR;
	bool mark[MAX_MASK_DIR] = {0};

	for (int i = 0; i < ROBOTS; i++){
		if (missing[i] || i == query_id) continue;
		if (length_dist_point(query_pos, Point{player_pos[i][0], player_pos[i][1]}) > 5*multiplier* THRESHOLD_DIST) continue;
		double account_angle = atan2( player_pos[i][1] - query_pos.second,  player_pos[i][0] - query_pos.first );
		mark[get_mask_dir(account_angle)] = 1;
	}
	for (int i = 0; i < MAX_MASK_DIR; i++)
		reward -= mark[i];

	reward *= K_PSEUDO_REWARD;

	reward += get_ball_distance_reward( Point{ball_pos[0], ball_pos[1]}, query_pos);

	return reward;
}

double get_passball_reward(Point query_start_pos, Point query_dest_pos, double root_speed, bool *missing, double player_pos[][3], int query_id){
	TackleTree c_tree = create_tree_1dep(PASS_MODE, query_id, query_start_pos, query_dest_pos, root_speed, missing, player_pos);
	if (c_tree.get_num_risk() >= THRESHOLD_NUM_RISK) return -40;
	return THRESHOLD_ME_PASS_RISK - c_tree.get_point_risk();
}

double get_block_shoot_reward(Point itr, bool *missing, double player_pos[][3], double *ball_pos, int query_id, int ball_holder){
	string ball_where = get_third_field(Point{ball_pos[0], ball_pos[1]}, query_id);
	string me_where = get_third_field(itr, query_id);
	if (ball_where == "ATK"){
		if (me_where == "DEF" && get_team(query_id) * get_team(ball_holder) > 0) 
			return get_passball_reward(Point{ball_pos[0], ball_pos[1]}, itr, V_BALL*1.5, missing, player_pos, ball_holder);
		else return -10;
	}
	else{ // normally called dis function where we have ball, if opp have ball then call switch_to_defense

		Point before = {player_pos[query_id][0], player_pos[query_id][1]};
		player_pos[query_id][0] = itr.first;
		player_pos[query_id][1] = itr.second;
		double Return_value = -get_combine_shoot_reward(Point{ball_pos[0], ball_pos[1]}, missing, player_pos, ball_holder);
		player_pos[query_id][0] = before.first;
		player_pos[query_id][1] = before.second;
		
		return Return_value;
	}
}

Point choose_from_list_best_combine_reward(vector<Point> list_candidate, int ball_holder, bool *missing, double player_pos[][3], double *ball_pos, int query_id){
	double best_reward = -1000;
	Point chosen_point = {-1000, -1000};

	for (auto itr: list_candidate){
		double future_reward = get_offball_reward(itr, missing, player_pos, ball_pos, query_id, 3);

		if (query_id == 0 || query_id == 7)
			future_reward += get_block_shoot_reward(itr, missing, player_pos, ball_pos, query_id, ball_holder);
		else if (4 <= query_id && query_id <= 6)
			future_reward += get_combine_shoot_reward(itr, missing, player_pos, query_id);
		else if (11 <= query_id && query_id <= 13)
			future_reward += get_combine_shoot_reward(itr, missing, player_pos, query_id);
		else
			future_reward += get_passball_reward(Point{ball_pos[0], ball_pos[1]}, itr, V_BALL*1.5, missing, player_pos, ball_holder);

		if (future_reward > best_reward){
			best_reward = future_reward;
			chosen_point = itr;
		}
	}
	if (ROLE_DEBUG_MODE) cout << "REWARD ROLE ACTING" << " new reward " << best_reward << " chose " << chosen_point.first << ' ' << chosen_point.second << '\n';
	return chosen_point;
}

Point get_list_point_furthest(vector<Point> list_candidate, double ref_x){
	double best_dist = -1000;
	Point chosen_point = {-1000, -1000};

	for (auto itr: list_candidate)
		if (best_dist < fabs(itr.first - ref_x)){
			best_dist = fabs(itr.first - ref_x);
			chosen_point = itr;
		}
	return chosen_point;
}

int get_ball_holder(int *player_ball){
	for (int i = 0; i < ROBOTS; i++)
		if (player_ball[i] > 0) return i;
	return -1;
}

int get_closest_robot_on_team(Point ref_point, bool *missing, double player_pos[][3], int query_id){
	int closest_robot = -1;
	for (int i = 0; i < ROBOTS; i++){
		if (missing[i]) continue;
		if (query_id != -1 && (get_team(i) * get_team(query_id) > 0) ) continue;// only find opponent, and specify player_team
		if (closest_robot == -1 || length_dist_point( Point{player_pos[i][0], player_pos[i][1]}, ref_point) < \
			length_dist_point( Point{player_pos[closest_robot][0], player_pos[closest_robot][1]}, ref_point) )
			closest_robot = i;
	}
	return closest_robot;
}
int get_closest_robot_off_team(Point ref_point, bool *missing, double player_pos[][3], int query_id){
	int closest_robot = -1;
	for (int i = 0; i < ROBOTS; i++){
		if (missing[i]) continue;
		if (query_id != -1 && (get_team(i) * get_team(query_id) < 0) ) continue;// only find opponent, and specify player_team
		if (closest_robot == -1 || length_dist_point( Point{player_pos[i][0], player_pos[i][1]}, ref_point) < \
			length_dist_point( Point{player_pos[closest_robot][0], player_pos[closest_robot][1]}, ref_point) )
			closest_robot = i;
	}
	return closest_robot;
}


// SCHEDULE 15/5
// F<- position, role_id, => attack/defense
// rnow, current situation always >= 0
Command_Pack role_acting(int brain_level, bool *missing, double player_pos[][3], int *player_ball, int *player_state, double *ball_pos, int this_id, int current_situation)
{
	int ball_holder = -1;
	for (int i = 0; i < ROBOTS; i++)
		if (player_ball[i] > 0) ball_holder = i;

	string pos_on_which_part = get_third_field(Point{player_pos[this_id][0], player_pos[this_id][1]}, this_id);
	if (ROLE_DEBUG_MODE){cout << "ROLE ACTING on " << this_id << " current role " << pos_on_which_part << '\n';		}

	int who_closest_me_now = get_closest_robot_on_team(Point{player_pos[this_id][0], player_pos[this_id][1]}, missing, player_pos, this_id);
	// SHOULD BE brain_level here
	if (brain_level == 1 || brain_level == 3){

		if (current_situation >= 0){
			vector<Point> possible_candidate;
			for (int dir = 0; dir < MAX_DIR_ID; dir++)
				possible_candidate.push_back( get_move_to_dir(Point{player_pos[this_id][0], player_pos[this_id][1]}, dir) );

			Point chosen_point= choose_from_list_best_combine_reward(possible_candidate, ball_holder, missing, player_pos, ball_pos, this_id);
			return Command_Pack{this_id, 3, chosen_point.first, chosen_point.second};
		}
		else{
			// if (length_dist_vector(player_pos[who_closest_me_now][0], player_pos[who_closest_me_now][1], player_pos[this_id][0], player_pos[this_id][1]) < HALF_LENGTH_LEG*2*4)
				// return Command_Pack{this_id, 5, double(who_closest_me_now), -1000}; 
			return IDLE_COMMAND;
		}
	}
	else if (brain_level == 2){ // ALL ATTACKING, stupid need to fix
		return Command_Pack{this_id, 5, double(who_closest_me_now), -1000}; 
	}
	else {
		assert(false);
		return IDLE_COMMAND;
	}
}

// INVEST MORE, SHOULD BE AFTER IMPLEMENT ATTACKING
Command_Pack switch_to_defense(int brain_level, bool *missing, double player_pos[][3], int *player_ball, int *player_state, double *ball_pos, int this_id, int ball_holder = -1){
	if (ball_holder == -1) {
		for (int i = 0; i < ROBOTS; i++)
			if (player_ball[i] > 0) ball_holder = i;
	}
	assert(get_team(this_id)*get_team(ball_holder)<0);
	if (brain_level == BRAIN_LEVEL_0){
		return Command_Pack{this_id, 8, -1000, -1000};
	}
	else if (brain_level == BRAIN_LEVEL_1){
		// auto_call_block_goal

		string pos_on_which_part = get_third_field(Point{player_pos[this_id][0], player_pos[this_id][1]}, this_id);
		if (pos_on_which_part == "DEF")
			return Command_Pack{this_id, 8, -1000, -1000};
		else return Command_Pack{this_id, 5, double(ball_holder), -1000};
	}
	else if (brain_level == BRAIN_LEVEL_2){
		int who_closest_me_now = get_closest_robot_on_team(Point{player_pos[this_id][0], player_pos[this_id][1]}, missing, player_pos, this_id);

		if (length_dist_vector(ball_pos[0], ball_pos[1], player_pos[this_id][0], player_pos[this_id][1]) > HALF_LENGTH_LEG * 2 * 6) return Command_Pack{this_id, 5, double(who_closest_me_now), -1000};
		else return Command_Pack{this_id, 8, -1000, -1000};
	}
	else if (brain_level == BRAIN_LEVEL_3){
		if (this_id == 0 || this_id == 7){
			pair<int, pair<double, int>> get_risk = get_direct_shoot_risk(Point{ball_pos[0], ball_pos[1]}, missing, player_pos, ball_holder);
			if (get_risk.first == 0) return IDLE_COMMAND;
			else {
				Point risky_goal_point = get_goal(ball_holder, get_risk.second.second);
				return Command_Pack{this_id, 3, risky_goal_point.first, risky_goal_point.second};
			}
		}	
		else if (this_id < 4 || (7< this_id && this_id < 11)){
			Point best_point = Point{player_pos[this_id][0], player_pos[this_id][1]};
			double best_rew = -1000;
			for (int dir = 0; dir < MAX_DIR_ID; dir++){
				Point test_point = get_move_to_dir(Point{player_pos[this_id][0], player_pos[this_id][1]}, dir);
				double future_reward = get_block_shoot_reward(test_point, missing, player_pos, ball_pos, this_id, ball_holder);
				if (future_reward > best_rew)
					best_rew = future_reward,
					best_point = test_point;
			}
			return Command_Pack{this_id, 3, best_point.first, best_point.second};
		}
		else
			return Command_Pack{this_id, 8, -1000, -1000};

		// string pos_on_which_part = get_third_field(Point{player_pos[this_id][0], player_pos[this_id][1]}, this_id);

		// Point meStart = Point{player_pos[this_id][0], player_pos[this_id][1]};
		// Point oppStart = Point{ball_pos[0], ball_pos[1]};
		// Point goalMid = get_goal(ball_holder, 0);
		
		// pair<double, Point> temp_result = cone_get_line(oppStart, goalMid, V_BALL*2, meStart, V_ROBOT*1.5, 1);

		// if (pos_on_which_part == "DEF") {
		// 	if (length_dist_point(temp_result.second, meStart) <= HALF_LENGTH_LEG*2*8)
		// 		return Command_Pack{this_id, 4, temp_result.second.first, temp_result.second.second};
		// 	else return role_acting(BRAIN_LEVEL_3, missing, player_pos, player_ball, player_state, ball_pos, this_id, -1);
		// }
		// else {
		// 	int who_closest_me_now = get_closest_robot_on_team(Point{player_pos[this_id][0], player_pos[this_id][1]}, missing, player_pos, this_id);

		// 	if (length_dist_vector(ball_pos[0], ball_pos[1], player_pos[this_id][0], player_pos[this_id][1]) <= HALF_LENGTH_LEG * 2 * 6) 
		// 		return Command_Pack{this_id, 8, -1000, -1000};
		// 	else if (length_dist_vector(player_pos[who_closest_me_now][0], player_pos[who_closest_me_now][1], player_pos[this_id][0], player_pos[this_id][1]) <= HALF_LENGTH_LEG * 2 * 10) 
		// 		return Command_Pack{this_id, 5, double(who_closest_me_now), -1000};
		// 	else return role_acting(BRAIN_LEVEL_3, missing, player_pos, player_ball, player_state, ball_pos, this_id, -1);
		// }
	}
	else {
		throw string("Unknown defense level\n");
	}
}

struct Attack_Pack {
	unsigned int time_step = 0;

	double best_pass_reward = -2000;
	TackleTree best_ball_tree;
	Point best_field_point = {-1000, -1000};
	int best_rec;
} attack_pack;

// MAYBE ADD A DICT/MARKER SO OTHER ROBOT DONT NEED TO CALCULATE AGAIN, BY ADDING TIMESTEP
Command_Pack switch_to_attack(unsigned int time_step_now, int brain_level, bool *missing, double player_pos[][3], int *player_ball, int *player_state, double *ball_pos, int this_id){
	int ball_holder = get_ball_holder(player_ball);
	assert ( get_team(this_id) * get_team(ball_holder) > 0 );
	Point ball_point = Point{ball_pos[0], ball_pos[1]};

	int BALL_HOLDER_IN_STUCK = 0;
	for(int player = 0; player < ROBOTS; player++)
		if (!missing[player] && player != ball_holder && get_team(player)*get_team(ball_holder) < 0 && length_dist_vector(player_pos[player][0], player_pos[player][1], ball_pos[0], ball_pos[1]) < HALF_LENGTH_LEG*3)
			BALL_HOLDER_IN_STUCK = 1;

	pair<int, pair<double, int>> shoot_risk = get_direct_shoot_risk(ball_point, missing, player_pos, ball_holder);
	double shoot_reward = get_combine_shoot_reward(ball_point, missing, player_pos, ball_holder);
	if (ATTACK_DEBUG_MODE && this_id == ball_holder){ cout << " BALL HOLDER CHECK SHOOT chance " << shoot_risk.first << " reward " << shoot_reward << " id " << shoot_risk.second.second << " goal rew " << get_goal_distance_reward(ball_holder, ball_point) << '\n'; }

	if (shoot_risk.first > 0 && shoot_reward > THRESHOLD_SHOOT_REWARD) {
		if (this_id == ball_holder) {
			// if (BALL_HOLDER_IN_STUCK) return Command_Pack{this_id, 6, 2, -1000};
			// else
			return Command_Pack{this_id, 2, double(shoot_risk.second.second), -1000};
		}
		else return role_acting(brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id, 1);
	}
	else{ // CAN NOT SHOOT, HAVE TO PASS
		TackleTree best_ball_tree; Point best_field_point = {-1000, -1000}; int best_rec = -1;
		double best_pass_reward = -1000;

		if (time_step_now > attack_pack.time_step){

			bool boundary_near = (time_step_now == attack_pack.time_step + 1);

			TackleTree me_pass_tree[ROBOTS];

			std::vector< pair<double, int> > possible_pass_candidate;

	// brute first, but NEED TO BIAS OVER PREVIOUSLY TO AVOID BIG JUMP -> STAGGERING
	// NEED TO CONSIDER TASK and FORFEIT TASK
			for (int gid = 0; gid < GRIDIFY_FIELD; gid++){
				Point field_pos= Point{grid_x[gid], grid_y[gid]};

				if (!PAPER_ATTAK_MODE && boundary_near && length_dist_point(field_pos, attack_pack.best_field_point) > 5) continue;
				// check if closest team robot exist

				int closest_robot = get_closest_robot_off_team(field_pos, missing, player_pos, ball_holder);
				// if (closest_robot == ball_holder) continue;
				// if (get_team(ball_holder) * get_team(closest_robot) < 0) continue;
				double receiver_attack_reward = get_combine_shoot_reward( field_pos, missing, player_pos, closest_robot);
				double receiver_offball_reward = get_offball_reward(field_pos, missing, player_pos, ball_pos, closest_robot, 3);

			

				// check if ball safe
				double ball_traj_reward;
				TackleTree ball_traj_tree = create_tree_1dep(PASS_MODE, ball_holder, ball_point, field_pos, V_BALL*1.5, missing, player_pos);
				if (ball_traj_tree.get_num_risk() >= THRESHOLD_NUM_RISK) continue; // ball_traj_reward = -10000;
				else {
					ball_traj_tree.build_second_layer(missing, player_pos, closest_robot);
					ball_traj_reward = THRESHOLD_ME_PASS_RISK - ball_traj_tree.get_point_risk();
				}
				// check if trajectory from now to field_pos fine

				double receiver_traj_reward;
				TackleTree receiver_traj_tree = create_tree_1dep(PASS_MODE, closest_robot, Point{player_pos[closest_robot][0], player_pos[closest_robot][1]}, field_pos, V_ROBOT, missing, player_pos);
				if (receiver_traj_tree.get_num_risk() >= THRESHOLD_NUM_RISK) continue;// receiver_traj_reward = -10000;
				else receiver_traj_reward = THRESHOLD_ME_PASS_RISK - receiver_traj_tree.get_point_risk();

				double total_attack_point = ball_traj_reward + receiver_offball_reward + receiver_attack_reward + receiver_traj_reward;

				if (PAPER_ATTAK_MODE) {
					string reward = std::to_string(gid) + " " + std::to_string(ball_traj_reward) + " " + std::to_string(receiver_offball_reward) +\
					 " " + std::to_string(receiver_traj_reward) + " " + std::to_string(receiver_attack_reward) + "\n";
					log_to_file("reward", reward);
				}
				if (BALL_HOLDER_IN_STUCK) total_attack_point -= receiver_attack_reward;
	// SLACK TIME

				// double ref_dist = length_dist_point(field_pos, Point{player_pos[8][0], player_pos[8][1]});
				// if (ATTACK_DEBUG_MODE && ref_dist < 2) cout << "PASS MODE cal with CB have reward  " << total_attack_point << " at point " << field_pos.first << " " << field_pos.second << " with component ball_traj_rew " << ball_traj_reward << " ball_call " << ball_traj_tree.get_num_risk() << " rec_offb_rew " << receiver_offball_reward << " rec_atk_rew " << receiver_attack_reward << " rec_traj_rew " << receiver_traj_reward  << '\n';

				if (ball_traj_reward < 20) continue;
				if (total_attack_point > best_pass_reward){
					best_pass_reward = total_attack_point;
					best_ball_tree = ball_traj_tree;
					best_field_point = field_pos;
					best_rec = closest_robot;
				}
			}
			if (PAPER_ATTAK_MODE) log_to_file("reward", "end\n\n\n");

			for (int rid = 0; rid < ROBOTS; rid++){
				if (missing[rid]) continue;
				string position = std::to_string(rid) + " " + std::to_string(player_pos[rid][0]) + " " + std::to_string(player_pos[rid][1]) + "\n";
				if (PAPER_ATTAK_MODE) log_to_file("position", position);
			}
			if (PAPER_ATTAK_MODE) log_to_file("position", "end\n\n\n");
			if (ATTACK_DEBUG_MODE) cout << "PASS MODE chose PASS_>id " << best_rec << " with t_reward  " << best_pass_reward << " at point " << best_field_point.first << " " << best_field_point.second << \
			 " with component best_ball_risk " << best_ball_tree.get_point_risk() << " ball_call " << best_ball_tree.get_num_risk() << '\n';

			attack_pack = Attack_Pack{time_step_now, best_pass_reward, best_ball_tree, best_field_point, best_rec};
		}
		else{
			best_pass_reward = attack_pack.best_pass_reward;
			best_ball_tree = attack_pack.best_ball_tree;
			best_field_point = attack_pack.best_field_point;
			best_rec = attack_pack.best_rec;
		}


		if (best_pass_reward < THRESHOLD_ME_PASS_REWARD){ // no option
			// must dribble 
			if (this_id == ball_holder) return Command_Pack{this_id, 6, 2, -1000};
			else return role_acting(brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id, 1);
			 // throw string("DONW KNOW HOW TO DO \n");
		}
		else{

			// cout << "best ball tree cal by " << this_id << " plan here \n require ";
			// for (int i = 0; i < ROBOTS; i++)
			// 	if (best_ball_tree.cover_plan[i] >= 0) cout << "ROB " << i << " ";
			// cout << '\n';

			if (this_id == ball_holder){ 
				// if (ATTACK_DEBUG_MODE && this_id == ball_holder){ cout << "ATTACK MODE chose PASS_>id " << best_rec << " with reward  " << best_pass_reward<< '\n'; }

				if (BALL_HOLDER_IN_STUCK && best_ball_tree.get_point_risk() > 0) return Command_Pack{this_id, 6, 2, -1000};
				else 
				return Command_Pack{this_id, 9, best_field_point.first, best_field_point.second};
			}
			else if (this_id == best_rec) {
				// if (ATTACK_DEBUG_MODE && this_id == best_rec){ cout << "ATTACK MODE implicit pass RECEIVED>id " << best_rec << '\n'; }
				return Command_Pack{this_id, 14, best_field_point.first, best_field_point.second};
			}
			else if (best_ball_tree.cover_plan[this_id] >= 0) {
				int nID = best_ball_tree.cover_plan[this_id];
				Point Sink = best_ball_tree.ListNode[nID].Sink;
				return Command_Pack{this_id, 13, Sink.first, Sink.second}; // cover_plan default -1
			}
			else return role_acting(brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id, 1);
		}


	}
}

// #define FRICTION 0.2

Point predicted(double *ball_pos, Point ball_moving_direction, double ball_velo)
{	
  // ball_velo; ball_moving_direction, ball_possesion; => ball_final_pos	
  // double friction = 32/(1000*FRICTION); // NEED  TO BE CALCULATE BY HAND	

  double delta_ball_dir = length_vector(ball_moving_direction.first, ball_moving_direction.second);	
  double travel_distance = ( (ball_velo - 0.55)/0.13 + 1.74  )/delta_ball_dir;	
  // printf("BALL TRAVEL_DIST %f and BALL_VELO %f\n", travel_distance, ball_velo);	
  // if (fabs(ball_velo) < 0.0001) travel_distance = 0.0001; 	
  if (isnan(travel_distance)) travel_distance = 0.0001;  	
  return Point{ball_pos[0] + ball_moving_direction.first*travel_distance, ball_pos[1] + ball_moving_direction.second*travel_distance};
}

//  NOT SO SURE
Command_Pack free_ball_strategy(int brain_level, bool *missing, double player_pos[][3], int *player_ball, int *player_state, double *ball_pos, int this_id, Point ball_moving_direction, double ball_velo){
	
	Point predicted_ball = predicted(ball_pos, ball_moving_direction, ball_velo);
	bool no_player_near_ball = clear_line(Point{ball_pos[0], ball_pos[1]}, predicted_ball, vector<int>{}, missing, player_pos, 3);

	// if (ball_velo > V_ROBOT && no_player_near_ball) ball_velo = ball_velo;
	// else ball_velo = 0;
	// predicted_ball = predicted(ball_pos, ball_moving_direction, ball_velo);

	if (!no_player_near_ball)
		predicted_ball = first_collide(Point{ball_pos[0], ball_pos[1]}, predicted_ball, missing, player_pos, 3);
	// cout << predicted_ball.first << ' ' << predicted_ball.second << ' ' << ball_velo << '\n';

	int best_teamate = -1;
	double best_risk = 1000;
	TackleTree tactic_tree[ROBOTS];
	// BUILD TREE HERE find the best teamate among us

	for (int player = 0; player < ROBOTS; player++){

		if (missing[player]) continue;
		if (get_team(this_id) * get_team(player) < 0) continue;
		// tactic_tree[player]._root(RIVAL_ONLY, TackleNode{player, Point{player_pos[player][0], player_pos[player][1]}, Point{ball_pos[0], ball_pos[1]}, V_ROBOT, INF, 1});
		tactic_tree[player] = create_tree_1dep(RIVAL_ONLY, player, Point{player_pos[player][0], player_pos[player][1]}, predicted_ball, V_ROBOT, missing, player_pos);
		if (tactic_tree[player].get_num_risk() >= THRESHOLD_NUM_RISK) continue;

		tactic_tree[player].build_second_layer(missing, player_pos);
		double this_risk = tactic_tree[player].get_point_risk();
		assert(this_risk != -1000);
		// MAYBE MAKE RISKPOINT NEGATIVE SO COULD SORT DESCENDING
		if (this_risk < best_risk)
			best_risk = this_risk,
			best_teamate = player;
	}

	if (STRATEGY_DEBUG_MODE && this_id == 8){ 
		cout << "FREE BALL best teamate " << best_teamate << " best risk " << best_risk << '\n';	
		cout << "critical: id 7 " << tactic_tree[7].get_point_risk() <<  " id 8 " << tactic_tree[8].get_point_risk() << " id 11 " << tactic_tree[11].get_point_risk() << '\n';
	}

	if (best_risk > THRESHOLD_RISK){ // YABAI
// NORMAL ALGORITHM -> TRY TO GET BALL by GET CLOSER TO JUNCTION, WHICH RESULTED IN COMPLETE FAULTY
//					 											, BUT ACTUALLY OPP TEAMATE WILL BLOCKED DEFENDER and MAKE ROOM OF MOVEMENT FOR HOLDER
// SO A BETTER CHOICE is to switch as if defense
// COULD ALSO ACT AS OPP and know who will get ball, but simplified coz nostromos nunca get ball, so could predict

		int who_closest_ball_then = get_closest_robot_on_team(predicted_ball, missing, player_pos, -1);
		// cout << " CHECK ASSERT " << who_closest_ball_then << " and mul " << get_team(this_id) * get_team(who_closest_ball_then) << '\n';
		// check assert
		// assert(get_team(this_id) * get_team(who_closest_ball_then) < 0);
		if (this_id == who_closest_ball_then) return Command_Pack{this_id, 8, -1000, -1000};
		else
			return switch_to_defense(brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id, who_closest_ball_then);
	}
	else{ // get the least risk teamate
// optional
		if (this_id == best_teamate) return Command_Pack{this_id, 8, -1000, -1000};
		else if (tactic_tree[best_teamate].cover_plan[this_id] >= 0) {
			int nID = tactic_tree[best_teamate].cover_plan[this_id];
			Point Sink = tactic_tree[best_teamate].ListNode[nID].Sink;
			return Command_Pack{this_id, 13, Sink.first, Sink.second}; // cover_plan default -1
		}
		else return role_acting(brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id, 0);
	}
}

Command_Pack change_mode(unsigned int time_step_now, int brain_level, bool *missing, double player_pos[][3], int *player_ball, int *player_state, double *ball_pos, int this_id, Point ball_moving_direction, double ball_velo){
// easy first, decision here, high_level

	int whose_team_ball = get_team_possesion(player_ball);
	// if (TACKLE_DEBUG_MODE)
	// 	cout << "whose_team_ball " << whose_team_ball << " this_team " << get_team(this_id) << " this_id " << this_id << '\n';

	if (whose_team_ball == 404) // UH OH, SELF CONFLICT
	{
		// for (int qq = 0; qq < 10; qq++)
		// 	printf("DAMNED CONGESTION UP\n");

		return IDLE_COMMAND;
		// if (brain_level == BRAIN_LEVEL_1) return IDLE_COMMAND;
		// else if (brain_level == BRAIN_LEVEL_2) return Command_Pack{this_id, 10, -1000, -1000};
		// else throw string("Unknown brain_level");
	}
	else if (abs(whose_team_ball)>2){ // self-conflict
		// conflict_strategy(missing, player_pos, player_ball, player_state, ball_pos, this_id);

		return IDLE_COMMAND;
		// if (brain_level == BRAIN_LEVEL_1) return IDLE_COMMAND;
		// else if (brain_level == BRAIN_LEVEL_2) return Command_Pack{this_id, 10, -1000, -1000};
		// else throw string("Unknown brain_level");
	}
	else if (whose_team_ball == 0) // ball free SCHEDULED -> 05/05
	{
		return free_ball_strategy(brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id, ball_moving_direction, ball_velo);
	}
	else if (get_team(this_id) * whose_team_ball > 0) {// team get ball
		return switch_to_attack(time_step_now, brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id);
	}
	else if (get_team(this_id) * whose_team_ball < 0){ // opponent get ball 
		return switch_to_defense(brain_level, missing, player_pos, player_ball, player_state, ball_pos, this_id);
	}
	else { 
		throw string("Unknown change mode situation");
		return IDLE_COMMAND;
	}

}

#endif // BASIC STRATEGY