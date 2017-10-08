#include "vehicle.h"
#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include "common.h"
#include "jmt.h"
#include "spline.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(double v_ref, double v_max, double a_max, size_t history_size) {
    this->v_ref = v_ref;
    this->v_max = v_max;
    this->a_max = a_max;
    this->history_size = history_size;
}

Vehicle::~Vehicle() {}

vector<double> Vehicle::LeaderState(const vector<vector<double> > &others,
                                    const Road &road,
                                    int lane) const {
    vector<double> leader{};

    if ((road.DToLane(D()) + lane) < 0) {
        return leader;
    }

    double min_s = 100000.;
    for (auto other : others) {
        if (road.DToLane(other[6]) == (road.DToLane(D()) + lane) &&
            other[5] > S() &&
            other[5] - S() < min_s) {
            leader.clear();
            copy(other.begin(), other.end(), back_inserter(leader));
            min_s = other[5] - S();
        }
    }

    return leader;
}

vector<double> Vehicle::FollowerState(const vector<vector<double> > &others,
                                      const Road &road,
                                      int lane) const {
    vector<double> follower{};

    if ((road.DToLane(D()) + lane) < 0) {
        return follower;
    }

    double max_s = -100000.;
    for (auto other : others) {
        if (road.DToLane(other[6]) == (road.DToLane(D()) + lane) &&
            other[5] < S() &&
            other[5] - S() > max_s) {
            follower.clear();
            copy(other.begin(), other.end(), back_inserter(follower));
            max_s = other[5] - S();
        }
    }

    return follower;
}

double Vehicle::PreferredBuffer() const {
    return std::max(V(), 30.);
}

bool Vehicle::RequireAttention(const vector<double> &other_state) const {
    if (!other_state.empty()) {
        return (abs(other_state[5] - S()) < PreferredBuffer());
    } else {
        return false;
    }
}
/*
double Vehicle::TimeToCatchUpLeader(vector<double> leader_state) const {
    double v_leader = sqrt(leader_state[3] * leader_state[3]
                           + leader_state[4] * leader_state[4]);

    if (V() <= mph2mps(v_leader)) {
        return -1.0;
    }

    double s_diff = leader_state[5] - S();// - PreferredBuffer();
    return (s_diff / (V() - mph2mps(v_leader)));
}
*/
vector<vector<double> > Vehicle::CalcTrajectory(const vector<vector<double> > &others,
                                                const Road &road) {
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<int> lanes = {-1, 0, 1};
    double cost = 10000;
    vector<vector<double> > traj;
    double new_v;
    std::cout << "*********************************" << std::endl;

    for (auto l : lanes) {
        if (((road.DToLane(D()) + l) < 0) ||
            ((road.DToLane(D()) + l) >= road.num_lane)) {
            continue;
        }

        double v = v_ref;
        auto leader = LeaderState(others, road, l);
        auto follower = FollowerState(others, road, l);

        if (!leader.empty()) {
            std::cout << l << ": #: " << leader[0] << ", diff_s: " << abs(leader[5] - S()) << ", ?: " << (abs(leader[5] - S()) < PreferredBuffer())  << std::endl;
        } else {
            std::cout << l << " clear" << std::endl;
        }

        if (RequireAttention(leader)) {
            std::cout << l << ": -----------" << v << std::endl;
            //this->v -= this->a_max;
            double leader_v = sqrt(leader[3]*leader[3]
                                   + leader[4]*leader[4]);
            if (v > leader_v) {
                if ((v - leader_v) < a_max) {
                    v = leader_v;
                } else {
                    v -= a_max;
                }
            } else {
                v += a_max;
                if (v > leader_v) {
                    v = leader_v;
                }
            }
        } else if (v < v_max) {
            std::cout << l << ": >>>>>>>>>>>>>" << std::endl;
            v += a_max;
        }

        auto tss = Trajectory(v, road, l);
        double c = CalcCost(tss, leader, follower, road, l);
        if (c < cost) {
            cost = c;
            traj = tss;
            new_v = v;
        }
    }
    v_ref = new_v;

    return traj;
}

double Vehicle::CalcCost(const vector<vector<double> > &trajectory,
                         const vector<double> &leader_state,
                         const vector<double> &follower_state,
                         const Road &road,
                         int lane) {
    const int keep_lane = 0;
    double cost = 0;

    if (!leader_state.empty()) {
        /*
        Vehicle leader = Vehicle(0, 0, 0, 1);
        double psi_leader = rad2deg(atan2(leader_state[3], leader_state[4]));
        leader.Update({leader_state[1],
                        leader_state[2],
                        leader_state[5],
                        leader_state[6],
                        psi_leader},
            {}, {});
        */
        double v_leader = mph2mps(sqrt(leader_state[3] * leader_state[3]
                                       + leader_state[4] * leader_state[4]));
        /*
        auto leader_trajectory = leader.Trajectory(v_leader, road, keep_lane);
        for (size_t i=0; i < leader_trajectory[0].size(); i++) {
            std::cout << leader_trajectory[0][i] << ", " << leader_trajectory[1][i] << std::endl;
        }
        */
        if (v_leader < V()) {
            cost += std::max(-10 / (PreferredBuffer() * 3) * (leader_state[5] - S()) + 10, 0.);
        }
    }

    if (!follower_state.empty()) {
        /*
        Vehicle follower = Vehicle(0, 0, 0, 1);
        double psi_follower = atan2(follower_state[3], follower_state[4]);
        follower.Update({follower_state[1],
                        follower_state[2],
                        follower_state[5],
                        follower_state[6],
                        psi_follower},
            {}, {});
        */
        double v_follower = mph2mps(sqrt(follower_state[3] * follower_state[3]
                                         + follower_state[4] * follower_state[4]));
        //auto follower_trajectory = follower.Trajectory(v_follower, road, keep_lane);

        if (v_follower > V()) {
            cost += std::max(10 / (PreferredBuffer() * 3) * (follower_state[5] - S()) + 10, 0.);
        }
    }

    cost += abs(lane) * 2;

    return cost;
}

vector<vector<double> > Vehicle::Trajectory(double ref_vel,
                                            const Road &road,
                                            int lane) {
    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = X();
    double ref_y = Y();
    double ref_yaw = deg2rad(Psi());

    if (previous_path_x.size() < 2) {
        double prev_car_x = X() - cos(deg2rad(Psi()));
        double prev_car_y = Y() - sin(deg2rad(Psi()));

        ptsx.push_back(prev_car_x);
        ptsx.push_back(X());

        ptsy.push_back(prev_car_y);
        ptsy.push_back(Y());
    } else {
        ref_x = previous_path_x.back();
        ref_y = previous_path_y.back();

        double ref_x_prev = previous_path_x.end()[-2];
        double ref_y_prev = previous_path_y.end()[-2];
        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    double step_s;
    if (V() < 8) {
        step_s = 30;
    } else {
        step_s = 60;
    }
    auto next_wp0 = road.GetXY(S()+step_s, road.LaneToD(road.DToLane(D()) + lane));
    auto next_wp1 = road.GetXY(S()+step_s*2, road.LaneToD(road.DToLane(D()) + lane));
    auto next_wp2 = road.GetXY(S()+step_s*3, road.LaneToD(road.DToLane(D()) + lane));

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); ++i) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(-ref_yaw)
                  - shift_y * sin(-ref_yaw);
        ptsy[i] = shift_x * sin(-ref_yaw)
                  + shift_y * cos(-ref_yaw);
    }

    tk::spline s;
    s.set_points(ptsx, ptsy);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    for (auto x : previous_path_x) {
        next_x_vals.push_back(x);
    }
    for (auto y : previous_path_y) {
        next_y_vals.push_back(y);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0.;
    for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {
        double n = target_dist / (.02 * mph2mps(ref_vel));
        double x_point = x_add_on + target_x / n;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return {next_x_vals, next_y_vals};
}
/*
vector<double> Vehicle::StepsToFollowLeader(vector<double> leader_state,
                                            double control_period,
                                            int num_period_latancy) const {
    double t = TimeToCatchUpLeader(leader_state);
    vector<double> steps{};
    if (t < 0) {
        return steps;
    }

    double v_ego;
    if (previous_path_x.size() > 1) {
        double diff_x = previous_path_x.back() - previous_path_x.end()[-2];
        double diff_y = previous_path_y.back() - previous_path_y.end()[-2];
        v_ego = sqrt(diff_x*diff_x + diff_y*diff_y) / control_period;
    } else {
        v_ego = V();
    }
    double v_leader = mph2mps(sqrt(leader_state[3]*leader_state[3]
                                   + leader_state[4]*leader_state[4]));
    std::cout << "s: " << S() << ", v: " << v_ego << std::endl;
    std::cout << "leader s: " << leader_state[5] << ", leader v: " << v_leader << std::endl;

    auto coeff = Jmt({0, v_ego, 0},
                     {leader_state[5] - S() - control_period * num_period_latancy * v_ego,// - PreferredBuffer(),
                                 v_leader,
                                 0},
                     t);
    double max_t = std::min(t, 1.);
    std::cout << "aaaaa" << t << std::endl;
    for (double i = 0; i < max_t; i += control_period) {
        double t2 = i * i;
        double t3 = t2 * i;
        double t4 = t3 * i;
        double t5 = t4 * i;
        double v = coeff[0] + coeff[1] * i + coeff[2] * t2 +
                   coeff[3] * t3 + coeff[4] * t4 + coeff[5] * t5;
        steps.push_back(v);
        std::cout << "pushed" << i << " " << v <<" " << control_period<< std::endl;
    }

    return steps;
}
*/
void Vehicle::Update(const vector<double> &state,
                     const vector<double> &previous_path_x,
                     const vector<double> &previous_path_y) {
    if (states.size() >= history_size) {
        states.pop_front();
    }
    states.push_back(state);

    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;
}

void Vehicle::Configure(vector<int> road_data) {
    /*
      Called by simulator before simulation begins. Sets various
      parameters which will impact the ego vehicle.
    */
    //target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

string Vehicle::Display() {
    ostringstream oss;

    oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";

    return oss.str();
}

void Vehicle::Increment(int dt = 1) {
    this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::StateAt(int t) {
    /*
      Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::CollidesWith(Vehicle other, int at_time) {
    /*
      Simple collision detection.
    */
    vector<int> check1 = StateAt(at_time);
    vector<int> check2 = other.StateAt(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::WillCollideWith(Vehicle other, int timesteps) {
    Vehicle::collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1;

    for (int t = 0; t < timesteps+1; t++) {
        if (CollidesWith(other, t)) {
            collider_temp.collision = true;
            collider_temp.time = t;
            return collider_temp;
        }
    }

    return collider_temp;
}

void Vehicle::RealizeState(map<int, vector<vector<int> > > predictions) {
    /*
      Given a state, realize it by adjusting acceleration and lane.
      Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if (state.compare("CS") == 0) {
        RealizeConstantSpeed();
    } else if (state.compare("KL") == 0) {
        RealizeKeepLane(predictions);
    } else if (state.compare("LCL") == 0) {
        RealizeLaneChange(predictions, "L");
    } else if (state.compare("LCR") == 0) {
        RealizeLaneChange(predictions, "R");
    } else if (state.compare("PLCL") == 0) {
        RealizePrepLaneChange(predictions, "L");
    } else if (state.compare("PLCR") == 0) {
        RealizePrepLaneChange(predictions, "R");
    }
}

void Vehicle::RealizeConstantSpeed() {
    a = 0;
}

int Vehicle::_MaxAccelForLane(map<int, vector<vector<int> > > predictions,
                                 int lane,
                                 int s) {
    int delta_v_til_target = target_speed - v;
    int max_acc = std::min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while (it != predictions.end()) {
        int v_id = it->first;

        vector<vector<int> > v = it->second;

        if ((v[0][0] == lane) && (v[0][1] > s)) {
            in_front.push_back(v);
        }
        it++;
    }

    if (in_front.size() > 0) {
        int min_s = 1000;
        vector<vector<int> > leading = {};
        for (int i = 0; i < in_front.size(); i++) {
            if ((in_front[i][0][1]-s) < min_s) {
                min_s = (in_front[i][0][1]-s);
                leading = in_front[i];
            }
        }

        int next_pos = leading[1][1];
        int my_next = s + this->v;
        int separation_next = next_pos - my_next;
        int available_room = separation_next - preferred_buffer;
        max_acc = std::min(max_acc, available_room);
    }

    return max_acc;
}

void Vehicle::RealizeKeepLane(map<int, vector<vector<int> > > predictions) {
    this->a = _MaxAccelForLane(predictions, this->lane, this->s);
}

void Vehicle::RealizeLaneChange(map<int, vector<vector<int> > > predictions,
                                  string direction) {
    int delta = -1;
    if (direction.compare("L") == 0) {
        delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _MaxAccelForLane(predictions, lane, s);
}

void Vehicle::RealizePrepLaneChange(map<int, vector<vector<int> > > predictions,
                                    string direction) {
    int delta = -1;
    if (direction.compare("L") == 0) {
        delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while (it != predictions.end()) {
        int v_id = it->first;
        vector<vector<int> > v = it->second;

        if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
            at_behind.push_back(v);
        }
        it++;
    }
    if (at_behind.size() > 0) {
        int max_s = -1000;
        vector<vector<int> > nearest_behind = {};
        for (int i = 0; i < at_behind.size(); i++) {
            if ((at_behind[i][0][1]) > max_s) {
                max_s = at_behind[i][0][1];
                nearest_behind = at_behind[i];
            }
        }
        int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
        int delta_v = this->v - target_vel;
        int delta_s = this->s - nearest_behind[0][1];
        if (delta_v != 0) {
            int time = -2 * delta_s/delta_v;
            int a;
            if (time == 0) {
                a = this->a;
            } else {
                a = delta_v/time;
            }
            if (a > this->max_acceleration) {
                a = this->max_acceleration;
            }
            if (a < -this->max_acceleration) {
                a = -this->max_acceleration;
            }
            this->a = a;
        } else {
            int my_min_acc = std::max(-this->max_acceleration, -delta_s);
            this->a = my_min_acc;
        }
    }
}

vector<vector<int> > Vehicle::GeneratePredictions(int horizon = 10) {
    vector<vector<int> > predictions;
    for (int i = 0; i < horizon; i++) {
        vector<int> check1 = StateAt(i);
        vector<int> lane_s = {check1[0], check1[1]};
        predictions.push_back(lane_s);
    }
    return predictions;
}
