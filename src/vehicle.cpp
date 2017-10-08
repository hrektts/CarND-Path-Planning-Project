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
    this->line_kept = 1;
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
    int new_lane = 0;
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
            new_lane = l;
        }
    }
    v_ref = new_v;
    if (new_lane == 0) {
        if (line_kept < 250) {
            line_kept += 1;
        }
    } else {
        line_kept = 1;
    }

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
        Vehicle leader = Vehicle(0, 0, 0, 1);
        double v_leader = sqrt(leader_state[3] * leader_state[3]
                               + leader_state[4] * leader_state[4]);
        double psi_leader = rad2deg(atan2(leader_state[4], leader_state[3]));
        leader.Update({leader_state[1],
                        leader_state[2],
                        leader_state[5],
                        leader_state[6],
                        v_leader,
                        psi_leader},
            {}, {});
        auto leader_trajectory = leader.Trajectory(v_leader, road, keep_lane);

        size_t check_size = std::min(trajectory[0].size(), leader_trajectory[0].size());
        for (size_t i = 0; i < check_size; i++) {
            if (distance(trajectory[0][i], trajectory[1][i],
                         leader_trajectory[0][i], leader_trajectory[1][i])
                < check_range_front) {
                cost += 1000;
                break;
            }
        }

        if (v_leader < V()) {
            cost += std::max(-100 / (PreferredBuffer() * 3) * (leader_state[5] - S()) + 100, 0.);
        }
    }

    if (!follower_state.empty()) {
        Vehicle follower = Vehicle(0, 0, 0, 1);
        double v_follower = sqrt(follower_state[3] * follower_state[3]
                                 + follower_state[4] * follower_state[4]);
        double psi_follower = rad2deg(atan2(follower_state[4], follower_state[3]));
        follower.Update({follower_state[1],
                        follower_state[2],
                        follower_state[5],
                        follower_state[6],
                        v_follower,
                        psi_follower},
            {}, {});
        auto follower_trajectory = follower.Trajectory(v_follower, road, keep_lane);

        size_t check_size = std::min(trajectory[0].size(), follower_trajectory[0].size());
        for (size_t i = 0; i < check_size; i++) {
            if (distance(trajectory[0][i], trajectory[1][i],
                         follower_trajectory[0][i], follower_trajectory[1][i])
                < check_range_rear) {
                cost += 1000;
                break;
            }
        }

        if (v_follower > V() && lane != 0) {
            cost += std::max(100 / (PreferredBuffer() * 3) * (follower_state[5] - S()) + 100, 0.);
        }
    }

    cost += abs(lane) * 10;

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
