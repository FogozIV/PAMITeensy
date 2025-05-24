//
// Created by fogoz on 20/05/2025.
//

#ifndef PAMITEENSY_SPEEDPROFILEPOINT_H
#define PAMITEENSY_SPEEDPROFILEPOINT_H

#include "vector"
#include "cmath"

/**
 * @brief Point in a speed-time-position profile
 * 
 * This structure represents a single point in a motion profile,
 * containing:
 * - Position (s)
 * - Time (t)
 * - Velocity (v)
 */
struct SpeedProfilePoint {
    double s;  ///< Position
    double t;  ///< Time
    double v;  ///< Velocity
};

/**
 * @brief Computes velocity profile for motion
 * 
 * This function generates a velocity profile that respects:
 * - Maximum acceleration
 * - Maximum velocity
 * - Total distance
 * 
 * The profile is computed using a two-pass algorithm:
 * 1. Forward pass: Acceleration-limited profile
 * 2. Backward pass: Deceleration-limited profile
 * 
 * @param s_total Total distance to travel
 * @param a_max Maximum acceleration
 * @param v_max Maximum velocity
 * @param ds Position step size
 * @param v_profile Output velocity profile v(s)
 */
inline void compute_speed_profile(
        double s_total,
        double a_max,
        double v_max,
        double ds,
        std::vector<double> &v_profile // output: v(s)
) {
    int N = static_cast<int>(s_total / ds) + 1;
    v_profile.resize(N);

    // Forward pass: Acceleration-limited profile
    v_profile[0] = 0.0;
    for (int i = 1; i < N; ++i) {
        double v_prev = v_profile[i - 1];
        v_profile[i] = std::min(std::sqrt(v_prev * v_prev + 2.0 * a_max * ds), v_max);
    }

    // Backward pass: Deceleration-limited profile
    v_profile[N - 1] = std::min(v_profile[N - 1], 0.0);
    for (int i = N - 2; i >= 0; --i) {
        double v_next = v_profile[i + 1];
        double v_lim = std::sqrt(v_next * v_next + 2.0 * a_max * ds);
        v_profile[i] = std::min(v_profile[i], v_lim);
    }
}

/**
 * @brief Computes time profile from velocity profile
 * 
 * This function calculates the time at each position point
 * based on the velocity profile. It ensures:
 * - Minimum velocity threshold
 * - Proper time integration
 * - Continuous time profile
 * 
 * @param v_profile Input velocity profile
 * @param ds Position step size
 * @param t_profile Output time profile
 */
inline void compute_time_profile(
        const std::vector<double>& v_profile,
        double ds,
        std::vector<double>& t_profile // output
) {
    const double EPS = 1e-4;  // Minimum velocity threshold
    int N = v_profile.size();
    t_profile.resize(N);
    double t = 0.0;
    t_profile[0] = 0.0;

    for (int i = 1; i < N; ++i) {
        double v = std::max(v_profile[i], EPS);
        double dt = ds / v;
        t += dt;
        t_profile[i] = t;
    }
}

/**
 * @brief Builds complete motion profile
 * 
 * This function generates a complete motion profile containing:
 * - Position points
 * - Velocity at each point
 * - Time at each point
 * 
 * The profile respects:
 * - Maximum acceleration
 * - Maximum velocity
 * - Total distance
 * 
 * @param s_total Total distance
 * @param a_max Maximum acceleration
 * @param v_max Maximum velocity
 * @param ds Position step size
 * @param profile Output motion profile
 */
inline void build_speed_time_profile(
        double s_total,
        double a_max,
        double v_max,
        double ds,
        std::vector<SpeedProfilePoint>& profile // final result
) {
    std::vector<double> v_profile;
    std::vector<double> t_profile;

    compute_speed_profile(s_total, a_max, v_max, ds, v_profile);
    compute_time_profile(v_profile, ds, t_profile);

    profile.clear();
    for (size_t i = 0; i < v_profile.size(); ++i) {
        profile.push_back({i * ds, v_profile[i], t_profile[i]});
    }
}

/**
 * @brief Interpolates position at a given time
 * 
 * This function finds the position at any time by:
 * - Finding the surrounding profile points
 * - Linear interpolation between points
 * - Handling boundary conditions
 * 
 * @param profile Motion profile
 * @param t_query Query time
 * @return double Interpolated position
 */
inline double get_position_at_time(const std::vector<SpeedProfilePoint>& profile, double t_query) {
    if (t_query <= profile.front().t) return profile.front().s;
    if (t_query >= profile.back().t) return profile.back().s;

    for (size_t i = 1; i < profile.size(); ++i) {
        if (t_query < profile[i].t) {
            double t0 = profile[i - 1].t;
            double t1 = profile[i].t;
            double s0 = profile[i - 1].s;
            double s1 = profile[i].s;

            double alpha = (t_query - t0) / (t1 - t0);
            return s0 + alpha * (s1 - s0);
        }
    }
    return profile.back().s; // fallback
}

#endif //PAMITEENSY_SPEEDPROFILEPOINT_H
