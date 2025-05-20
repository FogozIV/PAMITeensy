//
// Created by fogoz on 20/05/2025.
//

#ifndef PAMITEENSY_SPEEDPROFILEPOINT_H
#define PAMITEENSY_SPEEDPROFILEPOINT_H

#include "vector"
#include "cmath"

struct SpeedProfilePoint {
    double s;
    double t;
    double v;
};

inline void compute_speed_profile(
        double s_total,
        double a_max,
        double v_max,
        double ds,
        std::vector<double> &v_profile // output: v(s)
) {
    int N = static_cast<int>(s_total / ds) + 1;
    v_profile.resize(N);

    // Forward pass
    v_profile[0] = 0.0;
    for (int i = 1; i < N; ++i) {
        double v_prev = v_profile[i - 1];
        v_profile[i] = std::min(std::sqrt(v_prev * v_prev + 2.0 * a_max * ds), v_max);
    }

    // Backward pass
    v_profile[N - 1] = std::min(v_profile[N - 1], 0.0);
    for (int i = N - 2; i >= 0; --i) {
        double v_next = v_profile[i + 1];
        double v_lim = std::sqrt(v_next * v_next + 2.0 * a_max * ds);
        v_profile[i] = std::min(v_profile[i], v_lim);
    }
}

inline void compute_time_profile(
        const std::vector<double>& v_profile,
        double ds,
        std::vector<double>& t_profile // output
) {
    const double EPS = 1e-4;
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
