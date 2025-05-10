//
// Created by fogoz on 13/12/2024.
//

#ifndef CODEPAMI_INTEGRATOR_H
#define CODEPAMI_INTEGRATOR_H
#include <functional>
#include <vector>
// Implement the generalized RK4 method
inline void runge_kutta_4(double t0, std::vector<double>& y0, double t_end, double h, std::function<void(double t, std::vector<double>& y, std::vector<double>& dydt)> f) {
    double t = t0; // Current time
    int n = y0.size();
    std::vector<double> y(n);
    std::vector<double> k1(n);
    std::vector<double> k2(n);
    std::vector<double> k3(n);
    std::vector<double> k4(n);
    std::vector<double> temp(n);

    // Initialize y with the initial condition y0
    y.assign(y0.begin(), y0.end());

    // Main integration loop
    while (t < t_end) {
        // Compute k1
        f(t, y, k1);
        for (int i = 0; i < n; ++i) {
            k1[i] *= h;
            temp[i] = y[i] + k1[i] / 2.0;
        }

        // Compute k2
        f(t + h / 2.0, temp, k2);
        for (int i = 0; i < n; ++i) {
            k2[i] *= h;
            temp[i] = y[i] + k2[i] / 2.0;
        }

        // Compute k3
        f(t + h / 2.0, temp, k3);
        for (int i = 0; i < n; ++i) {
            k3[i] *= h;
            temp[i] = y[i] + k3[i];
        }

        // Compute k4
        f(t + h, temp, k4);
        for (int i = 0; i < n; ++i) {
            k4[i] *= h;
        }

        // Update y
        for (int i = 0; i < n; ++i) {
            y[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        }

        // Update time
        t += h;
    }
    y0.assign(y.begin(), y.end());
}

inline double runge_kutta_4_maximized(double t0, std::vector<double>& y0, double t_end, double h, std::function<void(double t, std::vector<double>& y, std::vector<double>& dydt)> f, const std::vector<double> lookup_length) {
    double t = t0; // Current time
    int n = y0.size();
    std::vector<double> y(n);
    std::vector<double> k1(n);
    std::vector<double> k2(n);
    std::vector<double> k3(n);
    std::vector<double> k4(n);
    std::vector<double> temp(n);

    // Initialize y with the initial condition y0
    y.assign(y0.begin(), y0.end());


    // Main integration loop
    while (t < t_end) {
        int j = 0;
        for(int i = 0; i < n; i++){
            if(y[i] > lookup_length[i])
                j++;
        }
        if(j == n){
            y0.assign(y.begin(), y.end());
            return t;
        }
        // Compute k1
        f(t, y, k1);
        for (int i = 0; i < n; ++i) {
            k1[i] *= h;
            temp[i] = y[i] + k1[i] / 2.0;
        }

        // Compute k2
        f(t + h / 2.0, temp, k2);
        for (int i = 0; i < n; ++i) {
            k2[i] *= h;
            temp[i] = y[i] + k2[i] / 2.0;
        }

        // Compute k3
        f(t + h / 2.0, temp, k3);
        for (int i = 0; i < n; ++i) {
            k3[i] *= h;
            temp[i] = y[i] + k3[i];
        }

        // Compute k4
        f(t + h, temp, k4);
        for (int i = 0; i < n; ++i) {
            k4[i] *= h;
        }

        // Update y
        for (int i = 0; i < n; ++i) {
            y[i] += (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;
        }

        // Update time
        t += h;
    }
    y0.assign(y.begin(), y.end());
    return t_end;
}

#endif //CODEPAMI_INTEGRATOR_H
