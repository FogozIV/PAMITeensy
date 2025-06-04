//
// Created by fogoz on 03/06/2025.
//

#ifndef OSCILLATIONTRACKER_H
#define OSCILLATIONTRACKER_H
#include <vector>
#include <limits>
#include <cmath>
#include <numeric>

class OscillationTracker {
public:
    struct Peak {
        double value;
        unsigned long time_us;
    };

private:
    double threshold = 0.1f;
    double last_value = 0.0f;
    bool looking_for_max = true;
    unsigned long min_peak_interval_us = 10000;

    std::vector<Peak> peaks;
    size_t max_peaks_to_store = 20;

    double last_amplitude = 0.0f;
    double current_amplitude = 0.0f;
    unsigned long current_time_us = 0;
    double alpha = 0.1;
    double last_extreme = 0.0;
    bool waiting_for_turn = false;
public:
    void update_absolute(double value, unsigned long time_us) {
        value = alpha * value + (1.0 - alpha) * value;
        // Smooth signal or filter out small changes
        if (std::abs(value - last_value) < threshold) {
            return;
        }

        // Detect local extrema
        if (looking_for_max) {
            if (value > last_value) {
                last_extreme = value;
            } else if (value < last_value && waiting_for_turn) {
                register_peak({last_extreme, time_us});
                looking_for_max = false;
                waiting_for_turn = false;
            } else {
                waiting_for_turn = true;
            }
        } else {
            if (value < last_value) {
                last_extreme = value;
            } else if (value > last_value && waiting_for_turn) {
                register_peak({last_extreme, time_us});
                looking_for_max = true;
                waiting_for_turn = false;
            } else {
                waiting_for_turn = true;
            }
        }

        last_value = value;

        last_value = value;
    }

    void update(double value, unsigned long dt) {
        current_time_us += dt;
        update_absolute(value, current_time_us);
    }

    bool is_oscillating() const {
        return peaks.size() >= 4;  // at least 2 full cycles
    }

    double get_mean_amplitude() const {
        if (peaks.size() < 2) return 0.0;

        float total = 0;
        size_t count = 0;
        for (size_t i = 1; i < peaks.size(); i += 2) {
            float amp = std::abs(peaks[i].value - peaks[i - 1].value) / 2.0f;
            total += amp;
            ++count;
        }
        return (count > 0) ? total / count : 0.0f;
    }

    double get_oscillation_period_s() const {
        if (peaks.size() < 3) return 0.0f;

        float total_period = 0.0f;
        int count = 0;
        for (size_t i = 2; i < peaks.size(); i += 2) {
            total_period += (peaks[i].time_us - peaks[i - 2].time_us) / 1e6f;
            ++count;
        }
        return (count > 0) ? total_period / count : 0.0f;
    }

    enum class AmplitudeTrend {
        Increasing,
        Decreasing,
        Stable,
        Unknown
    };

    AmplitudeTrend get_amplitude_trend() {
        if (peaks.size() < 6) return AmplitudeTrend::Unknown;

        float prev = last_amplitude;
        last_amplitude = get_mean_amplitude();
        float delta = last_amplitude - prev;

        if (std::abs(delta) < 0.01f) return AmplitudeTrend::Stable;
        return (delta > 0) ? AmplitudeTrend::Increasing : AmplitudeTrend::Decreasing;
    }

private:
    void register_peak(const Peak& peak) {
        if (!peaks.empty() && peak.time_us - peaks.back().time_us < min_peak_interval_us)
            return;
        if (peaks.size() >= max_peaks_to_store)
            peaks.erase(peaks.begin());
        peaks.push_back(peak);
    }
};
#endif //OSCILLATIONTRACKER_H
