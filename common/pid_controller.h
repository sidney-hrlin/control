#pragma once

#include <cassert>
#include <optional>

class PIDControllerTest;

enum class AntiWindupMethod {
    NONE = 0,
    CLAMPING = 1,
    BACK_CALCULATION = 2,
};

struct PIDControllerConf {
    double dt{0.01};
    double kp{1.0};
    double ki{0.0};
    double kd{0.0};

    AntiWindupMethod anti_windup_method{AntiWindupMethod::NONE};

    std::optional<std::pair<double, double>> cmd_bound;
    std::optional<double> kb;
    std::optional<double> diff_lpf_cutoff_freq;
};

class PIDController {
   public:
    explicit PIDController(const PIDControllerConf conf) : conf_(conf) {
        assert(conf_.dt > kMinTimeStep);
        if (conf_.cmd_bound.has_value()) {
            auto [lb, ub] = conf_.cmd_bound.value();
            assert(lb < ub);
        }
        if (conf_.diff_lpf_cutoff_freq.has_value()) {
            const double wc = 2.0 * M_PI * conf_.diff_lpf_cutoff_freq.value();
            alpha_ = 1.0 / (1.0 + wc * conf_.dt);
        }
    }

    double Control(double error) {
        double diff = 0.0;
        if (first_hit_) {
            first_hit_ = false;
        } else {
            diff = (1.0 - alpha_) * (error - prev_error_) / conf_.dt +
                   alpha_ * prev_diff_;
        }

        // calculate integral term with anti-windup algorithms
        switch (conf_.anti_windup_method) {
            case AntiWindupMethod::CLAMPING:
                CalIntegralWithClamping(error, diff);
                break;
            case AntiWindupMethod::BACK_CALCULATION:
                CalIntegralWithBackCalculation(error, diff);
                break;
            default:
                CalIntegral(error);
        }

        // calculate command
        double cmd = conf_.kp * error + integral_ + conf_.kd * diff;

        // applu command saturation
        if (conf_.cmd_bound.has_value()) {
            auto [cmd_lb, cmd_ub] = conf_.cmd_bound.value();
            if (cmd > cmd_ub) {
                cmd = cmd_ub;
            }
            if (cmd < cmd_lb) {
                cmd = cmd_lb;
            }
        }

        // update internal states
        prev_error_ = error;
        prev_diff_ = diff;

        return cmd;
    }

    void Reset() {
        prev_error_ = 0.0;
        prev_diff_ = 0.0;
        integral_ = 0.0;
        first_hit_ = true;
    }

   private:
    void CalIntegral(const double error) {
        integral_ += (conf_.ki * error * conf_.dt);
    }

    void CalIntegralWithClamping(const double error, const double diff) {
        assert(conf_.cmd_bound.has_value());
        const double cmd = conf_.kp * error + integral_ +
                           conf_.ki * error * conf_.dt + conf_.kd * diff;
        auto [cmd_lb, cmd_ub] = conf_.cmd_bound.value();
        if (const bool is_cmd_saturated = (cmd < cmd_lb) || (cmd > cmd_ub);
            is_cmd_saturated && (error * cmd) > 0.0) {
            // stop accumulating integral term if error will increase
            // command saturation when command is already saturated
        } else {
            CalIntegral(error);
        }
    }

    void CalIntegralWithBackCalculation(const double error, const double diff) {
        assert(conf_.kb.has_value());
        assert(conf_.cmd_bound.has_value());
        const double cmd = conf_.kp * error + integral_ +
                           conf_.ki * error * conf_.dt + conf_.kd * diff;
        auto [cmd_lb, cmd_ub] = conf_.cmd_bound.value();
        const double offset = (cmd > cmd_ub)   ? (cmd_ub - cmd)
                              : (cmd < cmd_lb) ? (cmd_lb - cmd)
                                               : 0.0;
        // apply a negative feedback on intergral
        integral_ += conf_.kb.value() * offset + conf_.ki * error * conf_.dt;
    }

    PIDControllerConf conf_;

    double prev_error_{0.0};

    double prev_diff_{0.0};

    double integral_{0.0};

    double alpha_{0.0};

    bool first_hit_{true};

    static constexpr double kMinTimeStep{1e-6};

    friend PIDControllerTest;
};