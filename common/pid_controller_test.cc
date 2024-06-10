#include "pid_controller.h"

#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

class PIDControllerTest {
   public:
    double TestControl(const double error) {
        assert(pid_controller_ != nullptr);
        return pid_controller_->Control(error);
    }

    double GetIntegral() const {
        assert(pid_controller_ != nullptr);
        return pid_controller_->integral_;
    }

    void SetControllerConf(const PIDControllerConf& conf) {
        pid_controller_.reset(new PIDController(conf));
    }

   private:
    std::unique_ptr<PIDController> pid_controller_{nullptr};
};

int main() {
    PIDControllerTest test;
    PIDControllerConf conf{.kp = 1.0,
                           .ki = 1.0,
                           .kd = 0.0,
                           .anti_windup_method = AntiWindupMethod::CLAMPING,
                           .cmd_bound = std::make_pair(-1.5, 1.5)};
    test.SetControllerConf(conf);

    std::vector<double> errors{1.0, 1.2, 1.4, 1.6, 1.8, 1.4, 1.2};
    for (double error : errors) {
        std::cout << "error: " << std::fixed << std::setprecision(3) << error
                  << ",command: " << test.TestControl(error)
                  << ",integral: " << test.GetIntegral() << std::endl;
    }

    return 0;
}