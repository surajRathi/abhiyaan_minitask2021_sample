#include <mpc_simple/MPC.h>
#include <cppad/ipopt/solve.hpp>
#include <cppad/ipopt/solve_result.hpp>

using namespace mpc_simple;

const std::map<size_t, std::string> ipopt_error_string = { // NOLINT(cert-err58-cpp)
        {0,  "not_defined"},
        {1,  "success"},
        {2,  "max_iter_exceeded"},
        {3,  "stop_at_tiny_step"},
        {4,  "stop_at_acceptable_point"},
        {5,  "local_infeasibility"},
        {6,  "user_requested_stop"},
        {7,  "feasible_point_found"},
        {8,  "diverging_iterates"},
        {9,  "restoration_failure"},
        {10, "error_in_step_computation"},
        {11, "invalid_number_detected"},
        {12, "too_few_degrees_of_freedom"},
        {13, "internal_error"},
        {14, "unknown"}
};

MPC::MPC(Params p_in) : p{p_in}, _vars{p.timesteps * 2},
                        vars_b{{p.timesteps * 2},
                               {p.timesteps * 2}},
                        cons_b({{p.timesteps * 3},
                                {p.timesteps * 3}}) {

    for (auto i = 0; i < p.timesteps; i++) {
        vars_b.low[i] = p.a.low;
        vars_b.high[i] = p.a.high;
        vars_b.low[i + p.timesteps] = p.alpha_dot.low;
        vars_b.high[i + p.timesteps] = p.alpha_dot.high;

        cons_b.low[i] = p.v.low;
        cons_b.high[i] = p.v.high;
        cons_b.low[i + p.timesteps] = p.alpha.low;
        cons_b.high[i + p.timesteps] = p.alpha.high;

        cons_b.low[i + p.timesteps * 2] = p.safe_dist * p.safe_dist;
        cons_b.high[i + p.timesteps * 2] = 2e19;
    }


    ipopt_options += "Integer print_level  0\n"; // Disables all debug information
    ipopt_options += "String sb yes\n"; // Disables printing IPOPT creator banner
    ipopt_options += "Sparse  true        forward\n";
    //ipopt_options += "Sparse  true        reverse\n";
    ipopt_options += "Numeric max_cpu_time          0.5\n";
}

MPC::ControlInputs MPC::get_control_input(
        double v, double alpha_in,
        double x_self, double y_self, double theta,
        double x_other, double y_other,
        double x_goal, double y_goal) {

    _v = v;
    _al = alpha_in;
    _x_s = x_self;
    _y_s = y_self;
    _theta = theta;
    _x_o = x_other;
    _y_o = y_other;
    _x_g = x_goal;
    _y_g = y_goal;

    CppAD::ipopt::solve_result<Dvector> solution;

#include <iostream>
#include <iomanip>

//    std::cout << std::fixed;
//    std::cout << std::setprecision(2);
//    std::cout << v << " " << alpha_in << std::endl;
    const auto start = std::chrono::high_resolution_clock::now();
    CppAD::ipopt::solve(ipopt_options, _vars, vars_b.low, vars_b.high, cons_b.low, cons_b.high, *this, solution);
//    std::cout << "IPOPT " << std::chrono::duration_cast<std::chrono::milliseconds>(
//            std::chrono::high_resolution_clock::now() - start).count() << "ms." << std::endl;
//    std::cout << solution.x << std::endl;
//    std::cout << solution.g << std::endl;
//    std::cout << solution.obj_value << std::endl;
//    std::cout << ipopt_error_string.at(solution.status) << std::endl;

    return {solution.x[0], solution.x[0 + p.timesteps]};
}

class ConsWrapper {
    MPC::ADvector &_outputs;
public:
    explicit ConsWrapper(MPC::ADvector &outputs) : _outputs(outputs) {}

    MPC::ADvector::value_type &operator[](size_t index) { return _outputs[1 + index]; }
};

void MPC::operator()(MPC::ADvector &outputs, MPC::ADvector &vars) const {
    auto &objective_func = outputs[0];
    ConsWrapper cons{outputs};
    objective_func = 0;

    ADvector::value_type x{_x_s}, y{_y_s}, theta{_theta}, v{_v}, al{_al};

    for (auto i = 0; i < p.timesteps; i++) {
        v += p.dt * vars[i];
        al += p.dt * vars[i + p.timesteps];
        auto dx = v * p.dt;
        auto dy = p.dt * v * CppAD::tan(al) / 2;
        auto c = CppAD::cos(theta), s = CppAD::sin(theta);
        x += c * dx - s * dy;
        y += c * dy + s * dx;
        theta += p.dt * v * CppAD::tan(al) / 1.0;

        cons[i + 0 * p.timesteps] = v;
        cons[i + 1 * p.timesteps] = al;
        cons[i + 2 * p.timesteps] = pow(x - _x_o, 2) + pow(y - _y_o, 2);

        objective_func +=
                CppAD::pow(x - _x_g, 2) + CppAD::pow(y - _y_g, 2) + 1 / (pow(x - _x_o, 2) + pow(y - _y_o, 2));
    }


}