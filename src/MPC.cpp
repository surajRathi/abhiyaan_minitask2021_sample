#include <mpc_simple/MPC.h>

#include <cppad/ipopt/solve.hpp>
#include <cppad/ipopt/solve_result.hpp>

#include <ros/ros.h>


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

MPC::ControlInputs MPC::get_control_input(const State &s) {
    state = s;

    const auto start = std::chrono::high_resolution_clock::now();

    CppAD::ipopt::solve_result<Dvector> solution;
    CppAD::ipopt::solve(ipopt_options, _vars, vars_b.low, vars_b.high, cons_b.low, cons_b.high, *this, solution);

    ROS_INFO("IPOPT %li ms.", std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start).count());

    ROS_INFO("%s", ipopt_error_string.at(solution.status).c_str());

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

    // Replace with State<ADvector::value_type> ??
    ADvector::value_type x{state.x_s}, y{state.y_s}, theta{state.theta}, v{state.v}, al{state.al};
    const auto x_o = state.x_o, y_o = state.y_o, x_g = state.x_g, y_g = state.y_g;

    for (auto i = 0; i < p.timesteps; i++) {
        v += p.dt * vars[i];
        al += p.dt * vars[i + p.timesteps];
        auto dx = p.dt * v;
        auto dy = p.dt * v * CppAD::tan(al) / 2;
        auto c = CppAD::cos(theta), s = CppAD::sin(theta);
        x += c * dx - s * dy;
        y += c * dy + s * dx;
        theta += p.dt * v * CppAD::tan(al) / 1.0;

        cons[i + 0 * p.timesteps] = v;
        cons[i + 1 * p.timesteps] = al;
        cons[i + 2 * p.timesteps] = CppAD::pow(x - x_o, 2) + CppAD::pow(y - y_o, 2);

        objective_func += CppAD::pow(x - x_g, 2) + CppAD::pow(y - y_g, 2); // Goal
        objective_func += 1.0 / (CppAD::pow(x - x_o, 2) + CppAD::pow(y - y_o, 2)); // Obstacle
        objective_func += 10 * CppAD::abs(theta - CppAD::atan2(y_g - y, x_g - x));
    }


}
