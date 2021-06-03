//
// Created by suraj on 6/2/21.
//

#ifndef MPC_SIMPLE_MPC_H
#define MPC_SIMPLE_MPC_H


#include <turtlesim/Pose.h>
#include <cppad/cppad.hpp>

namespace mpc_simple {
    template<typename T>
    // using vector = std::vector<T>;
    // using vector = std::valarray<T>;
    using vector = CppAD::vector<T>;
    using Dvector = vector<double>;


    template<typename T>
    struct LH {
        T low, high;

        explicit LH(T val) : low{-val}, high{val} {
            assert(val >= 0);
        }

        LH(T l, T h) : low{l}, high{h} {}
    };


    struct Params {
        size_t timesteps = 10;
        float dt = 0.1;

        LH<double> a{0.4}, v{1},
                alpha_dot{0.1}, alpha{0.4};

        double safe_dist = 1;

    };

    class MPC {
    public:
        using ADvector = vector<CppAD::AD<double>>;
    private:
        const Params p;
        std::string ipopt_options;
        double _v{0}, _al{0}, _x_s{0}, _y_s{0}, _theta{0},
                _x_o{0}, _y_o{0}, _x_g{0}, _y_g{0};

    public:
        explicit MPC(Params p_in = Params{});

        Dvector _vars;
        LH<Dvector> vars_b, cons_b;

        struct ControlInputs {
            double a, alpha_dot;
        };

        ControlInputs
        get_control_input(double v, double alpha_in,
                          double x_self, double y_self, double theta,
                          double x_other, double y_other,
                          double x_goal, double y_goal);

        void operator()(ADvector &outputs, ADvector &vars) const;
    };

}

#endif //MPC_SIMPLE_MPC_H
