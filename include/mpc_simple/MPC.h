#ifndef MPC_SIMPLE_MPC_H
#define MPC_SIMPLE_MPC_H

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
        size_t timesteps = 20;
        float dt = 0.1;

        LH<double> a{0.4}, v{1},
                alpha_dot{0.3}, alpha{0.7};

        double safe_dist = 1;

    };

    struct State {
        double v{0}, al{0},
                x_s{0}, y_s{0}, theta{0},
                x_o{0}, y_o{0}, x_g{0}, y_g{0};
    };

    class MPC {
    public:
        using ADvector = vector<CppAD::AD<double>>;
    private:
        const Params p;
        std::string ipopt_options;

        State state;

    public:
        explicit MPC(Params p_in = Params{});

        // variables: {a0, a1, a2 ... a_timesteps, alpha_dot_0, alpha_dot_1 ... alpha_dot_timesteps}
        // constraints: {v0... , alpha0... , dist_from_obstacle0... }
        Dvector _vars;
        LH<Dvector> vars_b, cons_b;


        struct ControlInputs {
            double a, alpha_dot;
        };

        ControlInputs get_control_input(const State &s);

        void operator()(ADvector &outputs, ADvector &vars) const;
    };

}

#endif //MPC_SIMPLE_MPC_H
