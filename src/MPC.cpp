#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.1;


const int x_start = 0;
const int y_start = N;
const int v_start = 2*N;
const int psi_start = 3*N;
const int cte_start = 4*N;
const int epsi_start = 5*N;
const int delta_start = 6*N;
const int a_start = 6*N + (N-1);


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

class FG_eval {
    public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector& fg, const ADvector& vars) 
    {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
    

        // ********************************
        // Initialize the cost constraints:
        // ********************************
        fg[0] = 0;

        // Cost based on reference state
        for (int t=0; t<N; ++t)
        {
            fg[0] += CppAD::pow(vars[cte_start+t], 2);
            fg[0] += CppAD::pow(vars[epsi_start+t], 2);
            fg[0] += CppAD::pow(vars[v_start+t], 2);
        }

        // Minimize actuator use
        for (int t=0; t<N-1; ++t)
        {
            fg[0] += CppAD::pow(vars[delta_start+t], 2);
            fg[0] += CppAD::pow(vars[a_start+t], 2);
        }

        // Minimize value gap for the actuators
        for (int t=0; t<N-2; ++t)
        {
            fg[0] += CppAD::pow(vars[delta_start+t+1] - vars[delta_start+t], 2);
            fg[0] += CppAD::pow(vars[a_start+t+1] - vars[a_start+1], 2);
        }


        // ********************************
        // Initialization & Constraints
        // ********************************

        // INITIALIZATION
        fg[x_start+1] = vars[x_start];
        fg[y_start+1] = vars[y_start];
        fg[psi_start+1] = vars[psi_start];
        fg[v_start+1] = vars[v_start];
        fg[cte_start+1] = vars[cte_start];
        fg[epsi_start+1] = vars[epsi_start];


        // CONSTRAINTS
        // Notice that the loop starts with t=1.  (cf. initialization above)
        for (int t=1; t<N; ++t)
        {
            // Time t+1
            AD<double> x1 = vars[x_start+t];
            AD<double> y1 = vars[y_start+t];
            AD<double> v1 = vars[v_start+t];
            AD<double> psi1 = vars[psi_start+t];
            AD<double> cte1 = vars[cte_start+t];
            AD<double> epsi1 = vars[epsi_start+t];
            
            // Time t
            AD<double> x0 = vars[x_start+t-1];
            AD<double> y0 = vars[y_start+t-1];
            AD<double> v0 = vars[v_start+t-1];
            AD<double> psi0 = vars[psi_start+t-1];
            AD<double> cte0 = vars[cte_start+t-1];            
            AD<double> epsi0 = vars[epsi_start+t-1];
            AD<double> delta0 = vars[delta_start+t-1];
            AD<double> a0 = vars[a_start+t-1];



            fg[x_start+t+1] = x1 - (x0 + v0 * cos(psi0)*dt);
            fg[y_start+t+1] = y1 - (y0 + v0 * sin(psi0)*dt);
            fg[v_start+t+1] = v1 - (v0 + a0*dt);
            fg[psi_start + t + 1] = psi1 - x0 + (psi0 + v0 * delta0/Lf *dt);
            
            AD<double> f0 = coeffs[0] + coeffs[1] * x0;
            fg[cte_start+t+1] = cte1 - (f0 - y0 + (v0 * sin(epsi0)*dt) );

            AD<double> psides0 = CppAD::atan(coeffs[1]);
            fg[epsi_start+t+1] = epsi1 - (psi0 - psides0 + (v0 * delta0/Lf *dt));
        }



    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) 
{
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9

    size_t n_vars = 6*N + 2*(N-1);
    size_t n_constraints = N*7;





    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    
    for (int i = 0; i < n_vars; i++) 
    {
        vars[i] = 0;
    }


    // Initial variables
    vars[x_start] = state[0];
    vars[y_start] = state[1];
    vars[v_start] = state[2];
    vars[psi_start] = state[3];
    vars[cte_start] = state[4];
    vars[epsi_start] = state[5];


    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set lower and upper limits for variables.
    for (int i=0; i<n_vars; ++i)
    {
        vars_lowerbound[i] = -0.5;
        vars_upperbound[i] = 0.5;
    }


    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; ++i) 
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);
    
    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;


    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    
    //std::cout << "delta: " << solution.x[delta_start] << std::endl;
    //std::cout << "a: " << solution.x[a_start] << std::endl;

    return {};
}
