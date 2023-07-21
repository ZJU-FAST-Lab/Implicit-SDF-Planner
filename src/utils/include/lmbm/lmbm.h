#ifndef LMBM_HPP
#define LMBM_HPP

#include <cstdio>
#include <cstdlib>


namespace lmbm
{

/**
 * LMBM optimization parameters. 
 *  Parameters are initialized to default values.
 */
struct lmbm_parameter_t
{
    /**
     * The timeout in seconds for optimization.
     */
    float timeout = 300.0;//见Fortran Subroutines for large scale nonsmooth optimization user manul 11页限时

    /**
     * Size of the bundle (>=2).
     *  Recommended value ranges from [2,10].
     */
    int bundle_size = 2;

    /**
     * Number of stored correction pairs.
     *  Recommended value ranges from [3,15].
     *  ini_corrections <= max_corrections is required.
     */
    int ini_corrections = 7;

    /**
     * Upper limit for number of stored correction pairs.
     *  Recommended value ranges from [3,20].
     */
    int max_corrections = 15;

    /**
     * Exponent for distance measure. 
     *  Used for the calculations of subgradient locality 
     *  measure. 
     */
    int exponent_distmeasure = 2;

    /**
     * Maximum number of iterations.
     */
    int max_iterations = 10000;

    /**
     * Maximum number of function evaluations.
     */
    int max_evaluations = 20000;

    /**
     * Distance for delta-based convergence test.
     *  This parameter determines the distance, in iterations, to compute
     *  the rate of decrease of the objective function.结合delta_past使用判断收敛条件
     */
    int past = 10;

    /**
     * Printout specification.
     * -1: No printout. 
     *  0: Print only the error messages. 
     *  1: Print basic details of the final solution. 
     *  2: Print basic details of the final solution and the most serious 
     *     warning messages. 
     *  3: Print the whole final solution including the components of X 
     *     and the most serious warning messages. 
     *  4: Print details (including warning messages) of every iteration 
     *     excluding the components of X. 
     *  5: Print details of every iteration including the components of X 
     *     and the most serious warning messages.
     */
    int verbose = -1;

    /**
     * Update scheme of variable metric matrices.
     *  0: SR1 update. 
     *  1: L-BFGS update. 
     */
    int update_method = 0;

    /**
     * Scaling strategy for variable metric updates. 
     *  0: Scaling at every iteration with <s,u>/<u,u>, where s and u 
     *     denotes the difference on current and auxiliary iteration 
     *     points and subgradients, respectively.
     *  1: Scaling at every iteration with <s,s>/<s,u>.
     *  2: Interval scaling with <s,u>/<u,u>.
     *  3: Interval scaling with <s,s>/<s,u>.
     *  4: Preliminary scaling with <s,u>/<u,u>.
     *  5: Preliminary scaling with <s,s>/<s,u>.
     *  6: No scaling.
     * 
     *  Recommended selections are,  
     *      scaling_strategy = 0 for update_method = 0,
     *      scaling_strategy = 2 for update_method = 1.
     */
    int scaling_strategy = 0;

    /**
     * Delta for convergence test.
     *  This parameter determines the minimum rate of decrease 
     *  of the objective function after serious steps. The library 
     *  stops iterations when the following condition is met:
     *      fabs(f' - f) < delta_past,
     *  where f' is the objective value of past iterations ago, 
     *  and f is the objective value of the current iteration.
     */
    double delta_past = 1.0e-8;

    /**
     * Relative epsilon for function values. 
     *  Note that with this tolerance, the computation will terminate 
     *  due to just one sufficiently small change in the function values 
     *  after a serious step. Thus, to shun premature terminations, very 
     *  large values of this parameter should be avoided. Note however, 
     *  that this parameter is indeed used to be multiplied by an epsilon 
     *  for machine precision. This parameter and corresponding termination 
     *  tests may be ignored by choosing f_rel_eps < 0.
     */
    double f_rel_eps = 1.0e+4;

    /**
     * Lower bound for funciton values.
     *  The function to be minimized should be lower bounded.
     *  This bound is explicitly required in case of unbounded minimum 
     *  function value.
     */
    double f_lower_bound = -1.0e+60;

    /**
     * Magnitute threshold for directional dervative under locality measure 
     * and metric matrix.
     */
    double terminate_param1 = 1.0e-6;

    /**
     * Magnitute threshold for gradient under locality measure (>=terminate_param1).
     *  It prevents the premature terminations arising from the limited memory matrix 
     *  updating. This second termination criterion is somewhat difficult to satisfy. 
     *  Thus, terminate_param2 >= terminate_param1 is required.
     */
    double terminate_param2 = 1.0e-6;

    /**
     * Distance measure parameter. 
     *  Used for the calculations of the subgradient locality measure. 
     *  Small values on range [0.0, 1.0e-4] are recommended for convex 
     *  problems while larger values on range [1.0e-2, 1.0] are recommended 
     *  for nonconvex problems.
     */
    double distance_measure = 0.5;

    /**
     * A parameter to control the accuracy of the line search routine.
     *  This parameter should be greater than zero and smaller than 0.25.
     */
    double sufficient_dec = 1.0e-4;

    /**
     * Maximum step size (>1.0). 
     *  This parameter reduces the step size such that it plays an important 
     *  role in the vicinity of a nonsmooth point. It may influence the 
     *  efficiency of the method considerably and, thus, it should be tuned 
     *  carefully.
     */
    double max_stepsize = 1.5;
};

enum
{
    /** LMBM converges with desired accuracy based on terminate_param1 and terminate_param2. */
    LMBM_CONVERGENCE = 0,
    /** LMBM satisfies stopping criteria. */
    LMBM_STOP,
    /** The change in function value is sufficiently small (in serious step) considering f_rel_eps. */
    LMBM_NOMOREPROGRESS,
    /** Reaches maximum number of function calls. */
    LMBM_MAXFCALLS,
    /** Reaches maximum number of iterations. */
    LMBM_MAXITERS,
    /** Reaches maximum computation time. */
    LMBM_MAXTIME,
    /** Function value reaches its lower bound. */
    LMBM_LOWBOUND,
    // 手动取消//
    LMBM_CANCLLEL,
    /** Unknown error. */
    LMBMERR_UNKNOWNERROR = -1024,
    /** Invalid input parameters whose error message can be specified by verbose. */
    LMBMERR_INVALIDPARAMS,
    /** Failure in attaining the demanded accuracy. */
    LMBMERR_ROUNDINGERR,
    /** Number of restarts is greater that the maximum number of restarts (internal parameters 2000)*/
    LMBMERR_FREQUENTRESTART,
    /** Two consecutive restarts or bad choice of maximum step size. */
    LMBMERR_CONSECUTIVEFAIL,
};

typedef double (*lmbm_evaluate_t)(void *instance,
                                  const double *x,
                                  double *g,
                                  const int n);

typedef int (*lmbm_progress_t)(void *instance,
                                const double *x,
                                const int k );
int lmbm_optimize(int n,
                  double *x,
                  double *ptr_fx,
                  lmbm_evaluate_t proc_evaluate,
                  void *instance,
                  lmbm_progress_t progress_evac,
                  lmbm_parameter_t *param);


} // namespace lmbm

#endif