#include "lmbm.h"


static lmbm::lmbm_evaluate_t proc_evaluate_curr = nullptr;
static lmbm::lmbm_progress_t early_exit_func    = nullptr;
static void *instance_curr = nullptr;

extern "C"
{
    int lmbmu_(int *n, int *na, int *mcu, int *mc, int *nw,
               double *x, double *f, double *rpar, int *ipar,
               int *iout, float *time, float *rtim, double *w);

    void lmbmcb_(const int *n, const double *x, double *f, double *g)
    {
        *f = proc_evaluate_curr(instance_curr, x, g, *n);
        return;
    }

    void earlyexit_(double *x, int *k,int *result)
    {
         *result=early_exit_func(instance_curr, x, *k);
         return;
    }
}

int lmbm::lmbm_optimize(int n,
                        double *x,
                        double *ptr_fx,
                        lmbm::lmbm_evaluate_t proc_evaluate,
                        void *instance,
                        lmbm::lmbm_progress_t progress_evac,
                        lmbm::lmbm_parameter_t *param)
{
    proc_evaluate_curr = proc_evaluate;
    early_exit_func    = progress_evac;
    instance_curr = instance;

    float time = param->timeout;
    int na = param->bundle_size;
    int mc = param->ini_corrections;
    int mcu = param->max_corrections;

    int ipar[7];
    ipar[0] = param->exponent_distmeasure;
    ipar[1] = param->max_iterations;
    ipar[2] = param->max_evaluations;
    ipar[3] = param->past;
    ipar[4] = param->verbose;
    ipar[5] = param->update_method;
    ipar[6] = param->scaling_strategy;

    double rpar[8];
    rpar[0] = param->delta_past;
    rpar[1] = param->f_rel_eps;
    rpar[2] = param->f_lower_bound;
    rpar[3] = param->terminate_param1;
    rpar[4] = param->terminate_param2;
    rpar[5] = param->distance_measure;
    rpar[6] = param->sufficient_dec;
    rpar[7] = param->max_stepsize;

    int nw = 1 +//dimension of the work vector W
             9 * n +
             2 * n * na +
             3 * na +
             2 * n * (mcu + 1) +
             3 * (mcu + 2) * (mcu + 1) / 2 +
             9 * (mcu + 1);

    int iout[3];
    float rtim[2];

    double *w = (double *)malloc(nw * sizeof(double));

    lmbmu_(&n, &na, &mcu, &mc, &nw, x, ptr_fx,
           rpar, ipar, iout, &time, rtim, w);

    int ret = 0;

    switch (iout[2])
    {
    case 1:
        ret = lmbm::LMBM_CONVERGENCE;
        break;
    case 2:
        ret = lmbm::LMBM_STOP;
        break;
    case 3:
        ret = lmbm::LMBM_NOMOREPROGRESS;
        break;
    case 4:
        ret = lmbm::LMBM_MAXFCALLS;
        break;
    case 5:
        ret = lmbm::LMBM_MAXITERS;
        break;
    case 6:
        ret = lmbm::LMBM_MAXTIME;
        break;
    case 7:
        ret = lmbm::LMBM_LOWBOUND;
        break;
    case 8:
        ret =lmbm::LMBM_CANCLLEL;
        break;
    case -1:
        ret = lmbm::LMBMERR_CONSECUTIVEFAIL;
        break;
    case -2:
        ret = lmbm::LMBMERR_FREQUENTRESTART;
        break;
    case -4:
        ret = lmbm::LMBMERR_ROUNDINGERR;
        break;
    case -5:
        ret = lmbm::LMBMERR_INVALIDPARAMS;
        break;
    default:
        ret = lmbm::LMBMERR_UNKNOWNERROR;
        break;
    }

    free(w);
    proc_evaluate_curr = nullptr;
    instance_curr = nullptr;

    return ret;
}
