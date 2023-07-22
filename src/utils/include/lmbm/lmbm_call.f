*************************************************************************
*                                                                       *
*   LMBM_CALL - Limited Memory Bundle Method for Large-Scale        *
*                   Nonsmooth Optimization                              *
*                                                                       *
*************************************************************************
*     
*
*
*
*     References:
*
*     M.Haarala, K.Miettinen and M.M.Mäkelä: "New Limited Memory Bundle 
*     Method for Large-Scale Nonsmooth Optimization". Optimization 
*     Methods and Software 19(6): 673 - 692, 2004.
*
*     N.Haarala, K.Miettinen and M.M.Mäkelä: "Globally Convergent 
*     Limited Memory Bundle Method for Large-Scale Nonsmooth 
*     Optimization". Mathematical Programming, Vol. 109, No. 1, 
*     pp. 181-205, 2007.
*
*     J.Vlcek, L.Luksan: Globally Convergent Variable Metric
*     Method for Nonconvex Nondifferentiable Unconstrained
*     Minimization. Journal of Optimization Theory and Applications
*     111(2): 407 - 430, 2001.
*
*
*
*     Original LMBM (fortran sources):
*           Napsu Karmitsa (maiden name Haarala) 2002 - 2004
*           last modified 2007      
*
*     Callback-version:
*           Seppo Pulkkinen 2009
*
*
*************************************************************************
*
*     Remark:
*
*     At the beginning of each FORTRAN- file, there is a list of the 
*     subroutines and functions included to that file. Moreover, at the 
*     beginning of each subroutine, there is a description of parameters 
*     used in the routine (at least those needed in calling sequences). 
*     The types of the parameters (or arguments) are introduced with two
*     letters. The first letter is either I for integer arguments or R 
*     for double precision real arguments.  The second letter specifies 
*     whether the argument must have a value defined on the entry to
*     the subroutine (I), whether it is a value which will be returned
*     (O), or both (U), or whether it is an auxiliary value (A). Note 
*     that the arguments of the types II and RI can be changed on output
*     under some circumstances: especially, if improper input values are
*     given or if set zero. In the latter case the default values will 
*     be used (if applicable).
*
*
*************************************************************************
*************************************************************************
*************************************************************************
*
*
*     lmbm_call.f includes the following subroutines
*
*     S   FUNDER        Computation of the value and the subgradient 
*                         of the objective function.
*      
*
************************************************************************
*      
*     * SUBROUTINE FUNDER *
*
*      
*     * Purpose *
*
*     Computation of the value and the subgradient of the objective
*     function.
*
*      
*     * Calling sequence *
*
*     CALL FUNDER(N,X,F,G,ITERM)
*
*
*     * Parameters *
*      
*     II  N             Number of variables.
*     RI  X(N)          Vector of variables.
*     RO  F             Value of the objective function.
*     RO  G(N)          Subgradient of the objective function.
*     IO  ITERM         Cause of termination:
*                          0  - Everything is ok.
*                         -3  - Failure in function or subgradient
*                               calculations (assigned by the user).
*
*
*     * Variables in COMMON /PROB/ *
*
*     I   NEXT          Number of the test problem.
*
*
*     * Subprograms used *
*      
*     S   FUNC          Computation of the value and the subgradient for
*                       problem next.
*     
*      
*     Napsu Haarala (2002-2004)
*     Callback-version Seppo Pulkkinen 2009
*      
     
      SUBROUTINE FUNDER(N,X,F,G,ITERM)

*     Scalar Arguments
      INTEGER N,ITERM
      DOUBLE PRECISION F

*     Array Arguments
      DOUBLE PRECISION G(*),X(*)

     
*     
*     Function and subgradient evaluation
*    

      CALL LMBMCB(N, X, F, G)
      ITERM = 0

      RETURN
      END

      ! SUBROUTINE EARLYEXIT(x)
      !       ! 在这里编写EARLYEXIT函数的代码
      !       ! 可以使用传递进来的x变量进行计算或其他操作
      !       ! ...
      !       WRITE (*, * ,*) "earlyexit..."
      ! END SUBROUTINE EARLYEXIT

