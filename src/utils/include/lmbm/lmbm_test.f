*************************************************************************
*                                                                       *
*     LMBM - Limited Memory Bundle Method for Large-Scale               *
*            Nonsmooth Optimization                                     *
*                                                                       *
*************************************************************************
*     
*
*     Codes included:
* 
*     tlmbm.f     - testprogram for limited memory bundle method.
*     lmbm.f      - limited memory bundle method.
*     lmsub.f     - subprograms for limited memory bundle method.
*     matcal.f    - matrix and vector calculus.
*     Makefile    - makefile.
*
*     tnsunc.f    - large-scale nonsmooth test problems.
*
*
*     References:
*
*     M.Haarala, K.Miettinen and M.M.MÃ¤kelÃ¤: "New Limited Memory Bundle 
*     Method for Large-Scale Nonsmooth Optimization". Optimization 
*     Methods and Software 19(6): 673 - 692, 2004.
*
*     N.Haarala, K.Miettinen and M.M.MÃ¤kelÃ¤: "Globally Convergent 
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
*     Napsu Karmitsa (maiden name Haarala) 2002 - 2004
*     last modified 2007      
*
*
*************************************************************************
*
*     Remark:
*
*     At the beginning of each file, there is a list of the subroutines 
*     and functions included to that file. Moreover, at the beginning 
*     of each subroutine, there is a description of parameters used in 
*     the routine (at least those needed in the calling sequence). The 
*     types of the parameters (or arguments) are introduced with two
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
*     TLMBM includes the following subroutines
*
*     P   TLMBM         Test main program for limited memory bundle
*                         method.
*     S   FUNDER        Computation of the value and the subgradient 
*                         of the objective function.
*      
*
*************************************************************************
*
*     * PROGRAM TLMBM *
*
*      
*     * Purpose *
*
*     Test program for limited memory bundle subroutine for large-scale
*     unconstrained nonsmooth optimization.
*
*     
*     * Parameters *
*
*     I   N             Number of variables.
*     I   NA            Maximum bundle dimension, NA >= 2.
*     I   MCU           Upper limit for maximum number of stored
*                         corrections, MCU >= 3.
*     I   NW            Dimension of the work vector W:
*                         NW >= 1 + 9*N + 2*N*NA + 3*NA + 2*N*(MCU+1) 
*                               + 3*(MCU+2)*(MCU+1)/2 + 9*(MCU+1)
*
*      
*     * Variables *
*      
*     I   MC            Maximum number of stored corrections,
*                         MCU >= MC >= 3.
*     R   X(N)          Vector of variables.
*     R   F             Value of the objective function.
*     R   TIME          Maximum CPU-time in seconds. If TIME <= 0.0
*                         the maximum time is ignored. REAL argument.
*     R   RTIM(2)       Auxiliary array. REAL array.
*                         On output RTIM(1) contains the CPU-time used.
*     R   RPAR(8)       Real parameters:
*           RPAR(1)       Tolerance for change of function values.
*           RPAR(2)       Second Tolerance for change of function values.
*           RPAR(3)       Tolerance for the function value.
*           RPAR(4)       Tolerance for the first termination criterion.
*           RPAR(5)       Tolerance for the second termination criterion.
*           RPAR(6)       Distance measure parameter, 0 <= RPAR(6).
*           RPAR(7)       Line search parameter, 0 < RPAR(7) < 0.25.
*           RPAR(8)       Maximum stepsize, 1 < RPAR(8).
*                           If RPAR(I) <= 0 for I=1,3,4,5,7, and 8 the
*                           default value of the parameter will be used.
*                           If RPAR(2) < 0 the the parameter and the
*                           corresponding termination criterion will be
*                           ignored. If RPAR(2) = 0 default value will
*                           be used. If RPAR(6) < 0 the default value
*                           will be used.
*     I   IPAR(7)       Integer paremeters:
*           IPAR(1)       Exponent for distance measure.
*           IPAR(2)       Maximum number of iterations.
*           IPAR(3)       Maximum number of function evaluations.
*           IPAR(4)       Maximum number of iterations with changes of
*                           function values smaller than RPAR(1).
*           IPAR(5)       Printout specification:
*                             -1  - No printout.
*                              0  - Only the error messages.
*                              1  - The final values of the objective
*                                   function.
*                              2  - The final values of the objective
*                                   function and the most serious
*                                   warning messages.
*                              3  - The whole final solution. 
*                              4  - At each iteration values of the
*                                   objective function.
*                              5  - At each iteration the whole
*                                   solution
*           IPAR(6)       Selection of the method:
*                              0  - Limited memory bundle method.
*                              1  - L-BFGS bundle method.
*           IPAR(7)       Selection of the scaling:
*                              0  - Scaling at every iteration with STU/UTU.
*                              1  - Scaling at every iteration with STS/STU.
*                              2  - Interval scaling with STU/UTU.
*                              3  - Interval scaling with STS/STU.
*                              4  - Preliminary scaling with STU/UTU.
*                              5  - Preliminary scaling with STS/STU.
*                              6  - No scaling.      
*                           If IPAR(I) <= 0 the default value of the
*                           parameter will be used.
*     I   IOUT(3)       Output integer parameters:
*           IOUT(1)       Number of used iterations.
*           IOUT(2)       Number of used function evaluations.
*           IOUT(3)       Cause of termination:
*                              1  - The problem has been solved.
*                                   with desired accuracy.
*                              2  - Changes in function values < RPAR(1)
*                                   in IPAR(4) subsequent iterations.
*                              3  - Changes in function values < RPAR(2)
*                                   *SMALL*MAX(|F_k|,|F_k+1|,1), where
*                                   SMALL is the smallest positive
*                                   number such that 1.0 + SMALL > 1.0.
*                              4  - Number of function calls > IPAR(3).
*                              5  - Number of iterations > IPAR(2).
*                              6  - Time limit exceeded. 
*                              7  - F < RPAR(3).
*                              8  - terminater by earlyexit (ztr)
*                             -1  - Two consecutive restarts or number
*                                   of restarts > maximum number of
*                                   restarts.
*                             -2  - TMAX < TMIN in two subsequent
*                                   iterations.
*                             -3  - Failure in function or subgradient
*                                   calculations (assigned by the user).
*                             -4  - Failure in attaining the demanded
*                                   accuracy.
*                             -5  - Invalid input parameters.
*                             -6  - Not enough working space.
*     R   W(NW)         Work vector.
*
*
*     * Variables in COMMON /PROB/ *
*
*     I   NEXT          Number of the test problem.
*     
*
*     * Subprograms used *
*      
*     S   LMBMU         Initialization of limited memory bundle method
*                       for nonsmooth optimization.
*     S   STARTX        Initiation of X.
*     
*      
*     Napsu Karmitsa (2002 - 2004, last modified 2007)
*     
*

      
      PROGRAM TLMBM

*     Parameters
      INTEGER N,NA,MCU,NW
      PARAMETER(
     &     N = 1000,      
     &     NA = 2,
     &     MCU = 7,
     &     NW = 1 + 9*N + 2*N*NA + 3*NA + 2*N*(MCU+1) +
     &     3*(MCU+2)*(MCU+1)/2 + 9*(MCU+1))

*     Scalar Arguments
      INTEGER MC
      DOUBLE PRECISION F

*     Array Arguments
      INTEGER IPAR(7),IOUT(3)
      DOUBLE PRECISION W(NW),X(N),RPAR(8)

*     Local Scalars
      INTEGER MCINIT,I,J
      
*     CPU-time
      REAL TIME,RTIM(2)

*     Scalars depending on problem
      INTEGER NEXT
      COMMON /PROB/NEXT
      
*     External Subroutines
      EXTERNAL LMBMU,STARTX


      TIME = 300.0E+00
c      TIME = 0.0E+00
      

*
*     Loop for test problems
*      

      DO 100 J=1,10
         

*     
*     Initial number of stored corrections
*

      MC = 7
      MCINIT = MC


*     
*     Number for the test problems
*

      NEXT=J


*     
*     Initiation of X
*

      CALL STARTX(N,X,NEXT)
      IF (NEXT .EQ. -1) GOTO 999


*     
*     Choice of integer parameters
*

      DO 10 I = 1,7
         IPAR(I) = 0
 10   CONTINUE

*     
*     Printout specification
*      
      IPAR(5) = 1

*      
*     Selection of the method
*      
      IPAR(6) = 0
      
*      
*     Selection of the scaling
*      
      IPAR(7) = 0
      
*
*     Maximum numbers of iterations and function evaluations
*      
      IPAR(2) =5000000
      IPAR(3) =5000000


*     
*     Choice of real parameters
*

      DO 20 I = 1,8
         RPAR(I) = 0.0D0
 20   CONTINUE

      
*
*     Desired accuracy
*
      RPAR(4) = 1.0D-5
c      RPAR(5) = 1.0D-3

*
*     Locality measure
*      
      RPAR(6) = 0.50D+00

*      
*     Line search parameter
*      
c      RPAR(7) = 0.01D+00

*      
*     Stepsize
*      
c      RPAR(8) = 2.0D+00



*     
*     Solution
*

      CALL LMBMU(N,NA,MCU,MC,NW,X,F,RPAR,IPAR,IOUT,TIME,RTIM,W)
    

*
*     Result (additional printout)
*      

      PRINT*
      PRINT*,'NEXT    = ',NEXT
      PRINT*,'ITERM   = ',IOUT(3)
      PRINT*
      PRINT*,'F(X)    = ',F
      PRINT*,'N       = ',N
      PRINT*,'NA      = ',NA
      PRINT*,'MCINIT  = ',MCINIT
      PRINT*,'MC      = ',MC
      PRINT*,'MCU     = ',MCU
      PRINT*,'NIT     = ',IOUT(1)
      PRINT*,'NFE     = ',IOUT(2)
      PRINT*,'XMAX    = ',RPAR(8)
      PRINT*,'GAM     = ',RPAR(6)
      PRINT*,'EPSL    = ',RPAR(7)
      PRINT*,'EPS     = ',RPAR(4)
      PRINT*,'METHOD  = ',IPAR(6)
      PRINT*,'SCALING = ',IPAR(7)
      
      PRINT* 
      PRINT*,'Used time = ',RTIM(1)
      PRINT* 

 100  CONTINUE
 999  CONTINUE

      STOP
      END


      
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
*     * CALLING SEQUENCE *
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
*
*      
     
      SUBROUTINE FUNDER(N,X,F,G,ITERM)

*     Scalar Arguments
      INTEGER N,ITERM
      DOUBLE PRECISION F

*     Array Arguments
      DOUBLE PRECISION G(*),X(*)

*     External Subroutines
      EXTERNAL FUNC

*     Common blocks
      INTEGER NEXT
      COMMON /PROB/NEXT

     
*     
*     Function and subgradient evaluation
*      

      CALL FUNC(N,X,F,G,NEXT)
      IF (NEXT .LT.1) ITERM = -3
      
      RETURN
      END
