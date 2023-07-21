************************************************************************
*
*
*     LMBM_MAIN includes the following subroutines
*
*     S   LMBMU         Initialization for limited memory bundle 
*                         subroutine.
*     S   LMBM          Limited memory bundle subroutine for nonsmooth 
*                         large-scale optimization.
*      
*
************************************************************************
*      
*     * SUBROUTINE LMBMU *
*
*      
*     * Purpose *
*
*     Initialization for limited memory bundle subroutine for
*     large-scale unconstrained nonsmooth optimization.
*
*     
*     * Calling sequence *
*     
*     CALL LMBMU(N,NA,MCU,MC,NW,X,F,RPAR,IPAR,IOUT,TIME,RTIM,W)
*     
*     
*     * Parameters *
*      
*     II  N             Number of variables.
*     II  NA            Maximum bundle dimension, NA >= 2.
*     IU  MC            Maximum number of stored corrections, MC >= 3.
*     II  MCU           Upper limit for maximum number of stored
*                         corrections, MCU >= MC.
*     RU  X(N)          Vector of variables.
*     RO  F             Value of the objective function.
*     RI  TIME          Maximum CPU-time in seconds. If TIME <= 0.0
*                         the maximum time is ignored. REAL argument.
*     RI  RTIM(2)       Auxiliary array. REAL array
*                         On output RTIM(1) contains the execution time.
*     RI  RPAR(8)       Real parameters:
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
*     II  IPAR(7)       Integer paremeters:
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
*     IO  IOUT(3)       Integer parameters:
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
*                              8  - canceled  by user(ztr)
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
*     RA  W(NW)         Work vector.
*     II  NW            Dimension of the work vector W:
*                         NW >= 1 + 9*N + 2*N*NA + 3*NA + 2*N*(MCU+1) 
*                               + 3*(MCU+2)*(MCU+1)/2 + 9*(MCU+1)
*     
*
*     * Subprograms used *
*      
*     S   LMBM         Limited memory bundle method for nonsmooth
*                        optimization.
*     S   WPRINT       Printout the error and warning messages.
*     S   GETIME       Execution time.
*
*     
*      
*     Napsu Karmitsa (2002 - 2004, last modified 2007)      
*     

      
      SUBROUTINE LMBMU(N,NA,MCU,MC,NW,X,F,RPAR,IPAR,IOUT,TIME,RTIM,W)
    
*     Scalar Arguments
      INTEGER N,NA,MC,MCU,NW
      DOUBLE PRECISION F
      
*     Array Arguments
      INTEGER IPAR(*),IOUT(*)
      DOUBLE PRECISION X(*),RPAR(*),W(*)
      
*     Local Scalars
      INTEGER LXO,LS,LG,LGP,LGA,LU,LD,LAX,LAG,LAF,LSM,LUM,LRM,LUMTUM,
     &     LC,LSMTGP,LUMTGP,LTMC1,LTMC2,LTMC3,LTMC4,LTMC5,LTMC6,LTN1,
     &     LTN2,LTMAT
      
*     External Subroutines
      EXTERNAL LMBM,WPRINT,GETIME

*     CPU-time
      REAL TIME,START,FINI
      REAL RTIM(2)
      
*     
*     CPU-time
*      

      CALL GETIME(START,RTIM)


*     
*     Initialization and error checking
*

      IOUT(3) = 0
      
      IF (N .LE. 0) THEN
         IOUT(3) = -5
         CALL WPRINT(IOUT(3),IPAR(5),1)
         RETURN
      END IF

      
      IF (MCU .LT. 3) THEN
         IOUT(3) = -5
         CALL WPRINT(IOUT(3),IPAR(5),2)
         RETURN
      END IF


      IF (NA .LT. 2) THEN
         IOUT(3) = -5
         CALL WPRINT(IOUT(3),IPAR(5),3)
         RETURN
      END IF

      
      IF (RPAR(7) .GE. 0.25D+00) THEN
         IOUT(3) = -5
         CALL WPRINT(IOUT(3),IPAR(5),4)
         RETURN
      END IF

      
      IF (NW .LT. 1 + 9*N + 2*N*NA + 3*NA + 2*N*(MCU+1) +
     &     3*(MCU+2)*(MCU+1)/2 + 9*(MCU+1)) THEN
         IOUT(3) = -6
         CALL WPRINT(IOUT(3),IPAR(5),0)
         RETURN
      END IF


      IF (IPAR(6) .GT. 1 .OR. IPAR(6) .LT. 0) IPAR(6) = 0
      IF (IPAR(7) .GT. 6 .OR. IPAR(7) .LT. 0) IPAR(7) = 2

      
      IF (MC .GT. MCU) THEN
         MC = MCU
         CALL WPRINT(IOUT(3),IPAR(5),-1)
      END IF
         
      IF (MC .LE. 0) MC = 3


*     
*     Pointers for working array W
*

      LXO    = 1
      LS     = LXO    + N
      LG     = LS     + N
      LGP    = LG     + N
      LGA    = LGP    + N
      LU     = LGA    + N
      LD     = LU     + N
      LAX    = LD     + N
      LAG    = LAX    + N*NA
      LAF    = LAG    + N*NA
      LSM    = LAF    + 3*NA
      LUM    = LSM    + N*(MCU+1)
      LRM    = LUM    + N*(MCU+1)
      LUMTUM = LRM    + (MCU+2)*(MCU+1)/2
      LC     = LUMTUM + (MCU+2)*(MCU+1)/2
      LSMTGP = LC     + MCU+1
      LUMTGP = LSMTGP + MCU+1
      LTMC1  = LUMTGP + MCU+1
      LTMC2  = LTMC1  + MCU+1
      LTMC3  = LTMC2  + MCU+1
      LTMC4  = LTMC3  + MCU+1
      LTMC5  = LTMC4  + MCU+1
      LTMC6  = LTMC5  + MCU+1
      LTN1   = LTMC6  + MCU+1
      LTN2   = LTN1   + N
      LTMAT  = LTN2   + N
      

*     
*     Solution
*

      CALL LMBM(N,NA,MC,MCU,X,W(LXO),W(LS),W(LG),W(LGP),W(LGA),W(LU),
     &     W(LD),F,W(LAX),W(LAG),W(LAF),W(LSM),W(LUM),W(LRM),W(LUMTUM),
     &     W(LC),W(LSMTGP),W(LUMTGP),W(LTMC1),W(LTMC2),W(LTMC3),W(LTMC4)
     &     ,W(LTMC5),W(LTMC6),W(LTN1),W(LTN2),W(LTMAT),RPAR(1),RPAR(2),
     &     RPAR(3),RPAR(4),RPAR(5),RPAR(6),RPAR(7),RPAR(8),IPAR(2),
     &     IPAR(3),IPAR(1),IPAR(4),IPAR(5),IPAR(6),IPAR(7),IOUT(1),
     &     IOUT(2),IOUT(3),TIME,RTIM)


*     
*     CPU-time
*      

      CALL GETIME(FINI,RTIM)
      RTIM(1) = FINI - START

      RETURN
      END



************************************************************************
*
*     * SUBROUTINE LMBM *
*
*      
*     * Purpose *
*      
*     Limited memory bundle subroutine for nonsmooth optimization.
*
*      
*     * Calling sequence *
*     
*     CALL LMBM(N,NA,MC,MCU,X,XO,S,G,GP,GA,U,D,F,AX,AG,AF,SM,UM,RM,
*    &     UMTUM,C,SMTGP,UMTGP,TMPMC1,TMPMC2,TMPMC3,TMPMC4,TMPMC5,
*    &     TMPMC6,TMPN1,TMPN2,TMPMAT,TOLF,TOLB,TOLG,TOLG2,ETA,EPSL,
*    &     XMAX,MIT,MFE,MOS,MTESF,IPRINT,METHOD,ISCALE,NIT,NFE,ITERM,
*    &     TIME,RTIM)
*
*      
*     * Parameters *
*      
*     II  N               Number of variables.
*     II  NA              Maximum bundle dimension.
*     IU  MC              Maximum number of stored corrections.
*     II  MCU             Upper limit for maximum number of stored
*                           corrections, MCU >= MC.
*     RU  X(N)            Vector of variables.
*     RA  XO(N)           Previous vector of variables.
*     RA  G(N)            Subgradient of the objective function.
*     RA  GP(N)           Previous subgradient of the objective function.
*     RA  GA(N)           Aggregate subgradient.
*     RA  S(N)            Difference of current and previous variables.
*     RA  U(N)            Difference of current and previous
*                           subgradients.
*     RA  D(N)            Search direction.
*     RO  F               Value of the objective function.
*     RA  AX(N*NA)        Matrix whose columns are bundle points.
*     RA  AG(N*NA)        Matrix whose columns are bundle subgradients.
*     RA  AF(3*NA)        Vector of bundle values.
*     RA  SM(N*(MC+1))    Matrix whose columns are stored differences of
*                           variables.
*     RA  UM(N*(MC+1))    Matrix whose columns are stored subgradient
*                           differences.
*     RA  RM((MC+2)*(MC+1)/2)  Upper triangular matrix stored columnwise
*                                in the one-dimensional array.
*     RU  UMTUM((MC+2)*(MC+1)/2)  Auxiliary matrix: TRANS(UM)*UM.
*     RU  C(MC+1)         Diagonal matrix.
*     RU  SMTGP(MC+1)     Auxiliary vector.
*     RU  UMTGP(MC+1)     Auxiliary vector.
*     RA  TMPMC#(MC+1)    Auxiliary arrays; # = 1,...,6.
*     RA  TMPN#(N)        Auxiliary arrays; # = 1,2.
*     RA  TMPMAT((MC+1)*(MC)/2)   Auxiliary matrix.
*     RI  TIME            Maximum CPU-time in seconds. If TIME <= 0.0
*                           the maximum time is ignored. REAL argument.
*     RI  RTIM(2)         Auxiliary array. REAL array.
*                           On input RTIM(1) contains the starting time.
*     RI  TOLF            Tolerance for change of function values.
*     RI  TOLF2           Second tolerance for change of function
*                           values.
*     RI  TOLB            Tolerance for the function value.
*     RI  TOLG            Tolerance for the first termination criterion.
*     RI  TOLG2           Tolerance for the second termination criterion.
*     RI  ETA             Distance measure parameter, ETA >= 0.
*     RI  EPSL            Line search parameter, 0 < EPSL < 0.25.
*     RI  XMAX            Maximum stepsize, 1 < XMAX.
*     II  MIT             Maximun number of iterations.
*     II  MFE             Maximun number of function evaluations.
*     II  MOS             Exponent for distance measure.
*     II  MTESF           Maximum number of iterations with changes of
*                           function values smaller than TOLF.
*     II  IPRINT          Printout specification:
*                            -1  - No printout.
*                             0  - Only the error messages.
*                             1  - The final values of the objective
*                                  function.
*                             2  - The final values of the objective
*                                  function and the most serious
*                                  warning messages.
*                             3  - The whole final solution. 
*                             4  - At each iteration values of the
*                                  objective function.
*                             5  - At each iteration the whole
*                                  solution
*     II  METHOD          Selection of the method:
*                             0  - Limited memory bundle method.
*                             1  - L-BFGS bundle method.
*     II  ISCALE          Selection of the scaling:
*                             0  - Scaling at every iteration
*                                  with STU/UTU.
*                             1  - Scaling at every iteration
*                                  with STS/STU.
*                             2  - Interval scaling with STU/UTU.
*                             3  - Interval scaling with STS/STU.
*                             4  - Preliminary scaling with STU/UTU.
*                             5  - Preliminary scaling with STS/STU.
*                             6  - No scaling.      
*     IO  NIT             Number of used iterations.
*     IO  NFE             Number of used function evaluations.
*     IO  ITERM           Cause of termination:
*                             1  - The problem has been solved.
*                                  with desired accuracy.
*                             2  - (F - FO) < TOLF in MTESF
*                                  subsequent iterations.
*                             3  - (F - FO) < TOLF*SMALL*MAX(|F|,|FO|,1).
*                             4  - Number of function calls > MFE.
*                             5  - Number of iterations > MIT.
*                             6  - Time limit exceeded. 
*                             7  - F < TOLB.
*                             8  - early exit cancel(ztr)
*                            -1  - Two consecutive restarts.
*                            -2  - Number of restarts > maximum number
*                                  of restarts.
*                            -3  - Failure in function or subgradient
*                                  calculations (assigned by the user).
*                            -4  - Failure in attaining the demanded
*                                  accuracy.
*                            -5  - Invalid input parameters.
*                            -6  - Not enough working space.
*
*
*     * Local parameters *
*
*     I   MAXEPS          Maximum number of consecutive equal stopping
*                           criterions.
*     I   MAXNRS          Maximum number of restarts.
*     R   ETA9            Maximum for real numbers.
*     R   FMIN            Smallest acceptable value of the function.
*     R   TMIN            Minimum stepsize.
*     R   LENGTHD         Direction vector length.
*     R   RHO             Correction parameter.
*     
*      
*     * Local variables *
*
*     I   INEW            Index for the circular arrays.
*     I   IBUN            Index for the circular arrays in bundle.
*     I   IBFGS           Index of the type of BFGS update.
*     I   ISR1            Index of the type of SR1 update.
*     I   ITERS           Null step indicator.
*                              0  - Null step.
*                              1  - Serious step.
*     I   IC              Correction indicator.
*     I   ICN             Correction indicator for null steps.
*     I   IFLAG           Index for adaptive version:
*                              0  - Maximum number of stored corrections
*                                     has not been changed.
*                              1  - Maximum number of stored corrections
*                                     has been changed.
*     I   NAC             Current size of the bundle.
*     I   MCC             Current number of stored corrections.
*     I   MCINIT          Initial maximum number of stored corrections.
*     I   NEPS            Number of consecutive equal stopping
*                           criterions.
*     I   NNK             Consecutive null steps counter.
*     I   NRES            Number of consecutive restarts.
*     I   NRESS           Number of consecutive restarts in case of
*                           TMAX<TMIN.
*     I   NCRES           Number of restars.      
*     I   NTESF           Number of tests on function decrease.
*     R   ALFN            Locality measure.
*     R   ALFV            Aggregate locality measure.
*     R   EPSR            Line search parameter.
*     R   GAMMA           Scaling parameter.
*     R   P               Directional derivative.
*     R   FO              Previous value of objective function.
*     R   DNORM           Euclidean norm of the direction vector.
*     R   GNORM           Euclidean norm of the aggregate subgradient
*                           vector.
*     R   XNORM           Stopping criterion.
*     R   PXNORM          Previous stopping criterion.
*     R   T               Stepsize.
*     R   TMAX            Maximum stepsize.
*     R   THETA           Correction parameter for stepsize.
*     R   SMALL           The smallest positive number such that
*                           1.0 + SMALL > 1.0.
*
*     
*      
*     * Subprograms used *
*      
*     S   AGBFGS          Simplified subgradient aggregation.
*     S   AGGSR1          Subgradient aggregation.
*     S   AGSKIP          Subgradient aggregation using BFGS update.
*     S   DLBFGS          Computing the search direction by limited
*                           memory BFGS update.
*     S   DLSKIP          Skipping the updates and computing the search
*                           direction by limited memory BFGS update.    
*     S   DLSR1           Computing the search direction by limited
*                           memory SR1 update.    
*     S   DOBUN           Bundle selection.
*     S   LLS             Line search using function values and
*                           derivatives.
*     S   RESTAR          Initialization.
*     S   TINIT           Calculation of initial step size.
*     S   COPY            Copying of a vector.
*     S   GETIME          Execution time.
*     S   XDIFFY          Difference of two vectors.
*     S   RPRINT          Printout the results.
*     S   WPRINT          Printout the error and warning messages.
*     RF  EPS0            The smallest positive number such that
*                           1.0 + EPS0 > 1.0. 
*     RF  VDOT            Dot product of two vectors.
*     
*
*     * EXTERNAL SUBROUTINES *
*      
*     SE  FUNDER          Computation of the value and the subgradient of
*                         the objective function. Calling sequence:
*                         CALL FUNDER(N,X,F,G,ITERM), where N is a number of
*                         variables, X(N) is a vector of variables, F is
*                         the value of the objective function, G(N) is
*                         the subgradient of the objective function, and 
*                         ITERM is the error indicator.
*
*      
*      
*     Napsu Karmitsa (2002 - 2004, last modified 2007)      
*
*
      
      SUBROUTINE LMBM(N,NA,MC,MCU,X,XO,S,G,GP,GA,U,D,F,AX,AG,AF,SM,UM,
     &     RM,UMTUM,C,SMTGP,UMTGP,TMPMC1,TMPMC2,TMPMC3,TMPMC4,TMPMC5,
     &     TMPMC6,TMPN1,TMPN2,TMPMAT,TOLF,TOLF2,TOLB,TOLG,TOLG2,ETA,
     &     EPSL,XMAX,MIT,MFE,MOS,MTESF,IPRINT,METHOD,ISCALE,NIT,NFE,
     &     ITERM,TIME,RTIM)


*     Scalar Arguments
      INTEGER N,NA,MC,MCU,MIT,MFE,MOS,MTESF,IPRINT,NIT,NFE,ITERM,METHOD
     &     ,ISCALE
      DOUBLE PRECISION F,ETA,EPSL,TOLF,TOLF2,TOLB,TOLG,TOLG2,XMAX

*     Array Arguments
      DOUBLE PRECISION X(*),XO(*),S(*),G(*),GP(*),GA(*),U(*),D(*),
     &     AX(*),AG(*),AF(*),SM(*),UM(*),RM(*),UMTUM(*),C(*),
     &     SMTGP(*),UMTGP(*),TMPMC1(*),TMPMC2(*),TMPMC3(*),TMPMC4(*),
     &     TMPMC5(*),TMPMC6(*),TMPN1(*),TMPN2(*),TMPMAT(*)

*     Local Scalars
      INTEGER I,INEW,IBFGS,ISR1,ITERS,MAL,MCC,MCINIT,NNK,NTESF,NRES,EARLYRESULT
     &     NCRES,IC,ICN,NRESS,NEPS,IFLAG,IBUN,NOUT
      DOUBLE PRECISION ALFN,ALFV,EPSR,DNORM,GNORM,XNORM,P,TMAX,T,
     &     FO,GAMMA,PXNORM,THETA,SMALL

*     External Functions
      DOUBLE PRECISION VDOT,EPS0
      EXTERNAL VDOT,EPS0

*     External Subroutines
      EXTERNAL FUNDER,COPY,XDIFFY,DLBFGS,DLSKIP,DLSR1,
     &     LLS,AGBFGS,AGGSR1,AGSKIP,DOBUN,TINIT,RESTAR,COPY2,
     &     RPRINT,WPRINT,GETIME,EARLYEXIT

*     Intrinsic Functions
      INTRINSIC ABS,MAX,SQRT

*     Computational Time
      REAL TIME,STRTIM,CTIM,RTIM(2)
       
*     Parameters
      INTEGER MAXEPS,MAXNRS
      DOUBLE PRECISION ETA9,FMIN,TMIN,LENGTHD,RHO
      PARAMETER(
     &     MAXEPS = 20,
     &     MAXNRS = 2000,
     &     ETA9 = 1.0D+60,
     &     FMIN = -1.0D+60,
     &     TMIN = 1.0D-12,
     &     LENGTHD = 1.0D+20,
     &     RHO = 1.0D-12)

      
      
      IF (IPRINT .GT. 3) THEN
         IF (METHOD .EQ. 0) WRITE (6,FMT='(1X,''Entry to LMBM:'')')
         IF (METHOD .EQ. 1) WRITE (6,FMT='(1X,''Entry to LBB:'')')
      END IF

      
*     
*     Initialization
*
      EARLYRESULT=0 
      NOUT = 0
      NIT = 0
      NFE = 0
      NTESF = 0
      NRES = 1
      NCRES = -1
      NRESS = 0
      NEPS = 0
      ITERM = 0
      ITERS = 1
      NNK = 0
      ISR1 = 0
      ALFN = 0.0D+00
      ALFV = 0.0D+00
      MCINIT=MC
      
      SMALL = EPS0()
      STRTIM = RTIM(1)

      IF (TOLF  .LE. 0.0D+00) TOLF = 1.0D-8
      IF (TOLF2 .EQ. 0.0D+00) TOLF2 = 1.0D+04
      IF (TOLB  .EQ. 0.0D+00) TOLB = FMIN + SMALL
      IF (TOLG  .LE. 0.0D+00) TOLG = 1.0D-06
      IF (TOLG2 .LE. 0.0D+00) TOLG2 = TOLG
      IF (XMAX  .LE. 0.0D+00) XMAX = 1.5D+00
      IF (ETA   .LT. 0.0D+00) ETA = 0.50D+00
      IF (EPSL  .LE. 0.0D+00) EPSL = 1.0D-04
      IF (MOS   .LE. 0) MOS = 2
      IF (MTESF .LE. 0) MTESF = 10
      IF (MIT   .LE. 0) MIT = 10000
      IF (MFE   .LE. 0) MFE = 20000
      
      TMAX = XMAX
      GNORM = 0.0D+00
      XNORM = ETA9

      EPSR = 0.25D+00+SMALL
      IF (2.0D+00*EPSL .GE. EPSR) THEN
         EPSR = 2.0D+00*EPSL + SMALL
         IF (EPSR .GE. 0.5D+00) THEN
            CALL WPRINT(ITERM,IPRINT,-2)
         END IF
      END IF
            

*     
*     Computation of the value and the subgradient of the objective
*     function and the search direction for the first iteration
*      

      CALL FUNDER(N,X,F,G,ITERM)
      NFE = NFE + 1

      IF (ITERM .NE. 0) GOTO 900

      CALL RESTAR(N,MC,MCC,MCINIT,INEW,IBUN,IBFGS,ITERS,GP,G,NNK,
     &     ALFV,ALFN,GAMMA,D,IC,ICN,MAL,NCRES,IFLAG)
      
      CALL DOBUN(N,NA,MAL,X,G,F,AX,AG,AF,ITERS,IBUN)

      GOTO 200


*     
*     Start of the iteration
*

 100  CONTINUE

      
*     
*     Serious step initialization
*

      IF (ITERS.GT.0) THEN
         ICN = 0
         ALFN = 0.0D+00
         ALFV = 0.0D+00
      END IF

      
*     
*     Direction finding
*

      IF (ITERS.GT.0) THEN

         
*     
*     BFGS update and direction determination
*

         CALL DLBFGS(N,MC,MCC,INEW,IBFGS,IFLAG,D,G,GP,S,U,SM,UM,RM,
     &        UMTUM,C,SMTGP,UMTGP,GAMMA,TMPMC1,TMPMC2,TMPMC3,
     &        TMPMC4,TMPN1,SMALL,METHOD,ISCALE)

      ELSE
         IF (METHOD .EQ. 0) THEN
            
*     
*     SR1 update and direction determination
*

            CALL DLSR1(N,MC,MCC,INEW,ISR1,IFLAG,D,GP,GA,S,U,SM,UM,RM,
     &           UMTUM,C,SMTGP,UMTGP,GAMMA,TMPMC1,TMPMC2,TMPMC3,TMPMC4,
     &           TMPMC5,TMPMC6,TMPN1,TMPN2,TMPMAT,NNK,SMALL,IPRINT)
            IBFGS=0
         ELSE

            
*     
*     BFGS skipping and direction determination
*

            CALL DLSKIP(N,MC,MCC,INEW,IBFGS,IFLAG,D,GA,SM,UM,RM,
     &           UMTUM,C,TMPMC1,TMPMC2,GAMMA,TMPMC3,TMPMC4,TMPMC5,
     &           TMPN1,ISCALE)

         END IF
      END IF

            
 200  CONTINUE

      
*
*     Computational time
*

      IF (TIME .GT. 0.0E+00) THEN
         CALL GETIME(CTIM,RTIM)
         IF (CTIM-STRTIM .GT. TIME) THEN
            ITERM = 6
            GOTO 900
         END IF
      END IF


*
*     Computation of norms
*

      IF (ITERS .GT. 0) THEN
         GNORM = VDOT(N,G,G)
         DNORM = SQRT(VDOT(N,D,D))

         P = VDOT(N,G,D)

      ELSE
         GNORM = VDOT(N,GA,GA)
         DNORM = SQRT(VDOT(N,D,D))

         P = VDOT(N,GA,D)
      END IF


*     
*     Test on descent direction
*

      IF (P+SMALL*SQRT(GNORM)*DNORM .LE. 0.0D+00) THEN
         NRES = 0

      ELSE
         NRES = NRES + 1
         IF (NRES .EQ. 1) THEN
            CALL WPRINT(ITERM,IPRINT,-3)
           
            CALL RESTAR(N,MC,MCC,MCINIT,INEW,IBUN,IBFGS,ITERS,GP,G,NNK,
     &           ALFV,ALFN,GAMMA,D,IC,ICN,MAL,NCRES,IFLAG)
            IF (NCRES .GT. MAXNRS) THEN
               NOUT = MAXNRS
               ITERM = -2
               GO TO 900
            END IF

            CALL DOBUN(N,NA,MAL,X,G,F,AX,AG,AF,ITERS,IBUN)
            
            GOTO 200
         END IF
         NOUT = -1
         ITERM = -1
         GOTO 900
      END IF

      
*      
*     Stopping criterion
*      

      NIT = NIT + 1
      PXNORM = XNORM
      XNORM = -P + 2.0D+00*ALFV
**************************ztr***********************************
      CALL EARLYEXIT(X,NIT,EARLYRESULT)
      IF (EARLYRESULT .GT. 0) THEN
         ITERM = 8
         CALL WPRINT(ITERM,IPRINT,NOUT)
         GOTO 900
      END IF
**************************ztr***********************************

*     
*     Tests for termination
*

      IF (XNORM .LE. 1.0D+03*TOLG .AND.
     &     (MCC .GT. 0 .OR. IBFGS .EQ. 2)) THEN

         IF(0.5D+00*GNORM + ALFV .LE. TOLG2 .AND.
     &        XNORM .LE. TOLG) THEN

            ITERM = 1
            GOTO 900
         END IF
         
         IF (MC .LT. MCU .AND. IFLAG .EQ. 0) THEN
            MC=MC+1
            IFLAG=1
         END IF
      END IF


      IF (NFE.GE.MFE) THEN
         NOUT = MFE
         ITERM = 4
         GOTO 900
      END IF

      
      IF (NIT.GE.MIT) THEN
         NOUT = MIT
         ITERM = 5
         GOTO 900
      END IF

      
      IF (F.LE.TOLB) THEN
         ITERM = 7
         GOTO 900
      END IF

      
      IF (ITERS .EQ. 0) THEN
         IF (ABS(XNORM - PXNORM) .LE. SMALL) THEN
            NEPS = NEPS + 1

            IF (NEPS .GT. MAXEPS) THEN
               ITERM = -4
               GOTO 900
            END IF

         ELSE
            NEPS = 0
         END IF

      ELSE
         NEPS = 0
      END IF


*
*     Correction
*

      IF (-P .LT. RHO*GNORM .OR. ICN .EQ. 1) THEN

         XNORM =XNORM + RHO*GNORM
         DNORM=SQRT(DNORM*DNORM-2*RHO*P+RHO*RHO*GNORM)
         
         IF (ITERS .GT. 0) THEN
            DO 230 I=1,N
               D(I)=D(I)-RHO*G(I)
 230        CONTINUE

         ELSE
            DO 240 I=1,N
               D(I)=D(I)-RHO*GA(I)
 240        CONTINUE
            ICN = 1
         END IF

         IC=1

      ELSE
         IC=0
      END IF


      IF (PXNORM .LT. XNORM .AND. NNK .GT. 2) THEN
         CALL WPRINT(ITERM,IPRINT,-4)
      END IF
      

      CALL RPRINT(N,NIT,NFE,X,F,XNORM,0.5D+00*GNORM+ALFV,ITERM,IPRINT)
      
      
*     
*     Preparation of line search
*

      FO = F
      
      IF (ITERS .GT. 0) THEN
         CALL COPY2(N,X,XO,G,GP)
      END IF

      IF (DNORM.GT.0.0D+00) TMAX = XMAX/DNORM
      
      IF (TMAX .GT. TMIN) THEN
         NRESS = 0

      ELSE
         NRESS = NRESS + 1
         IF (NRESS .EQ. 1) THEN
            CALL WPRINT(ITERM,IPRINT,-5)

            CALL RESTAR(N,MC,MCC,MCINIT,INEW,IBUN,IBFGS,ITERS,GP,G,NNK,
     &           ALFV,ALFN,GAMMA,D,IC,ICN,MAL,NCRES,IFLAG)

            IF (NCRES .GT. MAXNRS) THEN
               NOUT = MAXNRS
               ITERM = -2
               GO TO 900
            END IF
            
            CALL DOBUN(N,NA,MAL,X,G,F,AX,AG,AF,ITERS,IBUN)
            
            GOTO 200
         END IF
         ITERM = -1
         GOTO 900
      END IF

      
*
*     Initial step size
*

      CALL TINIT(N,NA,MAL,X,AF,AG,AX,IBUN,D,F,P,T,TMAX,TMIN,
     &     ETA,ETA9,MOS,ITERS)

      
*     
*     Line search with directional derivatives which allows null steps
*

      THETA=1.0D+00
      IF (DNORM .GT. LENGTHD) THEN
         THETA=LENGTHD/DNORM
      END IF
      
      CALL LLS(N,X,G,D,XO,T,FO,F,P,ALFN,TMIN,
     &     DNORM,XNORM,THETA,EPSL,EPSR,ETA,MOS,ITERS,NFE,NNK,ITERM)

      IF (ITERM .NE. 0) GOTO 900

      IF (TOLF2 .GE. 0) THEN
         IF (ABS(FO-F) .LE. TOLF2*SMALL*MAX(ABS(F),ABS(FO),1.0D+00)
     &        .AND. ITERS .EQ. 1) THEN
         
            ITERM = 3
            GOTO 900
         
         END IF
      END IF

      IF (ABS(FO-F) .LE. TOLF) THEN
         NTESF = NTESF + 1
         
         IF (NTESF .GE. MTESF .AND. ITERS .EQ. 1) THEN
            ITERM = 2
            GOTO 900
         END IF
         
      ELSE
         NTESF = 0
      END IF
      
      
*
*     Bundle updating
*      

      CALL DOBUN(N,NA,MAL,X,G,F,AX,AG,AF,ITERS,IBUN)

      
*
*     Computation of variables difference 
*

      CALL XDIFFY(N,X,XO,S)

      
*
*     Computation of aggregate values and gradients difference
*

      IF (ITERS.EQ.0) THEN
         NNK = NNK + 1

         IF (NNK.EQ.1) THEN
            CALL COPY(N,GP,TMPN1)
            CALL XDIFFY(N,G,GP,U)
            CALL AGBFGS(N,MC,MCC,INEW,IBFGS,IFLAG,G,GP,GA,U,D,SM,UM,
     &           RM,C,UMTUM,ALFN,ALFV,GAMMA,TMPMC1,TMPMC2,IC,RHO)
            
         ELSE
            IF (METHOD .EQ. 0) THEN
               CALL COPY(N,GA,TMPN1)
               CALL AGGSR1(N,MC,MCC,INEW,IFLAG,G,GP,GA,D,ALFN,ALFV
     &              ,TMPMAT,UMTUM,RM,GAMMA,SMTGP,UMTGP,TMPMC1,TMPMC2,SM
     &              ,UM,TMPMC3,TMPMC4,TMPN2,X,U,ICN,RHO,SMALL)
               CALL XDIFFY(N,G,GP,U)
            ELSE
               CALL COPY(N,GA,TMPN1)
               CALL XDIFFY(N,G,GP,U)
               CALL AGSKIP(N,MC,MCC,INEW,IFLAG,G,GP,GA,D,U,ALFN,ALFV
     &              ,UMTUM,RM,C,GAMMA,SMTGP,UMTGP,TMPMC1,TMPMC2,SM
     &              ,UM,TMPMC3,TMPMC4,TMPN2,ICN,RHO)
            END IF
         END IF

         CALL COPY(N,XO,X)
         F = FO
         
      ELSE
         IF (NNK .NE. 0) THEN
            CALL COPY(N,GA,TMPN1)
         ELSE
            CALL COPY(N,GP,TMPN1)
         END IF
         NNK = 0
         CALL XDIFFY(N,G,GP,U)
      END IF

      GOTO 100

      
 900  CONTINUE

      
*
*     Printout the final results
*

      IF (IPRINT .GT. 3) THEN
         IF (METHOD .EQ. 0) WRITE (6,FMT='(1X,''Exit from LMBM:'')')
         IF (METHOD .EQ. 1) WRITE (6,FMT='(1X,''Exit from LBB:'')')
      END IF
      CALL WPRINT(ITERM,IPRINT,NOUT)
      CALL RPRINT(N,NIT,NFE,X,F,XNORM,0.5D+00*GNORM+ALFV,ITERM,IPRINT)
      
      RETURN
      END

************************************************************************
      
