************************************************************************
*
*
*     LMBM_SUB includes the following subroutines
*
*     S   AGBFGS          Simplified subgradient aggregation.
*     S   AGGSR1          Subgradient aggregation.
*     S   AGSKIP          Subgradient aggregation using BFGS update.
*     S   DESTEP          Stepsize determination for descent steps.
*     S   DLBFGS          Computing the search direction by limited
*                           memory BFGS update.
*     S   DLSKIP          Skipping the updates and computing the search
*                           direction by limited memory BFGS update.    
*     S   DLSR1           Computing the search direction by limited
*                           memory SR1 update.    
*     S   DOBUN           Bundle selection.
*     S   GETIME          Execution time.
*     S   INDIC1          Initialization of indices.
*     S   LLS             Line search using function values and
*                           derivatives.
*     S   NULSTP          Stepsize determination for null steps.
*     S   QINT            Quadratic interpolation for line search
*                           with one directional derivative.
*     S   RESTAR          Initialization.
*     S   RPRINT          Printout the results.
*     S   WPRINT          Printout the error and warning messages.
*     S   TINIT           Calculation of initial step size.
*      
*     RF  SCLPAR          Calculation of the scaling parameter.
*
*
************************************************************************
*
*     * SUBROUTINE DLBFGS *
*
*      
*     * Purpose *
*      
*     Matrix update and computation of the search direction D = -DM*G
*     by the limited memory BFGS update.
*
*      
*     * Calling sequence *
*     
*     CALL DLBFGS(N,MC,MCC,INEW,IBFGS,IFLAG,D,G,GP,S,U,SM,UM,R,
*    &     UMTUM,C,SMTGP,UMTGP,GAMMA,TMPMC1,TMPMC2,TMPMC3,TMPMC4,
*    &     TMPN1,SMALL,METHOD,ISCALE)
*
*      
*     * Parameters *
*
*     II  N               Number of variables.
*     II  MC              Declared number of stored corrections.
*     IU  MCC             Current number of stored corrections.
*     IU  INEW            Index for circular arrays.
*     IO  IBFGS           Index of the type of BFGS update:
*                             1  - BFGS update: the corrections are
*                                    stored.
*                             2  - BFGS update: the corrections are not
*                                    stored.
*                             3  - BFGS update is skipped.
*     IU  IFLAG           Index for adaptive version:
*                             0  - Maximum number of stored corrections
*                                    has not been changed at previous
*                                    iteration.
*                             1  - Maximum number of stored corrections
*                                    has been changed at previous
*                                    iteration.
*     RO  D(N)            Search direction.
*     RI  G(N)            Current subgradient of the objective
*                           function.
*     RI  GP(N)           Previous subgradient of the objective
*                           function.
*     RI  S(N)            Difference of current and previous variables.
*     RI  U(N)            Difference of current and previous
*                           subgradients.
*     RU  SM(N*(MC+1))    Matrix whose columns are stored corrections.
*     RU  UM(N*(MC+1))    Matrix whose columns are stored subgradient
*                           differences.
*     RU  R((MC+2)*(MC+1)/2)  Upper triangular matrix stored columnwise
*                               in the one-dimensional array.
*     RU  UMTUM((MC+2)*(MC+1)/2)  Matrix UMTUM = TRANS(UM) * UM.
*     RU  C(MC+1)         Diagonal matrix.
*     RU  SMTGP(MC+1)     Vector SMTGP = TRANS(SM)*GP.
*     RU  UMTGP(MC+1)     Vector UMTGP = TRANS(UM)*GP.
*     RU  GAMMA           Scaling parameter.
*     RA  TMPMC#(MC+1)    Auxiliary arrays; # = 1,...,4.
*     RA  TMPN1(N)        Auxiliary array:
*                           On input: Previous aggregate subgradient.
*     RI  SMALL           Small positive value.
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
*     
*     
*     * Local variables *
*
*     I   MCNEW           Current size of vectors.
*     I   IOLD            Index of the oldest corrections.
*     R   STU             STU = TRANS(S)*U. 
*     R   STS             STS = TRANS(S)*S. 
*     I   IFLAG2          Index for adaptive version.
*
*
*     * Subprograms used *
*      
*     S   COPY2           Copying of two vectors.
*     S   INDIC1          Initialization of indices.
*     S   XDIFFY          Difference of two vectors.
*     S   VXDIAG          Multiplication of a vector and a diagonal 
*                           matrix.
*     S   VNEG            Copying of a vector with change of the sign
*     S   XSUMY           Sum of two vectors.
*     S   SCSUM           Sum of a vector and the scaled vector.
*     S   SCDIFF          Difference of the scaled vector and a vector.
*     S   CWMAXV          Multiplication of a vector by a dense
*                           rectangular matrix.
*     S   RWAXV2          Multiplication of two rowwise stored dense 
*                           rectangular matrices A and B by vectors X 
*                           and Y.
*     S   SYMAX           Multiplication of a dense symmetric matrix
*                           by a vector.
*     S   TRLIEQ          Solving X from linear equation L*X=Y or
*                           TRANS(L)*X=Y.
*     
*     RF  VDOT            Dot product of two vectors.
*     RF  SCLPAR          Calculation of the scaling parameter.
*
*
*     The variable and subgradient differences and all the MC-vectors are
*     stored in a circular order controlled by the parameter point inew.
*
*      
*
*     Napsu Karmitsa (2002,2003, last modified 2007)
*

     
      SUBROUTINE DLBFGS(N,MC,MCC,INEW,IBFGS,IFLAG,D,G,GP,S,U,SM,UM,R,
     &     UMTUM,C,SMTGP,UMTGP,GAMMA,TMPMC1,TMPMC2,TMPMC3,TMPMC4,
     &     TMPN1,SMALL,METHOD,ISCALE)
      
*     Scalar Arguments
      INTEGER N,MC,MCC,INEW,IBFGS,IFLAG,METHOD,ISCALE
      DOUBLE PRECISION GAMMA,SMALL
      
*     Array Arguments
      DOUBLE PRECISION D(*),G(*),GP(*),S(*),U(*),SM(*),UM(*),R(*),
     &     UMTUM(*),C(*),SMTGP(*),UMTGP(*),TMPMC1(*),TMPMC2(*),
     &     TMPMC3(*),TMPMC4(*),TMPN1(*)      

*     Local Scalars
      INTEGER I,J,K,MCNEW,IOLD,IFLAG2,IERR
      DOUBLE PRECISION STU,STS

*     External Functions
      DOUBLE PRECISION VDOT,SCLPAR
      EXTERNAL VDOT,SCLPAR

*     Intrinsic Functions
      INTRINSIC SQRT, MIN, MAX

*     External Subroutines
      EXTERNAL XDIFFY,VXDIAG,VNEG,XSUMY,CWMAXV,RWAXV2,
     &     SYMAX,TRLIEQ,SCSUM,SCDIFF,COPY2,INDIC1

      
      IBFGS = 0
      IFLAG2 = 0
      STU = VDOT(N,S,U)
      STS = VDOT(N,S,S)

      
*
*     Positive definiteness
*

      IF (STU .GT. 0.0D+00) THEN
         
         IF (-VDOT(N,D,U)-VDOT(N,TMPN1,S) .LT. -SMALL .OR. METHOD
     $        .EQ. 1) THEN

            
*     
*     Update matrices
*         

            IBFGS = 1


*
*     Initialization of indices.
*            

            CALL INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,IBFGS)

            
*     
*     Update SM and UM
*

            CALL COPY2(N,S,SM((INEW-1)*N+1),U,UM((INEW-1)*N+1))

            
*     
*     Computation of TRANS(SM)*G and TRANS(UM)*G
*

            IF (INEW .GE. MCNEW) THEN
               CALL RWAXV2(N,MCNEW,SM((INEW-MCNEW)*N+1),
     &              UM((INEW-MCNEW)*N+1),G,G,TMPMC1(IOLD),TMPMC2(IOLD))

            ELSE
               CALL RWAXV2(N,INEW,SM,UM,G,G,TMPMC1,TMPMC2)
               CALL RWAXV2(N,MCNEW-INEW,SM((IOLD-1)*N+1),
     &              UM((IOLD-1)*N+1),G,G,TMPMC1(IOLD),TMPMC2(IOLD))
            END IF               
            
            
*         
*     Computation of TRANS(SM)*U and TRANS(UM)*U
*

            IF (INEW .GE. MCNEW) THEN
               DO 30 I=IOLD,INEW-1
                  TMPMC3(I)=TMPMC1(I) - SMTGP(I)
                  SMTGP(I)=TMPMC1(I)
                  TMPMC4(I)=TMPMC2(I) - UMTGP(I)
                  UMTGP(I)=TMPMC2(I)
 30            CONTINUE

            ELSE
               DO 10 I=1,INEW-1
                  TMPMC3(I)=TMPMC1(I) - SMTGP(I)
                  SMTGP(I)=TMPMC1(I)
                  TMPMC4(I)=TMPMC2(I) - UMTGP(I)
                  UMTGP(I)=TMPMC2(I)
 10            CONTINUE

               DO 20 I=IOLD,MCNEW+1
                  TMPMC3(I)=TMPMC1(I) - SMTGP(I)
                  SMTGP(I)=TMPMC1(I)
                  TMPMC4(I)=TMPMC2(I) - UMTGP(I)
                  UMTGP(I)=TMPMC2(I)
 20            CONTINUE
            END IF
            
            TMPMC3(INEW)=TMPMC1(INEW) - VDOT(N,S,GP)
            SMTGP(INEW)=TMPMC1(INEW)
            TMPMC4(INEW)=TMPMC2(INEW) - VDOT(N,U,GP)
            UMTGP(INEW)=TMPMC2(INEW)

            
*         
*     Update R and UMTUM
*

            IF (MCC .GE. MC .AND. IFLAG2 .NE. 1) THEN
               DO 40 I=1,MCNEW-1
                  J=(I-1)*I/2+1
                  K=I*(I+1)/2+2
                  CALL COPY2(I,R(K),R(J),UMTUM(K),UMTUM(J))
 40            CONTINUE
            END IF

            
            IF (INEW .GE. MCNEW) THEN
               CALL COPY2(MCNEW,TMPMC3(IOLD),R((MCNEW-1)*MCNEW/2+1),
     &              TMPMC4(IOLD),UMTUM((MCNEW-1)*MCNEW/2+1))

            ELSE
               CALL COPY2(MCNEW-INEW,TMPMC3(IOLD),R((MCNEW-1)*MCNEW/2+1)
     &              ,TMPMC4(IOLD),UMTUM((MCNEW-1)*MCNEW/2+1))

               CALL COPY2(INEW,TMPMC3,R((MCNEW-1)*MCNEW/2+MCNEW-INEW+1),
     &              TMPMC4,UMTUM((MCNEW-1)*MCNEW/2+MCNEW-INEW+1))
            END IF
            

            
*
*     Update C
*

            C(INEW) = STU
            
            
*         
*     Computation of GAMMA
*

            GAMMA = SCLPAR(MCC,ISCALE,METHOD,STS,STU,TMPMC4(INEW),SMALL)

            
            INEW = INEW + 1
            IF (INEW .GT. MC + 1) INEW = 1
            IF (IFLAG .EQ. 0 .AND. MCC .LT. MC + 1) MCC = MCC + 1

         ELSE

            
*     
*     BFGS update, corrections are not saved.
*     

            IBFGS = 2

*
*     Initialization of indices.
*            

            CALL INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,IBFGS)

          
*     
*     Update SM and UM
*     

            CALL COPY2(N,S,SM((INEW-1)*N+1),U,UM((INEW-1)*N+1))
            
*     
*     Computation of TRANS(SM)*G and TRANS(UM)*G
*

            CALL RWAXV2(N,MCNEW,SM,UM,G,G,TMPMC1,TMPMC2)

            
*         
*     Computation of TRANS(SM)*U and TRANS(UM)*U
*

            IF (IOLD .NE. 1) THEN
               DO 50 I=1,INEW-1
                  TMPMC3(I)=TMPMC1(I) - SMTGP(I)
                  SMTGP(I)=TMPMC1(I)
                  TMPMC4(I)=TMPMC2(I) - UMTGP(I)
                  UMTGP(I)=TMPMC2(I)
 50            CONTINUE

               DO 60 I=IOLD,MCNEW
                  TMPMC3(I)=TMPMC1(I) - SMTGP(I)
                  SMTGP(I)=TMPMC1(I)
                  TMPMC4(I)=TMPMC2(I) - UMTGP(I)
                  UMTGP(I)=TMPMC2(I)
 60            CONTINUE

            ELSE
               DO 70 I=1,MCNEW-1
                  TMPMC3(I)=TMPMC1(I) - SMTGP(I)
                  SMTGP(I)=TMPMC1(I)
                  TMPMC4(I)=TMPMC2(I) - UMTGP(I)
                  UMTGP(I)=TMPMC2(I)
 70            CONTINUE
               
            END IF

            TMPMC3(INEW)=TMPMC1(INEW) - VDOT(N,S,GP)
            SMTGP(INEW)=TMPMC1(INEW)
            TMPMC4(INEW)=TMPMC2(INEW) - VDOT(N,U,GP)
            UMTGP(INEW)=TMPMC2(INEW)

            
*         
*     Update R and UMTUM
*

            IF (IOLD .NE. 1) THEN
               
               CALL COPY2(MCNEW-INEW,TMPMC3(IOLD),
     &              R((MCNEW-1)*MCNEW/2+1),TMPMC4(IOLD),
     &              UMTUM((MCNEW-1)*MCNEW/2+1))

               CALL COPY2(INEW,TMPMC3,
     &              R((MCNEW-1)*MCNEW/2+MCNEW-INEW+1),TMPMC4,
     &              UMTUM((MCNEW-1)*MCNEW/2+MCNEW-INEW+1))
            
            ELSE

               CALL COPY2(MCNEW,TMPMC3,R((MCNEW-1)*MCNEW/2+1)
     &              ,TMPMC4,UMTUM((MCNEW-1)*MCNEW/2+1))
            END IF


           
*
*     Update C
*

            C(INEW) = STU
            
            
*         
*     Computation of GAMMA
*

            GAMMA = SCLPAR(MCC,ISCALE,METHOD,STS,STU,TMPMC4(INEW),SMALL)

               
         END IF

      ELSE

         
*     
*     BFGS update is skipped
*     

         IBFGS = 3

         IF (MCC .EQ. 0) THEN
            IFLAG = 0
            CALL VNEG(N,G,D)
            RETURN
         END IF
         
*
*     Initialization of indices.
*            

         CALL INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,IBFGS)


*         
*     Computation of GAMMA
*

         IF (ISCALE .GE. 4) GAMMA=1.0D+00
               

         
*         
*     Computation of TRANS(SM)*G and TRANS(UM)*G
*

         IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
            CALL RWAXV2(N,MCNEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),G,G,
     &           SMTGP(IOLD),UMTGP(IOLD))

         ELSE
            CALL RWAXV2(N,INEW-1,SM,UM,G,G,SMTGP,UMTGP)
            CALL RWAXV2(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),
     &           UM((IOLD-1)*N+1),G,G,SMTGP(IOLD),UMTGP(IOLD))
         END IF

      END IF

      
*
*     Computation of two intermediate values TMPMC1 and TMPMC2
*

      IF (IOLD .EQ. 1 .OR. IBFGS .EQ. 2) THEN
         CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC1,SMTGP,1,IERR)
         
         CALL SYMAX(MCNEW,MCNEW,IOLD,UMTUM,TMPMC1,TMPMC3)
         CALL VXDIAG(MCNEW,C,TMPMC1,TMPMC2)
         CALL SCSUM(MCNEW,GAMMA,TMPMC3,TMPMC2,TMPMC2)
         CALL SCSUM(MCNEW,-GAMMA,UMTGP,TMPMC2,TMPMC3)
         CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC2,TMPMC3,0,IERR)

      ELSE IF (IFLAG .EQ. 0) THEN
         CALL TRLIEQ(MCNEW,MC+1,IOLD,R,TMPMC1,SMTGP,1,IERR)
         CALL SYMAX(MCNEW,MC+1,IOLD,UMTUM,TMPMC1,TMPMC3)
         CALL VXDIAG(MC+1,C,TMPMC1,TMPMC2)
         CALL SCSUM(MC+1,GAMMA,TMPMC3,TMPMC2,TMPMC2)
         CALL SCSUM(MC+1,-GAMMA,UMTGP,TMPMC2,TMPMC3)
         CALL TRLIEQ(MCNEW,MC+1,IOLD,R,TMPMC2,TMPMC3,0,IERR)

      ELSE
         CALL TRLIEQ(MCNEW,MC,IOLD,R,TMPMC1,SMTGP,1,IERR)
         CALL SYMAX(MCNEW,MC,IOLD,UMTUM,TMPMC1,TMPMC3)
         CALL VXDIAG(MC,C,TMPMC1,TMPMC2)
         CALL SCSUM(MC,GAMMA,TMPMC3,TMPMC2,TMPMC2)
         CALL SCSUM(MC,-GAMMA,UMTGP,TMPMC2,TMPMC3)
         CALL TRLIEQ(MCNEW,MC,IOLD,R,TMPMC2,TMPMC3,0,IERR)
      END IF

      
*
*     Computation of the search direction D
*

      IF (IOLD .EQ. 1 .OR. IBFGS .EQ. 2) THEN
         CALL CWMAXV(N,MCNEW,UM,TMPMC1,D,1.0D+00)
        
      ELSE 
         CALL CWMAXV(N,INEW-1,UM,TMPMC1,D,1.0D+00)
         CALL CWMAXV(N,MCNEW-INEW+1,UM((IOLD-1)*N+1),TMPMC1(IOLD),TMPN1,
     &        1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
      END IF

      
      CALL XDIFFY(N,D,G,D)
      
      IF (IOLD .EQ. 1 .OR. IBFGS .EQ. 2) THEN
         CALL CWMAXV(N,MCNEW,SM,TMPMC2,TMPN1,1.0D+00)
         CALL SCDIFF(N,GAMMA,D,TMPN1,D)

      ELSE
         CALL CWMAXV(N,INEW-1,SM,TMPMC2,TMPN1,1.0D+00)
         CALL SCDIFF(N,GAMMA,D,TMPN1,D)
         CALL CWMAXV(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),TMPMC2(IOLD),TMPN1,
     &        1.0D+00)
         CALL XDIFFY(N,D,TMPN1,D)
      END IF

      RETURN
      END


************************************************************************
*
*     * SUBROUTINE DLSKIP *
*
*      
*     * Purpose *
*      
*     Matrix skipping and computation of the search direction D = -DM*GA
*     by the limited memory BFGS update.
*
*      
*     * Calling sequence *
*     
*     CALL DLSKIP(N,MC,MCC,INEW,IBFGS,IFLAG,D,GA,SM,UM,R,UMTUM,C,TMPMC1,
*    &     TMPMC2,GAMMA,TMPMC3,TMPMC4,TMPMC5,TMPN1,ISCALE)
*
*      
*     * Parameters *
*
*     II  N               Number of variables.
*     II  MC              Declared number of stored corrections.
*     IU  MCC             Current number of stored corrections.
*     IU  INEW            Index for circular arrays.
*     IO  IBFGS           Index of the type of BFGS update:
*                             1  - BFGS update: the corrections are
*                                    stored.
*                             2  - BFGS update: the corrections are not
*                                    stored.
*                             3  - BFGS update is skipped.
*     II  IFLAG           Index for adaptive version:
*                             0  - Maximum number of stored corrections
*                                    has not been changed at previous
*                                    iteration.
*                             1  - Maximum number of stored corrections
*                                    has been changed at previous
*                                    iteration.
*     RO  D(N)            Search direction.
*     RI  GA(N)           Current aggregate subgradient.
*     RU  SM(N*(MC+1))    Matrix whose columns are stored corrections.
*     RU  UM(N*(MC+1))    Matrix whose columns are stored subgradient
*                           differences.
*     RU  R((MC+2)*(MC+1)/2)  Upper triangular matrix stored columnwise
*                               in the one-dimensional array.
*     RU  UMTUM((MC+2)*(MC+1)/2)  Matrix UMTUM = TRANS(UM) * UM.
*     RU  C(MC+1)         Diagonal matrix.
*     RU  GAMMA           Scaling parameter.
*     RA  TMPMC1(MC+1)    Auxiliary array:
*                           On output: TRANS(SM)*GA.
*     RA  TMPMC2(MC+1)    Auxiliary array: 
*                           On output: TRANS(UM)*GA.
*     RA  TMPMC#(MC+1)    Auxiliary arrays; # = 3-5.
*     RA  TMPN1(N)        Auxiliary array:
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
*
*     
*     * Local variables *
*
*     I   MCNEW           Current size of vectors.
*     I   IOLD            Index of the oldest corrections.
*
*     
*     * Subprograms used *
*      
*     S   XDIFFY          Difference of two vectors.
*     S   VXDIAG          Multiplication of a vector and a diagonal 
*                           matrix.
*     S   VNEG            Copying of a vector with change of the sign
*     S   XSUMY           Sum of two vectors.
*     S   CWMAXV          Multiplication of a vector by a dense
*                           rectangular matrix.
*     S   RWAXV2          Multiplication of two rowwise stored dense 
*                           rectangular matrices A and B by vectors X 
*                           and Y.
*     S   SCSUM           Sum of a vector and the scaled vector.
*     S   SCDIFF          Difference of the scaled vector and a vector.
*     S   SYMAX           Multiplication of a dense symmetric matrix
*                           by a vector.
*     S   TRLIEQ          Solving X from linear equation L*X=Y or
*                           TRANS(L)*X=Y.
*     S   INDIC1          Initialization of indices.
*     
*
*     The variable and subgradient differences and all the MC-vectors are
*     stored in a circular order controlled by the parameter point inew.
*
*      
*
*     Napsu Karmitsa (2004, last modified 2007)
*

      SUBROUTINE DLSKIP(N,MC,MCC,INEW,IBFGS,IFLAG,D,GA,SM,UM,R,
     &     UMTUM,C,TMPMC1,TMPMC2,GAMMA,TMPMC3,TMPMC4,TMPMC5,TMPN1,
     &     ISCALE)
      
   
*     Scalar Arguments
      INTEGER N,MC,MCC,INEW,IBFGS,IFLAG,ISCALE
      DOUBLE PRECISION GAMMA
      
*     Array Arguments
      DOUBLE PRECISION D(*),GA(*),SM(*),UM(*),R(*),
     &     UMTUM(*),C(*),TMPMC1(*),TMPMC2(*),TMPMC3(*),TMPMC4(*),
     &     TMPMC5(*),TMPN1(*)      

*     Local Scalars
      INTEGER MCNEW,IOLD,IERR

*     External Subroutines
      EXTERNAL XDIFFY,VXDIAG,VNEG,XSUMY,CWMAXV,RWAXV2,
     &     SYMAX,TRLIEQ,SCSUM,SCDIFF,INDIC1

      
*     
*     BFGS update is skipped
*

      IBFGS = 3

      IF (MCC .EQ. 0) THEN
         IFLAG = 0
         CALL VNEG(N,GA,D)
         RETURN
      END IF
      
   
*
*     Initialization of indices.
*            

      CALL INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,IBFGS)

      
*         
*     Computation of TRANS(SM)*GA and TRANS(UM)*GA
*

      IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
         CALL RWAXV2(N,MCNEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),GA,GA,
     &        TMPMC1(IOLD),TMPMC2(IOLD))
         
      ELSE
         CALL RWAXV2(N,INEW-1,SM,UM,GA,GA,TMPMC1,TMPMC2)
         CALL RWAXV2(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),
     &        GA,GA,TMPMC1(IOLD),TMPMC2(IOLD))
      END IF


*         
*     Computation of GAMMA
*

      IF (ISCALE .GE. 4) GAMMA=1.0D+00


*     
*     Computation of two intermediate values TMPMC3 and TMPMC4
*

      IF (IOLD .EQ. 1) THEN
         CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC3,TMPMC1,1,IERR)
         CALL SYMAX(MCNEW,MCNEW,IOLD,UMTUM,TMPMC3,TMPMC5)
         CALL VXDIAG(MCNEW,C,TMPMC3,TMPMC4)
         CALL SCSUM(MCNEW,GAMMA,TMPMC5,TMPMC4,TMPMC4)
         CALL SCSUM(MCNEW,-GAMMA,TMPMC2,TMPMC4,TMPMC5)
         CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC4,TMPMC5,0,IERR)

      ELSE IF (IFLAG .EQ. 0) THEN
         CALL TRLIEQ(MCNEW,MC+1,IOLD,R,TMPMC3,TMPMC1,1,IERR)
         CALL SYMAX(MCNEW,MC+1,IOLD,UMTUM,TMPMC3,TMPMC5)
         CALL VXDIAG(MC+1,C,TMPMC3,TMPMC4)
         CALL SCSUM(MC+1,GAMMA,TMPMC5,TMPMC4,TMPMC4)
         CALL SCSUM(MC+1,-GAMMA,TMPMC2,TMPMC4,TMPMC5)
         CALL TRLIEQ(MCNEW,MC+1,IOLD,R,TMPMC4,TMPMC5,0,IERR)

      ELSE
         CALL TRLIEQ(MCNEW,MC,IOLD,R,TMPMC3,TMPMC1,1,IERR)
         CALL SYMAX(MCNEW,MC,IOLD,UMTUM,TMPMC3,TMPMC5)
         CALL VXDIAG(MC,C,TMPMC3,TMPMC4)
         CALL SCSUM(MC,GAMMA,TMPMC5,TMPMC4,TMPMC4)
         CALL SCSUM(MC,-GAMMA,TMPMC2,TMPMC4,TMPMC5)
         CALL TRLIEQ(MCNEW,MC,IOLD,R,TMPMC4,TMPMC5,0,IERR)
      END IF

      
*
*     Computation of the search direction D
*

      IF (IOLD .EQ. 1) THEN
         CALL CWMAXV(N,MCNEW,UM,TMPMC3,D,1.0D+00)
        
      ELSE 
         CALL CWMAXV(N,INEW-1,UM,TMPMC3,D,1.0D+00)
         CALL CWMAXV(N,MCNEW-INEW+1,UM((IOLD-1)*N+1),TMPMC3(IOLD),TMPN1,
     &        1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
      END IF

      
      CALL XDIFFY(N,D,GA,D)

      
      IF (IOLD .EQ. 1) THEN
         CALL CWMAXV(N,MCNEW,SM,TMPMC4,TMPN1,1.0D+00)
         CALL SCDIFF(N,GAMMA,D,TMPN1,D)

      ELSE
         CALL CWMAXV(N,INEW-1,SM,TMPMC4,TMPN1,1.0D+00)
         CALL SCDIFF(N,GAMMA,D,TMPN1,D)
         CALL CWMAXV(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),TMPMC4(IOLD),TMPN1,
     &        1.0D+00)
         CALL XDIFFY(N,D,TMPN1,D)
      END IF

      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE DLSR1 *
*
*      
*     * Purpose *
*      
*     Matrix update and computation of the search direction D = -DM*GA
*     by the limited memory SR1 update.
*
*      
*     * Calling sequence *
*     
*     CALL DLSR1(N,MC,MCC,INEW,ISR1,IFLAG,D,GP,GA,S,U,SM,UM,R,UMTUM,C,
*    &     SMTGP,UMTGP,GAMMA,TMPMC1,TMPMC2,TMPMC3,TMPMC4,TMPMC5,TMPMC6,
*    &     TMPN1,TMPN2,TMPMAT,NNK,SMALL,IPRINT)
*     
*     
*     * Parameters *
*
*     II  N               Number of variables.
*     II  MC              Declared number of stored corrections.
*     IU  MCC             Current number of stored corrections.
*     IU  INEW            Index for circular arrays.
*     IO  ISR1            Index of the type of SR1 update.
*                           1  - SR1 update: the corrections are stored.
*                           3  - SR1 update is skipped.
*     IU  IFLAG           Index for adaptive version:
*                             0  - Maximum number of stored corrections
*                                    has not been changed at previous
*                                    iteration.
*                             1  - Maximum number of stored corrections
*                                    has been changed at previous
*                                    iteration.
*     RO  D(N)            Search direction.
*     RI  GP(N)           Basic subgradient of the objective function.
*     RI  GA(N)           Current aggregate subgradient.
*     RI  S(N)            Difference of auxiliary and current variables.
*     RI  U(N)            Difference of auxiliary and current
*                           subgradients.
*     RU  SM(N*(MC+1))    Matrix whose columns are stored corrections.
*     RU  UM(N*(MC+1))    Matrix whose columns are stored subgradient
*                           differences.
*     RU  R((MC+2)*(MC+1)/2)  Upper triangular matrix stored columnwise
*                               in the one-dimensional array.
*     RU  UMTUM((MC+2)*(MC+1)/2)  Matrix UMTUM = TRANS(UM) * UM.
*     RU  C(MC+1)         Diagonal matrix.
*     RU  SMTGP(MC+1)     Vector SMTGP = TRANS(SM)*GP.
*     RU  UMTGP(MC+1)     Vector UMTGP = TRANS(UM)*GP.
*     RU  GAMMA           Scaling parameter.
*     RA  TMPMC1(MC+1)    Auxiliary array:
*                           On output: TRANS(SM)*GA.
*     RA  TMPMC2(MC+1)    Auxiliary array: 
*                           On output: TRANS(UM)*GA.
*     RA  TMPMC#(MC+1)    Auxiliary arrays: # = 3,6.
*     RA  TMPN1(N)        Auxiliary array:
*                           On input: previous aggregate subgradient.
*     RA  TMPN2(N)        Auxiliary array.
*     RA  TMPMAT((MC+1)*(MC)/2)  Auxiliary matrix.
*     II  NNK             Consecutive null steps counter.
*     RI  SMALL           Small positive value.
*     II  IPRINT          Printout specification.
*     
*     
*     * Local variables *
*
*     I   MCNEW           Current size of vectors.
*     I   IOLD            Index of the oldest corrections.
*     R   STU             STU = TRANS(S)*U. 
*     I   IFLAG2          Index for adaptive version.
*     R   A               TRANS(GA) DM_(K-1) GA.
*     R   B               TRANS(GA) DM_K GA.
*
*      
*     * Subprograms used *
*      
*     RF  VDOT            Dot product of two vectors.
*     S   COPY            Copying of a vector.
*     S   XDIFFY          Difference of two vectors.
*     S   VNEG            Copying of a vector with change of the sign
*     S   SCALEX          Scaling a vector.
*     S   XSUMY           Sum of two vectors.
*     S   CWMAXV          Multiplication of a vector by a dense
*                           rectangular matrix.
*     S   RWAXV2          Multiplication of two rowwise stored dense 
*                           rectangular matrices A and B by vectors X 
*                           and Y.
*     S   CALQ            Solving X from linear equation A*X=Y.
*     S   SCDIFF          Difference of the scaled vector and a vector.
*     S   COPY2           Copying of two vectors.
*     S   INDIC1          Initialization of indices.
*     
*
*     The variable and subgradient differences and all the MC-vectors are
*     stored in a circular order controlled by the parameter point inew.
*
*     
*
*     Napsu Karmitsa (2002 - 2004, last modified 2007)
*      
  

      SUBROUTINE DLSR1(N,MC,MCC,INEW,ISR1,IFLAG,D,GP,GA,S,U,SM,UM,R,
     &     UMTUM,C,SMTGP,UMTGP,GAMMA,TMPMC1,TMPMC2,TMPMC3,TMPMC4,
     &     TMPMC5,TMPMC6,TMPN1,TMPN2,TMPMAT,NNK,SMALL,IPRINT)
      
*     Scalar Arguments
      INTEGER N,MC,MCC,INEW,ISR1,IFLAG,NNK,IPRINT
      DOUBLE PRECISION GAMMA,SMALL
      
*     Array Arguments
      DOUBLE PRECISION D(*),GP(*),GA(*),S(*),U(*),SM(*),UM(*),
     &     R(*),UMTUM(*),C(*),SMTGP(*),UMTGP(*),TMPMC1(*),
     &     TMPMC2(*),TMPMC3(*),TMPMC4(*),TMPMC5(*),TMPMC6(*),
     &     TMPN1(*),TMPN2(*),TMPMAT(*)
      
*     Local Scalars
      INTEGER I,J,K,MCNEW,IOLD,IFLAG2
      DOUBLE PRECISION STU,A,B
      
*     External Functions
      DOUBLE PRECISION VDOT
      EXTERNAL VDOT
 
*     External Subroutines
      EXTERNAL CWMAXV,RWAXV2,COPY,VNEG,XSUMY,XDIFFY,SCALEX,CALQ,
     &     SCDIFF,COPY2,INDIC1


      IFLAG2 = 0
      ISR1 = 0 

      
*
*     Initialization of indices
*      

      CALL INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,3)

      
*         
*     Computation of GAMMA 
*

      GAMMA = 1.0D+00


*     
*     Computation of TRANS(SM)*GA and TRANS(UM)*GA
*

      IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
         CALL RWAXV2(N,MCNEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),GA,GA,
     &        TMPMC1(IOLD),TMPMC2(IOLD))

      ELSE
         CALL RWAXV2(N,INEW-1,SM,UM,GA,GA,TMPMC1,TMPMC2)
         CALL RWAXV2(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),
     &        GA,GA,TMPMC1(IOLD),TMPMC2(IOLD))
      END IF

      
*
*     Positive definiteness
*

      IF (-VDOT(N,D,U) - VDOT(N,TMPN1,S) .GE. -SMALL) THEN

      
*     
*     SR1 update is skipped
*     

         ISR1 = 3

         IF (MCC .EQ. 0) THEN
            IFLAG = 0
            CALL VNEG(N,GA,D)
            RETURN
         END IF

         GO TO 200

      END IF

      
      STU = VDOT(N,S,U)

      
      TMPMC1(INEW) = VDOT(N,S,GA)
      TMPMC2(INEW) = VDOT(N,U,GA)

      
*
*     Convergence conditions
*

      IF (NNK .EQ. 1 .OR. MCC .LT. MC) GO TO 100

      IF (IFLAG .EQ. 1 .AND. (INEW .EQ. 1 .OR. INEW .EQ. MC)) GO TO 100

      
*
*     Calculation of matrix (UMTUM-R-TRANS(R)+C) from previous iteration
*      

      DO 10 I=1,MCNEW*(MCNEW+1)/2
         TMPMAT(I)= GAMMA * UMTUM(I) - R(I)
 10   CONTINUE

      
*     
*     Computation of TMPMAT*TMPMC4 = GAMMA*TRANS(UM)*GA-TRANS(SM)*GA
*

      IF (IOLD .EQ. 1) THEN
         CALL SCDIFF(MCNEW,GAMMA,TMPMC2,TMPMC1,TMPMC5)
         CALL CALQ(MCNEW,MCNEW,IOLD,TMPMAT,TMPMC4,TMPMC5,SMALL,0)

      ELSE IF (IFLAG .EQ. 0) THEN
         CALL SCDIFF(MC+1,GAMMA,TMPMC2,TMPMC1,TMPMC5)
         CALL CALQ(MCNEW,MC+1,IOLD,TMPMAT,TMPMC4,TMPMC5,SMALL,0)

      ELSE
         CALL SCDIFF(MC,GAMMA,TMPMC2,TMPMC1,TMPMC5)
         CALL CALQ(MCNEW,MC,IOLD,TMPMAT,TMPMC4,TMPMC5,SMALL,0)

      END IF

      
*
*     Computation of A = -TRANS(GA)*DM_(K-1)*GA
*
      
      IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
         CALL SCALEX(MCNEW,GAMMA,TMPMC4(IOLD),TMPMC3(IOLD))
         CALL CWMAXV(N,MCNEW,SM((IOLD-1)*N+1),TMPMC4(IOLD),TMPN1,
     &        1.0D+00)
         CALL SCDIFF(N,-GAMMA,GA,TMPN1,TMPN2)
         CALL CWMAXV(N,MCNEW,UM((IOLD-1)*N+1),TMPMC3(IOLD),TMPN1,
     &        1.0D+00)
         CALL XSUMY(N,TMPN2,TMPN1,TMPN2)

      ELSE
         CALL SCALEX(MCC,GAMMA,TMPMC4,TMPMC3)
         CALL CWMAXV(N,INEW-1,SM,TMPMC4,TMPN1,1.0D+00)
         CALL SCDIFF(N,-GAMMA,GA,TMPN1,TMPN2)
         CALL CWMAXV(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),TMPMC4(IOLD),
     &        TMPN1,1.0D+00)
         CALL XDIFFY(N,TMPN2,TMPN1,TMPN2)
         CALL CWMAXV(N,INEW-1,UM,TMPMC3,TMPN1,1.0D+00)
         CALL XSUMY(N,TMPN2,TMPN1,TMPN2)
         CALL CWMAXV(N,MCNEW-INEW+1,UM((IOLD-1)*N+1),TMPMC3(IOLD),
     &        TMPN1,1.0D+00)
         CALL XSUMY(N,TMPN2,TMPN1,TMPN2)
      END IF

      
      A = VDOT(N,GA,TMPN2)

      
      
      IF (IFLAG .EQ. 0) THEN
         MCNEW = MC
         IOLD = INEW + 2
         IF (IOLD .GT. MC+1) IOLD = IOLD - MC - 1
         
      ELSE
         MCNEW = MC - 1
         IOLD = INEW + 2
         IF (IOLD .GT. MC) IOLD = IOLD - MC
         
      END IF

      
*
*     Calculation of the new canditate for search direction
*     Updates are not necessarily saved
*
      
*     
*     Update SM and UM
*

      CALL COPY2(N,S,SM((INEW-1)*N+1),U,UM((INEW-1)*N+1))

            
*     
*     Computation of TRANS(SM)*U and TRANS(UM)*U
*

      IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
         CALL RWAXV2(N,MCNEW-1,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),U,U,
     &        TMPMC3(IOLD),TMPMC4(IOLD))

      ELSE
         CALL RWAXV2(N,INEW-1,SM,UM,U,U,TMPMC3,TMPMC4)
         CALL RWAXV2(N,MCNEW-INEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),U,U,
     &        TMPMC3(IOLD),TMPMC4(IOLD))
         
      END IF
       
      TMPMC3(INEW) = STU
      TMPMC4(INEW) = VDOT(N,U,U)


*
*     Calculation of matrix (UMTUM-R-TRANS(R)+C) without updating
*     matrices R, UMTUM and C
*      

      DO 20 I=1,MCNEW*(MCNEW+1)/2
         TMPMAT(I)= GAMMA * UMTUM(I) - R(I)
 20   CONTINUE

      DO 30 I=1,MCNEW-1
         J=(I-1)*I/2+1
         K=I*(I+1)/2+2
         CALL COPY(I,TMPMAT(K),TMPMAT(J))
 30   CONTINUE

      
      CALL SCDIFF(MCNEW+1,GAMMA,TMPMC4,TMPMC3,TMPMC5)

      IF (INEW .GE. MCNEW) THEN
         CALL COPY(MCNEW,TMPMC5(IOLD),
     &        TMPMAT((MCNEW-1)*MCNEW/2+1))

      ELSE
         CALL COPY(MCNEW-INEW,TMPMC5(IOLD),
     &        TMPMAT((MCNEW-1)*MCNEW/2+1))
         CALL COPY(INEW,TMPMC5,
     &        TMPMAT((MCNEW-1)*MCNEW/2+MCNEW-INEW+1))
      END IF
            
      
      IF (IFLAG .EQ. 0) THEN
         CALL SCDIFF(MC+1,GAMMA,TMPMC2,TMPMC1,TMPMC5)
         CALL CALQ(MCNEW,MC+1,IOLD,TMPMAT,TMPMC5,TMPMC5,SMALL,IPRINT)

      ELSE
         CALL SCDIFF(MC,GAMMA,TMPMC2,TMPMC1,TMPMC5)
         CALL CALQ(MCNEW,MC,IOLD,TMPMAT,TMPMC5,TMPMC5,SMALL,IPRINT)

      END IF

      
*
*     Calculation of the new canditate for search direction D = -DM_K*GA
*     and computation of B = -TRANS(GA)*DM_K*GA
*
      
      IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
         CALL SCALEX(MCNEW,GAMMA,TMPMC5(IOLD),TMPMC6(IOLD))
         CALL CWMAXV(N,MCNEW,SM((IOLD-1)*N+1),TMPMC5(IOLD),TMPN1,
     &        1.0D+00)
         CALL SCDIFF(N,-GAMMA,GA,TMPN1,D)
         CALL CWMAXV(N,MCNEW,UM((IOLD-1)*N+1),TMPMC6(IOLD),TMPN1,
     &        1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
         
      ELSE
         CALL SCALEX(MCNEW+1,GAMMA,TMPMC5,TMPMC6)
         CALL CWMAXV(N,INEW,SM,TMPMC5,TMPN1,1.0D+00)
         CALL SCDIFF(N,-GAMMA,GA,TMPN1,D)
         CALL CWMAXV(N,MCNEW-INEW,SM((IOLD-1)*N+1),TMPMC5(IOLD),
     &        TMPN1,1.0D+00)
         CALL XDIFFY(N,D,TMPN1,D)
         CALL CWMAXV(N,INEW,UM,TMPMC6,TMPN1,1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
         CALL CWMAXV(N,MCNEW-INEW,UM((IOLD-1)*N+1),TMPMC6(IOLD),
     &        TMPN1,1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
      END IF


      B = VDOT(N,GA,D)


*
*     Checking the convergence conditions
*      

      IF (B - A .LT. 0.0D+00) THEN
         ISR1 = 3
         CALL COPY(N,TMPN2,D)
            
      ELSE

         ISR1 = 1

         
*     
*     Update TRANS(SM)*GP and TRANS(UM)*GP
*     

         SMTGP(INEW) = VDOT(N,S,GP)
         UMTGP(INEW) = VDOT(N,U,GP)

            
*         
*     Update R and UMTUM
*
         DO 40 I=1,MCNEW-1
            J=(I-1)*I/2+1
            K=I*(I+1)/2+2
            CALL COPY2(I,R(K),R(J),UMTUM(K),UMTUM(J))
 40      CONTINUE


         IF (INEW .GE. MCNEW) THEN
            CALL COPY2(MCNEW,TMPMC3(IOLD),
     &           R((MCNEW-1)*MCNEW/2+1),TMPMC4(IOLD),
     &           UMTUM((MCNEW-1)*MCNEW/2+1))

         ELSE
            CALL COPY2(MCNEW-INEW,TMPMC3(IOLD),
     &           R((MCNEW-1)*MCNEW/2+1),TMPMC4(IOLD),
     &           UMTUM((MCNEW-1)*MCNEW/2+1))
            CALL COPY2(INEW,TMPMC3,
     &           R((MCNEW-1)*MCNEW/2+MCNEW-INEW+1),TMPMC4,
     &           UMTUM((MCNEW-1)*MCNEW/2+MCNEW-INEW+1))
         END IF
            
          
*
*     Update C
*

         C(INEW) = STU

         
         INEW = INEW + 1
         IF (INEW .GT. MC + 1) INEW = 1
         IF (IFLAG .EQ. 0 .AND. MCC .LT. MC + 1) MCC = MCC + 1

      END IF

      GO TO 300

      
 100  CONTINUE

      ISR1 = 1
      
 
*
*     Initialization of indices
*      

      CALL INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,1)        

      IF (IFLAG2 .EQ. 1 .AND. IOLD .EQ. 2) THEN
         TMPMC1(INEW) = TMPMC1(1)
         TMPMC2(INEW) = TMPMC2(1)
      END IF

            
*     
*     Update SM and UM
*

      CALL COPY2(N,S,SM((INEW-1)*N+1),U,UM((INEW-1)*N+1))

            
*         
*     Update TRANS(SM)*GP and TRANS(UM)*GP
*

      SMTGP(INEW) = VDOT(N,S,GP)
      UMTGP(INEW) = VDOT(N,U,GP)

      
*     
*     COMPUTATION OF TRANS(SM)*U and TRANS(UM)*U
*

      IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
         CALL RWAXV2(N,MCNEW-1,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),U,U,
     &        TMPMC3(IOLD),TMPMC4(IOLD))

      ELSE
         CALL RWAXV2(N,INEW-1,SM,UM,U,U,TMPMC3,TMPMC4)
         CALL RWAXV2(N,MCNEW-INEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),U,U,
     &        TMPMC3(IOLD),TMPMC4(IOLD))
         
      END IF

      TMPMC3(INEW) = STU
      TMPMC4(INEW) = VDOT(N,U,U)

      
*         
*     Update R and UMTUM
*

      IF (MCC .GE. MC .AND. IFLAG2 .NE. 1) THEN
         DO 110 I=1,MCNEW-1
            J=(I-1)*I/2+1
            K=I*(I+1)/2+2
            CALL COPY2(I,R(K),R(J),UMTUM(K),UMTUM(J))
 110     CONTINUE
      END IF


      IF (INEW .GE. MCNEW) THEN
         CALL COPY2(MCNEW,TMPMC3(IOLD),R((MCNEW-1)*MCNEW/2+1),
     &        TMPMC4(IOLD),UMTUM((MCNEW-1)*MCNEW/2+1))

      ELSE
         CALL COPY2(MCNEW-INEW,TMPMC3(IOLD),
     &        R((MCNEW-1)*MCNEW/2+1),TMPMC4(IOLD),
     &        UMTUM((MCNEW-1)*MCNEW/2+1))
         CALL COPY2(INEW,TMPMC3,
     &        R((MCNEW-1)*MCNEW/2+MCNEW-INEW+1),TMPMC4,
     &        UMTUM((MCNEW-1)*MCNEW/2+MCNEW-INEW+1))
      END IF

      
*
*     Update C
*

      C(INEW) = STU

      INEW = INEW + 1
      IF (INEW .GT. MC + 1) INEW = 1
      IF (IFLAG .EQ. 0 .AND. MCC .LT. MC + 1) MCC = MCC + 1

      
 200  CONTINUE

      DO 210 I=1,MCNEW*(MCNEW+1)/2
         TMPMAT(I)= GAMMA * UMTUM(I) - R(I)
 210  CONTINUE

      
*     
*     Computation of TMPMAT*TMPMC4 = GAMMA*TRANS(UM)*GA-TRANS(SM)*GA
*

      IF (IOLD .EQ. 1) THEN
         CALL SCDIFF(MCNEW,GAMMA,TMPMC2,TMPMC1,TMPMC4)
         CALL CALQ(MCNEW,MCNEW,IOLD,TMPMAT,TMPMC4,TMPMC4,SMALL,IPRINT)
         
      ELSE IF (IFLAG .EQ. 0) THEN
         CALL SCDIFF(MC+1,GAMMA,TMPMC2,TMPMC1,TMPMC4)
         CALL CALQ(MCNEW,MC+1,IOLD,TMPMAT,TMPMC4,TMPMC4,SMALL,IPRINT)

      ELSE
         CALL SCDIFF(MC,GAMMA,TMPMC2,TMPMC1,TMPMC4)
         CALL CALQ(MCNEW,MC,IOLD,TMPMAT,TMPMC4,TMPMC4,SMALL,IPRINT)
      END IF

      
*
*     Computation of the search direction D
*
      
      IF (IOLD .EQ. 1 .OR. IOLD .EQ. 2) THEN
         CALL SCALEX(MCNEW,GAMMA,TMPMC4(IOLD),TMPMC3(IOLD))
         CALL CWMAXV(N,MCNEW,SM((IOLD-1)*N+1),TMPMC4(IOLD),TMPN1,
     &        1.0D+00)
         CALL SCDIFF(N,-GAMMA,GA,TMPN1,D)
         CALL CWMAXV(N,MCNEW,UM((IOLD-1)*N+1),TMPMC3(IOLD),TMPN1,
     &        1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
         
      ELSE
         CALL SCALEX(MCC,GAMMA,TMPMC4,TMPMC3)
         CALL CWMAXV(N,INEW-1,SM,TMPMC4,TMPN1,1.0D+00)
         CALL SCDIFF(N,-GAMMA,GA,TMPN1,D)
         CALL CWMAXV(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),TMPMC4(IOLD),
     &        TMPN1,1.0D+00)
         CALL XDIFFY(N,D,TMPN1,D)
         CALL CWMAXV(N,INEW-1,UM,TMPMC3,TMPN1,1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
         CALL CWMAXV(N,MCNEW-INEW+1,UM((IOLD-1)*N+1),TMPMC3(IOLD),
     &        TMPN1,1.0D+00)
         CALL XSUMY(N,D,TMPN1,D)
      END IF
      
      
 300  CONTINUE
      
      RETURN
      END
      
      
************************************************************************
*
*     * SUBROUTINE AGBFGS *
*
*      
*     * Purpose *
*      
*     Computation of aggregate values by the limited memory BFGS update.
*
*      
*     * Calling sequence *
*     
*     CALL AGBFGS(N,MC,MCC,INEW,IBFGS,IFLAG,G,GP,GA,U,D,SM,UM,R,C,UMTUM,
*     &      ALFN,ALFV,GAMMA,TMPMC1,TMPMC2,IC,RHO)
*     
*     
*     * Parameters *
*
*     II  N               Number of variables.
*     II  MC              Declared number of stored corrections.
*     II  MCC             Current number of stored corrections.
*     II  INEW            Index for circular arrays.
*     II  IBFGS           Index of the type of BFGS update.
*     IU  IFLAG           Index for adaptive version:
*                             0  - Maximum number of stored corrections
*                                    has not been changed at this
*                                    iteration.
*                             1  - Maximum number of stored corrections
*                                    has been changed at this iteration.
*     RI  ALFN            Locality measure.
*     RO  ALFV            Aggregate locality measure.
*     RI  D(N)            Search direction.
*     RI  G(N)            Current (auxiliary) subgradient of the
*                           objective function.
*     RI  GP(N)           Previous subgradient and also aggregate
*                           subradient of the objective function
*     RO  GA(N)           Next aggregate subgradient of the objective
*                           function.
*     RI  U(N)            Difference of trial and aggregate gradients.
*     RI  SM(N*(MC+1))    Matrix whose columns are stored corrections.
*     RI  UM(N*(MC+1))    Matrix whose columns are stored subgradient
*                           differences.
*     RI  R((MC+2)*(MC+1)/2)  Upper triangular matrix.
*     RI  UMTUM((MC+2)*(MC+1)/2)  Matrix UMTUM = TRANS(UM) * UM.
*     RI  C(MC+1)         Diagonal matrix.
*     RI  GAMMA           Scaling parameter.
*     RA  TMPMC#(MC+1)    Auxiliary arrays; # = 1,2.
*     II  IC              Correction indicator.
*     RI  RHO             Correction parameter.
*
*     
*     * Local variables *
*      
*     I   MCNEW           Current size of vectors.
*     I   IOLD            Index of the oldest corrections.
*     R   P               P = TRANS(D)*U - ALFN.
*     R   Q               Q = TRANS(U)*DM*U, where DM is the inverse
*                            approximation of the hessian calculated
*                            by using the L-BFGS formula.
*     R   LAM             Multiplier used to calculate aggregate
*                            values.
*     R   W               Correction.
*      
*     
*     * Subprograms used *
*      
*     RF  VDOT            Dot product of two vectors.
*     S   SYMAX           Multiplication of a dense symmetric matrix
*                           by a vector.
*     S   RWAXV2          Multiplication of two rowwise stored dense 
*                           rectangular matrices A and B by vectors X 
*                           and Y.
*     S   TRLIEQ          Solving X from linear equation L*X=Y or
*                           TRANS(L)*X=Y.
*     
*
*     The variable and subgradient differences and all the MC-vectors are
*     stored in a circular order controlled by the parameter point inew.
*
*      
*
*     Napsu Karmitsa (2002,2003, last modified 2007)
*
      
      
      SUBROUTINE AGBFGS(N,MC,MCC,INEW,IBFGS,IFLAG,G,GP,GA,U,D,SM,UM,
     &     R,C,UMTUM,ALFN,ALFV,GAMMA,TMPMC1,TMPMC2,IC,RHO)

*     Scalar Arguments
      INTEGER N,MC,MCC,INEW,IBFGS,IFLAG,IC
      DOUBLE PRECISION GAMMA,ALFN,ALFV,RHO
      
*     Array Arguments
      DOUBLE PRECISION G(*),GP(*),GA(*),U(*),D(*),SM(*),UM(*),
     &     R(*),C(*),UMTUM(*),TMPMC1(*),TMPMC2(*)

*     Local Scalars
      INTEGER I,MCNEW,IOLD,IERR
      DOUBLE PRECISION P,Q,LAM,W

*     External Functions
      DOUBLE PRECISION VDOT
      EXTERNAL VDOT

*     Intrinsic Functions
      INTRINSIC MAX,MIN,SIGN

*     External Subroutines
      EXTERNAL RWAXV2,SYMAX,TRLIEQ
      

      IF (MCC .LT. MC) THEN
         IF (IBFGS .EQ.2) THEN
            MCNEW = MCC + 1

         ELSE
            MCNEW = MCC
         END IF

         IOLD = 1

      ELSE
         IF (IFLAG .EQ. 0) THEN
            IF (IBFGS .EQ. 2) THEN
               MCNEW = MC + 1
            ELSE
               MCNEW = MC
            END IF

            IOLD = INEW + 1
            IF (IOLD .GT. MC+1) IOLD = 1

         ELSE
            IF (IBFGS .EQ. 2) THEN
               MCNEW = MC
            ELSE
               MCNEW = MC - 1
            END IF

            IOLD = INEW + 1
            IF (IOLD .GT. MC) IOLD = 1
         END IF 

      END IF 
      
      
*     
*     Computation of TRANS(D) * U - ALFN
*

      P = VDOT(N,D,U) - ALFN
      Q = VDOT(N,U,U)

      IF (IC .EQ. 1) THEN
         W = RHO * Q
      ELSE
         W = 0.0D+00
      END IF
         
        
*     
*     Computation of the product TRANS(U)*DM*U
*

      IF (MCC .GT. 0 .OR. IBFGS .EQ. 2) THEN

         IF (IOLD .EQ. 1 .OR. IBFGS .EQ. 2) THEN
            CALL RWAXV2(N,MCNEW,SM,UM,U,U,TMPMC1,TMPMC2)
            CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC1,TMPMC1,1,IERR)

            Q = Q - 2*VDOT(MCNEW,TMPMC2,TMPMC1)
            Q = GAMMA*Q
            
            DO 10 I=1,MCNEW
               TMPMC2(I) = C(I)*TMPMC1(I)
 10         CONTINUE

            Q = Q + VDOT(MCNEW,TMPMC1,TMPMC2)

            CALL SYMAX(MCNEW,MCNEW,IOLD,UMTUM,TMPMC1,TMPMC2)

            Q = Q + GAMMA*VDOT(MCNEW,TMPMC1,TMPMC2)

         ELSE
            CALL RWAXV2(N,INEW-1,SM,UM,U,U,TMPMC1,TMPMC2)
            CALL RWAXV2(N,MCC-INEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),
     &           U,U,TMPMC1(IOLD),TMPMC2(IOLD))
            CALL TRLIEQ(MCNEW,MCC,IOLD,R,TMPMC1,TMPMC1,1,IERR)

            Q = Q - 2*(VDOT(MCC-INEW,TMPMC2(IOLD),TMPMC1(IOLD)) + 
     &           VDOT(INEW-1,TMPMC2,TMPMC1))
            Q = GAMMA*Q

            DO 20 I=1,MCC
               TMPMC2(I) = C(I)*TMPMC1(I)
 20         CONTINUE

            Q = Q + VDOT(MCC-INEW,TMPMC1(IOLD),TMPMC2(IOLD)) + 
     &           VDOT(INEW-1,TMPMC1,TMPMC2)

            CALL SYMAX(MCNEW,MCC,IOLD,UMTUM,TMPMC1,TMPMC2)

            Q = Q + GAMMA*(VDOT(MCC-INEW,TMPMC1(IOLD),TMPMC2(IOLD)) + 
     &           VDOT(INEW-1,TMPMC1,TMPMC2))
         END IF

      END IF


      Q = Q + W
      
      LAM = 0.5D+00 + SIGN(0.5D+00,P)
      IF (Q .GT. 0.0D+00) LAM = MIN(1.0D+00,MAX(0.0D+00,P/Q))

      
*
*     Computation of the aggregate values
*      

      P = 1.0D+00 - LAM
      DO 30 I=1,N
         GA(I)=LAM*G(I) + P*GP(I)
 30   CONTINUE
      
      ALFV = LAM*ALFN
      
      RETURN
      END

      

************************************************************************
*
*     * SUBROUTINE AGGSR1 *
*
*      
*     * Purpose *
*      
*     Computation of aggregate values by the limited memory SR1 update.
*
*      
*     * Calling sequence *
*     
*     CALL AGGSR1(N,MC,MCC,INEW,IFLAG,G,GP,GA,D,ALFN,ALFV,TMPMAT,UMTUM,
*    &     R,GAMMA,SMTGP,UMTGP,SMTGA,UMTGA,SM,UM,TMPMC3,TMPMC4,TMPN2,
*    &     TMPN3,TMPN4,ICN,RHO,SMALL)
*
*      
*     * Parameters *
*
*     II  N               Number of variables.
*     II  MC              Declared number of stored corrections.
*     II  MCC             Current number of stored corrections.
*     II  INEW            Index for circular arrays.
*     IU  IFLAG           Index for adaptive version:
*                             0  - Maximum number of stored corrections
*                                    has not been changed at this
*                                    iteration.
*                             1  - Maximum number of stored corrections
*                                    has been changed at this iteration.
*     RI  G(N)            Current (auxiliary) subgradient of the
*                           objective function.
*     RI  GP(N)           Basic subgradient of the objective function.
*     RU  GA(N)           Current aggregate subgradient.
*     RI  D(N)            Search direction.
*     RI  ALFN            Locality measure.
*     RO  ALFV            Aggregate locality measure.
*     RI  SM(N*(MC+1))    Matrix whose columns are stored corrections.
*     RI  UM(N*(MC+1))    Matrix whose columns are stored subgradient.
*     RI  UMTUM((MC+2)*(MC+1)/2)  Matrix UMTUM = TRANS(UM) * UM.
*     RI  R((MC+2)*(MC+1)/2)   Upper triangular matrix.
*     RI  SMTGP(MC+1)     Vector SMTGP = TRANS(SM)*GP.
*     RI  SMTGA(MC+1)     Vector SMTGA = TRANS(SM)*GA.
*     RI  UMTGP(MC+1)     Vector UMTGP = TRANS(UM)*GP.
*     RI  UMTGA(MC+1)     Vector UMTGA = TRANS(UM)*GA.
*     RI  GAMMA           Scaling parameter.
*     RA  TMPMC#(MC+1)    Auxiliary arrays; # = 3,4.
*     RA  TMPN#(N)        Auxiliary arrays; # = 2,...,4.
*     RA  TMPMAT((MC+1)*(MC)/2)  Auxiliary matrix.
*     II  ICN             Correction indicator.
*     RI  RHO             Correction parameter.
*     RI  SMALL           Small positive value.
*
*     
*     * Local variables *
*
*     I   MCNEW           Current size of vectors.
*     I   IOLD            Index of the oldest corrections.
*     R   LAM#            Multipliers: # = 1,2.
*     R   PR              PR = TRANS(GP-GA) DM (GP-GA), where DM
*                           presents the L-SR1- approximation of Hessian.
*     R   RRP             RRP = TRANS(GP-GA) DM GA - ALFV.
*     R   PRQR            PRQR = TRANS(GP-GA) DM (G-GA).
*     R   RRQ             RRQ = TRANS(G-GA) DM GA - ALFV + ALFN.
*     R   QR              QR = TRANS(G-GA) DM (G-GA).
*     R   PQ              PQ = TRANS(G-GP) DM (G-GP).
*     R   QQP             QQP = TRANS(G-GP) DM G + ALFN.
*     R   TMP1            Auxiliary scalar.
*     R   TMP2            Auxiliary scalar.
*     R   W               Correction.
*
*     
*     * Subprograms used *
*      
*     RF  VDOT            Dot product of two vectors.
*     S   XSUMY           Sum of two vectors.
*     S   RWAXV2          Multiplication of two rowwise stored dense 
*                           rectangular matrices A and B by vectors X 
*                           and Y.
*     S   CWMAXV          Multiplication of a vector by a dense
*                           rectangular matrix.
*     S   SCALEX          Scaling a vector.
*     S   XDIFFY          Difference of two vectors.
*     S   SCSUM           Sum of a vector and the scaled vector.
*     S   SCDIFF          Difference of the scaled vector and a vector.
*     S   CALQ            Solving X from linear equation A*X=Y.
*     S   LINEQ           Solver for linear equation.
*     
*
*     The variable and subgradient differences and all the MC-vectors are
*     stored in a circular order controlled by the parameter point inew.
*
*     
*      
*     Napsu Karmitsa (2002 - 2004, last modified 2007)      
*
      
      SUBROUTINE AGGSR1(N,MC,MCC,INEW,IFLAG,G,GP,GA,D,ALFN,ALFV,TMPMAT,
     &     UMTUM,R,GAMMA,SMTGP,UMTGP,SMTGA,UMTGA,SM,UM,TMPMC3,TMPMC4,
     &     TMPN2,TMPN3,TMPN4,ICN,RHO,SMALL)
      
*     Scalar Arguments
      INTEGER N,MC,MCC,INEW,IFLAG,ICN
      DOUBLE PRECISION ALFN,ALFV,GAMMA,RHO,SMALL
      
*     Array Arguments
      DOUBLE PRECISION G(*),GP(*),GA(*),D(*),TMPMAT(*),UMTUM(*),R(*),
     &     SMTGP(*),UMTGP(*),SMTGA(*),UMTGA(*),SM(*),UM(*),
     &     TMPMC3(*),TMPMC4(*),TMPN2(*),TMPN3(*),TMPN4(*)
      
*     Local Scalars
      INTEGER I,IOLD,MCNEW,IERR
      DOUBLE PRECISION LAM1,LAM2,PR,RRP,PRQR,RRQ,QR,PQ,QQP,W,TMP1,TMP2
      
*     External Functions
      DOUBLE PRECISION VDOT
      EXTERNAL VDOT

*     External Subroutines
      EXTERNAL XDIFFY,SCALEX,XSUMY,CWMAXV,RWAXV2,CALQ,SCDIFF,SCSUM,LINEQ

*     Intrinsic Functions
      INTRINSIC MIN,MAX

      IERR = 0
      
      IF (MCC .LT. MC) THEN
         IOLD = 1
         MCNEW = MCC

      ELSE IF (IFLAG .EQ. 0) THEN
         MCNEW = MC
         IOLD = INEW + 1
         IF (IOLD .GT. MC+1) IOLD = 1

      ELSE
         MCNEW = MC - 1
         IOLD = INEW + 1
         IF (IOLD .GT. MC) IOLD = 1
      END IF 

      
      CALL XDIFFY(N,GP,GA,TMPN2)

      
*      
*     Calculation of TMPN3 = TRANS(GP - GA)DM
*

      IF (MCC .GT. 0) THEN

         DO 10 I=1,MCNEW*(MCNEW+1)/2
            TMPMAT(I)= GAMMA * UMTUM(I) - R(I)
 10      CONTINUE

         IF (IOLD .EQ. 1) THEN
            CALL XDIFFY(MCNEW,UMTGP,UMTGA,TMPMC4)
            CALL SCDIFF(MCNEW,GAMMA,TMPMC4,SMTGP,TMPMC4)
            CALL XSUMY(MCNEW,TMPMC4,SMTGA,TMPMC4)

            CALL CALQ(MCNEW,MCNEW,IOLD,TMPMAT,TMPMC3,TMPMC4,SMALL,0)
            CALL SCALEX(MCNEW,GAMMA,TMPMC3,TMPMC4)

            CALL CWMAXV(N,MCNEW,SM,TMPMC3,TMPN4,1.0D+00)
            CALL SCSUM(N,GAMMA,TMPN2,TMPN4,TMPN3)
            CALL CWMAXV(N,MCNEW,UM,TMPMC4,TMPN4,1.0D+00)
            CALL XDIFFY(N,TMPN3,TMPN4,TMPN3)

         ELSE
            CALL XDIFFY(MCC,UMTGP,UMTGA,TMPMC4)
            CALL SCDIFF(MCC,GAMMA,TMPMC4,SMTGP,TMPMC4)
            CALL XSUMY(MCC,TMPMC4,SMTGA,TMPMC4)

            CALL CALQ(MCNEW,MCC,IOLD,TMPMAT,TMPMC3,TMPMC4,SMALL,0)
            CALL SCALEX(MCC,GAMMA,TMPMC3,TMPMC4)

            CALL CWMAXV(N,INEW-1,SM,TMPMC3,TMPN4,1.0D+00)
            CALL SCSUM(N,GAMMA,TMPN2,TMPN4,TMPN3)
            CALL CWMAXV(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),TMPMC3(IOLD)
     &           ,TMPN4,1.0D+00)
            CALL XSUMY(N,TMPN3,TMPN4,TMPN3)
            CALL CWMAXV(N,INEW-1,UM,TMPMC4,TMPN4,1.0D+00)
            CALL XDIFFY(N,TMPN3,TMPN4,TMPN3)
            CALL CWMAXV(N,MCNEW-INEW+1,UM((IOLD-1)*N+1),TMPMC4(IOLD)
     &           ,TMPN4,1.0D+00)
            CALL XDIFFY(N,TMPN3,TMPN4,TMPN3)
         END IF
         
         IF (ICN .EQ. 1) THEN
            CALL SCSUM(N,RHO,TMPN2,TMPN3,TMPN3)
         END IF
         
         PR = VDOT(N,TMPN3,TMPN2)
         RRP = VDOT(N,TMPN3,GA) 
         CALL XDIFFY(N,G,GA,TMPN4)
         PRQR = VDOT(N,TMPN3,TMPN4)
         RRQ = -VDOT(N,TMPN4,D)

      ELSE

         PR = VDOT(N,TMPN2,TMPN2)
         RRP = VDOT(N,TMPN2,GA) 
         CALL XDIFFY(N,G,GA,TMPN4)
         PRQR = VDOT(N,TMPN2,TMPN4)
         RRQ = -VDOT(N,TMPN4,D)
      END IF

      
*
*     Calculation of QR = TRANS(G - GA) DM (G - GA)
*

      QR = VDOT(N,TMPN4,TMPN4)
      IF (ICN .EQ. 1) THEN
         W = RHO*QR
      ELSE
         W = 0.0D+00
      END IF
      
      IF (MCC .GT. 0) THEN
         QR = GAMMA*QR

         IF (IOLD .EQ. 1) THEN
            CALL RWAXV2(N,MCNEW,SM,UM,TMPN4,TMPN4,TMPMC4,TMPMC3)
            CALL SCSUM(MCNEW,-GAMMA,TMPMC3,TMPMC4,TMPMC4)
            CALL LINEQ(MCNEW,MCNEW,IOLD,TMPMAT,TMPMC3,TMPMC4,IERR)
            
            QR = QR - VDOT(MCNEW,TMPMC4,TMPMC3) + W

         ELSE
            CALL RWAXV2(N,INEW-1,SM,UM,TMPN4,TMPN4,TMPMC4,TMPMC3)
            CALL RWAXV2(N,MCNEW-INEW+1,SM((IOLD-1)*N+1),
     &           UM((IOLD-1)*N+1),TMPN4,TMPN4,TMPMC4(IOLD),TMPMC3(IOLD))
            CALL SCSUM(MCC,-GAMMA,TMPMC3,TMPMC4,TMPMC4)
            CALL LINEQ(MCNEW,MCC,IOLD,TMPMAT,TMPMC3,TMPMC4,IERR)
            
            QR = QR - VDOT(MCC-INEW,TMPMC4(IOLD),TMPMC3(IOLD)) -
     &           VDOT(INEW-1,TMPMC4,TMPMC3) + W
         END IF

      END IF
      
      PQ = QR - 2*PRQR + PR
      QQP = PQ + PRQR + RRQ - PR - RRP + ALFN
      RRP = RRP - ALFV
      RRQ = RRQ + ALFN - ALFV

      
*     
*     Computation of multipliers LAM1 and LAM2
*

      IF (PR .LE. 0.0D+00 .OR. QR .LE. 0.0D+00) GOTO 100
      TMP1 = RRQ/QR
      TMP2 = PRQR/QR
      W = PR - PRQR*TMP2
      IF (W .EQ. 0.0D+00) GOTO 100
      LAM1 = (TMP1*PRQR - RRP)/W
      LAM2 = -TMP1 - LAM1*TMP2
      IF (LAM1*(LAM1 - 1.0D+00) .LT. 0.0D+00 .AND.
     &     LAM2*(LAM1 + LAM2 - 1.0D+00) .LT. 0.0D+00) GOTO 200

      
*
*     Minimum on the boundary
*

 100  CONTINUE
      LAM1 = 0.0D+00
      LAM2 = 0.0D+00
      IF (ALFN .LE. ALFV) LAM2 = 1.0D+00
      IF (QR .GT. 0.0D+00) LAM2 = MIN(1.0D+00,MAX(0.0D+00,-RRQ/QR))
      W = (LAM2*QR + 2.0D+00*RRQ)*LAM2
      TMP1 = 0.0D+00
      IF (ALFV .GE. 0.0D+00) TMP1 = 1.0D+00
      IF (PR .GT. 0.0D+00) TMP1 = MIN(1.0D+00,MAX(0.0D+00,-RRP/PR))
      TMP2 = (TMP1*PR + 2.0D+00*RRP)*TMP1
      IF (TMP2 .LT. W) THEN
         W = TMP2
         LAM1 = TMP1
         LAM2 = 0.0D+00
      END IF
      
      IF (QQP*(QQP - PQ) .GE. 0.0D+00) GOTO 200

      IF (QR + 2.0D+00*RRQ - QQP*QQP/PQ .GE. W) GOTO 200
      LAM1 = QQP/PQ
      LAM2 = 1.0D+00 - LAM1
      
 200  CONTINUE
      IF (LAM1 .EQ. 0.0D+00 .AND. LAM2*(LAM2 - 1.0D+00) .LT. 0.0D+00
     &     .AND. -RRP - LAM2*PRQR .GT. 0.0D+00 .AND. PR .GT. 0.0D+00)
     &     LAM1 = MIN(1.0D+00 - LAM2, (-RRP-LAM2*PRQR)/PR)

      
*
*     Computation of the aggregate values
*      

      TMP1 = 1.0D+00 - LAM1 - LAM2
      DO 30 I=1,N
         GA(I)=LAM1*GP(I)+LAM2*G(I)+TMP1*GA(I)
 30   CONTINUE
      
      ALFV = LAM2*ALFN + TMP1*ALFV
      
      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE AGSKIP *
*
*      
*     * Purpose *
*      
*     Computation of aggregate values after consecutive null steps
*     by the limited memory BFGS update.
*
*      
*     * Calling sequence *
*     
*     CALL AGSKIP(N,MC,MCC,INEW,IFLAG,G,GP,GA,D,U,ALFN,ALFV,UMTUM,R,C,
*    &     GAMMA,SMTGP,UMTGP,SMTGA,UMTGA,SM,UM,TMPMC3,TMPMC4,TMPN2,ICN,
*    &     RHO)
*     
*     
*     * Parameters *
*
*     II  N               Number of variables.
*     II  MC              Declared number of stored corrections.
*     II  MCC             Current number of stored corrections.
*     II  INEW            Index for circular arrays.
*     IU  IFLAG           Index for adaptive version:
*                             0  - Maximum number of stored corrections
*                                    has not been changed at this
*                                    iteration.
*                             1  - Maximum number of stored corrections
*                                    has been changed at this iteration.
*     RI  G(N)            Current (auxiliary) subgradient of the
*                           objective function.
*     RI  GP(N)           Basic subgradient of the objective function.
*     RU  GA(N)           Current aggregate subgradient.
*     RI  D(N)            Search direction.
*     RI  U(N)            Difference of trial and aggregate gradients.
*     RI  ALFN            Locality measure.
*     RO  ALFV            Aggregate locality measure.
*     RI  SM(N*(MC+1))    Matrix whose columns are stored corrections.
*     RI  UM(N*(MC+1))    Matrix whose columns are stored subgradient.
*     RI  UMTUM((MC+2)*(MC+1)/2)  Matrix UMTUM = TRANS(UM) * UM.
*     RI  R((MC+2)*(MC+1)/2)   Upper triangular matrix.
*     RI  C(MC+1)         Diagonal matrix.
*     RI  SMTGP(MC+1)     Vector SMTGP = TRANS(SM)*GP.
*     RI  SMTGA(MC+1)     Vector SMTGA = TRANS(SM)*GA.
*     RI  UMTGP(MC+1)     Vector UMTGP = TRANS(UM)*GP.
*     RI  UMTGA(MC+1)     Vector UMTGA = TRANS(UM)*GA.
*     RI  GAMMA           Scaling parameter.
*     RA  TMPMC#(MC+1)    Auxiliary arrays; # = 3,4.
*     RA  TMPN2(N)        Auxiliary array.
*     II  ICN             Correction indicator.
*     RI  RHO             Correction parameter.
*     RI  SMALL           Small positive value.
*
*     
*     * Local variables *
*
*     I   MCNEW           Current size of vectors.
*     I   IOLD            Index of the oldest corrections.
*     R   LAM#            Multipliers: # = 1,2.
*     R   PR              PR = TRANS(GP-GA) DM (GP-GA), where DM
*                           presents the L-BFGS- approximation of Hessian.
*     R   RRP             RRP = TRANS(GP-GA) DM GA - ALFV.
*     R   PRQR            PRQR = TRANS(GP-GA) DM (G-GA).
*     R   RRQ             RRQ = TRANS(G-GA) DM GA - ALFV + ALFN.
*     R   QR              QR = TRANS(G-GA) DM (G-GA).
*     R   PQ              PQ = TRANS(G-GP) DM (G-GP).
*     R   QQP             QQP = TRANS(G-GP) DM G + ALFN.
*     R   TMP1            Auxiliary scalar.
*     R   TMP2            Auxiliary scalar.
*     R   W               Correction.
*
*     
*     * Subprograms used *
*      
*     RF  VDOT            Dot product of two vectors.
*     S   RWAXV2          Multiplication of two rowwise stored dense 
*                           rectangular matrices A and B by vectors X 
*                           and Y.
*     S   XDIFFY          Difference of two vectors.
*     S   SYMAX           Multiplication of a dense symmetric matrix
*                           by a vector.
*     S   TRLIEQ          Solving X from linear equation L*X=Y or
*                           TRANS(L)*X=Y.
*     
*
*     The variable and subgradient differences and all the MC-vectors are
*     stored in a circular order controlled by the parameter point inew.
*
*     
*
*     Napsu Karmitsa (2002,2003, last modified 2007)
*
      
      SUBROUTINE AGSKIP(N,MC,MCC,INEW,IFLAG,G,GP,GA,D,U,ALFN,ALFV,
     &     UMTUM,R,C,GAMMA,SMTGP,UMTGP,SMTGA,UMTGA,SM,UM,TMPMC3,TMPMC4,
     &     TMPN2,ICN,RHO)
      
*     Scalar Arguments
      INTEGER N,MC,MCC,INEW,IFLAG,ICN
      DOUBLE PRECISION ALFN,ALFV,GAMMA,RHO
      
*     Array Arguments
      DOUBLE PRECISION G(*),GP(*),GA(*),D(*),U(*),UMTUM(*),R(*),C(*),
     &     SMTGP(*),UMTGP(*),SMTGA(*),UMTGA(*),SM(*),UM(*),
     &     TMPMC3(*),TMPMC4(*),TMPN2(*)
      
*     Local Scalars
      INTEGER I,IOLD,MCNEW,IERR
      DOUBLE PRECISION LAM1,LAM2,PR,RRP,PRQR,RRQ,QR,PQ,QQP,W,TMP1,TMP2

*     External Functions
      DOUBLE PRECISION VDOT
      EXTERNAL VDOT

*     External Subroutines
      EXTERNAL XDIFFY,RWAXV2,SYMAX,TRLIEQ

*     Intrinsic Functions
      INTRINSIC MIN,MAX

           
      IF (MCC .LT. MC) THEN
         IOLD = 1
         MCNEW = MCC

      ELSE
         IF (IFLAG .EQ. 0) THEN
            MCNEW = MC
            IOLD = INEW + 1
            IF (IOLD .GT. MC+1) IOLD = 1

         ELSE
            MCNEW = MC - 1
            IOLD = INEW + 1
            IF (IOLD .GT. MC) IOLD = 1
         END IF
      END IF 

      
*
*     Calculation of PQ = TRANS(G-GP) DM (G-GP) = TRANS(U) DM U.
*

      PQ = VDOT(N,U,U)

      IF (ICN .EQ. 1) THEN
         W = RHO * PQ
      ELSE
         W = 0.0D+00
      END IF

      IF (MCC .GT. 0) THEN

         IF (IOLD .EQ. 1) THEN
            CALL RWAXV2(N,MCNEW,SM,UM,U,U,TMPMC3,TMPMC4)
            CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC3,TMPMC3,1,IERR)

            PQ = PQ - 2*VDOT(MCNEW,TMPMC4,TMPMC3)
            PQ = GAMMA*PQ
            
            DO 10 I=1,MCNEW
               TMPMC4(I) = C(I)*TMPMC3(I)
 10         CONTINUE

            PQ = PQ + VDOT(MCNEW,TMPMC3,TMPMC4)

            CALL SYMAX(MCNEW,MCNEW,IOLD,UMTUM,TMPMC3,TMPMC4)

            PQ = PQ + GAMMA*VDOT(MCNEW,TMPMC3,TMPMC4)

         ELSE
            CALL RWAXV2(N,INEW-1,SM,UM,U,U,TMPMC3,TMPMC4)
            CALL RWAXV2(N,MCC-INEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),
     &           U,U,TMPMC3(IOLD),TMPMC4(IOLD))
            CALL TRLIEQ(MCNEW,MCC,IOLD,R,TMPMC3,TMPMC3,1,IERR)

            PQ = PQ - 2*(VDOT(MCC-INEW,TMPMC4(IOLD),TMPMC3(IOLD)) + 
     &           VDOT(INEW-1,TMPMC4,TMPMC3))
            PQ = GAMMA*PQ

            DO 20 I=1,MCC
               TMPMC4(I) = C(I)*TMPMC3(I)
 20         CONTINUE

            PQ = PQ + VDOT(MCC-INEW,TMPMC3(IOLD),TMPMC4(IOLD)) + 
     &           VDOT(INEW-1,TMPMC3,TMPMC4)

            CALL SYMAX(MCNEW,MCC,IOLD,UMTUM,TMPMC3,TMPMC4)

            PQ = PQ + GAMMA*(VDOT(MCC-INEW,TMPMC3(IOLD),TMPMC4(IOLD))
     &           + VDOT(INEW-1,TMPMC3,TMPMC4))
         END IF

      END IF

      PQ = PQ + W

      
*
*     Calculation of PR = TRANS(GP-GA) DM (GP-GA).
*
      
      CALL XDIFFY(N,GP,GA,TMPN2)
      PR = VDOT(N,TMPN2,TMPN2)

      IF (ICN .EQ. 1) THEN
         W = RHO * PR
      ELSE
         W = 0.0D+00
      END IF

      IF (MCC .GT. 0) THEN

         IF (IOLD .EQ. 1) THEN 
            DO 301 I=1, MCNEW
               TMPMC3(I)=SMTGP(I)-SMTGA(I)
               TMPMC4(I)=UMTGP(I)-UMTGA(I)
 301        CONTINUE
            CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC3,TMPMC3,1,IERR)

            PR = PR - 2*VDOT(MCNEW,TMPMC4,TMPMC3)
            PR = GAMMA*PR
            
            DO 30 I=1,MCNEW
               TMPMC4(I) = C(I)*TMPMC3(I)
 30         CONTINUE

            PR = PR + VDOT(MCNEW,TMPMC3,TMPMC4)

            CALL SYMAX(MCNEW,MCNEW,IOLD,UMTUM,TMPMC3,TMPMC4)

            PR = PR + GAMMA*VDOT(MCNEW,TMPMC3,TMPMC4)

         ELSE
            DO 401 I=1, MCC
               TMPMC3(I)=SMTGP(I)-SMTGA(I)
               TMPMC4(I)=UMTGP(I)-UMTGA(I)
 401        CONTINUE
            CALL TRLIEQ(MCNEW,MCC,IOLD,R,TMPMC3,TMPMC3,1,IERR)

            PR = PR - 2*(VDOT(MCC-INEW,TMPMC4(IOLD),TMPMC3(IOLD)) + 
     &           VDOT(INEW-1,TMPMC4,TMPMC3))
            PR = GAMMA*PR

            DO 40 I=1,MCC
               TMPMC4(I) = C(I)*TMPMC3(I)
 40         CONTINUE

            PR = PR + VDOT(MCC-INEW,TMPMC3(IOLD),TMPMC4(IOLD)) + 
     &           VDOT(INEW-1,TMPMC3,TMPMC4)

            CALL SYMAX(MCNEW,MCC,IOLD,UMTUM,TMPMC3,TMPMC4)

            PR = PR + GAMMA*(VDOT(MCC-INEW,TMPMC3(IOLD),TMPMC4(IOLD))
     &           + VDOT(INEW-1,TMPMC3,TMPMC4))
         END IF

      END IF

      PR = PR + W

      
*     
*     Calculation of RRP = TRANS(GP-GA) DM GA - ALFV.
*
      
      RRP = - VDOT(N,TMPN2,D) - ALFV

      
*
*     Calculation of QR = TRANS(G-GA) DM (G-GA).
*      

      CALL XDIFFY(N,G,GA,TMPN2)
      QR = VDOT(N,TMPN2,TMPN2)

      IF (ICN .EQ. 1) THEN
         W = RHO * QR
      ELSE
         W = 0.0D+00
      END IF

      IF (MCC .GT. 0) THEN

         IF (IOLD .EQ. 1) THEN
            CALL RWAXV2(N,MCNEW,SM,UM,TMPN2,TMPN2,TMPMC3,TMPMC4)
            CALL TRLIEQ(MCNEW,MCNEW,IOLD,R,TMPMC3,TMPMC3,1,IERR)

            QR = QR - 2*VDOT(MCNEW,TMPMC4,TMPMC3)
            QR = GAMMA*QR
            
            DO 50 I=1,MCNEW
               TMPMC4(I) = C(I)*TMPMC3(I)
 50         CONTINUE

            QR = QR + VDOT(MCNEW,TMPMC3,TMPMC4)

            CALL SYMAX(MCNEW,MCNEW,IOLD,UMTUM,TMPMC3,TMPMC4)

            QR = QR + GAMMA*VDOT(MCNEW,TMPMC3,TMPMC4)

         ELSE
            CALL RWAXV2(N,INEW-1,SM,UM,TMPN2,TMPN2,TMPMC3,TMPMC4)
            CALL RWAXV2(N,MCC-INEW,SM((IOLD-1)*N+1),UM((IOLD-1)*N+1),
     &           TMPN2,TMPN2,TMPMC3(IOLD),TMPMC4(IOLD))
            CALL TRLIEQ(MCNEW,MCC,IOLD,R,TMPMC3,TMPMC3,1,IERR)

            QR = QR - 2*(VDOT(MCC-INEW,TMPMC4(IOLD),TMPMC3(IOLD)) + 
     &           VDOT(INEW-1,TMPMC4,TMPMC3))
            QR = GAMMA*QR

            DO 60 I=1,MCC
               TMPMC4(I) = C(I)*TMPMC3(I)
 60         CONTINUE

            QR = QR + VDOT(MCC-INEW,TMPMC3(IOLD),TMPMC4(IOLD)) + 
     &           VDOT(INEW-1,TMPMC3,TMPMC4)

            CALL SYMAX(MCNEW,MCC,IOLD,UMTUM,TMPMC3,TMPMC4)

            QR = QR + GAMMA*(VDOT(MCC-INEW,TMPMC3(IOLD),TMPMC4(IOLD))
     &           +VDOT(INEW-1,TMPMC3,TMPMC4))
         END IF

      END IF

      QR = QR + W

      
*
*     Calculation of RRQ = TRANS(G-GA) DM GA - ALFV + ALFN.
*      

      RRQ = - VDOT(N,TMPN2,D) - ALFV + ALFN

*     
*     Calculation of PRQR = TRANS(GP-GA) DM (G-GA).
*      

      PRQR = (QR - PQ + PR)/2.0D+00

*     
*     Calculation of QQP = TRANS(G-GP) DM G + ALFN.
*

      QQP = PQ + PRQR + RRQ - PR - RRP

      
*     
*     Computation of multipliers LAM1 and LAM2
*

      IF (PR .LE. 0.0D+00 .OR. QR .LE. 0.0D+00) GOTO 100

      TMP1 = RRQ/QR
      TMP2 = PRQR/QR
      W = PR - PRQR*TMP2

      IF (W .EQ. 0.0D+00) GOTO 100

      LAM1 = (TMP1*PRQR - RRP)/W
      LAM2 = -TMP1 - LAM1*TMP2

      IF (LAM1*(LAM1 - 1.0D+00) .LT. 0.0D+00 .AND.
     &     LAM2*(LAM1 + LAM2 - 1.0D+00) .LT. 0.0D+00) GOTO 200

      
*
*     Minimum on the boundary
*

 100  CONTINUE
      LAM1 = 0.0D+00
      LAM2 = 0.0D+00
      IF (ALFN .LE. ALFV) LAM2 = 1.0D+00
      IF (QR .GT. 0.0D+00) LAM2 = MIN(1.0D+00,MAX(0.0D+00,-RRQ/QR))
      W = (LAM2*QR + 2.0D+00*RRQ)*LAM2
      TMP1 = 0.0D+00
      IF (ALFV .GE. 0.0D+00) TMP1 = 1.0D+00
      IF (PR .GT. 0.0D+00) TMP1 = MIN(1.0D+00,MAX(0.0D+00,-RRP/PR))
      TMP2 = (TMP1*PR + 2.0D+00*RRP)*TMP1
      IF (TMP2 .LT. W) THEN
         W = TMP2
         LAM1 = TMP1
         LAM2 = 0.0D+00
      END IF
      
      IF (QQP*(QQP - PQ) .GE. 0.0D+00) GOTO 200

      IF (QR + 2.0D+00*RRQ - QQP*QQP/PQ .GE. W) GOTO 200
      LAM1 = QQP/PQ
      LAM2 = 1.0D+00 - LAM1
      
 200  CONTINUE
      IF (LAM1 .EQ. 0.0D+00 .AND. LAM2*(LAM2 - 1.0D+00) .LT. 0.0D+00
     &     .AND. -RRP - LAM2*PRQR .GT. 0.0D+00 .AND. PR .GT. 0.0D+00)
     &     LAM1 = MIN(1.0D+00 - LAM2, (-RRP-LAM2*PRQR)/PR)

      
*
*     Computation of the aggregate values
*      

      TMP1 = 1.0D+00 - LAM1 - LAM2
      DO 210 I=1,N
         GA(I)=LAM1*GP(I)+LAM2*G(I)+TMP1*GA(I)
 210   CONTINUE
      
      ALFV = LAM2*ALFN + TMP1*ALFV
      
      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE LLS *
*
*      
*     * Purpose *
*      
*     Special line search for limited memory bundle method
*
*      
*     * Calling sequence *
*     
*     CALL LLS(N,X,G,D,XO,T,FO,F,P,ALFN,TMIN,DNORM,WK,THETA,EPSL,
*    &     EPSR,ETA,MOS,ITERS,NFE,NNK,ITERM)
*     
*     
*     * PARAMETERS *
*      
*     II  N             Number of variables.
*     RU  X(N)          Vector of variables.
*     RI  XO(N)         Previous vector of variables.
*     RO  G(N)          Subgradient of the objective function.
*     RI  D(N)          Direction vector.
*     RU  T             Stepsize.
*     RO  F             Value of the objective function.
*     RI  FO            Previous value of the objective function.
*     RU  P             Directional derivative.
*     RI  TMIN          Minimum value of the stepsize.
*     RI  DNORM         Euclidean norm of the direction vector.
*     RI  WK            Stopping parameter.
*     RI  EPSL          Termination tolerance for line search (in test
*                         on the change of the function value).
*     RI  EPSR          Termination tolerance for line search (in test
*                         on the directional derivative).
*     RI  ETA           Distance measure parameter.
*     RO  ALFN          Locality measure.
*     RI  THETA         Scaling parameter.
*     II  MOS           Locality measure parameter.
*     IO  ITERS         Null step indicator.
*                            0  - Null step.
*                            1  - Serious step.
*     II  NNK           Number of consequtive null steps.
*     IU  NFE           Number of function evaluations.
*     IO  ITERM         Cause of termination:
*                          0  - Everything is ok.
*                         -3  - Failure in function or subgradient
*                               calculations.
*
*      
*     * Local parameters *
*
*     I   MAXINT        Maximum number of interpolations.
*
*      
*     * Local variables *
*
*     I   NIN           Number of interpolations.
*     R   TL,TU         Lower and upper limits for T used in
*                         interpolation.
*     R   FL            Value of the objective function for T=TL.
*     R   FU            Value of the objective function for T=TU.
*     R   EPSA          Line search parameter.
*     R   EPST          Line search parameter.
*     R   EPSLK         Line search parameter.
*     R   EPSRK         Line search parameter.
*      
*
*     * Subprograms used *
*      
*     RF  VDOT          Dot product of two vectors.
*     S   QINT          Quadratic interpolation for line search
*                         with one directional derivative.
*     S   SCSUM         Sum of a vector and the scaled vector.
*
*      
*     * EXTERNAL SUBROUTINES *
*      
*     SE  FUNDER        Computation of the value and the subgradient of
*                       the objective function. Calling sequence:
*                       CALL FUNDER(N,X,F,G,ITERM), where N is a number of
*                       variables, X(N) is a vector of variables, F is
*                       the value of the objective function and G(N) is
*                       the subgradient of the objective function and 
*                       ITERM is the error indicator.
*      
*      
*
*     * Original method *
*      
*     Special line search for nonsmooth nonconvex variable metric
*     method (PS1L08) by J.Vlcek (1999) 
*
*      
*     * Limited memory version *
*      
*     Napsu Karmitsa (2002 - 2003, last modified 2007).      
*


      SUBROUTINE LLS(N,X,G,D,XO,T,FO,F,P,ALFN,TMIN,
     &     DNORM,WK,THETA,EPSL,EPSR,ETA,MOS,ITERS,NFE,NNK,ITERM)

*     Scalar Arguments
      INTEGER N,MOS,ITERS,NFE,NNK,ITERM
      DOUBLE PRECISION T,FO,F,P,ALFN,TMIN,DNORM,WK,THETA,EPSL,
     &     EPSR,ETA

*     Array Arguments
      DOUBLE PRECISION X(*),G(*),D(*),XO(*)

*     Local Scalars
      INTEGER NIN
      DOUBLE PRECISION FL,FU,TL,TU,EPSA,EPST,EPSLK,EPSRK

*     Parameters
      INTEGER MAXINT
      PARAMETER(MAXINT = 200)

*     External Functions
      DOUBLE PRECISION VDOT
      EXTERNAL VDOT

*     Intrinsic Functions
      INTRINSIC ABS,MAX

*     External Subroutines
      EXTERNAL FUNDER,SCSUM,QINT

      
*
*     Initialization
*      

      NIN=0

      EPST = 2.0D+00 * EPSL
      EPSA = (EPSR - EPST)/2.0D+00
      EPSLK = 1.00D-04
      EPSRK = 0.25D+00
      
      TL = 0.0D+00
      TU = T
      FL = FO

      IF (THETA .LT. 1.0D+00) THEN
         EPST=THETA*EPST
         EPSA=THETA*EPSA
         EPSLK=EPSL
         EPSL=THETA*EPSL
         EPSRK=EPSR
         EPSR=THETA*EPSR
      END IF

      
*     
*     Function and gradient evaluation at a new point
*

 10   CONTINUE
      CALL SCSUM(N,THETA*T,D,XO,X)
      CALL FUNDER(N,X,F,G,ITERM)
      NFE = NFE + 1

      IF (ITERM .NE. 0) RETURN


      P = THETA*VDOT(N,G,D)
      ALFN = MAX(ABS(FO-F+P*T),ETA*(T*THETA*DNORM)**MOS)
      

*     
*     Null/descent step test (ITERS=0/1)
*

      ITERS = 1
      IF (F .LE. FO - T*EPST*WK) THEN
         TL = T
         FL = F

      ELSE
         TU = T
         FU = F
      END IF

      
*
*     Serious step
*      

      IF (F .LE. FO - T*EPSL*WK .AND. (T .GE. TMIN .OR.
     &     ALFN .GT. EPSA*WK)) GO TO 40

      
*
*     Additional interpolation
*      

      IF (F .GT. FO .AND. TU-TL .GE. TMIN*1.0D-01
     &     .AND. NNK .GE. 1 .AND. NIN .LT. MAXINT) THEN
         GO TO 20
      ENDIF

      
*
*     Null step
*      

      IF (P-ALFN .GE. -EPSR*WK .OR. TU-TL .LT. TMIN*1.0D-01 .OR.
     &     NIN .GE. MAXINT) GO TO 30


*
*     Interpolation
*      

 20   CONTINUE
      
      NIN=NIN+1
      IF (TL.EQ.0.0D+00 .AND. WK.GT.0.0D+00) THEN
         CALL QINT(TU,FL,FU,WK,T,1.0D+00-0.5D+00/(1.0D+00-EPST))

      ELSE
         T = 5.0D-01* (TU+TL)
      END IF

      GO TO 10

 30   CONTINUE
      ITERS = 0
      
 40   CONTINUE
      IF (THETA .NE. 1.0D+00) THEN
         EPSL=EPSLK
         EPSR=EPSRK
      END IF

      RETURN
      END


************************************************************************
*
*     * SUBROUTINE QINT *
*
*      
*     * Purpose *
*      
*     Quadratic interpolation for line search with one directional
*     derivative.
*
*
*     * Calling sequence *
*     
*     CALL QINT(TU,FL,FU,WK,T,KAPPA)
*     
*     
*     * Parameters *
*
*     RI  TU            Upper value of the stepsize parameter.
*     RI  FL            Value of the objective function.
*     RI  FU            Value of the objective function for T=TU.
*     RI  WK            Directional derivative.
*     RO  T             Stepsize parameter.
*     RI  KAPPA         Interpolation parameter.
*
*      
*
*     Napsu Haarala (2004).      
*
      

      SUBROUTINE QINT(TU,FL,FU,WK,T,KAPPA)
      
*     Scalar Arguments
      DOUBLE PRECISION FL,FU,WK,T,TU,KAPPA

*     Local Scalars
      DOUBLE PRECISION TMP1,TMP2

*     Intrinsic Functions
      INTRINSIC MAX


      TMP1 = (FU-FL)/ (-WK*TU)

      
*     
*     Quadratic interpolation with one directional derivative
*

      TMP2 = 2.0D+00 * (1.0D+00 - TMP1)

      IF (TMP2 .GT. 1.0D+00) THEN

         
*     
*     Interpolation accepted
*

         T = MAX(KAPPA*TU,TU/TMP2)

         RETURN

      END IF

      
*     
*     Bisection
*
      T = 0.50D+00 * TU
      
      RETURN
      END


************************************************************************
*
*     * SUBROUTINE TINIT *
*
*      
*     * Purpose *
*      
*     Initial stepsize selection for limited memory bundle method
*
*
*     * Calling sequence *
*     
*     CALL TINIT(N,NA,MAL,X,AF,AG,AY,IBUN,D,F,P,T,TMAX,TMIN,ETA,ETA9,
*    &     MOS,ITERS)
*     
*     
*     * Parameters *
*      
*     II  N             Number of variables.
*     II  NA            Maximum size of the bundle.
*     II  MAL           Current size of the bundle.
*     RU  X(N)          Vector of variables.
*     RU  AF(4*NA)      Vector of values of bundle functions.
*     RU  AG(N*NA)      Matrix whose columns are bundle subgradients.
*     RU  AY(N*NA)      Matrix whose columns are bundle points.
*     II  IBUN          Index for the circular arrays in bundle.
*     RI  D(N)          Direction vector.
*     RI  F             Value of the objective function.
*     RI  DF            Directional derivative.
*     RO  T             Value of the stepsize parameter.
*     RI  TMIN          Lower limit for stepsize parameter.
*     RI  TMAX          Upper limit for stepsize parameter.
*     RI  ETA           Distance measure parameter.
*     RI  ETA9          Maximum for real numbers.
*     RI  MOS           Locality measure parameter.
*     II  ITERS         Null step indicator.
*                            0  - Null step.
*                            1  - Serious step.
*     
*
*     * Subprograms used *
*      
*     S   DESTEP        Stepsize determination for descent steps.
*     S   NULSTP        Stepsize determination for null steps.
*
*      
*
*     Napsu Haarala (2003)
*
      
      
      SUBROUTINE TINIT(N,NA,MAL,X,AF,AG,AY,IBUN,D,F,P,T,TMAX,TMIN,ETA,
     &     ETA9,MOS,ITERS)

*     Scalar Arguments
      INTEGER N,NA,MAL,IBUN,MOS,ITERS
      DOUBLE PRECISION P,ETA,ETA9,F,T,TMAX,TMIN

*     Array Arguments
      DOUBLE PRECISION AF(*),AG(*),AY(*),D(*),X(*)

*     External Subroutines
      EXTERNAL DESTEP,NULSTP

*     Intrinsic Functions
      INTRINSIC MAX,MIN

      T = MIN(1.0D+00,TMAX)

      IF (P .EQ. 0.0D+00) GO TO 10

      IF (ITERS.EQ.1) THEN
         CALL DESTEP(N,NA,MAL,X,AF,AG,AY,IBUN,D,F,P,T,ETA,ETA9,MOS)

      ELSE
         CALL NULSTP(N,NA,MAL,X,AF,AG,AY,IBUN,D,F,P,T,ETA,ETA9,MOS)
      END IF

 10   CONTINUE
      T = MIN(MAX(T,TMIN),TMAX)

      RETURN
      END


************************************************************************
*
*     * SUBROUTINE DESTEP *
*
*      
*     * Purpose *
*      
*     Stepsize selection using polyhedral approximation for descent step
*     in limited memory bundle method.
*
*
*     * Calling sequence *
*     
*     CALL DESTEP(N,NA,MAL,X,AF,AG,AY,IBUN,D,F,DF,T,ETA,ETA9,MOS)
*     
*     
*     * Parameters *
*      
*     II  N             Number of variables.
*     II  NA            Maximum size of the bundle.
*     II  MAL           Current size of the bundle.
*     RU  X(N)          Vector of variables.
*     RU  AF(4*NA)      Vector of values of bundle functions.
*     RU  AG(N*NA)      Matrix whose columns are bundle subgradients.
*     RU  AY(N*NA)      Matrix whose columns are bundle points.
*     II  IBUN          Index for the circular arrays in bundle.
*     RI  D(N)          Direction vector.
*     RI  F             Value of the objective function.
*     RI  DF            Directional derivative.
*     RO  T             Value of the stepsize parameter.
*     RI  ETA           Distance measure parameter.
*     RI  ETA9          Maximum for real numbers.
*     RI  MOS           Locality measure parameter.
*
*
*      
*     * Original method *
*      
*     PNSTP4 by J. Vleck (1999)
*
*      
*     * Limited memory version *
*
*     Napsu Haarala (2002,2003)
*      


      SUBROUTINE DESTEP(N,MA,MAL,X,AF,AG,AY,IBUN,D,F,DF,T,ETA,ETA9,MOS)

*     Scalar Arguments
      INTEGER N,MA,MAL,MOS,IBUN
      DOUBLE PRECISION DF,ETA,ETA9,F,T

*     Array Arguments
      DOUBLE PRECISION AF(*),AG(*),AY(*),D(*),X(*)

*     Local Scalars
      INTEGER I,J,JN,K,L,LQ,IB
      DOUBLE PRECISION ALF,ALFL,ALFR,BET,BETL,BETR,DX,Q,R,W

*     Intrinsic Functions
      INTRINSIC ABS,DBLE,MAX,MIN,SQRT

      ALFL=0.0D+00
      BETL=0.0D+00
      
      W = DF*T* (1.0D0-T*0.5D0)
      
      
*     
*     Initial choice of possibly active lines
*      

      K = 0
      L = -1
      JN = (IBUN-1)*N
      BETR = -ETA9
      DO 20 J = 1,MAL - 1
         IB = IBUN - 1 + J
         IF (IB .GT. MAL) IB = IB - MAL
         IF (JN .GE. MAL*N) JN = JN - MAL*N
         R = 0.0D0
         BET = 0.0D0
         ALFL = AF(IB) - F
         DO 10 I = 1,N
            DX = X(I) - AY(JN+I)
            Q = AG(JN+I)
            R = R + DX*DX
            ALFL = ALFL + DX*Q
            BET = BET + D(I)*Q
 10      CONTINUE
         IF (MOS.NE.2) R = R** (DBLE(MOS)*0.5D0)
         ALF = MAX(ABS(ALFL),ETA*R)
         R = 1.0D0 - BET/DF
         IF (R*R+ (ALF+ALF)/DF.GT.1.0D-6) THEN
            K = K + 1
            AF(MA+K) = ALF
            AF(MA+MA+K) = BET
            R = T*BET - ALF
            IF (R.GT.W) THEN
               W = R
               L = K
            END IF

         END IF

         BETR = MAX(BETR,BET-ALF)
         JN = JN + N
         
 20   CONTINUE
      LQ = -1
      IF (BETR.LE.DF*0.5D0) RETURN
      LQ = 1
      IF (L.LT.0) RETURN
      BETR = AF(MA+MA+L)
      IF (BETR.LE.0.0D0) THEN
         IF (T.LT.1.0D0 .OR. BETR.EQ.0.0D0) RETURN
         LQ = 2
      END IF

      ALFR = AF(MA+L)

      
*
*     Iteration loop
*

 30   IF (LQ.GE.1) THEN
         Q = 1.0D0 - BETR/DF
         R = Q + SQRT(Q*Q+ (ALFR+ALFR)/DF)
         IF (BETR.GE.0.0D0) R = - (ALFR+ALFR)/ (DF*R)
         R = MIN(1.95D0,MAX(0.0D0,R))
         
      ELSE
         IF (ABS(BETR-BETL)+ABS(ALFR-ALFL).LT.-1.0D-4*DF) RETURN
         R = (ALFR-ALFL)/ (BETR-BETL)
      END IF

      IF (ABS(T-R).LT.1.0D-4) RETURN
      T = R
      AF(MA+L) = -1.0D0
      W = T*BETR - ALFR
      L = -1
      DO 40 J = 1,K
         ALF = AF(MA+J)
         IF (ALF.LT.0.0D0) GO TO 40
         BET = AF(MA+MA+J)
         R = T*BET - ALF
         IF (R.GT.W) THEN
            W = R
            L = J
         END IF

 40   CONTINUE
      IF (L.LT.0) RETURN
      BET = AF(MA+MA+L)
      IF (BET.EQ.0.0D0) RETURN

      
*
*     New interval selection
*

      ALF = AF(MA+L)
      IF (BET.LT.0.0D0) THEN
         IF (LQ.EQ.2) THEN
            ALFR = ALF
            BETR = BET
            
         ELSE
            ALFL = ALF
            BETL = BET
            LQ = 0
         END IF

      ELSE
         IF (LQ.EQ.2) THEN
            ALFL = ALFR
            BETL = BETR
            LQ = 0
         END IF

         ALFR = ALF
         BETR = BET
      END IF

      GO TO 30

      END


************************************************************************
*
*     * SUBROUTINE NULSTP *
*
*
*     * Purpose *
*      
*     Stepsize selection using polyhedral approximation for null step
*     in limited memory bundle method.
*
*      
*     * Calling sequence *
*     
*     CALL NULSTP(N,NA,MAL,X,AF,AG,AY,IBUN,D,F,DF,T,ETA,ETA9,MOS)
*     
*     
*     * Parameters *
*      
*     II  N             Number of variables.
*     II  NA            Maximum size of the bundle.
*     II  MAL           Current size of the bundle.
*     RU  X(N)          Vector of variables.
*     RU  AF(4*NA)      Vector of values of bundle functions.
*     RU  AG(N*NA)      Matrix whose columns are bundle subgradients.
*     RU  AY(N*NA)      Matrix whose columns are bundle points.
*     II  IBUN          Index for the circular arrays in bundle.
*     RI  D(N)          Direction vector.
*     RI  F             Value of the objective function.
*     RI  DF            Directional derivative.
*     RO  T             Value of the stepsize parameter.
*     RI  ETA           Distance measure parameter.
*     RI  ETA9          Maximum for real numbers.
*     RI  MOS           Locality measure parameter.
*
*      
*     * Original method *
*      
*     PNSTP5 by J. Vleck (1999)
*
*      
*     * Limited memory version *
*
*     Napsu Haarala (2002,2003)
*      

      
      SUBROUTINE NULSTP(N,MA,MAL,X,AF,AG,AY,IBUN,D,F,DF,T,ETA,ETA9,MOS)

*     Scalar Arguments
      INTEGER MA,MAL,MOS,N,IBUN
      DOUBLE PRECISION DF,ETA,ETA9,F,T

*     Array Arguments
      DOUBLE PRECISION AF(*),AG(*),AY(*),D(*),X(*)

*     Local Scalars
      INTEGER I,J,JN,K,L,IB
      DOUBLE PRECISION ALF,ALFL,ALFR,BET,BETL,BETR,DX,Q,R,W

*     Intrinsic Functions
      INTRINSIC ABS,DBLE,MAX,MIN,SQRT

      W = DF*T

*     
*     Initial choice of possibly active parabolas
*

      K = 0
      L = -1
      JN = (IBUN-1)*N
      BETR = -ETA9
      DO 20 J = 1,MAL - 1
         IB = IBUN - 1 + J
         IF (IB .GT. MAL) IB = IB - MAL
         IF (JN .GE. MAL*N) JN = JN - MAL*N
         BET = 0.0D0
         R = 0.0D0
         ALFL = AF(IB) - F
         DO 10 I = 1,N
            DX = X(I) - AY(JN+I)
            R = R + DX*DX
            Q = AG(JN+I)
            ALFL = ALFL + DX*Q
            BET = BET + D(I)*Q
 10      CONTINUE
         IF (MOS.NE.2) R = R**(DBLE(MOS)*0.5D0)
         ALF = MAX(ABS(ALFL),ETA*R)
         BETR = MAX(BETR,BET-ALF)
         IF (ALF.LT.BET-DF) THEN
            K = K + 1
            R = T*BET - ALF
            AF(MA+K) = ALF
            AF(MA+MA+K) = BET
            IF (R.GT.W) THEN
               W = R
               L = K
            END IF

         END IF

         JN = JN + N
 20   CONTINUE
      IF (L.LT.0) RETURN
      BETR = AF(MA+MA+L)
      ALFR = AF(MA+L)
      ALF = ALFR
      BET = BETR
      ALFL = 0.0D0
      BETL = DF


*     
*     Iteration loop
*

 30   W = BET/DF
      IF (ABS(BETR-BETL)+ABS(ALFR-ALFL).LT.-1.0D-4*DF) RETURN
      IF (BETR-BETL.EQ.0.0D0) STOP 11
      R = (ALFR-ALFL)/ (BETR-BETL)
      IF (ABS(T-W).LT.ABS(T-R)) R = W
      Q = T
      T = R
      IF (ABS(T-Q).LT.1.0D-3) RETURN
      AF(MA+L) = -1.0D0
      W = T*BET - ALF
      L = -1
      DO 40 J = 1,K
         ALF = AF(MA+J)
         IF (ALF.LT.0.0D0) GO TO 40
         BET = AF(MA+MA+J)
         R = T*BET - ALF
         IF (R.GT.W) THEN
            W = R
            L = J
         END IF

 40   CONTINUE
      IF (L.LT.0) RETURN
      BET = AF(MA+MA+L)
      Q = BET - T*DF
      IF (Q.EQ.0.0D0) RETURN

      
*     
*     New interval selection
*
      ALF = AF(MA+L)
      IF (Q.LT.0.0D0) THEN
         ALFL = ALF
         BETL = BET

      ELSE
         ALFR = ALF
         BETR = BET
      END IF

      GO TO 30

      END


************************************************************************
*
*     * SUBROUTINE DOBUN *
*
*
*     * Purpose *
*      
*     Bundle construction for limited memory bundle method
*
*      
*     * Calling sequence *
*     
*     CALL DOBUN(N,NA,MAL,X,G,F,AY,AG,AF,ITERS,IBUN)
*     
*     
*     * Parameters *
*      
*     II  N             Number of variables.
*     II  NA            Maximum size of the bundle.
*     II  MAL           Current size of the bundle.
*     RI  X(N)          Vector of variables.
*     RI  G(N)          Subgradient of the objective function.
*     RI  F             Value of the objective function.
*     RU  AY(N*NA)      Matrix whose columns are bundle points.
*     RU  AG(N*NA)      Matrix whose columns are bundle subgradients.
*     RU  AF(4*NA)      Vector of values of bundle functions.
*     IU  IBUN          Index for the circular arrays.
*     II  ITERS         Null step indicator.
*                            0  - Null step.
*                            1  - Serious step.
*
*      
*     * Subprograms used *
*
*     S   COPY2         Copying of two vectors.
*
*      
*      Napsu Haarala (2003)
*

      
      SUBROUTINE DOBUN(N,MA,MAL,X,G,F,AY,AG,AF,ITERS,IBUN)

*     Scalar Arguments
      INTEGER ITERS,MA,MAL,N,IBUN
      DOUBLE PRECISION F
      
*     Array Arguments
      DOUBLE PRECISION AF(*),AG(*),AY(*),G(*),X(*)
      
*     Local Scalars
      INTEGER I,J
      
*     External Subroutines
      EXTERNAL COPY2


      IF (ITERS .EQ. 1) THEN


*     
*     Serious step
*      

         AF(IBUN) = F
         I = (IBUN-1)*N + 1
         CALL COPY2(N,G,AG(I),X,AY(I))

      ELSE


*
*     Null step
*      

         IF (MAL .LT. MA) THEN

            AF(IBUN) = AF(MAL)
            AF(MAL) = F

            I = MAL*N + 1
            CALL COPY2(N,AG(I-N),AG(I),AY(I-N),AY(I))
            CALL COPY2(N,G,AG(I-N),X,AY(I-N))
            
         ELSE
            I = IBUN-1
            IF (I .LT. 1) I = MAL
            AF(IBUN) = AF(I)
            AF(I) = F

            I = (IBUN-2)*N + 1
            IF (I .LT. 1) I = (MAL-1)*N + 1
            J = (IBUN-1)*N + 1
            CALL COPY2(N,AG(I),AG(J),AY(I),AY(J))
            CALL COPY2(N,G,AG(I),X,AY(I))
         END IF


      END IF
      
      MAL = MAL + 1
      IF (MAL .GT. MA) MAL = MA

      IBUN = IBUN + 1
      IF (IBUN .GT. MA) IBUN = 1

      RETURN
      END
      
      
************************************************************************
*
*     * SUBROUTINE RESTAR *
*
*      
*     * Purpose *
*      
*     Initialization.
*
*      
*     * Calling sequence *
*     
*     CALL RESTAR(N,MC,MCC,MCINIT,INEW,IBUN,IBFGS,ITERS,GP,G,NNK,ALFV,
*    &     ALFN,GAMMA,D,IC,ICN,MAL,NCRES,IFLAG)
*     
*     
*     * Parameters *
*      
*     II  N               Number of variables.
*     IO  MC              Current maximum number of stored corrections.
*     IO  MCC             Current number of stored corrections.
*     II  MCINIT          Initial maximum number of stored corrections.
*     IO  INEW            Index for the circular arrays.
*     IO  IBUN            Index for the circular arrays in bundle
*                           updating.
*     IO  IBFGS           Index of the type of BFGS update.
*     IU  ITERS           Null step indicator.
*                              0  - Null step.
*                              1  - Serious step.
*     IO  NNK             Consecutive null steps counter.
*     RI  GP(N)           Basic subgradient of the objective function.
*     RO  G(N)            Current (auxiliary) subgradient of the
*                           objective function.
*     RO  ALFN            Locality measure.
*     RO  ALFV            Aggregate locality measure.
*     RO  D(N)            Search direction.
*     RO  GAMMA           Scaling parameter.
*     IO  IC              Correction indicator.
*     IO  ICN             Correction indicator for null steps.
*     IO  MAL             Current size of the bundle.
*     IO  NCRES           Number of restarts.
*     IO  IFLAG           Index for adaptive version.
*
*      
*     * Subprograms used *
*      
*     S   COPY            Copying of a vector.
*     S   VNEG            Copying of a vector with change of the sign.
*     
*     
*     Napsu Karmitsa (2004, last modified 2007)
*      
  
      
      SUBROUTINE RESTAR(N,MC,MCC,MCINIT,INEW,IBUN,IBFGS,ITERS,GP,G,NNK,
     &     ALFV,ALFN,GAMMA,D,IC,ICN,MAL,NCRES,IFLAG)

*     Scalar Arguments
      INTEGER N,MC,MCC,MCINIT,INEW,IBUN,IBFGS,ITERS,NNK,IC,ICN,MAL,
     &     NCRES,IFLAG
      DOUBLE PRECISION ALFN,ALFV,GAMMA

*     Array Arguments
      DOUBLE PRECISION G(*),GP(*),D(*)

*     External Subroutines
      EXTERNAL COPY,VNEG

      MC = MCINIT
      MCC = 0
      INEW = 1
      IBUN = 1
      IBFGS = 0
      IC = 0
      ICN = 0
      MAL = 0
      NCRES = NCRES + 1
      IFLAG = 0

      IF (ITERS .EQ. 0) THEN
         CALL COPY(N,GP,G)
         ITERS = 1
         NNK = 0
         ALFV=0.0D+00
         ALFN=0.0D+00
      END IF

      GAMMA=1.0D+00
      CALL VNEG(N,G,D)
      
      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE RPRINT *
*
*      
*     * Purpose *
*      
*     Printout the (final) results.
*
*      
*     * Calling sequence *
*     
*     SUBROUTINE RPRINT(N,NIT,NFE,X,F,WK,QK,ITERM,IPRINT,NOUT)
*     
*     
*     * Parameters *
*      
*     II  N               Number of variables.
*     II  NIT             Number of used iterations.
*     II  NFE             Number of used function evaluations.
*     RI  X(N)            Vector of variables.
*     RI  F               Value of the objective function.
*     RI  WK              Value of the first stopping criterion.
*     RI  QK              Value of the second stopping criterion.
*     II  ITERM           Cause of termination:
*                             1  - The problem has been solved.
*                                  with desired accuracy.
*                             2  - (F - FO) < TOLF in MTESF
*                                  subsequent iterations.
*                             3  - (F - FO) < TOLF*SMALL*MAX(|F|,|FO|,1).
*                             4  - Number of function calls > MFV.
*                             5  - Number of iterations > MIT.
*                             6  - Time limit exceeded. 
*                             7  - F < TOLB.
*                            -1  - Two consecutive restarts.
*                            -2  - Number of restarts > maximum number
*                                  of restarts.
*                            -3  - Failure in function or subgradient
*                                  calculations (assigned by the user).
*                            -4  - Failure in attaining the demanded
*                                  accuracy.
*                            -5  - Invalid input parameters.
*                            -6  - Not enough working space.
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
*     
*      
*     
*     Napsu Karmitsa (2004, last modified 2007)
*      

      
      SUBROUTINE RPRINT(N,NIT,NFE,X,F,WK,QK,ITERM,IPRINT)

*     Scalar Arguments
      INTEGER N,NIT,NFE,ITERM,IPRINT
      DOUBLE PRECISION F,WK,QK

*     Array Arguments
      DOUBLE PRECISION X(*)

         
*
*     Intermediate results
*

      IF (ITERM .EQ. 0) THEN
         IF (IPRINT .GT. 3) WRITE (6,FMT='(1X,''NIT='',I5,2X,
     &        ''NFE='',I5,2X,''F='',D15.8,2X,''WK='',D11.4,2X,
     &        ''QK='',D11.4,2X)')
     &        NIT,NFE,F,WK,QK
         IF (IPRINT .EQ. 5) WRITE (6,FMT='(1X,''X='',
     &        5D15.7:/(4X,5D15.7))')(X(I),I=1,N)
         RETURN
      END IF

         
*
*     Printout the final results
*

      IF (IPRINT .GT. 0) WRITE (6,FMT='(1X,''NIT='',I5,2X,
     &     ''NFE='',I5,2X,''F='',D15.8,2X,''WK='',D11.4,2X,
     &     ''QK='',D11.4,2X,''ITERM='',I3)')
     &     NIT,NFE,F,WK,QK,ITERM
      IF (IPRINT .EQ. 3 .OR. IPRINT .EQ. 5)
     &     WRITE (6,FMT='(1X,''X='',5D15.7:/(4X,5D15.7))')(X(I),I=1
     $     ,N)
      
      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE WPRINT *
*
*      
*     * Purpose *
*      
*     Printout the warning and error messages.
*
*      
*     * Calling sequence *
*     
*     SUBROUTINE WPRINT(ITERM,IPRINT,NOUT)
*     
*     
*     * Parameters *
*      
*     II  ITERM           Cause of termination:
*                             1  - The problem has been solved.
*                                  with desired accuracy.
*                             2  - (F - FO) < TOLF*MAX(|F|,1) in MTESF
*                                  subsequent iterations.
*                             3  - (F - FO) < TOLF*SMALL*MAX(|F|,|FO|,1).
*                             4  - Number of function calls > MFV.
*                             5  - Number of iterations > MIT.
*                             6  - Time limit exceeded. 
*                             7  - F < TOLB.
*                             8  - F < cancelled by user
*                            -1  - Two consecutive restarts.
*                            -2  - Number of restarts > maximum number
*                                  of restarts.
*                            -3  - Failure in function or subgradient
*                                  calculations (assigned by the user).
*                            -4  - Failure in attaining the demanded
*                                  accuracy.
*                            -5  - Invalid input parameters.
*                            -6  - Not enough working space.
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
*     II  NOUT            Auxilary printout specification.
*     
*      
*
*     Napsu Karmitsa (2004, last modified 2007)
*      

      
      SUBROUTINE WPRINT(ITERM,IPRINT,NOUT)

*     Scalar Arguments
      INTEGER ITERM,IPRINT,NOUT


      IF (IPRINT .GE. 0) THEN


*     
*     Initial error messages
*

         IF (ITERM .LE. -5) THEN
            IF (ITERM .EQ. -5) THEN
               IF (NOUT .EQ. 1) WRITE (6,FMT='(1X,''Error: ''
     &              ''Number of variables (N) is too small. ITERM='',I3)
     &              ')ITERM
               IF (NOUT .EQ. 2) WRITE (6,FMT='(1X,''Error: ''
     &              ''The maximum number of stored corrections (MCU) ''
     &              ''is too small. ITERM='',I3)')ITERM
               IF (NOUT .EQ. 3) WRITE (6,FMT='(1X,''Error: ''
     &              ''The size of the bundle (NA) is too small. ITERM=''
     &              ,I3)')ITERM
               IF (NOUT .EQ. 4) WRITE (6,FMT='(1X,''Error: ''
     &              ''Line search parameter RPAR(6) >= 0.25. ITERM=''
     &              ,I3)')ITERM
            ELSE IF (ITERM .EQ. -6) THEN
               WRITE (6,FMT='(1X,''Error: ''
     &              ''Not enough working space. ITERM='',I3)')ITERM
               
            END IF
            RETURN
         END IF


         
*
*     Warning messages
*

         IF (IPRINT .GE. 2) THEN
            IF (ITERM .EQ. 0) THEN
               IF (NOUT .EQ. -1) WRITE (6,FMT='(1X,''Warning: ''
     &              ''MC > MCU. Assigned MC = MCU.'')')
               IF (NOUT .EQ. -2) WRITE (6,FMT='(1X,''Warning: ''
     &              ''A line search parameter EPSR >= 0.5.'')')
               IF (NOUT .EQ. -3) WRITE (6,FMT='(1X,''Warning: ''
     &              ''A nondescent search direction occured. Restart.'')
     &              ')
               IF (NOUT .EQ. -4) WRITE (6,FMT='(1X,''Warning: ''
     &              ''Does not converge.'')')
               IF (NOUT .EQ. -5) WRITE (6,FMT='(1X,''Warning: ''
     &              ''TMAX < TMIN. Restart.'')')
               
               RETURN
            END IF
         

*
*     Printout the final results
*
            
            IF (ITERM .EQ. 6) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''Time is up.'')')
            IF (ITERM .EQ. 7) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''F < TOLB.'')')
            IF (ITERM .EQ. 8) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''cancelled by user'')')
            IF (ITERM .EQ. 2) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''Too many steps without significant progress.'')
     &           ')
            IF (ITERM .EQ. 3) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''The value of the function does not change.'')')
            IF (ITERM .EQ. 5) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''Number of iterations > '',I5)') NOUT
            IF (ITERM .EQ. 4) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''Number of function evaluations > '',I5)') NOUT
            IF (ITERM .EQ. -1) THEN
               IF (NOUT .EQ. -1) THEN
                  WRITE (6,FMT='(1X,''Abnormal exit: ''
     &                 ''Two consecutive restarts.'')')
               ELSE
                  WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''TMAX < TMIN in two subsequent iterations.'')')
               END IF
            END IF
            IF (ITERM .EQ. -2) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &                 ''Number of restarts > '',I5''.'')') NOUT
            IF (ITERM .EQ. -3) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''Failure in function or subgradient calculations.'')')
            IF (ITERM .EQ. -4) WRITE (6,FMT='(1X,''Abnormal exit: ''
     &           ''Failure in attaining the demanded accuracy.'')')
         END IF

      END IF
      
      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE INDIC1 *
*
*      
*     * Purpose *
*      
*     Initialization of indices.
*     
*      
*     * Calling sequence *
*     
*     CALL INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,ITYPE)
*
*      
*     * Parameters *
*
*     II  MC              Declared number of stored corrections.
*     II  MCC             Current number of depositories used.
*     IO  MCNEW           Current size of vectors.
*     II  ITYPE           Type of Initialization:
*                             1  - corrections are stored,
*                             2  - corrections are not stored,
*                             3  - update is skipped.
*     IU  INEW            Index for circular arrays.
*     IO  IOLD            Index of the oldest corrections.
*     IU  IFLAG           Index for adaptive version:
*                             0  - Maximum number of stored corrections
*                                    has not been changed at previous
*                                    iteration.
*                             1  - Maximum number of stored corrections
*                                    has been changed at previous
*                                    iteration.
*     IU  IFLAG2          Index for adaptive version:
*                             0  - IFLAG has not been changed.
*                             1  - IFLAG has been changed.
*     
*      
*
*     Napsu Haarala (2002,2003, Last modified 2005)
*      


      SUBROUTINE INDIC1(MC,MCC,MCNEW,INEW,IOLD,IFLAG,IFLAG2,ITYPE)
      
*     Scalar Arguments
      INTEGER MC,MCC,INEW,IFLAG,MCNEW,IOLD,IFLAG2,ITYPE

      
      IF (ITYPE .EQ. 1) THEN
         IF (MCC .LT. MC) THEN
            MCNEW = MCC + 1
            IOLD = 1
            IFLAG = 0

         ELSE
            IF (IFLAG .EQ. 0) THEN
               MCNEW = MC
               IOLD = INEW + 2
               IF (IOLD .GT. MC+1) IOLD = IOLD - MC - 1

            ELSE
               IF (INEW .EQ. 1) THEN
                  INEW = MC + 1
                  MCNEW = MC
                  IOLD = 2
                  IFLAG = 0
                  IFLAG2 = 1

               ELSE IF (INEW .EQ. MC) THEN
                  MCNEW = MC
                  IOLD = 1
                  IFLAG = 0
                  IFLAG2 = 1

               ELSE
                  MCNEW = MC - 1
                  IOLD = INEW + 2
                  IF (IOLD .GT. MC) IOLD = IOLD - MC

               END IF

            END IF

         END IF
      
      ELSE IF (ITYPE .EQ. 2) THEN

         IF (MCC .LT. MC) THEN
            MCNEW = MCC + 1
            IOLD = 1
            IFLAG = 0

         ELSE
            IF (IFLAG .EQ. 0) THEN
               MCNEW = MC + 1
               IOLD = INEW + 1
               IF (IOLD .GT. MC + 1) IOLD = 1

            ELSE
               MCNEW = MC
               IOLD = INEW + 1
               IF (IOLD .GT. MC) IOLD = 1
            END IF
               
         END IF
      
      ELSE 
         
         IF (MCC .LT. MC) THEN
            MCNEW = MCC
            IOLD = 1
            IFLAG = 0

         ELSE
            IF (IFLAG .EQ. 0) THEN
               MCNEW = MC
               IOLD = INEW + 1
               IF (IOLD .GT. MC + 1) IOLD = 1
               
            ELSE
               MCNEW = MC - 1
               IOLD = INEW + 1
               IF (IOLD .GT. MC) IOLD = 1
            END IF
            
         END IF

      END IF
      
      RETURN
      END


************************************************************************
*
*     * DOUBLE PRECISION FUNCTION SCLPAR *
*
*      
*     * Purpose *
*      
*     Calculation of the scaling parameter appointed by parameter ISCALE.
*
*      
*     * Calling sequence *
*      
*      GAMMA = SCLPAR(MCC,ISCALE,METHOD,STS,STU,UTU,SMALL)
*
*
*      
*     * Parameters *
*
*     II  MCC             Current number of depositories used.
*     RI  STS             STS = TRANS(S)*S. 
*     RI  STU             STU = TRANS(S)*U. 
*     RI  UTU             UTU = TRANS(U)*U. 
*     RI  SMALL           Small positive value.
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
*
*      
*     Napsu Karmitsa (2004, Last modified 2007)
*


      DOUBLE PRECISION FUNCTION SCLPAR(MCC,ISCALE,METHOD,STS,STU,UTU,
     &     SMALL)
      
*     Scalar Arguments
      INTEGER MCC,METHOD,ISCALE
      DOUBLE PRECISION STS,STU,UTU,SMALL
      
*     Intrinsic Functions
      INTRINSIC SQRT


*        
*     Computation of scaling parameter.
*

      SCLPAR =1.0D+00


*
*     Scaling parameter = STU/UTU
*            

      IF (ISCALE .EQ. 0 .OR. ISCALE .EQ. 2 .OR. ISCALE .EQ. 4)
     &     THEN
         IF (UTU .LT. SQRT(SMALL)) THEN
            SCLPAR = 1.0D+00
            GO TO 80
         ELSE
            SCLPAR = STU/UTU
         END IF

         
*     
*     Scaling parameter = STS/STU
*               

      ELSE IF (ISCALE .EQ. 1 .OR. ISCALE .EQ. 3 .OR.
     &        ISCALE .EQ. 5) THEN
         IF (STU .LT. SQRT(SMALL)) THEN
            SCLPAR = 1.0D+00
            GO TO 80
         ELSE
            SCLPAR = STS/STU
         END IF
      ELSE


*     
*     No scaling
*               

         SCLPAR = 1.0D+00
         GO TO 80
      END IF

               
*     
*     Scaling
*               
            
      IF (MCC .EQ. 0) THEN
         IF (SCLPAR .LT. 0.01D+00) SCLPAR=0.01D+00
         IF (SCLPAR .GT. 100.0D+00) SCLPAR=100.0D+00


*               
*     Interval scaling
*               

      ELSE IF (ISCALE .EQ. 2) THEN
         IF (METHOD .EQ. 0) THEN
            IF (SCLPAR .LT. 0.6D+00 .OR. SCLPAR .GT. 6.0D+00) THEN
               SCLPAR = 1.0D+00
            END IF
         ELSE
            IF (SCLPAR .LT. 0.01D+0 .OR. SCLPAR .GT. 100.0D+0) THEN
               SCLPAR = 1.0D+00
            END IF
         END IF

      ELSE IF (ISCALE .EQ. 3) THEN
         IF (SCLPAR .LT. 0.5D+00 .OR. SCLPAR .GT. 5.0D+00) THEN
            SCLPAR = 1.0D+00
         END IF
         
               
*     
*     Preliminary scaling
*     

      ELSE IF (ISCALE .EQ. 4 .OR. ISCALE .EQ. 5) THEN
         SCLPAR = 1.0D+00
               

*     
*     Scaling at every iteration
*               

      ELSE
         CONTINUE
      END IF


 80   CONTINUE

      IF (SCLPAR .LE. 1.0D+03*SMALL) SCLPAR = 1.0D+03*SMALL
         
      RETURN
      END


************************************************************************
*
*     * SUBROUTINE GETIME *
*
*      
*     * Purpose *
*      
*     Execution time.
*     
*      
*     * Calling sequence *
*     
*     CALL GETIME(CTIM,RTIM)
*
*      
*     * Parameters *
*     
*     RO  CTIM          Current time. REAL argument
*     RA  RTIM(2)       Auxiliary array. REAL array.
*     
*     
*     * Subprograms used *
*      
*     RF  ETIME         Execution time.
*      
*
*     Napsu Karmitsa (2007)
*      

      SUBROUTINE GETIME(CTIM,RTIM)
      
*     Scalar Arguments
      REAL CTIM

*     Array arguments
      REAL RTIM(2)

*     Intrinsic Functions
      INTRINSIC ETIME


      CTIM = ETIME(RTIM)
      CTIM = RTIM(1)
      
      RETURN
      END

************************************************************************
