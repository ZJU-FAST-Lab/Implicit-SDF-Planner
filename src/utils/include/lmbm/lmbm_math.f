************************************************************************
*
*
*     LMBM_MATH includes the following subroutines
*
*     S   CALQ            Solving X from linear equation A*X=Y.
*     S   COPY            Copying of vector.
*     S   COPY2           Copying of two vectors.
*     S   CWMAXV          Multiplication of a columnwise stored dense 
*                           rectangular matrix by a vector.
*     S   LINEQ           Solver for linear equation.
*     S   MXDPGF          Gill-Murray decomposition of a dense symmetric
*                           matrix. Coded by L. Luksan.
*     S   RWAXV2          Multiplication of two rowwise stored dense 
*                           rectangular matrices A and B by vectors X 
*                           and Y.
*     S   SCALEX          Scaling a vector.
*     S   SCDIFF          Difference of the scaled vector and a vector.
*     S   SCSUM           Sum of a vector and the scaled vector.
*     S   SYMAX           Multiplication of a dense symmetric matrix
*                           by a vector.
*     S   TRLIEQ          Solving X from linear equation L*X=Y or
*                           L'*X=Y.
*     S   VNEG            Change the signs of vector elements.
*     S   VXDIAG          Multiplication of a vector and a diagonal
*                           matrix.
*     S   XDIFFY          Difference of two vectors.
*     S   XSUMY           Sum of two vectors.
*     RF  EPS0            The smallest positive number such that
*                           1.0 + EPS0 > 1.0. 
*     RF  VDOT            Dot product of two vectors.
*
*
************************************************************************
*
*     * SUBROUTINE SYMAX *
*
*      
*     * Purpose *
*      
*     Multiplication of a dense symmetric matrix A by a vector X.
*
*      
*     * Calling sequence *
*     
*     CALL SYMAX(N,M,IOLD,A,X,Y)
*
*      
*     * Parameters *
*      
*     II  N               Order of matrix A.
*     RI  A(N*(N+1)/2)    Dense symmetric matrix stored in the packed
*                           form.
*     II  M               Length of vector X, M >= N, note that only N
*                           components from vector X are used.
*     RI  X(M)            Input vector stored in a circular order.
*     II  IOLD            Index, which controlls the circular order of
*                           the vector X.
*     RO  Y(M)            Output vector equal to A*X. The vector Y has
*                           the same circular order than X.
*
*      
*     Napsu Haarala (2002).
*      

      
      SUBROUTINE SYMAX(N,M,IOLD,A,X,Y)

*     Scalar Arguments
      INTEGER N,M,IOLD
      
*     Array Arguments
      DOUBLE PRECISION A(*),X(*),Y(*)
      
*     Local Scalars
      INTEGER I,J,K,L
      

      DO 20 J=1,N
         L=J+IOLD-1
         IF (L .GT. M) L=L-M
         Y(L)=0.0D+00
         K=L
         DO 10 I=J,N
            Y(L) = A((I-1)*I/2+J)*X(K)+Y(L)
            K=K+1
            IF (K .GT. M) K=K-M
 10      CONTINUE
 20   CONTINUE

      DO 40 J=2,N
         L=J+IOLD-1
         IF (L .GT. M) L=L-M
         K=IOLD
         DO 30 I=1,J-1
            IF (K .GT. M) K=K-M
            Y(L) = A((J-1)*J/2+I)*X(K)+Y(L)
            K=K+1
 30      CONTINUE
 40   CONTINUE
      
      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE CWMAXV *
*
*      
*     * Purpose *
*
*     Multiplication of a columnwise stored dense rectangular matrix A
*     by a vector X with a possibility of scaling.
*
*     
*     * Calling sequence *
*     
*     CALL CWMAXV(N,M,A,X,Y,S)
*     
*     
*     * Parameters *
*
*     II  N             Number of rows of the matrix A.
*     II  M             Number of columns of the matrix A.
*     RI  A(N*M)        Rectangular matrix stored columnwise in the
*                       one-dimensional array.
*     RI  X(M)          Input vector.
*     RI  S             Scaling parameter.
*     RO  Y(N)          Output vector equal to S*A*X. If M = 0 Y is 
*                         a zero vector. 
*
*      
*     * Subprograms used *
*      
*     S   SCSUM         Sum of a vector and the scaled vector.
*
*      
*     * Napsu Haarala, 2006
*      

      SUBROUTINE CWMAXV(N,M,A,X,Y,S)
      
*     Scalar Arguments
      INTEGER M,N
      DOUBLE PRECISION S
      
*     Array Arguments
      DOUBLE PRECISION A(*),X(*),Y(*)
      
*     Local Scalars
      INTEGER I,J,K
      
*     External Subroutines
      EXTERNAL SCSUM

      DO 10 I = 1,N
         Y(I) = 0.0D+00
 10   CONTINUE

      K = 0
      DO 20 J = 1,M
         CALL SCSUM(N,S*X(J),A(K+1),Y,Y)
         K = K + N
 20   CONTINUE

      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE RWAXV2 *
*
*      
*     * Purpose *
*
*     Multiplication of two rowwise stored dense rectangular matrices A
*     and B by vectors X and Y.
*
*     
*     * Calling sequence *
*     
*     CALL RWAXV2(N,M,A,B,X,Y,V,W)
*     
*     
*     * Parameters *
*
*     II  N             Number of columns of the matrices A and B.
*     II  M             Number of rows of the matrices A and B.
*     RI  A(N*M)        Rectangular matrix stored rowwise in the
*                       one-dimensional array.
*     RI  B(N*M)        Rectangular matrix stored rowwise in the
*                       one-dimensional array.
*     RI  X(N)          Input vector.
*     RI  Y(N)          Input vector.
*     RO  V(M)          Output vector equal to A*X.
*     RO  W(M)          Output vector equal to B*Y.
*
*          
*     Napsu Haarala, 2006 
*      
*
      SUBROUTINE RWAXV2(N,M,A,B,X,Y,V,W)

*     Scalar Arguments
      INTEGER N,M

*     Array Arguments
      DOUBLE PRECISION A(*),B(*),X(*),Y(*),V(*),W(*)
      
*     Local Scalars
      INTEGER I,J,K
      DOUBLE PRECISION TMP1,TMP2

      K = 0
      DO 10 I = 1,M
         TMP1 = 0.0D+00
         TMP2 = 0.0D+00
         DO 20 J = 1,N
            TMP1 = TMP1 + A(K+J)*X(J)
            TMP2 = TMP2 + B(K+J)*Y(J)
 20      CONTINUE
         V(I) = TMP1
         W(I) = TMP2
         K = K + N
 10   CONTINUE

      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE MXDPGF *
*
*      
*     * Purpose *
*
*     Factorization A+E=L*D*trans(L) of a dense symmetric positive
*     definite matrix A+E, where D and E are diagonal positive definite
*     matrices and L is a lower triangular matrix. If A is sufficiently
*     positive definite then E=0.
*
*     
*     * CALLING SEQUENCE *
*     
*     CALL MXDPGF(N,A,INF,ALF,TAU)
*     
*     
*     * PARAMETERS *
*
*     II  N             Order of the matrix A.
*     RU  A(N*(N+1)/2)  On input a given dense symmetric (usually
*                       positive definite) matrix A stored in the packed
*                       form. On output the computed factorization
*                       A+E=L*D*trans(L).
*     IO  INF           An information obtained in the factorization
*                       process:
*                         INF=0  - A is sufficiently positive definite
*                                  and E=0. 
*                         INF<0  - A is not sufficiently positive
*                                  definite and E>0.
*                         INF>0  - A is indefinite and INF is an index
*                                  of the most negative diagonal element
*                                  used in the factorization process.
*     RU  ALF           On input a desired tolerance for positive
*                       definiteness. On output the most negative
*                       diagonal element used in the factorization
*                       process (if INF>0).
*     RO  TAU           Maximum diagonal element of the matrix E.
*
*      
*     * Method *
*      
*     P.E.Gill, W.Murray : Newton type methods for unconstrained and
*     linearly constrained optimization, Math. Programming 28 (1974)
*     pp. 311-350.
*
*      
*     * Original version *
*      
*     L.Luksan
*      
*      
      
      SUBROUTINE MXDPGF(N,A,INF,ALF,TAU)
      
*     Scalar Arguments
      INTEGER INF,N
      DOUBLE PRECISION ALF,TAU
      
*     Array Arguments
      DOUBLE PRECISION A(*)
      
*     Local Scalars
      INTEGER I,IJ,IK,J,K,KJ,KK,L
      DOUBLE PRECISION BET,DEL,GAM,RHO,SIG,TOL
      
*     Intrinsic Functions
      INTRINSIC ABS,MAX

      L = 0
      INF = 0
      TOL = ALF
      
*
*     Estimation of the matrix norm
*
      ALF = 0.0D0
      BET = 0.0D0
      GAM = 0.0D0
      TAU = 0.0D0
      KK = 0

      DO 20 K = 1,N
         KK = KK + K
         BET = MAX(BET,ABS(A(KK)))
         KJ = KK
         DO 10 J = K + 1,N
            KJ = KJ + J - 1
            GAM = MAX(GAM,ABS(A(KJ)))
 10      CONTINUE
 20   CONTINUE
      BET = MAX(TOL,BET,GAM/N)

      DEL = TOL*MAX(BET,1.0D0)
      KK = 0
      DO 60 K = 1,N
         KK = KK + K
         
*
*     Determination of a diagonal correction
*
         SIG = A(KK)
         IF (ALF.GT.SIG) THEN
            ALF = SIG
            L = K
         END IF

         GAM = 0.0D0
         KJ = KK
         DO 30 J = K + 1,N
            KJ = KJ + J - 1
            GAM = MAX(GAM,ABS(A(KJ)))
 30      CONTINUE
         GAM = GAM*GAM
         RHO = MAX(ABS(SIG),GAM/BET,DEL)
         IF (TAU.LT.RHO-SIG) THEN
            TAU = RHO - SIG
            INF = -1
         END IF
         
*
*     Gaussian elimination
*
         A(KK) = RHO
         KJ = KK
         DO 50 J = K + 1,N
            KJ = KJ + J - 1
            GAM = A(KJ)
            A(KJ) = GAM/RHO
            IK = KK
            IJ = KJ
            DO 40 I = K + 1,J
               IK = IK + I - 1
               IJ = IJ + 1
               A(IJ) = A(IJ) - A(IK)*GAM
 40         CONTINUE
 50      CONTINUE
 60   CONTINUE
      IF (L.GT.0 .AND. ABS(ALF).GT.DEL) INF = L
      
      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE CALQ *
*
*      
*     * Purpose *
*      
*     Solving X from linear equation A*X=Y.
*
*      
*     * Calling sequence *
*     
*     CALL CALQ(N,M,IOLD,A,X,Y,SMALL,IPRINT)
*
*      
*     * Parameters *
*
*     II  N               Order of matrix A.
*     RU  A(N*(N+1)/2)    On input: Dense symmetric matrix stored in
*                           the packed form. On output: factorization
*                           A+E=L*D*TRANS(L).
*     II  M               Length of vector Y, M >= N, note that only N
*                           components from vector Y are used
*     RI  Y(M)            Input vector stored in a circular order.
*     II  IOLD            Index, which controlls the circular order of
*                           the vector Y.
*     RO  X(M)            Output vector equal to the solution of A*X=Y.
*                           The vector X has the same circular order
*                           than Y. Note that X may be equal to Y in
*                           calling sequence.
*     RI  SMALL           Small positive value. 
*     II  IPRINT          Printout specification.
*
*     
*     * Local variables *
*
*     I   INF             
*     R   ETA
*     R   BET      
*     I   IERR            Error indicador: 
*                             0  - Everything is ok.
*                            -2  - Error in LINEQ.
*
*     
*     * Subprograms used *
*
*     S   MXDPGF          Gill-Murray decomposition of a dense symmetric
*                           matrix.
*     S   LINEQ           Solver for linear equation.
*
*     
*     Napsu Haarala (2003)
*

      
      SUBROUTINE CALQ(N,M,IOLD,A,X,Y,SMALL,IPRINT)

*     Scalar Arguments
      INTEGER N,M,IOLD,IPRINT
      DOUBLE PRECISION SMALL
      
*     Array Arguments
      DOUBLE PRECISION A(*),X(*),Y(*)

*     Local Variables
      INTEGER INF,IERR
      DOUBLE PRECISION ETA,BET
      
*     External Subroutines
      EXTERNAL MXDPGF,LINEQ

      ETA = SMALL
      
      CALL MXDPGF(N,A,INF,ETA,BET)

      IF (INF .LT. 0) THEN
         IF (IPRINT .EQ. 2) THEN
            WRITE (6,FMT='(1X,''Warning: Insufficiently positive''
     &           '' definite matrix detected. '')')
            WRITE (6,FMT='(1X,''Correction added.'')')
         END IF
         
      ELSE IF (INF .GT. 0) THEN
         IF (IPRINT .EQ. 2) WRITE (6,FMT='(1X,''Warning: Indefinite''
     &        '' matrix detected. ''
     &        ''Correction added.'')')
      END IF

      
      CALL LINEQ(N,M,IOLD,A,X,Y,IERR)
      IF (IERR .NE. 0) THEN
         IF (IPRINT .EQ. 2) THEN
            WRITE (6,FMT='(1X,''Warning: ''
     &           '' Indefinite matrix detected. '')')
         END IF
      END IF


      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE LINEQ *
*
*      
*     * Purpose *
*      
*     Solving X from linear equation A*X=Y. Positive definite matrix A+E
*     is given using the factorization A+E=L*D*L' obtained by the
*     subroutine MXDPGF.
*
*      
*     * Calling sequence *
*     
*     CALL LINEQ(N,M,IOLD,A,X,Y,IERR)
*
*      
*     * Parameters *
*
*     II  N               Order of matrix A.
*     RI  A(N*(N+1)/2)    Factorization A+E=L*D*L' obtained by the
*                           subroutine MXDPGF.
*     II  M               Length of vector Y, M >= N. Note that only N
*                           components from vector Y are used
*     RI  Y(M)            Input vector stored in a circular order.
*     II  IOLD            Index, which controlls the circular order of
*                           the vector Y.
*     RO  X(M)            Output vector equal to the solution of A*X=Y.
*                           The vector X has the same circular order
*                           than Y. Note that X may be equal to Y in
*                           calling sequence.
*     IO  IERR            Error indicador: 
*                             0  - Everything is ok.
*                            -2  - Error; 0 at diagonal.
*
*     
*     * Method *
*      
*     Forward and backward substitution
*
*      
*     Napsu Haarala (2003)
*

      
      SUBROUTINE LINEQ(N,M,IOLD,A,X,Y,IERR)

*     Scalar Arguments
      INTEGER N,M,IOLD,IERR

*     Array Arguments
      DOUBLE PRECISION A(*),X(*),Y(*)

*     Local Variables
      INTEGER I,II,IJ,J,K,L

      IERR = -2
      
*     
*     Phase 1: X=L**(-1)*X
*
      IJ = 0
      DO 10 I = 1,N
         L=I+IOLD-1
         IF (L .GT. M) L=L-M
         X(L) = Y(L)
c        II = II + I
         
         DO 20 J = 1,I - 1
            IJ = IJ + 1
            K=J+IOLD-1
            IF (K .GT. M) K=K-M
            X(L) = X(L) - A(IJ)*X(K)
 20      CONTINUE
         IJ = IJ + 1
 10   CONTINUE

*
*     Phase 2 : X:=D**(-1)*X
*
      II = 0
      DO 30 I = 1,N
         II = II + I
         IF (A(II) .EQ. 0.0D+00) GOTO 999
         L=I+IOLD-1
         IF (L .GT. M) L=L-M
         X(L) = X(L)/A(II)
 30   CONTINUE

*
*     Phase 3 : X:=TRANS(L)**(-1)*X
*
      II = N* (N-1)/2
      DO 50 I = N - 1,1,-1
         IJ = II
         L=I+IOLD-1
         IF (L .GT. M) L=L-M
         DO 40 J = I + 1,N
            K=J+IOLD-1
            IF (K .GT. M) K=K-M
            IJ = IJ + J - 1
            X(L) = X(L) - A(IJ)*X(K)
 40      CONTINUE
         II = II - I
 50   CONTINUE
c     Phase 2 and 3
c      II = N * (N+1)/2
c      DO 30 I = N,1,-1
c         L=I+IOLD-1
c         IF (L .GT. M) L=L-M
c         X(L) = X(L)/A(II)
c         IJ = II
c         DO 40 J = I + 1,N
c            K=J+IOLD-1
c            IF (K .GT. M) K=K-M
c            IJ = IJ + J - 1
c            X(L) = X(L) - A(IJ)*X(K)
c 40      CONTINUE
c         II = II - I
c 30   CONTINUE

      IERR = 0

 999  CONTINUE
      
      RETURN
      END

      
************************************************************************
*
*     * SUBROUTINE TRLIEQ *
*
*      
*     * Purpose *
*      
*     Solving X from linear equation U*X=Y or U'*X=Y, where U is
*     an upper triangular matrix.
*
*      
*     * Calling sequence *
*     
*     CALL TRLIEQ(N,M,IOLD,U,X,Y,JOB,SMALL,IERR)
*
*      
*     * Parameters *
*
*     II  N               Order of matrix U.
*     RI  U(N*(N+1)/2)    Triangular matrix.
*     II  M               Length of vector Y, M >= N. Note that only N
*                           components from vector Y are used
*     RI  Y(M)            Input vector stored in a circular order.
*     II  IOLD            Index, which controlls the circular order of
*                           the vector Y.
*     RO  X(M)            Output vector equal to the solution of U*X=Y
*                           or U'*X=Y. The vector X has the same
*                           circular order than Y. Note that X may be
*                           equal to Y in calling sequence.
*     II  JOB             Option:
*                             0  - X:=(U')**(-1)*Y, U upper
*                                  triangular.
*                             1  - X:=U**(-1)*Y, U upper triangular.
*     RI  SMALL           Small positive value.
*     IO  IERR            Error indicador: 
*                             0  - Everything is ok.
*                            -3  - Error; 0 at diagonal.
*     
*
*     
*     * Method *
*      
*     Forward and backward substitution
*
*      
*     Napsu Haarala (2003)
*

      
      SUBROUTINE TRLIEQ(N,M,IOLD,U,X,Y,JOB,IERR)

*     Scalar Arguments
      INTEGER N,M,IOLD,JOB,IERR

*     Array Arguments
      DOUBLE PRECISION U(*),X(*),Y(*)

*     Local Variables
      INTEGER I,II,IJ,J,K,L,JI

*     Intrinsic Functions
      INTRINSIC ABS
      

      IERR = -3
      
      DO I=1,M
         X(I)=Y(I)
      ENDDO
      
      IF (JOB .EQ. 0) THEN
*     
*     X=U'**(-1)*Y, U' = [u1         ] is lower triangular.
*                        [u2 u3      ]
*                        [u4 u5 u6   ]
*                        [.  .  .  . ]
*         
         II = 0
         DO 10 I = 1,N
            II=II+I
            L=I+IOLD-1
            IF (L .GT. M) L=L-M
            IF (U(II) .EQ. 0.0D+00) GO TO 999
            X(L) = X(L)/U(II)
            DO 20 J = I+1,N
               JI = (J-1)*J/2+I
               K=J+IOLD-1
               IF (K .GT. M) K=K-M
               X(K) = X(K) - U(JI)*X(L)
 20         CONTINUE
 10      CONTINUE

         
      ELSE IF (JOB .EQ. 1) THEN
*     
*     X=U**(-1)*Y, U = [u1 u2 u4 . ] is upper triangular.
*                      [   u3 u5 . ]
*                      [      u6 . ]
*                      [         . ]
*         

         II = N* (N+1)/2
         DO 50 I = N,1,-1
            L=I+IOLD-1
            IF (L .GT. M) L=L-M
            IF (U(II) .EQ. 0.0D+00) GO TO 999
            IJ = II
            DO 60 J = I + 1,N
               K=J+IOLD-1
               IF (K .GT. M) K=K-M
               IJ = IJ + J - 1
               X(L) = X(L) - U(IJ)*X(K)
 60         CONTINUE
            X(L)=X(L)/U(II)
            II = II - I
 50      CONTINUE
         
         
      ELSE
         
         GO TO 999
      END IF
      
      IERR = 0

 999  CONTINUE
      
      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE SCALEX *
*
*      
*     * Purpose *
*
*     Scaling a vector.
*
*     
*     * Calling sequence *
*     
*     CALL SCALEX(N,A,X,Y)
*     
*     
*     * Parameters *
*
*     II  N             Vector dimension.
*     RI  X(N)          Input vector.
*     RI  A             Scaling parameter.
*     RO  Y(N)          Output vector, where Y:= A*X.
*
*      
*     Napsu Haarala (2006)
*      

      SUBROUTINE SCALEX(N,A,X,Y)
      
*     Scalar Arguments
      INTEGER N
      DOUBLE PRECISION A
      
*     Array Arguments
      DOUBLE PRECISION X(*),Y(*)
      
*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Y(I) = A*X(I)
 10   CONTINUE

      RETURN
      END

      
************************************************************************
*      
*     * SUBROUTINE SCDIFF *
*
*      
*     * Purpose *
*
*     Difference of the scaled vector and a vector.
*
*     
*     * Calling sequence *
*     
*     CALL SCDIFF(N,A,X,Y,Z)
*     
*     
*     * Parameters *
*
*     II  N             Vector dimension.
*     RI  A             Scaling factor.
*     RI  X(N)          Input vector.
*     RI  Y(N)          Input vector.
*     RO  Z(N)          Output vector, where Z:= A*X - Y.
*
*      
*     Napsu Haarala (2004)
*      

      SUBROUTINE SCDIFF(N,A,X,Y,Z)
      
*     Scalar Arguments
      INTEGER N
      DOUBLE PRECISION A
      
*     Array Arguments
      DOUBLE PRECISION X(*),Y(*),Z(*)
      
*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Z(I) = A*X(I) - Y(I)
 10   CONTINUE

      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE SCSUM *
*
*      
*     * Purpose *
*
*     Sum of a vector and the scaled vector.
*
*     
*     * Calling sequence *
*     
*     CALL SCSUM(N,A,X,Y,Z)
*     
*     
*     * Parameters *
*
*     II  N             Vector dimension.
*     RI  A             Scaling factor.
*     RI  X(N)          Input vector.
*     RI  Y(N)          Input vector.
*     RO  Z(N)          Output vector, where Z:= Y + A*X.
*
*      
*     Napsu Haarala (2004)
*      

      SUBROUTINE SCSUM(N,A,X,Y,Z)
      
*     Scalar Arguments
      INTEGER N
      DOUBLE PRECISION A
      
*     Array Arguments
      DOUBLE PRECISION X(*),Y(*),Z(*)
      
*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Z(I) = Y(I) + A*X(I)
 10   CONTINUE

      RETURN
      END

      
************************************************************************
*      
*     * SUBROUTINE COPY *
*
*
*     * Purpose *
*
*     Copying of vector.
*
*     
*     * Calling sequence *
*     
*     CALL COPY(N,X,Y)
*     
*     
*     * Parameters *
*
*     II  N               Vectors dimension.
*     RI  X(N)            Input vector.
*     RO  Y(N)            Output vector where Y:= X.
*
*      
*     Napsu Haarala (2004)
*     
*      

      SUBROUTINE COPY(N,X,Y)
      
*     Scalar Arguments
      INTEGER N
      
*     Array Arguments
      DOUBLE PRECISION X(*),Y(*)

*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Y(I) = X(I)
 10   CONTINUE
      
      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE COPY2 *
*
*
*     * Purpose *
*
*     Copying of two vectors.
*
*     
*     * Calling sequence *
*     
*     CALL COPY2(N,X,Y,Z,V)
*     
*     
*     * Parameters *
*
*     II  N             Vectors dimension.
*     RI  X(N)          Input vector.
*     RO  Y(N)          Output vector where Y:= X.
*     RI  Z(N)          Input vector.
*     RO  V(N)          Output vector where V:= Z.
*
*      
*     Napsu Haarala (2004)
*     
*      

      SUBROUTINE COPY2(N,X,Y,Z,V)
      
*     Scalar Arguments
      INTEGER N
      
*     Array Arguments
      DOUBLE PRECISION X(*),Y(*),Z(*),V(*)

*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Y(I) = X(I)
         V(I) = Z(I)
 10   CONTINUE
      
      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE VNEG *
*
*      
*     * Purpose *
*
*     Change the signs of vector elements.
*
*     
*     * Calling sequence *
*     
*     CALL VNEG(N,X,Y)
*     
*     
*     * Parameters *
*
*     II  N               Vector dimension.
*     RI  X(N)            Input vector.
*     RO  Y(N)            Output vector, where Y:= -X.
*
*      
*     Napsu Haarala (2006)
*      

      SUBROUTINE VNEG(N,X,Y)
      
*     Scalar Arguments
      INTEGER N

*     Array Arguments
      DOUBLE PRECISION X(*),Y(*)

*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Y(I) = -X(I)
 10   CONTINUE

      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE VXDIAG *
*
*      
*     * Purpose *
*
*     Vector is multiplied by a diagonal matrix.
*
*     
*     * Calling sequence *
*     
*     CALL VXDIAG(N,D,X,Y)
*     
*     
*     * Parameters *
*
*     II  N               Vector dimension.
*     RI  D(N)            Diagonal matrix stored as a vector with N elements.
*     RI  X(N)            Input vector.
*     RO  Y(N)            Output vector where Y:=D*X.
*
*      
*     Napsu Haarala (2005)
*     

      SUBROUTINE VXDIAG(N,D,X,Y)
      
*     Scalar Arguments
      INTEGER N
      
*     Array Arguments
      DOUBLE PRECISION D(*),X(*),Y(*)
      
*     Local Scalars
      INTEGER I
      
      DO 10 I = 1,N
         Y(I) = X(I)*D(I)
 10   CONTINUE

      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE XDIFFY *
*
*      
*     * Purpose *
*
*     Difference of two vectors.
*
*     
*     * Calling sequence *
*     
*     CALL XDIFFY(N,X,Y,Z)
*     
*     
*     * Parameters *
*
*     II  N               Vector dimension.
*     RI  X(N)            Input vector.
*     RI  Y(N)            Input vector.
*     RO  Z(N)            Output vector, where Z:= X - Y.
*
*      
*     Napsu Haarala (2006)
*      

      SUBROUTINE XDIFFY(N,X,Y,Z)
      
*     Scalar Arguments
      INTEGER N
      
*     Array Arguments
      DOUBLE PRECISION X(*),Y(*),Z(*)
      
*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Z(I) = X(I) - Y(I)
 10   CONTINUE

      RETURN
      END


************************************************************************
*      
*     * SUBROUTINE XSUMY *
*
*      
*     * Purpose *
*
*     Sum of two vectors.
*
*     
*     * Calling sequence *
*     
*     CALL XSUMY(N,X,Y,Z)
*     
*     
*     * Parameters *
*
*     II  N               Vector dimension.
*     RI  X(N)            Input vector.
*     RI  Y(N)            Input vector.
*     RO  Z(N)            Output vector, where Z:= Y + X.
*
*      
*     Napsu Haarala (2006)
*      

      SUBROUTINE XSUMY(N,X,Y,Z)
      
*     Scalar Arguments
      INTEGER N
      
*     Array Arguments
      DOUBLE PRECISION X(*),Y(*),Z(*)
      
*     Local Scalars
      INTEGER I

      DO 10 I = 1,N
         Z(I) = Y(I) + X(I)
 10   CONTINUE

      RETURN
      END

      
************************************************************************
*      
*     * DOUBLE PRECISION FUNCTION VDOT *
*
*
*     * Purpose *
*
*     Dot product of two vectors.
*
*     
*     * Calling sequence *
*     
*     XTY = VDOT(N,X,Y)
*     
*     
*     * Parameters *
*
*     II  N               Vectors dimension.
*     RI  X(N)            Input vector.
*     RI  Y(N)            Input vector.
*     RO  VDOT            Value of dot product VDOT=trans(X)*Y.
*
*      
*     Napsu Haarala (2006)
*     
*
      
      DOUBLE PRECISION FUNCTION VDOT(N,X,Y)

*     Scalar Arguments
      INTEGER N

*     Array Arguments
      DOUBLE PRECISION X(*),Y(*)

*     Local Scalars
      INTEGER I
      DOUBLE PRECISION TMP

      TMP = 0.0D0

      DO 10 I = 1,N
         TMP = TMP + X(I)*Y(I)
 10   CONTINUE

      VDOT = TMP

      RETURN
      END

      
************************************************************************
*
*     * DOUBLE PRECISION FUNCTION EPS0 *
*
*      
*     * Purpose *
*      
*     Computation of the smallest positive number such that
*     1.0 + EPS0 > 1.0.
*      
*     
*     * Calling sequence *
*     
*     SMALL = EPS0()
*      
*     
*     Napsu Haarala (2003)
*      
*
      
      DOUBLE PRECISION FUNCTION EPS0()

*     Local scalars
      DOUBLE PRECISION EPSNEW
      
      EPS0=1.0D+00

 100  EPSNEW=EPS0/2.0D+00

      IF(1.0D+00+EPSNEW .EQ. 1.0D+00) RETURN

      EPS0=EPSNEW
      GO TO 100

      END

************************************************************************
      
