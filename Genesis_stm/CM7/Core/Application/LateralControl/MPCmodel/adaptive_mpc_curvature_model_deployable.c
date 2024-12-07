/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: adaptive_mpc_curvature_model_deployable.c
 *
 * Code generated for Simulink model 'adaptive_mpc_curvature_model_deployable'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Dec  6 17:21:10 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "adaptive_mpc_curvature_model_deployable.h"
#include "rtwtypes.h"
#include <string.h>
#include <math.h>

/* Named constants for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
#define Uscale                         (5.0)
#define Wdu                            (0.010000000000000002)
#define degrees                        (5)
#define p                              (30)
#define uoff                           (0.0)

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Continuous states */
X rtX;

/* Disabled State Vector */
XDis rtXDis;

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real_T rt_hypotd(real_T u0, real_T u1);
static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

/* private model entry point functions */
extern void adaptive_mpc_curvature_model_deployable_derivatives(void);

/* Forward declaration for local functions */
static int32_T xpotrf(real_T b_A[25]);
static void trisolve(const real_T b_A[25], real_T b_B[25]);
static real_T norm(const real_T x[5]);
static real_T xnrm2(int32_T n, const real_T x[25], int32_T ix0);
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[25], int32_T ia0,
                  const real_T x[25], int32_T ix0, real_T y[5]);
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[5], real_T b_A[25], int32_T ia0);
static real_T KWIKfactor(const real_T b_Ac[40], const int32_T iC[8], int32_T nA,
  const real_T b_Linv[25], real_T RLinv[25], real_T b_D[25], real_T b_H[25],
  int32_T n);
static real_T mtimes(const real_T b_A[5], const real_T b_B[5]);
static void DropConstraint(int32_T kDrop, boolean_T iA[8], int32_T *nA, int32_T
  iC[8]);
static void qpkwik(const real_T b_Linv[25], const real_T b_Hinv[25], const
                   real_T f[5], const real_T b_Ac[40], const real_T b[8],
                   boolean_T iA[8], int32_T maxiter, real_T FeasTol, real_T x[5],
                   real_T lambda[8], int32_T *status);
static void mpcblock_optimizer(const real_T rseq[60], const real_T vseq[31],
  const real_T x[6], real_T old_u, const boolean_T iA[8], const real_T b_Mlim[8],
  real_T b_Mx[48], real_T b_Mu1[8], real_T b_Mv[248], real_T b_uoff, real_T b_H
  [25], real_T b_Ac[40], const real_T b_Wy[2], const real_T b_Jm[120], const
  real_T b_A[36], const real_T Bu[186], const real_T Bv[186], const real_T b_C
  [12], const real_T Dv[62], const int32_T b_Mrows[8], real_T *u, real_T useq[31],
  real_T *status, boolean_T iAout[8]);
static int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 1;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  adaptive_mpc_curvature_model_deployable_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  adaptive_mpc_curvature_model_deployable_step();
  adaptive_mpc_curvature_model_deployable_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  adaptive_mpc_curvature_model_deployable_step();
  adaptive_mpc_curvature_model_deployable_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static int32_T xpotrf(real_T b_A[25])
{
  int32_T b_k;
  int32_T info;
  int32_T j;
  int32_T jm1;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 5)) {
    real_T c;
    real_T ssq;
    int32_T idxAjj;
    idxAjj = j * 5 + j;
    ssq = 0.0;
    if (j >= 1) {
      for (b_k = 0; b_k < j; b_k++) {
        c = b_A[b_k * 5 + j];
        ssq += c * c;
      }
    }

    ssq = b_A[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = sqrt(ssq);
      b_A[idxAjj] = ssq;
      if (j + 1 < 5) {
        if (j != 0) {
          int32_T b_iy;
          b_iy = ((j - 1) * 5 + j) + 2;
          for (b_k = j + 2; b_k <= b_iy; b_k += 5) {
            int32_T d;
            jm1 = b_k - j;
            c = -b_A[div_nde_s32_floor(jm1 - 2, 5) * 5 + j];
            d = jm1 + 3;
            for (jm1 = b_k; jm1 <= d; jm1++) {
              int32_T tmp;
              tmp = ((idxAjj + jm1) - b_k) + 1;
              b_A[tmp] += b_A[jm1 - 1] * c;
            }
          }
        }

        ssq = 1.0 / ssq;
        jm1 = (idxAjj - j) + 5;
        for (b_k = idxAjj + 2; b_k <= jm1; b_k++) {
          b_A[b_k - 1] *= ssq;
        }
      }

      j++;
    } else {
      b_A[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }

  return info;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void trisolve(const real_T b_A[25], real_T b_B[25])
{
  int32_T b_k;
  int32_T i;
  int32_T j;
  for (j = 0; j < 5; j++) {
    int32_T jBcol;
    jBcol = 5 * j;
    for (b_k = 0; b_k < 5; b_k++) {
      real_T b_B_0;
      int32_T b_B_tmp;
      int32_T kAcol;
      kAcol = 5 * b_k;
      b_B_tmp = b_k + jBcol;
      b_B_0 = b_B[b_B_tmp];
      if (b_B_0 != 0.0) {
        b_B[b_B_tmp] = b_B_0 / b_A[b_k + kAcol];
        for (i = b_k + 2; i < 6; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          b_B[tmp] -= b_A[(i + kAcol) - 1] * b_B[b_B_tmp];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real_T norm(const real_T x[5])
{
  real_T scale;
  real_T y;
  int32_T k;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (k = 0; k < 5; k++) {
    real_T absxk;
    absxk = fabs(x[k]);
    if (absxk > scale) {
      real_T t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      real_T t;
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real_T xnrm2(int32_T n, const real_T x[25], int32_T ix0)
{
  real_T y;
  int32_T k;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        real_T absxk;
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void xgemv(int32_T b_m, int32_T n, const real_T b_A[25], int32_T ia0,
                  const real_T x[25], int32_T ix0, real_T y[5])
{
  int32_T b_iy;
  int32_T ia;
  if ((b_m != 0) && (n != 0)) {
    int32_T b;
    if (n - 1 >= 0) {
      memset(&y[0], 0, (uint32_T)n * sizeof(real_T));
    }

    b = (n - 1) * 5 + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 5) {
      real_T c;
      int32_T d;
      c = 0.0;
      d = (b_iy + b_m) - 1;
      for (ia = b_iy; ia <= d; ia++) {
        c += x[((ix0 + ia) - b_iy) - 1] * b_A[ia - 1];
      }

      ia = div_nde_s32_floor(b_iy - ia0, 5);
      y[ia] += c;
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void xgerc(int32_T b_m, int32_T n, real_T alpha1, int32_T ix0, const
                  real_T y[5], real_T b_A[25], int32_T ia0)
{
  int32_T ijA;
  int32_T j;
  if (alpha1 != 0.0) {
    int32_T jA;
    jA = ia0;
    for (j = 0; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        temp *= alpha1;
        b = (b_m + jA) - 1;
        for (ijA = jA; ijA <= b; ijA++) {
          b_A[ijA - 1] += b_A[((ix0 + ijA) - jA) - 1] * temp;
        }
      }

      jA += 5;
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real_T KWIKfactor(const real_T b_Ac[40], const int32_T iC[8], int32_T nA,
  const real_T b_Linv[25], real_T RLinv[25], real_T b_D[25], real_T b_H[25],
  int32_T n)
{
  real_T R[25];
  real_T TL[25];
  real_T b_A[25];
  real_T tau[5];
  real_T work[5];
  real_T RLinv_0;
  real_T Status;
  real_T b_A_0;
  real_T beta1;
  int32_T b_coltop;
  int32_T b_lastv;
  int32_T coltop;
  int32_T exitg1;
  int32_T ii;
  int32_T k_i;
  int32_T knt;
  boolean_T exitg2;
  Status = 1.0;
  memset(&RLinv[0], 0, 25U * sizeof(real_T));
  for (k_i = 0; k_i < nA; k_i++) {
    b_lastv = iC[k_i];
    for (b_coltop = 0; b_coltop < 5; b_coltop++) {
      RLinv_0 = 0.0;
      for (knt = 0; knt < 5; knt++) {
        RLinv_0 += b_Ac[((knt << 3) + b_lastv) - 1] * b_Linv[5 * knt + b_coltop];
      }

      RLinv[b_coltop + 5 * k_i] = RLinv_0;
    }
  }

  memcpy(&b_A[0], &RLinv[0], 25U * sizeof(real_T));
  for (k_i = 0; k_i < 5; k_i++) {
    tau[k_i] = 0.0;
    work[k_i] = 0.0;
  }

  for (k_i = 0; k_i < 5; k_i++) {
    ii = k_i * 5 + k_i;
    if (k_i + 1 < 5) {
      RLinv_0 = b_A[ii];
      b_lastv = ii + 2;
      tau[k_i] = 0.0;
      beta1 = xnrm2(4 - k_i, b_A, ii + 2);
      if (beta1 != 0.0) {
        b_A_0 = b_A[ii];
        beta1 = rt_hypotd(b_A_0, beta1);
        if (b_A_0 >= 0.0) {
          beta1 = -beta1;
        }

        if (fabs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          coltop = (ii - k_i) + 5;
          do {
            knt++;
            for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
              b_A[b_coltop - 1] *= 9.9792015476736E+291;
            }

            beta1 *= 9.9792015476736E+291;
            RLinv_0 *= 9.9792015476736E+291;
          } while ((fabs(beta1) < 1.0020841800044864E-292) && (knt < 20));

          beta1 = rt_hypotd(RLinv_0, xnrm2(4 - k_i, b_A, ii + 2));
          if (RLinv_0 >= 0.0) {
            beta1 = -beta1;
          }

          tau[k_i] = (beta1 - RLinv_0) / beta1;
          RLinv_0 = 1.0 / (RLinv_0 - beta1);
          for (b_coltop = b_lastv; b_coltop <= coltop; b_coltop++) {
            b_A[b_coltop - 1] *= RLinv_0;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            beta1 *= 1.0020841800044864E-292;
          }

          RLinv_0 = beta1;
        } else {
          tau[k_i] = (beta1 - b_A_0) / beta1;
          RLinv_0 = 1.0 / (b_A_0 - beta1);
          b_coltop = (ii - k_i) + 5;
          for (knt = b_lastv; knt <= b_coltop; knt++) {
            b_A[knt - 1] *= RLinv_0;
          }

          RLinv_0 = beta1;
        }
      }

      b_A[ii] = 1.0;
      if (tau[k_i] != 0.0) {
        b_lastv = 5 - k_i;
        knt = (ii - k_i) + 4;
        while ((b_lastv > 0) && (b_A[knt] == 0.0)) {
          b_lastv--;
          knt--;
        }

        knt = 4 - k_i;
        exitg2 = false;
        while ((!exitg2) && (knt > 0)) {
          b_coltop = ((knt - 1) * 5 + ii) + 5;
          coltop = b_coltop;
          do {
            exitg1 = 0;
            if (coltop + 1 <= b_coltop + b_lastv) {
              if (b_A[coltop] != 0.0) {
                exitg1 = 1;
              } else {
                coltop++;
              }
            } else {
              knt--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        knt = 0;
      }

      if (b_lastv > 0) {
        xgemv(b_lastv, knt, b_A, ii + 6, b_A, ii + 1, work);
        xgerc(b_lastv, knt, -tau[k_i], ii + 1, work, b_A, ii + 6);
      }

      b_A[ii] = RLinv_0;
    } else {
      tau[4] = 0.0;
    }
  }

  for (k_i = 0; k_i < 5; k_i++) {
    for (ii = 0; ii <= k_i; ii++) {
      R[ii + 5 * k_i] = b_A[5 * k_i + ii];
    }

    for (ii = k_i + 2; ii < 6; ii++) {
      R[(ii + 5 * k_i) - 1] = 0.0;
    }

    work[k_i] = 0.0;
  }

  for (k_i = 4; k_i >= 0; k_i--) {
    b_lastv = (k_i * 5 + k_i) + 6;
    if (k_i + 1 < 5) {
      b_A[b_lastv - 6] = 1.0;
      if (tau[k_i] != 0.0) {
        knt = 5 - k_i;
        b_coltop = b_lastv - k_i;
        while ((knt > 0) && (b_A[b_coltop - 2] == 0.0)) {
          knt--;
          b_coltop--;
        }

        b_coltop = 4 - k_i;
        exitg2 = false;
        while ((!exitg2) && (b_coltop > 0)) {
          coltop = (b_coltop - 1) * 5 + b_lastv;
          ii = coltop;
          do {
            exitg1 = 0;
            if (ii <= (coltop + knt) - 1) {
              if (b_A[ii - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ii++;
              }
            } else {
              b_coltop--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        knt = 0;
        b_coltop = 0;
      }

      if (knt > 0) {
        xgemv(knt, b_coltop, b_A, b_lastv, b_A, b_lastv - 5, work);
        xgerc(knt, b_coltop, -tau[k_i], b_lastv - 5, work, b_A, b_lastv);
      }

      b_coltop = b_lastv - k_i;
      for (knt = b_lastv - 4; knt < b_coltop; knt++) {
        b_A[knt - 1] *= -tau[k_i];
      }
    }

    b_A[b_lastv - 6] = 1.0 - tau[k_i];
    for (knt = 0; knt < k_i; knt++) {
      b_A[(b_lastv - knt) - 7] = 0.0;
    }
  }

  k_i = 0;
  do {
    exitg1 = 0;
    if (k_i <= nA - 1) {
      if (fabs(R[5 * k_i + k_i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        k_i++;
      }
    } else {
      for (k_i = 0; k_i < n; k_i++) {
        for (ii = 0; ii < n; ii++) {
          RLinv_0 = 0.0;
          for (b_coltop = 0; b_coltop < 5; b_coltop++) {
            RLinv_0 += b_Linv[5 * k_i + b_coltop] * b_A[5 * ii + b_coltop];
          }

          TL[k_i + 5 * ii] = RLinv_0;
        }
      }

      memset(&RLinv[0], 0, 25U * sizeof(real_T));
      for (k_i = nA; k_i >= 1; k_i--) {
        b_coltop = (k_i - 1) * 5;
        knt = (k_i + b_coltop) - 1;
        RLinv[knt] = 1.0;
        for (ii = k_i; ii <= nA; ii++) {
          coltop = ((ii - 1) * 5 + k_i) - 1;
          RLinv[coltop] /= R[knt];
        }

        if (k_i > 1) {
          for (ii = 0; ii <= k_i - 2; ii++) {
            for (b_lastv = k_i; b_lastv <= nA; b_lastv++) {
              knt = (b_lastv - 1) * 5;
              coltop = knt + ii;
              RLinv[coltop] -= RLinv[(knt + k_i) - 1] * R[b_coltop + ii];
            }
          }
        }
      }

      for (k_i = 0; k_i < n; k_i++) {
        for (ii = k_i + 1; ii <= n; ii++) {
          b_coltop = (ii - 1) * 5 + k_i;
          b_H[b_coltop] = 0.0;
          for (b_lastv = nA + 1; b_lastv <= n; b_lastv++) {
            knt = (b_lastv - 1) * 5;
            b_H[b_coltop] -= TL[(knt + ii) - 1] * TL[knt + k_i];
          }

          b_H[(ii + 5 * k_i) - 1] = b_H[b_coltop];
        }
      }

      for (k_i = 0; k_i < nA; k_i++) {
        for (ii = 0; ii < n; ii++) {
          b_coltop = 5 * k_i + ii;
          b_D[b_coltop] = 0.0;
          for (b_lastv = k_i + 1; b_lastv <= nA; b_lastv++) {
            knt = (b_lastv - 1) * 5;
            b_D[b_coltop] += TL[knt + ii] * RLinv[knt + k_i];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static real_T mtimes(const real_T b_A[5], const real_T b_B[5])
{
  real_T b_C;
  int32_T k;
  b_C = 0.0;
  for (k = 0; k < 5; k++) {
    b_C += b_A[k] * b_B[k];
  }

  return b_C;
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void DropConstraint(int32_T kDrop, boolean_T iA[8], int32_T *nA, int32_T
  iC[8])
{
  int32_T i;
  if (kDrop > 0) {
    iA[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      for (i = kDrop; i < *nA; i++) {
        iC[i - 1] = iC[i];
      }
    }

    iC[*nA - 1] = 0;
    (*nA)--;
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void qpkwik(const real_T b_Linv[25], const real_T b_Hinv[25], const
                   real_T f[5], const real_T b_Ac[40], const real_T b[8],
                   boolean_T iA[8], int32_T maxiter, real_T FeasTol, real_T x[5],
                   real_T lambda[8], int32_T *status)
{
  real_T RLinv[25];
  real_T U[25];
  real_T b_D[25];
  real_T b_H[25];
  real_T Opt[10];
  real_T Rhs[10];
  real_T cTol[8];
  real_T b_Ac_0[5];
  real_T r[5];
  real_T z[5];
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T rMin;
  real_T t;
  int32_T iC[8];
  int32_T b_exponent;
  int32_T exitg1;
  int32_T exitg3;
  int32_T exponent;
  int32_T i;
  int32_T iC_0;
  int32_T iSave;
  int32_T kDrop;
  int32_T nA;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  for (i = 0; i < 5; i++) {
    x[i] = 0.0;
  }

  memset(&lambda[0], 0, sizeof(real_T) << 3U);
  *status = 1;
  for (i = 0; i < 5; i++) {
    r[i] = 0.0;
  }

  rMin = 0.0;
  cTolComputed = false;
  for (i = 0; i < 8; i++) {
    cTol[i] = 1.0;
    iC[i] = 0;
  }

  nA = 0;
  for (i = 0; i < 8; i++) {
    if (iA[i]) {
      nA++;
      iC[nA - 1] = i + 1;
    }
  }

  guard1 = false;
  if (nA > 0) {
    memset(&Opt[0], 0, 10U * sizeof(real_T));
    for (i = 0; i < 5; i++) {
      Rhs[i] = f[i];
      Rhs[i + 5] = 0.0;
    }

    DualFeasible = false;
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (nA > 0) && (*status <= maxiter)) {
        Xnorm0 = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, b_D, b_H, degrees);
        if (Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            nA = 0;
            for (i = 0; i < 8; i++) {
              iA[i] = false;
              iC[i] = 0;
            }

            ColdReset = true;
          }
        } else {
          for (i = 0; i < nA; i++) {
            Rhs[i + 5] = b[iC[i] - 1];
            for (kDrop = i + 1; kDrop <= nA; kDrop++) {
              iC_0 = (5 * i + kDrop) - 1;
              U[iC_0] = 0.0;
              for (iSave = 0; iSave < nA; iSave++) {
                U[iC_0] += RLinv[(5 * iSave + kDrop) - 1] * RLinv[5 * iSave + i];
              }

              U[i + 5 * (kDrop - 1)] = U[iC_0];
            }
          }

          for (i = 0; i < 5; i++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 5; iC_0++) {
              Xnorm0 += b_H[5 * iC_0 + i] * Rhs[iC_0];
            }

            Opt[i] = Xnorm0;
            for (kDrop = 0; kDrop < nA; kDrop++) {
              Opt[i] += b_D[5 * kDrop + i] * Rhs[kDrop + 5];
            }
          }

          for (i = 0; i < nA; i++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 5; iC_0++) {
              Xnorm0 += b_D[5 * i + iC_0] * Rhs[iC_0];
            }

            Opt[i + 5] = Xnorm0;
            for (kDrop = 0; kDrop < nA; kDrop++) {
              Opt[i + 5] += U[5 * kDrop + i] * Rhs[kDrop + 5];
            }
          }

          Xnorm0 = -1.0E-12;
          kDrop = -1;
          for (i = 0; i < nA; i++) {
            cMin = Opt[i + 5];
            lambda[iC[i] - 1] = cMin;
            if ((cMin < Xnorm0) && (i + 1 <= nA)) {
              kDrop = i;
              Xnorm0 = cMin;
            }
          }

          if (kDrop + 1 <= 0) {
            DualFeasible = true;
            for (i = 0; i < 5; i++) {
              x[i] = Opt[i];
            }
          } else {
            (*status)++;
            if (*status > 5) {
              nA = 0;
              for (i = 0; i < 8; i++) {
                iA[i] = false;
                iC[i] = 0;
              }

              ColdReset = true;
            } else {
              lambda[iC[kDrop] - 1] = 0.0;
              DropConstraint(kDrop + 1, iA, &nA, iC);
            }
          }
        }
      } else {
        if (nA <= 0) {
          memset(&lambda[0], 0, sizeof(real_T) << 3U);
          for (i = 0; i < 5; i++) {
            Xnorm0 = 0.0;
            for (iC_0 = 0; iC_0 < 5; iC_0++) {
              Xnorm0 += -b_Hinv[5 * iC_0 + i] * f[iC_0];
            }

            x[i] = Xnorm0;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    for (i = 0; i < 5; i++) {
      Xnorm0 = 0.0;
      for (iC_0 = 0; iC_0 < 5; iC_0++) {
        Xnorm0 += -b_Hinv[5 * iC_0 + i] * f[iC_0];
      }

      x[i] = Xnorm0;
    }

    guard1 = true;
  }

  if (guard1) {
    Xnorm0 = norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= maxiter)) {
      cMin = -FeasTol;
      i = -1;
      for (kDrop = 0; kDrop < 8; kDrop++) {
        if (!cTolComputed) {
          for (iSave = 0; iSave < 5; iSave++) {
            z[iSave] = fabs(b_Ac[(iSave << 3) + kDrop] * x[iSave]);
          }

          cVal = z[0];
          if (z[0] < z[1]) {
            cVal = z[1];
          }

          if (cVal < z[2]) {
            cVal = z[2];
          }

          if (cVal < z[3]) {
            cVal = z[3];
          }

          if (cVal < z[4]) {
            cVal = z[4];
          }

          cTol[kDrop] = fmax(cTol[kDrop], cVal);
        }

        if (!iA[kDrop]) {
          t = 0.0;
          for (iC_0 = 0; iC_0 < 5; iC_0++) {
            t += b_Ac[(iC_0 << 3) + kDrop] * x[iC_0];
          }

          cVal = (t - b[kDrop]) / cTol[kDrop];
          if (cVal < cMin) {
            cMin = cVal;
            i = kDrop;
          }
        }
      }

      cTolComputed = true;
      if (i + 1 <= 0) {
        exitg2 = true;
      } else if (*status == maxiter) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((i + 1 > 0) && (*status <= maxiter)) {
            guard2 = false;
            if (nA == 0) {
              for (iC_0 = 0; iC_0 < 5; iC_0++) {
                cMin = 0.0;
                for (kDrop = 0; kDrop < 5; kDrop++) {
                  cMin += b_Hinv[5 * kDrop + iC_0] * b_Ac[(kDrop << 3) + i];
                }

                z[iC_0] = cMin;
              }

              guard2 = true;
            } else {
              cMin = KWIKfactor(b_Ac, iC, nA, b_Linv, RLinv, b_D, b_H, degrees);
              if (cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (iC_0 = 0; iC_0 < 25; iC_0++) {
                  U[iC_0] = -b_H[iC_0];
                }

                for (iC_0 = 0; iC_0 < 5; iC_0++) {
                  cMin = 0.0;
                  for (kDrop = 0; kDrop < 5; kDrop++) {
                    cMin += U[5 * kDrop + iC_0] * b_Ac[(kDrop << 3) + i];
                  }

                  z[iC_0] = cMin;
                }

                for (kDrop = 0; kDrop < nA; kDrop++) {
                  t = 0.0;
                  for (iC_0 = 0; iC_0 < 5; iC_0++) {
                    t += b_Ac[(iC_0 << 3) + i] * b_D[5 * kDrop + iC_0];
                  }

                  r[kDrop] = t;
                }

                guard2 = true;
              }
            }

            if (guard2) {
              kDrop = 0;
              cMin = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (nA > 0) {
                iSave = 0;
                exitg4 = false;
                while ((!exitg4) && (iSave <= nA - 1)) {
                  if (r[iSave] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    iSave++;
                  }
                }
              }

              if ((nA != 0) && (!ColdReset)) {
                for (iSave = 0; iSave < nA; iSave++) {
                  cVal = r[iSave];
                  if (cVal > 1.0E-12) {
                    cVal = lambda[iC[iSave] - 1] / cVal;
                    if ((kDrop == 0) || (cVal < rMin)) {
                      rMin = cVal;
                      kDrop = iSave + 1;
                    }
                  }
                }

                if (kDrop > 0) {
                  cMin = rMin;
                  DualFeasible = false;
                }
              }

              for (iC_0 = 0; iC_0 < 5; iC_0++) {
                b_Ac_0[iC_0] = b_Ac[(iC_0 << 3) + i];
              }

              cVal = mtimes(z, b_Ac_0);
              if (cVal <= 0.0) {
                cVal = 0.0;
                ColdReset = true;
              } else {
                t = 0.0;
                for (iC_0 = 0; iC_0 < 5; iC_0++) {
                  t += b_Ac[(iC_0 << 3) + i] * x[iC_0];
                }

                cVal = (b[i] - t) / cVal;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  t = cMin;
                } else if (DualFeasible) {
                  t = cVal;
                } else {
                  t = fmin(cMin, cVal);
                }

                for (iSave = 0; iSave < nA; iSave++) {
                  iC_0 = iC[iSave];
                  lambda[iC_0 - 1] -= t * r[iSave];
                  if ((iC_0 <= 8) && (lambda[iC_0 - 1] < 0.0)) {
                    lambda[iC_0 - 1] = 0.0;
                  }
                }

                lambda[i] += t;
                frexp(1.0, &exponent);
                if (fabs(t - cMin) < 2.2204460492503131E-16) {
                  DropConstraint(kDrop, iA, &nA, iC);
                }

                if (!ColdReset) {
                  for (iC_0 = 0; iC_0 < 5; iC_0++) {
                    x[iC_0] += t * z[iC_0];
                  }

                  frexp(1.0, &b_exponent);
                  if (fabs(t - cVal) < 2.2204460492503131E-16) {
                    if (nA == degrees) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      nA++;
                      iC[nA - 1] = i + 1;
                      kDrop = nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (kDrop + 1 > 1)) {
                        iC_0 = iC[kDrop - 1];
                        if (iC[kDrop] > iC_0) {
                          exitg4 = true;
                        } else {
                          iSave = iC[kDrop];
                          iC[kDrop] = iC_0;
                          iC[kDrop - 1] = iSave;
                          kDrop--;
                        }
                      }

                      iA[i] = true;
                      i = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            cMin = norm(x);
            if (fabs(cMin - Xnorm0) > 0.001) {
              Xnorm0 = cMin;
              for (i = 0; i < 8; i++) {
                cTol[i] = fmax(fabs(b[i]), 1.0);
              }

              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S32>/FixedHorizonOptimizer' */
static void mpcblock_optimizer(const real_T rseq[60], const real_T vseq[31],
  const real_T x[6], real_T old_u, const boolean_T iA[8], const real_T b_Mlim[8],
  real_T b_Mx[48], real_T b_Mu1[8], real_T b_Mv[248], real_T b_uoff, real_T b_H
  [25], real_T b_Ac[40], const real_T b_Wy[2], const real_T b_Jm[120], const
  real_T b_A[36], const real_T Bu[186], const real_T Bv[186], const real_T b_C
  [12], const real_T Dv[62], const int32_T b_Mrows[8], real_T *u, real_T useq[31],
  real_T *status, boolean_T iAout[8])
{
  real_T c_Sx[360];
  real_T WySuJm[240];
  real_T c_SuJm[240];
  real_T c_Kv[124];
  real_T WduJm[120];
  real_T CA_0[62];
  real_T Sum_0[60];
  real_T c_Su1[60];
  real_T L[25];
  real_T c_Kx[24];
  real_T CA[12];
  real_T CA_1[12];
  real_T b_Mlim_0[8];
  real_T b_Mlim_1[8];
  real_T b_Mv_0[8];
  real_T varargin_1[5];
  real_T zopt[5];
  real_T c_Ku1[4];
  real_T Sum[2];
  real_T b_C_0[2];
  real_T WySuJm_0;
  real_T normH;
  real_T s;
  int32_T CA_tmp;
  int32_T i;
  int32_T i1;
  int32_T kidx;
  int16_T ixw;
  int8_T b[25];
  int8_T rows[2];
  int8_T kidx_0;
  int8_T rows_0;
  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  memset(&useq[0], 0, 31U * sizeof(real_T));
  for (i = 0; i < 8; i++) {
    iAout[i] = false;
  }

  for (i1 = 0; i1 < 2; i1++) {
    Sum[i1] = 0.0;
    b_C_0[i1] = 0.0;
    for (i = 0; i < 6; i++) {
      normH = 0.0;
      for (CA_tmp = 0; CA_tmp < 6; CA_tmp++) {
        normH += b_C[(CA_tmp << 1) + i1] * b_A[6 * i + CA_tmp];
      }

      CA_tmp = (i << 1) + i1;
      CA[CA_tmp] = normH;
      normH = b_C[CA_tmp];
      Sum[i1] += normH * Bu[i];
      b_C_0[i1] += normH * Bv[i];
    }

    rtDW.c_Hv[i1] = b_C_0[i1];
    rtDW.c_Hv[i1 + 60] = Dv[i1];
  }

  for (i1 = 0; i1 < 29; i1++) {
    i = (i1 + 2) * 60;
    rtDW.c_Hv[i] = 0.0;
    rtDW.c_Hv[i + 1] = 0.0;
  }

  for (i1 = 0; i1 < 31; i1++) {
    memset(&rtDW.c_Hv[i1 * 60 + 2], 0, 58U * sizeof(real_T));
  }

  for (i1 = 0; i1 < 6; i1++) {
    i = i1 << 1;
    c_Sx[60 * i1] = CA[i];
    c_Sx[60 * i1 + 1] = CA[i + 1];
    memset(&c_Sx[i1 * 60 + 2], 0, 58U * sizeof(real_T));
  }

  c_Su1[0] = Sum[0];
  c_Su1[1] = Sum[1];
  memset(&c_Su1[2], 0, 58U * sizeof(real_T));
  rtDW.Su[0] = Sum[0];
  rtDW.Su[1] = Sum[1];
  for (i1 = 0; i1 < 29; i1++) {
    i = (i1 + 1) * 60;
    rtDW.Su[i] = 0.0;
    rtDW.Su[i + 1] = 0.0;
  }

  for (i1 = 0; i1 < 30; i1++) {
    memset(&rtDW.Su[i1 * 60 + 2], 0, 58U * sizeof(real_T));
  }

  for (kidx = 0; kidx < 29; kidx++) {
    kidx_0 = (int8_T)(((kidx + 1) << 1) + 1);
    for (i1 = 0; i1 < 2; i1++) {
      rows_0 = (int8_T)(i1 + kidx_0);
      rows[i1] = rows_0;
      normH = 0.0;
      for (i = 0; i < 6; i++) {
        normH += CA[(i << 1) + i1] * Bu[i];
      }

      normH += Sum[i1];
      Sum[i1] = normH;
      c_Su1[rows_0 - 1] = normH;
      Sum_0[i1] = normH;
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 29; i1++) {
      i = (i1 + 1) << 1;
      Sum_0[i] = rtDW.Su[(60 * i1 + rows_0) - 3];
      Sum_0[i + 1] = rtDW.Su[(60 * i1 + kidx_0) - 3];
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 30; i1++) {
      i = i1 << 1;
      rtDW.Su[(rows_0 + 60 * i1) - 1] = Sum_0[i];
      rtDW.Su[(kidx_0 + 60 * i1) - 1] = Sum_0[i + 1];
    }

    for (i1 = 0; i1 < 2; i1++) {
      normH = 0.0;
      for (i = 0; i < 6; i++) {
        normH += CA[(i << 1) + i1] * Bv[i];
      }

      CA_0[i1] = normH;
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 30; i1++) {
      CA_tmp = (i1 + 1) << 1;
      CA_0[CA_tmp] = rtDW.c_Hv[(60 * i1 + rows_0) - 3];
      CA_0[CA_tmp + 1] = rtDW.c_Hv[(60 * i1 + kidx_0) - 3];
    }

    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 31; i1++) {
      i = i1 << 1;
      rtDW.c_Hv[(rows_0 + 60 * i1) - 1] = CA_0[i];
      rtDW.c_Hv[(kidx_0 + 60 * i1) - 1] = CA_0[i + 1];
    }

    for (i1 = 0; i1 < 2; i1++) {
      for (i = 0; i < 6; i++) {
        normH = 0.0;
        for (CA_tmp = 0; CA_tmp < 6; CA_tmp++) {
          normH += CA[(CA_tmp << 1) + i1] * b_A[6 * i + CA_tmp];
        }

        CA_1[i1 + (i << 1)] = normH;
      }
    }

    memcpy(&CA[0], &CA_1[0], 12U * sizeof(real_T));
    rows_0 = rows[0];
    kidx_0 = rows[1];
    for (i1 = 0; i1 < 6; i1++) {
      i = i1 << 1;
      c_Sx[(rows_0 + 60 * i1) - 1] = CA[i];
      c_Sx[(kidx_0 + 60 * i1) - 1] = CA[i + 1];
    }
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i = 0; i < 60; i++) {
      normH = 0.0;
      for (CA_tmp = 0; CA_tmp < 30; CA_tmp++) {
        normH += rtDW.Su[60 * CA_tmp + i] * b_Jm[30 * i1 + CA_tmp];
      }

      c_SuJm[i + 60 * i1] = normH;
    }
  }

  if (b_Mrows[0] > 0) {
    kidx = 0;
    exitg1 = false;
    while ((!exitg1) && (kidx < 8)) {
      if (b_Mrows[kidx] <= 60) {
        i = b_Mrows[kidx];
        b_Ac[kidx] = -c_SuJm[i - 1];
        b_Ac[kidx + 8] = -c_SuJm[i + 59];
        b_Ac[kidx + 16] = -c_SuJm[i + 119];
        b_Ac[kidx + 24] = -c_SuJm[i + 179];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 6; i1++) {
          b_Mx[kidx + (i1 << 3)] = -c_Sx[(60 * i1 + i) - 1];
        }

        b_Mu1[kidx] = -c_Su1[b_Mrows[kidx] - 1];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 31; i1++) {
          b_Mv[kidx + (i1 << 3)] = -rtDW.c_Hv[(60 * i1 + i) - 1];
        }

        kidx++;
      } else if (b_Mrows[kidx] <= 120) {
        i = b_Mrows[kidx];
        b_Ac[kidx] = c_SuJm[i - 61];
        b_Ac[kidx + 8] = c_SuJm[i - 1];
        b_Ac[kidx + 16] = c_SuJm[i + 59];
        b_Ac[kidx + 24] = c_SuJm[i + 119];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 6; i1++) {
          b_Mx[kidx + (i1 << 3)] = c_Sx[(60 * i1 + i) - 61];
        }

        b_Mu1[kidx] = c_Su1[b_Mrows[kidx] - 61];
        i = b_Mrows[kidx];
        for (i1 = 0; i1 < 31; i1++) {
          b_Mv[kidx + (i1 << 3)] = rtDW.c_Hv[(60 * i1 + i) - 61];
        }

        kidx++;
      } else {
        exitg1 = true;
      }
    }
  }

  ixw = 1;
  for (kidx = 0; kidx < 60; kidx++) {
    normH = b_Wy[ixw - 1];
    WySuJm[kidx] = normH * c_SuJm[kidx];
    WySuJm[kidx + 60] = c_SuJm[kidx + 60] * normH;
    WySuJm[kidx + 120] = c_SuJm[kidx + 120] * normH;
    WySuJm[kidx + 180] = c_SuJm[kidx + 180] * normH;
    ixw++;
    if (ixw > 2) {
      ixw = 1;
    }
  }

  for (i1 = 0; i1 < 120; i1++) {
    WduJm[i1] = b_Jm[i1] * Wdu;
  }

  for (i1 = 0; i1 < 4; i1++) {
    for (i = 0; i < 4; i++) {
      normH = 0.0;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        normH += c_SuJm[60 * i1 + CA_tmp] * WySuJm[60 * i + CA_tmp];
      }

      s = 0.0;
      for (CA_tmp = 0; CA_tmp < 30; CA_tmp++) {
        s += b_Jm[30 * i1 + CA_tmp] * WduJm[30 * i + CA_tmp];
      }

      b_H[i1 + 5 * i] = normH + s;
    }

    normH = 0.0;
    for (i = 0; i < 60; i++) {
      normH += WySuJm[60 * i1 + i] * c_Su1[i];
    }

    c_Ku1[i1] = normH;
  }

  for (i1 = 0; i1 < 6; i1++) {
    for (i = 0; i < 4; i++) {
      normH = 0.0;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        normH += c_Sx[60 * i1 + CA_tmp] * WySuJm[60 * i + CA_tmp];
      }

      c_Kx[i1 + 6 * i] = normH;
    }
  }

  for (i1 = 0; i1 < 31; i1++) {
    for (i = 0; i < 4; i++) {
      s = 0.0;
      for (CA_tmp = 0; CA_tmp < 60; CA_tmp++) {
        s += rtDW.c_Hv[60 * i1 + CA_tmp] * WySuJm[60 * i + CA_tmp];
      }

      c_Kv[i1 + 31 * i] = s;
    }
  }

  for (i1 = 0; i1 < 240; i1++) {
    WySuJm[i1] = -WySuJm[i1];
  }

  kidx = 0;
  memcpy(&L[0], &b_H[0], 25U * sizeof(real_T));
  i = xpotrf(L);
  guard1 = false;
  if (i == 0) {
    for (i = 0; i < 5; i++) {
      varargin_1[i] = L[5 * i + i];
    }

    normH = varargin_1[0];
    if (varargin_1[0] > varargin_1[1]) {
      normH = varargin_1[1];
    }

    if (normH > varargin_1[2]) {
      normH = varargin_1[2];
    }

    if (normH > varargin_1[3]) {
      normH = varargin_1[3];
    }

    if (normH > varargin_1[4]) {
      normH = varargin_1[4];
    }

    if (normH > 1.4901161193847656E-7) {
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }

  if (guard1) {
    normH = 0.0;
    for (i = 0; i < 5; i++) {
      s = 0.0;
      for (i1 = 0; i1 < 5; i1++) {
        s += fabs(b_H[5 * i1 + i]);
      }

      if (s > normH) {
        normH = s;
      }
    }

    if (normH >= 1.0E+10) {
      kidx = 2;
    } else {
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i <= 4)) {
        normH = pow(10.0, i) * 1.4901161193847656E-7;
        for (i1 = 0; i1 < 25; i1++) {
          b[i1] = 0;
        }

        for (kidx = 0; kidx < 5; kidx++) {
          b[kidx + 5 * kidx] = 1;
        }

        for (i1 = 0; i1 < 25; i1++) {
          s = normH * (real_T)b[i1] + b_H[i1];
          b_H[i1] = s;
          L[i1] = s;
        }

        kidx = xpotrf(L);
        guard2 = false;
        if (kidx == 0) {
          for (kidx = 0; kidx < 5; kidx++) {
            varargin_1[kidx] = L[5 * kidx + kidx];
          }

          normH = varargin_1[0];
          if (varargin_1[0] > varargin_1[1]) {
            normH = varargin_1[1];
          }

          if (normH > varargin_1[2]) {
            normH = varargin_1[2];
          }

          if (normH > varargin_1[3]) {
            normH = varargin_1[3];
          }

          if (normH > varargin_1[4]) {
            normH = varargin_1[4];
          }

          if (normH > 1.4901161193847656E-7) {
            kidx = 1;
            exitg1 = true;
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }

        if (guard2) {
          kidx = 3;
          i++;
        }
      }
    }
  }

  if (kidx > 1) {
    *u = old_u + b_uoff;
    for (i = 0; i < 31; i++) {
      useq[i] = *u;
    }

    *status = -2.0;
  } else {
    for (i1 = 0; i1 < 25; i1++) {
      b[i1] = 0;
    }

    for (kidx = 0; kidx < 5; kidx++) {
      b[kidx + 5 * kidx] = 1;
    }

    for (kidx = 0; kidx < 5; kidx++) {
      for (i = 0; i < 5; i++) {
        i1 = 5 * kidx + i;
        b_H[i1] = b[i1];
      }

      varargin_1[kidx] = 0.0;
    }

    trisolve(L, b_H);
    for (kidx = 0; kidx < 4; kidx++) {
      normH = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        normH += c_Kx[6 * kidx + i1] * x[i1];
      }

      WySuJm_0 = 0.0;
      for (i1 = 0; i1 < 60; i1++) {
        WySuJm_0 += WySuJm[60 * kidx + i1] * rseq[i1];
      }

      s = 0.0;
      for (i1 = 0; i1 < 31; i1++) {
        s += c_Kv[31 * kidx + i1] * vseq[i1];
      }

      varargin_1[kidx] = ((normH + WySuJm_0) + c_Ku1[kidx] * old_u) + s;
    }

    for (i = 0; i < 8; i++) {
      iAout[i] = iA[i];
      normH = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        normH += b_Mx[(i1 << 3) + i] * x[i1];
      }

      b_Mlim_0[i] = (b_Mlim[i] + normH) + b_Mu1[i] * old_u;
      normH = 0.0;
      for (i1 = 0; i1 < 31; i1++) {
        normH += b_Mv[(i1 << 3) + i] * vseq[i1];
      }

      b_Mv_0[i] = normH;
    }

    for (i1 = 0; i1 < 5; i1++) {
      for (i = 0; i < 5; i++) {
        s = 0.0;
        for (CA_tmp = 0; CA_tmp < 5; CA_tmp++) {
          s += b_H[5 * i1 + CA_tmp] * b_H[5 * i + CA_tmp];
        }

        L[i1 + 5 * i] = s;
      }
    }

    for (i1 = 0; i1 < 8; i1++) {
      b_Mlim_1[i1] = -(b_Mlim_0[i1] + b_Mv_0[i1]);
    }

    qpkwik(b_H, L, varargin_1, b_Ac, b_Mlim_1, iAout, 120, 1.0E-6, zopt,
           b_Mlim_0, &kidx);
    if ((kidx < 0) || (kidx == 0)) {
      for (i = 0; i < 5; i++) {
        zopt[i] = 0.0;
      }
    }

    *status = kidx;
    *u = (old_u + zopt[0]) + b_uoff;
  }
}

/* Model step function */
void adaptive_mpc_curvature_model_deployable_step(void)
{
  /* local block i/o variables */
  real_T rtb_xk[4];
  real_T rtb_xk1[4];
  real_T rtb_ySum[2];
  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  {
    real_T Cm[372];
    real_T tmp_2[248];
    real_T Bu[186];
    real_T Bv[186];
    real_T CovMat[64];
    real_T Dv[62];
    real_T rseq[60];
    real_T tmp_1[48];
    real_T b_tmp[40];
    real_T L_tmp[36];
    real_T L_tmp_0[36];
    real_T b_A[36];
    real_T b_B[36];
    real_T rtb_useq[31];
    real_T vseq[31];
    real_T rtb_A_h[16];
    real_T Cm_0[12];
    real_T L[12];
    real_T b_C[12];
    real_T c_A_tmp[12];
    real_T tmp_0[12];
    real_T b_Mlim[8];
    real_T b_xoff[6];
    real_T xk[6];
    real_T xk_0[6];
    real_T Kinv[4];
    real_T rtb_X[4];
    real_T Y[2];
    real_T rtb_TmpSignalConversionAtSFunct[2];
    real_T Memory1_PreviousInput;
    real_T Memory1_PreviousInput_0;
    real_T Memory1_PreviousInput_1;
    real_T U;
    real_T a22;
    real_T rtb_U;
    int32_T CovMat_tmp;
    int32_T r1;
    int32_T r2;
    int32_T rseq_tmp;
    int8_T rtb_C_l[8];
    int8_T UnknownIn[5];
    int8_T b_b[4];
    boolean_T tmp;
    static const int8_T tmp_3[8] = { 0, 0, 1, 0, 0, 1, 0, 0 };

    static const int8_T c[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0,
      0, 0, 0, 0, 10, 5, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

    static const real_T d[36] = { 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    static const real_T e[12] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.2, 0.0, 0.0, 1.0,
      0.0, 0.0, 1.0 };

    static const int8_T f[8] = { 1, 5, 1, 5, 1, 5, 1, 5 };

    static const int8_T b_D[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1 };

    static const int32_T b_Mrows[8] = { 121, 122, 123, 124, 151, 152, 153, 154 };

    static const int8_T a[4] = { 0, 0, 0, 1 };

    static const real_T b_RYscale[2] = { 1.0, 0.2 };

    static const real_T o[8] = { -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0 };

    static const real_T q[25] = { 1.6144290100000002E+6, 1.5648280000000002E+6,
      1.5072270000000002E+6, 1.4496260000000002E+6, 0.0, 1.5648280000000002E+6,
      1.5568280100000002E+6, 1.5072270000000002E+6, 1.4496260000000002E+6, 0.0,
      1.5072270000000002E+6, 1.5072270000000002E+6, 1.4992270100000002E+6,
      1.4496260000000002E+6, 0.0, 1.4496260000000002E+6, 1.4496260000000002E+6,
      1.4496260000000002E+6, 1.4416260100000002E+6, 0.0, 0.0, 0.0, 0.0, 0.0,
      100000.0 };

    static const real_T s[40] = { -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0,
      -0.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, -0.0, -0.0, -1.0, -1.0, 0.0,
      0.0, 1.0, 1.0, -0.0, -0.0, -0.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0 };

    static const real_T t[2] = { 0.64000000000000012, 0.040000000000000008 };

    static const real_T v[120] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    real_T q_0[25];
    real_T o_0[8];
    tmp = rtmIsMajorTimeStep(rtM);
    if (tmp) {
      /* Memory: '<Root>/Memory' */
      rtb_U = rtDW.Memory_PreviousInput_o;

      /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
       *  Constant: '<Root>/L'
       *  Inport: '<Root>/Velocity'
       *  Memory: '<S3>/Memory1'
       */
      for (rseq_tmp = 0; rseq_tmp < 8; rseq_tmp++) {
        rtb_C_l[rseq_tmp] = tmp_3[rseq_tmp];
      }

      rtb_A_h[0] = 0.0;
      rtb_A_h[4] = 0.0;
      rtb_A_h[8] = -rtU.Velocity;
      rtb_A_h[12] = 0.0;
      rtb_A_h[1] = 0.0;
      rtb_A_h[5] = 0.0;
      rtb_A_h[9] = rtU.Velocity;
      rtb_A_h[13] = rtU.Velocity;
      rtb_A_h[2] = 0.0;
      rtb_A_h[6] = 0.0;
      rtb_A_h[10] = 0.0;
      rtb_A_h[14] = rtU.Velocity / 0.2;
      rtb_A_h[3] = 0.0;
      rtb_X[0] = rtDW.Memory1_PreviousInput[0];

      /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
       *  MATLAB Function: '<Root>/MATLAB Function'
       */
      b_b[0] = 0;

      /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
       *  Memory: '<S3>/Memory1'
       */
      rtb_A_h[7] = 0.0;
      rtb_X[1] = rtDW.Memory1_PreviousInput[1];

      /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
       *  MATLAB Function: '<Root>/MATLAB Function'
       */
      b_b[1] = 0;

      /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
       *  Memory: '<S3>/Memory1'
       */
      rtb_A_h[11] = 0.0;
      rtb_X[2] = rtDW.Memory1_PreviousInput[2];

      /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
       *  MATLAB Function: '<Root>/MATLAB Function'
       */
      b_b[2] = 0;

      /* MATLAB Function: '<Root>/MATLAB Function' incorporates:
       *  Memory: '<S3>/Memory1'
       */
      rtb_A_h[15] = 0.0;
      rtb_X[3] = rtDW.Memory1_PreviousInput[3];

      /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
       *  MATLAB Function: '<Root>/MATLAB Function'
       */
      b_b[3] = 1;
      memset(&Bu[0], 0, 186U * sizeof(real_T));
      memset(&Bv[0], 0, 186U * sizeof(real_T));
      memset(&Dv[0], 0, 62U * sizeof(real_T));
      memset(&Cm[0], 0, 372U * sizeof(real_T));
      for (rseq_tmp = 0; rseq_tmp < 36; rseq_tmp++) {
        b_A[rseq_tmp] = c[rseq_tmp];
        b_B[rseq_tmp] = d[rseq_tmp];
      }

      memcpy(&b_C[0], &e[0], 12U * sizeof(real_T));
      for (rseq_tmp = 0; rseq_tmp < 4; rseq_tmp++) {
        r2 = rseq_tmp << 1;
        b_C[r2] = (real_T)rtb_C_l[r2] / (real_T)f[r2];
        b_C[r2 + 1] = (real_T)rtb_C_l[r2 + 1] / (real_T)f[r2 + 1];
        r2 = rseq_tmp << 2;
        b_A[6 * rseq_tmp] = rtb_A_h[r2];
        b_A[6 * rseq_tmp + 1] = rtb_A_h[r2 + 1];
        b_A[6 * rseq_tmp + 2] = rtb_A_h[r2 + 2];
        b_A[6 * rseq_tmp + 3] = rtb_A_h[r2 + 3];
        b_B[rseq_tmp] = (real_T)b_b[rseq_tmp] * Uscale;
      }

      for (rseq_tmp = 0; rseq_tmp < 6; rseq_tmp++) {
        Bu[rseq_tmp] = b_B[rseq_tmp];
        Bv[rseq_tmp] = b_B[rseq_tmp + 6];
        r2 = rseq_tmp << 1;
        Cm[r2] = b_C[r2];
        Cm[r2 + 1] = b_C[r2 + 1];
      }

      UnknownIn[0] = 1;
      UnknownIn[1] = 3;
      UnknownIn[2] = 4;
      UnknownIn[3] = 5;
      UnknownIn[4] = 6;
      for (rseq_tmp = 0; rseq_tmp < 5; rseq_tmp++) {
        for (r2 = 0; r2 < 6; r2++) {
          b_tmp[r2 + (rseq_tmp << 3)] = b_B[(UnknownIn[rseq_tmp] - 1) * 6 + r2];
        }

        r2 = (UnknownIn[rseq_tmp] - 1) << 1;
        r1 = rseq_tmp << 3;
        b_tmp[r1 + 6] = b_D[r2];
        b_tmp[r1 + 7] = b_D[r2 + 1];
      }

      Dv[0] = 0.0;
      Dv[1] = 0.0;
      for (r1 = 0; r1 < 8; r1++) {
        for (rseq_tmp = 0; rseq_tmp < 8; rseq_tmp++) {
          U = 0.0;
          for (r2 = 0; r2 < 5; r2++) {
            CovMat_tmp = r2 << 3;
            U += b_tmp[CovMat_tmp + r1] * b_tmp[CovMat_tmp + rseq_tmp];
          }

          CovMat[r1 + (rseq_tmp << 3)] = U;
        }

        b_Mlim[r1] = 0.028000000000000004;
      }

      for (r1 = 0; r1 < 6; r1++) {
        b_xoff[r1] = 0.0;
      }

      U = rtb_U / Uscale;

      /* Memory: '<S3>/Memory1' incorporates:
       *  MATLAB Function: '<Root>/MATLAB Function'
       */
      Memory1_PreviousInput = rtDW.Memory1_PreviousInput[1];
      a22 = rtDW.Memory1_PreviousInput[0];
      Memory1_PreviousInput_0 = rtDW.Memory1_PreviousInput[2];
      Memory1_PreviousInput_1 = rtDW.Memory1_PreviousInput[3];

      /* MATLAB Function: '<S32>/FixedHorizonOptimizer' incorporates:
       *  Inport: '<Root>/Inport'
       *  Inport: '<Root>/Inport1'
       *  MATLAB Function: '<Root>/MATLAB Function'
       *  Memory: '<S3>/Memory'
       *  Memory: '<S3>/Memory1'
       *  Memory: '<S4>/LastPcov'
       *  Memory: '<S4>/Memory'
       *  Memory: '<S4>/last_x'
       *  UnitDelay: '<S4>/last_mv'
       */
      for (rseq_tmp = 0; rseq_tmp < 2; rseq_tmp++) {
        Y[rseq_tmp] = ((((real_T)rtb_C_l[rseq_tmp + 2] * Memory1_PreviousInput +
                         (real_T)rtb_C_l[rseq_tmp] * a22) + (real_T)
                        rtb_C_l[rseq_tmp + 4] * Memory1_PreviousInput_0) +
                       (real_T)rtb_C_l[rseq_tmp + 6] * Memory1_PreviousInput_1) /
          (4.0 * (real_T)rseq_tmp + 1.0);
      }

      for (r1 = 0; r1 < 8; r1++) {
        if (b_Mrows[r1] <= 150) {
          b_Mlim[r1] += uoff - U;
        } else {
          b_Mlim[r1] -= uoff - U;
        }
      }

      for (rseq_tmp = 0; rseq_tmp < 4; rseq_tmp++) {
        b_xoff[rseq_tmp] = rtb_X[rseq_tmp];
        Bv[rseq_tmp] = ((((rtb_A_h[rseq_tmp + 4] * rtDW.Memory1_PreviousInput[1]
                           + rtb_A_h[rseq_tmp] * rtDW.Memory1_PreviousInput[0])
                          + rtb_A_h[rseq_tmp + 8] * rtDW.Memory1_PreviousInput[2])
                         + rtb_A_h[rseq_tmp + 12] * rtDW.Memory1_PreviousInput[3])
                        + (real_T)a[rseq_tmp] * rtb_U) -
          rtDW.Memory1_PreviousInput[rseq_tmp];
      }

      for (r1 = 0; r1 < 31; r1++) {
        vseq[r1] = 1.0;
      }

      rtb_U = Y[0];
      Memory1_PreviousInput = Y[1];
      for (r1 = 0; r1 < 30; r1++) {
        rseq_tmp = r1 << 1;
        rseq[rseq_tmp] = rtU.lateral_deviation - rtb_U;
        rseq[rseq_tmp + 1] = rtU.relative_yaw_angle * 0.2 -
          Memory1_PreviousInput;
      }

      b_b[0] = 0;
      b_b[1] = 0;
      b_b[2] = 0;
      b_b[3] = 0;
      for (r1 = 0; r1 < 2; r1++) {
        b_b[r1 + (r1 << 1)] = 1;
        for (rseq_tmp = 0; rseq_tmp < 6; rseq_tmp++) {
          CovMat_tmp = (rseq_tmp << 1) + r1;
          c_A_tmp[rseq_tmp + 6 * r1] = Cm[CovMat_tmp];
          rtb_U = 0.0;
          for (r2 = 0; r2 < 6; r2++) {
            rtb_U += Cm[(r2 << 1) + r1] * rtDW.LastPcov_PreviousInput[6 *
              rseq_tmp + r2];
          }

          Cm_0[CovMat_tmp] = rtb_U;
        }
      }

      for (rseq_tmp = 0; rseq_tmp < 2; rseq_tmp++) {
        for (r2 = 0; r2 < 2; r2++) {
          rtb_U = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            rtb_U += Cm_0[(r1 << 1) + rseq_tmp] * c_A_tmp[6 * r2 + r1];
          }

          rtb_X[rseq_tmp + (r2 << 1)] = CovMat[(((r2 + 6) << 3) + rseq_tmp) + 6]
            + rtb_U;
        }
      }

      if (fabs(rtb_X[1]) > fabs(rtb_X[0])) {
        r1 = 1;
        r2 = 0;
      } else {
        r1 = 0;
        r2 = 1;
      }

      rtb_U = rtb_X[r2] / rtb_X[r1];
      Memory1_PreviousInput = rtb_X[r1 + 2];
      a22 = rtb_X[r2 + 2] - Memory1_PreviousInput * rtb_U;
      rseq_tmp = r1 << 1;
      Kinv[rseq_tmp] = (real_T)b_b[0] / rtb_X[r1];
      r2 <<= 1;
      Kinv[r2] = ((real_T)b_b[2] - Kinv[rseq_tmp] * Memory1_PreviousInput) / a22;
      Kinv[rseq_tmp] -= Kinv[r2] * rtb_U;
      Kinv[rseq_tmp + 1] = (real_T)b_b[1] / rtb_X[r1];
      Kinv[r2 + 1] = ((real_T)b_b[3] - Kinv[rseq_tmp + 1] *
                      Memory1_PreviousInput) / a22;
      Kinv[rseq_tmp + 1] -= Kinv[r2 + 1] * rtb_U;
      for (rseq_tmp = 0; rseq_tmp < 6; rseq_tmp++) {
        for (r2 = 0; r2 < 6; r2++) {
          rtb_U = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            rtb_U += b_A[6 * r1 + rseq_tmp] * rtDW.LastPcov_PreviousInput[6 * r2
              + r1];
          }

          b_B[rseq_tmp + 6 * r2] = rtb_U;
        }

        for (r2 = 0; r2 < 2; r2++) {
          rtb_U = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            rtb_U += b_B[6 * r1 + rseq_tmp] * c_A_tmp[6 * r2 + r1];
          }

          Cm_0[rseq_tmp + 6 * r2] = CovMat[((r2 + 6) << 3) + rseq_tmp] + rtb_U;
        }

        rtb_U = Cm_0[rseq_tmp + 6];
        Memory1_PreviousInput = Cm_0[rseq_tmp];
        L[rseq_tmp] = rtb_U * Kinv[1] + Memory1_PreviousInput * Kinv[0];
        L[rseq_tmp + 6] = rtb_U * Kinv[3] + Memory1_PreviousInput * Kinv[2];
        xk[rseq_tmp] = rtDW.last_x_PreviousInput[rseq_tmp] - b_xoff[rseq_tmp];
      }

      for (rseq_tmp = 0; rseq_tmp < 2; rseq_tmp++) {
        rtb_U = 0.0;
        for (r2 = 0; r2 < 6; r2++) {
          rtb_U += Cm[(r2 << 1) + rseq_tmp] * xk[r2];
        }

        rtb_TmpSignalConversionAtSFunct[rseq_tmp] =
          (rtDW.Memory_PreviousInput[rseq_tmp] * b_RYscale[rseq_tmp] -
           Y[rseq_tmp]) - rtb_U;
      }

      for (rseq_tmp = 0; rseq_tmp < 6; rseq_tmp++) {
        for (r2 = 0; r2 < 2; r2++) {
          rtb_U = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            rtb_U += rtDW.LastPcov_PreviousInput[6 * r1 + rseq_tmp] * c_A_tmp[6 *
              r2 + r1];
          }

          tmp_0[rseq_tmp + 6 * r2] = rtb_U;
        }

        rtb_U = tmp_0[rseq_tmp + 6];
        Memory1_PreviousInput = tmp_0[rseq_tmp];
        xk_0[rseq_tmp] = ((rtb_U * Kinv[1] + Memory1_PreviousInput * Kinv[0]) *
                          rtb_TmpSignalConversionAtSFunct[0] + (rtb_U * Kinv[3]
          + Memory1_PreviousInput * Kinv[2]) * rtb_TmpSignalConversionAtSFunct[1])
          + xk[rseq_tmp];
      }

      memset(&tmp_1[0], 0, 48U * sizeof(real_T));
      memset(&tmp_2[0], 0, 248U * sizeof(real_T));
      memcpy(&o_0[0], &o[0], sizeof(real_T) << 3);
      memcpy(&q_0[0], &q[0], 25U * sizeof(real_T));
      memcpy(&b_tmp[0], &s[0], 40U * sizeof(real_T));
      mpcblock_optimizer(rseq, vseq, xk_0, rtDW.last_mv_DSTATE - U,
                         rtDW.Memory_PreviousInput_l, b_Mlim, tmp_1, o_0, tmp_2,
                         U, q_0, b_tmp, t, v, b_A, Bu, Bv, b_C, Dv, b_Mrows,
                         &rtb_U, rtb_useq, &a22, rtDW.iAout);
      rtDW.u = rtb_U;
      U = rtb_U - U;
      for (rseq_tmp = 0; rseq_tmp < 6; rseq_tmp++) {
        for (r2 = 0; r2 < 6; r2++) {
          rtb_U = 0.0;
          for (r1 = 0; r1 < 6; r1++) {
            rtb_U += b_B[6 * r1 + rseq_tmp] * b_A[6 * r1 + r2];
          }

          r1 = 6 * r2 + rseq_tmp;
          L_tmp[r1] = rtb_U;
          L_tmp_0[r1] = Cm_0[rseq_tmp + 6] * L[r2 + 6] + Cm_0[rseq_tmp] * L[r2];
        }
      }

      for (rseq_tmp = 0; rseq_tmp < 6; rseq_tmp++) {
        for (r2 = 0; r2 < 6; r2++) {
          r1 = 6 * rseq_tmp + r2;
          b_B[r1] = CovMat[(rseq_tmp << 3) + r2] + (L_tmp[r1] - L_tmp_0[r1]);
        }
      }

      for (rseq_tmp = 0; rseq_tmp < 6; rseq_tmp++) {
        rtb_U = 0.0;
        for (r2 = 0; r2 < 6; r2++) {
          r1 = 6 * rseq_tmp + r2;
          CovMat_tmp = 6 * r2 + rseq_tmp;
          rtDW.Pk1[r1] = (b_B[r1] + b_B[CovMat_tmp]) * 0.5;
          rtb_U += b_A[CovMat_tmp] * xk[r2];
        }

        rtDW.xk1[rseq_tmp] = (((Bu[rseq_tmp] * U + rtb_U) + Bv[rseq_tmp]) +
                              (L[rseq_tmp + 6] *
          rtb_TmpSignalConversionAtSFunct[1] + L[rseq_tmp] *
          rtb_TmpSignalConversionAtSFunct[0])) + b_xoff[rseq_tmp];
      }

      /* Gain: '<S4>/u_scale' */
      rtDW.u_scale = 5.0 * rtDW.u;

      /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
       *  Constant: '<Root>/L'
       *  Inport: '<Root>/Velocity'
       */
      rtb_A_h[0] = 0.0;
      rtb_A_h[4] = 0.0;
      rtb_A_h[8] = -rtU.Velocity;
      rtb_A_h[12] = 0.0;
      rtb_A_h[1] = 0.0;
      rtb_A_h[5] = 0.0;
      rtb_A_h[9] = rtU.Velocity;
      rtb_A_h[13] = rtU.Velocity;
      rtb_A_h[2] = 0.0;
      rtb_A_h[6] = 0.0;
      rtb_A_h[10] = 0.0;
      rtb_A_h[14] = rtU.Velocity / 0.2;

      /* Delay: '<S34>/State' */
      rtb_xk[0] = rtDW.State_DSTATE[0];

      /* MATLAB Function: '<S3>/MATLAB Function' */
      rtb_A_h[3] = 0.0;

      /* Delay: '<S34>/State' */
      rtb_xk[1] = rtDW.State_DSTATE[1];

      /* MATLAB Function: '<S3>/MATLAB Function' */
      rtb_A_h[7] = 0.0;

      /* Delay: '<S34>/State' */
      rtb_xk[2] = rtDW.State_DSTATE[2];

      /* MATLAB Function: '<S3>/MATLAB Function' */
      rtb_A_h[11] = 0.0;

      /* Delay: '<S34>/State' */
      rtb_xk[3] = rtDW.State_DSTATE[3];

      /* MATLAB Function: '<S3>/MATLAB Function' */
      rtb_A_h[15] = 0.0;

      /* Product: '<S34>/ProductA' incorporates:
       *  Delay: '<S34>/State'
       */
      U = rtb_xk[1];
      rtb_U = rtb_xk[0];
      Memory1_PreviousInput = rtb_xk[2];
      a22 = rtb_xk[3];
      for (rseq_tmp = 0; rseq_tmp < 4; rseq_tmp++) {
        /* Sum: '<S34>/dxSum' incorporates:
         *  Delay: '<S34>/State'
         *  MATLAB Function: '<S3>/MATLAB Function'
         *  Product: '<S34>/ProductA'
         *  Product: '<S34>/ProductB'
         */
        rtb_xk1[rseq_tmp] = (((rtb_A_h[rseq_tmp + 4] * U + rtb_A_h[rseq_tmp] *
          rtb_U) + rtb_A_h[rseq_tmp + 8] * Memory1_PreviousInput) +
                             rtb_A_h[rseq_tmp + 12] * a22) + (real_T)a[rseq_tmp]
          * rtDW.u_scale;
      }

      /* Product: '<S34>/ProductC' incorporates:
       *  Delay: '<S34>/State'
       *  MATLAB Function: '<S3>/MATLAB Function'
       */
      U = rtb_xk[1];
      rtb_U = rtb_xk[2];
      for (rseq_tmp = 0; rseq_tmp < 2; rseq_tmp++) {
        /* Sum: '<S34>/ySum' incorporates:
         *  Delay: '<S34>/State'
         *  MATLAB Function: '<S3>/MATLAB Function'
         *  Product: '<S34>/ProductC'
         */
        rtb_ySum[rseq_tmp] = (real_T)tmp_3[rseq_tmp + 2] * U + (real_T)
          tmp_3[rseq_tmp + 4] * rtb_U;
      }
    }

    /* Integrator: '<Root>/Integrator Limited' */
    /* Limited  Integrator  */
    if (rtX.IntegratorLimited_CSTATE >= 0.52) {
      rtX.IntegratorLimited_CSTATE = 0.52;
    } else if (rtX.IntegratorLimited_CSTATE <= -0.52) {
      rtX.IntegratorLimited_CSTATE = -0.52;
    }

    if (tmp) {
      /* Outport: '<Root>/Steering angle' incorporates:
       *  Integrator: '<Root>/Integrator Limited'
       */
      rtY.Steeringangle = rtX.IntegratorLimited_CSTATE;
    }
  }

  if (rtmIsMajorTimeStep(rtM)) {
    int32_T i;
    if (rtmIsMajorTimeStep(rtM)) {
      /* Update for Memory: '<S4>/LastPcov' */
      memcpy(&rtDW.LastPcov_PreviousInput[0], &rtDW.Pk1[0], 36U * sizeof(real_T));

      /* Update for Memory: '<S4>/Memory' */
      for (i = 0; i < 8; i++) {
        rtDW.Memory_PreviousInput_l[i] = rtDW.iAout[i];
      }

      /* End of Update for Memory: '<S4>/Memory' */

      /* Update for UnitDelay: '<S4>/last_mv' */
      rtDW.last_mv_DSTATE = rtDW.u;

      /* Update for Memory: '<S4>/last_x' */
      for (i = 0; i < 6; i++) {
        rtDW.last_x_PreviousInput[i] = rtDW.xk1[i];
      }

      /* End of Update for Memory: '<S4>/last_x' */

      /* Update for Memory: '<S3>/Memory' */
      rtDW.Memory_PreviousInput[0] = rtb_ySum[0];
      rtDW.Memory_PreviousInput[1] = rtb_ySum[1];

      /* Update for Memory: '<Root>/Memory' */
      rtDW.Memory_PreviousInput_o = rtDW.u_scale;

      /* Update for Memory: '<S3>/Memory1' */
      rtDW.Memory1_PreviousInput[0] = rtb_xk[0];

      /* Update for Delay: '<S34>/State' incorporates:
       *  Sum: '<S34>/dxSum'
       */
      rtDW.State_DSTATE[0] = rtb_xk1[0];

      /* Update for Memory: '<S3>/Memory1' */
      rtDW.Memory1_PreviousInput[1] = rtb_xk[1];

      /* Update for Delay: '<S34>/State' incorporates:
       *  Sum: '<S34>/dxSum'
       */
      rtDW.State_DSTATE[1] = rtb_xk1[1];

      /* Update for Memory: '<S3>/Memory1' */
      rtDW.Memory1_PreviousInput[2] = rtb_xk[2];

      /* Update for Delay: '<S34>/State' incorporates:
       *  Sum: '<S34>/dxSum'
       */
      rtDW.State_DSTATE[2] = rtb_xk1[2];

      /* Update for Memory: '<S3>/Memory1' */
      rtDW.Memory1_PreviousInput[3] = rtb_xk[3];

      /* Update for Delay: '<S34>/State' incorporates:
       *  Sum: '<S34>/dxSum'
       */
      rtDW.State_DSTATE[3] = rtb_xk1[3];
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.05s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.05, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void adaptive_mpc_curvature_model_deployable_derivatives(void)
{
  XDot *_rtXdot;
  boolean_T lsat;
  boolean_T usat;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<Root>/Integrator Limited' */
  lsat = (rtX.IntegratorLimited_CSTATE <= -0.52);
  usat = (rtX.IntegratorLimited_CSTATE >= 0.52);
  if (((!lsat) && (!usat)) || (lsat && (rtDW.u_scale > 0.0)) || (usat &&
       (rtDW.u_scale < 0.0))) {
    _rtXdot->IntegratorLimited_CSTATE = rtDW.u_scale;
  } else {
    /* in saturation */
    _rtXdot->IntegratorLimited_CSTATE = 0.0;
  }

  /* End of Derivatives for Integrator: '<Root>/Integrator Limited' */
}

/* Model initialize function */
void adaptive_mpc_curvature_model_deployable_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&rtM->solverInfo, (boolean_T**)
      &rtM->contStateDisabled);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetIsContModeFrozen(&rtM->solverInfo, false);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->contStates = ((X *) &rtX);
  rtM->contStateDisabled = ((XDis *) &rtXDis);
  rtM->Timing.tStart = (0.0);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetSolverName(&rtM->solverInfo,"ode3");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.05;

  /* InitializeConditions for Memory: '<S4>/LastPcov' */
  memcpy(&rtDW.LastPcov_PreviousInput[0], &rtConstP.LastPcov_InitialCondition[0],
         36U * sizeof(real_T));

  /* InitializeConditions for Integrator: '<Root>/Integrator Limited' */
  rtX.IntegratorLimited_CSTATE = 0.0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
