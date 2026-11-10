/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: AHRS_IKF.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 07-Nov-2026 17:12:12
 */

/* Include Files */
#include "AHRS_IKF.h"
#include "AHRS_IKF_data.h"
#include "AHRS_IKF_initialize.h"
#include "AHRS_IKF_types.h"
#include "diag.h"
#include "mrdivide_helper.h"
#include "mtimes.h"
#include "norm.h"
#include "quat2rotm.h"
#include "quaternion.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static const signed char iv[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                   0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                   0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};

/* Function Declarations */
static void AHRS_IKF_external_update(AHRS_IKF *obj, float ext_vec[3],
                                     const float G_expected_vec[3],
                                     const float noise_covariance_R[9]);

/* Function Definitions */
/*
 * Arguments    : AHRS_IKF *obj
 *                float ext_vec[3]
 *                const float G_expected_vec[3]
 *                const float noise_covariance_R[9]
 * Return Type  : void
 */
static void AHRS_IKF_external_update(AHRS_IKF *obj, float ext_vec[3],
                                     const float G_expected_vec[3],
                                     const float noise_covariance_R[9])
{
  float c_obj[36];
  float b_obj[18];
  float c_K[18];
  float b_ext_vec[6];
  float delta_q[4];
  float b_G_expected_vec[3];
  float t0_c;
  float t0_d;
  float vec_norm;
  int i;
  int i1;
  int i2;
  /*  Generic update from a known external vector */
  vec_norm = b_norm(ext_vec);
  /*  shouldn't happen, but check just in case */
  if (vec_norm > 1.1920929E-7F)
  {
    float d_K[36];
    float fv[36];
    float H[18];
    float b_K[18];
    float K[9];
    float delta_x[6];
    float vec_pred[3];
    float t0_a;
    int K_tmp;
    int obj_tmp;
    /*  normalize input to unit vector */
    ext_vec[0] /= vec_norm;
    ext_vec[1] /= vec_norm;
    ext_vec[2] /= vec_norm;
    quat2rotm(obj->q.a, obj->q.b, obj->q.c, obj->q.d, K);
    /*  Global to local rotation matrix */
    memset(&b_G_expected_vec[0], 0, 3U * sizeof(float));
    /*  1. Calculate measurement matrix H */
    /* SKEW Converts 3-element vector to skew-symmetric matrix */
    H[0] = 0.0F;
    H[7] = 0.0F;
    H[14] = 0.0F;
    vec_norm = G_expected_vec[0];
    t0_c = G_expected_vec[1];
    t0_d = G_expected_vec[2];
    for (i = 0; i < 3; i++)
    {
      H[6 * i + 3] = 0.0F;
      H[6 * i + 4] = 0.0F;
      t0_a = ((b_G_expected_vec[i] + vec_norm * K[i]) + t0_c * K[i + 3]) +
             t0_d * K[i + 6];
      H[6 * i + 5] = 0.0F;
      b_G_expected_vec[i] = t0_a;
      vec_pred[i] = t0_a;
    }
    H[1] = -vec_pred[2];
    H[2] = vec_pred[1];
    H[6] = vec_pred[2];
    H[8] = -vec_pred[0];
    H[12] = -vec_pred[1];
    H[13] = vec_pred[0];
    /*  2. Calculate residual r */
    /*  3. Covariance of residual */
    for (i = 0; i < 6; i++)
    {
      b_K[3 * i] = H[i];
      b_K[3 * i + 1] = H[i + 6];
      b_K[3 * i + 2] = H[i + 12];
    }
    /*  4. Kalman Gain */
    memset(&b_obj[0], 0, 18U * sizeof(float));
    for (i = 0; i < 3; i++)
    {
      for (i1 = 0; i1 < 6; i1++)
      {
        vec_norm = H[i1 + 6 * i];
        for (i2 = 0; i2 < 6; i2++)
        {
          obj_tmp = i2 + 6 * i;
          b_obj[obj_tmp] += obj->state_covariance_P[i2 + 6 * i1] * vec_norm;
        }
      }
    }
    memset(&c_K[0], 0, 18U * sizeof(float));
    for (i = 0; i < 6; i++)
    {
      vec_norm = c_K[3 * i];
      obj_tmp = 3 * i + 1;
      K_tmp = 3 * i + 2;
      for (i1 = 0; i1 < 6; i1++)
      {
        t0_c = obj->state_covariance_P[i1 + 6 * i];
        vec_norm += b_K[3 * i1] * t0_c;
        c_K[obj_tmp] += b_K[3 * i1 + 1] * t0_c;
        c_K[K_tmp] += b_K[3 * i1 + 2] * t0_c;
      }
      c_K[3 * i] = vec_norm;
    }
    for (i1 = 0; i1 < 3; i1++)
    {
      for (i2 = 0; i2 < 3; i2++)
      {
        vec_norm = 0.0F;
        for (i = 0; i < 6; i++)
        {
          vec_norm += b_K[i1 + 3 * i] * b_obj[i + 6 * i2];
        }
        obj_tmp = i1 + 3 * i2;
        K[obj_tmp] = vec_norm + noise_covariance_R[obj_tmp];
      }
    }
    mrdiv(c_K, K, b_K);
    /*  5. Correction vector */
    memset(&b_ext_vec[0], 0, 6U * sizeof(float));
    vec_norm = ext_vec[0] - vec_pred[0];
    t0_c = ext_vec[1] - vec_pred[1];
    t0_d = ext_vec[2] - vec_pred[2];
    for (i = 0; i < 6; i++)
    {
      t0_a = ((b_ext_vec[i] + vec_norm * b_K[3 * i]) + t0_c * b_K[3 * i + 1]) +
             t0_d * b_K[3 * i + 2];
      b_ext_vec[i] = t0_a;
      delta_x[i] = t0_a;
    }
    /*  6. Update quaternion */
    t0_c = delta_x[0] / 2.0F;
    vec_pred[0] = t0_c;
    vec_norm = t0_c * t0_c;
    t0_c = delta_x[1] / 2.0F;
    vec_pred[1] = t0_c;
    vec_norm += t0_c * t0_c;
    t0_c = delta_x[2] / 2.0F;
    vec_norm += t0_c * t0_c;
    if (vec_norm <= 1.0F)
    {
      delta_q[0] = sqrtf(1.0F - vec_norm);
      delta_q[1] = vec_pred[0];
      delta_q[2] = vec_pred[1];
      delta_q[3] = t0_c;
    }
    else
    {
      vec_norm = 1.0F / sqrtf(vec_norm + 1.0F);
      delta_q[0] = vec_norm;
      delta_q[1] = vec_norm * vec_pred[0];
      delta_q[2] = vec_norm * vec_pred[1];
      delta_q[3] = vec_norm * t0_c;
    }
    t0_a = quaternion_quaternion(delta_q, &vec_norm, &t0_c, &t0_d);
    obj->q.a = quaternionBase_mtimes(obj->q.a, obj->q.b, obj->q.c, obj->q.d,
                                     t0_a, vec_norm, t0_c, t0_d, &obj->q.b,
                                     &obj->q.c, &obj->q.d);
    /*  corrected q */
    /*  7. Update bias */
    obj->bias_b[0] += delta_x[3];
    obj->bias_b[1] += delta_x[4];
    obj->bias_b[2] += delta_x[5];
    /*  8. Calculate new state covariance */
    for (i = 0; i < 6; i++)
    {
      vec_norm = H[i];
      t0_c = H[i + 6];
      t0_d = H[i + 12];
      for (i1 = 0; i1 < 6; i1++)
      {
        obj_tmp = i + 6 * i1;
        d_K[obj_tmp] = (float)iv[obj_tmp] -
                       ((vec_norm * b_K[3 * i1] + t0_c * b_K[3 * i1 + 1]) +
                        t0_d * b_K[3 * i1 + 2]);
      }
    }
    memset(&c_obj[0], 0, 36U * sizeof(float));
    memset(&c_K[0], 0, 18U * sizeof(float));
    for (i2 = 0; i2 < 6; i2++)
    {
      for (i = 0; i < 6; i++)
      {
        vec_norm = d_K[i + 6 * i2];
        for (i1 = 0; i1 < 6; i1++)
        {
          obj_tmp = i1 + 6 * i2;
          c_obj[obj_tmp] += obj->state_covariance_P[i1 + 6 * i] * vec_norm;
        }
      }
      vec_norm = c_K[3 * i2];
      obj_tmp = 3 * i2 + 1;
      K_tmp = 3 * i2 + 2;
      for (i = 0; i < 3; i++)
      {
        t0_c = b_K[i + 3 * i2];
        vec_norm += noise_covariance_R[3 * i] * t0_c;
        c_K[obj_tmp] += noise_covariance_R[3 * i + 1] * t0_c;
        c_K[K_tmp] += noise_covariance_R[3 * i + 2] * t0_c;
      }
      c_K[3 * i2] = vec_norm;
      for (i = 0; i < 6; i++)
      {
        vec_norm = 0.0F;
        for (i1 = 0; i1 < 6; i1++)
        {
          vec_norm += d_K[i1 + 6 * i] * c_obj[i1 + 6 * i2];
        }
        fv[i + 6 * i2] = vec_norm;
      }
    }
    for (i = 0; i < 6; i++)
    {
      vec_norm = c_K[3 * i];
      t0_c = c_K[3 * i + 1];
      t0_d = c_K[3 * i + 2];
      for (i1 = 0; i1 < 6; i1++)
      {
        d_K[i1 + 6 * i] = (b_K[3 * i1] * vec_norm + b_K[3 * i1 + 1] * t0_c) +
                          b_K[3 * i1 + 2] * t0_d;
      }
    }
    for (i = 0; i < 36; i++)
    {
      obj->state_covariance_P[i] = fv[i] + d_K[i];
    }
    /*  debug type check to make sure everything is single precision */
    /*  whos */
  }
}

/*
 * Arguments    : AHRS_IKF *obj
 *                float gyro_noise
 *                float gyro_bias
 *                const float mag_noise[3]
 *                const float accel_noise[3]
 *                const float global_mag_ref[3]
 *                float small_angle_threshold
 *                float b_time
 * Return Type  : void
 */
void AHRS_IKF_AHRS_IKF(AHRS_IKF *obj, float gyro_noise, float gyro_bias,
                       const float mag_noise[3], const float accel_noise[3],
                       const float global_mag_ref[3],
                       float small_angle_threshold, float b_time)
{
  int i;
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  for (i = 0; i < 36; i++)
  {
    obj->state_covariance_P[i] = iv[i];
  }
  float y;
  obj->gyro_noise = gyro_noise;
  obj->gyro_bias = gyro_bias;
  obj->small_angle_threshold = small_angle_threshold;
  y = b_norm(global_mag_ref);
  obj->bias_b[0] = 0.0F;
  obj->mag_noise[0] = mag_noise[0];
  obj->accel_noise[0] = accel_noise[0];
  obj->G_mag[0] = global_mag_ref[0] / y;
  obj->bias_b[1] = 0.0F;
  obj->mag_noise[1] = mag_noise[1];
  obj->accel_noise[1] = accel_noise[1];
  obj->G_mag[1] = global_mag_ref[1] / y;
  obj->bias_b[2] = 0.0F;
  obj->mag_noise[2] = mag_noise[2];
  obj->accel_noise[2] = accel_noise[2];
  obj->G_mag[2] = global_mag_ref[2] / y;
  obj->q.a = 1.0F;
  obj->q.b = 0.0F;
  obj->q.c = 0.0F;
  obj->q.d = 0.0F;
  obj->last_update_time = b_time;
}

/*
 * Arguments    : const AHRS_IKF *obj
 *                float result[3]
 * Return Type  : void
 */
void AHRS_IKF_get_accel_noise(const AHRS_IKF *obj, float result[3])
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  result[0] = obj->accel_noise[0];
  result[1] = obj->accel_noise[1];
  result[2] = obj->accel_noise[2];
}

/*
 * Arguments    : const AHRS_IKF *obj
 * Return Type  : quaternion
 */
quaternion AHRS_IKF_get_attitude(const AHRS_IKF *obj)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  /*         %% Getters */
  return obj->q;
}

/*
 * Arguments    : const AHRS_IKF *obj
 * Return Type  : float
 */
float AHRS_IKF_get_gyro_bias(const AHRS_IKF *obj)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  return obj->gyro_bias;
}

/*
 * Arguments    : const AHRS_IKF *obj
 * Return Type  : float
 */
float AHRS_IKF_get_gyro_noise(const AHRS_IKF *obj)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  return obj->gyro_noise;
}

/*
 * Arguments    : const AHRS_IKF *obj
 * Return Type  : float
 */
float AHRS_IKF_get_last_update_time(const AHRS_IKF *obj)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  return obj->last_update_time;
}

/*
 * Arguments    : const AHRS_IKF *obj
 *                float result[3]
 * Return Type  : void
 */
void AHRS_IKF_get_mag_noise(const AHRS_IKF *obj, float result[3])
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  result[0] = obj->mag_noise[0];
  result[1] = obj->mag_noise[1];
  result[2] = obj->mag_noise[2];
}

/*
 * Arguments    : const AHRS_IKF *obj
 * Return Type  : float
 */
float AHRS_IKF_get_small_angle_threshold(const AHRS_IKF *obj)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  return obj->small_angle_threshold;
}

/*
 * Arguments    : AHRS_IKF *obj
 *                const float val[3]
 * Return Type  : void
 */
void AHRS_IKF_set_accel_noise(AHRS_IKF *obj, const float val[3])
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  obj->accel_noise[0] = val[0];
  obj->accel_noise[1] = val[1];
  obj->accel_noise[2] = val[2];
}

/*
 * Arguments    : AHRS_IKF *obj
 *                float val
 * Return Type  : void
 */
void AHRS_IKF_set_gyro_bias(AHRS_IKF *obj, float val)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  obj->gyro_bias = val;
}

/*
 * Arguments    : AHRS_IKF *obj
 *                float val
 * Return Type  : void
 */
void AHRS_IKF_set_gyro_noise(AHRS_IKF *obj, float val)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  obj->gyro_noise = val;
}

/*
 * Arguments    : AHRS_IKF *obj
 *                float val
 * Return Type  : void
 */
void AHRS_IKF_set_last_update_time(AHRS_IKF *obj, float val)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  obj->last_update_time = val;
}

/*
 * Arguments    : AHRS_IKF *obj
 *                const float val[3]
 * Return Type  : void
 */
void AHRS_IKF_set_mag_noise(AHRS_IKF *obj, const float val[3])
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  obj->mag_noise[0] = val[0];
  obj->mag_noise[1] = val[1];
  obj->mag_noise[2] = val[2];
}

/*
 * Arguments    : AHRS_IKF *obj
 *                float val
 * Return Type  : void
 */
void AHRS_IKF_set_small_angle_threshold(AHRS_IKF *obj, float val)
{
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  obj->small_angle_threshold = val;
}

/*
 * Arguments    : AHRS_IKF *obj
 *                const float accel[3]
 * Return Type  : void
 */
void AHRS_IKF_update_accel(AHRS_IKF *obj, const float accel[3])
{
  static const float fv1[3] = {0.0F, 0.0F, 1.0F};
  float fv[9];
  float b_accel[3];
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  diag(obj->accel_noise, fv);
  b_accel[0] = accel[0];
  b_accel[1] = accel[1];
  b_accel[2] = accel[2];
  AHRS_IKF_external_update(obj, b_accel, fv1, fv);
}

/*
 * Arguments    : AHRS_IKF *obj
 *                const float omega_m[3]
 *                float b_time
 * Return Type  : void
 */
void AHRS_IKF_update_gyro(AHRS_IKF *obj, const float omega_m[3], float b_time)
{
  static const signed char b_iv[18] = {0, 0, 0, 1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0, 0, 0, 1};
  static const signed char c_b[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static const signed char d_a[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
  float b_obj[36];
  float noise_covariance_Q[36];
  float state_transition_Phi[36];
  float b_b[9];
  float omega_corrected[3];
  float b;
  float dt;
  float omega_mag;
  float t1_a;
  float t1_d;
  float trig_arg;
  int b_i;
  int b_tmp;
  int i;
  int i2;
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  /*         %% Update functions */
  /*  radians */
  dt = b_time - obj->last_update_time;
  obj->last_update_time = b_time;
  /*  1. Propagate bias */
  /*  2. New turn rate estimate */
  omega_corrected[0] = omega_m[0] - obj->bias_b[0];
  omega_corrected[1] = omega_m[1] - obj->bias_b[1];
  omega_corrected[2] = omega_m[2] - obj->bias_b[2];
  omega_mag = b_norm(omega_corrected);
  /*  3. Propagate quaternion (simplified) */
  if (omega_mag > 0.0F)
  {
    float fv[4];
    trig_arg = omega_mag * dt / 2.0F;
    b = sinf(trig_arg);
    fv[0] = cosf(trig_arg);
    fv[1] = omega_corrected[0] / omega_mag * b;
    fv[2] = omega_corrected[1] / omega_mag * b;
    fv[3] = omega_corrected[2] / omega_mag * b;
    t1_a = quaternion_quaternion(fv, &trig_arg, &b, &t1_d);
    obj->q.a = quaternionBase_mtimes(obj->q.a, obj->q.b, obj->q.c, obj->q.d,
                                     t1_a, trig_arg, b, t1_d, &obj->q.b,
                                     &obj->q.c, &obj->q.d);
  }
  /*  4. State transition matrix Phi and discrete noise covariance */
  /*  matrix Q */
  if (omega_mag < obj->small_angle_threshold)
  {
    float wx[9];
    float a;
    float c_a;
    float w_dt;
    float y;
    int b_b_tmp;
    /* SKEW Converts 3-element vector to skew-symmetric matrix */
    wx[0] = 0.0F;
    wx[1] = -omega_corrected[2];
    wx[2] = omega_corrected[1];
    wx[3] = omega_corrected[2];
    wx[4] = 0.0F;
    wx[5] = -omega_corrected[0];
    wx[6] = -omega_corrected[1];
    wx[7] = omega_corrected[0];
    wx[8] = 0.0F;
    memset(&b_b[0], 0, 9U * sizeof(float));
    w_dt = dt * dt;
    t1_a = w_dt / 2.0F;
    y = powf(dt, 3.0F);
    a = y / 6.0F;
    for (i = 0; i < 3; i++)
    {
      b = b_b[3 * i];
      b_tmp = 3 * i + 1;
      b_b_tmp = 3 * i + 2;
      for (b_i = 0; b_i < 3; b_i++)
      {
        trig_arg = wx[b_i + 3 * i];
        b += wx[3 * b_i] * trig_arg;
        b_b[b_tmp] += wx[3 * b_i + 1] * trig_arg;
        b_b[b_b_tmp] += wx[3 * b_i + 2] * trig_arg;
      }
      b_b[3 * i] = b;
      t1_d = wx[3 * i];
      state_transition_Phi[6 * i] = ((float)c_b[3 * i] - dt * t1_d) + t1_a * b;
      state_transition_Phi[6 * i + 3] =
          ((float)d_a[3 * i] * dt + t1_a * t1_d) - a * b;
      b_tmp = 3 * i + 1;
      t1_d = wx[b_tmp];
      trig_arg = b_b[b_tmp];
      state_transition_Phi[6 * i + 1] =
          ((float)c_b[b_tmp] - dt * t1_d) + t1_a * trig_arg;
      state_transition_Phi[6 * i + 4] =
          ((float)d_a[b_tmp] * dt + t1_a * t1_d) - a * trig_arg;
      b_tmp = 3 * i + 2;
      t1_d = wx[b_tmp];
      trig_arg = b_b[b_tmp];
      state_transition_Phi[6 * i + 2] =
          ((float)c_b[b_tmp] - dt * t1_d) + t1_a * trig_arg;
      state_transition_Phi[6 * i + 5] =
          ((float)d_a[b_tmp] * dt + t1_a * t1_d) - a * trig_arg;
      for (b_i = 0; b_i < 6; b_i++)
      {
        state_transition_Phi[b_i + 6 * (i + 3)] = b_iv[b_i + 6 * i];
      }
    }
    /* SKEW Converts 3-element vector to skew-symmetric matrix */
    t1_d = obj->gyro_noise * dt;
    t1_a = 2.0F * powf(dt, 5.0F) / 120.0F;
    c_a = obj->gyro_bias;
    trig_arg = powf(dt, 4.0F) / 24.0F;
    b = -obj->gyro_bias;
    for (b_i = 0; b_i < 9; b_i++)
    {
      wx[b_i] = b * (((float)c_b[b_i] * w_dt / 2.0F - a * wx[b_i]) +
                     trig_arg * b_b[b_i]);
    }
    trig_arg = obj->gyro_bias * dt;
    for (b_i = 0; b_i < 3; b_i++)
    {
      signed char i1;
      i1 = c_b[3 * b_i];
      noise_covariance_Q[6 * b_i] =
          t1_d * (float)i1 + c_a * ((float)i1 * y / 3.0F + t1_a * b_b[3 * b_i]);
      noise_covariance_Q[6 * b_i + 3] = wx[3 * b_i];
      b_b_tmp = 6 * (b_i + 3);
      noise_covariance_Q[b_b_tmp] = wx[b_i];
      noise_covariance_Q[b_b_tmp + 3] = trig_arg * (float)i1;
      b_tmp = 3 * b_i + 1;
      i1 = c_b[b_tmp];
      noise_covariance_Q[6 * b_i + 1] =
          t1_d * (float)i1 + c_a * ((float)i1 * y / 3.0F + t1_a * b_b[b_tmp]);
      noise_covariance_Q[6 * b_i + 4] = wx[b_tmp];
      noise_covariance_Q[b_b_tmp + 1] = wx[b_i + 3];
      noise_covariance_Q[b_b_tmp + 4] = trig_arg * (float)i1;
      b_tmp = 3 * b_i + 2;
      i1 = c_b[b_tmp];
      noise_covariance_Q[6 * b_i + 2] =
          t1_d * (float)i1 + c_a * ((float)i1 * y / 3.0F + t1_a * b_b[b_tmp]);
      noise_covariance_Q[6 * b_i + 5] = wx[b_tmp];
      noise_covariance_Q[b_b_tmp + 2] = wx[b_i + 6];
      noise_covariance_Q[b_b_tmp + 5] = trig_arg * (float)i1;
    }
  }
  else
  {
    float fv1[9];
    float wx[9];
    float u[3];
    float a;
    float b_a;
    float c_a;
    float d_b;
    float w_dt;
    float y;
    int b_b_tmp;
    w_dt = omega_mag * dt;
    /* SKEW Converts 3-element vector to skew-symmetric matrix */
    wx[0] = 0.0F;
    wx[1] = -omega_corrected[2];
    wx[2] = omega_corrected[1];
    wx[3] = omega_corrected[2];
    wx[4] = 0.0F;
    wx[5] = -omega_corrected[0];
    wx[6] = -omega_corrected[1];
    wx[7] = omega_corrected[0];
    wx[8] = 0.0F;
    a = cosf(w_dt);
    t1_a = sinf(w_dt);
    /* SKEW Converts 3-element vector to skew-symmetric matrix */
    trig_arg = (1.0F - a) / (omega_mag * omega_mag);
    b_a = (w_dt - t1_a) / powf(omega_mag, 3.0F);
    memset(&b_b[0], 0, 9U * sizeof(float));
    for (b_i = 0; b_i < 3; b_i++)
    {
      u[b_i] = omega_corrected[b_i] / omega_mag;
      b = b_b[3 * b_i];
      b_tmp = 3 * b_i + 1;
      b_b_tmp = 3 * b_i + 2;
      for (i = 0; i < 3; i++)
      {
        t1_d = wx[i + 3 * b_i];
        b += wx[3 * i] * t1_d;
        b_b[b_tmp] += wx[3 * i + 1] * t1_d;
        b_b[b_b_tmp] += wx[3 * i + 2] * t1_d;
      }
      b_b[3 * b_i] = b;
    }
    fv1[0] = 0.0F;
    fv1[1] = t1_a * -u[2];
    fv1[2] = t1_a * u[1];
    fv1[3] = t1_a * u[2];
    fv1[4] = 0.0F;
    fv1[5] = t1_a * -u[0];
    fv1[6] = t1_a * -u[1];
    fv1[7] = t1_a * u[0];
    fv1[8] = 0.0F;
    for (i = 0; i < 3; i++)
    {
      state_transition_Phi[6 * i] =
          (a * (float)c_b[3 * i] - fv1[3 * i]) + (1.0F - a) * (u[0] * u[i]);
      state_transition_Phi[6 * i + 3] =
          ((float)d_a[3 * i] * dt + trig_arg * wx[3 * i]) - b_a * b_b[3 * i];
      b_tmp = 3 * i + 1;
      state_transition_Phi[6 * i + 1] =
          (a * (float)c_b[b_tmp] - fv1[b_tmp]) + (1.0F - a) * (u[1] * u[i]);
      state_transition_Phi[6 * i + 4] =
          ((float)d_a[b_tmp] * dt + trig_arg * wx[b_tmp]) - b_a * b_b[b_tmp];
      b_tmp = 3 * i + 2;
      state_transition_Phi[6 * i + 2] =
          (a * (float)c_b[b_tmp] - fv1[b_tmp]) + (1.0F - a) * (u[2] * u[i]);
      state_transition_Phi[6 * i + 5] =
          ((float)d_a[b_tmp] * dt + trig_arg * wx[b_tmp]) - b_a * b_b[b_tmp];
      for (b_i = 0; b_i < 6; b_i++)
      {
        state_transition_Phi[b_i + 6 * (i + 3)] = b_iv[b_i + 6 * i];
      }
    }
    /* SKEW Converts 3-element vector to skew-symmetric matrix */
    y = obj->gyro_noise * dt;
    d_b = powf(dt, 3.0F);
    t1_a = ((powf(w_dt, 3.0F) + 2.0F * t1_a) - 2.0F * w_dt) /
           powf(omega_mag, 5.0F);
    c_a = obj->gyro_bias;
    t1_d = dt * dt;
    trig_arg = ((w_dt * w_dt / 2.0F + a) - 1.0F) / powf(omega_mag, 4.0F);
    b = -obj->gyro_bias;
    for (b_i = 0; b_i < 9; b_i++)
    {
      wx[b_i] = b * (((float)c_b[b_i] * t1_d / 2.0F - b_a * wx[b_i]) +
                     trig_arg * b_b[b_i]);
    }
    trig_arg = obj->gyro_bias * dt;
    for (i = 0; i < 3; i++)
    {
      signed char i1;
      i1 = c_b[3 * i];
      noise_covariance_Q[6 * i] =
          y * (float)i1 + c_a * ((float)i1 * d_b / 3.0F + t1_a * b_b[3 * i]);
      noise_covariance_Q[6 * i + 3] = wx[3 * i];
      b_b_tmp = 6 * (i + 3);
      noise_covariance_Q[b_b_tmp] = wx[i];
      noise_covariance_Q[b_b_tmp + 3] = trig_arg * (float)i1;
      b_tmp = 3 * i + 1;
      i1 = c_b[b_tmp];
      noise_covariance_Q[6 * i + 1] =
          y * (float)i1 + c_a * ((float)i1 * d_b / 3.0F + t1_a * b_b[b_tmp]);
      noise_covariance_Q[6 * i + 4] = wx[b_tmp];
      noise_covariance_Q[b_b_tmp + 1] = wx[i + 3];
      noise_covariance_Q[b_b_tmp + 4] = trig_arg * (float)i1;
      b_tmp = 3 * i + 2;
      i1 = c_b[b_tmp];
      noise_covariance_Q[6 * i + 2] =
          y * (float)i1 + c_a * ((float)i1 * d_b / 3.0F + t1_a * b_b[b_tmp]);
      noise_covariance_Q[6 * i + 5] = wx[b_tmp];
      noise_covariance_Q[b_b_tmp + 2] = wx[i + 6];
      noise_covariance_Q[b_b_tmp + 5] = trig_arg * (float)i1;
    }
  }
  /*  5. State covariance matrix  */
  memset(&b_obj[0], 0, 36U * sizeof(float));
  for (b_i = 0; b_i < 6; b_i++)
  {
    for (i = 0; i < 6; i++)
    {
      trig_arg = state_transition_Phi[i + 6 * b_i];
      for (i2 = 0; i2 < 6; i2++)
      {
        b_tmp = i2 + 6 * b_i;
        b_obj[b_tmp] += obj->state_covariance_P[i2 + 6 * i] * trig_arg;
      }
    }
  }
  for (i = 0; i < 6; i++)
  {
    for (i2 = 0; i2 < 6; i2++)
    {
      trig_arg = 0.0F;
      for (b_i = 0; b_i < 6; b_i++)
      {
        trig_arg += state_transition_Phi[b_i + 6 * i] * b_obj[b_i + 6 * i2];
      }
      b_tmp = i + 6 * i2;
      obj->state_covariance_P[b_tmp] = trig_arg + noise_covariance_Q[b_tmp];
    }
  }
}

/*
 * Arguments    : AHRS_IKF *obj
 *                const float mag[3]
 * Return Type  : void
 */
void AHRS_IKF_update_mag(AHRS_IKF *obj, const float mag[3])
{
  float fv[9];
  float b_mag[3];
  if (!isInitialized_AHRS_IKF)
  {
    AHRS_IKF_initialize();
  }
  diag(obj->mag_noise, fv);
  b_mag[0] = mag[0];
  b_mag[1] = mag[1];
  b_mag[2] = mag[2];
  AHRS_IKF_external_update(obj, b_mag, obj->G_mag, fv);
}

/*
 * File trailer for AHRS_IKF.c
 *
 * [EOF]
 */
