#define __KALMAN_GLOBALS

#include "kalman.h"

#define KalmanFilter_GM_default \
{\
	 0.0f,\
	{0.0f},\
	{0.0f},\
	{0.0f},\
	{0.0f},\
	{1.0f,0.0,0.0f,1.0f},\
	{0.0f},\
	{1.0f,0.0f,0.0f,1.0f},\
	{0.0f},\
	{0.0f},\
	{1.0f,0.005f,0.0f,1.0f},\
	{1.0f,0.0f,0.0f,1.0f},\
	{1.0f,0.0f,0.0f,1.0f},\
	{20.0f,0.0f,0.0f,100.0f},\
}


kalman_filter_t kalman_filter_gimbal_yaw_matrix={0};
kalman_filter_init_t kalman_filter_init_gimbal_yaw_data=KalmanFilter_GM_default;
kalman_filter_t kalman_filter_gimbal_pitch_matrix={0};
kalman_filter_init_t kalman_filter_init_gimbal_pitch_data=KalmanFilter_GM_default;


void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{

  mat_init(&F->xhat,2,1,(float *)I->xhat_data);
  mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
	mat_init(&F->z,2,1,(float *)I->z_data);
	mat_init(&F->A,2,2,(float *)I->A_data);
	mat_init(&F->H,2,2,(float *)I->H_data);
	mat_init(&F->Q,2,2,(float *)I->Q_data);
	mat_init(&F->R,2,2,(float *)I->R_data);
	mat_init(&F->P,2,2,(float *)I->P_data);	
	mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
	mat_init(&F->K,2,2,(float *)I->K_data);
  mat_init(&F->AT,2,2,(float *)I->AT_data);
  mat_init(&F->HT,2,2,(float *)I->HT_data);
  mat_trans(&F->H, &F->HT);
	mat_trans(&F->A, &F->AT);
}


void kalman_filter_matdata_init(void)
{
	 kalman_filter_init(&kalman_filter_gimbal_yaw_matrix,&kalman_filter_init_gimbal_yaw_data);
	 kalman_filter_init(&kalman_filter_gimbal_pitch_matrix,&kalman_filter_init_gimbal_pitch_data);
}


float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)   //本次角度 上次角度
{

  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);

  F->z.pData[0] = signal1;
  F->z.pData[1] = signal2-signal1;

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)
  mat_mult(&F->H, &F->Pminus, &F->K);
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);
  mat_sub(&F->z, &TEMP21, &F->xhat);
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);
  mat_sub(&F->Q, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];
  F->filtered_value[1] = F->xhat.pData[1];

  return F->filtered_value;
}




	



