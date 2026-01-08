/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UART_DMA.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <stdlib.h>
#include "pid.h"
#include "dwt.h"

pid_type_def pid_cur_q;
pid_type_def pid_cur_d;

float whl_whl=0.0f;

#define _3PI_2 4.71238898038f
#define _2PI 6.28318530718f
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit=24.0f;
float voltage_power_supply=24.0f;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
int DIR= 1;    //传感器方向
int PP=14;    //电机极对数
float angle_prev=0; // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
void setPwm(float Ua, float Ub, float Uc){
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
  htim3.Instance->CCR1 = 1000-dc_a*1000;
  htim3.Instance->CCR2 = 1000-dc_b*1000;
  htim3.Instance->CCR4 = 1000-dc_c*1000;
}

void setSVPWM(float Ua, float Ub, float Uc) {
  float X ,Y ,Z = 0;
  float T1,T2,T1Temp,T2Temp = 0;
  uint8_t A,B,C,N = 0;
  uint16_t Ta,Tb,Tc = 0;
  if(Ua > 0){A = 1;} else{A = 0;}
  if(Ub > 0){B = 1;} else{B = 0;}
  if(Uc > 0){C = 1;} else{C = 0;}
  N = 4 * C + 2 * B + A;

  X = (1.732f * 2000 * Ualpha) / voltage_power_supply;
  Y = (1.5f * Ubeta * 2000 + 0.866f * Ualpha * 2000) / voltage_power_supply;
  Z = (-1.5f * Ubeta * 2000 + 0.866f * Ualpha * 2000) / voltage_power_supply;

  switch(N) {
    case 3: {T1 = -Z; T2 =  X;} break;
    case 1: {T1 =  Z; T2 =  Y;} break;
    case 5: {T1 =  X; T2 = -Y;} break;
    case 4: {T1 = -X; T2 =  Z;} break;
    case 6: {T1 = -Y; T2 = -Z;} break;
    case 2: {T1 =  Y; T2 = -X;} break;
    default:{T1 = 0;  T2=0;}    break;
  }
      T1Temp = T1;
      T2Temp = T2;

      if(T1+T2 > 1900)
      {
        T1 = 1900 * T1Temp / (T1Temp + T2Temp);
        T2 = 1900 * T2Temp / (T1Temp + T2Temp);
      }

      Ta = (2000 - T1 - T2) * 0.25f;
      Tb = Ta + T1 * 0.5f;
      Tc = Tb + T2 * 0.5f;

      switch(N)
      {
        case 3:
        {
          htim3.Instance->CCR1 = Ta;
          htim3.Instance->CCR2 = Tb;
          htim3.Instance->CCR4 = Tc;
        } break;
        case 1:
        {
           htim3.Instance->CCR1 = Tb;
           htim3.Instance->CCR2 = Ta;
           htim3.Instance->CCR4 = Tc;
        } break;
        case 5:
        {
           htim3.Instance->CCR1 = Tc;
           htim3.Instance->CCR2 = Ta;
           htim3.Instance->CCR4 = Tb;
        } break;
        case 4:
        {
           htim3.Instance->CCR1 = Tc;
           htim3.Instance->CCR2 = Tb;
           htim3.Instance->CCR4 = Ta;
        } break;
        case 6:
        {
           htim3.Instance->CCR1 = Tb;
           htim3.Instance->CCR2 = Tc;
           htim3.Instance->CCR4 = Ta;
        } break;
        case 2:
        {
           htim3.Instance->CCR1 = Ta;
           htim3.Instance->CCR2 = Tc;
           htim3.Instance->CCR4 = Tb;
        } break;
        default:
        {
           htim3.Instance->CCR1 = Ta;
           htim3.Instance->CCR2 = Tb;
           htim3.Instance->CCR4 = Tc;
        }break;
      }


}
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);
}

float my_position=0.0f;
uint8_t as5600_data[2] = {};
uint16_t raw_angle;
float angle_deg;
int32_t full_rotations=0; // 总圈数计数
int read_AS5600(){
  HAL_I2C_Mem_Read(&hi2c2, 0x36<<1, 0x0E, I2C_MEMADD_SIZE_8BIT, as5600_data, 2, 0xffff);
  raw_angle = (as5600_data[0] << 8) | as5600_data[1];
  return raw_angle;
}
void getAngle_Without_track(float* position){//my_position*180.0f/PI/0.08789
  *position = read_AS5600()*0.08789* PI / 180;    //得到弧度制的角度
}
float _electricalAngle(){
  return  _normalizeAngle((float)(DIR *  PP) * my_position-zero_electric_angle);
}
float getAngle(){
  float val = my_position;
  float d_angle = val - angle_prev;  //计算旋转的总圈数
  if(abs(d_angle) > (0.5f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1;//通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
  angle_prev = val;
  return (float)full_rotations * 6.28318530718f + angle_prev;
}

void setTorque_QD(float Uq, float Ud, float angle_el) {
  getAngle(); //更新传感器数值
  // 限制 Uq 和 Ud 的范围
  Uq = _constrain(Uq, -(voltage_power_supply)/2, (voltage_power_supply)/2);
  Ud = _constrain(Ud, -(voltage_power_supply)/2, (voltage_power_supply)/2);
  angle_el = _normalizeAngle(angle_el);
  // 提前计算 sin 和 cos，节省计算资源
  float ct = arm_cos_f32(angle_el);
  float st = arm_sin_f32(angle_el);
  // 帕克逆变换 (Inverse Park Transform)
  Ualpha = Ud * ct - Uq * st;
  Ubeta  = Ud * st + Uq * ct;
  // 克拉克逆变换 (Inverse Clarke Transform)
  Ua = Ualpha + voltage_power_supply/2;
  Ub = 0.866f*Ubeta   - 0.5f*Ualpha + voltage_power_supply/2.0f;
  Uc = - 0.866f*Ubeta -0.5f*Ualpha + voltage_power_supply/2.0f;
  setPwm(Ua, Ub, Uc);

}
void setTorque_QD_SVPWM(float Uq, float Ud, float angle_el) {
  getAngle(); //更新传感器数值
  // 限制 Uq 和 Ud 的范围
  Uq = _constrain(Uq, -(voltage_power_supply)/2, (voltage_power_supply)/2);
  Ud = _constrain(Ud, -(voltage_power_supply)/2, (voltage_power_supply)/2);
  angle_el = _normalizeAngle(angle_el);
  // 提前计算 sin 和 cos，节省计算资源
  float ct = arm_cos_f32(angle_el);
  float st = arm_sin_f32(angle_el);
  // 帕克逆变换 (Inverse Park Transform)
  Ualpha = Ud * ct - Uq * st;
  Ubeta  = Ud * st + Uq * ct;
  // 克拉克逆变换 (Inverse Clarke Transform)
  Ua = Ualpha ;
  Ub = 0.866f*Ubeta - 0.5f*Ualpha ;
  Uc = - 0.866f*Ubeta -0.5f*Ualpha ;

  setSVPWM(Ua, Ub, Uc);
}

// 定时器更新中断回调函数
float I_u = 0.0f;
float I_v = 0.0f;
float I_w = 0.0f;
float I_q = 0.0f;
float I_q1_I_d2[2] = {0.0f,0.0f};
int16_t I_cnt = 0;
uint8_t I_flag = 0;
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define SQRT3_2 0.8660254038f
float my_U = 0;
float my_Iq_out = 0.0f;
float my_Id_out = 0.0f;
int32_t whl_cnt = 0;
float speed_rpm = 0.0f;

void cal_Iq_Id(float current_a, float current_c, float current_b, float angle_el, float *ptr_Iq, float *ptr_Id) {
    float I_alpha = 0.66666667f * (current_a - 0.5f * current_b - 0.5f * current_c);
    float I_beta = _1_SQRT3 * (current_b - current_c);
    float ct = arm_cos_f32(angle_el);
    float st = arm_sin_f32(angle_el);

    // Park变换计算 Id (直轴) 和 Iq (交轴)
    *ptr_Iq = I_beta * ct - I_alpha * st;
    *ptr_Id = I_alpha * ct + I_beta * st;
}

float elecangle =0;
int first_cnt = 0;

float my_aim = 0.0f;

void process_adc(const uint16_t* adc_data )
{
  static float last_Iu = 0.0f;
  static float last_Iw = 0.0f;
  static float Iu_offset = 0.0f;
  static float Iw_offset = 0.0f;
  last_Iu = I_u;
  last_Iw = I_w;
  I_u = 0.9f*(((float)(-(adc_data[0] - Iu_offset))*3.3f/4096.0f)/50.0f/0.01f) + 0.1f*last_Iu;
  I_w = 0.9f*(((float)(-(adc_data[1] - Iw_offset))*3.3f/4096.0f)/50.0f/0.01f) + 0.1f*last_Iw;
  I_v = -I_u - I_w;

  if (I_flag < 2){
    if( I_cnt < 5000){
      Iu_offset += (float)adc_data[0];
      Iw_offset += (float)adc_data[1];
      I_cnt++;
    }else{
      Iu_offset /= 5000.0f;
      Iw_offset /= 5000.0f;
      I_flag = 2;
      I_cnt = 0;

    }
  }
  if (I_flag == 2){
    I_cnt++;
    if (I_cnt == 1000){
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      I_cnt = 0;
    }
    cal_Iq_Id(I_u,I_w,I_v,_electricalAngle(), &I_q1_I_d2[0], &I_q1_I_d2[1]);
    if (first_cnt < 5000){
      first_cnt++;
    }else{
      my_Iq_out = PID_calc(&pid_cur_q,I_q1_I_d2[0],my_aim);
      my_Id_out = PID_calc(&pid_cur_d,I_q1_I_d2[1],0.0f);
    }
  }
  whl_cnt++;
  DMA_to_Vofa_v5(speed_rpm,I_q1_I_d2[0],I_q1_I_d2[1],I_v,I_w);

}
//ADC采样中断函数
uint16_t aADCxINJConvertedData[2] = {0};
int64_t whl_cnt1 = 0;
int64_t whl_cnt2 = 0;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
  if(hadc->Instance == ADC1){
    aADCxINJConvertedData[0] = hadc->Instance->JDR1;
    aADCxINJConvertedData[1] = hadc->Instance->JDR2;
    getAngle_Without_track(&my_position);
    whl_cnt2++;
    process_adc(aADCxINJConvertedData);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  static uint16_t TIM3_Cnt1 = 0;
  if(htim->Instance == TIM3){ // 判断是定时器3发生的中断
    TIM3_Cnt1 ++;             // 定时器3每中断一次，计数器自加1
    if (I_flag < 2){
      setTorque_QD(0,0,_electricalAngle());
    }else{
      ////想看电流波形，解除注释
      // elecangle=elecangle+0.005f;
      // if (elecangle > 2*PI){
      //   elecangle -= 2*PI;
      // }
      // setTorque_QD_SVPWM(3.0f,0.0f,elecangle);

      // setTorque_QD(3.0f,0.0f,elecangle);

      setTorque_QD(my_Iq_out,my_Id_out,_electricalAngle());
      // setTorque_QD_SVPWM(my_Iq_out,my_Id_out,_electricalAngle());
    }

  }
}

CAN_Data_Callback CAN_Data_Handler(uint8_t* data, size_t len){
  if (len != 8)return NULL;
#if motor_id==1
   my_aim = (float)((int16_t)(data[0] << 8 | data[1]) /32768.0f); //1
#endif
#if motor_id==2
   my_aim = (float)((int16_t)(data[2] << 8 | data[3]) /32768.0f); //2
#endif
#if motor_id==3
   my_aim = (float)((int16_t)(data[4] << 8 | data[5]) /32768.0f); //3
#endif
#if motor_id==4
  my_aim = (float)((int16_t)(data[6] << 8 | data[7]) /32768.0f); //4
#endif
  return NULL;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // 启动TIM3的PWM通道
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET);
  //编码器0点校准，取消注释后生效
  // setPhaseVoltage(5, 0,_3PI_2);
  // HAL_Delay(100);
  // getAngle_Without_track(&my_position);
  // zero_electric_angle=_electricalAngle();
  // setPhaseVoltage(0, 0,_3PI_2);
#if motor_id==1
  zero_electric_angle = 4.2546978f;     //1
#endif
#if motor_id==2
  zero_electric_angle = 3.53115416f;     //2
#endif
#if motor_id==3
  zero_electric_angle = 1.09209728;    //3
#endif
#if motor_id==4
  zero_electric_angle = 0.26024437f;     //4
#endif

  HAL_TIM_Base_Start_IT(&htim3);
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
  HAL_ADCEx_InjectedStart(&hadc1);
  /*
   * FOC PI参数计算 (FOC PI Parameter Calculation)
   * L (Phase Inductance) = 3.26 mH = 0.00326 H
   * R (Phase Resistance) = 10.32 Ohm
   * Ts (Sampling Time) = 1 / 21250 Hz ≈ 47 us (Assuming 21.25kHz PWM/Control Loop)
   * Bandwidth (Target Current Loop Bandwidth) = 1000 rad/s (~160 Hz)
   *
   * Formula:
   * Kp = L * Bandwidth
   * Ki = R * Bandwidth * Ts
   *
   * Calculation:
   * Kp = 0.00326 * 1000 = 3.26
   * Ki = 10.32 * 1000 * 0.000047 ≈ 0.485
   */
  float pid_param[3] = {3.26f, 0.485f, 0.0f}; //1.0f 0.038f
  PID_init(&pid_cur_q,PID_POSITION,pid_param,12.0f,12.0f);
  PID_init(&pid_cur_d,PID_POSITION,pid_param,12.0f,12.0f);

  PID_clear(&pid_cur_q);
  PID_clear(&pid_cur_d);

  USER_FDCAN_Filter_Init();
  CAN_RegisterCallback((CAN_Data_Callback)CAN_Data_Handler);

  BSP_DWT_Init();
  uint32_t loop_tick = 0;
  BSP_DWT_GetDeltaT(&loop_tick);
  double Dt_Encoder = 0.0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Dt_Encoder = BSP_DWT_GetDeltaT64(&loop_tick);
    float speed_rpm_last = 0.0f;
    speed_rpm_last = speed_rpm;
    static int16_t Last_Encoder = 0;
    static int16_t Encoder = 0;
    int16_t Encoder_Err = 0;
    Last_Encoder = Encoder;
    Encoder = (int16_t)(my_position*180.0f/PI/0.08789*2.0f);
    Encoder_Err = Encoder - Last_Encoder;//Encoder > Last_Encoder反转转到0了;
     if (abs(Encoder_Err) > 4096){
      if (Encoder_Err > 0) Encoder_Err = -(Last_Encoder + 8192 - Encoder);
      else Encoder_Err =  Encoder + 8192 - Last_Encoder;
     }
     speed_rpm = 0.9f*Encoder_Err / 8192.0f/Dt_Encoder * 60.0f + 0.1f*speed_rpm_last;
    uint8_t tx_data[8] = {0};
    CAN_cmd(Encoder,(int16_t)speed_rpm,(int16_t)(I_q1_I_d2[0]*10000),0x00);
    HAL_Delay(1);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
