
#include "bsp_motor.h"
#include "tim.h"        /* 需要 htim1 的声明 */
#include "main.h"       /* HAL 函数 */
void PWMinit(){
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//启动PWM1输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);//启动PWM2输出

    /* ★ 启动编码器计数：TIM2/TIM4 已配置成编码器模式（tim.c），
     *   但必须先 HAL_TIM_Encoder_Start 才会开始计数，否则 read_encoder 恒为 0 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}
void setPWM1(float v1){
    if(v1>1.0f)v1=1.0f; // 限制占空比范围为 0~1
    if(v1<-1.0f) v1=-1.0f; //   
    if(v1>0){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // 设置PA8为高电平

    }
    else{
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // 设置PA8为低电平
        v1=-v1;
        
    }
    uint32_t arr1=__HAL_TIM_GetAutoreload(&htim1); // 获取ARR寄存器的值
    uint32_t ccr1=v1*(arr1+1); // 计算CCR寄存器的值
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1); // 设置PWM1占空比
}
void setPWM2(float v2){
    if(v2>1.0f) v2=1.0f; // 限制占空比范围为 0~1
    if(v2<-1.0f) v2=-1.0f; //   
    if(v2>0){
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // 设置PA13为高电平

    }
    else{
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // 设置PA13为低电平
        v2=-v2;
    }
    uint32_t arr2=__HAL_TIM_GetAutoreload(&htim1); // 获取ARR寄存器的值
    uint32_t ccr2=v2*(arr2+1); // 计算CCR寄存器的值
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ccr2); // 设置PWM2占空比
    }

int16_t read_encoder1(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);  /* ← 就是这一行，读取 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);                        /* 清零 */
    return count;                                            /* 返回 */
}

int16_t read_encoder2(void)
{
    int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);  /* ← 就是这一行，读取 */
    __HAL_TIM_SET_COUNTER(&htim4, 0);                        /* 清零 */
    return count;                                            /* 返回 */
}
    
