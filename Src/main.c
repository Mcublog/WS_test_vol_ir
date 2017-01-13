/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "init_main.h"
#include "Nec.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim1_ch1_ch2_ch3;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct
{
	uint8_t ctrl;
	uint8_t vol;
}vol_t __attribute__((aligned (8)));

typedef struct 
{ 
	uint8_t adr_pot;

	uint8_t vol;//общая громкость
	
	vol_t vol_ctrl_l;
	vol_t vol_ctrl_r;
}vol_ch_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define BUFF_LEN (255)
uint32_t i,j=0;
uint16_t buff_cnt1[BUFF_LEN];
uint16_t buff_cnt2[BUFF_LEN];

vol_ch_t ch_vol;

nec_ir_t Nec;

#define DG_POT_ADR (0x58)

#define BT_PROG (0x05)//PROGRAM
#define BT_P10  (0x0E)//+10 Vol
#define BT_N10  (0x1B)//-10
#define BT_P1   (0x06)//+1 Vol 
#define BT_N1   (0x0A)//-1

void ch_set_vol_all(vol_ch_t* vol_ch_ctrl)
{
	vol_ch_ctrl->vol_ctrl_l.vol=vol_ch_ctrl->vol;
	vol_ch_ctrl->vol_ctrl_r.vol=vol_ch_ctrl->vol;
	HAL_I2C_Master_Transmit(&hi2c1, vol_ch_ctrl->adr_pot,(uint8_t*) &vol_ch_ctrl->vol_ctrl_l, 2, 1000);	
	HAL_I2C_Master_Transmit(&hi2c1, vol_ch_ctrl->adr_pot,(uint8_t*) &vol_ch_ctrl->vol_ctrl_r, 2, 1000);	
}

void myNecDecodedCallback(uint16_t address, uint8_t cmd) 
{
	int32_t i=0;
	int32_t inc=0;
	uint8_t dec=0;//декремент

	ch_vol.adr_pot=DG_POT_ADR;
	ch_vol.vol_ctrl_l.ctrl=0x98;
	ch_vol.vol_ctrl_r.ctrl=0x18;
	
	if (address==0xFF00)
	{
		switch (cmd)
		{
			case BT_PROG:
			{
				HAL_GPIO_TogglePin(GPIOB, nSHDN_Pin);
			}return;
			case BT_P10: inc=10;break;
			case BT_N10: inc=-10;break;
			case BT_P1:	 inc=1;break;
			case BT_N1:	 inc=-1;break;
			default:;
		}
	}
//----------------------управление громкостью-------------------
	if 			(ch_vol.vol+inc>=255)  ch_vol.vol=255;
	else if (ch_vol.vol+inc<0)		 ch_vol.vol=0;
	else 
	{
		if (inc<0)
		{
			inc=inc*(-1);
			dec=1;
		}
		HAL_GPIO_WritePin(GPIOB, nSHDN_Pin, GPIO_PIN_SET);
		for (i=0;i<inc;i++)
		{
			if (dec)ch_vol.vol--;
			else 		ch_vol.vol++;
			ch_set_vol_all(&ch_vol);
			HAL_Delay (1);
		}
	}
	if (ch_vol.vol==0)HAL_GPIO_WritePin(GPIOB, nSHDN_Pin, GPIO_PIN_RESET);
//-------------------------------------------------------------	
}

void myNecRepeatCallback() 
{
	Nec.NEC_DecodedCallback(Nec.addr,Nec.cmd);
}

/* USER CODE END 0 */

int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
	MX_TIM3_Init();
  MX_TIM8_Init();

  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  for (i=0;i<BUFF_LEN;i++)Nec._len_pulse_raw[i]=0;

	i=0;
	Nec.type=EXTENDED_TYPE;
	Nec.start_seq_time=12000;
	Nec.bit_time=1500;
	Nec._cnt_repeat=0;
	
	Nec.NEC_DecodedCallback=myNecDecodedCallback;
	Nec.NEC_RepeatCallback=myNecRepeatCallback;
	
	tim_icc_dma_burst_link(TIM1, DMA2_Stream6, 1, 0, Nec._len_pulse_raw, BUFF_LEN);
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	SET_BIT(TIM1->CR1, TIM_CR1_URS);//Update прерывание только по переполнению
	//SET_BIT(TIM1->DIER, TIM_DIER_UIE);//разрешить по переполенению
	SET_BIT(TIM1->DIER, TIM_DIER_CC1IE);//разрешаю прерывание по захвату
	
	uint8_t data_l[2]={0x18,0};
	uint8_t data_r[2]={0x98,0};
	HAL_GPIO_WritePin(GPIOB, nSHDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, nSHDN_Pin, GPIO_PIN_SET);
	HAL_I2C_Master_Transmit(&hi2c1, DG_POT_ADR, data_l, 2, 1000);
	HAL_I2C_Master_Transmit(&hi2c1, DG_POT_ADR, data_r, 2, 1000);

  osKernelStart();
  while (1){}
}


/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t dc=0;
	uint32_t d=0;
  /* Infinite loop */
  for(;;)
  {
		if 			(dc==1000)d=1;
		else if (dc==0)	 d=0;
		if (d)dc--;
		else  dc++;
		
		TIM3->CCR2=dc;
		HAL_GPIO_TogglePin(GPIOD, LED3_Pin);
    osDelay(10);
  }
  /* USER CODE END 5 */ 
}


/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
