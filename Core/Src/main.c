/* USER CODE BEGIN Header */
/**
 * @brief            : ADC + MIMU + PPG 双模式采集程序 (支持心率/血氧切换)
 * @details          :
 * 1. 通过修改 CURRENT_WORK_MODE 宏定义切换工作模式
 * 2. MAX30101使用FIFO异步采样模式，主循环批量读取并计算均值
 * 3. 心率模式：绿光单LED，血氧模式：红光+红外双LED
 * 4. 统一21字节数据包格式，支持温度补偿
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "ADC_ADS124.h"
#include "MIMU.h"
#include "MAX30101.h"
#include "hr_algorithm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 255

// PPG 数据起始索引 (2头 + 8ADC + 3ACC = 13)
#define PPG_START_INDEX 13
// 温度数据起始索引 (2头 + 8ADC + 3ACC + 4PPG = 17)
#define TEMP_START_INDEX 17
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 接收相关变量 */
uint8_t rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_len = 0;
volatile uint8_t recv_end_flag = 0;

/* ----------------- 外部关联变量 ----------------- */
uint8_t ADC_1to4Voltage_flag = 0; // ADC采集状态机
uint8_t ACC_XYZ[6]  = {0};         // MIMU 原始数据
uint8_t GYRO_XYZ[6] = {0};
uint8_t MAG_XYZ[6]  = {0};
uint8_t DIN[4] = {0X12, 0, 0, 0};
uint8_t DOUT[4] = {0, 0, 0, 0};
/* ------------------------------------------------ */

/* 发送缓冲区 */
uint8_t allData[50] = {0};

/* ADC 相关 */
uint8_t Utop_times1 = 0;
uint8_t Utop_times2 = 5;

/* PPG 相关变量 */
#if (CURRENT_WORK_MODE == MODE_SPO2)
// 血氧模式变量
static uint32_t last_red_avg = 0;
static uint32_t last_ir_avg = 0;
#else
// 心率模式变量
static uint32_t last_avg_green = 0;
#endif

/* 温度补偿相关 - 1Hz非阻塞测温状态机 */
static uint8_t die_temp_int = 0;
static uint8_t die_temp_frac = 0;

/* --- 在线心率算法相关 --- */
static HR_Config_t hr_config;
static HR_State_t  hr_state;
static uint8_t algorithm_initialized = 0;

// 校验函数
uint8_t CheckXOR(uint8_t *Buf, uint8_t Len);

// PPG配置函数
static void PPG_Config_SpO2_Hardcoded(void);
static void PPG_Config_Green_Hardcoded(void);

// 蓝牙初始化函数
static void BLE_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  uint8_t ADCbuff[] = "ADC ERROR!";
  uint8_t MIMUbuff[] = "MIMU ERROR!";
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  // 2. 初始化 MSP (通常由 HAL 自动调用，这里显式调用也可)
  HAL_UART_MspInit(&huart2);
  HAL_SPI_MspInit(&hspi1);
  HAL_SPI_MspInit(&hspi2);

  // 3. 开启传感器电源 (在 SPI 初始化之后再上电，避免引脚干扰)
  HAL_GPIO_WritePin(V5_0_CE_GPIO_Port, V5_0_CE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(V1_8_CE_GPIO_Port, V1_8_CE_Pin, GPIO_PIN_SET);

  // 等待电源稳定，发送调试信息
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: Power On (5V & 1.8V)...\r\n", 32, 1000);
  HAL_Delay(500);

  /* ====================================================================
   * 蓝牙模块初始化
   * ==================================================================== */
//  BLE_Init();
//  HAL_Delay(200);
//  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: BLE Init OK\r\n", 21, 1000);

  /* ====================================================================
   * 模块检测阶段 (Sensor Check Phase)
   * ==================================================================== */

  /* 1. MAX30101 检测 */
  if (MAX_Check() != 0) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: MAX30101 Found!\r\n", 24, 100);
      HAL_Delay(100);
  } else {
      HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR: MAX30101 Check Failed!\r\n", 31, 100);
  }

  /* 2. ADC 检测 (阻塞式) */
  while (!ADC_check()) {
      HAL_UART_Transmit(&huart2, ADCbuff, sizeof(ADCbuff), 100);
      HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
      HAL_Delay(500);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: ADC Found!\r\n", 19, 100);
  HAL_Delay(100);

  /* 3. MIMU 检测 (阻塞式) */
  while (!MIMU_check()) {
      HAL_UART_Transmit(&huart2, MIMUbuff, sizeof(MIMUbuff), 100);
      HAL_Delay(500);
  }
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: MIMU Found!\r\n", 20, 100);
  HAL_Delay(100);

  /* ====================================================================
   * 模块初始化配置 (Sensor Init Phase)
   * ==================================================================== */

  /* 1. ADC 初始化 */
  ADC_Init();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: ADC Init OK\r\n", 20, 1000);
  HAL_Delay(200);

  /* 2. MIMU 初始化 */
  MIMU_Init();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: MIMU Init OK\r\n", 21, 1000);
  HAL_Delay(200);

  /* 3. MAX30101 初始化 */
  MAX30101_Init();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: PPG Init OK\r\n", 20, 1000);
  HAL_Delay(200);

  /* 4. 根据当前模式配置 PPG 参数 */
#if (CURRENT_WORK_MODE == MODE_SPO2)
  PPG_Config_SpO2_Hardcoded();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: PPG SpO2 Mode Config.\r\n", 30, 100);
#else
  PPG_Config_Green_Hardcoded();
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: PPG HR Mode Config.\r\n", 28, 100);
#endif

  /* ====================================================================
   * 心率在线算法初始化
   * ==================================================================== */
  HR_GetDefaultConfig(&hr_config);
  HR_Init(&hr_config, &hr_state);
  algorithm_initialized = 1;
  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: HR Algorithm Init OK\r\n", 29, 1000);

  /* ====================================================================
   * 系统准备就绪
   * ==================================================================== */

  /* 初始化帧头帧尾 */
  allData[0] = 0xAA;
  allData[1] = 0xBB;

  /* 4. 启动定时器 (最后一步开启中断) */
  ADC_1to4Voltage_flag = 0;

  HAL_TIM_Base_Start_IT(&htim16); // ADC + MIMU tick

  HAL_UART_Transmit(&huart2, (uint8_t*)"DEBUG: System Start Loop...\r\n", 29, 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ---- 心率算法: 每秒执行一次解算 ---- */
    if (algorithm_initialized && hr_state.flag_1s_ready) {
        float bpm = HR_RunSolver(&hr_state);
        if (bpm > 0) {
            /* 构建心率结果数据包 */
            uint8_t hr_packet[22];
            hr_packet[0] = 0xAA;       /* 帧头 */
            hr_packet[1] = 0xCC;

            /* 融合心率 (x10, 0.1 BPM 精度) */
            uint16_t hr_x10 = (uint16_t)(bpm * 10.0f + 0.5f);
            hr_packet[2] = (hr_x10 >> 8) & 0xFF;
            hr_packet[3] = hr_x10 & 0xFF;

            /* 运动标志 */
            hr_packet[4] = hr_state.is_motion;

            /* 窗口填充状态 */
            hr_packet[5] = hr_state.win_filled;

            /* LMS-HF 路径 BPM (x10) */
            uint16_t hf_x10 = (uint16_t)(hr_state.hr_lms_hf * 600.0f + 0.5f);
            hr_packet[6] = (hf_x10 >> 8) & 0xFF;
            hr_packet[7] = hf_x10 & 0xFF;

            /* LMS-ACC 路径 BPM (x10) */
            uint16_t acc_x10 = (uint16_t)(hr_state.hr_lms_acc * 600.0f + 0.5f);
            hr_packet[8] = (acc_x10 >> 8) & 0xFF;
            hr_packet[9] = acc_x10 & 0xFF;

            /* FFT 路径 BPM (x10) */
            uint16_t fft_x10 = (uint16_t)(hr_state.hr_fft * 600.0f + 0.5f);
            hr_packet[10] = (fft_x10 >> 8) & 0xFF;
            hr_packet[11] = fft_x10 & 0xFF;

            /* PPG 信号均值 (窗口第一个通道的平均值, 用于信号强度) */
            /* 使用 buf_1s_ppg 的均值作为信号强度参考 */
            float ppg_mean = 0;
            for (int i = 0; i < HR_STEP_SAMPLES; i++) ppg_mean += hr_state.buf_1s_ppg[i];
            ppg_mean /= HR_STEP_SAMPLES;
            uint16_t ppg_u16 = (uint16_t)(ppg_mean + 0.5f);
            hr_packet[12] = (ppg_u16 >> 8) & 0xFF;
            hr_packet[13] = ppg_u16 & 0xFF;

            /* ACC 幅值 std (x100) */
            /* 运动检测中已计算, 这里简化: 直接从 motion threshold 判断 */
            hr_packet[14] = hr_state.motion_calibrated;

            /* 时间戳 (秒, 低16位) */
            static uint16_t ts = 0;
            ts++;
            hr_packet[15] = (ts >> 8) & 0xFF;
            hr_packet[16] = ts & 0xFF;

            /* 校准状态字节 */
            hr_packet[17] = (hr_state.calib_windows_done < 8) ? hr_state.calib_windows_done : 8;

            /* XOR 校验 */
            uint8_t xor_val = 0;
            for (int i = 2; i < 18; i++) xor_val ^= hr_packet[i];
            hr_packet[18] = xor_val;

            /* 帧尾 */
            hr_packet[19] = 0xCC;

            HAL_UART_Transmit_DMA(&huart2, hr_packet, 20);
        }
    }

    /* ---- 数据采集: ADC 完成后触发 ---- */
    if(ADC_1to4Voltage_flag == 4){

      /* --- 1. 打包 ADC (保留原逻辑) --- */
      // 数据已由中断填充在 allData[2] ~ allData[9]

      /* --- 2. 打包 ACC 高位 (保留原逻辑) --- */
      allData[10] = ACC_XYZ[1]; // X High
      allData[11] = ACC_XYZ[3]; // Y High
      allData[12] = ACC_XYZ[5]; // Z High

      /* --- 3. PPG 数据采集 --- */
      uint8_t wr_ptr = MAX_ReadOneByte(FIFO_WR_PTR_REG);
      uint8_t rd_ptr = MAX_ReadOneByte(FIFO_RD_PTR_REG);
      uint8_t sample_count = (wr_ptr - rd_ptr) & 0x1F;

#if (CURRENT_WORK_MODE == MODE_SPO2)
      /* --- 3.1 血氧模式打包逻辑 --- */
      /* --- 3.2 温度补偿状态机 (1Hz, 非阻塞) --- */
      static uint8_t temp_tick = 0;
      temp_tick++;
      if (temp_tick >= 125) {
          temp_tick = 0;
      }
      // 触发阶段：temp_tick == 0 时启动温度转换
      if (temp_tick == 0) {
          MAX_WriteOneByte(DIE_TEMP_CONFIG_REG, 0x01);
      }
      // 读取阶段：temp_tick == 4 时 (32ms后，满足29ms转换时间)
      else if (temp_tick == 4) {
          die_temp_int = MAX_ReadOneByte(DIE_TEMP_INT_REG);
          die_temp_frac = MAX_ReadOneByte(DIE_TEMP_FRAC_REG);
      }

      uint32_t sum_red = 0, sum_ir = 0;
      uint8_t buf[6];  // 每个样本6字节（3字节Red + 3字节IR）

      for (uint8_t i = 0; i < sample_count; i++) {
          MAX_ReadFIFO_Burst(buf, 6);
          uint32_t red_val = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x03FFFF;
          uint32_t ir_val = ((buf[3] << 16) | (buf[4] << 8) | buf[5]) & 0x03FFFF;
          sum_red += red_val;
          sum_ir += ir_val;
      }

      // 计算均值并转换为16-bit
      uint32_t red_avg, ir_avg;
      if (sample_count > 0) {
          red_avg = sum_red / sample_count;
          ir_avg = sum_ir / sample_count;
          red_avg >>= 2;
          ir_avg >>= 2;
          last_red_avg = red_avg;
          last_ir_avg = ir_avg;
      } else {
          red_avg = last_red_avg;
          ir_avg = last_ir_avg;
      }

      allData[PPG_START_INDEX + 0] = (red_avg >> 8) & 0xff;
      allData[PPG_START_INDEX + 1] = red_avg & 0xff;
      allData[PPG_START_INDEX + 2] = (ir_avg >> 8) & 0xff;
      allData[PPG_START_INDEX + 3] = ir_avg & 0xff;

      // 扩展位：温度数据
      allData[TEMP_START_INDEX] = die_temp_int;
      allData[TEMP_START_INDEX + 1] = die_temp_frac;

#else
      /* --- 3.3 心率模式: 计算 PPG 均值用于算法 + 保留原始数据包 --- */
      uint32_t sum_green = 0;
      uint8_t buf[3];

      for (uint8_t i = 0; i < sample_count; i++) {
          MAX_ReadFIFO_Burst(buf, 3);
          sum_green += ((buf[0] << 16) | (buf[1] << 8) | buf[2]) & 0x03FFFF;
      }

      /* 原始数据包: 保留 sum + count 格式 */
      allData[PPG_START_INDEX]     = (sum_green >> 16) & 0xFF;
      allData[PPG_START_INDEX + 1] = (sum_green >> 8)  & 0xFF;
      allData[PPG_START_INDEX + 2] =  sum_green        & 0xFF;
      allData[PPG_START_INDEX + 3] = sample_count;

      /* 算法数据: 推送 float 采样点 */
      if (algorithm_initialized && sample_count > 0) {
          float ppg_val = (float)sum_green / (float)sample_count;

          /* ACC 完整 16 位数据 */
          int16_t ax_raw = (int16_t)((ACC_XYZ[1] << 8) | ACC_XYZ[0]);
          int16_t ay_raw = (int16_t)((ACC_XYZ[3] << 8) | ACC_XYZ[2]);
          int16_t az_raw = (int16_t)((ACC_XYZ[5] << 8) | ACC_XYZ[4]);

          /* HF/ADC: 使用第一个 ADC 通道 (allData[8..9] = 桥中1) */
          int16_t hf_raw = (int16_t)((allData[8] << 8) | allData[9]);

          HR_PushSample(&hr_state, ppg_val,
                        (float)ax_raw, (float)ay_raw, (float)az_raw,
                        (float)hf_raw);
      }

      allData[TEMP_START_INDEX] = 0x00;
      allData[TEMP_START_INDEX + 1] = 0xFF;
#endif

      /* --- 4. 计算校验位 (校验从 allData[2] 开始的 17 个字节) --- */
      allData[19] = CheckXOR(&allData[2], XOR_CHECK_LEN);

      /* --- 5. 填充帧尾 --- */
      allData[20] = 0xCC;

      /* --- 6. DMA 发送 (21 字节) --- */
      HAL_UART_Transmit_DMA(&huart2, allData, PACKET_LEN);

      /* --- 7. 清除标志位 --- */
      ADC_1to4Voltage_flag = 0;
    }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

// 蓝牙初始化
static void BLE_Init(void)
{
    static const uint8_t CMD_WAKE[]     = "<ST_WAKE=FOREVER>";
    static const uint8_t CMD_TX_POWER[] = "<ST_TX POWER=+2.5>";
    static const uint8_t CMD_NAME[]     = "<ST_NAME=HJ-131-LYX-5>";
    static const uint8_t CMD_SECRET[]   = "<ST_SECRET=123456>";
    static const uint8_t CMD_BAUD[]     = "<ST_BAUD=115200>";
    static const uint8_t CMD_MIN_GAP[]  = "<ST_CON_MIN_GAP=75>";

    // 0. 发送唤醒序列 RX 接收缓冲区清零，当接收到 0XAA 判断唤醒 BLE
    uint8_t wakeup_seq[] = {0xAA, 0xAA, 0xAA, 0xAA};
    HAL_UART_Transmit(&huart2, wakeup_seq, sizeof(wakeup_seq), 100);
    HAL_Delay(50);

    // 1. 设置全常开模式（不睡眠）
    HAL_UART_Transmit(&huart2, CMD_WAKE, sizeof(CMD_WAKE)-1, 0xFFFF);
    HAL_Delay(50);

    // 2. 设置发射功率及蓝牙名称
    HAL_UART_Transmit(&huart2, CMD_TX_POWER, sizeof(CMD_TX_POWER)-1, 0xFFFF);
    HAL_Delay(50);
    HAL_UART_Transmit(&huart2, CMD_NAME, sizeof(CMD_NAME)-1, 0xFFFF);
    HAL_Delay(50);

    // 3. 设置配对密码，供 APP 连接权限
    HAL_UART_Transmit(&huart2, CMD_SECRET, sizeof(CMD_SECRET)-1, 0xFFFF);
    HAL_Delay(50);

    // 4. 发送波特率切换指令 (此时 MCU 仍然是 19200bps)
    HAL_UART_Transmit(&huart2, CMD_BAUD, sizeof(CMD_BAUD)-1, 0xFFFF);
    HAL_Delay(50);

    // 5. MCU 切换到新的波特率
    huart2.Init.BaudRate = 115200;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_Delay(10);

    // 6. 用新的 115200bps 发送后续指令
    HAL_UART_Transmit(&huart2, CMD_MIN_GAP, sizeof(CMD_MIN_GAP)-1, 0xFFFF);
    HAL_Delay(50);
}

// 校验函数
uint8_t CheckXOR(uint8_t *Buf, uint8_t Len)
{
  uint8_t i = 0;
  uint8_t x = 0;
  for(i = 0; i < Len; i++)
  {
    x = x ^ (*(Buf + i));
  }
  return x;
}

// PPG 血氧模式硬编码配置
static void PPG_Config_SpO2_Hardcoded(void)
{
    // --- 1. 工作模式配置 (血氧模式) ---
    MAX_WriteOneByte(MODE_CONFIG_REG, 0x03);

    // --- 2. LED 亮度配置 (Red和IR) ---
    MAX_WriteOneByte(LED1_PA_REG, 0x91);  // Red LED 电流 ~29mA
    MAX_WriteOneByte(LED2_PA_REG, 0x91);  // IR LED 电流 ~29mA

    // --- 3. SPO2/ADC/采样率/脉宽 配置 ---
    // ADC_RGE=16384nA(0x60) | SR=400sps(0x0C) | PW=411us/18-bit(0x03)
    // 组合值：0x6F
    MAX_WriteOneByte(SPO2_CONFIG_REG, 0x6F);

    // --- 4. FIFO 配置 ---
    // SMP_AVE=不平均(0x00) | FIFO_ROLLOVER_EN(0x10) | FIFO_A_FULL=15(0x0F)
    // 组合值：0x1F
    MAX_WriteOneByte(FIFO_CONFIG_REG, 0x1F);

    // --- 5. 清除 FIFO 指针/计数器 ---
    MAX_WriteOneByte(FIFO_WR_PTR_REG, 0x00);
    MAX_WriteOneByte(OVF_COUNTER_REG, 0x00);
    MAX_WriteOneByte(FIFO_RD_PTR_REG, 0x00);
}

// PPG 绿光心率模式硬编码配置
static void PPG_Config_Green_Hardcoded(void)
{
    // --- 1. Mode Configuration (0x09) = 0x07: Multi-LED 模式 ---
    // (HR模式只能用绿光，必须使用Multi-LED模式)
    MAX_WriteOneByte(MODE_CONFIG_REG, 0x07);

    // --- 2. LED_CONTROL1 (0x11) = 0x03: SLOT1=LED3(Green), SLOT2关闭 ---
    MAX_WriteOneByte(LED_CONTROL1, 0x03);

    // --- 3. LED_CONTROL2 (0x12) = 0x00: 关闭 SLOT3 和 SLOT4 ---
    MAX_WriteOneByte(LED_CONTROL2, 0x00);

    // --- 4. LED3_PA_REG (0x0E) = 0x5F: 绿光亮度约 19mA ---
    MAX_WriteOneByte(LED3_PA_REG, 0x71);

    // --- 5. SPO2_CONFIG_REG (0x0A) = 0x77 ---
    // Bit7-5: 011 = 16384nA 满量程
    // Bit4-2: 111 = 1000sps 内部采样频率
    // Bit1-0: 11 = 411us/18-bit 最长脉宽
    MAX_WriteOneByte(SPO2_CONFIG_REG, 0x77);

    // --- 6. FIFO_CONFIG_REG (0x08) = 0x5F ---
    // Bit6: 1 (Sample Average = 4倍硬件平均)
    // Bit5: 1 (使能)
    // Bit4: 1 (FIFO Rollover 使能)
    // Bit0-3: 1111 (FIFO_A_FULL = 15)
    MAX_WriteOneByte(FIFO_CONFIG_REG, 0x5F);

    // --- 7. 清除 FIFO 指针 ---
    MAX_WriteOneByte(FIFO_WR_PTR_REG, 0x00);
    MAX_WriteOneByte(OVF_COUNTER_REG, 0x00);
    MAX_WriteOneByte(FIFO_RD_PTR_REG, 0x00);
}

/* ==============================================================================
 * 中断回调函数
 * ============================================================================== */

// 定时器中断处理
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // TIM16: 触发 ADC 采集 (固定频率)
    if (htim == (&htim16)){
        HAL_GPIO_WritePin(CS_A_G_GPIO_Port, CS_A_G_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
    }
}

// GPIO外部中断：读取 ADC 和 MIMU 数据
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == DRDY_Pin){
        HAL_GPIO_WritePin(GPIOA, SPI1_NSS_Pin, GPIO_PIN_RESET);

        if(ADC_1to4Voltage_flag == 0){
            if(Utop_times1 != 10){
                if(Utop_times2 == 10) ADC_1to4Voltage_flag = 1;
                else                  ADC_1to4Voltage_flag = 2;
            }
        }

        if(ADC_1to4Voltage_flag != 4){
            switch(ADC_1to4Voltage_flag){
                case 0:
                    ADC_RDATA(&allData[8]);  // 填入桥中1
                    if(Utop_times2 == 10){
                        ADC_WREG(ADC_MUX_REG, 0X3C);
                        ADC_1to4Voltage_flag = 1;
                    } else {
                        ADC_WREG(ADC_MUX_REG, 0X0C);
                        ADC_1to4Voltage_flag = 2;
                    }
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
                    Utop_times1 = 0;
                    break;
                case 1:
                    ADC_RDATA(&allData[6]);  // 填入桥中2
                    ADC_WREG(ADC_MUX_REG, 0X0C);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
                    Utop_times2 = 0;
                    ADC_1to4Voltage_flag = 2;
                    break;
                case 2:
                    ADC_RDATA(&allData[4]);  // 填入桥顶1
                    ADC_1to4Voltage_flag = 3;
                    ADC_WREG(ADC_MUX_REG, 0X2C);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, START_CONV_Pin, GPIO_PIN_RESET);
                    break;
                case 3:
                    ADC_RDATA(&allData[2]);  // 填入桥顶2
                    ADC_1to4Voltage_flag = 4; // 标志一轮 ADC 完成！(触发 Main 发送)
                    Utop_times1++;
                    Utop_times2++;
                    if(Utop_times1 == 10)      ADC_WREG(ADC_MUX_REG, 0X1C);
                    else if(Utop_times2 == 10) ADC_WREG(ADC_MUX_REG, 0X3C);
                    else                       ADC_WREG(ADC_MUX_REG, 0X0C);
                    break;
                default:
                    ADC_1to4Voltage_flag = 5;
            }
        }

        // 读取 MIMU 数据
        if(ADC_1to4Voltage_flag == 4)
            ACC_6BytesRead();
    }
}

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

#ifdef  USE_FULL_ASSERT
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

