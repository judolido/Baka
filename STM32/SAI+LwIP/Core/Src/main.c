
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c (LwIP UDP audio sender)
  * @brief          : Main program body
  ******************************************************************************
  * NOTE: This file was adapted to send audio via UDP using LwIP.
  * - SAI + DMA captures 24-bit audio in 32-bit slots.
  * - An ISR pushes raw samples into a lock-free ring buffer.
  * - The main loop pumps LwIP and sends UDP packets of 480 samples
  *   (1440-byte payload) with a small 8-byte header.
  *
  * Packet format (little-endian):
  *   [0]   0xAA
  *   [1]   0x55
  *   [2]   seq (u8)
  *   [3]   flags (u8, reserved=0)
  *   [4:7] timestamp/sample_counter (u32, increases by 480 per packet)
  *   [8:]  480 * PCM24 {LSB, Mid, MSB}
  *
  * Set DEST_IP_STR below to your PC's IPv4 address (or a subnet broadcast).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "lwip.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AUDIO_BUFFER_SIZE     2048   // DMA buffer words (32-bit per sample)

// ---- LwIP/UDP config ----
#define DEST_IP_STR           "192.168.1.100"   // <-- set to your PC's IP or "255.255.255.255" for broadcast
#define DEST_UDP_PORT         5004
#define SAMPLES_PER_PKT       480               // 480 * 3 = 1440B payload fits under 1472B UDP MTU
#define UDP_HDR_SIZE          8
#define UDP_PAYLOAD_SIZE      (SAMPLES_PER_PKT*3)
#define UDP_PKT_SIZE          (UDP_HDR_SIZE + UDP_PAYLOAD_SIZE)

// ---- Audio ring buffer (32-bit words from SAI) ----
#define RING_ORDER            14                // 2^14 = 16384 samples (~64KB)
#define RING_SIZE             (1U << RING_ORDER)
#define RING_MASK             (RING_SIZE - 1U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai2_b;

/* USER CODE BEGIN PV */

// DMA buffer for SAI
uint32_t sai_dma_buffer[AUDIO_BUFFER_SIZE];

// Audio ring written in ISR, read in main loop
static volatile uint32_t audio_ring[RING_SIZE];
static volatile uint32_t ring_wr = 0;
static volatile uint32_t ring_rd = 0;

// UDP state
static struct udp_pcb *g_udp = NULL;
static ip_addr_t g_dest_ip;
static uint8_t  g_seq = 0;
static uint32_t g_sample_counter = 0;
static uint8_t  g_udp_buf[UDP_PKT_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI2_Init(void);

/* USER CODE BEGIN PFP */
static void Net_Init(void);
static void UDP_Send_Pump(void);
static inline void push_samples_to_ring(const uint32_t *src, uint16_t count);
static inline void pack24le(uint8_t *dst3, uint32_t w);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern struct netif gnetif; // declared by CubeMX in lwip.c

static inline void pack24le(uint8_t *dst3, uint32_t w)
{
  // SAI gives 24-bit signed left-justified in 32-bit slot
  // Keep sign, shift down 8, then write little-endian 24-bit
  int32_t s = (int32_t)w;
  uint32_t v = ((uint32_t)s) >> 8;
  dst3[0] = (uint8_t)(v & 0xFF);
  dst3[1] = (uint8_t)((v >> 8) & 0xFF);
  dst3[2] = (uint8_t)((v >> 16) & 0xFF);
}

static inline void push_samples_to_ring(const uint32_t *src, uint16_t count)
{
  for (uint16_t i = 0; i < count; ++i) {
    audio_ring[ring_wr & RING_MASK] = src[i];
    ring_wr++;
    // Optional: drop oldest on overflow (keep ring within 1*RING_SIZE window)
    if ((ring_wr - ring_rd) > RING_SIZE) {
      ring_rd = ring_wr - RING_SIZE;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI2_Init();

  /* USER CODE BEGIN 2 */
  // Start audio capture
  HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t *)sai_dma_buffer, 2 * AUDIO_BUFFER_SIZE);

  // Bring up LwIP/Ethernet and UDP socket
  Net_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    // Run LwIP background tasks (NO_SYS=1)
    MX_LWIP_Process();

    // Send as many UDP packets as available in the ring
    UDP_Send_Pump();

    // Optional: small sleep to reduce CPU usage
    // HAL_Delay(1);
  }
}

/**
  * @brief Bring up LwIP and connect UDP PCB
  */
static void Net_Init(void)
{
  MX_LWIP_Init();

  // Wait until netif is up and has a valid IPv4
  while (!netif_is_up(&gnetif) || ip4_addr_isany_val(*netif_ip4_addr(&gnetif))) {
    MX_LWIP_Process();
    HAL_Delay(10);
  }

  g_udp = udp_new();
  if (!g_udp) {
    Error_Handler();
  }

  // Destination IP
  if (!ipaddr_aton(DEST_IP_STR, &g_dest_ip)) {
    Error_Handler();
  }

  udp_connect(g_udp, &g_dest_ip, DEST_UDP_PORT);

  // If you set DEST_IP_STR to a broadcast address, uncomment:
  // udp_set_flags(g_udp, UDP_FLAGS_BROADCAST);
}

/**
  * @brief Send UDP packets while enough samples exist in the ring
  */
static void UDP_Send_Pump(void)
{
  while ((ring_wr - ring_rd) >= SAMPLES_PER_PKT) {
    // Header
    uint8_t *p = g_udp_buf;
    *p++ = 0xAA;
    *p++ = 0x55;
    *p++ = g_seq++;
    *p++ = 0x00; // flags
    uint32_t ts = g_sample_counter;
    memcpy(p, &ts, sizeof(ts));
    p += 4;

    // Payload: 480 samples -> 1440 bytes
    for (int i = 0; i < SAMPLES_PER_PKT; ++i) {
      uint32_t w = audio_ring[ring_rd & RING_MASK];
      ring_rd++;
      pack24le(p, w);
      p += 3;
    }
    g_sample_counter += SAMPLES_PER_PKT;

    // Send
    struct pbuf *pb = pbuf_alloc(PBUF_TRANSPORT, UDP_PKT_SIZE, PBUF_RAM);
    if (!pb) {
      // Out of buffers—drop this packet (next loop will try again)
      return;
    }
    memcpy(pb->payload, g_udp_buf, UDP_PKT_SIZE);
    udp_send(g_udp, pb);
    pbuf_free(pb);
  }
}

/* SAI DMA callbacks: push half/full buffers into ring */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI2_Block_B) {
    push_samples_to_ring(&sai_dma_buffer[0], AUDIO_BUFFER_SIZE / 2);
  }
}
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI2_Block_B) {
    push_samples_to_ring(&sai_dma_buffer[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE / 2);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{
  hsai_BlockB2.Instance = SAI2_Block_B;
  hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockB2.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockB2.Init.DataSize = SAI_DATASIZE_24;
  hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_22K; // adjust to 32K in CubeMX if desired
  hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockB2.FrameInit.FrameLength = 64;
  hsai_BlockB2.FrameInit.ActiveFrameLength = 32;
  hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockB2.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockB2.SlotInit.FirstBitOffset = 1;
  hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockB2.SlotInit.SlotNumber = 2;
  hsai_BlockB2.SlotInit.SlotActive = 0x00000001;
  if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  // Ethernet pins configured by CubeMX

  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
