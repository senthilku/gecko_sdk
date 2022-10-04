/*******************************************************************************
* # License
* <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
*******************************************************************************
*
* The licensor of this software is Silicon Laboratories Inc. Your use of this
* software is governed by the terms of Silicon Labs Master Software License
* Agreement (MSLA) available at
* www.silabs.com/about-us/legal/master-software-license-agreement. This
* software is distributed to you in Source Code format and is governed by the
* sections of the MSLA applicable to Source Code.
*
******************************************************************************/
/*================================================================================
 * @brief
 * This files contains example for Flash memory read and write
=================================================================================*/

/* Includes ------------------------------------------------------------------*/

#include "rsi_chip.h"
#include "UDMA.h"
#include "rsi_board.h"
//#include "Ecode.h"

//#define FLASH_TEST_WRAPPER 1

//#ifdef FLASH_TEST_WRAPPER //senthil

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
//#define    DMA_READ
#define IO_READ

#ifdef CHIP_9118
#define PadSelectionEnable 15
#define QSPI_MODE          2

/*M4 QSPI  pin set */
#define M4SS_QSPI_CLK  58
#define M4SS_QSPI_CSN0 59
#define M4SS_QSPI_D0   60
#define M4SS_QSPI_D1   61
#define M4SS_QSPI_D2   62
#define M4SS_QSPI_D3   63
#endif

#ifdef CHIP_9117
#define PadSelectionEnable_CLK  16
#define PadSelectionEnable_D0   17
#define PadSelectionEnable_D1   18
#define PadSelectionEnable_CSN0 19
#define PadSelectionEnable_D2   20
#define PadSelectionEnable_D3   21

#define QSPI_MODE 9

/*M4 QSPI  pin set */
#define M4SS_QSPI_CLK  52
#define M4SS_QSPI_D0   53
#define M4SS_QSPI_D1   54
#define M4SS_QSPI_CSN0 55
#define M4SS_QSPI_D2   56
#define M4SS_QSPI_D3   57
#endif

/* Private macro -------------------------------------------------------------*/
#define SINGLE_MODE_ENABLE 1
#define ADDRESS            0x802B000
#define SIZE               256
#define CHNL0              0

/* Private variables ---------------------------------------------------------*/
volatile uint32_t done;
uint32_t read_data[SIZE], data_write[SIZE];

#if defined(__CC_ARM)
extern RSI_UDMA_DESC_T UDMA0_Table[CONTROL_STRUCT0];
extern RSI_UDMA_DESC_T UDMA1_Table[CONTROL_STRUCT1];
#endif /* defined (__CC_ARM) */

#if defined(__GNUC__)
extern RSI_UDMA_DESC_T __attribute__((section(".udma_addr0"))) UDMA0_Table[CONTROL_STRUCT0];
extern RSI_UDMA_DESC_T __attribute__((section(".udma_addr1"))) UDMA1_Table[CONTROL_STRUCT1];
#endif /* defined (__GNUC__) */

extern UDMA_Channel_Info udma0_chnl_info[32];

/* UDMA0 Resources */
extern UDMA_RESOURCES UDMA0_Resources;

/* UDMA1 Resources */
extern UDMA_RESOURCES UDMA1_Resources;

extern RSI_UDMA_HANDLE_T udmaHandle0;

extern uint32_t dma_rom_buff0[30]; //we can keep wrapeers

#define  DEBUG 1

#if DEBUG
#if 1
char _acTestData[256] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
#else
char _acTestData[512] = {
  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
  0x04, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
  0x08, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00,
  0x0C, 0x00, 0x00, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00,
  0x10, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00,
  0x14, 0x00, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00,
  0x18, 0x00, 0x00, 0x00, 0x19, 0x00, 0x00, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x00,
  0x1C, 0x00, 0x00, 0x00, 0x1D, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00,
  0x20, 0x00, 0x00, 0x00, 0x21, 0x00, 0x00, 0x00, 0x22, 0x00, 0x00, 0x00, 0x23, 0x00, 0x00, 0x00,
  0x24, 0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x27, 0x00, 0x00, 0x00,
  0x28, 0x00, 0x00, 0x00, 0x29, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x2B, 0x00, 0x00, 0x00,
  0x2C, 0x00, 0x00, 0x00, 0x2D, 0x00, 0x00, 0x00, 0x2E, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00,
  0x30, 0x00, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00,
  0x34, 0x00, 0x00, 0x00, 0x35, 0x00, 0x00, 0x00, 0x36, 0x00, 0x00, 0x00, 0x37, 0x00, 0x00, 0x00,
  0x38, 0x00, 0x00, 0x00, 0x39, 0x00, 0x00, 0x00, 0x3A, 0x00, 0x00, 0x00, 0x3B, 0x00, 0x00, 0x00,
  0x3C, 0x00, 0x00, 0x00, 0x3D, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00,
  0x40, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x43, 0x00, 0x00, 0x00,
  0x44, 0x00, 0x00, 0x00, 0x45, 0x00, 0x00, 0x00, 0x46, 0x00, 0x00, 0x00, 0x47, 0x00, 0x00, 0x00,
  0x48, 0x00, 0x00, 0x00, 0x49, 0x00, 0x00, 0x00, 0x4A, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x00, 0x00,
  0x4C, 0x00, 0x00, 0x00, 0x4D, 0x00, 0x00, 0x00, 0x4E, 0x00, 0x00, 0x00, 0x4F, 0x00, 0x00, 0x00,
  0x50, 0x00, 0x00, 0x00, 0x51, 0x00, 0x00, 0x00, 0x52, 0x00, 0x00, 0x00, 0x53, 0x00, 0x00, 0x00,
  0x54, 0x00, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00, 0x56, 0x00, 0x00, 0x00, 0x57, 0x00, 0x00, 0x00,
  0x58, 0x00, 0x00, 0x00, 0x59, 0x00, 0x00, 0x00, 0x5A, 0x00, 0x00, 0x00, 0x5B, 0x00, 0x00, 0x00,
  0x5C, 0x00, 0x00, 0x00, 0x5D, 0x00, 0x00, 0x00, 0x5E, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00,
  0x60, 0x00, 0x00, 0x00, 0x61, 0x00, 0x00, 0x00, 0x62, 0x00, 0x00, 0x00, 0x63, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x67, 0x00, 0x00, 0x00,
  0x68, 0x00, 0x00, 0x00, 0x69, 0x00, 0x00, 0x00, 0x6A, 0x00, 0x00, 0x00, 0x6B, 0x00, 0x00, 0x00,
  0x6C, 0x00, 0x00, 0x00, 0x6D, 0x00, 0x00, 0x00, 0x6E, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00,
  0x70, 0x00, 0x00, 0x00, 0x71, 0x00, 0x00, 0x00, 0x72, 0x00, 0x00, 0x00, 0x73, 0x00, 0x00, 0x00,
  0x74, 0x00, 0x00, 0x00, 0x75, 0x00, 0x00, 0x00, 0x76, 0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00,
  0x78, 0x00, 0x00, 0x00, 0x79, 0x00, 0x00, 0x00, 0x7A, 0x00, 0x00, 0x00, 0x7B, 0x00, 0x00, 0x00,
  0x7C, 0x00, 0x00, 0x00, 0x7D, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00
};
#endif
#endif

void Genrate_sample_data(void)
{
  int i;
  for (i = 0; i <= SIZE; i++) {
    data_write[i] = i;
  }
}

#define _FLASH_BASE_ADDR (0x802B000) //(0x08000000)

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       main
*
*  Function description
*    Main function. Performs some sample operations with
*    the RAMCode for verification.
*/

int nvm3_hal_wrapper(void);

int nvm3_hal_wrapper(void) {

#if 1
	int res = -1;
       unsigned int err = 0;

	/* Genrates data buffer */
  Genrate_sample_data();

#if 0
	Test_NVM3_Usage_Main();
#endif

	//Ecode_t status;
	err = nvm3_initDefault();

	
	nvm3_WriteConfigValue();
	nvm3_ReadConfigValue();
	
	Init(0, 0, 0);
	res = EraseSector(_FLASH_BASE_ADDR);
	res = ProgramPage(_FLASH_BASE_ADDR, sizeof(data_write), (char *)data_write);
	res = ReadFlash(_FLASH_BASE_ADDR, sizeof(read_data), (char *)read_data);

	res = EraseSector(_FLASH_BASE_ADDR);
	res = ProgramPage(_FLASH_BASE_ADDR, 10, (char *)data_write);
	res = EraseSector(_FLASH_BASE_ADDR);
	res = ProgramPage(_FLASH_BASE_ADDR, 10, (char *)_acTestData);
	//res = ReadFlash(_FLASH_BASE_ADDR, sizeof(_acTestData), (char *)_acTestData);
	
	return 1;
#else	
  int r; 
  Init(0, 0, 0);
  r = EraseSector(_FLASH_BASE_ADDR);
  if (r != 0) {  // Error?
    while (1);
  }
  UnInit(0);
  //
  // Program page
  //
  Init(0, 0, 0);
  r = ProgramPage(_FLASH_BASE_ADDR, sizeof(_acTestData), (char *)_acTestData);
  if (r != 0) {  // Error?
    while (1);
  }
  UnInit(0);
  return r;
#endif
}

//#else //Spi working code

/* Private functions ---------------------------------------------------------*/

/**
 * @brief      UDMA  controller transfer descriptor chain complete callback 
 * @param[in]  event dma transfer events
 * @param[in]  ch    dma channel number
 * @return     None
 */
void udmaTransferComplete(uint32_t event, uint8_t ch)
{
  if (event == UDMA_EVENT_XFER_DONE) {
    if (ch == 0) {
      done = 1;
    }
  }
}
/**
 * @brief  This function configures DMA for read operation.
 * @param  None
 * @retval None
 */
void UDMA_Read(void)
{
  RSI_UDMA_CHA_CONFIG_DATA_T control;
  RSI_UDMA_CHA_CFG_T config;

  memset(&control, 0, sizeof(RSI_UDMA_CHA_CONFIG_DATA_T));
  memset(&config, 0, sizeof(RSI_UDMA_CHA_CFG_T));

  config.altStruct       = 0;
  config.burstReq        = 1;
  config.channelPrioHigh = 0;
  config.dmaCh           = CHNL0;
  config.periAck         = 0;
  config.periphReq       = 0;
  config.reqMask         = 0;

  control.transferType       = UDMA_MODE_BASIC;
  control.nextBurst          = 0;
  control.totalNumOfDMATrans = 255;
  control.rPower             = ARBSIZE_256;
  control.srcProtCtrl        = 0x000;
  control.dstProtCtrl        = 0x000;
  control.srcSize            = SRC_SIZE_32;
  control.srcInc             = SRC_INC_32;
  control.dstSize            = DST_SIZE_32;
  control.dstInc             = DST_INC_32;

  /* Initialize dma */
  udmaHandle0 = UDMAx_Initialize(&UDMA0_Resources, UDMA0_Table, udmaHandle0, dma_rom_buff0);

  UDMAx_ChannelConfigure(&UDMA0_Resources,
                         CHNL0,
                         ADDRESS,
                         (uint32_t)read_data,
                         SIZE,
                         control,
                         &config,
                         udmaTransferComplete,
                         udma0_chnl_info,
                         udmaHandle0);

  /* Enable dma channel */
  UDMAx_ChannelEnable(CHNL0, &UDMA0_Resources, udmaHandle0);

  /* Enable dma controller */
  UDMAx_DMAEnable(&UDMA0_Resources, udmaHandle0);

  RSI_UDMA_ChannelSoftwareTrigger(udmaHandle0, CHNL0);
}

#if 0 //senthil
/**
 * @brief  This function fills QSPI configuration structures.
 * @param  spi_config : structures to configure qspi
 * @retval None
 */
void GetQspiConfig(spi_config_t *spi_config)
{
  memset(spi_config, 0, sizeof(spi_config_t));

  spi_config->spi_config_1.inst_mode         = SINGLE_MODE;
  spi_config->spi_config_1.addr_mode         = SINGLE_MODE;
  spi_config->spi_config_1.data_mode         = SINGLE_MODE;
  spi_config->spi_config_1.dummy_mode        = SINGLE_MODE;
  spi_config->spi_config_1.extra_byte_mode   = SINGLE_MODE;
  spi_config->spi_config_1.read_cmd          = READ;
  spi_config->spi_config_1.no_of_dummy_bytes = 0;
  spi_config->spi_config_1.prefetch_en       = DIS_PREFETCH;
  spi_config->spi_config_1.dummy_W_or_R      = DUMMY_READS;
  spi_config->spi_config_1.extra_byte_en     = 0;
  spi_config->spi_config_1.d3d2_data         = 3;
  spi_config->spi_config_1.continuous        = DIS_CONTINUOUS;
  spi_config->spi_config_1.flash_type        = MX_QUAD_FLASH;

  spi_config->spi_config_2.auto_mode                   = EN_AUTO_MODE;
  spi_config->spi_config_2.cs_no                       = CHIP_ZERO;
  spi_config->spi_config_2.neg_edge_sampling           = NEG_EDGE_SAMPLING;
  spi_config->spi_config_2.qspi_clk_en                 = QSPI_FULL_TIME_CLK;
  spi_config->spi_config_2.protection                  = DNT_REM_WR_PROT;
  spi_config->spi_config_2.dma_mode                    = NO_DMA;
  spi_config->spi_config_2.swap_en                     = SWAP;
  spi_config->spi_config_2.full_duplex                 = IGNORE_FULL_DUPLEX;
  spi_config->spi_config_2.wrap_len_in_bytes           = NO_WRAP;
  spi_config->spi_config_2.addr_width_valid            = 0;
  spi_config->spi_config_2.addr_width                  = _24BIT_ADDR;
  spi_config->spi_config_2.pinset_valid                = 0;
  spi_config->spi_config_2.flash_pinset                = GPIO_58_TO_63;
  spi_config->spi_config_2.dummy_cycles_for_controller = 0;

  spi_config->spi_config_3.wr_cmd            = 0x2;
  spi_config->spi_config_3.wr_inst_mode      = SINGLE_MODE;
  spi_config->spi_config_3.wr_addr_mode      = SINGLE_MODE;
  spi_config->spi_config_3.wr_data_mode      = SINGLE_MODE;
  spi_config->spi_config_3.xip_mode          = 0;
  spi_config->spi_config_3._16bit_cmd_valid  = 0;
  spi_config->spi_config_3._16bit_rd_cmd_msb = 0;
#ifdef CHIP_9118
  spi_config->spi_config_3.ddr_mode_en = 0;
#endif
  spi_config->spi_config_3.dummys_4_jump = 1;

  spi_config->spi_config_4._16bit_wr_cmd_msb = 0;
  spi_config->spi_config_4.high_perf_mode_en = 0;
#ifdef CHIP_9118
  spi_config->spi_config_4.qspi_manual_ddr_phasse = 0;
  spi_config->spi_config_4.ddr_data_mode          = 0;
  spi_config->spi_config_4.ddr_addr_mode          = 0;
  spi_config->spi_config_4.ddr_inst_mode          = 0;
  spi_config->spi_config_4.ddr_dummy_mode         = 0;
  spi_config->spi_config_4.ddr_extra_byte         = 0;
#endif
  spi_config->spi_config_4.dual_flash_mode      = 0;
  spi_config->spi_config_4.secondary_csn        = 1;
  spi_config->spi_config_4.polarity_mode        = 0;
  spi_config->spi_config_4.valid_prot_bits      = 4;
  spi_config->spi_config_4.no_of_ms_dummy_bytes = 0;
#ifdef CHIP_9118
  spi_config->spi_config_4.ddr_dll_en = 0;
#endif
  spi_config->spi_config_4.continue_fetch_en = 0;

  spi_config->spi_config_5.block_erase_cmd      = BLOCK_ERASE;
  spi_config->spi_config_5.busy_bit_pos         = 0;
  spi_config->spi_config_5.d7_d4_data           = 0xf;
  spi_config->spi_config_5.dummy_bytes_for_rdsr = 0x0;
  spi_config->spi_config_5.reset_type           = 0x0;

  spi_config->spi_config_6.chip_erase_cmd   = CHIP_ERASE;
  spi_config->spi_config_6.sector_erase_cmd = SECTOR_ERASE;

  spi_config->spi_config_7.status_reg_write_cmd = 0x1;
  spi_config->spi_config_7.status_reg_read_cmd  = 0x5;
}
#endif
/**
 * @brief	Configures the pin MUX for QSPI interface
 * @return	Nothing
 */

#if 0 //senthil (mul difines)
void RSI_QSPI_PinMuxInit(void)
{
#ifdef CHIP_9118
  /*Pad selection enable */
  RSI_EGPIO_PadSelectionEnable(15);
#endif

#ifdef CHIP_9117
  /*Pad selection enable */
  RSI_EGPIO_PadSelectionEnable(PadSelectionEnable_CLK);
  RSI_EGPIO_PadSelectionEnable(PadSelectionEnable_D0);
  RSI_EGPIO_PadSelectionEnable(PadSelectionEnable_D1);
  RSI_EGPIO_PadSelectionEnable(PadSelectionEnable_CSN0);
  RSI_EGPIO_PadSelectionEnable(PadSelectionEnable_D2);
  RSI_EGPIO_PadSelectionEnable(PadSelectionEnable_D3);
#endif

  /*Receive enable for QSPI GPIO*/
  RSI_EGPIO_PadReceiverEnable(M4SS_QSPI_CLK);
  RSI_EGPIO_PadReceiverEnable(M4SS_QSPI_CSN0);
  RSI_EGPIO_PadReceiverEnable(M4SS_QSPI_D0);
  RSI_EGPIO_PadReceiverEnable(M4SS_QSPI_D1);
  RSI_EGPIO_PadReceiverEnable(M4SS_QSPI_D2);
  RSI_EGPIO_PadReceiverEnable(M4SS_QSPI_D3);

  /*Set GPIO pin MUX for QSPI*/
  RSI_EGPIO_SetPinMux(EGPIO, 0, M4SS_QSPI_CLK, QSPI_MODE);
  RSI_EGPIO_SetPinMux(EGPIO, 0, M4SS_QSPI_CSN0, QSPI_MODE);
  RSI_EGPIO_SetPinMux(EGPIO, 0, M4SS_QSPI_D0, QSPI_MODE);
  RSI_EGPIO_SetPinMux(EGPIO, 0, M4SS_QSPI_D1, QSPI_MODE);
  RSI_EGPIO_SetPinMux(EGPIO, 0, M4SS_QSPI_D2, QSPI_MODE);
  RSI_EGPIO_SetPinMux(EGPIO, 0, M4SS_QSPI_D3, QSPI_MODE);
}
#endif
/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
//int main()
int rsi_qspi_example(void);

int rsi_qspi_example()
{
  /* At this stage the MICROCONTROLLER clock setting is already configured,
	 * this is done through SystemInit() function which is called from startup
	 * file (startup_RS1xxxx.s) before to branch to application main.
	 * To reconfigure the default setting of SystemInit() function, refer to
	 * system_RS1xxxx.c file
	 */
  int i = 0;
  /*Init the QSPI configurations structure */
  spi_config_t spi_configs_init;

  /*Configures the system default clock and power configurations*/
  SystemCoreClockUpdate();

  /* Initialize UART for debug prints */
  DEBUGINIT();

  /* Genrates data buffer */
  Genrate_sample_data();

  /*Get QSPI  structures configuration     */
  GetQspiConfig(&spi_configs_init);

  /* Configures the pin MUX for QSPI  pins*/
  DEBUGOUT("\r\n Configure Flash Pins \r\n");
  RSI_QSPI_PinMuxInit();
  /*Disable all IRQ before accessing flash */
  __disable_irq();
  /* initializes QSPI  */
  DEBUGOUT("\r\n QSPI Initialisation \r\n");
  RSI_QSPI_SpiInit((qspi_reg_t *)QSPI_BASE, &spi_configs_init, 1, 0, 0);

  /* Erases the SECTOR   */
  DEBUGOUT("\r\n Erase Sector \r\n");
  RSI_QSPI_SpiErase((qspi_reg_t *)QSPI_BASE, &spi_configs_init, SECTOR_ERASE, ADDRESS, 1, 0);

  /* writes the data to required address using qspi */
  DEBUGOUT("\r\n Write Data to Flash Memory \r\n");
#if 0
  RSI_QSPI_SpiWrite((qspi_reg_t *)QSPI_BASE,
                    &spi_configs_init,
                    0x2,
                    ADDRESS,
                    (uint8_t *)&_acTestData[0],
                    sizeof(_acTestData),
                    256,
                    _1BYTE,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0);
#else
  RSI_QSPI_SpiWrite((qspi_reg_t *)QSPI_BASE,
                    &spi_configs_init,
                    0x2,
                    ADDRESS,
                    (uint8_t *)&data_write[0],
                    sizeof(data_write),
                    256,
                    _1BYTE,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0);
#endif
  /*Enable all IRQ after accessing flash */
  __enable_irq();
  /* DMA Read configs */
#ifdef DMA_READ
  /* Read the data by using UDMA */
  DEBUGOUT("\r\n Read Data From Flash Memory Using DMA \r\n");
  UDMA_Read();
  /* Wait till dma done */
  while (!done)
    ;
#endif

    /* IO Read config */
#ifdef IO_READ
  /* Reads from the address in manual mode */
  DEBUGOUT("\r\n Read Data From Flash Memory Using Manual Mode\r\n");
  RSI_QSPI_ManualRead((qspi_reg_t *)(QSPI_BASE),
                      &spi_configs_init,
                      ADDRESS,
                      (uint8_t *)read_data,
                      0 /*_32BIT*/,
                      sizeof(data_write),
                      0,
                      0,
                      0);
#endif

#if 0 //senthil - 
  /* compares both read and write data */
  for (i = 0; i < (sizeof(data_write) / sizeof(data_write[0])); i++) {
    if (read_data[i] == data_write[i]) {
      continue;
    } else {
      break;
    }
  }

  if (i == (sizeof(data_write) / sizeof(data_write[0]))) {
    /*  Prints on hyper-terminal  */
    DEBUGOUT("\r\n Data comparison success\r\n");
  } else {
    /*  Prints on hyper-terminal  */
    DEBUGOUT("\r\n  Data comparison fail\r\n");
  }

  while (1)
    ;
#else
	return 1;
#endif
}

//#endif //senthil
