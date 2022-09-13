/***************************************************************************//**
 * @file
 * @brief Non-Volatile Memory Wear-Leveling driver HAL implementation
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/

#include <stdbool.h>
#include <string.h>
#include "nvm3.h"
#include "nvm3_hal_flash.h"
#include "em_system.h"
#include "em_msc.h"

/***************************************************************************//**
 * @addtogroup nvm3
 * @{
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup nvm3hal
 * @{
 ******************************************************************************/

/******************************************************************************
 ******************************    MACROS    **********************************
 *****************************************************************************/

#define CHECK_DATA  1           ///< Macro defining if data should be checked

//CCP test
#define _FLASH_BASE_ADDR (0x802B000) //(0x08000000)

/******************************************************************************
 ***************************   LOCAL VARIABLES   ******************************
 *****************************************************************************/

/******************************************************************************
 ***************************   LOCAL FUNCTIONS   ******************************
 *****************************************************************************/

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */

/***************************************************************************//**
 * @brief
 *   Convert return type.
 *
 * @details
 *   This function converts between the return type of the emlib and the
 *   NVM3 API.
 *
 * @param[in] result
 *   Operation result.
 *
 * @return
 *   Returns remapped status code.
 ******************************************************************************/
static Ecode_t convertMscStatusToNvm3Status(MSC_Status_TypeDef result)
{
  Ecode_t ret;

  switch (result) {
    case mscReturnOk:
      ret = ECODE_NVM3_OK;
      break;
    case mscReturnInvalidAddr:
      ret = ECODE_NVM3_ERR_INT_ADDR_INVALID;
      break;
    default:
      ret = ECODE_NVM3_ERR_INT_EMULATOR;
      break;
  }

  return ret;
}

// Check if the page is erased.
static bool isErased(void *adr, size_t len)
{
  size_t i;
  size_t cnt;
  uint32_t *dat = adr;

  cnt = len / sizeof(uint32_t);
  for (i = 0U; i < cnt; i++) {
    if (*dat != 0xFFFFFFFFUL) {
      return false;
    }
    dat++;
  }

  return true;
}

/** @endcond */

#ifndef CCP_SI917_BRINGUP
//#if 1
static Ecode_t nvm3_halFlashOpen(nvm3_HalPtr_t nvmAdr, size_t flashSize)
{
  (void)nvmAdr;
  (void)flashSize;
  MSC_Init();

  return ECODE_NVM3_OK;
}

static void nvm3_halFlashClose(void)
{
  MSC_Deinit();
}

static Ecode_t nvm3_halFlashGetInfo(nvm3_HalInfo_t *halInfo)
{
  SYSTEM_ChipRevision_TypeDef chipRev;

  SYSTEM_ChipRevisionGet(&chipRev);
#if defined(_SYSCFG_CHIPREV_PARTNUMBER_MASK)
  halInfo->deviceFamilyPartNumber = chipRev.partNumber;
#else
  halInfo->deviceFamilyPartNumber = chipRev.family;
#endif
  halInfo->memoryMapped = 1;
#if defined(_SILICON_LABS_32B_SERIES_2)
  halInfo->writeSize = NVM3_HAL_WRITE_SIZE_32;
#else
  halInfo->writeSize = NVM3_HAL_WRITE_SIZE_16;
#endif
  halInfo->pageSize = SYSTEM_GetFlashPageSize();

  return ECODE_NVM3_OK;
}

static void nvm3_halFlashAccess(nvm3_HalNvmAccessCode_t access)
{
  (void)access;
}

static Ecode_t nvm3_halFlashReadWords(nvm3_HalPtr_t nvmAdr, void *dst, size_t wordCnt)
{
  uint32_t *pSrc = (uint32_t *)nvmAdr;
  uint32_t *pDst = dst;

  if ((((size_t)pSrc % 4) == 0) && (((size_t)pDst % 4) == 0)) {
    while (wordCnt > 0U) {
      *pDst++ = *pSrc++;
      wordCnt--;
    }
  } else {
    (void)memcpy(dst, nvmAdr, wordCnt * sizeof(uint32_t));
  }

  return ECODE_NVM3_OK;
}

static Ecode_t nvm3_halFlashWriteWords(nvm3_HalPtr_t nvmAdr, void const *src, size_t wordCnt)
{
  const uint32_t *pSrc = src;
  uint32_t *pDst = (uint32_t *)nvmAdr;
  MSC_Status_TypeDef mscSta;
  Ecode_t halSta;
  size_t byteCnt;

  byteCnt = wordCnt * sizeof(uint32_t);
  mscSta = MSC_WriteWord(pDst, pSrc, byteCnt);
  halSta = convertMscStatusToNvm3Status(mscSta);

#if CHECK_DATA
  if (halSta == ECODE_NVM3_OK) {
    if (memcmp(pDst, pSrc, byteCnt) != 0) {
      halSta = ECODE_NVM3_ERR_WRITE_FAILED;
    }
  }
#endif

  return halSta;
}

static Ecode_t nvm3_halFlashPageErase(nvm3_HalPtr_t nvmAdr)
{
  MSC_Status_TypeDef mscSta;
  Ecode_t halSta;

  mscSta = MSC_ErasePage((uint32_t *)nvmAdr);
  halSta = convertMscStatusToNvm3Status(mscSta);

#if CHECK_DATA
  if (halSta == ECODE_NVM3_OK) {
    if (!isErased(nvmAdr, SYSTEM_GetFlashPageSize())) {
      halSta = ECODE_NVM3_ERR_ERASE_FAILED;
    }
  }
#endif

  return halSta;
}


#else /* CCP_SI917_BRINGUP */

#define SIZE               256
uint32_t nvm3_read_data[SIZE], nvm3_data_write[SIZE];
char nvm3_acTestData[SIZE] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

void nvm3_Genrate_sample_data(void)
{
  int i;
  for (i = 0; i <= SIZE; i++) {
    nvm3_data_write[i] = i;
  }
}

static Ecode_t nvm3_halFlashOpen(nvm3_HalPtr_t nvmAdr, size_t flashSize)
{
  int res = -1;
  (void)nvmAdr;
  (void)flashSize;

#if 0  
  MSC_Init();
#else
	/* Genrates data buffer */
       nvm3_Genrate_sample_data();

	Init(0, 0, 0);

#if 0	
	res = EraseSector(_FLASH_BASE_ADDR);
	res = ProgramPage(_FLASH_BASE_ADDR, sizeof(nvm3_data_write), (char *)nvm3_data_write);
	res = ReadFlash(_FLASH_BASE_ADDR, sizeof(nvm3_read_data), (char *)nvm3_read_data);

	res = EraseSector(_FLASH_BASE_ADDR);
	res = ProgramPage(_FLASH_BASE_ADDR, 10, (char *)nvm3_data_write);
	res = EraseSector(_FLASH_BASE_ADDR);
	res = ProgramPage(_FLASH_BASE_ADDR, 10, (char *)nvm3_acTestData);
#endif

#endif

  return ECODE_NVM3_OK;
}

static void nvm3_halFlashClose(void)
{
  //MSC_Deinit();
}

static Ecode_t nvm3_halFlashGetInfo(nvm3_HalInfo_t *halInfo)
{
  SYSTEM_ChipRevision_TypeDef chipRev;

  SYSTEM_ChipRevisionGet(&chipRev);
#if defined(_SYSCFG_CHIPREV_PARTNUMBER_MASK)
  halInfo->deviceFamilyPartNumber = chipRev.partNumber;
#else
  halInfo->deviceFamilyPartNumber = chipRev.family;
#endif
  halInfo->memoryMapped = 1;
#if defined(_SILICON_LABS_32B_SERIES_2)
  halInfo->writeSize = NVM3_HAL_WRITE_SIZE_32;
#else
  halInfo->writeSize = NVM3_HAL_WRITE_SIZE_16;
#endif
  halInfo->pageSize = SYSTEM_GetFlashPageSize();

  return ECODE_NVM3_OK;
}

static void nvm3_halFlashAccess(nvm3_HalNvmAccessCode_t access)
{
  (void)access;
}

static Ecode_t nvm3_halFlashReadWords(nvm3_HalPtr_t nvmAdr, void *dst, size_t wordCnt)
{
  uint32_t *pSrc = (uint32_t *)nvmAdr;
  uint32_t *pDst = dst;

  if ((((size_t)pSrc % 4) == 0) && (((size_t)pDst % 4) == 0)) {
    while (wordCnt > 0U) {
      *pDst++ = *pSrc++;
      wordCnt--;
    }
  } else {
    (void)memcpy(dst, nvmAdr, wordCnt * sizeof(uint32_t));
  }

  return ECODE_NVM3_OK;
}

static Ecode_t nvm3_halFlashWriteWords(nvm3_HalPtr_t nvmAdr, void const *src, size_t wordCnt)
{
  const uint32_t *pSrc = src;
  uint32_t *pDst = (uint32_t *)nvmAdr;
  MSC_Status_TypeDef mscSta;
  Ecode_t halSta;
  size_t byteCnt;

  byteCnt = wordCnt * sizeof(uint32_t);
#if 0  
  mscSta = MSC_WriteWord(pDst, pSrc, byteCnt);
#else
  //ProgramPage(_FLASH_BASE_ADDR, sizeof(nvm3_data_write), (char *)nvm3_data_write);
  ProgramPage(pDst, byteCnt, (char *)pSrc);
#endif
  halSta = convertMscStatusToNvm3Status(mscSta);

#if CHECK_DATA
  if (halSta == ECODE_NVM3_OK) {
    if (memcmp(pDst, pSrc, byteCnt) != 0) {
      halSta = ECODE_NVM3_ERR_WRITE_FAILED;
    }
  }
#endif

  //return halSta;
  return ECODE_NVM3_OK;
}

static Ecode_t nvm3_halFlashPageErase(nvm3_HalPtr_t nvmAdr)
{
  MSC_Status_TypeDef mscSta;
  Ecode_t halSta;

#if 0
  //mscSta = MSC_ErasePage((uint32_t *)nvmAdr);
#else
  mscSta = EraseSector((uint32_t *)nvmAdr);
#endif

  halSta = convertMscStatusToNvm3Status(mscSta);

#if CHECK_DATA
  if (halSta == ECODE_NVM3_OK) {
    if (!isErased(nvmAdr, SYSTEM_GetFlashPageSize())) {
      halSta = ECODE_NVM3_ERR_ERASE_FAILED;
    }
  }
#endif

  //return halSta;
  return ECODE_NVM3_OK;
}


#endif /* CCP_SI917_BRINGUP */
/*******************************************************************************
 ***************************   GLOBAL VARIABLES   ******************************
 ******************************************************************************/

const nvm3_HalHandle_t nvm3_halFlashHandle = {
  .open = nvm3_halFlashOpen,                    ///< Set the open function
  .close = nvm3_halFlashClose,                  ///< Set the close function
  .getInfo = nvm3_halFlashGetInfo,              ///< Set the get-info function
  .access = nvm3_halFlashAccess,                ///< Set the access function
  .pageErase = nvm3_halFlashPageErase,          ///< Set the page-erase function
  .readWords = nvm3_halFlashReadWords,          ///< Set the read-words function
  .writeWords = nvm3_halFlashWriteWords,        ///< Set the write-words function
};

/** @} (end addtogroup nvm3hal) */
/** @} (end addtogroup nvm3) */
