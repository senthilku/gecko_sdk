/***************************************************************************//**
 * @file
 * @brief NVM3 definition of the default data structures.
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

#include "nvm3.h"
#include "nvm3_hal_flash.h"

#ifndef NVM3_DEFAULT_CACHE_SIZE
#define NVM3_DEFAULT_CACHE_SIZE  100
#endif
#ifndef NVM3_DEFAULT_NVM_SIZE
#define NVM3_DEFAULT_NVM_SIZE  36864
#endif
#ifndef NVM3_DEFAULT_MAX_OBJECT_SIZE
#define NVM3_DEFAULT_MAX_OBJECT_SIZE  NVM3_MAX_OBJECT_SIZE
#endif
#ifndef NVM3_DEFAULT_REPACK_HEADROOM
#define NVM3_DEFAULT_REPACK_HEADROOM  0
#endif

#ifndef NVM3_BASE

#if defined (__ICCARM__)

#ifndef __NVM3__
#define __NVM3__ "SIMEE"
#endif

__root uint8_t nvm3Storage[NVM3_DEFAULT_NVM_SIZE] @ __NVM3__;
#define NVM3_BASE (nvm3Storage)

#elif defined (__GNUC__)

#ifndef __NVM3__
#define __NVM3__ ".simee"
#endif

__attribute__((used)) uint8_t nvm3Storage[NVM3_DEFAULT_NVM_SIZE] __attribute__ ((section(__NVM3__)));
/* If linker does not provide __nvm3Base symbol, then use nvm3Storage*/
extern char __nvm3Base __attribute__((alias("nvm3Storage")));
#define NVM3_BASE (&__nvm3Base)

#else
#error "Unsupported toolchain"
#endif

#endif //NVM3_BASE

uint8_t       *nvm3Address = (uint8_t *)NVM3_BASE;
nvm3_Handle_t  nvm3_defaultHandleData;
nvm3_Handle_t *nvm3_defaultHandle = &nvm3_defaultHandleData;

static nvm3_CacheEntry_t defaultCache[NVM3_DEFAULT_CACHE_SIZE];

nvm3_Init_t    nvm3_defaultInitData =
{
  (nvm3_HalPtr_t)NVM3_BASE,
  NVM3_DEFAULT_NVM_SIZE,
  defaultCache,
  NVM3_DEFAULT_CACHE_SIZE,
  NVM3_DEFAULT_MAX_OBJECT_SIZE,
  NVM3_DEFAULT_REPACK_HEADROOM,
  &nvm3_halFlashHandle,
};

nvm3_Init_t   *nvm3_defaultInit = &nvm3_defaultInitData;

uint32_t MapNvm3Error(Ecode_t nvm3Res)
{
    //CHIP_ERROR err;
    unsigned int err = 0;

    switch (nvm3Res)
    {
    case ECODE_NVM3_OK:
        err = 1;//CHIP_NO_ERROR;
        break;
    case ECODE_NVM3_ERR_KEY_NOT_FOUND:
        err = 2;//CHIP_DEVICE_ERROR_CONFIG_NOT_FOUND;
        break;
    default:
        err = 3;//CHIP_ERROR(ChipError::Range::kPlatform, (nvm3Res & 0xFF) + CHIP_DEVICE_CONFIG_EFR32_NVM3_ERROR_MIN);
        break;
    }

    return err;
}

Ecode_t nvm3_initDefault(void)
{
  Ecode_t err = 1;
  err = nvm3_open(nvm3_defaultHandle, nvm3_defaultInit);
  MapNvm3Error(err);
  return err;
}

Ecode_t nvm3_deinitDefault(void)
{
  return nvm3_close(nvm3_defaultHandle);
}

unsigned int key = 0x087300;
//typedef uint32_t Ecode_t;

Ecode_t nvm3_WriteConfigValue(void)
{
       unsigned int ret = 0;
	unsigned int objectType;
	size_t dataLen;
	uint32_t tmpVal1 = 0;

	uint32_t val = 0;
	val = 0x35AABB35;
	val = 0x12345671;
	//val = 0x10012002;

       //nvm3_repack(nvm3_defaultHandle);
  nvm3_deleteObject(nvm3_defaultHandle, key);
	ret = nvm3_writeData(nvm3_defaultHandle, 0x087300, &val, sizeof(val))
;
  //ret = nvm3_getObjectInfo(nvm3_defaultHandle, 0x087300, &objectType, &dataLen);
  ret = nvm3_readData(nvm3_defaultHandle, 0x087300, &tmpVal1, dataLen);

	printf("ret:%d\n", ret);
	return 1;
}

Ecode_t nvm3_ReadConfigValue(void)
{
	unsigned int ret = 0;
	unsigned int objectType;
	size_t dataLen;
	uint32_t tmpVal = 0;
	
	ret = nvm3_getObjectInfo(nvm3_defaultHandle, key, &objectType, &dataLen);
	ret = nvm3_readData(nvm3_defaultHandle, key, &tmpVal, dataLen);
	return 1;
}

