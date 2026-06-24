#include "paramstore.h"

#include "main.h"

#include <math.h>

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE  0x400U
#endif

typedef struct
{
  uint32_t magic;
  float r;
  float bias;
} ParamStoreData;

typedef union
{
  float value;
  uint32_t bits;
} FloatBits;

uint8_t ParamStore_Load(float *r, float *bias)
{
  const ParamStoreData *stored;

  stored = (const ParamStoreData *)PARAMSTORE_ADDRESS;
  if (stored->magic != PARAMSTORE_MAGIC)
  {
    return 0U;
  }

  *r = stored->r;
  *bias = stored->bias;

  if ((!isfinite(*r)) || (*r <= 0.0f))
  {
    return 0U;
  }

  return 1U;
}

uint8_t ParamStore_Save(float r, float bias)
{
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error;
  FloatBits r_word;
  FloatBits bias_word;

  r_word.value = r;
  bias_word.value = bias;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return 0U;
  }

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.PageAddress = PARAMSTORE_ADDRESS;
  erase.NbPages = 1U;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAMSTORE_ADDRESS,
                        PARAMSTORE_MAGIC) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAMSTORE_ADDRESS + 4U,
                        r_word.bits) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAMSTORE_ADDRESS + 8U,
                        bias_word.bits) != HAL_OK)
  {
    HAL_FLASH_Lock();
    return 0U;
  }

  if (HAL_FLASH_Lock() != HAL_OK)
  {
    return 0U;
  }

  return 1U;
}
