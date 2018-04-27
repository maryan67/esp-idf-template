/*
 * NVSOperator.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: secar
 */

#include "NVSOperator.h"

NVSModule::NVSModule () const throw (GeneralErrorCodes_te)
{

  esp_err_t APIReturnedErrorCode = nvs_flash_init ();
  if (APIReturnedErrorCode == ESP_OK)
    {
      APIReturnedErrorCode = nvs_open ("storage", NVS_READWRITE,
				       &NvsHandler_u8);
      if (APIReturnedErrorCode != ESP_OK)
	  throw(HAL_ERROR);
    }

}

NVSModule::~NVSModule ()
{
  nvs_close (NvsHandler_u8);
}

uint16_t NVSModule::GetValueOfField_u16 (char* FieldName_str)
{

  uint16_t ReturnedValue = 0;
  nvs_get_u16 (NvsHandler_u8, "test_variable", &ReturnedValue);

  return ReturnedValue;
}

void NVSModule::Writeu16ToField_v (char*FieldName_Str,
				   uint16_t Value_u16) const
				       throw (GeneralErrorCodes_te)
{
  if (ESP_OK != nvs_set_u16 (NvsHandler_u8, FieldName_Str, Value_u16))
    throw(HAL_ERROR);
}

