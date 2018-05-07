/*
 * NVMOperator.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef MAIN_NVMOPERATOR_NVSOPERATOR_H_
#define MAIN_NVMOPERATOR_NVSOPERATOR_H_

extern "C" {
#include "nvs.h"
#include "nvs_flash.h"
}
#include <string>
#include <stdio.h>
#include "GeneralErrorCodes.h"



class NVSModule
{
  public:
    NVSModule() noexcept(false);
    ~NVSModule();
    void Writeu16ToField_v(char *FieldName_str, uint16_t Value_str) noexcept(false);
    uint16_t GetValueOfField_u16(char *FieldName_str);

  private:
    nvs_handle NvsHandler_u8;
};

#endif /* MAIN_NVMOPERATOR_NVSOPERATOR_H_ */
