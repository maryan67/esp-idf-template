/*
 * NVMOperator.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef MAIN_NVMOPERATOR_NVSOPERATOR_H_
#define MAIN_NVMOPERATOR_NVSOPERATOR_H_



#include "nvs_flash.h"
#include "nvs.h"
#include <string>
#include <stdio.h>
#include "GeneralErrorCodes.h"

class NVSModule
{
public :
    NVSModule()const throw(GeneralErrorCodes_te);
    ~NVSModule();
    void Writeu16ToField_v(char*FieldName_str, uint16_t Value_str) const throw (GeneralErrorCodes_te);;
    uint16_t  GetValueOfField_u16(char* FieldName_str);

private:

    nvs_handle NvsHandler_u8;


};



#endif /* MAIN_NVMOPERATOR_NVSOPERATOR_H_ */
