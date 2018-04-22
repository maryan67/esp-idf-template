/*
 * NVMOperator.h
 *
 *  Created on: Apr 22, 2018
 *      Author: secar
 */

#ifndef MAIN_NVMOPERATOR_NVMOPERATOR_H_
#define MAIN_NVMOPERATOR_NVMOPERATOR_H_



#include "nvs_flash.h"
#include "nvs.h"
#include <string>
#include <stdio.h>

class NVSModule
{
public :
    NVSModule();
    void WriteString(std::string FieldName_str, std::string Value_str);
    std::string  GetValueOfField_str(std::string FieldName_str);

private:


};



#endif /* MAIN_NVMOPERATOR_NVMOPERATOR_H_ */
