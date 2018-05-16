/*
 * DroneHandler.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: secar
 */

#include "DroneHandler.h"


DroneHandler* DroneHandler::getSingletonInstance()
{
    if(this->DroneHandlerSingleton_po == NULL)
    {
        static DroneHandler * droneHandler_o = new DroneHandler();
        this->DroneHandlerSingleton_po = droneHandler_o;

    }
    return this->DroneHandlerSingleton_po;
}