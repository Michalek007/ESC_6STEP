
/**
  ******************************************************************************
  * @file    power_stage_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a power stage.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef POWER_STAGE_PARAMETERS_H
#define POWER_STAGE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************* PWM Driving signals section **************/
/******** BEMF reading parameters section ******/

#define BEMF_ON_SENSING_DIVIDER              5.545455 /*!< BEMF voltage divider during PWM ON time
                                                   for zero crossing detection */
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR             0.052164840897235255 /*!< It expresses how
                                                      much the Vbus is attenuated
                                                      before being converted into
                                                      digital value */
#define NOMINAL_BUS_VOLTAGE_V                12
/******** Current reading parameters section ******/
/*** Topology ***/
#define None

#define RSHUNT                               0.01

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN                   6.6
#define CURR_REF_DIVIDER                     3.2  /*!< Divider of the current limiter threshold  */
#define OCP_INT_REF                          250  /*!< Internal over-current protecion threshold (mV) */

/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                                 0.290 /*!< in Volts */
#define T0_C                                 25 /*!< in Celsius degrees */
#define dV_dT                                0.025 /*!< V/Celsius degrees */
#define T_MAX                                70 /*!< Sensor measured
                                                     temperature at maximum
                                                     power stage working
                                                     temperature, Celsius degrees */

#endif /*POWER_STAGE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
