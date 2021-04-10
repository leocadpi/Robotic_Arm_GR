/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \brief   Driver for MegaPi board.
 * @file    MeMegaPi.h
 * @author  MakeBlock
 * @version V1.0.1
 * @date    2016/03/10
 * @brief   Driver for MegaPi board.
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is the driver for MegaPi board by MakeBlock.
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan         2016/02/20     1.0.0            Build the New.
 * Mark Yan         2016/03/10     1.0.1            Port mapping changes.
 * </pre>
 */
#ifndef MeMegaPi_H
#define MeMegaPi_H

#include <Arduino.h>
#include "MeConfig.h"
#include "MePort.h"
#include "MeStepperOnBoard.h"

/*********************  MegaPi Board GPIO Map *********************************/
// struct defined in MeMegaPi.h
#define PORT1A  PORT_1
#define PORT1B  PORT_9
#define PORT2A  PORT_2
#define PORT2B  PORT_10
#define PORT3A  PORT_3
#define PORT3B  PORT_11
#define PORT4A  PORT_4
#define PORT4B  PORT_12

MePort_Sig mePort[15] =
 {
   { NC, NC }, {  NC,  NC }, {  NC,  NC }, {  NC,  NC }, {  NC,  NC }, 
   { 16, 17 }, {  A8,  A9 }, { A10, A11 }, { A13, A12 }, {  NC,  NC }, 
   { NC, NC }, {  NC,  NC }, {  NC,  NC }, {  NC,  NC }, {  NC,  NC },
 };
#endif // MeMegaPi_H
