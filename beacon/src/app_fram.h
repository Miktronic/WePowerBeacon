/** @file fram.h
* 
* @brief FRAM interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl.  All rights reserved.
*/ 

#ifndef APP_FRAM_H
#define APP_FRAM_H

#define FRAM_ERROR -1
#define FRAM_SUCCESS 0

int app_fram_service(uint32_t *counter);

#endif /* APP_FRAM_H */

