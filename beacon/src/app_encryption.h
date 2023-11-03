/** @file app_encryption.h
* 
* @brief Encryption interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl.  All rights reserved.
*/ 

#ifndef APP_ENCRYPTION_H
#define APP_ENCRYPTION_H

#include <stdint.h>

#define ENCRYPTION_ERROR -1
#define ENCRYPTION_SUCCESS 0
int app_encrypt_payload(uint8_t *cleartext, uint8_t cleartext_len, uint8_t *encrypted, uint8_t encrypted_len);

#endif /* APP_ENCRYPTION_H */

