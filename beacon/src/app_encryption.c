/** @file app_encryption.c
* 
* @brief Encryption interface
*
* @par       
* COPYRIGHT NOTICE: (c) 2022 Ovyl. All rights reserved.
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/types.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/crypto/crypto.h>

#include "app_encryption.h"

#define CRYPTO_DRV_NAME CONFIG_CRYPTO_MBEDTLS_SHIM_DRV_NAME
#define CRYPTO_DEV_COMPAT nordic_nrf_ecb

uint32_t cap_flags;

// static void print_buffer_comparison(const uint8_t *wanted_result,
// 				    uint8_t *result, size_t length)
// {
// 	int i, j;

// 	printk("Was waiting for: \n");

// 	for (i = 0, j = 1; i < length; i++, j++) {
// 		printk("0x%02x ", wanted_result[i]);

// 		if (j == 10) {
// 			printk("\n");
// 			j = 0;
// 		}
// 	}

// 	printk("\n But got:\n");

// 	for (i = 0, j = 1; i < length; i++, j++) {
// 		printk("0x%02x ", result[i]);

// 		if (j == 10) {
// 			printk("\n");
// 			j = 0;
// 		}
// 	}

// 	printk("\n");
// }

void ecb_mode(const struct device *dev)
{
	/* from FIPS-197 test vectors */


	
	// struct cipher_pkt decrypt = {
	// 	.in_buf = encrypt.out_buf,
	// 	.in_len = sizeof(encrypted),
	// 	.out_buf = decrypted,
	// 	.out_buf_max = sizeof(decrypted),
	// };

	

	

	// if (cipher_begin_session(dev, &ini, CRYPTO_CIPHER_ALGO_AES,
	// 			 CRYPTO_CIPHER_MODE_ECB,
	// 			 CRYPTO_CIPHER_OP_DECRYPT)) {
	// 	return;
	// }

	// if (cipher_block_op(&ini, &decrypt)) {
	// 	printk("ECB mode DECRYPT - Failed");
	// 	goto out;
	// }

	// printk("Output length (decryption): %d", decrypt.out_len);

	// if (memcmp(decrypt.out_buf, ecb_plaintext, sizeof(ecb_plaintext))) {
	// 	printk("ECB mode DECRYPT - Mismatch between plaintext and "
	// 		    "decrypted cipher text");
	// 	print_buffer_comparison(ecb_plaintext, decrypt.out_buf,
	// 				sizeof(ecb_plaintext));
	// 	goto out;
	// }

	// printk("ECB mode DECRYPT - Match");

}

int validate_hw_compatibility(const struct device *dev)
{
	uint32_t flags = 0U;

	flags = crypto_query_hwcaps(dev);
	if ((flags & CAP_RAW_KEY) == 0U) {
		printk("Please provision the key separately "
			"as the module doesnt support a raw key");
		return -1;
	}

	if ((flags & CAP_SYNC_OPS) == 0U) {
		printk("The app assumes sync semantics. "
		  "Please rewrite the app accordingly before proceeding");
		return -1;
	}

	if ((flags & CAP_SEPARATE_IO_BUFS) == 0U) {
		printk("The app assumes distinct IO buffers. "
		"Please rewrite the app accordingly before proceeding");
		return -1;
	}

	cap_flags = CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS;

	return 0;

}

int app_encrypt_payload(uint8_t *cleartext, uint8_t cleartext_len, uint8_t *encrypted, uint8_t encrypted_len)
{
    const struct device *dev_crypto = device_get_binding(CRYPTO_DRV_NAME);

	if (!dev_crypto) {
		printk("%s crypto device not found", CRYPTO_DRV_NAME);
		return ENCRYPTION_ERROR;
	}

    if (validate_hw_compatibility(dev_crypto)) {
		printk("Incompatible h/w");
		return ENCRYPTION_ERROR;
	}

    printk("Crypto Init Success!\n");

    uint8_t ecb_key[16] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
	};

	struct cipher_ctx ini = {
		.keylen = sizeof(ecb_key),
		.key.bit_stream = ecb_key,
		.flags = cap_flags,
	};
	struct cipher_pkt encrypt = {
		.in_buf = cleartext,
		.in_len = cleartext_len,
		.out_buf_max = encrypted_len,
		.out_buf = encrypted,
	};

    if (cipher_begin_session(dev_crypto, &ini, CRYPTO_CIPHER_ALGO_AES,
				 CRYPTO_CIPHER_MODE_ECB,
				 CRYPTO_CIPHER_OP_ENCRYPT)) {
		return ENCRYPTION_ERROR;
	}

	if (cipher_block_op(&ini, &encrypt)) {
		printk("ERROR: ECB mode ENCRYPT - Failed\n");
		goto out;
	}

    printk("Output length (encryption): %d\n", encrypt.out_len);

	if (memcmp(encrypt.out_buf, encrypted, encrypted_len)) {
		printk("ERROR: ECB mode ENCRYPT - Mismatch between expected and "
			    "returned cipher text\n");
		goto out;
	}

	printk("ECB mode ENCRYPT - Match\n");
    cipher_free_session(dev_crypto, &ini);
    return ENCRYPTION_SUCCESS;

out:
	cipher_free_session(dev_crypto, &ini);
    return ENCRYPTION_ERROR;
}