/*******************************************************************************
*
* FILE: 
* 		crypto.h
*
* DESCRIPTION: 
* 		Contains encryption functions for SDR code.
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CRYPTO_H 
#define CRYPTO_H 

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/
typedef struct _AES_KEY
    {
    uint8_t key_buf[16];
    } AES_KEY;

/* Status enum */
typedef enum CRYP_STATUS {
    CRYP_OK = 0,
    CRYP_FAIL,
    CRYP_CONFIG_FAIL
} CRYP_STATUS;

/*------------------------------------------------------------------------------
 Macros & Inlines
------------------------------------------------------------------------------*/





/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
void crypto_init(void);

CRYP_STATUS crypto_set_key(AES_KEY *new_aes_key);

CRYP_STATUS crypto_encrypt(uint32_t *input, uint16_t size, uint32_t *output);

CRYP_STATUS crypto_decrypt(uint32_t *input, uint16_t size, uint32_t *output);

CRYP_STATUS crypto_rng
    (
    uint8_t* ret_buf,
    uint8_t rng_size
    );

#ifdef __cplusplus
}
#endif
#endif /* CRYPTO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/

