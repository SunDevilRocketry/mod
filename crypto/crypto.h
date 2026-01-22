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
enum CRYP_STATUS {
    CRYP_FAIL,
    CRYP_CONFIG_FAIL,
    CRYP_OK
};

/*------------------------------------------------------------------------------
 Macros & Inlines
------------------------------------------------------------------------------*/





/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/
void crypto_init(void);

enum CRYP_STATUS crypto_set_key(AES_KEY *new_aes_key);

enum CRYP_STATUS crypto_encrypt(uint32_t *input, uint16_t size, uint32_t *output);

enum CRYP_STATUS crypto_decrypt(uint32_t *input, uint16_t size, uint32_t *output);




#ifdef __cplusplus
}
#endif
#endif /* CRYPTO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/

