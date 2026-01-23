/*******************************************************************************
*
* FILE: 
* 		crypto.c
*
* DESCRIPTION: 
* 		Contains encryption functions for SDR code 
*
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "init.h"
#include "main.h"
#include "crypto.h"
#include "stm32h7xx_hal_cryp.h"
#include "error_sdr.h"



/*------------------------------------------------------------------------------
 Global Variables  
------------------------------------------------------------------------------*/
extern CRYP_HandleTypeDef hcryp;


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		crypto_init                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Initialize cryptography                                                *
*                                                                              *
*******************************************************************************/
void crypto_init
    (
    void
    ) 
{ 
    CRYP_Init();
} /* crypto_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		crypto_set_key                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Set the AES encryption key                                             *
*                                                                              *
*******************************************************************************/
enum CRYP_STATUS crypto_set_key
    (
    AES_KEY *new_aes_key
    ) 
{
    /* locals */
    CRYP_ConfigTypeDef newConf; /* can be allocated on the stack since params are copied to the global hcryp handle */
    HAL_StatusTypeDef hal_status;

    /* set up config struct */
    memcpy(&newConf, &hcryp.Init, sizeof( newConf) ); /* make a new config struct and copy the previous contents */
    newConf.pKey = (uint32_t*)&new_aes_key;

    /* call HAL function */
    hal_status = HAL_CRYP_SetConfig( &hcryp, &newConf );

    /* check status return */
    if (hal_status == HAL_OK) 
        {
            return CRYP_OK;
        }
    else
        {
            return CRYP_CONFIG_FAIL;
        }
    //memcpy(hcryp.Init.pKey, new_aes_key, sizeof(AES_KEY));
    //CRYP_SetKey(&hcryp, sizeof(AES_KEY));
} /* crypto_set_key */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		crypto_encrypt                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Encrypt a message                                                      *
*                                                                              *
*******************************************************************************/
enum CRYP_STATUS crypto_encrypt
    (
    uint32_t *input,
    uint16_t size,
    uint32_t *output
    ) 
{
    if ( size % 4 != 0 ) 
        {
        return CRYP_FAIL;
        }
    
    HAL_StatusTypeDef status;
    status = HAL_OK;
    status = HAL_CRYP_Encrypt(&hcryp, 
                            input, 
                            size, 
                            output,
                            HAL_DEFAULT_TIMEOUT);

    if ( status == HAL_OK ) 
        {
        return CRYP_OK;
        } 
    else 
        {
        return CRYP_FAIL;
        }
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		crypto_decrypt                                                         *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Decrypt an encrypted message                                           *
*                                                                              *
*******************************************************************************/
enum CRYP_STATUS crypto_decrypt
    (
    uint32_t *input,
    uint16_t size, 
    uint32_t *output
    )
{
    if ( size % 4 != 0 ) 
    {
        return CRYP_FAIL;
    }
    HAL_StatusTypeDef status;
    status = HAL_OK;
    status = HAL_CRYP_Decrypt(&hcryp, 
                            input, 
                            size, 
                            output,
                            HAL_DEFAULT_TIMEOUT);
    
    if ( status == HAL_OK ) 
        {
        return CRYP_OK;
        } 
    else 
        {
        return CRYP_FAIL;
        }
}

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/