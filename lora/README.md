# Avionics RFM95 LoRa Library

Note: Currently, this README is not mean to serve as documentation, merely a place for several notes that if in code comments would not be conducive to clean code. This may change in the future.


 For now, we only allow the frequency to be set to integer megahertz frequencies, but we may expose the chip native unit directly in the future e.g so that SDEC could convert a decimal frequency like 914.6 MHz into the chip format and send it over serial.

 This function contains the formula we use for convention the megahertz frequency to the chip's internal unit.
```C
uint32_t lora_helper_mhz_to_reg_val( uint32_t mhz_freq ) {
    return ( (2^19) * mhz_freq * 10^6 )/( 32 * 10^6 );
}
```