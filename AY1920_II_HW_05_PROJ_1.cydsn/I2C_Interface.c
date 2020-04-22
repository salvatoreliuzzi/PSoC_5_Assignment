/*
* This file includes all the required source code to interface
* the I2C peripheral.
*/

/**
*   \brief Value returned if device present on I2C bus.
*/
#ifndef DEVICE_CONNECTED
    #define DEVICE_CONNECTED 1
#endif

/**
*   \brief Value returned if device not present on I2C bus.
*/
#ifndef DEVICE_UNCONNECTED
    #define DEVICE_UNCONNECTED 0
#endif

#include "I2C_Interface.h" 
#include "I2C_Master.h"

    /*******************************************************************************************************************/
    /*                                             PERIPHERAL START                                                    */
    /*******************************************************************************************************************/

    ErrorCode I2C_Peripheral_Start(void) 
    {
        // Start I2C peripheral
        I2C_Master_Start();  
        
        // Return no error since start function does not return any error
        return NO_ERROR;
    }
    
    
    /*******************************************************************************************************************/
    /*                                             PERIPHERAL STOP                                                     */
    /*******************************************************************************************************************/
    
    ErrorCode I2C_Peripheral_Stop(void)
    {
        // Stop I2C peripheral
        I2C_Master_Stop();
        // Return no error since stop function does not return any error
        return NO_ERROR;
    }
    
    
    /*******************************************************************************************************************/
    /*                                            READ SINGLE REGISTER                                                 */
    /*******************************************************************************************************************/

    ErrorCode I2C_Peripheral_ReadRegister(uint8_t device_address, 
                                          uint8_t register_address,
                                          uint8_t* data)
    {
        // Send start condition
        uint8_t error = I2C_Master_MasterSendStart(device_address,I2C_Master_WRITE_XFER_MODE);
        if (error == I2C_Master_MSTR_NO_ERROR)
        {
            // Write address of register to be read
            error = I2C_Master_MasterWriteByte(register_address);
            if (error == I2C_Master_MSTR_NO_ERROR)
            {
                // Send restart condition
                error = I2C_Master_MasterSendRestart(device_address, I2C_Master_READ_XFER_MODE);
                if (error == I2C_Master_MSTR_NO_ERROR)
                {
                    // Read data without acknowledgement
                    *data = I2C_Master_MasterReadByte(I2C_Master_NAK_DATA);
                }
            }
        }
        // Send stop condition
        I2C_Master_MasterSendStop();
        // Return error code
        return error ? ERROR : NO_ERROR;
    }
        
    
    /*******************************************************************************************************************/
    /*                                            READ MULTIPLE REGISTERs                                              */
    /*******************************************************************************************************************/
    
    ErrorCode I2C_Peripheral_ReadRegisterMulti(uint8_t device_address,
                                               uint8_t register_address,
                                               uint8_t register_count,
                                               uint8_t* data)
    {
       // Send start condition
        uint8_t error = I2C_Master_MasterSendStart(device_address,I2C_Master_WRITE_XFER_MODE);
        if (error == I2C_Master_MSTR_NO_ERROR)
        {
            // Write address of register to be read - |0x80 enables multiple read/write
            error = I2C_Master_MasterWriteByte(register_address|0x80);
            if (error == I2C_Master_MSTR_NO_ERROR){
                
                // Send restart condition
                error = I2C_Master_MasterSendRestart(device_address, I2C_Master_READ_XFER_MODE);
                
                // cycle 
                for(int i=0; i<register_count-1; i++){
                    if (error == I2C_Master_MSTR_NO_ERROR){
                        // Read next data with acknowledgement
                        *(data+i) = I2C_Master_MasterReadByte(I2C_Master_ACK_DATA);      
                    }
                }
                
                // last byte
                if (error == I2C_Master_MSTR_NO_ERROR){
                    // Read next data without acknowledgement
                    *(data+register_count-1) = I2C_Master_MasterReadByte(I2C_Master_NAK_DATA);
                }
            }
        }
        // Send stop condition
        I2C_Master_MasterSendStop();
        // Return error code
        return error ? ERROR : NO_ERROR;
    }
    

    
    
    /*******************************************************************************************************************/
    /*                                            WRITE SINGLE REGISTER                                                */
    /*******************************************************************************************************************/
    
    ErrorCode I2C_Peripheral_WriteRegister(uint8_t device_address,
                                           uint8_t register_address,
                                           uint8_t data)
    {
        // Send start condition
        uint8_t error = I2C_Master_MasterSendStart(device_address, I2C_Master_WRITE_XFER_MODE);
        if (error == I2C_Master_MSTR_NO_ERROR){
            
            // Write register address
            error = I2C_Master_MasterWriteByte(register_address);
            if (error == I2C_Master_MSTR_NO_ERROR)
            {
                // Write byte of interest
                error = I2C_Master_MasterWriteByte(data);
            }
        }
        // Send stop condition
        I2C_Master_MasterSendStop();
        // Return error code
        return error ? ERROR : NO_ERROR;
    }
    
    
    /*******************************************************************************************************************/
    /*                                            WRITE MULTIPLE REGISTERs                                             */
    /*******************************************************************************************************************/
    
    ErrorCode I2C_Peripheral_WriteRegisterMulti(uint8_t device_address,
                                                uint8_t register_address,
                                                uint8_t register_count,
                                                uint8_t* data)                 
    {
        // Send start condition
        uint8_t error = I2C_Master_MasterSendStart(device_address, I2C_Master_WRITE_XFER_MODE);
        if (error == I2C_Master_MSTR_NO_ERROR)
        {
            // Write address of register to be written - |0x80 enables multiple read/write
            error = I2C_Master_MasterWriteByte(register_address|0x80); 

            // cycle
            for(int i=0; i<register_count; i++){
                if (error == I2C_Master_MSTR_NO_ERROR){
                    // Write next byte of interest
                    error = I2C_Master_MasterWriteByte(*(data+i));  
                } 
            }
        }
                
        // Send stop condition
        I2C_Master_MasterSendStop();
        // Return error code
        return error ? ERROR : NO_ERROR;
    }
    
    
    
    /*******************************************************************************************************************/
    /*                                          IS THE DEVICE CONNECTED?                                               */
    /*******************************************************************************************************************/
    
    uint8_t I2C_Peripheral_IsDeviceConnected(uint8_t device_address)
    {
        // Send a start condition followed by a stop condition
        uint8_t error = I2C_Master_MasterSendStart(device_address, I2C_Master_WRITE_XFER_MODE);
        I2C_Master_MasterSendStop();
        // If no error generated during stop, device is connected
        if (error == I2C_Master_MSTR_NO_ERROR)
        {
            return DEVICE_CONNECTED;
        }
        return DEVICE_UNCONNECTED;
    }

/* [] END OF FILE */
