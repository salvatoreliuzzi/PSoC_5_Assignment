/**
* \brief Main source file for the I2C-Master project.
*
* In this project we set up a I2C master device with
* to understand the I2C protocol and communicate with a
* a I2C Slave device (LIS3DH Accelerometer).
*
* \author Gabriele Belotti
* \date , 2020
*/

    /*******************************************************************************************************************/
    /*                                            INCLUDE HEADER FILES                                                 */
    /*******************************************************************************************************************/

#include "I2C_Interface.h"
#include "project.h"
#include "stdio.h"

    /*******************************************************************************************************************/
    /*                                               DEFINE ADDRESSES                                                  */
    /*******************************************************************************************************************/

// 7-bit I2C address of the slave device.
#define LIS3DH_DEVICE_ADDRESS 0x18

// Address of the WHO AM I register
#define LIS3DH_WHO_AM_I_REG_ADDR 0x0F

/* Request 2.2.1.1: Correctly set the control registers of the
*                   LIS3DH accelerometer to output 3 Axis accelerometer
*                   data in Normal Mode at 100 Hz...
*
* From datasheet:
*
*   |**************** CTRL_REG1 register ****************|
*   | ODR3 | ODR2 | ODR1 | ODR0 | LPen | Zen | Yen | Xen |
*   |  0   |  1   |  0   |  1   |  0   |  1  |  1  |  1  | 0x57
*
*   ODR[3:0]:   Data rate selection. Default value: 0000
*                                                   0101 HR / Normal / Low-power mode (100 Hz)
*   Zen:        Z-axis enable. Default value: 1
*   Yen:        Y-axis enable. Default value: 1
*   Xen:        X-axis enable. Default value: 1
*                             (0: X-axis disabled; 1: X-axis enabled)
*/
// Address of the Control register 1 - CTRL_REG1(20h)
#define LIS3DH_CTRL_REG1 0x20

// Hex value to set normal mode to the accelerator
#define LIS3DH_NORMAL_MODE_CTRL_REG1 0x57

/* Request 2.2.1.2: ...in the ±2.0g FSR.
*
* From datasheet:a
*
*    FS[1:0]: Full-scale selection. default value: 00
*             (00: ±2 g; 01: ±4 g; 10: ±8 g; 11: ±16 g)
*
*   |************ CTRL_REG4 register **************|
*   | BDU | BLE | FS1 | FS0 | HR | ST1 | ST0 | SIM |
*   |  1  |  0  |  0  |  0  | 0  |  0  |  0  |  0  | 0x80
*/

// Address of the Control register 4 - CTRL_REG4(23h)
#define LIS3DH_CTRL_REG4 0x23
#define LIS3DH_CTRL_REG4_BDU_ACTIVE 0x80

/* Request 2.2.2: Read Output registers at a correct frequency
*                 (verify new Data is available using StatusReg
*                 information). Carefully think about the possible
*                 options to read data at a constant rate.
*
*   |*************** STATUS_REG(27h) ***************|
*   |ZYXOR| ZOR | YOR | XOR |ZYXDA| ZDA | YDA | XDA |
*   |  0  |  0  |  0  |  0  |  0  |  0  |  0  |  0  | 0x0
*/

// Address of the Status register
#define LIS3DH_STATUS_REG 0x27

// ZYXDA Mask: X, Y and Z-axis new data available. Default value: 0
#define LIS3DH_STATUS_REG_ZYXDA 0x8

/* OUT_X_L (28h), OUT_X_H (29h)
*  OUT_Y_L (2Ah), OUT_Y_H (2Bh)
*  OUT_Z_L (2Ch), OUT_Z_H (2Dh) */
#define LIS3DH_OUT_X_L 0x28

// Address of FIFO control register
// Address of the Control register 5 - CTRL_REG5(20h)
#define LIS3DH_CTRL_REG5 0x24
#define LIS3DH_CTRL_REG5_FIFO_DISABLE 0x0 // FIFO 0x40
#define LIS3DH_FIFO_CTRL_REG 0x2E
#define LIS3DH_FIFO_DISABLE 0x0 // FIFO 0x40 / bypass 0x0 / Stream 0x80 / stream to fifo 0xC0

// Address of the Temperature Sensor Configuration register
#define LIS3DH_TEMP_CFG_REG 0x1F
#define LIS3DH_TEMP_CFG_REG_ACTIVE 0xC0

// Address of the ADC output LSB register
#define LIS3DH_OUT_ADC_3L 0x0C

// Address of the ADC output MSB register
#define LIS3DH_OUT_ADC_3H 0x0D

    /*******************************************************************************************************************/
    /*                                                    MAIN                                                         */
    /*******************************************************************************************************************/

int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    I2C_Peripheral_Start();
    UART_Debug_Start();

    CyDelay(5); //"The boot procedure is complete about 5 milliseconds after device power-up."

    // String to print out messages on the UART
    char message[50];

    // Check which devices are present on the I2C bus
    for (int i = 0 ; i < 128; i++)
    {
        if (I2C_Peripheral_IsDeviceConnected(i))
        {
            // print out the address is hex format
            sprintf(message, "Device 0x%02X is connected\r\n", i);
            UART_Debug_PutString(message);
        }

    }

    /*******************************************************************************************************************/
    /*                                                  I2C Reading                                                    */
    /*******************************************************************************************************************/

    /* Read WHO AM I REGISTER register */
    uint8_t who_am_i_reg;
    ErrorCode error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                                  LIS3DH_WHO_AM_I_REG_ADDR,
                                                  &who_am_i_reg);
    if (error == NO_ERROR)
    {
        sprintf(message, "WHO AM I REG: 0x%02X [Expected: 0x33]\r\n", who_am_i_reg);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm\r\n");
    }

    /*******************************************************************************************************************/
    /*                                         I2C Reading Status Register                                             */
    /*******************************************************************************************************************/

    uint8_t status_register;
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_STATUS_REG,
                                        &status_register);

    if (error == NO_ERROR)
    {
        sprintf(message, "STATUS REGISTER: 0x%02X\r\n", status_register);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read status register\r\n");
    }

    /*******************************************************************************************************************/
    /*                                            Read Control Register 1                                              */
    /*******************************************************************************************************************/

    uint8_t ctrl_reg1;
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG1,
                                        &ctrl_reg1);

    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 1: 0x%02X\r\n", ctrl_reg1);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register 1\r\n");
    }

    /*******************************************************************************************************************/
    /*                                                 I2C Writing                                                     */
    /*******************************************************************************************************************/

    UART_Debug_PutString("\r\nWriting new values..\r\n");

    if (ctrl_reg1 != LIS3DH_NORMAL_MODE_CTRL_REG1)
    {
        ctrl_reg1 = LIS3DH_NORMAL_MODE_CTRL_REG1;

        error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                             LIS3DH_CTRL_REG1,
                                             ctrl_reg1);

        if (error == NO_ERROR)
        {
            sprintf(message, "CONTROL REGISTER 1 successfully written as: 0x%02X\r\n", ctrl_reg1);
            UART_Debug_PutString(message);
        }
        else
        {
            UART_Debug_PutString("Error occurred during I2C comm to set control register 1\r\n");
        }
    }

    /*******************************************************************************************************************/
    /*                                               FIFO mode disabled                                                */
    /*******************************************************************************************************************/

    /* Previously it seemed as if the data were stuck, maybe because FIFO mode has been enabled in the past
    *  this mode could be usefull for achiving the following request:
    *  "Carefully think about the possible options to read data at a constant rate."
    */

    UART_Debug_PutString("\r\nWriting new values..\r\n");

    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG5,
                                         LIS3DH_CTRL_REG5_FIFO_DISABLE);

    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_FIFO_CTRL_REG,
                                         LIS3DH_FIFO_DISABLE);

    if (error == NO_ERROR)
    {
        sprintf(message, "FIFO mode disabled");
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred writing FIFO mode");
    }

    /*******************************************************************************************************************/
    /*                                         Read Control Register 1 again                                           */
    /*******************************************************************************************************************/

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG1,
                                        &ctrl_reg1);

    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 1 after overwrite operation: 0x%02X\r\n", ctrl_reg1);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register 1\r\n");
    }

    /*******************************************************************************************************************/
    /*                                   I2C Reading Temperature sensor CFG reg                                        */
    /*******************************************************************************************************************/

    uint8_t tmp_cfg_reg;

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_TEMP_CFG_REG,
                                        &tmp_cfg_reg);

    if (error == NO_ERROR)
    {
        sprintf(message, "TEMPERATURE CONFIG REGISTER: 0x%02X\r\n", tmp_cfg_reg);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read temperature config register\r\n");
    }


    tmp_cfg_reg = LIS3DH_TEMP_CFG_REG_ACTIVE; // must be changed to the appropriate value

    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_TEMP_CFG_REG,
                                         tmp_cfg_reg);

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_TEMP_CFG_REG,
                                        &tmp_cfg_reg);


    if (error == NO_ERROR)
    {
        sprintf(message, "TEMPERATURE CONFIG REGISTER after being updated: 0x%02X\r\n", tmp_cfg_reg);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read temperature config register\r\n");
    }

    uint8_t ctrl_reg4;

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &ctrl_reg4);

    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 4: 0x%02X\r\n", ctrl_reg4);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register4\r\n");
    }


    ctrl_reg4 = LIS3DH_CTRL_REG4_BDU_ACTIVE; // must be changed to the appropriate value

    error = I2C_Peripheral_WriteRegister(LIS3DH_DEVICE_ADDRESS,
                                         LIS3DH_CTRL_REG4,
                                         ctrl_reg4);

    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_CTRL_REG4,
                                        &ctrl_reg4);


    if (error == NO_ERROR)
    {
        sprintf(message, "CONTROL REGISTER 4 after being updated: 0x%02X\r\n", ctrl_reg4);
        UART_Debug_PutString(message);
    }
    else
    {
        UART_Debug_PutString("Error occurred during I2C comm to read control register4\r\n");
    }


    /*******************************************************************************************************************/
    /*                                             VARIABLE SETTINGS                                                   */
    /*******************************************************************************************************************/

    uint16_t xAcc;
    uint16_t yAcc;
    uint16_t zAcc;

    uint8_t header = 0xA0;
    uint8_t tail = 0xC0;

    uint8_t check;
    uint8_t status; // status of the status register

    uint8_t OutArray[8];
    OutArray[0] = header;
    uint8_t AccData[6];
    OutArray[7] = tail;

    /*******************************************************************************************************************/
    /*                                                    CYCLE                                                        */
    /*******************************************************************************************************************/

    for(;;)
    {
        CyDelay(10);

        // gathering the LIS3DH_STATUS_REG value
        error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                            LIS3DH_STATUS_REG,
                                            &status);

        // Check on ZYXDA cell of status register
        check = status & LIS3DH_STATUS_REG_ZYXDA;

        // if new data are available
        if(check == LIS3DH_STATUS_REG_ZYXDA){

        // gathering only x Acc
        error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS, // device_address
                                                 LIS3DH_OUT_X_L,        // register_address LIS3DH_OUT_X_L
                                                 6,                     // register_count 6
                                                 &AccData[0]);          // data pointer

        }

        if(error == NO_ERROR)
        {

            /*
            Convert the 3 axial outputs of the Accelerometer to 3
            right-justified 16-bit integers with the correct
            scaling (i.e. in mg).
            */

            xAcc = (int16_t)(((AccData[0] | AccData[1]<<8 )))>>6;
            OutArray[1] = (uint8_t)(xAcc >> 8);         // LSB
            OutArray[2] = (uint8_t)(xAcc & 0xFF);       // MSB

            yAcc = (int16_t)(((AccData[2] | AccData[3]<<8)))>>6;
            OutArray[3] = (uint8_t)(yAcc >> 8);         // LSB
            OutArray[4] = (uint8_t)(yAcc & 0xFF);       // MSB

            zAcc = (int16_t)(((AccData[4] | AccData[5]<<8)))>>6;
            OutArray[5] = (uint8_t)(zAcc >> 8);         // LSB
            OutArray[6] = (uint8_t)(zAcc & 0xFF);       // MSB

            UART_Debug_PutArray(OutArray, 8);

        }

    }
}

/* [] END OF FILE */
