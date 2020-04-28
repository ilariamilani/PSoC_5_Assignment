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

// Include required header files
#include "I2C_Interface.h"
#include "project.h"
#include "stdio.h"

/**
*   \brief 7-bit I2C address of the slave device.
*/
#define LIS3DH_DEVICE_ADDRESS 0x18

/**
*   \brief Address of the WHO AM I register
*/
#define LIS3DH_WHO_AM_I_REG_ADDR 0x0F

/**
*   \brief Address of the Status register
*/
#define LIS3DH_STATUS_REG 0x27

/**
*   \new Data on the Status register
*/
//#define LIS3DH_STATUS_REG_NEWDATA 0x08 //if ZYXDA == 1
#define LIS3DH_STATUS_REG_NEWDATAX 0x01 //if XDA == 1
#define LIS3DH_STATUS_REG_NEWDATAY 0x02 //if YDA == 1
#define LIS3DH_STATUS_REG_NEWDATAZ 0x04 //if ZDA == 1

/**
*   \brief Address of the Control register 1
*/
#define LIS3DH_CTRL_REG1 0x20 

/**
*   \brief Hex value to set normal mode to the accelerator
*/
#define LIS3DH_NORMAL_MODE_CTRL_REG1 0x57 //0101 to set 100hz
                                          //0111 normal or high res mode and axis enabled

/**
*   \brief  Address of the Temperature Sensor Configuration register
*/
// #define LIS3DH_TEMP_CFG_REG 0x1F

// #define LIS3DH_TEMP_CFG_REG_ACTIVE 0xC0

/**
*   \brief Address of the Control register 4
*/
#define LIS3DH_CTRL_REG4 0x23 

#define LIS3DH_CTRL_REG4_ACTIVE 0x9B // set BDU active, ±4.0 g FSR, high resol mode 

/**
*   \brief Address of the ADC output for the 3 axis, MSB (H) and LSB (L) register
*/
#define LIS3DH_OUT_X_L 0x28
#define LIS3DH_OUT_X_H 0x29
#define LIS3DH_OUT_Y_L 0x2A
#define LIS3DH_OUT_Y_H 0x2B
#define LIS3DH_OUT_Z_L 0x2C
#define LIS3DH_OUT_Z_H 0x2D


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
    
    /******************************************/
    /*            I2C Reading                 */
    /******************************************/
    
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
    
    /*      I2C Reading Status Register       */
    
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
    
    /******************************************/
    /*        Read Control Register 1         */
    /******************************************/
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
    
    /******************************************/
    /*            I2C Writing                 */
    /******************************************/
    
        
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
    
    /******************************************/
    /*     Read Control Register 1 again      */
    /******************************************/

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
    
    
     /******************************************/
     /*   Reading and Writing on ctrl reg 4 */
     /******************************************/

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
    
    
    ctrl_reg4 = LIS3DH_CTRL_REG4_ACTIVE;
    
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
    
    //int16_t OutX;
    //int16_t OutY;
    //int16_t OutZ;
    uint8_t header = 0xA0;
    uint8_t footer = 0xC0;
    //uint8_t OutArray[8]; 
    uint8_t Data[6];
    
    //OutArray[0] = header;
    //OutArray[7] = footer;
    
    int16_t OutX_ms2;
    int16_t OutY_ms2;
    int16_t OutZ_ms2;
    uint8_t OutArray_ms2[8]; 
    OutArray_ms2[0] = header;
    OutArray_ms2[7] = footer;
    
    float32 G = 9.80665;
    
    for(;;)
    {
        CyDelay(100);
        
        /*      I2C Reading Status Register       */
     
    error = I2C_Peripheral_ReadRegister(LIS3DH_DEVICE_ADDRESS,
                                        LIS3DH_STATUS_REG,
                                        &status_register);
    
    if (error != NO_ERROR)
    {
        UART_Debug_PutString("Error occurred during I2C comm to read status register\r\n");   
    }
    
    // if new data is available:
        
        if((LIS3DH_STATUS_REG & LIS3DH_STATUS_REG_NEWDATAX) == LIS3DH_STATUS_REG_NEWDATAX)
        {
            error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                 LIS3DH_OUT_X_L,
                                                 2, // 2 byte to be read
                                                 &Data[0]);
            if(error == NO_ERROR)
        {
            //OutX = (int16)((Data[0] | (Data[1]<<8)))>>4;
            //OutArray[1] = (uint8_t)(OutX & 0xFF);
            //OutArray[2] = (uint8_t)(OutX >> 8);
            
            OutX_ms2 = ((int16)((Data[0] | (Data[1]<<8)))>>4)*2/16; // 2 mg/digit --> *2/16 ( for the 4g FSR at HRM )
                                                                    // shift >>4 with the high resol mode, the info is in 12 bits, no more in 10
                                                                    // using 2/16 : error in the output value, to make it correct-ish I should do *2/128, 
                                                                    // then the value on the Z axis would be around 9.8
                                                                    // same for the other 2 axis
            OutX_ms2 = ((int16)(OutX_ms2/(G/10000)));  // the measure in output will be moltiplied per 10000 and saved as an int16
                                                       // then I will divide it for 10000 in the Bridge Control Panel
                                                       // testing it with *2/128/G/10000, I have a result with decimals,
                                                       // which are not visible with the values currently setted
            OutArray_ms2[1] = (uint8_t)(OutX_ms2 & 0xFF);
            OutArray_ms2[2] = (uint8_t)(OutX_ms2 >> 8);
            
        }
        }
        if((LIS3DH_STATUS_REG & LIS3DH_STATUS_REG_NEWDATAY) == LIS3DH_STATUS_REG_NEWDATAY)
        {
            error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                 LIS3DH_OUT_Y_L,
                                                 2, // 2 byte to be read
                                                 &Data[2]);
            if(error == NO_ERROR)
        {
            //OutY = (int16)((Data[2] | (Data[3]<<8)))>>4;
            //OutArray[3] = (uint8_t)(OutY & 0xFF);
            //OutArray[4] = (uint8_t)(OutY >> 8);
            
            OutY_ms2 = ((int16)((Data[2] | (Data[3]<<8)))>>4)*2/16;
            OutY_ms2 = ((int16)(OutY_ms2/(G/10000)));
            OutArray_ms2[3] = (uint8_t)(OutY_ms2 & 0xFF);
            OutArray_ms2[4] = (uint8_t)(OutY_ms2 >> 8);
        }
        }
        if((LIS3DH_STATUS_REG & LIS3DH_STATUS_REG_NEWDATAZ) == LIS3DH_STATUS_REG_NEWDATAZ)
        {
            error = I2C_Peripheral_ReadRegisterMulti(LIS3DH_DEVICE_ADDRESS,
                                                 LIS3DH_OUT_Z_L,
                                                 2, // 2 byte to be read
                                                 &Data[4]);
            if(error == NO_ERROR)
        {
            //OutZ = (int16)((Data[4] | (Data[5]<<8)))>>4;
            //OutArray[5] = (uint8_t)(OutZ & 0xFF);
            //OutArray[6] = (uint8_t)(OutZ >> 8);
            
            OutZ_ms2 = ((int16)((Data[4] | (Data[5]<<8)))>>4)*2/16;
            OutZ_ms2 = ((int16)(OutZ_ms2/(G/10000)));
            OutArray_ms2[5] = (uint8_t)(OutZ_ms2 & 0xFF);
            OutArray_ms2[6] = (uint8_t)(OutZ_ms2 >> 8);
        }
        }

        UART_Debug_PutArray(OutArray_ms2, 8);
        
    
    }
}


/* [] END OF FILE */
