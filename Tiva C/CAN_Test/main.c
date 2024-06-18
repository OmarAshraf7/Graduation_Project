#include <stdint.h>
#include <stdbool.h>

#include "tm4c123gh6pm.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_can.h"
#include "hw_ints.h"
#include "can.h"
#include "interrupt.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "uartstdio.h"

volatile bool txFlag = 0; /* msg sent flag */
volatile bool rxFlag = 0; /* msg recieved flag */
volatile bool errFlag = 0; /* error flag */

/* Define arrays to store received frames */
unsigned char ESP1_Frame[8];
unsigned char ESP2_Frame[8];
unsigned char ESP3_Frame[8];
unsigned char ESP4_Frame[8];

unsigned char sendData[8] = {'h', 'e' , 'l', 't', 'i', 'v', 's'};

void sendCANMessage(uint32_t id, uint8_t* data, uint8_t length)
{
    tCANMsgObject msg;
    msg.ui32MsgID = id;
    msg.ui32MsgIDMask = 0;
    msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    msg.ui32MsgLen = length;
    msg.pui8MsgData = data;

    CANMessageSet(CAN0_BASE, 2, &msg, MSG_OBJ_TYPE_TX); // use message object 2 for transmission
}

/* CAN interrupt handler */
void CANIntHandler(void)
{
    unsigned long status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE); /* read interrupt status */

    if(status == CAN_INT_INTID_STATUS) { /* controller status interrupt */
        status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        errFlag = 1;
    }
    else if(status == 1)
    { /* msg object 1 */
        CANIntClear(CAN0_BASE, 1); // clear interrupt
        rxFlag = 1; // set rx flag
        errFlag = 0; // clear any error flags
    }
    else if (status == 2)
    { /* msg object 2 (transmission) */
        CANIntClear(CAN0_BASE, 2); // clear interrupt
        txFlag = 1; // set tx flag
        errFlag = 0; // clear any error flags
    }
    else
    {
        // should never happen
    }

}


int main(void)
{
    tCANMsgObject msg; // the CAN msg object
    unsigned char msgData[8]; // 8 byte buffer for rx message data

    // Run from crystal at 50Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Set up CAN0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    CANInit(CAN0_BASE);
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
    CANIntRegister(CAN0_BASE, CANIntHandler); // use dynamic vector table allocation
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);


    // Use ID and mask 0 to recieved messages with any CAN ID
    msg.ui32MsgID = 0;
    msg.ui32MsgIDMask = 0;
    msg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    msg.ui32MsgLen = 8; // allow up to 8 bytes

    // Load msg into CAN peripheral message object 1 so it can trigger interrupts on any matched rx messages
    CANMessageSet(CAN0_BASE, 1, &msg, MSG_OBJ_TYPE_RX);


    while(1) {

        if(rxFlag) { // rx interrupt has occured

            msg.pui8MsgData = msgData; // set pointer to rx buffer
            CANMessageGet(CAN0_BASE, 1, &msg, 0); // read CAN message object 1 from CAN peripheral

            rxFlag = 0; // clear rx flag

            // Process the received message and store it in the corresponding array
            switch (msg.ui32MsgID)
            {
            case 0x12:
                memcpy(ESP1_Frame, msgData, 8);
                break;
            case 0x13:
                memcpy(ESP2_Frame, msgData, 8);
                break;
            case 0x14:
                memcpy(ESP3_Frame, msgData, 8);
                break;
            case 0x15:
                memcpy(ESP4_Frame, msgData, 8);
                break;
            default:
                // Handle unexpected ID
                break;
            }

            if(msg.ui32Flags & MSG_OBJ_DATA_LOST)
            {
              // check msg flags for any lost messages
              // UARTprintf("CAN message loss detected\n");
            }
        }

        sendCANMessage(0x10, sendData, 8);

        if (txFlag)
        {
            // Transmission was successful
            txFlag = 0; // clear tx flag
            // You can add additional actions here if needed
        }
    }
}
