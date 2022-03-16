/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    bridge.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _BRIDGE_H
#define _BRIDGE_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
    // DOM-IGNORE-END 

    // *****************************************************************************
    // *****************************************************************************
    // Section: Type Definitions
    // *****************************************************************************
    // *****************************************************************************

#define TX_WLAN_LIST_SIZE   20
#define TX_ETH_LIST_SIZE    5

#define WLAN_NET 0
#define ETH_NET  1    

#define BRIDGE_CONSOLE_PRINT(fmt, ...)    SYS_CONSOLE_PRINT("%s BRIDGE:      ",brdg.TimeStr);SYS_CONSOLE_PRINT(fmt, ##__VA_ARGS__);

#define BRDG_DHCP_CLIENT_TIMEOUT    6

    void BRIDGE_Wifi_Callback(uint32_t event, void * data, void *cookie);
    void * myCalloc(size_t, size_t);

    // *****************************************************************************

    /* Application states

      Summary:
        Application states enumeration

      Description:
        This enumeration defines the valid application states.  These states
        determine the behavior of the application at various times.
     */

    typedef enum {
        /* Application's state machine's initial state. */
        BRIDGE_STATE_INIT = 0,
        BRIDGE_STATE_WAIT_FOR_TCP_STACK_READY,
        BRIDGE_STATE_INIT_BRIDGE_MODE,
        BRIDGE_STATE_START_BRIDGING,
        BRIDGE_STATE_BRIDGE_MODE,
        BRIDGE_STATE_LEAVE_BRIDGE_MODE,
        BRIDGE_STATE_REINIT_NETWORK,
        BRIDGE_STATE_FILTER_ARP,
        BRIDGE_STATE_IDLE,
    } BRIDGE_STATES;

    typedef enum {
        ETH_DHCP_MODE_CLIENT,
        ETH_DHCP_MODE_SERVER
    } ETH_DHCP_MODE;

    typedef union {
        int64_t val64;

        struct {
            uint32_t lower32;
            uint32_t upper32;
        } val32;
    } INT64;
    
    // *****************************************************************************

    /* Application Data

      Summary:
        Holds application data

      Description:
        This structure holds the application's data.

      Remarks:
        Application strings and buffers are be defined outside this structure.
     */

    typedef struct {
        /* The application's current state */
        BRIDGE_STATES state;
        bool eth_with_ip_from_dhcp_only;
        TCPIP_NET_HANDLE wlan_net_hdl;
        TCPIP_NET_HANDLE eth_net_hdl;
        TCPIP_MAC_PACKET * wlan_tx_packets[TX_WLAN_LIST_SIZE];
        TCPIP_MAC_PACKET * eth_tx_packets[TX_ETH_LIST_SIZE];
        uint32_t wlan2eth;
        uint32_t wlan2ethBW;
        uint32_t eth2wlan;
        uint32_t eth2wlanBW;
        uint32_t wlan_ints_sum;
        bool trigger_every_second;
        bool eth_online;
        bool wlan_online;
        bool eth_dhcps_lease;
        bool wlan_dhcps_lease;
        bool eth_online_old;
        bool wlan_online_old;
        uint32_t eth_dhcp_client_timeout;
        uint32_t wlan_pack_count;
        uint32_t eth_pack_count;
        uint32_t wlan_drop_count;
        uint32_t wlan_rfmac_int_count;
        uint32_t wlan_rftm_int_count;
        uint32_t eth_int_count;
        uint32_t eth_drop_count;
        uint32_t w2e_count;
        uint32_t e2w_count;
        TCPIP_EVENT_HANDLE TCPIP_event_hdl;
        TCPIP_NET_HANDLE netH;
        SYS_STATUS tcpipStat;
        char TimeStr[10];
        uint32_t seconds;
        uint32_t minutes;
        uint32_t hours;
        bool status_display;
        bool block_data;
        bool stream_status;
        TCPIP_DHCP_HANDLE dhcp_hdl;
        const void* dhcp_eth_hParam;
    } BRIDGE_DATA;

    typedef struct {
        uint32_t magic;
        char msg[4096];
    } EXCEPT_MSG;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Application Callback Routines
    // *****************************************************************************
    // *****************************************************************************
    /* These routines are called by drivers when certain events occur.
     */

    // *****************************************************************************
    // *****************************************************************************
    // Section: Application Initialization and State Machine Functions
    // *****************************************************************************
    // *****************************************************************************

    void BRIDGE_Arp_Scan(void);

    /*******************************************************************************
      Function:
        void BRIDGE_Initialize ( void )

      Summary:
         MPLAB Harmony application initialization routine.

      Description:
        This function initializes the Harmony application.  It places the 
        application in its initial state and prepares it to run so that its 
        APP_Tasks function can be called.

      Precondition:
        All other system initialization routines should be called before calling
        this routine (in "SYS_Initialize").

      Parameters:
        None.

      Returns:
        None.

      Example:
        <code>
        BRIDGE_Initialize();
        </code>

      Remarks:
        This routine must be called from the SYS_Initialize function.
     */

    void BRIDGE_Initialize(void);


    /*******************************************************************************
      Function:
        void BRIDGE_Tasks ( void )

      Summary:
        MPLAB Harmony Demo application tasks function

      Description:
        This routine is the Harmony Demo application's tasks function.  It
        defines the application's state machine and core logic.

      Precondition:
        The system and application initialization ("SYS_Initialize") should be
        called before calling this.

      Parameters:
        None.

      Returns:
        None.

      Example:
        <code>
        BRIDGE_Tasks();
        </code>

      Remarks:
        This routine must be called from SYS_Tasks() routine.
     */

    void BRIDGE_Tasks(void);


    void BRIDGE_Clear_eth_Lease(void);
    void BRIDGE_Clear_wlan_Lease(void);
    void BRIDGE_SetNetConnectionState(TCPIP_NET_HANDLE hNet, TCPIP_EVENT event);

    void Wifi_callback(uint32_t event, void * data, void *cookie);

#endif /* _BRIDGE_H */

    //DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

