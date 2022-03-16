/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    bridge.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "bridge.h"
#include "tcpip/src/tcpip_packet.h"
#include "tcpip/src/tcpip_private.h"

#include "tcpip/src/arp_private.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define TCPIP_THIS_MODULE_ID    TCPIP_MODULE_NONE

#define BRIDGE_TIMER_SEC_COUNT      1000
#define BRIDGE_DELAY_1_SECOND       (1*(1000/BRIDGE_TIMER_SEC_COUNT))
#define BRIDGE_DELAY_2_SECONDS      (2*(1000/BRIDGE_TIMER_SEC_COUNT)) 


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

BRIDGE_DATA brdg;
volatile uint32_t display_flag __attribute__((persistent));
extern uint32_t LogHeapSize;
extern uint32_t LogHeapSizeMax;
extern EXCEPT_MSG last_expt_msg;

const void *MyWlanHandlerParam;
const void *MyEthHandlerParam;


uint32_t get_last_expt_msg(void);


char BridgeStates[][50] = {
    "BRIDGE_STATE_INIT",
    "BRIDGE_STATE_WAIT_FOR_TCP_STACK_READY",
    "BRIDGE_STATE_INIT_BRIDGE_MODE",
    "BRIDGE_STATE_START_BRIDGING",
    "BRIDGE_STATE_BRIDGE_MODE",
    "BRIDGE_STATE_LEAVE_BRIDGE_MODE",
    "BRIDGE_STATE_REINIT_NETWORK",
    "BRIDGE_STATE_FILTER_ARP",
    "BRIDGE_STATE_IDLE"
};

typedef struct
{
    size_t                  stgSize;        // size  + valid flag
    TCPIP_STACK_NET_IF_DCPT netDcptStg;     // configuration save
#if defined(TCPIP_STACK_USE_IPV6)
    uint8_t                 restoreBuff[sizeof(TCPIP_NETWORK_CONFIG) + 170]; // buffer to restore the configuration
#else
    uint8_t                 restoreBuff[sizeof(TCPIP_NETWORK_CONFIG) + 120]; // buffer to restore the configuration
#endif  // defined(TCPIP_STACK_USE_IPV6)
}TCPIP_COMMAND_STG_DCPT;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

bool BRIDGE_pktWlanHandler(TCPIP_NET_HANDLE hNet, struct _tag_TCPIP_MAC_PACKET* rxPkt, uint16_t frameType, const void* hParam);
bool BRIDGE_pktEthHandler(TCPIP_NET_HANDLE hNet, struct _tag_TCPIP_MAC_PACKET* rxPkt, uint16_t frameType, const void* hParam);

TCPIP_MAC_RES BRIDGE_ETHMAC_PIC32MACPacketSingleTx(DRV_HANDLE hMac, TCPIP_MAC_PACKET * pNewPacket);

void BRIDGE_Display_Status(void);
SYS_STATUS BRIDGE_is_TCPSTACK_Ready(void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


bool pktEthHandler(TCPIP_NET_HANDLE hNet, struct _tag_TCPIP_MAC_PACKET* rxPkt, uint16_t frameType, const void* hParam) {
    ARP_PACKET      *pArpPkt;
    bool ret_val = false;
    
    // Obtain the incoming ARP packet and process
    pArpPkt = (ARP_PACKET*)rxPkt->pNetLayer;
    
    pArpPkt->HardwareType     = TCPIP_Helper_htons(pArpPkt->HardwareType);
    pArpPkt->Protocol         = TCPIP_Helper_htons(pArpPkt->Protocol);
    pArpPkt->Operation        = TCPIP_Helper_htons(pArpPkt->Operation);

    // Validate the ARP packet
    if ( pArpPkt->HardwareType == HW_ETHERNET     &&
            pArpPkt->MACAddrLen == sizeof(TCPIP_MAC_ADDR)  &&
            pArpPkt->ProtocolLen == sizeof(IPV4_ADDR) )
    {
        SYS_CONSOLE_PRINT("[%s] arp packet, mac addr = %x:%x:%x:%x:%x:%x\r\n", __func__, pArpPkt->SenderMACAddr.v[0], pArpPkt->SenderMACAddr.v[1], pArpPkt->SenderMACAddr.v[2], pArpPkt->SenderMACAddr.v[3], pArpPkt->SenderMACAddr.v[4], pArpPkt->SenderMACAddr.v[5]);
    
        ret_val = TCPIP_STACK_NetAddressMacSet(brdg.wlan_net_hdl, &pArpPkt->SenderMACAddr);
        if (ret_val == false) {
            SYS_CONSOLE_PRINT("Set MAC address failed\r\n");
        }
        else
        {
            SYS_CONSOLE_PRINT("Set MAC address success\r\n");
            brdg.state = BRIDGE_STATE_REINIT_NETWORK;
        }
    }

    TCPIP_PKT_PacketAcknowledge(rxPkt, TCPIP_MAC_PKT_ACK_RX_OK);
    return true;
}


void BRIDGE_TcpipStack_EventHandler(TCPIP_NET_HANDLE hNet, TCPIP_EVENT event, const void *fParam) {
    const char *netName = TCPIP_STACK_NetNameGet(hNet);
    int ix;
    
    BRIDGE_CONSOLE_PRINT("TCP Stack Event Handler %s - %x - ", netName, event);
    if (event & TCPIP_EV_CONN_ESTABLISHED) {
        SYS_CONSOLE_PRINT("connection established\r\n");
        if (hNet == brdg.eth_net_hdl) {
            brdg.eth_online = true;
            
            for (ix = 0; ix < TX_WLAN_LIST_SIZE; ix++) {
                brdg.wlan_tx_packets[ix]->ackParam = (void*) 0xFF;
            }

            for (ix = 0; ix < TX_ETH_LIST_SIZE; ix++) {
                brdg.eth_tx_packets[ix]->ackParam = (void*) 0xFF;
            }
            
        } else if (hNet == brdg.wlan_net_hdl) {
            BRIDGE_CONSOLE_PRINT("action unknown1\r\n");
        }

    } else if (event & TCPIP_EV_CONN_LOST) {
        SYS_CONSOLE_PRINT("connection lost\r\n");
        if (hNet == brdg.eth_net_hdl) {
            brdg.eth_online = false; 
        } else if (hNet == brdg.wlan_net_hdl) {
            BRIDGE_CONSOLE_PRINT("action unknown2\r\n");
        }
    } else {
        BRIDGE_CONSOLE_PRINT("TCP Stack Event Handler %s Unknown event = %d\r\n", netName, event);
    }

}



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void BRIDGE_Initialize ( void )

  Remarks:
    See prototype in bridge.h.
 */

void BRIDGE_Initialize(void) {

    /* Place the App state machine in its initial state. */
    memset(&brdg, 0, sizeof (BRIDGE_DATA));

    brdg.wlan_net_hdl = TCPIP_STACK_IndexToNet(WLAN_NET);
    brdg.eth_net_hdl = TCPIP_STACK_IndexToNet(ETH_NET);
    brdg.status_display = false;
    brdg.block_data = false;

    int ix;
    for (ix = 0; ix < TX_WLAN_LIST_SIZE; ix++) {
        brdg.wlan_tx_packets[ix] = _TCPIP_PKT_PacketAlloc(sizeof (TCPIP_MAC_PACKET), 1614, 0);
        brdg.wlan_tx_packets[ix]->ackParam = (void*) 0xFF;
    }

    for (ix = 0; ix < TX_ETH_LIST_SIZE; ix++) {
        brdg.eth_tx_packets[ix] = _TCPIP_PKT_PacketAlloc(sizeof (TCPIP_MAC_PACKET), 1614, 0);
        brdg.eth_tx_packets[ix]->ackParam = (void*) 0xFF;
    }

    sprintf(brdg.TimeStr, "%02d:%02d:%02d", brdg.hours, brdg.minutes, brdg.seconds);

}

/******************************************************************************
  Function:
    void BRIDGE_Tasks ( void )

  Remarks:
    See prototype in bridge.h.
 */

void BRIDGE_Tasks(void) {
    static BRIDGE_STATES old_state = BRIDGE_STATE_INIT;
    SYS_MODULE_OBJ      tcpipStackObj;
    TCPIP_NETWORK_CONFIG*     pNetConf;    
    TCPIP_STACK_INIT    tcpip_init_data = {{0}};
    TCPIP_STACK_NET_IF_DCPT netDcptStg;
    uint8_t                 restoreBuff[sizeof(TCPIP_NETWORK_CONFIG) + 120]; // buffer to restore the configuration

    if (brdg.state != old_state) {
        old_state = brdg.state;
        BRIDGE_CONSOLE_PRINT("new state: %s\n\r", &BridgeStates[brdg.state][0]);
    }

    if (brdg.trigger_every_second) {
        brdg.trigger_every_second = false;
        BRIDGE_Display_Status();
    }

    /* Check the application's current state. */
    switch (brdg.state) {
            /* Application's initial state. */
        case BRIDGE_STATE_INIT:
        {
            bool appInitialized = true;
            BRIDGE_CONSOLE_PRINT("Bridge State Init\n\r");
            if (appInitialized) {
                brdg.eth_dhcp_client_timeout = BRDG_DHCP_CLIENT_TIMEOUT;
                brdg.state = BRIDGE_STATE_WAIT_FOR_TCP_STACK_READY;
            }
            brdg.eth_dhcps_lease = false;
            break;
        }

        case BRIDGE_STATE_WAIT_FOR_TCP_STACK_READY:
        {
            if (BRIDGE_is_TCPSTACK_Ready() == SYS_STATUS_READY) {
                SYS_CONSOLE_PRINT(
                        "======================================================\n\r");
                SYS_CONSOLE_PRINT("L2 Bridge Build Time  " __DATE__ " " __TIME__ "\n\r");

                BRIDGE_CONSOLE_PRINT("TCP Stack Ready\n\r");
                BRIDGE_CONSOLE_PRINT("Handle: eth %04x wlan %04x\n\r", brdg.eth_net_hdl, brdg.wlan_net_hdl);

                brdg.state = BRIDGE_STATE_FILTER_ARP; 
            }
            break;
        }
        
        case BRIDGE_STATE_FILTER_ARP:

            TCPIP_STACK_PacketHandlerRegister(TCPIP_STACK_IndexToNet(ETH_NET), pktEthHandler, MyEthHandlerParam);
            brdg.state = BRIDGE_STATE_IDLE;
            break;

        case BRIDGE_STATE_INIT_BRIDGE_MODE:
            if (brdg.eth_with_ip_from_dhcp_only == true) {
                brdg.state = BRIDGE_STATE_IDLE;
            } else {
                TCPIP_STACK_PacketHandlerRegister(brdg.wlan_net_hdl, BRIDGE_pktWlanHandler, MyWlanHandlerParam);
                TCPIP_STACK_PacketHandlerRegister(brdg.eth_net_hdl, BRIDGE_pktEthHandler, MyEthHandlerParam);
                brdg.state = BRIDGE_STATE_BRIDGE_MODE;
            }
            break;

        case BRIDGE_STATE_BRIDGE_MODE:

            break;

        case BRIDGE_STATE_LEAVE_BRIDGE_MODE:
            TCPIP_STACK_PacketHandlerDeregister(brdg.wlan_net_hdl, BRIDGE_pktWlanHandler);
            TCPIP_STACK_PacketHandlerDeregister(brdg.eth_net_hdl, BRIDGE_pktEthHandler);
            TCPIP_DHCP_Disable(brdg.eth_net_hdl);
            brdg.state = BRIDGE_STATE_IDLE;
            break;

        case BRIDGE_STATE_REINIT_NETWORK:

            TCPIP_STACK_PacketHandlerDeregister(brdg.eth_net_hdl, pktEthHandler);  

            /* network down */
            
            TCPIP_STACK_NetConfigGet(brdg.wlan_net_hdl, &netDcptStg, sizeof(netDcptStg), 0);

            if(TCPIP_STACK_NetIsUp(brdg.wlan_net_hdl) != 0)
            {
                SYS_CONSOLE_PRINT("NetDown\n\r");
                TCPIP_STACK_NetDown(brdg.wlan_net_hdl);
            }

            /* network up */
            
            // get the data passed at initialization
            tcpipStackObj = TCPIP_STACK_Initialize(0, 0);
            TCPIP_STACK_InitializeDataGet(tcpipStackObj, &tcpip_init_data);
            if(tcpip_init_data.pNetConf == 0)
            {
                SYS_CONSOLE_PRINT("Operation failed. No configuration\r\n");
            }

            pNetConf = TCPIP_STACK_NetConfigSet(&netDcptStg, restoreBuff, sizeof(restoreBuff), 0);
            // change the power mode to FULL
            pNetConf->powerMode = TCPIP_STACK_IF_POWER_FULL;

            TCPIP_STACK_NetUp(brdg.wlan_net_hdl, pNetConf);
            SYS_CONSOLE_PRINT("NetUp\n\r");
            
            sysObj.syswifi = SYS_WIFI_Initialize(NULL,BRIDGE_Wifi_Callback,NULL);
 
            brdg.state = BRIDGE_STATE_START_BRIDGING;
            
            break;
           
        case BRIDGE_STATE_START_BRIDGING:
            
            if (SYS_WIFI_GetStatus(sysObj.syswifi) > SYS_WIFI_STATUS_WDRV_OPEN_REQ)
            {
                brdg.state = BRIDGE_STATE_INIT_BRIDGE_MODE;
            }
            break;
       

        case BRIDGE_STATE_IDLE:
            break;

            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

bool MyWLANAckFunction(TCPIP_MAC_PACKET* pkt, const void* param) {
    brdg.wlan_pack_count--;
    pkt->ackParam = (void*) 0xFF;
    return false;
}

bool BRIDGE_pktWlanHandler(TCPIP_NET_HANDLE hNet, struct _tag_TCPIP_MAC_PACKET* rxPkt, uint16_t frameType, const void* hParam) {
    bool ret_val = false;
    TCPIP_NET_IF *pifeth = (TCPIP_NET_IF *) TCPIP_STACK_IndexToNet(ETH_NET);
    int res = 0;
    int HeapFreeSize;

    HeapFreeSize = xPortGetFreeHeapSize();

    if (HeapFreeSize < 30000) {
        TCPIP_PKT_PacketAcknowledge(rxPkt, TCPIP_MAC_PKT_ACK_RX_OK);
        brdg.wlan_drop_count++;
        return true;
    }

    if (brdg.state != BRIDGE_STATE_BRIDGE_MODE) {
        ret_val = false;
    } else {
        ret_val = true;
        brdg.w2e_count++;
        if (brdg.block_data == false) {

            res = (int) BRIDGE_ETHMAC_PIC32MACPacketSingleTx(pifeth->hIfMac, rxPkt);
            if (res != 0) {
                TCPIP_PKT_PacketAcknowledge(rxPkt, TCPIP_MAC_PKT_ACK_RX_OK);
            }
        } else {
            TCPIP_PKT_PacketAcknowledge(rxPkt, TCPIP_MAC_PKT_ACK_RX_OK);
        }
    }

    return ret_val;
}


bool BRIDGE_pktEthHandler(TCPIP_NET_HANDLE hNet, struct _tag_TCPIP_MAC_PACKET* rxPkt, uint16_t frameType, const void* hParam) {
    bool ret_val = false;
    uint32_t len;
    uint8_t *puc_s;
    uint8_t *puc_t;
    TCPIP_NET_IF *pifwlan = (TCPIP_NET_IF *) TCPIP_STACK_IndexToNet(WLAN_NET);
    struct _tag_TCPIP_MAC_PACKET Dummy_txPktWlanTemp;
    struct _tag_TCPIP_MAC_PACKET * txPktWlanTemp = &Dummy_txPktWlanTemp;
    int ix;
    int HeapFreeSize;

    HeapFreeSize = xPortGetFreeHeapSize();

    if (HeapFreeSize < 10000) {
        TCPIP_PKT_PacketAcknowledge(rxPkt, TCPIP_MAC_PKT_ACK_RX_OK);
        brdg.eth_drop_count++;
        return true;
    }

    if (brdg.state != BRIDGE_STATE_BRIDGE_MODE) {
        ret_val = false;
    } else {
        ret_val = true;
        brdg.e2w_count++;

        for (ix = 0; ix < TX_WLAN_LIST_SIZE; ix++) {
            if (brdg.wlan_tx_packets[ix]->ackParam == (void*) 0xFF) {
                brdg.wlan_tx_packets[ix]->ackParam = (void*) ix;
                txPktWlanTemp = brdg.wlan_tx_packets[ix];
                break;
            }
        }
        if ((ix == TX_WLAN_LIST_SIZE) || (brdg.block_data == true)) {
            brdg.eth_drop_count++;
            TCPIP_PKT_PacketAcknowledge(rxPkt, TCPIP_MAC_PKT_ACK_RX_OK);

            for (ix = 0; ix < TX_WLAN_LIST_SIZE; ix++) {
                brdg.wlan_tx_packets[ix]->ackParam = (void*) 0xFF;
            }
            return ret_val;
        }
        brdg.wlan_pack_count++;

        // copy over data
        txPktWlanTemp->pkt_next = 0;
        txPktWlanTemp->pktIf = pifwlan;
        txPktWlanTemp->ackFunc = MyWLANAckFunction;
        txPktWlanTemp->pktFlags = TCPIP_MAC_PKT_FLAG_QUEUED | TCPIP_MAC_PKT_FLAG_TX;
        txPktWlanTemp->pDSeg->segLen = rxPkt->pDSeg->segLen + 14;
        //txPktWlanTemp->pDSeg->segLoadOffset = (((uint32_t) rxPkt->pMacLayer - (uint32_t) rxPkt) - 68);
        len = txPktWlanTemp->pDSeg->segLen;
        brdg.eth2wlan += len;
        puc_s = rxPkt->pMacLayer;
        puc_t = txPktWlanTemp->pMacLayer;
        
        //SYS_CONSOLE_PRINT("ETH: mac data 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", puc_s[0], puc_s[1], puc_s[2], puc_s[3], puc_s[4], puc_s[5], puc_s[6], puc_s[7]);

        memcpy(puc_t, puc_s, len);

        int res = (int) ((*pifwlan->pMacObj->TCPIP_MAC_PacketTx)(pifwlan->hIfMac, txPktWlanTemp));
        if (res != 0) {
            TCPIP_PKT_PacketAcknowledge(txPktWlanTemp, TCPIP_MAC_PKT_ACK_RX_OK);
        }

        TCPIP_PKT_PacketAcknowledge(rxPkt, TCPIP_MAC_PKT_ACK_RX_OK);
    }

    return ret_val;
}

void BRIDGE_Display_Status(void) {
    char str[256];
    display_flag = 0xAFFEAFFE;

    if (++brdg.seconds == 60) {
        brdg.seconds = 0;
        if (++brdg.minutes == 59) {
            brdg.minutes = 0;
            brdg.hours++;
        }
    }

    sprintf(brdg.TimeStr, "%02d:%02d:%02d", brdg.hours, brdg.minutes, brdg.seconds);

    if (brdg.status_display == true) {
        int heapSize = xPortGetFreeHeapSize();

        sprintf(str,
                "Time:%02d:%02d:%02d Heap:%d Int:%d/%d(wlanrf)%d(wlantm)%d(eth) Pkt:%d(w2e)%d(e2w)\n\r"
                , brdg.hours, brdg.minutes, brdg.seconds, heapSize
                , brdg.wlan_rfmac_int_count, brdg.wlan_ints_sum, brdg.wlan_rftm_int_count
                , brdg.eth_int_count
                , brdg.w2e_count, brdg.e2w_count
                );

        SYS_CONSOLE_PRINT(str);

        brdg.eth2wlanBW = (brdg.eth2wlan * 8) >> 10;
        brdg.wlan2ethBW = (brdg.wlan2eth * 8) >> 10;
        brdg.eth2wlan = 0;
        brdg.wlan2eth = 0;
        brdg.wlan_ints_sum += brdg.wlan_rfmac_int_count;
        brdg.wlan_rfmac_int_count = 0;
        
        sprintf(str,
                "Mode:%s Buffer:%d(eth)%d(wlan) Drop:%d(wlan)%d(eth) BW:%d(e2w)%d(w2e)\n\r"
                , BridgeStates[brdg.state], brdg.eth_pack_count, brdg.wlan_pack_count
                , brdg.wlan_drop_count, brdg.eth_drop_count, brdg.eth2wlanBW, brdg.wlan2ethBW);

        SYS_CONSOLE_PRINT(str);

        display_flag = 0;
    }

}

#include ".\config\pic32mz_w1_curiosity_freertos\driver\ethmac\src\dynamic\_eth_dcpt_lists.h"
#include ".\config\pic32mz_w1_curiosity_freertos\driver\ethmac\src\dynamic\drv_eth_pic32_lib.h"
#include ".\config\pic32mz_w1_curiosity_freertos\driver\ethmac\src\drv_ethmac_local.h"
#include ".\config\pic32mz_w1_curiosity_freertos\driver\ethmac\src\dynamic\drv_ethmac_lib.h"

#if 0
static __inline__ void __attribute__((always_inline)) _DRV_ETHMAC_TxLock(DRV_ETHMAC_INSTANCE_DCPT* pMacD) {
    if (pMacD->mData._synchF != 0) {
        (*pMacD->mData._synchF)(&pMacD->mData._syncTxH, TCPIP_MAC_SYNCH_REQUEST_OBJ_LOCK);
    }
}
#endif

static void _MACTxPacketAckCallback(void* pBuff, void* fParam) {

    // restore packet the buffer belongs to 
    uint16_t buffOffset = *((uint16_t*) pBuff - 1);
    TCPIP_MAC_PACKET* ptrPacket = (TCPIP_MAC_PACKET*) ((uint8_t*) pBuff - buffOffset);

    // acknowledge the packet
    DRV_ETHMAC_INSTANCE_DCPT* pMacD = (DRV_ETHMAC_INSTANCE_DCPT*) fParam;
    (*pMacD->mData.pktAckF)(ptrPacket, TCPIP_MAC_PKT_ACK_TX_OK, TCPIP_THIS_MODULE_ID);
    pMacD->mData._txStat.nTxOkPackets++;

}

static void _MACTxAcknowledgeEth(DRV_ETHMAC_INSTANCE_DCPT* pMacD) { 
    DRV_ETHMAC_LibTxAcknowledgePacket(pMacD, 0, _MACTxPacketAckCallback, pMacD);
}

static void _MacTxDiscardQueues(DRV_ETHMAC_INSTANCE_DCPT* pMacD, TCPIP_MAC_PKT_ACK_RES ackRes) {
    TCPIP_MAC_PACKET* pPkt;

    while ((pPkt = (TCPIP_MAC_PACKET*) DRV_ETHMAC_SingleListHeadRemove(&pMacD->mData._TxQueue)) != 0) { // acknowledge the packet
        (*pMacD->mData.pktAckF)(pPkt, ackRes, TCPIP_THIS_MODULE_ID);
    }
}

static TCPIP_MAC_RES _MACTxPacket(DRV_ETHMAC_INSTANCE_DCPT* pMacD, TCPIP_MAC_PACKET * ptrPacket) {
    DRV_ETHMAC_RESULT ethRes;

    // Note: the TCPIP_MAC_DATA_SEGMENT is defined to be a perfect match for DRV_ETHMAC_PKT_DCPT !!!
    ethRes = DRV_ETHMAC_LibTxSendPacket(pMacD, (const DRV_ETHMAC_PKT_DCPT*) ptrPacket->pDSeg);

    if (ethRes == DRV_ETHMAC_RES_OK) {
        return TCPIP_MAC_RES_OK;
    } else if (ethRes == DRV_ETHMAC_RES_NO_DESCRIPTORS) {
        pMacD->mData._txStat.nTxQueueFull++;
        return TCPIP_MAC_RES_PENDING;
    }

    pMacD->mData._txStat.nTxErrorPackets++;
    return TCPIP_MAC_RES_PACKET_ERR;
}

static TCPIP_MAC_RES _MacTxPendingPackets(DRV_ETHMAC_INSTANCE_DCPT* pMacD) {
    TCPIP_MAC_PACKET* pPkt;
    TCPIP_MAC_RES pktRes;

    if (pMacD->mData._macFlags._linkPrev == false) { // discard the TX queues
        _MacTxDiscardQueues(pMacD, TCPIP_MAC_PKT_ACK_LINK_DOWN);
        // no need to try to schedule for TX
        return TCPIP_MAC_RES_PENDING;
    }


    while ((pPkt = (TCPIP_MAC_PACKET*) (pMacD->mData._TxQueue.head)) != 0) {
        pktRes = _MACTxPacket(pMacD, pPkt);
        if (pktRes == TCPIP_MAC_RES_PENDING) { // not enough room in the hw queue
            return TCPIP_MAC_RES_PENDING;
        }

        // on way or another we're done with this packet
        DRV_ETHMAC_SingleListHeadRemove(&pMacD->mData._TxQueue);
        if (pktRes != TCPIP_MAC_RES_OK) { // not transmitted
            (*pMacD->mData.pktAckF)(pPkt, TCPIP_MAC_PKT_ACK_BUFFER_ERR, TCPIP_THIS_MODULE_ID);
        }
    }


    return TCPIP_MAC_RES_OK;
}
#if 0
static __inline__ void __attribute__((always_inline)) _DRV_ETHMAC_TxUnlock(DRV_ETHMAC_INSTANCE_DCPT* pMacD) {
    if (pMacD->mData._synchF != 0) {
        (*pMacD->mData._synchF)(&pMacD->mData._syncTxH, TCPIP_MAC_SYNCH_REQUEST_OBJ_UNLOCK);
    }
}
#endif
static bool MyEthMACAckFunction(TCPIP_MAC_PACKET* pkt, const void* param) {
    brdg.eth_pack_count--;
    pkt->ackParam = (void*) 0xFF;
    return false;
}


TCPIP_MAC_RES BRIDGE_ETHMAC_PIC32MACPacketSingleTx(DRV_HANDLE hMac, TCPIP_MAC_PACKET * pNewPacket) {
    TCPIP_MAC_RES macRes;
    TCPIP_MAC_DATA_SEGMENT* pSeg;
    uint16_t* pHdrSpace;
    DRV_ETHMAC_INSTANCE_DCPT* pMacD = (DRV_ETHMAC_INSTANCE_DCPT*) hMac;
    TCPIP_MAC_PACKET * pTxPacketp;
    int length;
    uint8_t *puc_source, *puc_target;
    TCPIP_NET_IF *pifeth = (TCPIP_NET_IF *) TCPIP_STACK_IndexToNet(ETH_NET);
    int ix;

    TCPIP_MAC_PACKET* pPktIt = pNewPacket;
    ix = 0;

    if (pPktIt->next) {
        while (pPktIt) {
            pPktIt = pPktIt->next;
        }
    }

    //_DRV_ETHMAC_TxLock(pMacD);
    _MACTxAcknowledgeEth(pMacD);

#if defined(ETH_PIC32_INT_MAC_ISR_TX)
    _MACTxAcknowledgeAckQueue(pMacD);
#endif  // defined(ETH_PIC32_INT_MAC_ISR_TX)

    _DRV_ETHMAC_TxLock(pMacD);
    // transmit the pending packets...don't transmit out of order
    macRes = _MacTxPendingPackets(pMacD);

    for (ix = 0; ix < TX_ETH_LIST_SIZE; ix++) {
        if (brdg.eth_tx_packets[ix]->ackParam == (void*) 0xFF) {
            brdg.eth_tx_packets[ix]->ackParam = (void*) ix;
            pTxPacketp = brdg.eth_tx_packets[ix];
            break;
        }
    }
    if (ix == TX_ETH_LIST_SIZE) {
        brdg.wlan_drop_count++;
        TCPIP_PKT_PacketAcknowledge(pNewPacket, TCPIP_MAC_PKT_ACK_RX_OK);
        _DRV_ETHMAC_TxUnlock(pMacD);
        return TCPIP_MAC_RES_OK;
    }
    brdg.eth_pack_count++;
   
    // copy over data
    pTxPacketp->pkt_next = 0;
    pTxPacketp->pktIf = pifeth;
    pTxPacketp->ackFunc = MyEthMACAckFunction;
    pTxPacketp->pktFlags = TCPIP_MAC_PKT_FLAG_QUEUED | TCPIP_MAC_PKT_FLAG_TX;
    pTxPacketp->pDSeg->segLen = pNewPacket->pDSeg->segLen + 14;
    //pTxPacketp->pDSeg->segLoadOffset = (((uint32_t) pNewPacket->pMacLayer - (uint32_t) pNewPacket) - 68);
    length = pTxPacketp->pDSeg->segLen;
    brdg.wlan2eth += length;
    puc_source = pNewPacket->pMacLayer;
    puc_target = pTxPacketp->pMacLayer;
    
    memcpy(puc_target, puc_source, length);
    //SYS_CONSOLE_PRINT("WiFi: mac data 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", puc_source[0], puc_source[1], puc_source[2], puc_source[3], puc_source[4], puc_source[5], puc_source[6], puc_source[7]);


    pSeg = pTxPacketp->pDSeg;
    if (pSeg == 0) { // cannot send this packet
        _DRV_ETHMAC_TxUnlock(pMacD);
        return TCPIP_MAC_RES_PACKET_ERR;
    }
    // store packet info
    pHdrSpace = (uint16_t*) pSeg->segLoad - 1;
    *pHdrSpace = (uint8_t*) pSeg->segLoad - (uint8_t*) pTxPacketp;

    // And process them
    if (pTxPacketp && macRes == TCPIP_MAC_RES_OK) { // can schedule some packets
        // set the queue flag; avoid race condition if MACTx is really fast;
        pTxPacketp->pktFlags |= TCPIP_MAC_PKT_FLAG_QUEUED;
        macRes = _MACTxPacket(pMacD, pTxPacketp);

        if (macRes == TCPIP_MAC_RES_PACKET_ERR) { // no longer in our queue
            pNewPacket->pktFlags &= ~TCPIP_MAC_PKT_FLAG_QUEUED;
            _DRV_ETHMAC_TxUnlock(pMacD);
            return TCPIP_MAC_RES_PACKET_ERR;
        }
    }

    // Kill old MAC                  

    TCPIP_PKT_PacketAcknowledge(pNewPacket, TCPIP_MAC_PKT_ACK_RX_OK);

    _DRV_ETHMAC_TxUnlock(pMacD);

    return TCPIP_MAC_RES_OK;
}


void BRIDGE_Wifi_Callback(uint32_t event, void * data, void *cookie) {
    SYS_CONSOLE_PRINT("BRIDGE_Wifi_Callback\n\r");
    switch (event) {
        case SYS_WIFI_DISCONNECT:
            BRIDGE_CONSOLE_PRINT("WiFi Event DISCONNECT\n\r");
            brdg.wlan_dhcps_lease = false;
            brdg.wlan_online = false;
            
            break;
        case SYS_WIFI_CONNECT:
            BRIDGE_CONSOLE_PRINT("WiFi Event CONNECT\n\r");
            brdg.wlan_online = true;
            break;
    }

}

void * myCalloc(size_t n, size_t size) {
    void * ret_val;
    int count;

    count = n * size;
    ret_val = pvPortMalloc(count);
    memset(ret_val, 0, count);

    return ret_val;
}


SYS_STATUS BRIDGE_is_TCPSTACK_Ready(void) {
    int i, nNets;
    SYS_STATUS ret_val = SYS_STATUS_UNINITIALIZED;

    brdg.tcpipStat = TCPIP_STACK_Status(sysObj.tcpip);
    if (SYS_STATUS_READY == brdg.tcpipStat) {
        nNets = TCPIP_STACK_NumberOfNetworksGet();
        for (i = 0; i < nNets; i++) {
            brdg.netH = TCPIP_STACK_IndexToNet(i);
            brdg.TCPIP_event_hdl = TCPIP_STACK_HandlerRegister(brdg.netH, TCPIP_EV_CONN_ALL, BRIDGE_TcpipStack_EventHandler, NULL);
        }
        ret_val = brdg.tcpipStat;
    }
    return ret_val;
}



/*******************************************************************************
 End of File
 */
