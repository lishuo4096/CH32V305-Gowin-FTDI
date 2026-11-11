/********************************** (C) COPYRIGHT *******************************
* File Name          : usb_desc.h
* Description        : USB descriptors for Alternative_FT232H
*                      Emulates an FTDI FT2232D:
*                        interface 0 = MPSSE / JTAG  (bulk EP 0x02 OUT / 0x81 IN)
*                        interface 1 = UART / VCP    (bulk EP 0x04 OUT / 0x83 IN)
*                      VID/PID = 0403:6010 so that the FTDI signed drivers
*                      (ftdibus.sys + ftdiport.sys) bind automatically.
*******************************************************************************/
#ifndef __USB_DESC_H
#define __USB_DESC_H

#include "debug.h"

/* usb device info */
#define DEF_USB_VID                 0x0403
#define DEF_USB_PID                 0x6010
#define DEF_IC_PRG_VER              0x00

/* endpoint 0 / bulk packet sizes */
#define DEF_USBD_UEP0_SIZE          64
#define DEF_USBD_HS_PACK_SIZE       512
#define DEF_USBD_FS_PACK_SIZE       64
#define DEF_USBD_HS_ISO_PACK_SIZE   1024
#define DEF_USBD_FS_ISO_PACK_SIZE   1023

#define DEF_USB_EP1_HS_SIZE         DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP2_HS_SIZE         DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP3_HS_SIZE         DEF_USBD_HS_PACK_SIZE
#define DEF_USB_EP4_HS_SIZE         DEF_USBD_HS_PACK_SIZE

#define DEF_USB_EP1_FS_SIZE         DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP2_FS_SIZE         DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP3_FS_SIZE         DEF_USBD_FS_PACK_SIZE
#define DEF_USB_EP4_FS_SIZE         DEF_USBD_FS_PACK_SIZE

/* descriptor lengths */
#define DEF_USBD_DEVICE_DESC_LEN    ((uint16_t)MyDevDescr[0])
#define DEF_USBD_CONFIG_FS_DESC_LEN ((uint16_t)MyCfgDescr_FS[2] + (uint16_t)(MyCfgDescr_FS[3] << 8))
#define DEF_USBD_CONFIG_HS_DESC_LEN ((uint16_t)MyCfgDescr_HS[2] + (uint16_t)(MyCfgDescr_HS[3] << 8))
#define DEF_USBD_LANG_DESC_LEN      ((uint16_t)MyLangDescr[0])
#define DEF_USBD_MANU_DESC_LEN      ((uint16_t)MyManuInfo[0])
#define DEF_USBD_PROD_DESC_LEN      ((uint16_t)MyProdInfo[0])
#define DEF_USBD_SN_DESC_LEN        ((uint16_t)MySerNumInfo[0])
#define DEF_USBD_QUALFY_DESC_LEN    ((uint16_t)MyQuaDesc[0])
#define DEF_USBD_BOS_DESC_LEN       ((uint16_t)MyBOSDesc[2] + (uint16_t)(MyBOSDesc[3] << 8))
#define DEF_USBD_FS_OTH_DESC_LEN    (DEF_USBD_CONFIG_HS_DESC_LEN)
#define DEF_USBD_HS_OTH_DESC_LEN    (DEF_USBD_CONFIG_FS_DESC_LEN)

extern const uint8_t MyDevDescr[];
extern const uint8_t MyCfgDescr_FS[];
extern const uint8_t MyCfgDescr_HS[];
extern const uint8_t MyLangDescr[];
extern const uint8_t MyManuInfo[];
extern const uint8_t MyProdInfo[];
extern const uint8_t MySerNumInfo[];
extern const uint8_t MyQuaDesc[];
extern const uint8_t MyBOSDesc[];
extern uint8_t TAB_USB_FS_OSC_DESC[];
extern uint8_t TAB_USB_HS_OSC_DESC[];

#endif /* __USB_DESC_H */
