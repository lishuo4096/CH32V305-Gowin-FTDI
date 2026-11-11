/********************************** (C) COPYRIGHT *******************************
* File Name          : usb_desc.c
* Description        : descriptors for the FT2232D-alike device
*******************************************************************************/
#include "usb_desc.h"

/* ---------------------------------------------------------------------------
 * Device descriptor
 *   bcdUSB     2.00
 *   bDeviceClass 0x00 (class is defined per interface)
 *   VID:PID    0403:6010  (FTDI FT2232D)
 *   bcdDevice  0x0500     (FT2232D silicon revision, FTDI driver uses it to
 *                          select the chip type)
 * ------------------------------------------------------------------------- */
const uint8_t MyDevDescr[] =
{
    0x12,       /* bLength */
    0x01,       /* bDescriptorType (Device) */
    0x00, 0x02, /* bcdUSB 2.00 */
    0x00,       /* bDeviceClass */
    0x00,       /* bDeviceSubClass */
    0x00,       /* bDeviceProtocol */
    DEF_USBD_UEP0_SIZE,                 /* bMaxPacketSize0 = 64 */
    0x03, 0x04,                         /* idVendor  0x0403 */
    0x10, 0x60,                         /* idProduct 0x6010 */
    0x00, 0x05,                         /* bcdDevice 0x0500 */
    0x01,       /* iManufacturer  (string 1) */
    0x02,       /* iProduct       (string 2) */
    0x03,       /* iSerialNumber  (string 3) */
    0x01,       /* bNumConfigurations */
};

/* ---------------------------------------------------------------------------
 * Configuration descriptor, high speed (55 bytes)
 *   interface 0 : vendor specific, 2 bulk end-points  -> MPSSE / JTAG
 *   interface 1 : vendor specific, 2 bulk end-points  -> UART
 * NOTE: FTDI requires the IN and OUT end-point number of one channel to be
 *       different (that is why EP1/EP2 and EP3/EP4 are used).
 * ------------------------------------------------------------------------- */
const uint8_t MyCfgDescr_HS[] =
{
    /* configuration */
    0x09,       /* bLength */
    0x02,       /* bDescriptorType (Configuration) */
    0x37, 0x00, /* wTotalLength 55 */
    0x02,       /* bNumInterfaces 2 */
    0x01,       /* bConfigurationValue */
    0x00,       /* iConfiguration */
    0x80,       /* bmAttributes (bus powered, no remote wakeup) */
    0x32,       /* bMaxPower 100mA */

    /* interface 0 : JTAG / MPSSE */
    0x09,       /* bLength */
    0x04,       /* bDescriptorType (Interface) */
    0x00,       /* bInterfaceNumber 0 */
    0x00,       /* bAlternateSetting */
    0x02,       /* bNumEndpoints 2 */
    0xFF,       /* bInterfaceClass    (vendor specific) */
    0xFF,       /* bInterfaceSubClass */
    0xFF,       /* bInterfaceProtocol */
    0x00,       /* iInterface 0 (no interface string, like real FTDI) */

    0x07,       /* bLength */
    0x05,       /* bDescriptorType (Endpoint) */
    0x02,       /* bEndpointAddress 0x02 OUT */
    0x02,       /* bmAttributes (Bulk) */
    (uint8_t)DEF_USB_EP2_HS_SIZE, (uint8_t)(DEF_USB_EP2_HS_SIZE >> 8),
    0x00,       /* bInterval */

    0x07,       /* bLength */
    0x05,       /* bDescriptorType (Endpoint) */
    0x81,       /* bEndpointAddress 0x81 IN */
    0x02,       /* bmAttributes (Bulk) */
    (uint8_t)DEF_USB_EP1_HS_SIZE, (uint8_t)(DEF_USB_EP1_HS_SIZE >> 8),
    0x00,       /* bInterval */

    /* interface 1 : UART */
    0x09,       /* bLength */
    0x04,       /* bDescriptorType (Interface) */
    0x01,       /* bInterfaceNumber 1 */
    0x00,       /* bAlternateSetting */
    0x02,       /* bNumEndpoints 2 */
    0xFF,       /* bInterfaceClass    (vendor specific) */
    0xFF,       /* bInterfaceSubClass */
    0xFF,       /* bInterfaceProtocol */
    0x00,       /* iInterface 0 (no interface string, like real FTDI) */

    0x07,       /* bLength */
    0x05,       /* bDescriptorType (Endpoint) */
    0x04,       /* bEndpointAddress 0x04 OUT */
    0x02,       /* bmAttributes (Bulk) */
    (uint8_t)DEF_USB_EP4_HS_SIZE, (uint8_t)(DEF_USB_EP4_HS_SIZE >> 8),
    0x00,       /* bInterval */

    0x07,       /* bLength */
    0x05,       /* bDescriptorType (Endpoint) */
    0x83,       /* bEndpointAddress 0x83 IN */
    0x02,       /* bmAttributes (Bulk) */
    (uint8_t)DEF_USB_EP3_HS_SIZE, (uint8_t)(DEF_USB_EP3_HS_SIZE >> 8),
    0x00,       /* bInterval */
};

/* full speed configuration (same layout, 64 byte bulk packets) */
const uint8_t MyCfgDescr_FS[] =
{
    0x09,       /* bLength */
    0x02,       /* bDescriptorType (Configuration) */
    0x37, 0x00, /* wTotalLength 55 */
    0x02,       /* bNumInterfaces 2 */
    0x01,       /* bConfigurationValue */
    0x00,       /* iConfiguration */
    0x80,       /* bmAttributes */
    0x32,       /* bMaxPower 100mA */

    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x02,
    0x07, 0x05, 0x02, 0x02, (uint8_t)DEF_USB_EP2_FS_SIZE, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x02, (uint8_t)DEF_USB_EP1_FS_SIZE, 0x00, 0x00,

    0x09, 0x04, 0x01, 0x00, 0x02, 0xFF, 0xFF, 0xFF, 0x04,
    0x07, 0x05, 0x04, 0x02, (uint8_t)DEF_USB_EP4_FS_SIZE, 0x00, 0x00,
    0x07, 0x05, 0x83, 0x02, (uint8_t)DEF_USB_EP3_FS_SIZE, 0x00, 0x00,
};

/* Language descriptor */
const uint8_t MyLangDescr[] =
{
    0x04, 0x03, 0x09, 0x04
};

/* Manufacturer string : "FTDI" */
const uint8_t MyManuInfo[] =
{
    0x0A, 0x03, 'F', 0, 'T', 0, 'D', 0, 'I', 0
};

/* Product string : "Dual RS232-HS"
 *
 * Why not "FT2232D": Gowin's programmer decides the cable's display name from
 * the D2XX FT_GetDeviceInfoDetail() Description string.  If it contains the
 * substring "RS232-HS" (case sensitive) the cable is named
 * "Gowin USB Cable(FT2CH)"; otherwise the name is the hard-coded literal
 * "USB Debugger A".  (Reverse engineered out of ftdibase.py inside
 * programmer_cli.exe: 'RS232-HS' at 0x00c00b15 between
 * ftdi_get_device_info_detail and ftdi_open_ex; confirmed by an ablation
 * experiment that only rewrote the D2XX return field in memory.)
 * FTDI H-series parts with a blank EEPROM report "Single/Dual/Quad RS232-HS",
 * so that one substring covers every H part Gowin knows.  The FTDI driver
 * appends " A"/" B" per channel, and "Dual RS232-HS A" still contains it.
 * bLength is taken from MyProdInfo[0] (see DEF_USBD_PROD_DESC_LEN), so only
 * this array needs updating: 2 + 13*2 = 28 = 0x1C. */
const uint8_t MyProdInfo[] =
{
    0x1C, 0x03, 'D', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0,
    'R', 0, 'S', 0, '2', 0, '3', 0, '2', 0, '-', 0,
    'H', 0, 'S', 0
};

/* Serial number string : "ALT232H0001" (also present in the fake EEPROM) */
const uint8_t MySerNumInfo[] =
{
    0x18, 0x03, 'A', 0, 'L', 0, 'T', 0, '2', 0, '3', 0, '2', 0, 'H', 0,
                '0', 0, '0', 0, '0', 0, '1', 0
};

/* Device qualifier descriptor (needed because we run at high speed) */
const uint8_t MyQuaDesc[] =
{
    0x0A, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x01, 0x00,
};

/* Device BOS descriptor */
const uint8_t MyBOSDesc[] =
{
    0x05, 0x0F, 0x0C, 0x00, 0x01,
    0x07, 0x10, 0x02, 0x02, 0x00, 0x00, 0x00,
};

/* USB Full-Speed mode, other speed configuration (filled at run time) */
uint8_t TAB_USB_FS_OSC_DESC[sizeof(MyCfgDescr_HS)] =
{
    0x09, 0x07,
};

/* USB High-Speed mode, other speed configuration (filled at run time) */
uint8_t TAB_USB_HS_OSC_DESC[sizeof(MyCfgDescr_FS)] =
{
    0x09, 0x07,
};
