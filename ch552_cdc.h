#ifndef CH552_CDC_H
#define CH552_CDC_H

#include "ch554.h"
#include "ch552_defines.h"
#include "ch554_usb.h"

#ifndef CDC_BAUDRATE
#define CDC_BAUDRATE 9600
#endif //CDC_BAUDRATE

#define CDC_BAUDRATE_B(n) (uint8_t)(((uint32_t)CDC_BAUDRATE>>(n*8)) & 0xFF )

#define REQ_CDC_SET_LINE_CODING     0x20
#define REQ_CDC_GET_LINE_CODING     0x21
#define REQ_CDC_SET_CTRL_LINE_STATE 0x22

__xdata __at(0x0000) uint8_t __CDC_Ep0Buffer[DEFAULT_ENDP0_SIZE];
__xdata __at(0x0008) uint8_t __CDC_Ep2Buffer[2*MAX_PACKET_SIZE];

volatile uint8_t __CDC_readIndex;
volatile uint8_t __CDC_bytesReceived;

uint16_t SetupLen;
uint8_t   SetupReq,UsbConfig;
const uint8_t *  pDescr;                                                       //USB配置标志
USB_SETUP_REQ   SetupReqBuf;                                                   //暂存Setup包
#define UsbSetupBuf     ((PUSB_SETUP_REQ)__CDC_Ep0Buffer)

unsigned char  __code __CDC_LangDes[]={0x04,0x03,0x09,0x04};           //语言描述符
unsigned char  __code __CDC_SerDes[]={                                 //序列号字符串描述符
                                                                 0x14,0x03,
                                                                 0x32,0x00,0x30,0x00,0x31,0x00,0x37,0x00,0x2D,0x00,
                                                                 0x32,0x00,0x2D,0x00,
                                                                 0x32,0x00,0x35,0x00
                               };
unsigned char  __code __CDC_Prod_Des[]={                                //产品字符串描述符
                                                                  0x14,0x03,
                                                                  0x43,0x00,0x48,0x00,0x35,0x00,0x35,0x00,0x34,0x00,0x5F,0x00,
                                                                  0x43,0x00,0x44,0x00,0x43,0x00,
                                 };
unsigned char  __code __CDC_Manuf_Des[]={
    0x0A,0x03,
    0x5F,0x6c,0xCF,0x82,0x81,0x6c,0x52,0x60,
};

__code uint8_t __CDC_LineCoding[] =
{
  CDC_BAUDRATE_B(0),
  CDC_BAUDRATE_B(1),
  CDC_BAUDRATE_B(2),
  CDC_BAUDRATE_B(3),
  0x00,
  0x00,
  0x08
};

__code uint8_t __CDC_DeviceDescr[] =
{
  0x12,   //Size of DeviceDescr in bytes
  0x01,   //Descriptor type (0x01 : deviceDescriptor)
  0x10,   // |
  0x01,   // | USB specification version : 1.1
  0x02,   // Device class (0x02 : CDC control)
  0x00,   // Device subclass (0x00 : not set for CDC control)
  0x00,   // Device protocol (0x00 : not set for CDC control)
  DEFAULT_ENDP0_SIZE,   // Max packet size on endpoint 0 8 bytes
  0x86,   // |
  0x1A,   // | Vendor ID (0x1A86 : WCH)
  0x22,   // |
  0x57,   // | Product ID (0x5722 : ch55x)
  0x00,   // |
  0x01,   // | Device version (0x0001 : 0.1)
  0x00,   // Vendor index descriptor (optional)
  0x00,   // Product index descriptor (optional)
  0x00,   // Serial number (optional)
  0x01    // Number of configuration
};

__code uint8_t __CDC_ConfigDescr[] =
{
  0x09,   //Size of ConfigDescr in bytes
  0x02,   //Descriptor type (0x02 : configDescriptor)
  0x43,   // |
  0x00,   // | Size of complete configuration tree in bytes
  0x02,   // Number of interface
  0x01,   // Configuration value
  0x00,   // Configuration index descriptor (optional)
  0b10100000,   // Power attributes
  0x32,   // Max electrical consumption by 1:2mA (0x32 : 100mA)

  //<==== INTERFACE 0 ====>
      0x09,   //Size of InterfaceDescr in bytes
      0x04,   //Descriptor type (0x04 : interfaceDescriptor)
      0x00,   // Interface value
      0x00,   // Alternate Setting (optional)
      0x01,   // Number of endpoints (endpoint0 exluded)
      0x02,   // Interface class (0x02 : CDC control)
      0x02,   // Interface subclass
      0x02,   // Interface protocol
      0x00,    // Interface index descriptor (optional)
      //<==== FUNCTION HEADER ====>
          0x05,   //Size of terminaison
          0x24,   //Descriptor type (0x24 : CS interface)
          0x00,   //Descruptor subtype (0x00 : HEADER)
          0x10,   //  |
          0x01,   //  | BCD
      //<==== CALL MANAGMENT ====>
          0x05,   //Size of terminaison
          0x24,   //Descriptor type (0x24 : CS interface)
          0x01,   //Descriptor subtype (0x01 : Call managment)
          0x00,   //Capabilities (optional)
          0x00,   //Interface index descriptor (optional)
      //<==== ACM DESCRIPTOR ====>
          0x04,   //Size of terminaison
          0x24,   //Descriptor type (0x24 : CS interface)
          0x02,   //Descriptor subtype (0x02 : ACM)
          0x02,   //Capabilities (supports ACM commands)
      //<==== UNION FUNCTIONAL DESCRIPTOR ====>
          0x05,   //Size of terminaison
          0x24,   //Descriptor type (0x24 : CS interface)
          0x06,   //Descryptor subtype (0x06 : UNION)
          0x00,   //Interface of control (interface 0)
          0x01,   //Interface suborinate (interface 1)
      //<==== NOTIFICATION ENDPOINT ====>
          0x07,   //size of terminaison
          0x05,   //Descriptor type (0x05 : Endpoint)
          0x81,   //Endpoint Address
          0x03,   //Attribues
          0x08,   // |
          0x00,   // | max packet size in bytes (0x0008 : 64)
          0xFF,   // Interval

  //<==== INTERFACE 1 ====>
      0x09,   //Size of InterfaceDescr in bytes
      0x04,   //Descriptor type (0x04 : interfaceDescriptor)
      0x01,   //Interface value
      0x00,   //Alternate setting (optional)
      0x02,   //Number of endpoints (endpoint0 excluded)
      0x0A,   //Interface class (0x0A : CDC data)
      0x00,   //Interface subclass (not set)
      0x00,   //Interface protocol (not set)
      0x00,    //Interface index descriptor (optional)
      //<==== DATA OUT ENDPOINT ====>
          0x07,   //Size of terminaison in bytes
          0x05,   //Descriptor type (0x05 : endpoint)
          0x02,   //Endpoint address
          0x02,   //Attributes
          0x40,   // |
          0x00,   // | max packet size in bytes (0x0040)
          0x00,   //Interval
      //<==== DATA IN ENDPOINT ====>
          0x07,   //Size of terminaison in bytes
          0x05,   //Descriptor type (0x05 : endpoint)
          0x82,   //Endpoint address
          0x02,   //Attributes
          0x40,   // |
          0x00,   // | max packet size in bytes (0x0040)
          0x00    //Interval
};

void CDC_init()
{
  __CDC_readIndex = 0;
  __CDC_bytesReceived = 0;

  USB_CTRL = 0x00;
  USB_CTRL &= ~bUC_HOST_MODE;
  USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;
  USB_DEV_AD = 0x00;

  USB_CTRL &= ~bUC_LOW_SPEED;
  UDEV_CTRL &= ~bUD_LOW_SPEED;
  UDEV_CTRL = bUD_PD_DIS;
  UDEV_CTRL |= bUD_PORT_EN;

  UEP2_DMA = (uint16_t) __CDC_Ep2Buffer;
  UEP2_3_MOD = 0xCC;
  UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;

  UEP0_DMA = (uint16_t) __CDC_Ep0Buffer;
  UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  USB_INT_EN |= bUIE_TRANSFER;
  USB_INT_EN |= bUIE_SUSPEND;
  USB_INT_FG |= 0x1F;
  IE_USB = 1;
}

void CDC_send(uint8_t* buffer, uint8_t length)
{
  while(length > MAX_PACKET_SIZE)
  {
    memcpy(__CDC_Ep2Buffer+MAX_PACKET_SIZE,buffer,MAX_PACKET_SIZE);
    UEP2_T_LEN = MAX_PACKET_SIZE;
    UEP2_CTRL &= (bUEP_T_RES1 | bUEP_T_RES0);
    while((UEP2_CTRL & MASK_UEP_T_RES) == UEP_T_RES_ACK);
    length -= MAX_PACKET_SIZE;
  }
  memcpy(__CDC_Ep2Buffer+MAX_PACKET_SIZE,buffer,length);
  UEP2_T_LEN = length;
  UEP2_CTRL &= ~(bUEP_T_RES1 | bUEP_T_RES0);
}

void CDC_send_char(__idata char c)
{
  __CDC_Ep2Buffer[MAX_PACKET_SIZE] = c;
  UEP2_T_LEN = 1;
  UEP2_CTRL &= ~(bUEP_T_RES1 | bUEP_T_RES0);
}

inline uint8_t CDC_available()
{
  return __CDC_bytesReceived;
}

inline void CDC_clear()
{
  __CDC_bytesReceived = 0;
}

uint8_t CDC_read()
{
  --__CDC_bytesReceived;
  return __CDC_Ep2Buffer[__CDC_readIndex++];
}

void CDC_process()
{
  uint16_t len;
  if(UIF_TRANSFER)                                                            //USB传输完成标志
  {
      switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
      {
      case UIS_TOKEN_IN | 2:                                                  //endpoint 2# 端点批量上传
          UEP2_T_LEN = 0;                                                    //预使用发送长度一定要清空
          UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;           //默认应答NAK
          break;
      case UIS_TOKEN_OUT | 2:                                                 //endpoint 3# 端点批量下传
          if (U_TOG_OK)                                                     // 不同步的数据包将丢弃
          {
              __CDC_bytesReceived = USB_RX_LEN;
              __CDC_readIndex = 0;
	//UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;
               UEP2_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;
          }
          break;
      case UIS_TOKEN_SETUP | 0:                                                //SETUP事务
          len = USB_RX_LEN;
          if(len == (sizeof(USB_SETUP_REQ)))
          {
              SetupLen = ((uint16_t)UsbSetupBuf->wLengthH<<8) | (UsbSetupBuf->wLengthL);
              len = 0;                                                      // 默认为成功并且上传0长度
              SetupReq = UsbSetupBuf->bRequest;

              if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )//Non standard request
              {
                  switch( SetupReq )
                  {
                  case REQ_CDC_GET_LINE_CODING:   //0x21  currently configured
                      pDescr = __CDC_LineCoding;
                      len = sizeof(__CDC_LineCoding);
                      memcpy(__CDC_Ep0Buffer,pDescr,sizeof(__CDC_LineCoding));
                      SetupLen -= len;
                      pDescr += len;
                      break;
                  case REQ_CDC_SET_CTRL_LINE_STATE:  //0x22  generates RS-232/V.24 style control signals
                      break;
                  case REQ_CDC_SET_LINE_CODING:      //0x20  Configure
                      break;
                  default:
                      len = 0xFF;  								 					                 /*命令不支持*/
                      break;
                  }
              }
              else                                                             //标准请求
              {
                  switch(SetupReq)                                             //请求码
                  {
                  case USB_GET_DESCRIPTOR:
                      switch(UsbSetupBuf->wValueH)
                      {
                      case 1:                                                       //设备描述符
                          pDescr = __CDC_DeviceDescr;                                         //把设备描述符送到要发送的缓冲区
                          len = sizeof(__CDC_DeviceDescr);
                          break;
                      case 2:                                                        //配置描述符
                          pDescr = __CDC_ConfigDescr;                                          //把设备描述符送到要发送的缓冲区
                          len = sizeof(__CDC_ConfigDescr);
                          break;
                      case 3:
                          if(UsbSetupBuf->wValueL == 0)
                          {
                              pDescr = __CDC_LangDes;
                              len = sizeof(__CDC_LangDes);
                          }
                          else if(UsbSetupBuf->wValueL == 1)
                          {
                              pDescr = __CDC_Manuf_Des;
                              len = sizeof(__CDC_Manuf_Des);
                          }
                          else if(UsbSetupBuf->wValueL == 2)
                          {
                              pDescr = __CDC_Prod_Des;
                              len = sizeof(__CDC_Prod_Des);
                          }
                          else
                          {
                              pDescr = __CDC_SerDes;
                              len = sizeof(__CDC_SerDes);
                          }
                          break;
                      default:
                          len = 0xff;                                                //不支持的命令或者出错
                          break;
                      }
                      if ( SetupLen > len )
                      {
                          SetupLen = len;
                      }
                      len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                            //本次传输长度
                      memcpy(__CDC_Ep0Buffer,pDescr,len);                                  //加载上传数据
                      SetupLen -= len;
                      pDescr += len;
                      break;
                  case USB_SET_ADDRESS:
                      SetupLen = UsbSetupBuf->wValueL;                              //暂存USB设备地址
                      break;
                   case USB_SET_CONFIGURATION:
                       break;
                  case USB_GET_INTERFACE:
                      break;
                  default:
                      len = 0xff;                                                    //操作失败
                      break;
                  }
              }
          }
          else
          {
              len = 0xff;                                                         //包长度错误
          }
          if(len == 0xff)
          {
              SetupReq = 0xFF;
              UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
          }
          else if(len <= DEFAULT_ENDP0_SIZE)                                                       //上传数据或者状态阶段返回0长度包
          {
              UEP0_T_LEN = len;
              UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1，返回应答ACK
          }
          else
          {
              UEP0_T_LEN = 0;  //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
              UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//默认数据包是DATA1,返回应答ACK
          }
          break;
      case UIS_TOKEN_IN | 0:                                                      //endpoint0 IN
          switch(SetupReq)
          {
          case USB_GET_DESCRIPTOR:
              len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                                 //本次传输长度
              memcpy( __CDC_Ep0Buffer, pDescr, len );                                   //加载上传数据
              SetupLen -= len;
              pDescr += len;
              UEP0_T_LEN = len;
              UEP0_CTRL ^= bUEP_T_TOG;                                             //同步标志位翻转
              break;
          case USB_SET_ADDRESS:
              USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
              UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
              break;
          default:
              UEP0_T_LEN = 0;                                                      //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
              UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
              break;
          }
          break;
      }
      UIF_TRANSFER = 0;                                                           //写0清空中断
  }
  if(UIF_SUSPEND)
  {
    UIF_SUSPEND = 0;
  }
}



#endif //CH552_CDC_H
