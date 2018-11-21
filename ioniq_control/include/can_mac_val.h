#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>

//#define PCAN_CHANNEL PCAN_USBBUS1
//#define PCAN_BAUDRATE PCAN_BAUD_500K

#ifndef __APPLE__
#include <asm/types.h>
#define DWORD  __u32
#define WORD   unsigned short
#define BYTE   unsigned char
#define LPSTR  char*
#include "PCANBasic.h"
#else
#include "PCBUSB.h"
#endif

// message.ID       == frame.can_id
// message.DATA[]   == frame.data[]
// mesage.LEN       == frame.can_dlc

// standard CAN
#define CAN_MAX_DLEN 8

class SOCKETCAN_FRAME
{
public:
    unsigned long can_id;               // peakcan은 unsigned long, socketcan은 unsigned int
    unsigned char can_msgtype;
    unsigned char can_dlc;              // peakcan과 같음
    unsigned char data[CAN_MAX_DLEN];   // peakcan과 같음
};

class CAN_AVL
{
public:
    TPCANMsg message;
    TPCANStatus status;
    TPCANHandle dev_name;
    SOCKETCAN_FRAME frame;
    int fd;
    fd_set fds;
    
    CAN_AVL(TPCANHandle dev_name_, TPCANHandle dev_baud_rate_);
    void CAN_INIT(TPCANHandle dev_name_, TPCANHandle dev_baud_rate_);
    void PEAKCAN_TO_SOCKETCAN();
    void SOCKETCAN_TO_PEAKCAN();
    void write_param(int id_, int dlc_);
    void write_param(int id_, int dlc_, unsigned char can_msgtype_);
    //void PEAKCAN_WRITE();
    void can_write();
    void can_read();
    void can_memset();
};

CAN_AVL::CAN_AVL(TPCANHandle dev_name_, TPCANHandle dev_baud_rate_)
{
    CAN_INIT(dev_name_, dev_baud_rate_);
}

void CAN_AVL::CAN_INIT(TPCANHandle dev_name_, TPCANHandle dev_baud_rate_)
{
    status = CAN_Initialize(dev_name_, dev_baud_rate_, 0, 0, 0);
    printf("Initialize CAN: 0x%lx\n", status);
    if(status != PCAN_ERROR_OK)
    {
        perror("PCAN_ERROR_INIT.\n  --> Check Baud_Rate(500K), CAN Module Inserted State and Device Name.\n\n");
        exit(0);
    }
    dev_name = dev_name_;
    
    message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    
}

void CAN_AVL::can_read()
{
    // 입력 이벤트가 변할 때 만 출력하는 코드
    status = CAN_GetValue(dev_name, PCAN_RECEIVE_EVENT, &fd, sizeof(int));
    status = CAN_Read(dev_name, &message, NULL);
    
    
    
    // 입력 이벤트가 변하지 않아도 항상 출력하는 코드
//    status = CAN_GetValue(dev_name, PCAN_RECEIVE_EVENT, &fd, sizeof(int));
//    status = CAN_Read(dev_name, &message, NULL);
    
    
    
    
    // 입력 이벤트가 변할 때만 출력하는 코드. 아래 코드는 입력에 대한 딜레이가 존재. 해결 못했음.
//    status = CAN_GetValue(dev_name, PCAN_RECEIVE_EVENT, &fd, sizeof(int));
//    FD_ZERO(&fds);
//    FD_SET(fd, &fds);
//    while(select(fd+1, &fds, NULL, NULL, NULL) > 0) break;
//    status = CAN_Read(dev_name, &message, NULL);
//    if (status != PCAN_ERROR_OK) {
//        printf("Read Error 0x%lx\n", status);
//    }
    
    PEAKCAN_TO_SOCKETCAN(); // msg -> frame
}
void CAN_AVL::can_write()
{
    SOCKETCAN_TO_PEAKCAN();
//    printf("  - R | HEX | ID:%04x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
//           (int) message.ID, (int) message.LEN, (int) message.DATA[0],
//           (int) message.DATA[1], (int) message.DATA[2],
//           (int) message.DATA[3], (int) message.DATA[4],
//           (int) message.DATA[5], (int) message.DATA[6],
//           (int) message.DATA[7]);
    status = CAN_Write(dev_name, &message);
    
    if (status != PCAN_ERROR_OK) {
        printf("Write Error 0x%lx\n", status);
    }
    
    PEAKCAN_TO_SOCKETCAN(); // msg -> frame
}
void CAN_AVL::PEAKCAN_TO_SOCKETCAN()
{
    memcpy(&frame.data, &message.DATA, sizeof(unsigned char)*8);
    memcpy(&frame.can_dlc, &message.LEN, sizeof(unsigned char));
    memcpy(&frame.can_msgtype, &message.MSGTYPE, sizeof(unsigned char));
    memcpy(&frame.can_id, &message.ID, sizeof(unsigned long));
}

void CAN_AVL::SOCKETCAN_TO_PEAKCAN()
{
    memcpy(&message.DATA, &frame.data, sizeof(unsigned char)*8);
    memcpy(&message.LEN, &frame.can_dlc, sizeof(unsigned char));
    memcpy(&message.MSGTYPE, &frame.can_msgtype, sizeof(unsigned char));
    memcpy(&message.ID, &frame.can_id, sizeof(unsigned long));
}


void CAN_AVL::write_param(int id_, int dlc_)
{
    frame.can_id = id_;
    frame.can_dlc = dlc_;
    
//    message.ID = id_;
//    message.LEN = dlc_;
//    message.MSGTYPE = PCAN_MESSAGE_STANDARD;
//    printf("Initialize CAN: 0x%lx\n", status);
}

void CAN_AVL::write_param(int id_, int dlc_, unsigned char msgtype_)
{
    frame.can_id = id_;
    frame.can_dlc = dlc_;
    frame.can_msgtype = msgtype_;
    //    message.ID = id_;
    //    message.LEN = dlc_;
    //    message.MSGTYPE = PCAN_MESSAGE_STANDARD;
    //    printf("Initialize CAN: 0x%lx\n", status);
}

void CAN_AVL::can_memset()
{
//    memset(&frame, 0, sizeof(SOCKETCAN_FRAME));
    memset(frame.data, 0, frame.can_dlc);
    SOCKETCAN_TO_PEAKCAN();
}
