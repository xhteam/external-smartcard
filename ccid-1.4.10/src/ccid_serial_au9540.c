/*
 * ccid_serial_au9540.c: communicate with a AlcorMicro AU9540 Twin smart card reader
 * Based on ccid_serial.c
 * Copyright (C) 2001-2010 Ludovic Rousseau <ludovic.rousseau@free.fr>
 *
 * Thanks to Niki W. Waibel <niki.waibel@gmx.net> for a prototype version
 *
    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <sys/epoll.h>

#include <ifdhandler.h>

#include "config.h"
#include "defs.h"
#include "ccid_ifdhandler.h"
#include "debug.h"
#include "ccid.h"
#include "utils.h"
#include "commands.h"
#include "parser.h"
#include "strlcpycat.h"

/*
 * You may get read timeout after a card movement.
 * This is because you will get the echo of the CCID command
 * but not the result of the command.
 *
 * This is not an applicative issue since the card is either removed (and
 * powered off) or just inserted (and not yet powered on).
 */

/* 271 = max size for short APDU
 * 1 byte checksum
 */
#define AU9540_TWIN_MAXBUF (271+1)*2

#define AU9540_PACKET_MIN   (11)


typedef struct
{
	/*
	 * File handle on the serial port
	 */
	int fd;

	/*
	 * device used ("/dev/ttyS?" under Linux)
	 */
	/*@null@*/ char *device;

	
	/*
	 * serial communication buffer
	 */
	unsigned char buffer[AU9540_TWIN_MAXBUF];

	/*
	 * next available byte
	 */
	int buffer_offset;

	/*
	 * number of available bytes
	 */
	int buffer_offset_last;

	/*
	 * CCID infos common to USB and serial
	 */
	_ccid_descriptor ccid;

} _serialDevice;

/* The _serialDevice structure must be defined before including ccid_serial.h */
#include "ccid_serial.h"


/* no need to initialize to 0 since it is static */
static _serialDevice serialDevice[CCID_DRIVER_MAX_READERS];

static unsigned int SerialCustomDataRates[] = { 4800,9600,19200,38400, 0 };

/* unexported functions */
/*****************************************************************************
 *
 *					i2dw
 *
 ****************************************************************************/
static void i2dw(int value, unsigned char buffer[])
{
	buffer[0] = value & 0xFF;
	buffer[1] = (value >> 8) & 0xFF;
	buffer[2] = (value >> 16) & 0xFF;
	buffer[3] = (value >> 24) & 0xFF;
} /* i2dw */

/*****************************************************************************
*
*                  bei2i (big endian integer to host order interger)
*
****************************************************************************/

static unsigned int bei2i(unsigned char buffer[])
{
	return (buffer[0]<<24) + (buffer[1]<<16) + (buffer[2]<<8) + buffer[3];
}
static unsigned int lei2i(unsigned char buffer[])
{
	return (buffer[3]<<24) + (buffer[2]<<16) + (buffer[1]<<8) + buffer[0];
}

static int my_config(int fd,int baudrate,int datebites,int stopbits,int parity)
{
	int status;
	struct termios set_port;
	struct termios old_port;
	if(tcgetattr(fd,&old_port) != 0)//得到机器源端口的默认设置
	{
		DEBUG_INFO("tcgetattr error!\n");
		return -1;
	}  

	DEBUG_INFO4("my_config baudrate[%d],databits:%d,stopbits:%d\n",baudrate,datebites,stopbits);
	memset(&set_port,0,sizeof(set_port));  
    set_port.c_cflag |= CLOCAL | CREAD;//激活CLOCAL，CREAD用于本地连接和接收使能
    tcflush(fd,TCIOFLUSH);
        switch(baudrate)
    {
		case 4800:
            //tcflush(fd,TCIOFLUSH);//刷新输入输出缓冲
            cfsetispeed(&set_port,B4800);//分别设置输入和输出速率
            cfsetospeed(&set_port,B4800);
            break;
         
        case 9600:
            //tcflush(fd,TCIOFLUSH);//刷新输入输出缓冲
            cfsetispeed(&set_port,B9600);//分别设置输入和输出速率
            cfsetospeed(&set_port,B9600);
            break;
        case 19200:
            //tcflush(fd,TCIOFLUSH);
            cfsetispeed(&set_port,B19200);
            cfsetospeed(&set_port,B19200);
            break;
		case 38400:
            //tcflush(fd,TCIOFLUSH);//刷新输入输出缓冲
            cfsetispeed(&set_port,B38400);//分别设置输入和输出速率
            cfsetospeed(&set_port,B38400);
            break;
         default:
            cfsetispeed(&set_port,B19200);
            cfsetospeed(&set_port,B19200);
            DEBUG_INFO("default baudrate set to 19200\n");
            break;
    }
        
        if( tcsetattr(fd,TCSANOW,&set_port) != 0 )
          {
        DEBUG_INFO("can't set the speed!\n");
        return -1;
    }
    tcflush(fd,TCIOFLUSH);
    /*设置比特率结束*/
        /*********设置数据位**********/
    set_port.c_cflag &= ~CSIZE;
    switch(datebites)
    {    
        case 7:
            set_port.c_cflag |= CS7;
            break;
        case 8:
            set_port.c_cflag |= CS8;
            break;
                default:
                        //set_port.c_cflag |=CS8;
            DEBUG_INFO("please input the datebites. 7 or 8\n");
                return -1;    
    }
    tcflush(fd , TCIOFLUSH);
    /********设置数据位结束*******/
        /*********设置校验位*********/
        switch(parity)
    {
        case 'n':                 /*无校验*/
        case 'N':
            set_port.c_cflag &= ~PARENB;
            set_port.c_iflag &= ~INPCK;
            break;
        case 'o':                 /*奇检验*/
        case 'O':
            set_port.c_cflag |= (PARODD | PARENB);
            set_port.c_iflag |= INPCK;
            break;
        case 'e':                 /*偶校验*/
        case 'E':
            set_port.c_cflag |= PARENB;
            set_port.c_cflag &= ~PARODD;
            set_port.c_iflag |= INPCK;
            break;
        case 's':                /*Space校验*/
        case 'S':
            set_port.c_cflag &= ~PARENB;
            set_port.c_cflag &= ~CSTOPB;
            break;
        default:
            //set_port.c_cflag |= PARENB;
            //set_port.c_cflag &= ~PARODD;
            DEBUG_INFO("please input the right parity\n");
            return -1;
      
    }
    if(parity != 'n')
    set_port.c_iflag |= INPCK;
    tcflush(fd, TCIOFLUSH); 
    /**********停止位***********/
        switch(stopbits)
    {
        case 1:
            set_port.c_cflag &= ~CSTOPB;
            break;
        case 2:
            set_port.c_cflag |= CSTOPB;
            break;
        default:
            DEBUG_INFO("please input the right stopbites. 1 or 2\n");
            return -1;
    }
    tcflush(fd, TCIOFLUSH); 
    //设置停止位结束
       
        /*关掉ICRNL和IXON功能，使能接受二机制字符*/
    set_port.c_iflag &= ~(ICRNL | IXON);
    /*设置看控制时间*/
    set_port.c_lflag &=~ICANON;//设置串口为原始模式，在原始模式下下面两个字段才有效
    set_port.c_cc[VTIME]=0;
    set_port.c_cc[VMIN]=0;
        tcflush(fd,TCIOFLUSH);
    //刷新输入缓存或者输出缓存TCIFLUSH输入队列，TCIOFLUSH输入输出队列
    if(tcsetattr(fd,TCSANOW,&set_port)!=0)//立刻将设置写道串口中去
    {
        DEBUG_INFO("com set error!\n");//设置错误
        return -1;
    }
        //tcflush(fd,TCIOFLUSH);
    return 0;

}

static int epoll_register( int  epoll_fd, int  fd ) {
	struct epoll_event  ev;
	int                 ret, flags;

	/* important: make the fd non-blocking */
	flags = fcntl(fd, F_GETFL);
	fcntl(fd, F_SETFL, flags | O_NONBLOCK);

	ev.events  = EPOLLIN;
	ev.data.fd = fd;
	do {
		ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
	} while (ret < 0 && errno == EINTR);
	return ret;
}

static int epoll_deregister( int  epoll_fd, int  fd ) {
	int  ret;
	do {
		ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
	} while (ret < 0 && errno == EINTR);
	return ret;
}


/*****************************************************************************
 *
 *				WriteSerial: Send bytes to the card reader
 *
 *****************************************************************************/
status_t WriteSerial(unsigned int reader_index, unsigned int length,
	unsigned char *buffer)
{
	unsigned char low_level_buffer[AU9540_TWIN_MAXBUF];
	unsigned char lrc;
	unsigned int i;
	int fd = serialDevice[reader_index].fd;
	char debug_header[] = "-> 123456 ";

	(void)snprintf(debug_header, sizeof(debug_header), "-> %06X ",
		reader_index);


	memcpy(low_level_buffer,buffer,length);
	
	/* checksum */
	lrc = 0;
	for(i=0; i<length; i++)
		lrc ^= low_level_buffer[i];
	low_level_buffer[length] = lrc;

	DEBUG_XXD(debug_header, low_level_buffer, length+1);
	
	if (write(fd, low_level_buffer,
		length+1) != (ssize_t)(length+1))
	{
		DEBUG_CRITICAL2("write error: %s", strerror(errno));
		return STATUS_UNSUCCESSFUL;
	}

	return STATUS_SUCCESS;
} /* WriteSerial */

/*****************************************************************************
 *
 *				ReadChunk: read a minimum number of bytes
 *
 *****************************************************************************/
static int ReadChunk(unsigned int reader_index, unsigned char *buffer,
	int buffer_length, int min_length)
{
	int fd = serialDevice[reader_index].fd;
	unsigned char lrc=0;	
	struct timeval t;
	unsigned int readtimeout = serialDevice[reader_index].ccid.readTimeout;
	unsigned int totaltimeout=0;
	unsigned int timeout=100;
	int return_length;
	int to_read_length;
	int read_length;
	int epoll_fd   = epoll_create(1);

	read_length = 0 ;
	to_read_length = min_length;

	//DEBUG_COMM3("to read %d timeout=%d\n", to_read_length,readtimeout);

	epoll_register( epoll_fd, fd );

	for(;;){
		struct epoll_event	 events[2];
		int 				 ne, nevents;

		nevents = epoll_wait( epoll_fd, events,  1,
			timeout);
		if (nevents < 0) {
			if (errno != EINTR)
				DEBUG_COMM2("epoll_wait() unexpected error: %s", strerror(errno));
			return STATUS_COMM_ERROR;
		}
		
		for (ne = 0; ne < nevents; ne++) {
			if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
				DEBUG_COMM("EPOLLERR or EPOLLHUP after epoll_wait() !?");
				goto read_end;
			}

			
			if ((events[ne].events & EPOLLIN) != 0) {
				int  datafd = events[ne].data.fd;
				if(fd==datafd){
						int ret = read( fd, buffer + read_length,  buffer_length - read_length );
						if (ret > 0) {
							read_length+=ret;							
						}
				}
			}
		}

		if(read_length>=to_read_length)
			break;
		if(!nevents){
			//reach here means timeout or event 
			totaltimeout+=timeout;
			if(totaltimeout>=readtimeout){		
				goto read_end;
			}
		}
	}

	read_end:
	epoll_deregister(epoll_fd,fd);
	close(epoll_fd);
	
	return read_length>0?read_length:STATUS_COMM_ERROR;
} /* ReadChunk */

/*****************************************************************************
 *
 *				get_bytes: get n bytes
 *
 *****************************************************************************/
int get_bytes(unsigned int reader_index, unsigned char *buffer, int length)
{
	int offset = serialDevice[reader_index].buffer_offset;
	int offset_last = serialDevice[reader_index].buffer_offset_last;

	//DEBUG_COMM3("available: %d, needed: %d", offset_last-offset,
	//	length);
	/* enough data are available */
	if (offset + length <= offset_last)
	{
	//	DEBUG_COMM("data available");
		memcpy(buffer, serialDevice[reader_index].buffer + offset, length);
		serialDevice[reader_index].buffer_offset += length;
	}
	else
	{
		int present, rv;

		/* copy available data */
		present = offset_last - offset;

		if (present > 0)
		{
	//		DEBUG_COMM2("some data available: %d", present);
			memcpy(buffer, serialDevice[reader_index].buffer + offset,
				present);
		}

		/* get fresh data */
	//	DEBUG_COMM2("get more data: %d", length - present);
		rv = ReadChunk(reader_index, serialDevice[reader_index].buffer,
			sizeof(serialDevice[reader_index].buffer), length - present);
		if (rv < 0)
			return STATUS_COMM_ERROR;

		/* fill the buffer */
		memcpy(buffer + present, serialDevice[reader_index].buffer,
			length - present);
		serialDevice[reader_index].buffer_offset = length - present;
		serialDevice[reader_index].buffer_offset_last = rv;
	//	DEBUG_COMM3("offset: %d, last_offset: %d",
	//		serialDevice[reader_index].buffer_offset,
	//		serialDevice[reader_index].buffer_offset_last);
	}

	return STATUS_SUCCESS;
} /* get_bytes */



static int LRC_Status(unsigned char* buffer,int length){
	int i;
	unsigned char lrc=0;
	for (i=0; i<(length-1); i++)
		lrc ^= buffer[i];
	if(lrc!=buffer[length-1])
		return STATUS_DEVICE_PROTOCOL_ERROR;
	return STATUS_SUCCESS;
}

/*****************************************************************************
 *
 *				ReadSerial: Receive bytes from the card reader
 *
 *****************************************************************************/
status_t ReadSerial(unsigned int reader_index,
	unsigned int *length, unsigned char *buffer)
{
#if 0
	unsigned char low_level_buffer[AU9540_TWIN_MAXBUF];
	int fd = serialDevice[reader_index].fd;
	unsigned char lrc=0;	
	struct timeval t;
	unsigned int readtimeout = serialDevice[reader_index].ccid.readTimeout;
	unsigned int totaltimeout=0;
	unsigned int timeout=100;
	int rv;
	int return_length;
	int to_read_length;
	int read_length;
	int i;
	int loops=0;
	char debug_header[] = "<- 121234 ";
	int epoll_fd   = epoll_create(1);

	(void)snprintf(debug_header, sizeof(debug_header), "<- %06X ",
		(int)reader_index);
	read_length	= 0 ;
	to_read_length = *length;
	to_read_length++;//lrc byte
	if(to_read_length<AU9540_PACKET_MIN)
		to_read_length=AU9540_PACKET_MIN;

	if(to_read_length>AU9540_TWIN_MAXBUF)
		return STATUS_DEVICE_PROTOCOL_ERROR;

	DEBUG_COMM3("ReadSerial to read %d timeout=%d\n", to_read_length,readtimeout);

	memset(low_level_buffer,0,AU9540_TWIN_MAXBUF);
	epoll_register( epoll_fd, fd );
	
	for(;;){
		struct epoll_event   events[2];
		int                  ne, nevents;

		nevents = epoll_wait( epoll_fd, events,  1,
			timeout);
		if (nevents < 0) {
			if (errno != EINTR)
				DEBUG_COMM2("epoll_wait() unexpected error: %s", strerror(errno));
			return STATUS_COMM_ERROR;
		}
		
		for (ne = 0; ne < nevents; ne++) {
			if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
				fprintf(stderr,"EPOLLERR or EPOLLHUP after epoll_wait() !?");
				goto read_end;
			}

			
			if ((events[ne].events & EPOLLIN) != 0) {
				int  datafd = events[ne].data.fd;
				if(fd==datafd){
						int ret = read( fd, low_level_buffer + read_length,  AU9540_TWIN_MAXBUF - read_length );
						if (ret > 0) {
							read_length+=ret;							
						}
				}
			}
		}

	
		if(read_length>AU9540_PACKET_MIN)
			break;
		if(!nevents){
			//reach here means timeout or event 
			totaltimeout+=timeout;
			if(totaltimeout>=readtimeout){		
				DEBUG_COMM("read timeout\n");
				goto read_end;
			}
		}
	}

	read_end:
	epoll_deregister(epoll_fd,fd);
	close(epoll_fd);

	if(read_length<AU9540_PACKET_MIN)
		return STATUS_COMM_ERROR;
		
	//calculate lrc?	
	for (i=0; i<(read_length-1); i++)
		lrc ^= low_level_buffer[i];
	if(lrc!=low_level_buffer[read_length-1])
		return STATUS_COMM_ERROR;
	DEBUG_XXD(debug_header, low_level_buffer, read_length);
	*length = read_length-1;
	memcpy(buffer,low_level_buffer,*length);

#else	
	unsigned char c;
	int rv;
	int to_read;
	unsigned int ext_length;
	unsigned int packet_length;
	int basic_packet_size=AU9540_PACKET_MIN;
	unsigned char lowlevel_buffer[AU9540_TWIN_MAXBUF];	
	char debug_header[] = "<- 121234 ";
	(void)snprintf(debug_header, sizeof(debug_header), "<- %06X ",
		(int)reader_index);

	memset(lowlevel_buffer,0,AU9540_TWIN_MAXBUF);
	if ((rv = get_bytes(reader_index, lowlevel_buffer, basic_packet_size)) != STATUS_SUCCESS){
		return rv;
	}
	packet_length = basic_packet_size;

	ext_length = lei2i(&lowlevel_buffer[1]);
	if(length){
		if ((rv = get_bytes(reader_index, lowlevel_buffer+basic_packet_size, ext_length)) != STATUS_SUCCESS){
			return rv;
		}	
		packet_length += ext_length;
	}
	
	//LRC check
	if(STATUS_SUCCESS!=LRC_Status(lowlevel_buffer,packet_length)){
		return STATUS_DEVICE_PROTOCOL_ERROR;		
	}

	DEBUG_XXD(debug_header, lowlevel_buffer, packet_length);

	//remove LRC byte
	packet_length--;
	
	memcpy(buffer,lowlevel_buffer,packet_length);
	*length = packet_length;

#endif
	return STATUS_SUCCESS;
} /* ReadSerial */


/*****************************************************************************
 *
 *				OpenSerial: open the port
 *
 *****************************************************************************/
status_t OpenSerial(unsigned int reader_index, int channel)
{
	char dev_name[FILENAME_MAX];

	DEBUG_COMM3("Reader index: %X, Channel: %d", reader_index, channel);

	/*
	 * Conversion of old-style ifd-hanler 1.0 CHANNELID
	 */
	if (channel == 0x0103F8)
		channel = 1;
	else
		if (channel == 0x0102F8)
			channel = 2;
		else
			if (channel == 0x0103E8)
				channel = 3;
			else
				if (channel == 0x0102E8)
					channel = 4;

	if (channel < 0)
	{
		DEBUG_CRITICAL2("wrong port number: %d", channel);
		return STATUS_UNSUCCESSFUL;
	}

	(void)snprintf(dev_name, sizeof(dev_name), "/dev/pcsc/%d", channel);

	return OpenSerialByName(reader_index, dev_name);
} /* OpenSerial */

/*****************************************************************************
 *
 *				set_ccid_descriptor: init ccid descriptor
 *				depending on reader type specified in device.
 *
 *				return: STATUS_UNSUCCESSFUL,
 *						STATUS_SUCCESS,
 *						-1 (Reader already used)
 *
 *****************************************************************************/
static status_t set_ccid_descriptor(unsigned int reader_index,
	const char *reader_name, const char *dev_name)
{

	/* Common to all readers */
	serialDevice[reader_index].ccid.real_bSeq = 0;
	serialDevice[reader_index].ccid.pbSeq = &serialDevice[reader_index].ccid.real_bSeq;
	serialDevice[reader_index].ccid.bCurrentSlotIndex = 0;
	

	serialDevice[reader_index].ccid.dwMaxCCIDMessageLength = 271;
	serialDevice[reader_index].ccid.dwMaxIFSD = 254;
	serialDevice[reader_index].ccid.dwFeatures = CCID_CLASS_AUTO_VOLTAGE|CCID_CLASS_SHORT_APDU;
	serialDevice[reader_index].ccid.dwDefaultClock = 384;
	//0x058F:0x9540 AlcorMicro AU9540
	serialDevice[reader_index].ccid.readerID = ALCORMICRO_AU9540;
	serialDevice[reader_index].ccid.bPINSupport = 0x0;
	serialDevice[reader_index].ccid.dwMaxDataRate = 38400;
	serialDevice[reader_index].ccid.bMaxSlotIndex = 0;
	serialDevice[reader_index].ccid.arrayOfSupportedDataRates = SerialCustomDataRates;
	serialDevice[reader_index].ccid.dwSlotStatus = IFD_ICC_PRESENT;
	serialDevice[reader_index].ccid.bVoltageSupport = 0x01;	/*  5V */
	serialDevice[reader_index].ccid.gemalto_firmware_features = NULL;
	serialDevice[reader_index].ccid.readTimeout = 1000;
	

	return STATUS_SUCCESS;
} /* set_ccid_descriptor  */

/*****************************************************************************
 *
 *				OpenSerialByName: open the port
 *
 *****************************************************************************/
status_t OpenSerialByName(unsigned int reader_index, char *dev_name)
{
	struct termios current_termios;
	unsigned int reader = reader_index;
	/* 255 is MAX_DEVICENAME in pcscd.h */
	char reader_name[255] = "au9540";
	char *p;
	status_t ret;

	DEBUG_COMM3("Reader index: %X, Device: %s", reader_index, dev_name);

	/* parse dev_name using the pattern "device:name" */
	p = strchr(dev_name, ':');
	if (p)
	{
		/* copy the second part of the string */
		strlcpy(reader_name, p+1, sizeof(reader_name));

		/* replace ':' by '\0' so that dev_name only contains the device name */
		*p = '\0';
	}
	DEBUG_INFO3("Reader:%s,device:%s\n",reader_name,dev_name);

	ret = set_ccid_descriptor(reader_index, reader_name, dev_name);
	if (STATUS_UNSUCCESSFUL == ret)
		return STATUS_UNSUCCESSFUL;

	/* secondary slot so do not physically open the device */
	if (STATUS_SECONDARY_SLOT == ret)
		return STATUS_SUCCESS;

	DEBUG_INFO3("Prepare to open reader %s:%s\n",reader_name,dev_name);
	serialDevice[reader].fd = open(dev_name, O_RDWR|O_NOCTTY);

	if (serialDevice[reader].fd<0)
	{
		DEBUG_CRITICAL3("open %s: %s", dev_name, strerror(errno));
		return STATUS_UNSUCCESSFUL;
	}



	/* set channel used */
	serialDevice[reader].device = strdup(dev_name);
	#if 0

	/* empty in and out serial buffers */
	if (tcflush(serialDevice[reader].fd, TCIOFLUSH))
			DEBUG_INFO2("tcflush() function error: %s", strerror(errno));

	/* get config attributes */
	if (tcgetattr(serialDevice[reader].fd, &current_termios) == -1)
	{
		DEBUG_INFO2("tcgetattr() function error: %s", strerror(errno));
		(void)close(serialDevice[reader].fd);
		serialDevice[reader].fd = -1;

		return STATUS_UNSUCCESSFUL;
	}

	/* IGNBRK: ignore BREAK condition on input
	 * IGNPAR: ignore framing errors and parity errors. */
	current_termios.c_iflag = IGNBRK | IGNPAR;
	current_termios.c_oflag = 0;	/* Raw output modes */
	/* CS8: 8-bits character size
	 * CSTOPB: set two stop bits
	 * CREAD: enable receiver
	 * CLOCAL: ignore modem control lines */
	current_termios.c_cflag = CS8 | CSTOPB | CREAD | CLOCAL;

	/* Do not echo characters because if you connect to a host it or your modem
	 * will echo characters for you.  Don't generate signals. */
	current_termios.c_lflag = 0;

	/* set serial port speed to 38400 bauds */
	cfsetispeed(&current_termios, B38400);
	cfsetospeed(&current_termios, B38400);
	
	DEBUG_INFO("Set serial port baudrate to B38400 and correct configuration");

	if (tcsetattr(serialDevice[reader].fd, TCSANOW, &current_termios) == -1)
	{
		(void)close(serialDevice[reader].fd);
		serialDevice[reader].fd = -1;
		DEBUG_INFO2("tcsetattr error: %s", strerror(errno));

		return STATUS_UNSUCCESSFUL;
	}
	#else
	if(my_config(serialDevice[reader].fd,38400,8,1,'n'))
		return STATUS_UNSUCCESSFUL;
	#endif

#if 0
	if(!strcasecmp(reader_name,":au9540")){		
		unsigned char command[AU9540_TWIN_MAXBUF];
		unsigned char response[AU9540_TWIN_MAXBUF];
		status_t ret;
		unsigned int ret_length;
		_ccid_descriptor* descriptor = get_ccid_descriptor(reader_index);
		unsigned int old_readtimeout = descriptor->readTimeout;
		//get descriptor
		command[0] = 0x67;// SG_PC_to_RDR_ Get_Descriptor 		
		i2dw(0, &command[1]);	/* APDU length */
		command[5] = descriptor->bCurrentSlotIndex;//slot number
		command[6] = 0;//seq
		command[7] = 0;//get device descriptor
		command[8] = 0;//StrDesc,only used while command[7]=0x01
		command[9] = 0;//RFU

		descriptor->readTimeout = 200;
		do {
			WriteSerial(reader_index,10,command);
			ret_length = 28;
			ret = ReadSerial(reader_index,&ret_length,response);
			if(STATUS_SUCCESS==ret)
				break;
			else if(command[6]<3)
				command[6]++;
			else if(command[6]>=3)
				break;
		}while(1);
		
		if(STATUS_SUCCESS==ret){
			//apply device descriptor
			DEBUG_INFO("get device descriptor okay\n");
			//set_device_descriptor(reader_index,response[10]);
		}
		descriptor->readTimeout = old_readtimeout;

	}
#endif
	serialDevice[reader_index].ccid.sIFD_serial_number = "12345678";
	serialDevice[reader_index].ccid.sIFD_iManufacturer = "AlcorMicro";
	serialDevice[reader_index].ccid.IFD_bcdDevice = 0;

	return STATUS_SUCCESS;
} /* OpenSerialByName */


/*****************************************************************************
 *
 *				CloseSerial: close the port
 *
 *****************************************************************************/
status_t CloseSerial(unsigned int reader_index)
{
	unsigned int reader = reader_index;

	/* device not opened */
	if (NULL == serialDevice[reader_index].device)
		return STATUS_UNSUCCESSFUL;

	DEBUG_COMM2("Closing serial device: %s", serialDevice[reader_index].device);

	(void)close(serialDevice[reader].fd);
	serialDevice[reader].fd = -1;

	free(serialDevice[reader].device);
	serialDevice[reader].device = NULL;

	return STATUS_SUCCESS;
} /* CloseSerial */


/*****************************************************************************
 *
 *					get_ccid_descriptor
 *
 ****************************************************************************/
_ccid_descriptor *get_ccid_descriptor(unsigned int reader_index)
{
	return &serialDevice[reader_index].ccid;
} /* get_ccid_descriptor */


