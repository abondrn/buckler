// Main Pixy template class.  This class takes a link class and uses
// it to communicate with Pixy over I2C, SPI, UART or USB using the 
// Pixy packet protocol.

#ifndef _TPIXY2_H
#define _TPIXY2_H

// uncomment to turn on debug prints to console
#define PIXY_DEBUG

#define PIXY_BUFFERSIZE                      0x104
#define PIXY_CHECKSUM_SYNC                   0xc1af
#define PIXY_NO_CHECKSUM_SYNC                0xc1ae
#define PIXY_SEND_HEADER_SIZE                4
#define PIXY_MAX_PROGNAME                    33
#define PIXY_

#define PIXY_TYPE_REQUEST_CHANGE_PROG        0x02
#define PIXY_TYPE_REQUEST_RESOLUTION         0x0c
#define PIXY_TYPE_RESPONSE_RESOLUTION        0x0d
#define PIXY_TYPE_REQUEST_VERSION            0x0e
#define PIXY_TYPE_RESPONSE_VERSION           0x0f
#define PIXY_TYPE_RESPONSE_RESULT            0x01
#define PIXY_TYPE_RESPONSE_ERROR             0x03
#define PIXY_TYPE_REQUEST_BRIGHTNESS         0x10
#define PIXY_TYPE_REQUEST_SERVO              0x12
#define PIXY_TYPE_REQUEST_LED                0x14
#define PIXY_TYPE_REQUEST_LAMP               0x16
#define PIXY_TYPE_REQUEST_FPS                0x18

#define PIXY_RESULT_OK                       0
#define PIXY_RESULT_ERROR                    -1
#define PIXY_RESULT_BUSY                     -2
#define PIXY_RESULT_CHECKSUM_ERROR           -3
#define PIXY_RESULT_TIMEOUT                  -4
#define PIXY_RESULT_BUTTON_OVERRIDE          -5
#define PIXY_RESULT_PROG_CHANGING            -6

// RC-servo values
#define PIXY_RCS_MIN_POS                     0
#define PIXY_RCS_MAX_POS                     1000L
#define PIXY_RCS_CENTER_POS                  ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)

#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include <stdint.h> // for int types
#include <stdbool.h>
#include <stdio.h> // for printf
#include <string.h> // for nstrcpy
#include <stdlib.h> // for free and malloc
//#include "Pixy2CCC.h"
//#include "Pixy2Line.h"
//#include "Pixy2Video.h"

typedef struct Version {
  uint16_t hardware;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  uint16_t firmwareBuild;
  char firmwareType[10];   
} version_t;

void print_version(version_t *v) {
    printf("hardware ver: 0x%x firmware ver: %d.%d.%d %s", v->hardware, v->firmwareMajor, v->firmwareMinor, v->firmwareBuild, v->firmwareType);
}


typedef struct Pixy2 {
  version_t *version;
  uint16_t frameWidth;
  uint16_t frameHeight;

  // Color connected components, color codes
  //Pixy2CCC ccc;

  // Line following
  //Pixy2Line line;

  // Video
  //Pixy2Video video;

  uint8_t *m_buf;
  uint8_t *m_bufPayload;
  uint8_t m_type;
  uint8_t m_length;
  bool m_cs;

  nrf_drv_spi_t const * p_instance;
} pixy2_t;


#define PIXY_SPI_CLOCKRATE       2000000


uint16_t ssPin;


int8_t open(uint32_t arg);
ret_code_t recv(nrf_drv_spi_t const * const p_instance, uint8_t *buf, uint8_t len);
void send(nrf_drv_spi_t const * const p_instance, uint8_t *buf, uint8_t len);


/**
* @brief Function for starting the SPI data transfer.
*
* If an event handler was provided in the @ref nrf_drv_spi_init call, this function
* returns immediately and the handler is called when the transfer is done.
* Otherwise, the transfer is performed in blocking mode, which means that this function
* returns when the transfer is finished.
*
* @note Peripherals using EasyDMA (for example, SPIM) require the transfer buffers
*       to be placed in the Data RAM region. If they are not and an SPIM instance is
*       used, this function will fail with the error code NRF_ERROR_INVALID_ADDR.
*
* @param[in] p_instance       Pointer to the driver instance structure.
* @param[in] p_tx_buffer      Pointer to the transmit buffer. Can be NULL
*                             if there is nothing to send.
* @param     tx_buffer_length Length of the transmit buffer.
* @param[in] p_rx_buffer      Pointer to the receive buffer. Can be NULL
*                             if there is nothing to receive.
* @param     rx_buffer_length Length of the receive buffer.
*
* @retval NRF_SUCCESS            If the operation was successful.
* @retval NRF_ERROR_BUSY         If a previously started transfer has not finished
*                                yet.
* @retval NRF_ERROR_INVALID_ADDR If the provided buffers are not placed in the Data
*                                RAM region.
__STATIC_INLINE
ret_code_t nrf_drv_spi_transfer(nrf_drv_spi_t const * const p_instance,
                               uint8_t const * p_tx_buffer,
                               uint8_t         tx_buffer_length,
                               uint8_t       * p_rx_buffer,
                               uint8_t         rx_buffer_length);
*/


// You need to write send(), which takes a pointer to the data you want to send and the number of
// bytes to send via your serial port (SPI, I2C or UART).  It returns the number of bytes successfully sent.
void send(nrf_drv_spi_t const * const p_instance, uint8_t *data, uint8_t len) {
#ifdef PIXY_DEBUG
  //printf("writing: ");
  //for (int i=0; i<len; i++)
  //  printf("0x%hhx ", data[i]);
  //printf("\n");
#endif    
  APP_ERROR_CHECK(nrf_drv_spi_transfer(p_instance, data, len, NULL, 0));
}

// You also need to write recv(), which takes a pointer to a data buffer where the received data
// will be written/returned and the number of bytes to receive via your serial port (SPI, I2C or UART).
// It returns the number of bytes immediately available and written into the buffer (without needing
// to busy-wait.)
ret_code_t recv(nrf_drv_spi_t const * const p_instance, uint8_t *data, uint8_t len) {
  ret_code_t status = nrf_drv_spi_transfer(p_instance, NULL, 0, data, len);
#ifdef PIXY_DEBUG
  //printf("reading: ");
  //for (int i=0; i<len; i++)
  //  printf("0x%hhx ", data[i]);
  //printf("\n");
  //printf("status: 0x%hhx\n", status);
#endif
  return status;
}


int8_t init(pixy2_t** p, nrf_drv_spi_t const * const p_instance);
void close(pixy2_t* p);


// interface methods
int8_t getVersion(pixy2_t* p);
int8_t changeProg(pixy2_t* p, const char *prog);
int8_t setServos(pixy2_t* p, uint16_t s0, uint16_t s1);
int8_t setCameraBrightness(pixy2_t* p, uint8_t brightness);
int8_t setLED(pixy2_t* p, uint8_t r, uint8_t g, uint8_t b);
int8_t setLamp(pixy2_t* p, uint8_t upper, uint8_t lower);
int8_t getResolution(pixy2_t* p);
int8_t getFPS(pixy2_t* p);

// helper methods
int16_t getSync(pixy2_t* p);
int16_t recvPacket(pixy2_t* p);
int16_t sendPacket(pixy2_t* p);


int8_t init(pixy2_t **pref, nrf_drv_spi_t const * const p_instance) {

  pixy2_t *p = malloc(sizeof(struct Pixy2));
  *pref = p;

  // allocate buffer space for send/receive
  p->m_buf = (uint8_t *)malloc(PIXY_BUFFERSIZE);
  // shifted buffer is used for sending, so we have space to write header information
  p->m_bufPayload = p->m_buf + PIXY_SEND_HEADER_SIZE;
  p->frameWidth = p->frameHeight = 0;
  p->version = NULL;

  p->p_instance = p_instance;

  uint32_t t0;
  int8_t res;
  
  //if (res<0)
  //  return res;
  
  // wait for pixy to be ready -- that is, Pixy takes a second or 2 boot up
  // getVersion is an effective "ping".  We timeout after 5s.
  //for(t0=millis(); millis()-t0<5000; ) {
    if (getVersion(p) >= 0) { // successful version get -> pixy is ready
      getResolution(p); // get resolution so we have it
      return PIXY_RESULT_OK;
    }	  
    nrf_delay_ms(5000); // delay for sync
  //}
  // timeout
  return PIXY_RESULT_TIMEOUT;
}


void close(pixy2_t *p) {
  free(p->m_buf);
}


int16_t getSync(pixy2_t *p) {
  uint8_t i, j, c, cprev;
  int16_t res;
  uint16_t start;
  
  // parse bytes until we find sync
  for(i=j=0, cprev=0; true; i++) {
    res = recv(p->p_instance, &c, 1);
    if (res == NRF_SUCCESS) {
      // since we're using little endian, previous byte is least significant byte
      start = cprev;
      // current byte is most significant byte
      start |= c << 8;
      cprev = c;
      if (start==PIXY_CHECKSUM_SYNC) {
        p->m_cs = true;
        return PIXY_RESULT_OK;
      }
      if (start==PIXY_NO_CHECKSUM_SYNC) {
        p->m_cs = false;
        return PIXY_RESULT_OK;
      }
    }
	  // If we've read some bytes and no sync, then wait and try again.
	  // And do that several more times before we give up.  
	  // Pixy guarantees to respond within 100us.
    if (i>=4) {
      if (j>=4) {
#ifdef PIXY_DEBUG
        printf("error: no response\n");
#endif		  
        return PIXY_RESULT_ERROR;
      }
      nrf_delay_ms(25); 
      j++;
      i = 0;
    }
  }
}


int16_t recvPacket(pixy2_t *p) {
  uint16_t csCalc, csSerial;
  int16_t res;
  
  // clear out any stale data
  res = getSync(p);
  if (res == PIXY_RESULT_ERROR)
    return res;

  if (p->m_cs) {
    res = recv(p->p_instance, p->m_buf, 4);
    if (res != NRF_SUCCESS)
      return res;

    p->m_type = p->m_buf[0];
    p->m_length = p->m_buf[1];

    csSerial = *(uint16_t *)&p->m_buf[2];

    res = recv(p->p_instance, p->m_buf, p->m_length);
    if (res != NRF_SUCCESS)
      return res;

    csCalc = 0;
    for (uint8_t i=0; i<p->m_length; i++) {
      csCalc += p->m_buf[i];
    }

    if (csSerial!=csCalc) {
#ifdef PIXY_DEBUG
      printf("error: checksum\n");
#endif
      return PIXY_RESULT_CHECKSUM_ERROR;
    }
  } else {   
    res = recv(p->p_instance, p->m_buf, 2);
    if (res != NRF_SUCCESS)
      return res;

    p->m_type = p->m_buf[0];
    p->m_length = p->m_buf[1];

    res = recv(p->p_instance, p->m_buf, p->m_length);
    if (res != NRF_SUCCESS)
      return res;
  }
  return PIXY_RESULT_OK;
}


int16_t sendPacket(pixy2_t *p) {
  // write header info at beginnig of buffer
  p->m_buf[0] = PIXY_NO_CHECKSUM_SYNC&0xff;
  p->m_buf[1] = PIXY_NO_CHECKSUM_SYNC>>8;
  p->m_buf[2] = p->m_type;
  p->m_buf[3] = p->m_length;
  // send whole thing -- header and data in one call
  send(p->p_instance, p->m_buf, p->m_length+PIXY_SEND_HEADER_SIZE);
  return PIXY_RESULT_OK;
}


int8_t changeProg(pixy2_t *p, const char *prog) {
  int32_t res;
  
  // poll for program to change
  while(1) {
    strncpy((char *)p->m_bufPayload, prog, PIXY_MAX_PROGNAME);
    p->m_length = PIXY_MAX_PROGNAME;
    p->m_type = PIXY_TYPE_REQUEST_CHANGE_PROG;
    sendPacket(p);
    if ((res=recvPacket(p)) != PIXY_RESULT_OK)
      return res; // some kind of bitstream error
    res = *(uint32_t *)p->m_buf;
    if (res != 0) {
      getResolution(p);  // get resolution so we have it
      return PIXY_RESULT_OK; // success     
    }
    nrf_delay_ms(1000); 
  }
}


int8_t getVersion(pixy2_t *p) {
  int32_t res;

  p->m_length = 0;
  p->m_type = PIXY_TYPE_REQUEST_VERSION;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_VERSION) {
    p->version = (version_t *)p->m_buf;
    return p->m_length;
  } else if (p->m_type==PIXY_TYPE_RESPONSE_ERROR)
    return PIXY_RESULT_BUSY;
  else
    return PIXY_RESULT_ERROR;
}


int8_t getResolution(pixy2_t *p) {
  int32_t res;

  p->m_length = 1;
  p->m_bufPayload[0] = 0; // for future types of queries
  p->m_type = PIXY_TYPE_REQUEST_RESOLUTION;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESOLUTION) {
    p->frameWidth = *(uint16_t *)p->m_buf;
    p->frameHeight = *(uint16_t *)(p->m_buf+sizeof(uint16_t));
    return PIXY_RESULT_OK; // success
  } else 
    return PIXY_RESULT_ERROR;
}
    

int8_t setCameraBrightness(pixy2_t *p, uint8_t brightness) {
  uint32_t res;
  
  p->m_bufPayload[0] = brightness;
  p->m_length = 1;
  p->m_type = PIXY_TYPE_REQUEST_BRIGHTNESS;
  sendPacket(p);
  if ((res=recvPacket(p)) == PIXY_RESULT_OK) // && p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4)
  {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;	
  } else
    return res;  // some kind of bitstream error
}


int8_t setServos(pixy2_t *p, uint16_t s0, uint16_t s1) {
  uint32_t res;
  
  *(int16_t *)(p->m_bufPayload + 0) = s0;
  *(int16_t *)(p->m_bufPayload + 2) = s1;
  p->m_length = 4;
  p->m_type = PIXY_TYPE_REQUEST_SERVO;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;	
  } else
    return PIXY_RESULT_ERROR;  
}


int8_t setLED(pixy2_t *p, uint8_t r, uint8_t g, uint8_t b) {
  uint32_t res;
  
  p->m_bufPayload[0] = r;
  p->m_bufPayload[1] = g;
  p->m_bufPayload[2] = b;
  p->m_length = 3;
  p->m_type = PIXY_TYPE_REQUEST_LED;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;	
  } else
    return PIXY_RESULT_ERROR;
}

int8_t setLamp(pixy2_t *p, uint8_t upper, uint8_t lower) {
  uint32_t res;
  
  p->m_bufPayload[0] = upper;
  p->m_bufPayload[1] = lower;
  p->m_length = 2;
  p->m_type = PIXY_TYPE_REQUEST_LAMP;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;	
  } else
      return PIXY_RESULT_ERROR;
}

int8_t getFPS(pixy2_t *p) {
  uint32_t res;
  
  p->m_length = 0; // no args
  p->m_type = PIXY_TYPE_REQUEST_FPS;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;	
  } else
    return PIXY_RESULT_ERROR;  // some kind of bitstream error	
}

#endif
