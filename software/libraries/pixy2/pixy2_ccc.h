#ifndef _PIXY2CCC_H
#define _PIXY2CCC_H

#include <stdint.h> // for int types
#include <stdbool.h>
#include <stdio.h> // for printf

//#include "nrf_delay.h"


#define CCC_MAX_SIGNATURE                   7

#define CCC_RESPONSE_BLOCKS                 0x21
#define CCC_REQUEST_BLOCKS                  0x20

#define CCC_MAX_BLOCKS                      (PIXY_BUFFERSIZE-PIXY_SEND_HEADER_SIZE)/sizeof(struct Block)

// Defines for sigmap:
// You can bitwise "or" these together to make a custom sigmap.
// For example if you're only interested in receiving blocks
// with signatures 1 and 5, you could use a sigmap of 
// PIXY_SIG1 | PIXY_SIG5
#define CCC_SIG1                     1 
#define CCC_SIG2                     2
#define CCC_SIG3                     4
#define CCC_SIG4                     8
#define CCC_SIG5                     16
#define CCC_SIG6                     32
#define CCC_SIG7                     64
#define CCC_COLOR_CODES              128

#define CCC_SIG_ALL                  0xff // all bits or'ed together

typedef struct Block {
  uint16_t m_signature;
  uint16_t m_x;
  uint16_t m_y;
  uint16_t m_width;
  uint16_t m_height;
  int16_t m_angle;
  uint8_t m_index;
  uint8_t m_age;
} block_t;


// print block structure!
void print_block(block_t *b) {
    int i, j;
    char buf[128], sig[6], d;
    bool flag;  
    if (b->m_signature > CCC_MAX_SIGNATURE) { // color code! (CC)
        // convert signature number to an octal string
        for (i=12, j=0, flag=false; i>=0; i-=3) {
            d = (b->m_signature >> i)&0x07;
            if (d>0 && !flag)
                flag = true;
            if (flag)
                sig[j++] = d + '0';
        }
        sig[j] = '\0';  
        printf("CC block sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle: %d index: %d age: %d", sig, b->m_signature, b->m_x, b->m_y, b->m_width, b->m_height, b->m_angle, b->m_index, b->m_age);
    } else // regular block.  Note, angle is always zero, so no need to print
        printf("sig: %d x: %d y: %d width: %d height: %d index: %d age: %d", b->m_signature, b->m_x, b->m_y, b->m_width, b->m_height, b->m_index, b->m_age);   
}

  
int8_t getBlocks(bool wait, uint8_t sigmap, uint8_t maxBlocks);


int8_t getBlocks(pixy2_t *p, bool wait, uint8_t sigmap, uint8_t maxBlocks) {
  blocks_t *blocks = NULL;
  int8_t numBlocks = 0;
  
  while(1) {
    // fill in request data
    p->m_bufPayload[0] = sigmap;
    p->m_bufPayload[1] = maxBlocks;
    p->m_length = 2;
    p->m_type = CCC_REQUEST_BLOCKS;
  
    // send request
    sendPacket(p);
    if (recvPacket(p) == PIXY_RESULT_OK) {
      if (p->m_type==CCC_RESPONSE_BLOCKS) {
        blocks = (block_t*)p->m_buf;
        numBlocks = p->m_length/sizeof(struct Block);
        return numBlocks;
      }
	  // deal with busy and program changing states from Pixy (we'll wait)
      else if (p->m_type == PIXY_TYPE_RESPONSE_ERROR) {
        if ((int8_t)p->m_buf[0] == PIXY_RESULT_BUSY) {
          if(!wait)
            return PIXY_RESULT_BUSY; // new data not available yet
		} else if ((int8_t)p->m_buf[0] != PIXY_RESULT_PROG_CHANGING)
          return p->m_buf[0];
      }
    } else
      return PIXY_RESULT_ERROR;  // some kind of bitstream error
  
    // If we're waiting for frame data, don't thrash Pixy with requests.
    // We can give up half a millisecond of latency (worst case)	
    nrf_delay_ms(500);
  }
}

#endif
