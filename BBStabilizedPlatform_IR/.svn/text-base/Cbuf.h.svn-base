/*
  Copyright BlueBird Aero Systems Ltd 2011

  Revision 1.1  2012/01/04 12:23:45  EranS
  First save
 
 */

#ifndef CBUF_H_
#define CBUF_H_

/**< Circular Buffer Types */
typedef unsigned char INT8U;
typedef float KeyType;

typedef struct
{
   INT8U writePointer; /**< write pointer */
   INT8U readPointer;  /**< read pointer */
   INT8U size;         /**< size of circular buffer */
   KeyType keys[0];    /**< Element of circular buffer */
} CircularBuffer;

/**< Buffer Size */
#define BUFFER_SIZE  21
#define BLOCKSIZE  18

CircularBuffer* CircularBufferInit(CircularBuffer** pQue, int size);
int CircularBufferIsFull(CircularBuffer* que); 
inline int CircularBufferIsEmpty(CircularBuffer* que); 
int CircularBufferEnque(CircularBuffer* que, KeyType k); 
int CircularBufferDeque(CircularBuffer* que, KeyType* pK);
void stdev(CircularBuffer* que);


#endif
