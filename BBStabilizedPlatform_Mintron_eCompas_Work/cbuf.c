#include <stdlib.h>
#include <math.h>
#include "cbuf.h"
#include "bluebird.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
/*!
 * Circular Buffer
 * Compile: cbuf.c
 */
 
 
KeyType pSrc[BUFFER_SIZE];
float stddev=0.0f,sum=0.0f,sumOfSquares=0.0f,average=0.0f;	


/*
  * Calculates the standard deviation of the elements in the input vector
*/
void stdev(CircularBuffer* que)
{
     KeyType Src;
	 int isEmpty;	 
	 sum=0.0f;
	 sumOfSquares=0.0f;	 
	 
	 do {
		 isEmpty = CircularBufferDeque(que, &Src);
		 sumOfSquares += Src * Src;		 
		 sum += Src;	 
	 } while (!isEmpty);
	 
	   
	 stddev = sqrt((sumOfSquares - sum*sum / BLOCKSIZE) / (BLOCKSIZE - 1));	 
	 average = sum / BLOCKSIZE;
}

/**< Init Circular Buffer */
CircularBuffer* CircularBufferInit(CircularBuffer** pQue, int size)
{
        //int sz = size*sizeof(KeyType)+sizeof(CircularBuffer);        
		
        *pQue = (CircularBuffer*)pSrc;
        if(*pQue)
        {
            //printf("Init CircularBuffer: keys[%d] (%d)\n", size, sz);
            (*pQue)->size=size;
            (*pQue)->writePointer = 0;
            (*pQue)->readPointer  = 0;
        }
        return *pQue;
}
 
int CircularBufferIsFull(CircularBuffer* que)
{
     return ((que->writePointer + 1) % que->size == que->readPointer);
}

 
int CircularBufferIsEmpty(CircularBuffer* que)
{
     return (que->readPointer == que->writePointer);
}

 
int CircularBufferEnque(CircularBuffer* que, KeyType k)
{
        int isFull = CircularBufferIsFull(que);
        que->keys[que->writePointer] = k;
        que->writePointer++;
        que->writePointer %= que->size;
        return isFull;
}
 
int CircularBufferDeque(CircularBuffer* que, KeyType* pK)
{
        int isEmpty =  CircularBufferIsEmpty(que);
        *pK = que->keys[que->readPointer];
        que->readPointer++;
        que->readPointer %= que->size;
        return(isEmpty);
}
 

