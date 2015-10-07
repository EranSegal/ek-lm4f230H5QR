/*************************************************************************
 *                             FILE: tw2824.c
 * Video ADC controled by I2C
 *************************************************************************/
#include <sys/hyrtk.h>
#include "dmcdef.h"
#include "mtype.h"
#include "tw2824.h"
#include "Hy16xs.h"
#include "param.h"
#include "jpeg.h"
#include "hw_if.h"
#include "i2c.h"

/*--------------------------------------------------------------------------*/
/* project includes                                                         */
/*--------------------------------------------------------------------------*/
#include <systools.h>
#include <registry.h>
#include <tcpip.h>
#include <timesrv.h>
#include <sys_par.h>
#include <mon_api.h>
#include <message.h>
#include <devman.h>
#include <connect.h>
#include <isrv.h>

/*--------------------------------------------------------------------------*/
/* hyperstone includes                                                      */
/*--------------------------------------------------------------------------*/
#ifdef _DEBUG_OZVISION_
#include <stdio.h>
#endif

BYTE ATD_Read_I2C_Register(BYTE page, BYTE Addr);
void Set4VinRegs(BYTE page, BYTE *vin_reg ,BYTE bitmask, BYTE bitval);
void ColorKilling(HWORD val);
void SetFrameModeOnYPath(void);
void BrightnessInit(void);

extern jpeg_type jpeg;
extern VmdType Vmd;
VmdType VmdCopy;
BYTE tmpVmd[2*VMD_ROWS_NUM];

void ATD_Write_I2C_Register(BYTE page, BYTE Addr, BYTE Data)
{
   I2C_Start(); 
   I2CSendByte(SLAVE_ADD);
   I2CSendByte(page);
   I2CSendByte(Addr);
   I2CSendByte(Data);
   I2C_Stop();
}

BYTE ATD_Read_I2C_Register(BYTE page, BYTE Addr)
{
   BYTE data;

   I2C_Start(); 
   I2CSendByte(SLAVE_ADD);
   I2CSendByte(page);
   I2CSendByte(Addr);
   I2C_Start(); 
   I2CSendByte(SLAVE_ADD | 0x01); // read slave address.
   data = I2CReceiveLastByte();
   I2C_Stop();

   return data;
}

void ATD_Write_Table(BYTE page, BYTE start_add, PBYTE tbl_ptr, BYTE tbl_len)
{
   I2C_Start(); 
   I2CSendByte(SLAVE_ADD);
   I2CSendByte(page);
   I2CSendByte(start_add);
   for(; tbl_len; tbl_len--, tbl_ptr++)
      I2CSendByte(*tbl_ptr);      
   I2C_Stop();
}

void ATD_Read_Table(BYTE page, BYTE start_add, PBYTE tbl_ptr, BYTE tbl_len)
{
   I2C_Start(); 
   I2CSendByte(SLAVE_ADD);
   I2CSendByte(page);
   I2CSendByte(start_add);
   I2C_Start(); 
   I2CSendByte(SLAVE_ADD | 0x01); // read slave address.   
   for(; tbl_len>1; tbl_len--, tbl_ptr++)
      *tbl_ptr = I2CReceiveByte();
   *tbl_ptr = I2CReceiveLastByte();
   I2C_Stop();
}

void ATDUpdateVmdVal(void)
{
   BYTE cam, VmdTmpArray[3];
   HWORD tmp, tmp1;
      
   ////tmp = ~(OzVmd->Mask);
   // Enable motion and blind detection
   ////ATD_Write_I2C_Register(PAGE2, MB_DIS, (tmp<<4)&0xF0);
   
   // Enable motion and blind detection for all cameras.
   ATD_Write_I2C_Register(PAGE2, MB_DIS, 0);   
   // set blind detect level:
   tmp = OzVmd->BlindDedect;
   ATD_Write_I2C_Register(PAGE2, 0x7f, ((tmp<<2)&0x30)|tmp);
   
   for (cam=0;cam<CAMERS_NUM;cam++)
   {
//      if((1<<cam)&OzVmd->Mask)
//      {
         // set sensitivity control level.
         tmp = OzVmd->Val[cam] & 0x000F; // sensitivity bits.
         VmdTmpArray[0] = ((tmp<<4)&0x00c0)|tmp;
         if((tmp1 = OzVmd->Val[cam] & 0x00F0)) // velocity bits.
            tmp1 = (tmp1 >> 3)|0x80;
         VmdTmpArray[1] = tmp1;
         VmdTmpArray[2] = (tmp << 4)|tmp;
         ATD_Write_Table(PAGE2, 0x81+0x20*cam, VmdTmpArray, 3);
         ATD_Write_Table(PAGE2, 0x84+0x20*cam, (PBYTE)&(OzVmd->CellsState[cam]), 2*VMD_ROWS_NUM);
//      }      
   }   
}


/*
 * FUNCTTON: ATD_Read_Vmd_Table
 * DESCRIPTION: read video motion detection parameters (results or mask of vmd).
 * PARAM: mask_mode:0 - Reading result of motion detection, 1 - Reading mask information.
 *        tmpVmd: resault of reading is stored here.
 */ 
void ATD_Read_Vmd_Table(BYTE cam, BYTE mask_mode)
{
   ATD_Write_I2C_Register(PAGE2, 0x80+0x20*cam, (0x20 | mask_mode<<7));   
   ATD_Read_Table(PAGE2, 0x84+0x20*cam, tmpVmd, 2*VMD_ROWS_NUM);   
}


//=======================================================================
//              Page0 initialize table description : NTSC
//=======================================================================
BYTE tbl_ntsc_page0_quad[] = {        
//							    CH1			CH2			CH3			CH4
         0xc4,0xe5,0x30,//...0x01~0x03	0x41~0x43	0x81~0x83	0xC1~0xC3       
    0xD0,0x20,0xd0,0x88,//...0x04~0x07	0x44~0x47	0x84~0x87	0xC4~0xC7      
    0x20,0x06,0x20,0x08,//...0x08~0x0B	0x48~0x4B	0x88~0x8B	0xC8~0xCB  
    0xf0,0x42,0x00,0x80,//...0x0C~0x0F	0x4C~0x4F	0x8C~0x8F	0xCC~0xCF
    0x80,0x80,0x80,0x1F,//...0x10~0x13	0x50~0x53	0x90~0x93	0xD0~0xD3
    0x00,0x00,0x00,0x00,//...0x14~0x17	0x54~0x57	0x94~0x97	0xD4~0xD7
    0x7f,0xff,0xff,0xff,//...0x18~0x1b	0x58~0x5b	0x98~0x9b	0xd8~0xdb
    0x7f,0xff,0x79,0x00,//...0x1c~0x1f	0x5c~0x5f	0x9c~0x9f	0xdc~0xdf
    0x07,0x07,0x10,0x11 //...0x20~0x23	0x60~0x63	0xa0~0xA3	0xE0~0xE3    	
	};

#ifdef 0 
//=======================================================================
//              Page0 initialize table description : PAL
//=======================================================================
BYTE tbl_pal_page0_quad[] = {        
//                             CH1         CH2         CH3         CH4
        0x84,0xa5,0x30,//...0x01~0x03	0x41~0x43	0x81~0x83	0xC1~0xC3
   0xd0,0x20,0xd0,0x88,//...0x04~0x07	0x44~0x47	0x84~0x87	0xC4~0xC7
   0x20,0x06,0x20,0x0a,//...0x08~0x0B	0x48~0x4B	0x88~0x8B	0xC8~0xCB
   0x20,0x4a,0x02,0x80,//...0x0C~0x0F	0x4C~0x4F	0x8C~0x8F	0xCC~0xCF
   0x80,0x80,0x80,0x2f,//...0x10~0x13	0x50~0x53	0x90~0x93	0xD0~0xD3
   0x00,0x00,0x40,0x40,//...0x14~0x17	0x54~0x57	0x94~0x97	0xD4~0xD7
   0x7f,0xff,0xde,0x00,//...0x18~0x1b	0x58~0x5b	0x98~0x9b	0xd8~0xdb
   0x7f,0xff,0x79,0x00,//...0x1c~0x1f	0x5c~0x5f	0x9c~0x9f	0xdc~0xdf
   0x07,0x0f,0x00,0x11 //...0x20~0x23	0x60~0x63	0xa0~0xA3	0xE0~0xE3
}; 
#else
//=======================================================================
//              Page0 initialize table description : PAL
//=======================================================================
BYTE tbl_pal_page0_quad[] = {        
//                             CH1         CH2         CH3         CH4
        0x84,0xa5,0x30,//...0x01~0x03	0x41~0x43	0x81~0x83	0xC1~0xC3
   0xd0,0x20,0xd0,0x88,//...0x04~0x07	0x44~0x47	0x84~0x87	0xC4~0xC7
   0x20,0x06,0x20,0x0a,//...0x08~0x0B	0x48~0x4B	0x88~0x8B	0xC8~0xCB
   0x20,0x4a,0x02,0x80,//...0x0C~0x0F	0x4C~0x4F	0x8C~0x8F	0xCC~0xCF
   0x80,0x80,0x80,0x2f,//...0x10~0x13	0x50~0x53	0x90~0x93	0xD0~0xD3
   0x00,0x00,0x40,0x40,//...0x14~0x17	0x54~0x57	0x94~0x97	0xD4~0xD7
   0x58,0xe3,0xde,0x00,//...0x18~0x1b	0x58~0x5b	0x98~0x9b	0xd8~0xdb
   0x7f,0xff,0x79,0x00,//...0x1c~0x1f	0x5c~0x5f	0x9c~0x9f	0xdc~0xdf
   0x07,0x0f,0x00,0x11 //...0x20~0x23	0x60~0x63	0xa0~0xA3	0xE0~0xE3
}; 
#endif

//=======================================================================
//              Page1 initialize table description : NTSC
//=======================================================================
BYTE tbl_ntsc_page1_quad_X[] = {
//   00   01   02   03   04   05   06   07     08   09   0A   0B   0C   0D   0E   0F
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,  0x00,0x00,0x80,0x00,0x00,0x00,0x00,0xB3,//...  0x00
   0x80,0x02,0x00,0x03,0x5a,0x01,0x3c,0x81,  0x02,0x00,0x5a,0xaf,0x01,0x3c,0x82,0x02,//...  0x10
   0x00,0x04,0x5a,0x3c,0x77,0x83,0x02,0x00,  0x5a,0xaf,0x3c,0x77                     //...  0x20 (43Byte)
};

BYTE tbl_ntsc_page1_quad_Y[] = {
//   00   01   02   03   04   05   06   07     08   09   0A   0B   0C   0D   0E   0F
        0x44,0x00,0x00,0x00,0x80,0x00,0x00,  0x00,0x00,0x00,0x00,0x23,0x2d,0x24,0x00,//...  0x30
   0x80,0x00,0x00,0x02,0x52,0x00,0x3c,0x81,  0x02,0x00,0x51,0xb4,0x00,0x3c,0x82,0x02,//...  0x40
   0x00,0x00,0x52,0x3c,0x78,0x83,0x02,0x00,  0x52,0xb4,0x3c,0x78                     //...  0x50 (43Byte)
};


BYTE tbl_ntsc_page1_enc[] = {
//   70   71   72   73   74   75   76   77	 78   79
   0x77,0x17,0x01,0xc0,0x08,0x1d,0x7c,0x09,0xAA,0x00 //(10 Byte)
};

#ifdef 0
//=======================================================================
//              Page1 initialize table description : PAL
//=======================================================================
BYTE tbl_pal_page1_quad_X[] = {
//   00   01   02   03   04   05   06   07     08   09   0A   0B   0C   0D   0E   0F
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,  0x00,0x00,0x80,0x00,0x00,0x00,0x00,0xB3,//...  0x00
   0x80,0x02,0x00,0x03,0x5a,0x01,0x48,0x81,  0x02,0x00,0x5a,0xb4,0x01,0x48,0x82,0x02,//...  0x10
   0x00,0x03,0x5a,0x48,0x8f,0x83,0x02,0x00,  0x5a,0xb0,0x48,0x8f                     //...  0x20
};
#else
//=======================================================================
//              Page1 initialize table description : PAL
//=======================================================================
BYTE tbl_pal_page1_quad_X[] = {
//   00   01   02   03   04   05   06   07     08   09   0A   0B   0C   0D   0E   0F
//      0x00,0x00,0x00,0x00,0x00,0x00,0x00,  0x00,0x00,0x80,0x00,0x23,0x2d,0x24,0xa7,//...  0x00
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,  0x00,0x00,0x80,0x00,0x00,0x00,0x00,0xB3,//...  0x00
   0x80,0x02,0x00,0x03,0x5a,0x01,0x32,0x81,  0x02,0x00,0x5a,0xb0,0x01,0x32,0x82,0x02,//...  0x10
   0x00,0x03,0x5a,0x32,0x64,0x83,0x02,0x00,  0x5a,0xb0,0x32,0x64                     //...  0x20
};
#endif
BYTE tbl_pal_page1_quad_Y[] = {
//   00   01   02   03   04   05   06   07     08   09   0A   0B   0C   0D   0E   0F
        0x44,0x00,0x00,0x00,0x80,0x00,0x00,  0x00,0x00,0x00,0x00,0x23,0x2d,0x24,0x00,//...  0x30
   0x80,0x00,0x00,0x00,0x53,0x00,0x48,0x81,  0x02,0x00,0x50,0xb4,0x00,0x48,0x82,0x02,//...  0x40
   0x00,0x00,0x53,0x48,0x90,0x83,0x02,0x00,  0x50,0xb4,0x48,0x90                     //...  0x50
};

BYTE tbl_pal_page1_enc[] = {
//   70   71   72   73   74   75   76   77	 78   79
   0x77,0x17,0x01,0xc0,0x06,0x1b,0x7c,0x4c,0xAA,0x00
};


void InitPage0()
{
   BYTE t, *tbl, std;
   
   std = jpeg.mStandard & STD_BIT;
   tbl = std ? tbl_ntsc_page0_quad : tbl_pal_page0_quad;
   for(t=0;t<4;t++)   ATD_Write_Table(PAGE0, 0x01+0x40*t, tbl, 35);
   ATD_Write_I2C_Register(PAGE0,0x7c,0x55); // force MPPDEC pins to VMD mode.
}


void InitPage1()
{
	if(jpeg.mStandard & STD_BIT){  //... NTSC
		ATD_Write_I2C_Register(PAGE1,REG_CASCADE,0x00);		//... NTSC
		ATD_Write_Table(PAGE1,0x01,tbl_ntsc_page1_quad_X,43);
		ATD_Write_Table(PAGE1,0x31,tbl_ntsc_page1_quad_Y,43);
		ATD_Write_Table(PAGE1,0x70,tbl_ntsc_page1_enc,10);		
	}
	else{                          //... PAL
		ATD_Write_I2C_Register(PAGE1,REG_CASCADE,0x80);		//... PAL
		ATD_Write_Table(PAGE1,0x01,tbl_pal_page1_quad_X,43);
		ATD_Write_Table(PAGE1,0x31,tbl_pal_page1_quad_Y,43);
		ATD_Write_Table(PAGE1,0x70,tbl_pal_page1_enc,10);
	}
}
//********************************************************************

 
/*
 * FUNCTTON: Set4VinRegs
 * DESCRIPTION: Update 4 video inputs registers for the same values (global param).
 * PARAM: page - registers page number.
 *        vin_reg - an array of 4 register address that are being update,
 *                  corresponding to vin number.
 *        bitmask - bitmask, bitval - bit values.
 */
void Set4VinRegs(BYTE page, BYTE *vin_reg ,BYTE bitmask, BYTE bitval)
{
   BYTE data, i = 0;
   for(;i < 4;i++)
   {
      data = ((ATD_Read_I2C_Register(page, vin_reg[i]))& ~bitmask)|bitval;
      ATD_Write_I2C_Register(page, vin_reg[i], data);
   }   
}

void SetFrameModeOnYPath(void)
{
   ATD_Write_I2C_Register(PAGE1,0x32,0x80);
   ATD_Write_I2C_Register(PAGE1,0x2c,0x00);
   ATD_Write_I2C_Register(PAGE1,0x2d,0x00);
   ATD_Write_I2C_Register(PAGE1,0x2e,0x00);
   ATD_Write_I2C_Register(PAGE1,0x2f,0x00);
   ATD_Write_I2C_Register(PAGE1,0x5c,0x08);
   ATD_Write_I2C_Register(PAGE1,0x5d,0x00);
   ATD_Write_I2C_Register(PAGE1,0x5e,0x00);

   ATD_Write_I2C_Register(PAGE1,0x31,0x44); // for quad 
}

void SetATDQuadMode(BYTE path)
{
   InitPage0();
   InitPage1();
   SetFrameModeOnYPath();
   jpeg.cam_mode &= 0x0F;  
}

/*
void SetATDQuadMode(BYTE path)
{
   BYTE vin, *tbl, std;
   
   std = jpeg.mStandard & STD_BIT;
   if(std)
   {
      if(path)
         ATD_Write_Table(PAGE1, 0x40, tbl_ntsc_page1_quad_Y+0x0f, 0x1c);
      else
         ATD_Write_Table(PAGE1, 0x10, tbl_ntsc_page1_quad_X+0x0f, 0x1c);
      
      tbl = tbl_ntsc_page0_quad;              
   }
   else
   {
      if(path)
         ATD_Write_Table(PAGE1, 0x40, tbl_pal_page1_quad_Y+0x0f, 0x1c);
      else
         ATD_Write_Table(PAGE1, 0x10, tbl_pal_page1_quad_X+0x0f, 0x1c);
         
      tbl = tbl_pal_page0_quad;   
   }
   
   for(vin=0;vin<4;vin++)
   {
      ATD_Write_I2C_Register(PAGE0,(0x18+2*path+(0x40*vin)) ,*(tbl+0x17+2*path));
      ATD_Write_I2C_Register(PAGE0,(0x1c+2*path+(0x40*vin)) ,*(tbl+0x1b+2*path));   
   }
   
   jpeg.cam_mode &= 0x0F;  
}
*/


/*
 * FUNCTTON: ColorKilling
 * DESCRIPTION: change to color/BW mode.
 * PARAM: val: ON -  BW mode, OFF - Color mode.
 */

void ColorKilling(HWORD val)
{
   BYTE vin_reg[4] = {0x14,0x54, 0x94, 0xD4};      
   Set4VinRegs(PAGE0, vin_reg, CKIL, val ? CKIL : 0);
}


void VideoStandardUpdate(void)
{
   InitPage0();
   InitPage1();
   SetFrameModeOnYPath();
}

/*
 * FUNCTTON: VideoStandardDetect
 * DESCRIPTION: detect the mejoraty video input standard. update jpeg.mStandard 
 *              (standard masks) and majority cam standard bit.
 */ 
void VideoStandardDetect(void)
{
   BYTE i, nPal, nNtsc, std, old_std, vin;
   i = nPal = nNtsc = 0;
   vin = CAMERA_MASK();
   
   old_std = jpeg.mStandard;
   jpeg.mStandard =0;
   for(; vin; i++, vin>>=1)
   {
      if(!(jpeg.mVideoLoss & (1<<i)))
      {
         if(vin & 1)
         {            
            std = ATD_Read_I2C_Register(PAGE0, 1+0x40*i)&1;
            if(std > PAL)     
               jpeg.mStandard |= 1<<i;
         }
         if(jpeg.mStandard & (1<<i))
            nNtsc++;
         else
            nPal++;
      }  
   }
   if(nNtsc+nPal==0) // no active camera
   {
         jpeg.mStandard |= (old_std & STD_BIT);
         for(vin=0;vin<4;vin++)
         {
            ATD_Write_I2C_Register(PAGE0, 0x78 ,0x0f);
            DelayBy(1000);
            ATD_Write_I2C_Register(PAGE0, 0x78 ,0);
         }         
   }
   else
   {
   // update majority cam standard bit:
   if(nNtsc >= nPal)
      jpeg.mStandard |= STD_BIT;
      
   if((jpeg.mStandard ^ old_std) & STD_BIT)
      VideoStandardUpdate();
   }
}


/*
 * FUNCTTON: VideoLossDetect
 * DESCRIPTION: find out which video inputs have an active video (jpeg.mVideoLoss),
 *              number of active cameras (jpeg.Total),
 *              first active camera (jpeg.first_active_cam),
 *              and when VideoLoss camera become an active camera,
 *              detect its standard.
 */
void VideoLossDetect(void)
{
   BYTE nv, tmp, fac;  
   nv = ATD_Read_I2C_Register(PAGE0, 0x39)>>4; // mask: 1 - inactive video, 0 - active.

   if((tmp = ((nv ^ jpeg.mVideoLoss)&CAMERA_MASK()))) // a change occur (video loss become an active or opp.)
   {
      if(tmp & jpeg.mVideoLoss) // if video-loss camera becomes an active camera.
         DelayBy(300000); // 300ms camera stabilization delay.        

      jpeg.mVideoLoss = nv|0x80;
      VideoStandardDetect();
      
      jpeg.Total = jpeg.first_active_cam = fac = 0;
      if(nv) // if not all cameras are active
      {
         nv = (~nv)&CAMERA_MASK(); 
         for(tmp=0;tmp<4;tmp++)
         {
            if(nv & (1<<tmp))
            {
               jpeg.Total++;
               if(fac == 0)
               {
                  jpeg.first_active_cam = tmp;
                  fac++;
               }            
            }         
         }                  
         if(jpeg.Total)
            (jpeg.Total)--;
      }
      else
         jpeg.Total = CAMERS_NUM-1;
   }
}


/*
 * FUNCTTON: BrightnessInit
 * DESCRIPTION: adjust brightness level according to 
 *              VideoCorrection[vin].Brightness.
 */
void BrightnessInit(void)
{
   BYTE vin;
   for(vin=0;vin<4;vin++)
   {
      ATD_Write_I2C_Register(PAGE0, 0x12+0x40*vin, GetBrightnessLevel(vin));
   }      
}
 
/*
 * FUNCTTON: BlinkCamBoundary
 * DESCRIPTION: change the boundary state of selected cameras (mask), 
 *              according to val: 1 - blink the selected camera, 0 - do not blink.
 */ 
void BlinkCamBoundary(HWORD mask, HWORD val)
{
   BYTE i, cam_val;
   for(i=0; i<CAMERS_NUM; i++)
   {
      if((mask>>i)&1)
      {
         cam_val = (val>>i)&1;
         ATD_Write_I2C_Register(PAGE1, 0x11+7*i, (cam_val)?0x03:0x02);
      }      
   }
}

void ATDPowerUpInit(void)
{

   jpeg.mStandard = STD_BIT;  // ATD default standard is NTSC.
   InitPage0();
   InitPage1();   
   SetFrameModeOnYPath();
   VideoLossDetect();
   BrightnessInit();
   ColorKilling(COLOR_SUSPEND() ? ON : OFF);
   ATDUpdateVmdVal();

   ATD_INIT_DONE();
}

