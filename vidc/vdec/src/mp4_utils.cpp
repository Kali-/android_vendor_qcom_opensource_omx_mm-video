/*--------------------------------------------------------------------------
Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--------------------------------------------------------------------------*/
#include "mp4_utils.h"
#include "omx_vdec.h"
# include <stdio.h>

#ifdef ENABLE_TURBO_CLK_FOR_HIGH_MPEG4_SLICES
#define VISUAL_OBJECT_SEQUENCE_START_CODE 0x000001B0
#define VISUAL_OBJECT_SEQUENCE_END_CODE 	0x000001B1
#define USER_DATA_START_CODE	            0x000001B2
#define GROUP_START_CODE	                0x000001B3
#define VISUAL_OBJECT_START_CODE          0x000001B5
#define GOV_START_CODE                    0x000001B3
#define VOP_START_CODE                    0x000001B6

#define VIDEO_OBJECT_START_CODE_MASK        0xFFFFFFE0  // mask first 27 bits (ISO/IEC 14496-2, 6.3.2)
#define VIDEO_OBJECT_START_CODE             0x00000100
#define VIDEO_OBJECT_LAYER_START_CODE_MASK  0xFFFFFFF0  // mask first 28 bits (ISO/IEC 14496-2, 6.3.3)
#define VIDEO_OBJECT_LAYER_START_CODE       0x00000120


#define START_CODE_PREFIX       0x00000100
#define START_CODE_MASK         0xFFFFFF00

#define MPEG4_RESYNC_MARKER_BASE_LEN 16

#define CHK_RETURN_VALUE()      \
      {                       \
        if(ret<0)             \
        {                     \
          LOGE("%s:%d returned %d",__func__,__LINE__,ret);    \
          return ret;                     \
        }                                 \
      }
typedef struct  {
  unsigned char* Mp4bits;       //pointer to raw bitstream
  unsigned long  numBytes;      //Number of bytes in the video bitstream
  unsigned long Fpos;            /* forward bit position */
  unsigned long FBits;           /* no. of forward bits left in the buffer */
  unsigned long bitBuf;          /* Internal bit buffer for forward reading */
  unsigned long Rpos;            /* Reverse bit position */
  unsigned long RBits;           /* no. of bits in bitBufRev already used */

}mp4_slice_type;
mp4_slice_type mp4_slice;

unsigned int resync_marker_disable;
unsigned int data_partitioned;
unsigned char NBitsTime;
unsigned int quant_precision;
unsigned int video_object_layer_shape;
unsigned int fcode_forward;
unsigned int fcode_backward;
unsigned int ResyncMarkerLength;
unsigned int numFramesParsed;
bool high_slices_present = false;
bool parse_bitstream_for_slices = true;

#define MPEG4_IVOP 0
#define MPEG4_PVOP 1
#define MPEG4_BVOP 2
#define MPEG4_SVOP 3

#define MPEG4_SHAPE_RECTANGULAR	    0x00
#define MPEG4_SHAPE_BINARY 		      0x01
#define MPEG4_SHAPE_BINARY_ONLY     0x02
#define MPEG4_SHAPE_GRAYSCALE 	    0x03

unsigned int num_slices_threshold=50;
unsigned int num_frames_to_check_for_turbo = 50;

#define FETCH_FORWARD_BITS_IF_NEEDED( mp4_slice, BitPosition )             \
  {                                                                     \
    if ((BitPosition) > 32)                                             \
    {                                                                   \
      unsigned char *pBits;                                                     \
                                                                        \
      /* update bit position */                                         \
      mp4_slice.FBits = mp4_slice.Fpos & 7;                                 \
      /* Calculate starting byte */                                     \
      pBits = &mp4_slice.Mp4bits[mp4_slice.Fpos >> 3];                      \
      /* Combine 4 data bytes in an endian-independent manner */        \
      mp4_slice.bitBuf = (pBits[0]<<24) | (pBits[1]<<16) |                \
                       (pBits[2]<<8) | pBits[3];                        \
    }                                                                   \
  }


int mpeg4_parse_bitstream(char* mp4bits, unsigned long bufsize,unsigned int *slice_cnt);
void mpeg4_init_internal(char* mp4bits,unsigned long bufsize);
int mpeg4_get_start_code(unsigned int *startcode);
int mpeg4_frame_seek_start_code(unsigned int *startcode);
int decode_VOL_header();
int decode_vop();
int get_slice_count(unsigned int *slice_cnt);
int mpeg4_slice_read_bits(unsigned int   NumberOfBitsToRead,
                          unsigned int  *pResult);
int mpeg4_slice_flush_bits(unsigned int  NumberOfBitsToFlush);
#endif

MP4_Utils::MP4_Utils()
{
   m_SrcWidth = 0;
   m_SrcHeight = 0;
   vop_time_resolution = 0;
   vop_time_found = false;

}
MP4_Utils::~MP4_Utils()
{
}

uint32 MP4_Utils::read_bit_field(posInfoType * posPtr, uint32 size) {
   uint8 *bits = &posPtr->bytePtr[0];
   uint32 bitBuf =
       (bits[0] << 24) | (bits[1] << 16) | (bits[2] << 8) | bits[3];

   uint32 value = (bitBuf >> (32 - posPtr->bitPos - size)) & MASK(size);

   /* Update the offset in preparation for next field    */
   posPtr->bitPos += size;

   while (posPtr->bitPos >= 8) {
      posPtr->bitPos -= 8;
      posPtr->bytePtr++;
   }
   return value;
}
static uint8 *find_code
    (uint8 * bytePtr, uint32 size, uint32 codeMask, uint32 referenceCode) {
   uint32 code = 0xFFFFFFFF;
   for (uint32 i = 0; i < size; i++) {
      code <<= 8;
      code |= *bytePtr++;

      if ((code & codeMask) == referenceCode) {
         return bytePtr;
      }
   }

   DEBUG_PRINT_LOW("Unable to find code 0x%x\n", referenceCode);
   return NULL;
}
bool MP4_Utils::parseHeader(mp4StreamType * psBits) {
   uint32 profile_and_level_indication = 0;
   uint8 VerID = 1; /* default value */
   long hxw = 0;

   m_posInfo.bitPos = 0;
   m_posInfo.bytePtr = psBits->data;
   m_dataBeginPtr = psBits->data;

   m_posInfo.bytePtr = find_code(m_posInfo.bytePtr,4,
                                 MASK(32),VOP_START_CODE);
   if(m_posInfo.bytePtr) {
      return false;
   }

   m_posInfo.bitPos = 0;
   m_posInfo.bytePtr = psBits->data;
   m_dataBeginPtr = psBits->data;
   m_posInfo.bytePtr = find_code(m_posInfo.bytePtr,4,
                                 MASK(32),GOV_START_CODE);
   if(m_posInfo.bytePtr) {
      return false;
   }

   m_posInfo.bitPos = 0;
   m_posInfo.bytePtr = psBits->data;
   m_dataBeginPtr = psBits->data;
   /* parsing Visual Object Seqence(VOS) header */
   m_posInfo.bytePtr = find_code(m_posInfo.bytePtr,
                                 psBits->numBytes,
                                 MASK(32),
                                 VISUAL_OBJECT_SEQUENCE_START_CODE);
   if ( m_posInfo.bytePtr == NULL ){
      m_posInfo.bitPos  = 0;
      m_posInfo.bytePtr = psBits->data;
   }
   else {
      uint32 profile_and_level_indication = read_bit_field (&m_posInfo, 8);
   }
   /* parsing Visual Object(VO) header*/
   /* note: for now, we skip over the user_data */
   m_posInfo.bytePtr = find_code(m_posInfo.bytePtr,psBits->numBytes,
                             MASK(32),VISUAL_OBJECT_START_CODE);
   if(m_posInfo.bytePtr == NULL) {
      m_posInfo.bitPos = 0;
      m_posInfo.bytePtr = psBits->data;
   }
   else {
      uint32 is_visual_object_identifier = read_bit_field (&m_posInfo, 1);
      if ( is_visual_object_identifier ) {
         /* visual_object_verid*/
         read_bit_field (&m_posInfo, 4);
         /* visual_object_priority*/
         read_bit_field (&m_posInfo, 3);
      }

      /* visual_object_type*/
      uint32 visual_object_type = read_bit_field (&m_posInfo, 4);
      if ( visual_object_type != VISUAL_OBJECT_TYPE_VIDEO_ID ) {
        return false;
      }
      /* skipping video_signal_type params*/
      /*parsing Video Object header*/
      m_posInfo.bytePtr = find_code(m_posInfo.bytePtr,psBits->numBytes,
                                    VIDEO_OBJECT_START_CODE_MASK,VIDEO_OBJECT_START_CODE);
      if ( m_posInfo.bytePtr == NULL ) {
        return false;
      }
   }

   /* parsing Video Object Layer(VOL) header */
   m_posInfo.bitPos = 0;
   m_posInfo.bytePtr = find_code(m_posInfo.bytePtr,
                            psBits->numBytes,
                            VIDEO_OBJECT_LAYER_START_CODE_MASK,
                            VIDEO_OBJECT_LAYER_START_CODE);
   if ( m_posInfo.bytePtr == NULL ) {
      m_posInfo.bitPos = 0;
      m_posInfo.bytePtr = psBits->data;
   }

   // 1 -> random accessible VOL
   read_bit_field(&m_posInfo, 1);

   uint32 video_object_type_indication = read_bit_field (&m_posInfo, 8);
   if ( (video_object_type_indication != SIMPLE_OBJECT_TYPE) &&
       (video_object_type_indication != SIMPLE_SCALABLE_OBJECT_TYPE) &&
       (video_object_type_indication != CORE_OBJECT_TYPE) &&
       (video_object_type_indication != ADVANCED_SIMPLE) &&
       (video_object_type_indication != RESERVED_OBJECT_TYPE) &&
       (video_object_type_indication != MAIN_OBJECT_TYPE)) {
      return false;
   }
   /* is_object_layer_identifier*/
   uint32 is_object_layer_identifier = read_bit_field (&m_posInfo, 1);
   if (is_object_layer_identifier) {
      uint32 video_object_layer_verid = read_bit_field (&m_posInfo, 4);
      uint32 video_object_layer_priority = read_bit_field (&m_posInfo, 3);
      VerID = (unsigned char)video_object_layer_verid;
   }

  /* aspect_ratio_info*/
  uint32 aspect_ratio_info = read_bit_field (&m_posInfo, 4);
  if ( aspect_ratio_info == EXTENDED_PAR ) {
    /* par_width*/
    read_bit_field (&m_posInfo, 8);
    /* par_height*/
    read_bit_field (&m_posInfo, 8);
  }
   /* vol_control_parameters */
   uint32 vol_control_parameters = read_bit_field (&m_posInfo, 1);
   if ( vol_control_parameters ) {
      /* chroma_format*/
      uint32 chroma_format = read_bit_field (&m_posInfo, 2);
      if ( chroma_format != 1 ) {
         return false;
      }
      /* low_delay*/
      uint32 low_delay = read_bit_field (&m_posInfo, 1);
      /* vbv_parameters (annex D)*/
      uint32 vbv_parameters = read_bit_field (&m_posInfo, 1);
      if ( vbv_parameters ) {
         /* first_half_bitrate*/
         uint32 first_half_bitrate = read_bit_field (&m_posInfo, 15);
         uint32 marker_bit = read_bit_field (&m_posInfo, 1);
         if ( marker_bit != 1) {
            return false;
         }
         /* latter_half_bitrate*/
         uint32 latter_half_bitrate = read_bit_field (&m_posInfo, 15);
         marker_bit = read_bit_field (&m_posInfo, 1);
         if ( marker_bit != 1) {
            return false;
         }
         uint32 VBVPeakBitRate = (first_half_bitrate << 15) + latter_half_bitrate;
         /* first_half_vbv_buffer_size*/
         uint32 first_half_vbv_buffer_size = read_bit_field (&m_posInfo, 15);
         marker_bit = read_bit_field (&m_posInfo, 1);
         if ( marker_bit != 1) {
            return false;
         }
         /* latter_half_vbv_buffer_size*/
         uint32 latter_half_vbv_buffer_size = read_bit_field (&m_posInfo, 3);
         uint32 VBVBufferSize = (first_half_vbv_buffer_size << 3) + latter_half_vbv_buffer_size;
         /* first_half_vbv_occupancy*/
         uint32 first_half_vbv_occupancy = read_bit_field (&m_posInfo, 11);
         marker_bit = read_bit_field (&m_posInfo, 1);
         if ( marker_bit != 1) {
            return false;
         }
         /* latter_half_vbv_occupancy*/
         uint32 latter_half_vbv_occupancy = read_bit_field (&m_posInfo, 15);
         marker_bit = read_bit_field (&m_posInfo, 1);
         if ( marker_bit != 1) {
            return false;
         }
      }/* vbv_parameters*/
   }/*vol_control_parameters*/

   /* video_object_layer_shape*/
   uint32 video_object_layer_shape = read_bit_field (&m_posInfo, 2);
   uint8 VOLShape = (unsigned char)video_object_layer_shape;
   if ( VOLShape != MPEG4_SHAPE_RECTANGULAR ) {
       return false;
   }
   /* marker_bit*/
   uint32 marker_bit = read_bit_field (&m_posInfo, 1);
   if ( marker_bit != 1 ) {
      return false;
   }
   /* vop_time_increment_resolution*/
   uint32 vop_time_increment_resolution = read_bit_field (&m_posInfo, 16);
   vop_time_resolution = vop_time_increment_resolution;
   vop_time_found = true;
   return true;
}

bool MP4_Utils::is_notcodec_vop(unsigned char *pbuffer, unsigned int len)
{
   unsigned int index = 4,vop_bits=0;
   unsigned int temp = vop_time_resolution - 1;
   unsigned char vop_type=0,modulo_bit=0,not_coded=0;
   if (!vop_time_found || !pbuffer || len < 5) {
      return false;
   }
   if((pbuffer[0] == 0) && (pbuffer[1] == 0) && (pbuffer[2] == 1) && (pbuffer[3] == 0xB6)){
      while(temp) {
         vop_bits++;
         temp >>= 1;
      }
      vop_type = (pbuffer[index] & 0xc0) >> 6;
      unsigned bits_parsed = 2;
      do {
            modulo_bit = pbuffer[index]  & (1 << (7-bits_parsed));
            bits_parsed++;
            index += bits_parsed/8;
            bits_parsed = bits_parsed %8;
            if(index >= len) {
               return false;
            }
      }while(modulo_bit);
      bits_parsed++; //skip marker bit
      bits_parsed += vop_bits + 1;//Vop bit & Marker bits
      index += bits_parsed/8;
      if(index >= len) {
         return false;
      }
      bits_parsed = bits_parsed % 8;
      not_coded = pbuffer[index] & (1 << (7 - bits_parsed));
      if(!not_coded){
         return true;
      }
   }
   return false;
}
#ifdef ENABLE_TURBO_CLK_FOR_HIGH_MPEG4_SLICES
int mpeg4_parse_bitstream(char* mp4bits, unsigned long bufsize,unsigned int *slice_cnt)
{
  unsigned int start_code;
  int ret=0;

  mpeg4_init_internal(mp4bits,bufsize);

  do
  {
    ret = mpeg4_get_start_code(&start_code);
    if(ret <0 ){
     DEBUG_PRINT_ERROR(" no start code found ");
      return ret;
    }

     switch(start_code) {
       case VIDEO_OBJECT_LAYER_START_CODE:
       {
          ret = decode_VOL_header();
          if(ret < 0) {
            return ret;
          }
       }
       break;
       case VOP_START_CODE:
       {
          if(!resync_marker_disable)
          {
            ret = decode_vop();
            if(ret < 0) {
                return ret;
            }
            get_slice_count(slice_cnt);
          }
          else {
            *slice_cnt = 0;
          }
       }
       break;
    }
     //check end of stream
     if(mp4_slice.Fpos >= mp4_slice.Rpos){
       break;
     }
  } while (1);
  return ret;
}

void mpeg4_init_internal(char* mp4bits,unsigned long bufsize)
{
  mp4_slice.Mp4bits = (unsigned char*)mp4bits;
  mp4_slice.numBytes = bufsize;
  mp4_slice.FBits = 32;
  mp4_slice.RBits = 32;
  mp4_slice.Fpos = 0;
  mp4_slice.Rpos = (mp4_slice.numBytes << 3) - 1;
  mp4_slice.bitBuf = 0;
}

int mpeg4_get_start_code(unsigned int *startcode)
{
  int ret=0;
  do
  {
    ret = mpeg4_frame_seek_start_code(startcode);
    if(ret < 0) {
      //No start code found, flush the whole bitstream
      mp4_slice.Fpos = mp4_slice.Rpos;
      return ret;
    }
    if (*startcode >= 0x120 && *startcode <= 0x12F) {
      *startcode = VIDEO_OBJECT_LAYER_START_CODE;
    }

    if ((*startcode == VOP_START_CODE) ||
        (*startcode == VIDEO_OBJECT_LAYER_START_CODE)){
      break;
    }
  } while (1);
  return ret;
}

int mpeg4_frame_seek_start_code(unsigned int *startcode)
{
  unsigned int code;
  unsigned char *pBits;
  unsigned int Fpos=0;
  bool codeFound = false;

  int i = 0;
  int flag = 0;
  Fpos = mp4_slice.Fpos & ~(0x7);
  pBits = &mp4_slice.Mp4bits[Fpos >> 3];

  code = (pBits[i]<<24) | (pBits[i+1]<<16) |
         (pBits[i+2]<<8) | pBits[i+3];

  pBits += 4;

  while (1)
  {
    if ((code & START_CODE_MASK) == START_CODE_PREFIX) {
			codeFound = true;
      break;
		}

    if ((Fpos + 8) >= mp4_slice.Rpos ) {
			break;
		}
		else {
			code <<= 8;
			code |= *pBits++;
			Fpos += 8;
		}
  }

  if(codeFound) {
    *startcode = code;
    mp4_slice.Fpos = Fpos + 32;
    mp4_slice.FBits = 32;
    DEBUG_PRINT_LOW(" startcode = %x ",code);
    return 0;
  }
  *startcode = 0xFFFFFFFF;
  DEBUG_PRINT_LOW(" start code not found!!");
  return -1;
}


int decode_VOL_header()
{
  int ret;
  unsigned char *pBits;
  unsigned int Fpos;
  unsigned int start_code;

  mpeg4_slice_flush_bits(1);

  unsigned int video_object_type_indication;
  ret = mpeg4_slice_read_bits(8, &video_object_type_indication);
  DEBUG_PRINT_LOW("video_object_type_indication = %d",video_object_type_indication);
  CHK_RETURN_VALUE()

  if(video_object_type_indication == 0x12) {
    DEBUG_PRINT_ERROR(" ERROR: not supported!!");
    return -1;
  }

  unsigned int is_object_layer_identifier;
  ret = mpeg4_slice_read_bits(1, &is_object_layer_identifier);
  DEBUG_PRINT_LOW("is_object_layer_identifier = %d",is_object_layer_identifier);
  CHK_RETURN_VALUE()

  unsigned int video_object_layer_verid = 1; /* default value */

  if(is_object_layer_identifier) {
    ret = mpeg4_slice_read_bits(4, &video_object_layer_verid);
    DEBUG_PRINT_LOW("video_object_layer_verid = %d",video_object_layer_verid);
    CHK_RETURN_VALUE()
    mpeg4_slice_flush_bits(3);
  }

  unsigned int aspect_ratio;
  ret = mpeg4_slice_read_bits(4, &aspect_ratio);
  DEBUG_PRINT_LOW("aspect_ratio = %d",aspect_ratio);
  CHK_RETURN_VALUE()
  if(aspect_ratio == 0xf) {
    ret = mpeg4_slice_flush_bits(8+8);
    CHK_RETURN_VALUE()
  }

  unsigned int vol_control_parameters;
  ret = mpeg4_slice_read_bits(1, &vol_control_parameters);
  DEBUG_PRINT_LOW("vol_control_parameters = %d",vol_control_parameters);
  CHK_RETURN_VALUE()

  if (vol_control_parameters) {
    mpeg4_slice_flush_bits(2+1);
    unsigned int vbv_parameters;
    ret = mpeg4_slice_read_bits(1, &vbv_parameters);
    DEBUG_PRINT_LOW("vbv_parameters = %d",vbv_parameters);
    CHK_RETURN_VALUE()

    if(vbv_parameters) {
      mpeg4_slice_flush_bits(15+1 + 15+1 + 15+1 + 3+11 + 1 +15+1);
    }
  }

  ret = mpeg4_slice_read_bits(2, &video_object_layer_shape);
  DEBUG_PRINT_LOW("video_object_layer_shape = %d",video_object_layer_shape);
  if(video_object_layer_shape != 0) {
    DEBUG_PRINT_ERROR(" Error: Non Rectangular shape");
    return -1;
  }

  mpeg4_slice_flush_bits(1);

 /* vop_time_increment_resolution*/
  unsigned int vop_time_increment_resolution;
  ret = mpeg4_slice_read_bits(16, &vop_time_increment_resolution);
  DEBUG_PRINT_LOW("vop_time_increment_resolution = %d",vop_time_increment_resolution);
  CHK_RETURN_VALUE()

  mpeg4_slice_flush_bits(1);

  /* compute the nr. of bits for time information in bitstream*/
  {
    int i,j;

    i = vop_time_increment_resolution-1;
    j = 0;
    while (i)
    {
      j++;
      i>>=1;
    }
        if (j)
          NBitsTime = j;
        else
          NBitsTime = 1;
  }

  DEBUG_PRINT_LOW(" NBitsTime = %d",NBitsTime);

   /* fixed_vop_rate*/
  unsigned int fixed_vop_rate;
  ret = mpeg4_slice_read_bits(1, &fixed_vop_rate);
  DEBUG_PRINT_LOW("fixed_vop_rate = %d",fixed_vop_rate);
  CHK_RETURN_VALUE()

  if ( fixed_vop_rate ) {
    /* fixed_vop_increment*/
    unsigned int fixed_vop_increment;
    ret = mpeg4_slice_read_bits(NBitsTime, &fixed_vop_increment);
    DEBUG_PRINT_LOW("fixed_vop_increment = %d",fixed_vop_increment);
    CHK_RETURN_VALUE()
  }

  /* video_object_layer_shape == rectangular*/
  mpeg4_slice_flush_bits(1 + 13 + 1 + 13 + 1);

  mpeg4_slice_flush_bits(1 + 1);

  unsigned int sprite_enable;
  DEBUG_PRINT_ERROR("video_object_layer_verid = %d",video_object_layer_verid);
  if(video_object_layer_verid == 1){
    ret = mpeg4_slice_read_bits(1, &sprite_enable);
  }
  else {
    ret = mpeg4_slice_read_bits(2, &sprite_enable);
  }

  if (sprite_enable) {
    DEBUG_PRINT_ERROR("Error: Sprite_enable");
    return -1;
  }
  CHK_RETURN_VALUE()

  unsigned int not_8_bit;
  ret = mpeg4_slice_read_bits(1, &not_8_bit);
  DEBUG_PRINT_LOW("not_8_bit = %d",not_8_bit);
  CHK_RETURN_VALUE()

  if(not_8_bit) {
    ret = mpeg4_slice_read_bits(4, &quant_precision);
    DEBUG_PRINT_LOW("quant_precision = %d",quant_precision);
    CHK_RETURN_VALUE()
    mpeg4_slice_flush_bits(4);
  }
  else {
    quant_precision = 5; /* Default value when not_8_bit is 0 */
  }

  unsigned int quant_type;
  ret = mpeg4_slice_read_bits(1, &quant_type);
  DEBUG_PRINT_LOW("quant_type = %d",quant_type);
  CHK_RETURN_VALUE()

  if(quant_type) {
    /* load_intra_quant_mat */
    unsigned int load_intra_quant_mat;
    ret = mpeg4_slice_read_bits(1, &load_intra_quant_mat);
    DEBUG_PRINT_LOW("load_intra_quant_mat = %d",load_intra_quant_mat);
    CHK_RETURN_VALUE()

    if ( load_intra_quant_mat ) {
      unsigned int intra_quant_mat = 1;
      int i;
      for (i=0; i<64 && intra_quant_mat ; i++) {
        ret = mpeg4_slice_read_bits(8, &intra_quant_mat);
        CHK_RETURN_VALUE()
      }
    }

    /* load_nonintra_quant_mat  */
    unsigned int load_nonintra_quant_mat;
    ret = mpeg4_slice_read_bits(1, &load_nonintra_quant_mat);
    DEBUG_PRINT_LOW("load_nonintra_quant_mat = %d",load_nonintra_quant_mat);
    CHK_RETURN_VALUE()

    if ( load_nonintra_quant_mat ) {
      unsigned int nonintra_quant_mat = 1;
      int i;
      for (i=0; i<64 && nonintra_quant_mat ; i++) {
        ret = mpeg4_slice_read_bits(8, &nonintra_quant_mat);
        CHK_RETURN_VALUE()
      }
    }
  }

  if(video_object_layer_verid !=1) {
     mpeg4_slice_flush_bits(1);
  }

  /* complexity_estimation_disable*/
  unsigned int complexity_estimation_disable;
  ret = mpeg4_slice_read_bits(1, &complexity_estimation_disable);
  DEBUG_PRINT_LOW("complexity_estimation_disable = %d",complexity_estimation_disable);
  CHK_RETURN_VALUE()
  if (!complexity_estimation_disable ) {
    DEBUG_PRINT_ERROR("ERROR - VOP_COMPLEXITY_ESTIMATE ");
    return -1;
  }

  /* resync_marker_disable*/
  ret = mpeg4_slice_read_bits(1, &resync_marker_disable);
  DEBUG_PRINT_HIGH(" resync_marker_disable = %d", resync_marker_disable);
  CHK_RETURN_VALUE()

  if(resync_marker_disable) {
    parse_bitstream_for_slices = false;
  }

  /* data_partitioned*/
  ret = mpeg4_slice_read_bits(1, &data_partitioned);
  if(data_partitioned)
  {
    num_slices_threshold *= 2;
  }
  DEBUG_PRINT_HIGH(" data_partitioned = %d", data_partitioned);
  CHK_RETURN_VALUE()

  //signal end of header processing
  mp4_slice.Fpos = mp4_slice.Rpos;
  return 0;
}

int decode_vop()
{
   int ret =0;
   unsigned int Result;
   unsigned int predtype;

   ret = mpeg4_slice_read_bits(2, &predtype );
   CHK_RETURN_VALUE()

   do    /* modulo time base */
   {
     ret = mpeg4_slice_read_bits(1, &Result );
     CHK_RETURN_VALUE()
   } while (Result);

  mpeg4_slice_flush_bits(1);

  /* "vop_time_increment" */
  ret = mpeg4_slice_read_bits(NBitsTime,&Result);
  CHK_RETURN_VALUE()

  mpeg4_slice_flush_bits(1);

  /* Is the VOP coded? */
  ret = mpeg4_slice_read_bits(1, &Result );
  CHK_RETURN_VALUE()

  /* If not coded vop return success */
  if(!Result) {
    DEBUG_PRINT_ERROR(" VOP not coded!!");
    return 0;
  }

 /* "rounding_type" */
  if (predtype == MPEG4_PVOP) {
    mpeg4_slice_flush_bits(1);
  }

  /* intra_dc_vlc_thr */
  mpeg4_slice_flush_bits(3);

  if(video_object_layer_shape != MPEG4_SHAPE_BINARY_ONLY) {
    if(quant_precision < 3 || quant_precision > 9) {
      DEBUG_PRINT_ERROR("quant_precision out of range ... = %d",quant_precision);
      return -1;
    }
    ret = mpeg4_slice_read_bits(quant_precision,&Result);
    CHK_RETURN_VALUE()

    if(predtype != MPEG4_IVOP){
       ret = mpeg4_slice_read_bits( 3, &fcode_forward);
       DEBUG_PRINT_LOW("fcode_forward = %d",fcode_forward);
       CHK_RETURN_VALUE()
    }

    if (predtype == MPEG4_BVOP) {
      ret = mpeg4_slice_read_bits( 3, &fcode_backward);
      DEBUG_PRINT_LOW("fcode_backward = %d",fcode_backward);
      CHK_RETURN_VALUE()
    }
  }

  //compute resync marker length
  if (predtype == MPEG4_IVOP ||
      video_object_layer_shape == MPEG4_SHAPE_BINARY_ONLY) {
        ResyncMarkerLength = MPEG4_RESYNC_MARKER_BASE_LEN + 1;
  }
  else if (predtype == MPEG4_PVOP){
    ResyncMarkerLength =
    MPEG4_RESYNC_MARKER_BASE_LEN + fcode_forward;
  }
  else if (predtype == MPEG4_BVOP) {
    unsigned int reSyncMarkerLengthTemp = MPEG4_RESYNC_MARKER_BASE_LEN +
                             (fcode_forward > fcode_backward ? fcode_forward : fcode_backward);
    ResyncMarkerLength = (reSyncMarkerLengthTemp > 18 ? reSyncMarkerLengthTemp : 18);
  }
  return 0;
}

int get_slice_count(unsigned int *slice_cnt)
{
  unsigned int BytePosition;
  unsigned char *pBits;
  unsigned int CurrentData;
  unsigned int slicecount=0;

  BytePosition = mp4_slice.Fpos >> 3;
  pBits = mp4_slice.Mp4bits;

  while((BytePosition+2) < mp4_slice.numBytes)
  {
    CurrentData = ((pBits[BytePosition+0]<<16) |
               (pBits[BytePosition+1]<<8) |
               (pBits[BytePosition+2])) & 0xFFFFFF;

    if((CurrentData >> (24 - ResyncMarkerLength)) == 0x1)
      slicecount++;

    if(slicecount > num_slices_threshold) {
      break;
    }
    BytePosition++;
  }

  //signal stream end
  mp4_slice.Fpos = mp4_slice.Rpos;
  DEBUG_PRINT_LOW(" slicecount = %u",slicecount);

  if(slice_cnt) {
    *slice_cnt = slicecount;
  }
  return 0;
}

int mpeg4_slice_read_bits (unsigned int NumberOfBitsToRead,
                           unsigned int  *pResult)
{
  /* Check if there are enough bits to service the request */
  if ( (mp4_slice.Fpos + NumberOfBitsToRead) > mp4_slice.Rpos+1 ) {
    DEBUG_PRINT_ERROR("%s ...buffer overrun ",__func__);
    return -1;
  }

  FETCH_FORWARD_BITS_IF_NEEDED( mp4_slice, mp4_slice.FBits + NumberOfBitsToRead );

  /* Update buffer pointers to reflect used bits */
  mp4_slice.Fpos  += NumberOfBitsToRead;
  mp4_slice.FBits += NumberOfBitsToRead;

  /* Pick out the correct bits out of the 32 bit buffer */
  *pResult = (mp4_slice.bitBuf >> (32-mp4_slice.FBits)) &
    ((1 << NumberOfBitsToRead) - 1);

  return 0;
}

int mpeg4_slice_flush_bits(unsigned int  NumberOfBitsToFlush)
{
  if ( (mp4_slice.Fpos + NumberOfBitsToFlush) > mp4_slice.Rpos+1 ) {
    DEBUG_PRINT_ERROR("%s ...buffer overrun ",__func__);
    return -1;
  }

  /* Update buffer pointers to reflect used bits */
  mp4_slice.Fpos  += NumberOfBitsToFlush;
  mp4_slice.FBits += NumberOfBitsToFlush;

  FETCH_FORWARD_BITS_IF_NEEDED( mp4_slice, mp4_slice.FBits );

  return 0;
}
#endif
