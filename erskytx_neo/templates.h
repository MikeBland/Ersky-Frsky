/****************************************************************************
*  Copyright (c) 2021 by Michael Blandford. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************/

#ifndef TEMPLATES_H
#define TEMPLATES_H

#include <inttypes.h>


#define STK_RUD  1
#define STK_ELE  2
#define STK_THR  3
#define STK_AIL  4
#define STK_P1   5
#define STK_P2   6
#define STK_P3   7
#define NUM_TEMPLATES    9  // DIM(n_Templates)
#define NUM_TEMPLATE_MIX 8
#define TEMPLATE_NLEN    15

#define TRIM_ON  0
#define TRIM_OFF 1

//#ifdef FIX_MODE
#define CM(x)	(x)
//#else
//#define CM(x) (CONVERT_MODE(x))  //good for SRC
//#endif
#define CH(x) (CHOUT_BASE+(x))
#define CV(x) (CURVE_BASE+(x)-1)
#define CC(x) (CHANNEL_ORDER(x)) //need to invert this to work with dest

#define CURVE5(x) ((x)-1)
#define CURVE9(x) (MAX_CURVE5+(x)-1)

//extern const uint16_t n_Templates[] ;
extern const char *n_Templates[] ;




void clearMixes() ;
void clearCurves() ;
void applyTemplate(uint8_t idx) ;


#endif //TEMPLATES_H


