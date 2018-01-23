/*

Copyright (c) Future Technology Devices International 2014

THIS SOFTWARE IS PROVIDED BY FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

FTDI DRIVERS MAY BE USED ONLY IN CONJUNCTION WITH PRODUCTS BASED ON FTDI PARTS.

FTDI DRIVERS MAY BE DISTRIBUTED IN ANY FORM AS LONG AS LICENSE INFORMATION IS NOT MODIFIED.

IF A CUSTOM VENDOR ID AND/OR PRODUCT ID OR DESCRIPTION STRING ARE USED, IT IS THE
RESPONSIBILITY OF THE PRODUCT MANUFACTURER TO MAINTAIN ANY CHANGES AND SUBSEQUENT WHQL
RE-CERTIFICATION AS A RESULT OF MAKING THESE CHANGES.

Author : FTDI 

Revision History: 
0.1 - date 2013.04.24 - Initial Version
0.2 - date 2013.08.19 - made few changes.

*/

#ifndef _FT_HAL_UTILS_H_
#define _FT_HAL_UTILS_H_


#define RGB(r, g, b)  ((((vc_int32_t)(r)) << 16) | (((vc_int32_t)(g)) << 8) | (b))
#define SQ(v) ((v) * (v))
#define MIN(x,y)  ((x) > (y) ? (y) : (x))
#define MAX(x,y)  ((x) > (y) ? (x) : (y))
#define PLAYCOLOR        0x00A0A080
#define NOTE(n, sharp)   (((n) - 'C') + ((sharp) * 128))
#define F16(s)           ((vc_int32_t)((s) * 65536))
#define INVALID_TOUCH_XY   0x8000
#define ABS(x)  ((x) > (0) ? (x) : (-x))
#define ALIGN_TWO_POWER_N(Value,alignval) (((Value) + (alignval - 1))&(~(alignval - 1)))

#endif /* _FT_HAL_UTILS_H_ */



