/******************************************************************************
* 功能:                              linux 下写的test驱动平台
*        
* 文件:  MT5715 wireless.h
*        
* 说明:  无底板 模拟测试
*        
* 作者:  杨诚斋  11/5/2018
******************************************************************************/


#ifndef _MT5715_WIRELESS_
#define _MT5715_WIRELESS_

#define LBIT(n)      (1<<(n))
#define _LBIT(n)     (~BIT(n))

/* CMD reg 0x04*/
#define CEPMUTE                 LBIT(0)
#define CLRINT                  LBIT(1)
#define LDOTGL                  LBIT(2)
#define SENDPPP                 LBIT(3)
#define FASTCHARGE              LBIT(4)
/* CMD reg 0x04*/

/* reg */
#define  REG_CHIPID   0x0000
#define  REG_FW_VER   0x0002
#define  REG_CMD      0x0004
#define  REG_INT_FLAG 0x0008
#define  REG_INTCLR   0x000A
#define  REG_VOUT     0x000E
#define  REG_VRECT    0x0010
#define  REG_VOUTSET  0x0014
#define  REG_VFC      0x001E
/* reg */

#define  MT5715ID     0x5715


#endif
