#pragma once
/* GPIO */
#define GPIO_DIRSET    ADDR(0x50000518)
#define GPIO_DIRCLR    ADDR(0x5000051c)
#define GPIO_PINCNF   ARRAY(0x50000700)

#define GPIO_PINCNF_PULL_Pos 2
#define GPIO_PINCNF_PULL_Wid 2
#define GPIO_Pullup 0x3

#define GPIO_PINCNF_DRIVE_Pos 8
#define GPIO_PINCNF_DRIVE_Wid 3
#define GPIO_S0D1 0x6 // Open drain