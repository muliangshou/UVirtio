#define BIT(i) (1 << (i))

#define SET_BIT(reg, n) reg |= BIT(n)
#define CLR_BIT(reg, n) reg &= ~BIT(n)

#define SET_FIELD(reg, field, val)                              \
    reg = (reg & ~MASK(field)) | FIELD(field, val)

#define GET_FIELD(reg, field) \
    ((reg >> field##_Pos) & MASK0(field))

#define FIELD(field, val) \
    (((val) & MASK0(field)) << field##_Pos)

#define MASK(field) (MASK0(field) << field##_Pos)

#define MASK0(field) (~((-2) << (field##_Wid-1)))

#define SET_BYTE(reg, n, v) \
     reg = (reg & ~(0xff << 8*n)) | ((v & 0xff) << 8*n)

#define ADDR(x) (* (unsigned volatile *) (x))
#define ARRAY(x) ((unsigned volatile *) (x))
