#ifndef _COMPILER_ABSTRACTION_H
#define _COMPILER_ABSTRACTION_H


#ifndef __ASM
    #define __ASM               __asm
#endif

#ifndef __INLINE
    #define __INLINE            inline
#endif

#ifndef __WEAK
    #define __WEAK              __attribute__((weak))
#endif

#ifndef __ALIGN
    #define __ALIGN(n)          __attribute__((aligned(n)))
#endif

#ifndef __PACKED
    #define __PACKED           __attribute__((packed)) 
#endif

#define GET_SP()                gcc_current_sp()

static inline unsigned int gcc_current_sp(void)
{
    register unsigned sp __ASM("sp");
    return sp;
}

#endif