#include <cheri/cheric.h>

#define CHERI_PRINT_CAP(cap)                            \
	printf("%-20s: %-16s t:%d s:%d p:%08jx "			\
	       "b:%016jx l:%016zx o:%jx type:%lx\n",		\
	   __func__,							            \
	   #cap,							                \
	   cheri_gettag(cap),						        \
	   cheri_getsealed(cap),					        \
	   cheri_getperm(cap),						        \
	   cheri_getbase(cap),						        \
	   cheri_getlen(cap),						        \
	   cheri_getoffset(cap),						    \
	   cheri_gettype(cap))

#define CHERI_PRINT_CAP_LITE(cap)					    \
	printf("t:%x s:%x b:0x%16jx l:0x%16zx o:0x%jx",		\
	   cheri_gettag(cap),						        \
	   cheri_getsealed(cap),					        \
	   cheri_getbase(cap),						        \
	   cheri_getlen(cap),						        \
	   cheri_getoffset(cap))