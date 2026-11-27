/*
** Definitions for E2K CPUs.
** Copyright (C) 2005-2021 Mike Pall. See Copyright Notice in luajit.h
*/

#ifndef _LJ_TARGET_E2K_H
#define _LJ_TARGET_E2K_H

/* -- Registers IDs ------------------------------------------------------- */

/* Use registers from pipeline state to avoid misscomunications */
/* B0-B7 - rotating(arguments), R0-R2 - return value, R3-R31 - direct */
/* G16-G31 - global(tmpp reserved) */
/* PRED1-PRED3 - predicates, CTPR1-CTPR3 - cf */

#define GPRDEF(_) \
   _(B0)  _(B1)  _(B2)  _(B3)  _(B4)  _(B5)  _(B6)  _(B7) \
   _(R0)  _(R1)  _(R2)  _(R3)  _(R4)  _(R5)  _(R6)  _(R7) \
   _(R8)  _(R9) _(R10) _(R11) _(R12) _(R13) _(R14) _(R15) \
  _(R16) _(R17) _(R18) _(R19) _(R20) _(R21) _(R22) _(R23) \
  _(R24) _(R25) _(R26) _(R27) _(R28) _(R29) _(R30) _(R31) \
  _(G16) _(G17) _(G18) _(G19) _(G20) _(G21) _(G22) _(G23) \
  _(G24) _(G25) _(G26) _(G27) _(G28) _(G29) _(G30) _(G31)

#define NGPRDEF(_) \
  _(PRED0)  _(PRED1)  _(PRED2)  _(PRED3) \
  _(CTPR1) _(CTPR2) _(CTPR3)

#define FPRDEF(_)
#define VRIDDEF(_)
#define RIDENUM(name) RID_##name,

enum {
  GPRDEF(RIDENUM)  /* Directly addressable, rotating and global registers */
  RID_MAX,
  NGPRDEF(RIDENUM) /* Predicates and control flow registers */
  RID_TMP  = RID_G16, /* Keep in sync with E2K_REG_EXITNO */
  RID_TMP1 = RID_G16, /* Keep in sync with vm_exit_handler */
  RID_TMP2 = RID_G17,
  RID_TMP3 = RID_G18,
  RID_TMP4 = RID_G19,
  RID_BASE = RID_R4,  /* Interpreter BASE */
  RID_SP = RID_R6,  /* Interpreter stack pointer */
  RID_LPC = RID_R7, /* Interpreter PC */
  RID_DISPATCH = RID_R8, /* Interpreter dispatch */
  /* Calling conventions */
  RID_RETLO = RID_B0,
  RID_RETHI = RID_B1,
  RID_RET = RID_B0,
  RID_FPRET = RID_B0,

  RID_MIN_B = RID_B0,
  RID_MAX_B = RID_B7 + 1,
  RID_MIN_R = RID_R0,
  RID_MAX_R = RID_R31 + 1,
  RID_MIN_G = RID_G16,
  RID_MAX_G = RID_G31 + 1,
  RID_MIN_GPR = RID_MIN_B,
  RID_MAX_GPR = RID_MAX_G,
  RID_MIN_FPR = 0,
  RID_MAX_FPR = 0,
  RID_MIN_PRED = RID_PRED0,
  RID_MAX_PRED = RID_PRED3 + 1,
  RID_MIN_CTPR = RID_CTPR1,
  RID_MAX_CTPR = RID_CTPR3 + 1,

  RID_NUM_B = RID_MAX_B - RID_MIN_B,
  RID_NUM_R = RID_MAX_R - RID_MIN_R,
  RID_NUM_G = RID_MAX_G - RID_MIN_G,
  /* This definition must match with the *.dasc file(s).
   * See ->vm_exit_handler. */
  RID_NUM_GPR = RID_MAX_GPR - RID_MIN_GPR,
  RID_NUM_FPR = RID_MAX_FPR - RID_MIN_FPR,
  RID_NUM_PRED = RID_MAX_PRED - RID_MIN_PRED,
  RID_NUM_CTPR = RID_MAX_CTPR - RID_MIN_CTPR
};

#define RID_IN_RANGE(rid, min, max) ((rid) >= (min) && (rid) < (max))
#define RID_IS_B(rid)     RID_IN_RANGE(rid, RID_MIN_B, RID_MAX_B)
#define RID_IS_R(rid)     RID_IN_RANGE(rid, RID_MIN_R, RID_MAX_R)
#define RID_IS_G(rid)     RID_IN_RANGE(rid, RID_MIN_G, RID_MAX_G)
#define RID_IS_GPR(rid)   RID_IN_RANGE(rid, RID_MIN_B, RID_MAX_G)
#define RID_IS_PRED(rid)  RID_IN_RANGE(rid, RID_MIN_PRED, RID_MAX_PRED)
#define RID_IS_CTPR(rid)  RID_IN_RANGE(rid, RID_MIN_CTPR, RID_MAX_CTPR)

#define RID_NUM_KREF		RID_NUM_GPR
#define RID_MIN_KREF		RID_B0

/* -- Register sets ------------------------------------------------------- */

#define RSET_FIXED \
    (RID2RSET(RID_SP)|RID2RSET(RID_DISPATCH)|RID2RSET(RID_LPC) \
    |RSET_RANGE(RID_G16, RID_G20))
#define RSET_GPR    (RSET_RANGE(RID_MIN_GPR, RID_MAX_GPR) - RSET_FIXED)
#define RSET_FPR    0
#define RSET_ALL    RSET_GPR
#define RSET_INIT   RSET_ALL

#define RSET_SCRATCH_GPR  (RSET_RANGE(RID_MIN_B, RID_MAX_B) \
                          |RSET_RANGE(RID_G20, RID_MAX_G))
#define RSET_SCRATCH_FPR  0
#define RSET_SCRATCH    (RSET_SCRATCH_GPR|RSET_SCRATCH_FPR)

#define REGARG_FIRSTGPR RID_B0
#define REGARG_LASTGPR  RID_B7
#define REGARG_NUMGPR   8
#define STACKARG_OFS (8*8)
/* window setup */
#define JIT_WSZ     0x14 // (32 + 8)/2
#define JIT_NFX     0x1  // no fx operations
#define JIT_DBL     0x0  // disable 64-bit extension
#define JIT_RSZ     0x3  // (8)/2 - 1
#define JIT_RBS     0x10 // base of rotation
#define JIT_RCUR    0x0  // current base
#define JIT_WBS     JIT_RBS

/* -- Spill slots --------------------------------------------------------- */

/* Spill slots are 32 bit wide.
**
** SPS_FIXED: Available fixed spill slots in interpreter frame.
** This definition must match with the *.dasc file(s).
**
** SPS_FIRST: First spill slot for general use.
*/
#define SPS_FIXED 2
#define SPS_FIRST 2
#define SPOFS_TMP 0x0

#define sps_scale(slot)   (4 * (int32_t)(slot))
#define sps_align(slot)   (((slot) - SPS_FIXED + 3) & ~3)

/* -- Exit state ---------------------------------------------------------- */

/* This definition must match with the *.dasc file(s). */
typedef struct {
  intptr_t gpr[RID_NUM_GPR];
  int32_t spill[256];
} ExitState;

/* Highest exit + 1 indicates stack check. */
#define EXITSTATE_CHECKEXIT 1

#define E2K_BUNDLE_LEN(hs) (((hs) >> 4) * 2 + 2)
#define E2K_IS_NOP(hs) (((hs) & 0xfffffc0f) == 0)

/* Return the address of a per-trace exit stub. */
static LJ_AINLINE uint32_t *exitstub_trace_addr_(uint32_t *p, uint32_t exitno)
{
  while (E2K_IS_NOP(*p))
    p += E2K_BUNDLE_LEN(*p);
  return p;
}

/* Avoid dependence on lj_jit.h if only including lj_target.h. */
#define exitstub_trace_addr(T, exitno) \
  exitstub_trace_addr_((MCode *)((char *)(T)->mcode + (T)->szmcode), (exitno))

/* -- e2k bundle ---------------------------------------------------------- */

#define RES_SS_OFFSET   0
#define RES_ALS_OFFSET  (RES_SS_OFFSET + 1)
#define RES_CS_OFFSET   (RES_ALS_OFFSET + 6)
#define RES_AAS_OFFSET  (RES_CS_OFFSET + 2)
#define RES_LTS_OFFSET  (RES_AAS_OFFSET + 6)
#define RES_PLS_OFFSET  (RES_LTS_OFFSET + 4)
#define RES_CDS_OFFSET  (RES_PLS_OFFSET + 3)
#define RES_MASK_SIZE   (RES_CDS_OFFSET + 6)

#define RES_BIT(offset) (1ULL << (offset))

typedef enum E2kResourceMask {
  RES_NONE        = 0,
  RES_ALL         = RES_BIT(RES_MASK_SIZE) - 1,
  RES_INIT        = RES_ALL,

  RES_SS          = RES_BIT(RES_SS_OFFSET),
  RES_ALS0        = RES_BIT(RES_ALS_OFFSET + 0),
  RES_ALS1        = RES_BIT(RES_ALS_OFFSET + 1),
  RES_ALS2        = RES_BIT(RES_ALS_OFFSET + 2),
  RES_ALS3        = RES_BIT(RES_ALS_OFFSET + 3),
  RES_ALS4        = RES_BIT(RES_ALS_OFFSET + 4),
  RES_ALS5        = RES_BIT(RES_ALS_OFFSET + 5),
  RES_CS0         = RES_BIT(RES_CS_OFFSET + 0),
  RES_CS1         = RES_BIT(RES_CS_OFFSET + 1),
  RES_AAS0        = RES_BIT(RES_AAS_OFFSET + 0),
  RES_AAS1        = RES_BIT(RES_AAS_OFFSET + 1),
  RES_AAS2        = RES_BIT(RES_AAS_OFFSET + 2),
  RES_AAS3        = RES_BIT(RES_AAS_OFFSET + 3),
  RES_AAS4        = RES_BIT(RES_AAS_OFFSET + 4),
  RES_AAS5        = RES_BIT(RES_AAS_OFFSET + 5),
  RES_LTS0        = RES_BIT(RES_LTS_OFFSET + 0),
  RES_LTS1        = RES_BIT(RES_LTS_OFFSET + 1),
  RES_LTS2        = RES_BIT(RES_LTS_OFFSET + 2),
  RES_LTS3        = RES_BIT(RES_LTS_OFFSET + 3),
  RES_PLS0        = RES_BIT(RES_PLS_OFFSET + 0),
  RES_PLS1        = RES_BIT(RES_PLS_OFFSET + 1),
  RES_PLS2        = RES_BIT(RES_PLS_OFFSET + 2),
  RES_CDS00       = RES_BIT(RES_CDS_OFFSET + 0),
  RES_CDS01       = RES_BIT(RES_CDS_OFFSET + 1),
  RES_CDS10       = RES_BIT(RES_CDS_OFFSET + 2),
  RES_CDS11       = RES_BIT(RES_CDS_OFFSET + 3),
  RES_CDS20       = RES_BIT(RES_CDS_OFFSET + 4),
  RES_CDS21       = RES_BIT(RES_CDS_OFFSET + 5),
  RES_ALS_03      = RES_ALS0 | RES_ALS3,
  RES_ALS_14      = RES_ALS1 | RES_ALS4,
  RES_ALS_25      = RES_ALS2 | RES_ALS5,
  RES_ALS_0134    = RES_ALS_03 | RES_ALS_14,
  RES_ALS_0235    = RES_ALS_03 | RES_ALS_25,
  RES_ALS_012345  = RES_ALS_03 | RES_ALS_14 | RES_ALS_25,
  RES_ALS_ALL     = RES_ALS_012345,
  RES_CS_ALL      = RES_CS0 | RES_CS1,
  RES_LTS_ALL     = RES_LTS0 | RES_LTS1 | RES_LTS2 | RES_LTS3,
  RES_CDS0        = RES_CDS00 | RES_CDS01,
  RES_CDS1        = RES_CDS10 | RES_CDS11,
  RES_CDS2        = RES_CDS20 | RES_CDS21,
  RES_CDS_ALL     = RES_CDS0 | RES_CDS1 | RES_CDS2,
} E2kResourceMask;

typedef struct {
  uint8_t lng;
  uint8_t f1;
  uint8_t f2;
  uint8_t f3;
  uint8_t f4;
  uint8_t hs_cds;
  uint8_t hs_ales;
  uint8_t nop;
  E2kResourceMask res;
  uint32_t ss;
  uint32_t als[6];
  uint32_t cs[2];
  uint16_t ales[6];
  uint32_t lts[4];
  uint16_t cds[6];
  void *cs0_target;
} E2kBundle;

/* -- Instructions -------------------------------------------------------- */

/* Instruction fields. */
#define E2KF_DISP(x) (((uint32_t) (x) / 2) & 0x0fffffffu)

/* Check for valid field range. */
#define E2KF_S_OK(x, b) ((((x) + (1 << (b-1))) >> (b)) == 0)

typedef union {
  uint32_t i;
  struct {
    uint32_t mdl     : 4;
    uint32_t lng     : 3;
    uint32_t nop     : 3;
    uint32_t lm      : 1;
    uint32_t x_s_sw  : 3;
    uint32_t c       : 2;
    uint32_t cds     : 2;
    uint32_t pls     : 2;
    uint32_t ales    : 6;
    uint32_t als     : 6;
  } fields;
} E2kHS;

typedef union {
  uint32_t i;
  struct {
    uint32_t ctcond  : 9;
    uint32_t x       : 1;
    uint32_t ctop    : 2;
    uint32_t aa      : 4;
    uint32_t alc     : 2;
    uint32_t abp     : 2;
    uint32_t type    : 1;
    uint32_t abn     : 2;
    uint32_t abg     : 2;
    uint32_t rp_lo   : 1;
    uint32_t vfdi    : 1;
    uint32_t rp_hi   : 1;
    uint32_t bap     : 1;
    uint32_t eap     : 1;
    uint32_t ipd     : 2;
  } fields;
} E2kSS;

typedef union {
  uint32_t i;
  struct {
    uint32_t disp    : 28;
    uint32_t opc     : 2;
    uint32_t ctpr    : 2;
  } fields;
} E2kC0f2;

typedef union {
  uint32_t i;
  struct {
    uint32_t rbs       : 6;
    uint32_t rsz       : 6;
    uint32_t rcur      : 6;
    uint32_t psz       : 5;
    uint32_t x         : 3;
    uint32_t setbn     : 1;
    uint32_t setbp     : 1;
    uint32_t pad       : 4;
  } fields;
} E2kSetBR;

typedef union {
  uint32_t i;
  struct {
    uint32_t x       : 3;
    uint32_t dbl     : 1;
    uint32_t nfx     : 1;
    uint32_t wsz     : 7;
    uint32_t rpsz    : 5;
    uint32_t pad     : 15;
  } fields;
} E2kSetR;

typedef union {
  uint32_t i;
  struct {
    uint32_t params  : 28;
    uint32_t opc     : 4;
  } fields;
} E2kC1f1;

typedef union {
  uint16_t i;
  struct {
    uint16_t pred    : 7;
    uint16_t neg     : 3;
    uint16_t mask    : 4;
    uint16_t opc     : 2;
  } fields;
} E2kCDS;

typedef union {
  uint16_t i;
  struct {
    uint16_t opce2   : 8;
    uint16_t opc2    : 8;
  } fields;
} E2kAlef2;

typedef union {
  uint16_t i;
  struct {
    uint16_t src3    : 8;
    uint16_t opc2    : 8;
  } fields;
} E2kAlef1;

typedef enum {
  E2K_CONST =     0x0,
  E2K_CONST4 =    0x4,
  E2K_CONST5 =    0x5,
  E2K_CONST16 =   0x10,
  E2K_CONST32 =   0x20,
  E2K_CONST64 =   0x40,
  E2K_REG =       0x80,
  E2K_REG_R =     0x100,
  E2K_REG_B =     0x200,
  E2K_REG_G =     0x400,
  E2K_REG_PRED =  0x800,
  E2K_REG_CTPR =  0x1000,
  E2K_REG_RARG =  0x2000
} E2kOpT;

/* -- Scheduler ----------------------------------------------------------- */

#define E2K_SCHED_QUEUE   (1 << 0)
#define E2K_SCHED_NOPS    (1 << 1)
#define E2K_SCHED         (E2K_SCHED_NOPS | E2K_SCHED_QUEUE)
#define E2K_SCHED_WINDOW_SIZE 32

typedef struct E2kInsn E2kInsn;

typedef enum E2kAlopFlags {
  ALOP_NONE       = 0,
  ALOP_SPEC       = 1u << 0,
  ALOP_SRC1_GPR   = 1u << 1,
  ALOP_SRC2_GPR   = 1u << 2,
  ALOP_SRC2_LIT   = 1u << 3,
  ALOP_SRC3_GPR   = 1u << 4,
  ALOP_SRC4_GPR   = 1u << 5,
  ALOP_DST_REG    = 1u << 6,
  ALOP_COND_INV   = 1u << 7,
  ALOP_MRGC_INV   = 1u << 8,
} E2kAlopFlags;

struct E2kInsn {
  E2kInsn *next;
  E2kInsn *prev;
  uint32_t op;
  union {
    struct {
      uint64_t literal;
      uint16_t flags; /* E2kAlopFlags */
      uint16_t src1;
      uint16_t src2;
      uint16_t src3;
      union {
        uint16_t src4;
        uint16_t dst;
      };
      uint16_t cond;
      uint16_t mrgc;
    } alop;
    struct {
      void *target;
      int ctpr;
    } disp;
    struct {
      int ctpr;
      int pred;
      uint8_t inverted;
    } ct;
    struct {
      void *target;
      int pred;
      uint8_t inverted;
    } ibranch;
    struct {
      int ctpr;
      int pred;
      uint8_t inverted;
      uint8_t wbs;
    } call;
  };
};

typedef int16_t E2kCycle;

typedef enum E2kUseGprIndex {
  /* int unit */
  USE_GPR_I_C0,
  USE_GPR_I_C1,
  /* fpu unit */
  USE_GPR_F_C0,
  USE_GPR_F_C1,
  /* store unit */
  USE_GPR_S_C0,
  USE_GPR_S_C1,
  USE_GPR_LEN
} E2kUseGprIndex;

typedef struct E2kUseGpr {
  uint8_t used;
  /* Track the last source input for every unit in both clusters. */
  E2kCycle src[USE_GPR_LEN];
} E2kUseGpr;

typedef struct E2kUsePred {
  E2kCycle rlp;
  E2kCycle ct;
} E2kUsePred;

typedef struct E2kUseCtpr {
  E2kCycle ct;
} E2kUseCtpr;

typedef struct E2kSched {
  uint8_t enable;
  uint8_t dbg;
  uint8_t overflow;
  E2kCycle cycle;
  E2kInsn *head;
  E2kInsn *tail;
  E2kInsn *free;
  E2kUseGpr use_gpr[RID_NUM_B + RID_NUM_R + RID_NUM_G];
  E2kUsePred use_pred[RID_NUM_PRED];
  E2kUseCtpr use_ctpr[RID_NUM_CTPR];
  E2kInsn insn[E2K_SCHED_WINDOW_SIZE];
} E2kSched;

/* -- Opcodes ------------------------------------------------------------- */

#define E2K_DISP    0x0
#define E2K_IBRANCH 0x0
#define E2K_NOP(len, nops) \
  (((uint32_t) (nops) << 7) | (((uint32_t) (len) / 2 - 1) << 4))

#define E2K_SETR0   0x0
#define E2K_SETR1   0x1
#define E2K_SETEI   0x2
#define WAIT        0x3
#define E2K_SETBR   0x4
#define E2K_CALL    0x5
#define E2K_MAS     0x6
#define E2K_FLUSHR  0x7
#define E2K_BG      0x8

#define E2K_SETBN   0x1
#define E2K_SETBP   0x2
#define E2K_SETWD   0x4
#define E2K_VFRPSZ  0x8

#define SXT_BS      0x0
#define SXT_HS      0x1
#define SXT_WS      0x2
#define SXT_BZ      0x4
#define SXT_HZ      0x5
#define SXT_WZ      0x6

/* Keep in sync with e2kop. */
enum {
  /* ------------------------------------------------- */
  E2K_ANDS,     E2K_ANDD,     E2K_ANDNS,    E2K_ANDND,
  E2K_ORS,      E2K_ORD,      E2K_ORNS,     E2K_ORND,
  /* ------------------------------------------------- */
  E2K_XORS,     E2K_XORD,     E2K_XORNS,    E2K_XORND,
  E2K_SXT,      E2K_MERGES,   E2K_MERGED,
  /* ------------------------------------------------- */
  E2K_ADDS,     E2K_ADDD,     E2K_SUBS,     E2K_SUBD,
  E2K_SCLS,     E2K_SCLD,     E2K_SCRS,     E2K_SCRD,
  /* ------------------------------------------------- */
  E2K_SHLS,     E2K_SHLD,     E2K_SHRS,     E2K_SHRD,
  E2K_SARS,     E2K_SARD,     E2K_GETFS,    E2K_GETFD,
  /* ------------------------------------------------- */
  E2K_CMPBSB,   E2K_CMPESB,   E2K_CMPBESB,  E2K_CMPLSB,
  E2K_CMPLESB,  E2K_CMPBDB,   E2K_CMPEDB,   E2K_CMPBEDB,
  E2K_CMPLDB,   E2K_CMPLEDB,  E2K_CMPANDESB,
  E2K_STB,      E2K_STH,      E2K_STW,      E2K_STD,
  /* ------------------------------------------------- */
  E2K_FCMPEQSB, E2K_FCMPLTSB, E2K_FCMPLESB, E2K_FCMPNLTSB,
  E2K_FCMPNLESB,E2K_FCMPEQDB, E2K_FCMPLTDB, E2K_FCMPLEDB,
  E2K_FCMPNLTDB,E2K_FCMPNLEDB,
  /* ------------------------------------------------- */
  E2K_FADDS,    E2K_FADDD,    E2K_FSUBS,    E2K_FSUBD,
  E2K_FMINS,    E2K_FMIND,    E2K_FMAXS,    E2K_FMAXD,
  /* ------------------------------------------------- */
  E2K_FMULS,    E2K_FMULD,    E2K_FSTOISTR, E2K_ISTOFS,
  E2K_FDTOIDTR, E2K_IDTOFD,   E2K_FSTOIDTR, E2K_ISTOFD,
  E2K_FSTOFD,   E2K_FDTOISTR, E2K_IDTOFS,   E2K_FDTOFS,
  /* ------------------------------------------------- */
  E2K_MOVTD,    E2K_LDB,      E2K_LDH,      E2K_LDW,
  E2K_LDD,
  /* ------------------------------------------------- */
  E2K_MULS,     E2K_MULD,     E2K_UMULX,    E2K_SMULX,
  /* ------------------------------------------------- */
  E2K_FDIVD,    E2K_FSQRTID,
  /* ------------------------------------------------- */
  E2K_FSQRTTD,
  /* ------------------------------------------------- */
  E2K_GETSP,
  /* ------------------------------------------------- */
  E2K_FDTOIFD,
  /* ------------------------------------------------- */
  E2K_PSHUFB,
  /* ------------------------------------------------- */
  E2K_ALOP_END,
  E2K_INSN_DISP = E2K_ALOP_END,
  E2K_INSN_CT,
  E2K_INSN_IBRANCH,
  E2K_INSN_CALL
};
#endif
