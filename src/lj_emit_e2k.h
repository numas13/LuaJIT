/*
** E2K instruction emitter.
** Copyright (C) 2005-2025 Mike Pall. See Copyright Notice in luajit.h
*/

/* -- Declarations -------------------------------------------------------- */

/* Keep in sync with e2kres. */
typedef enum E2kInsnType {
  TYPE_INT,
  TYPE_IMUL,
  TYPE_ICMP,
  TYPE_SYS,
  TYPE_LOAD,
  TYPE_STORE,
  TYPE_FADD,
  TYPE_FMUL,
  TYPE_FDIV,
  TYPE_FSQRTT,
  TYPE_FCMP,
  TYPE_FMISC,
  TYPE_FCVT,
  TYPE_VINT,
  TYPE_PREP,
  TYPE_CT,
} E2kInsnType;

typedef enum E2kUnit {
  UNIT_INT,
  UNIT_FPU,
  UNIT_CT,
} E2kUnit;

typedef enum E2kIn {
  IN_I,     /* int inputs. */
  IN_F,     /* fp or ver inputs. */
  IN_S,     /* store source input. */
  IN_S_C,   /* like IN_S but from a another cluster. */
  IN_R,     /* all inputs from a another cluster. */
  IN_RLP,
  IN_ELP,
  IN_CT,
  IN_LEN,
} E2kIn;

typedef enum E2kOut {
  OUT_NONE,
  OUT_INT,
  OUT_ICMP,
  OUT_SYS,
  OUT_LOAD,
  OUT_FP,
  OUT_FCMP,
  OUT_FMISC,
  OUT_FDIV,
  OUT_FSQRT,
  OUT_FSQRTT,
  OUT_VINT,
  OUT_DISP,
  OUT_LEN,
} E2kOut;

typedef struct E2kReservation {
  uint8_t unit;   /* E2kUnit */
  uint8_t out;    /* E2kOut */
  E2kResourceMask mask;
} E2kReservation;

typedef struct E2kOp {
  char *name;
  uint8_t type; /* E2kInsnType */
  uint16_t ales;
  uint32_t als;
} E2kOp;

static uint8_t e2klat[OUT_LEN][IN_LEN] = { 0 };
static const E2kReservation e2kres[];
static const E2kOp e2kop[];

/* -- Bundling helpers ---------------------------------------------------- */

static E2kResourceMask try_alloc_resource(E2kBundle *bundle,
                                          E2kResourceMask mask)
{
  E2kResourceMask res = bundle->res & mask;
  if (res) {
    res = res & -res;
    bundle->res &= ~res;
  }
  return res;
}

static E2kResourceMask alloc_resource(ASMState *as, E2kBundle *bundle,
                                      E2kResourceMask mask)
{
  E2kResourceMask res = try_alloc_resource(bundle, mask);
  lj_assertA(res, "no available resources");
  return res;
}

static unsigned get_syllable_index(ASMState *as, E2kResourceMask res,
                                   unsigned offset)
{
  lj_assertA(res, "empty resource");
  return __builtin_ctz(res >> offset);
}

static unsigned alloc_syllable(ASMState *as, E2kBundle *bundle,
                               E2kResourceMask mask, unsigned offset)
{
  E2kResourceMask res = alloc_resource(as, bundle, mask);
  return get_syllable_index(as, res, offset);
}

#define checku4(x)  ((x) == (int32_t)(uint8_t)(x & 0xf))
#define checku5(x)  ((x) == (int32_t)(uint8_t)(x & 0x1f))

static intptr_t get_kval(ASMState *as, IRRef ref)
{
  IRIns *ir = IR(ref);
  if (ir->o == IR_KNULL || !irt_is64(ir->t)) {
    lj_assertA(ir->o == IR_KINT || ir->o == IR_KNULL,
               "bad 64 bit const IR op %d", ir->o);
    return ir->i; /* Sign-extended. */
  } else {
    return (intptr_t)ir_k64(ir)->u64;
  }
}

static E2kOpT get_reg_type(ASMState *as, intptr_t val)
{
  UNUSED(as);
  if (RID_IS_B(val))
    return E2K_REG_B;
  else if (RID_IS_R(val))
    return E2K_REG_R;
  else if (RID_IS_G(val))
    return E2K_REG_G;
  else if (RID_IS_PRED(val))
    return E2K_REG_PRED;
  else if (RID_IS_CTPR(val))
    return E2K_REG_CTPR;
  else
    lj_assertA(0, "bad register (%d)", val);
  return 0;
}

static E2kOpT get_const_type(intptr_t val)
{
  if (checku4(val))
    return E2K_CONST4;
  else if(checku5(val))
    return E2K_CONST5;
  else if (checki16(val))
    return E2K_CONST16;
  else if (checki32(val))
    return E2K_CONST32;
  else
    return E2K_CONST64;
}

/* -- Bundling ------------------------------------------------------------ */

static void bundle_reset(E2kBundle *bundle)
{
  memset(bundle, 0, sizeof(E2kBundle));
  bundle->res = RES_INIT;
  bundle->f1 = 1; /* HS itself */
}

static MCode *bundle_pack(ASMState *as, MCode *mxp)
{
  uint32_t hs_x_s_sw = (as->bundle.res & RES_SS) ? 0 : 0x2;
  uint32_t hs_c = (~as->bundle.res & RES_CS_ALL) >> RES_CS_OFFSET;
  uint32_t hs_cds = (as->bundle.hs_cds + 1) & -2;
  uint32_t hs_als = (~as->bundle.res & RES_ALS_ALL) >> RES_ALS_OFFSET;
  uint32_t hs_ales = as->bundle.hs_ales;

  uint8_t f1 = as->bundle.f1;
  uint8_t f2 = as->bundle.f2;
  int half_pad = as->bundle.f3 & 0x1;
  uint8_t f3 = (as->bundle.f3 >> 1) + half_pad;
  uint8_t f4 = as->bundle.f4 + (hs_cds >> 1);
  uint8_t lng = f1 + f2 + f3 + f4;
  uint8_t hs_lng = (lng + 1) & -2;

  /* Emit extra nops. */
  if (as->bundle.nop > 7) {
    int tail_nops = as->bundle.nop % 8;
    *--mxp = 0;
    *--mxp = E2K_NOP(2, tail_nops);
    as->bundle.nop -= tail_nops + 1;
    while (as->bundle.nop > 7) {
      as->bundle.nop -= 8;
      *--mxp = 0;
      *--mxp = E2K_NOP(2, 7);
    }
  }

  if (as->bundle.lng) {
    lj_assertA(hs_lng <= as->bundle.lng, "bundle overflow");
    hs_lng = as->bundle.lng;
  }
  lj_assertA(hs_lng % 2 == 0 && hs_lng <= 16, "invalid bundle size");

  if ((hs_c & 0x01) && as->bundle.cs0_target) {
    ptrdiff_t delta = (MCode *) as->bundle.cs0_target - mxp + hs_lng;
    lj_assertA(delta % 2 == 0, "branch target not aligned");
    lj_assertA(E2KF_S_OK(delta, 29), "branch target out of range");
    as->bundle.cs[0] &= 0xf0000000;
    as->bundle.cs[0] |= E2KF_DISP(delta);
  }

  E2kHS hs;
  hs.i = 0;
  hs.fields.mdl = (uint32_t)(f1 + f2 - 1);
  hs.fields.lng = (uint32_t)(hs_lng >> 1) - 1;
  hs.fields.nop = as->bundle.nop;
  hs.fields.x_s_sw = hs_x_s_sw;
  hs.fields.c = hs_c;
  hs.fields.cds = hs_cds >> 1;
  hs.fields.ales = hs_ales;
  hs.fields.als = hs_als;

  /* CDS[2-0], 16-bit-pairs */
  if (hs_cds > 0) *--mxp = (as->bundle.cds[0] << 16) | as->bundle.cds[1];
  if (hs_cds > 2) *--mxp = (as->bundle.cds[2] << 16) | as->bundle.cds[3];
  if (hs_cds > 4) *--mxp = (as->bundle.cds[4] << 16) | as->bundle.cds[5];
  /* LTS[3-0], 32-bit, 16-bit half syls not implemented */
  int used_lts = ((~as->bundle.res & RES_LTS_ALL)  >> RES_LTS_OFFSET);
  for (int i = 0; i < 4; i++) {
    if (used_lts & (1 << i)) {
      *--mxp = as->bundle.lts[i];
    }
  }

  /* Write padding syllables to match required bundle size. */
  for (int i = lng; i < hs_lng; i++)
    *--mxp = 0;

  /* ALES[0-1,3-4], 16-bit */
  uint32_t tmp_syl = 0;
  int hi = half_pad ? 1 : 0;
  for (int i = 5; i >= 0; i--) {
    if (i == 2 || i == 5) continue; // skip in v3
    if (hs_ales & (1 << i)) {
      if (hi) {
        *--mxp = ((uint32_t)as->bundle.ales[i] << 16) | tmp_syl;
        tmp_syl = 0;
        hi = 0;
      } else {
        tmp_syl = (uint32_t)as->bundle.ales[i];
        hi = 1;
      }
    }
  }

  /* CS[0-1], 32-bit */
  if (hs_c & 0x02) *--mxp = as->bundle.cs[1];
  if (hs_c & 0x01) {
    E2kC0f2 cs0 = { as->bundle.cs[0] };
    if (as->bundle.cs0_target == NULL) {
      /* Adjust disp if target is set manually. */
      cs0.fields.disp = cs0.fields.disp + (hs_lng >> 1);
    }
    *--mxp = cs0.i;
  }
  /* ALS[0-5], 32-bit */
  if (hs_als & 0x20) *--mxp = as->bundle.als[5];
  if (hs_als & 0x10) *--mxp = as->bundle.als[4];
  if (hs_als & 0x08) *--mxp = as->bundle.als[3];
  if (hs_als & 0x04) *--mxp = as->bundle.als[2];
  if (hs_als & 0x02) *--mxp = as->bundle.als[1];
  if (hs_als & 0x01) *--mxp = as->bundle.als[0];
  /* SS and HS */
  if (hs_x_s_sw) *--mxp = as->bundle.ss;
  *--mxp = hs.i; 

  return mxp;
}

static void bundle_finalize(ASMState *as, MCode **mxp)
{
  if (mxp) {
    *mxp = bundle_pack(as, *mxp);
    bundle_reset(&as->bundle);
  }
}

/* -- Bundle basic instructions ------------------------------------------- */

static uint32_t bundle_lts(ASMState *as, uint64_t mask, E2kOpT type, uint64_t val)
{
  /* 16-bit halfsyls not managed yet, using lo part only */
  uint64_t lts = 0;
  int lts_idx = 0;
  if (type == E2K_CONST16) {
    mask &= RES_LTS1|RES_LTS0;
  } else if (type == E2K_CONST32) {
    mask &= RES_LTS3|RES_LTS2|RES_LTS1|RES_LTS0;
  } else {
    mask &= RES_LTS2|RES_LTS1|RES_LTS0;
  }
  lts = alloc_resource(as, &as->bundle, mask);
  if (type == E2K_CONST64) /* check 2 syls for 64-bit literal */
    alloc_resource(as, &as->bundle, lts << 1);
  lts = lts >> RES_LTS_OFFSET;
  while (lts >>= 1)
    lts_idx++;

  switch (type) {
  case E2K_CONST16:
    as->bundle.lts[lts_idx] = (uint16_t)(val); break;
  case E2K_CONST32:
    as->bundle.lts[lts_idx] = (uint32_t)(val); break;
  case E2K_CONST64:
    as->bundle.lts[lts_idx] = (uint32_t)(val);
    as->bundle.lts[lts_idx+1] = (uint32_t)(val >> 32);
    as->bundle.f4++;
    break;
  }
  as->bundle.f4++;
  return lts_idx;
}

static uint32_t bundle_src1(ASMState *as, E2kOpT type, intptr_t src1)
{
  UNUSED(as);
  if (type == E2K_REG) {
    switch (get_reg_type(as, src1)) {
    case E2K_REG_B:
      return src1 - RID_B0;
    case E2K_REG_R:
      return (src1 - RID_R0) | 0x80;
    case E2K_REG_G:
      return (src1 - RID_G16 + 16) | 0xe0;
    default:
      lj_assertA(0, "bad reg for src1 (%d)", src1);
      return 0;
    }
  } else {
    switch (get_const_type(src1)) {
    case E2K_CONST4: case E2K_CONST5:
      return src1 | 0xc0;
    default:
      lj_assertA(0, "bad type for src1 (%d)", type);
      return 0;
    }
  }
}

static uint32_t bundle_src2(ASMState *as, E2kOpT type, intptr_t src2)
{
  if (type == E2K_REG) {
    switch (get_reg_type(as, src2)) {
    case E2K_REG_B:
      return src2 - RID_B0;
    case E2K_REG_R:
      return (src2 - RID_R0) | 0x80;
    case E2K_REG_G:
      return (src2 - RID_G16 + 16) | 0xe0;
    default:
      lj_assertA(0, "bad reg for src2 (%d)", src2);
      return 0;
    }
  } else {
    switch (get_const_type(src2)) {
    case E2K_CONST4:
      return src2 | 0xc0;
    case E2K_CONST5: case E2K_CONST16:
      return (bundle_lts(as, RES_LTS_ALL, E2K_CONST16, src2) | 0xd0);
    case E2K_CONST32:
      return (bundle_lts(as, RES_LTS_ALL, E2K_CONST32, src2) | 0xd8);
    case E2K_CONST64:
      return (bundle_lts(as, RES_LTS_ALL, E2K_CONST64, src2) | 0xdc);
    default:
      lj_assertA(0, "bad type for src2 (%d)", type);
      return 0;
    }
  }
}

static uint32_t bundle_src3(ASMState *as, E2kOpT type, intptr_t src3)
{
  UNUSED(as);
  if (type == E2K_REG) {
    switch (get_reg_type(as, src3)) {
    case E2K_REG_B:
      return src3 - RID_B0;
    case E2K_REG_R:
      return (src3 - RID_R0) | 0x80;
    case E2K_REG_G:
      return (src3 - RID_G16 + 16) | 0xe0;
    default:
      lj_assertA(0, "bad reg for src3 (%d)", src3);
    }
  } else {
    lj_assertA(0, "bad type for src3 (%d)", type);
  }
}

static uint32_t bundle_dst(ASMState *as, E2kOpT type, intptr_t dst)
{
  UNUSED(as);
  if (type == E2K_REG) {
    switch (get_reg_type(as, dst)) {
    case E2K_REG_B:
      return dst - RID_B0;
    case E2K_REG_R:
      return (dst - RID_R0) | 0x80;
    case E2K_REG_CTPR:
      return (dst - RID_CTPR1 + 1) | 0xd0;
    case E2K_REG_G:
      return (dst - RID_G16 + 16) | 0xe0;
    default:
      lj_assertA(0, "bad reg for dst (%d)", dst);
    }
  } else {
    lj_assertA(0, "bad type for dst (%d)", type);
  }
}

static uint32_t bundle_pdst(ASMState *as, E2kOpT type, intptr_t pred)
{
  UNUSED(as);
  if (type == E2K_REG_PRED) {
    return pred - RID_PRED0;
  } else {
    lj_assertA(0, "bad reg for pdst (%d)", pred);
    return 0;
  }
}

static int bundle_alop(ASMState *as, MCode **p, int spec, uint32_t op,
                       uint8_t src1, uint8_t src2, uint8_t src3, uint8_t dst)
{
  E2kInsnType type = e2kop[op].type;
  E2kResourceMask mask = e2kres[type].mask;
  int als_idx = alloc_syllable(as, &as->bundle, mask, RES_ALS_OFFSET);
  uint32_t als = e2kop[op].als;
  als |= dst;
  als |= (uint32_t) src2 << 8;
  als |= (uint32_t) src1 << 16;
  als |= spec ? 1 << 31 : 0;
  as->bundle.als[als_idx] = als;
  as->bundle.f1++;
  if (e2kop[op].ales) {
    as->bundle.hs_ales |= 1 << als_idx;
    as->bundle.ales[als_idx] = e2kop[op].ales | src3;
    if (als_idx != 2 && als_idx != 5)
      as->bundle.f3++;
  }
  bundle_finalize(as, p);
  return als_idx;
}

static void bundle_cds(ASMState *as, int opc, int als, Reg pred, int inverted,
                       MCode **p)
{
  int cds_idx = alloc_syllable(as, &as->bundle, RES_CDS_ALL, RES_CDS_OFFSET);
  E2kCDS syl;
  syl.i = 0;
  syl.fields.pred = (pred - RID_PRED0)|0x60;
  switch (1 << als) {
  case 0x1: case 0x8:
     if (inverted) syl.fields.neg = 1;
     syl.fields.mask = 1;
     break;
  case 0x2: case 0x10:
     if (inverted) syl.fields.neg = 2;
     syl.fields.mask = 2;
     break;
  case 0x4: case 0x20:
     if (inverted) syl.fields.neg = 4;
     syl.fields.mask = 4;
     break;
  }
  if (als >= 3) opc++;
  syl.fields.opc = opc;
  as->bundle.cds[cds_idx] = (uint32_t)syl.i;
  as->bundle.hs_cds++;
  bundle_finalize(as, p);
}

#define bundle_rlp(as, als, pred, inv, p) \
  bundle_cds(as, 0, als, pred, inv, p)
#define bundle_mrgc(as, als, pred, inv, p) \
  bundle_cds(as, 2, als, pred, inv, p)
#define ISRC1(src1) bundle_src1(as, E2K_CONST, src1)
#define RSRC1(src1) bundle_src1(as, E2K_REG, src1)
#define ISRC2(src2) bundle_src2(as, E2K_CONST, src2)
#define RSRC2(src2) bundle_src2(as, E2K_REG, src2)
#define RSRC3(src3) bundle_src3(as, E2K_REG, src3)
#define RSRC4(src4) bundle_src3(as, E2K_REG, src4)
#define RDST(dst)  bundle_dst(as, E2K_REG, dst)
#define PDST(pred) bundle_pdst(as, E2K_REG_PRED, pred)
#define bundle_alopf7_rr(as, spec, op, src1, src2, pred, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), RSRC2(src2), 0, PDST(pred))
#define bundle_alopf7_ri(as, spec, op, src1, src2, pred, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), ISRC2(src2), 0, PDST(pred))
#define bundle_alopf3_ri(as, spec, op, src1, src2, src3, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), ISRC2(src2), 0, RSRC3(src3))
#define bundle_alopf12_i(as, spec, op, src2, dst, p) \
  bundle_alop(as, p, spec, op, 0, ISRC2(src2), 0, RDST(dst))
#define bundle_alopf12_r(as, spec, op, src2, dst, p) \
  bundle_alop(as, p, spec, op, 0, RSRC2(src2), 0, RDST(dst))
#define bundle_alopf2_i(as, spec, op, src2, dst, p) \
  bundle_alop(as, p, spec, op, 0, ISRC2(src2), 0, RDST(dst))
#define bundle_alopf2_r(as, spec, op, src2, dst, p) \
  bundle_alop(as, p, spec, op, 0, RSRC2(src2), 0, RDST(dst))
#define bundle_alopf21_rrr(as, spec, op, src1, src2, src3, dst, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), RSRC2(src2), RSRC3(src3), RDST(dst))
#define bundle_alopf11_rr(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), RSRC2(src2), 0, RDST(dst))
#define bundle_alopf11_ir(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, ISRC1(src1), RSRC2(src2), 0, RDST(dst))
#define bundle_alopf11_ri(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), ISRC2(src2), 0, RDST(dst))
#define bundle_alopf11_ii(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, ISRC1(src1), ISRC2(src2), 0, RDST(dst))
#define bundle_alopf1_rr(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), RSRC2(src2), 0, RDST(dst))
#define bundle_alopf1_ir(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, ISRC1(src1), RSRC2(src2), 0, RDST(dst))
#define bundle_alopf1_ri(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, RSRC1(src1), ISRC2(src2), 0, RDST(dst))
#define bundle_alopf1_ii(as, spec, op, src1, src2, dst, p) \
  bundle_alop(as, p, spec, op, ISRC1(src1), ISRC2(src2), 0, RDST(dst))
#define bundle_snapno(as, snapno, p) \
  bundle_alopf1_ii(as, 0, E2K_ADDD, (snapno) < 32 ? (snapno) : 31, \
      (snapno) < 32 ? 0 : (snapno) - 31, RID_TMP, p);
#define bundle_nop(as, nops) \
  (as)->bundle.nop = (nops)
#define bundle_len(as, len) \
  (as)->bundle.lng = (len)

/* -- Bundle control-flow instructions ------------------------------------ */

static void bundle_ct(ASMState *as, Reg ctpr, Reg pred, int inverted, MCode **p)
{
  alloc_resource(as, &as->bundle, RES_SS); /* Uses only part of RES_SS */
  E2kSS syl;
  syl.i = 0;
  if (pred) { /* RID_PREDX is nonnull  */
    if (inverted) {
      syl.fields.ctcond = 0x60 + (pred - RID_PRED0);
    } else {
      syl.fields.ctcond = 0x40 + (pred - RID_PRED0);
    }
  } else {
    syl.fields.ctcond = 0x20; /* unconditional */
  }
  if (ctpr)  /* RID_CTPRX is nonnull */
    syl.fields.ctop = ctpr - RID_CTPR1 + 1;
  syl.fields.ipd = 3;

  as->bundle.ss = syl.i;
  as->bundle.f1++;
  bundle_finalize(as, p);
}

static void bundle_call(ASMState *as, Reg ctpr, Reg pred,
                        int inverted, int wbs, MCode **p)
{
  bundle_ct(as, ctpr, pred, inverted, 0);
  alloc_resource(as, &as->bundle, RES_CS1);
  E2kC1f1 syl;
  syl.i = 0;
  syl.fields.opc = E2K_CALL;
  syl.fields.params = wbs;
  as->bundle.cs[1] = syl.i;
  as->bundle.f2++;
  bundle_finalize(as, p);
}

static void bundle_copf2(ASMState *as, uint32_t opc, Reg ctpr,
                         uintptr_t disp, MCode **p)
{
  alloc_resource(as, &as->bundle, RES_CS0);
  E2kC0f2 syl;
  syl.i = 0;
  syl.fields.disp = disp >> 3;
  syl.fields.opc = opc;
  if (ctpr) /* RID_CTPRX is nonnull */
    syl.fields.ctpr = ctpr - RID_CTPR1 + 1;

  as->bundle.cs[0] = syl.i;
  as->bundle.f1++;
  bundle_finalize(as, p);
}

static void bundle_disp(ASMState *as, Reg ctpr, MCode *target, MCode **p)
{
  as->bundle.cs0_target = target;
  bundle_copf2(as, E2K_DISP, ctpr, 0, p);
}

static void bundle_prepcall(ASMState *as, Reg ctpr,
                            ASMFunction target, MCode **p)
{
  ptrdiff_t delta = (MCode *) target - as->mcp;
  if (E2KF_S_OK(delta + 64, 29)) {
    bundle_disp(as, ctpr, (MCode *) target, p);
  } else { /* Target out of range; need indirect call. */
    bundle_alopf2_i(as, 0, E2K_MOVTD, (intptr_t)target, ctpr, p);
  }
}

static void bundle_ibranch(ASMState *as, MCode *target, Reg pred,
                           int inverted, MCode **p)
{
  as->bundle.cs0_target = target;
  bundle_ct(as, 0, pred, inverted, 0);
  bundle_copf2(as, E2K_IBRANCH, 0, 0, p);
}

static void bundle_setr(ASMState *as, MCode **p,
                        int wsz, int nfx, int dbl,
                        int rsz, int rbs, int rcur,
                        int psz, int rpsz, int op)
{
    alloc_resource(as, &as->bundle, RES_CS1);
    E2kSetBR params;
    params.i = 0;
    params.fields.rsz = rsz;
    params.fields.rbs = rbs;
    params.fields.rcur = rcur;
    params.fields.psz = psz;
    params.fields.setbn = op & E2K_SETBN ? 1 : 0;
    params.fields.setbp = op & E2K_SETBP ? 1 : 0;
    E2kC1f1 syl;
    syl.i = 0;
    syl.fields.opc = op & E2K_VFRPSZ ? E2K_SETR1 :
                     op & E2K_SETWD ? E2K_SETR0 : E2K_SETBR;
    syl.fields.params = params.i;
    as->bundle.cs[1] = syl.i;
    as->bundle.f2++;

    if (op & E2K_SETWD) {
        /* extended format */
        E2kSetR lts0;
        lts0.i = 0;
        lts0.fields.rpsz = rpsz;
        lts0.fields.wsz = wsz;
        lts0.fields.nfx = nfx;
        lts0.fields.dbl = dbl;
        bundle_lts(as, RES_LTS0, E2K_CONST32, lts0.i);
    }

    bundle_finalize(as, p);
}

#define bundle_window(as, wsz, nfx, dbl, rsz, rbs, rcur, p) \
  bundle_setr(as, p, wsz, nfx, dbl, rsz, rbs, rcur, 0, 0, E2K_SETWD|E2K_SETBN)

/* -- E2K instruction scheduler ------------------------------------------- */

#ifdef LUA_USE_ASSERT
static void sched_dbg_impl(ASMState *as, const char *file, int line,
                           const char *func, const char *fmt, ...)
{
  va_list argp;
  va_start(argp, fmt);
#if 0
  fprintf(stderr, "LuaJIT SCHED %s:%d: %s: ", file, line, func);
#else
  UNUSED(file);
  UNUSED(line);
  UNUSED(func);
#endif
  vfprintf(stderr, fmt, argp);
  fputc('\n', stderr);
  va_end(argp);
}

#define sched_dbg(as, ...) do { \
    if ((as)->sched.dbg) \
      sched_dbg_impl(as, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__); \
  } while (0)
#else
#define sched_dbg(as, ...) ((void) 0)
#endif

#define sched_latency(res, to) e2klat[(res)->out][to]

#define ALC_CLUSTER(alc_idx) ((alc_idx) > 2)

static void sched_latency_set(E2kOut from, E2kIn to, uint8_t value)
{
  e2klat[from][to] = value;
}

static void sched_reset(ASMState *as)
{
  uint8_t saved_enable, saved_dbg;
  E2kInsn *insn;

  if (!as->sched.enable)
    return;

  saved_enable = as->sched.enable;
  saved_dbg = as->sched.dbg;
  /* Clean fresh start. */
  memset(&as->sched, 0, sizeof(as->sched));
  as->sched.enable = saved_enable;
  as->sched.dbg = saved_dbg;

  /* Initialize free list of insns. */
  insn = as->sched.free = as->sched.insn;
  for (int i = 0; i < E2K_SCHED_WINDOW_SIZE; i++, insn++) {
    insn->prev = i > 0 ? insn - 1 : NULL;
    insn->next = i + 1 < E2K_SCHED_WINDOW_SIZE ? insn + 1 : NULL;
  }

  sched_dbg(as, "---- SCHED WINDOW SIZE %d", E2K_SCHED_WINDOW_SIZE);
}

static void sched_remove(ASMState *as, E2kInsn *insn)
{
  /* Remove insn from the list. */
  if (as->sched.head == insn)
    as->sched.head = insn->next;
  if (as->sched.tail == insn)
    as->sched.tail = insn->prev;
  if (insn->next)
    insn->next->prev = insn->prev;
  if (insn->prev)
    insn->prev->next = insn->next;

  /* Add insn to the free list. */
  insn->next = as->sched.free;
  if (insn->next)
    insn->next->prev = insn;
  insn->prev = NULL;
  as->sched.free = insn;
}

#ifdef LUA_USE_ASSERT
static const char *sched_reg_name(int rid)
{
#define E2K_RIDNAME(name) #name,
  static const char *const regname[] = {
    GPRDEF(E2K_RIDNAME)
    FPRDEF(E2K_RIDNAME)
    VRIDDEF(E2K_RIDNAME)
    NULL,
    NGPRDEF(E2K_RIDNAME)
  };

  switch(rid) {
  case RID_TMP1:
    return "TMP1";
  case RID_TMP2:
    return "TMP2";
  case RID_TMP3:
    return "TMP3";
  case RID_TMP4:
    return "TMP4";
  case RID_BASE:
    return "BASE";
  case RID_SP:
    return "SP";
  case RID_LPC:
    return "LPC";
  case RID_DISPATCH:
    return "DISPATCH";
  default:
    return regname[rid];
  }
}

static void sched_dump_cond(int pred, int inv)
{
  if (pred)
    fprintf(stderr, " ? %s%s", inv ? "~" : "", sched_reg_name(pred));
}

static void sched_dump_insn(ASMState *as, E2kInsn *insn)
{
  fprintf(stderr, "%-12s ", e2kop[insn->op].name);

  switch (insn->op) {
  case E2K_INSN_DISP:
  case E2K_INSN_PREPCALL:
    fprintf(stderr, "%s %p", sched_reg_name(insn->disp.ctpr),
            insn->disp.target);
    break;
  case E2K_INSN_CT:
    fprintf(stderr, "%s", sched_reg_name(insn->ct.ctpr));
    sched_dump_cond(insn->ct.pred, insn->ct.inverted);
    break;
  case E2K_INSN_IBRANCH:
    fprintf(stderr, "%p", insn->ibranch.target);
    sched_dump_cond(insn->ibranch.pred, insn->ibranch.inverted);
    break;
  case E2K_INSN_CALL:
    fprintf(stderr, "%s, wbs=%d", sched_reg_name(insn->call.ctpr),
            insn->call.wbs);
    sched_dump_cond(insn->call.pred, insn->call.inverted);
    break;
  default:
    lj_assertA(insn->op < E2K_ALOP_END, "unimplemented dump %s",
               e2kop[insn->op].name);

    if (insn->alop.flags & ALOP_SRC1_GPR)
      fprintf(stderr, "%s", sched_reg_name(insn->alop.src1));
    else
      fprintf(stderr, "%d", insn->alop.src1 & 31);

    if (insn->alop.flags & ALOP_SRC2_GPR)
      fprintf(stderr, ", %s", sched_reg_name(insn->alop.src2));
    else if (insn->alop.flags & ALOP_SRC2_LIT)
      fprintf(stderr, ", 0x%llx", (long long) insn->alop.literal);
    else
      fprintf(stderr, ", %d", insn->alop.src2 & 15);

    if (insn->alop.flags & ALOP_SRC3_GPR)
      fprintf(stderr, ", %s", sched_reg_name(insn->alop.src3));
    else
      fprintf(stderr, ", 0x%x", insn->alop.src3);

    if (insn->alop.flags & (ALOP_DST_REG | ALOP_SRC4_GPR))
      fprintf(stderr, ", %s", sched_reg_name(insn->alop.dst));
    else
      fprintf(stderr, ", %#x", insn->alop.dst);

    if (insn->alop.mrgc) {
      fprintf(stderr, ", %s%s",
              insn->alop.flags & ALOP_MRGC_INV ? "~" : "",
              sched_reg_name(insn->alop.mrgc));
    }

    if (insn->alop.cond) {
      sched_dump_cond(insn->alop.cond, insn->alop.flags & ALOP_COND_INV);
    }
    break;
  }
}
#endif

static void sched_use_gpr(ASMState *as, E2kInsn *insn, int rid, E2kIn in, int cluster)
{
  E2kUseGpr *use;
  int index;
  lj_assertA(RID_IS_GPR(rid), "expect gpr");
  use = &as->sched.use_gpr[rid - RID_MIN_GPR];
  use->used = 1;
  if (in == IN_R) {
    switch (e2kres[e2kop[insn->op].type].unit) {
      case UNIT_INT:
        index = USE_GPR_I_C0;
        break;
      case UNIT_FPU:
        index = USE_GPR_F_C0;
        break;
      default:
        lj_assertA(0, "unexpected unit");
        abort();
    }
  } else if (in == IN_S) {
    index = USE_GPR_S_C0;
  } else {
    lj_assertA(0, "unexpected gpr use in %d", in);
    abort();
  }
  use->src[index + cluster] = as->sched.cycle;
}

static void sched_use_pred(ASMState *as, E2kInsn *insn, int rid, E2kIn in)
{
  E2kUsePred *use;
  lj_assertA(RID_IS_PRED(rid), "expect pred");
  use = &as->sched.use_pred[rid - RID_MIN_PRED];
  switch (in) {
  case IN_RLP:
    use->rlp = as->sched.cycle;
    break;
  case IN_CT:
    use->ct = as->sched.cycle;
    break;
  default:
    lj_assertA(0, "unexpected pred use %d", in);
    abort();
  }
}

static void sched_use_ctpr(ASMState *as, E2kInsn *insn, int rid, E2kIn in)
{
  E2kUseCtpr *use;
  lj_assertA(RID_IS_CTPR(rid), "expect ctpr");
  use = &as->sched.use_ctpr[rid - RID_MIN_CTPR];
  switch (in) {
  case IN_CT:
    use->ct = as->sched.cycle;
    break;
  default:
    lj_assertA(0, "unexpected ctpr use in %d", in);
    abort();
  }
}

static void sched_use(ASMState *as, E2kInsn *insn, int cluster)
{
  switch (insn->op) {
  case E2K_INSN_DISP:
  case E2K_INSN_PREPCALL:
    break;
  case E2K_INSN_CT:
    sched_use_ctpr(as, insn, insn->ct.ctpr, IN_CT);
    if (insn->ct.pred)
      sched_use_pred(as, insn, insn->ct.pred, IN_CT);
    break;
  case E2K_INSN_IBRANCH:
    if (insn->ibranch.pred)
      sched_use_pred(as, insn, insn->ibranch.pred, IN_CT);
    break;
  case E2K_INSN_CALL:
    sched_use_ctpr(as, insn, insn->call.ctpr, IN_CT);
    if (insn->call.pred)
      sched_use_pred(as, insn, insn->call.pred, IN_CT);
    break;
  default:
    lj_assertA(insn->op < E2K_ALOP_END, "expect alop");
    if (insn->alop.flags & ALOP_SRC1_GPR)
      sched_use_gpr(as, insn, insn->alop.src1, IN_R, cluster);
    if (insn->alop.flags & ALOP_SRC2_GPR)
      sched_use_gpr(as, insn, insn->alop.src2, IN_R, cluster);
    if (insn->alop.flags & ALOP_SRC3_GPR)
      sched_use_gpr(as, insn, insn->alop.src3, IN_R, cluster);
    if (insn->alop.flags & ALOP_SRC4_GPR)
      sched_use_gpr(as, insn, insn->alop.src4, IN_S, cluster);
    if (insn->alop.cond)
      sched_use_pred(as, insn, insn->alop.cond, IN_RLP);
    if (insn->alop.mrgc)
      sched_use_pred(as, insn, insn->alop.mrgc, IN_RLP);
    break;
  }
}

static E2kCycle sched_max_cycle(E2kCycle t0, E2kCycle t1)
{
  return t0 < t1 ? t1 : t0;
}

static int sched_cycles_to_ready_gpr(ASMState *as, E2kInsn *insn, int rid,
                                     int cluster)
{
  static const E2kIn map[3] = { IN_I, IN_F, IN_S };
  const E2kReservation *res;
  E2kUseGpr *use;
  E2kCycle max = 0;
  lj_assertA(RID_IS_GPR(rid), "expect gpr");
  use = &as->sched.use_gpr[rid - RID_MIN_GPR];
  if (use->used) {
    res = e2kres + e2kop[insn->op].type;
    for (int i = 0; i < USE_GPR_LEN; i++) {
      if (use->src[i]) {
        E2kIn in = map[i / 2];
        if ((i & 1) != cluster)
          in = in == IN_S ? IN_S_C : IN_R;
        max = sched_max_cycle(max, use->src[i] + sched_latency(res, in));
      }
    }
  }
  /* TODO: sched: cluster hint (any, cluster0, cluster1) */
  return max - as->sched.cycle;
}

static int sched_cycles_to_ready_pred(ASMState *as, E2kInsn *insn, int rid)
{
  const E2kReservation *res;
  E2kUsePred *use;
  E2kCycle max = 0;
  lj_assertA(RID_IS_PRED(rid), "expect pred");
  use = &as->sched.use_pred[rid - RID_MIN_PRED];
  res = e2kres + e2kop[insn->op].type;
  if (use->rlp)
    max = use->rlp + sched_latency(res, IN_RLP);
  if (use->ct)
    max = sched_max_cycle(max, use->ct + sched_latency(res, IN_CT));
  return max - as->sched.cycle;
}

static int sched_cycles_to_ready_ctpr(ASMState *as, E2kInsn *insn, int rid)
{
  const E2kReservation *res;
  E2kUseCtpr *use;
  E2kCycle max = 0;

  lj_assertA(RID_IS_CTPR(rid), "expect ctpr");
  use = &as->sched.use_ctpr[rid - RID_MIN_CTPR];
  res = e2kres + e2kop[insn->op].type;
  if (use->ct)
    max = use->ct + sched_latency(res, IN_CT);
  return max - as->sched.cycle;
}

/* Returns the number of cycles relative to the current cycle when
 * an instruction can be issued without lock contention. */
static int sched_cycles_to_ready(ASMState *as, E2kInsn *insn, int cluster)
{
  switch (insn->op) {
  case E2K_INSN_DISP:
  case E2K_INSN_PREPCALL:
    return sched_cycles_to_ready_ctpr(as, insn, insn->disp.ctpr);
  case E2K_INSN_CT:
  case E2K_INSN_IBRANCH:
  case E2K_INSN_CALL:
    return 0;
  default:
    lj_assertA(insn->op < E2K_ALOP_END, "expect alop");
    if (insn->alop.flags & ALOP_DST_REG) {
      if (RID_IS_GPR(insn->alop.dst))
        return sched_cycles_to_ready_gpr(as, insn, insn->alop.dst, cluster);
      else if (RID_IS_PRED(insn->alop.dst))
        return sched_cycles_to_ready_pred(as, insn, insn->alop.dst);
      else if (RID_IS_CTPR(insn->alop.dst))
        return sched_cycles_to_ready_ctpr(as, insn, insn->alop.dst);
      else
        abort();
    }
    return 0;
  }
}

static void sched_use_reset(ASMState *as, int rid)
{
  if (RID_IS_GPR(rid))
    memset(&as->sched.use_gpr[rid - RID_MIN_GPR], 0,
           sizeof(*as->sched.use_gpr));
  else if (RID_IS_PRED(rid))
    memset(&as->sched.use_pred[rid - RID_MIN_PRED], 0,
           sizeof(*as->sched.use_pred));
  else if (RID_IS_CTPR(rid))
    memset(&as->sched.use_ctpr[rid - RID_MIN_CTPR], 0,
           sizeof(*as->sched.use_ctpr));
  else
    abort();
}

static void sched_issued(ASMState *as, E2kInsn *insn)
{
  switch (insn->op) {
  case E2K_INSN_DISP:
  case E2K_INSN_PREPCALL:
    sched_use_reset(as, insn->disp.ctpr);
    break;
  case E2K_INSN_CT:
  case E2K_INSN_IBRANCH:
  case E2K_INSN_CALL:
    break;
  default:
    lj_assertA(insn->op < E2K_ALOP_END, "expect alop");
    if (insn->alop.flags & ALOP_DST_REG)
      sched_use_reset(as, insn->alop.dst);
    break;
  }
}

/* Adds an instruction to the bundle.  */
static int sched_bundle_insn(ASMState *as, const E2kInsn *insn)
{
  int als_idx = 0, src1, src2, src3, dst;

  switch (insn->op) {
  case E2K_INSN_DISP:
    bundle_disp(as, insn->disp.ctpr, insn->disp.target, 0);
    break;
  case E2K_INSN_PREPCALL:
    bundle_prepcall(as, insn->disp.ctpr, insn->disp.target, 0);
    break;
  case E2K_INSN_CT:
    bundle_ct(as, insn->ct.ctpr, insn->ct.pred, insn->ct.inverted, 0);
    break;
  case E2K_INSN_IBRANCH:
    bundle_ibranch(as, insn->ibranch.target, insn->ibranch.pred,
                   insn->ibranch.inverted, 0);
    break;
  case E2K_INSN_CALL:
    bundle_call(as, insn->call.ctpr, insn->call.pred, insn->call.inverted,
                insn->call.wbs, 0);
    break;
  default:
    lj_assertA(insn->op < E2K_ALOP_END, "expect alop");

    if (insn->alop.flags & ALOP_SRC1_GPR)
      src1 = RSRC1(insn->alop.src1);
    else
      src1 = insn->alop.src1;

    if (insn->alop.flags & ALOP_SRC2_GPR)
      src2 = RSRC2(insn->alop.src2);
    else if (insn->alop.flags & ALOP_SRC2_LIT)
      src2 = ISRC2(insn->alop.literal);
    else
      src2 = insn->alop.src2;

    if (insn->alop.flags & ALOP_SRC3_GPR)
      src3 = RSRC3(insn->alop.src3);
    else
      src3 = insn->alop.src3;

    if (insn->alop.flags & ALOP_DST_REG) {
      if (RID_IS_GPR(insn->alop.dst))
        dst = RDST(insn->alop.dst);
      else if (RID_IS_PRED(insn->alop.dst))
        dst = PDST(insn->alop.dst);
      else if (RID_IS_CTPR(insn->alop.dst))
        dst = RDST(insn->alop.dst);
      else
       abort();
    } else if (insn->alop.flags & ALOP_SRC4_GPR)
      dst = RSRC4(insn->alop.src4);
    else
      dst = insn->alop.dst;

    als_idx = bundle_alop(as, 0, (insn->alop.flags & ALOP_SPEC) != 0,
                          insn->op, src1, src2, src3, dst);

    if (insn->alop.cond) {
      bundle_rlp(as, als_idx, insn->alop.cond,
                 insn->alop.flags & ALOP_COND_INV ? 1 : 0, 0);
    }

    if (insn->alop.mrgc) {
      bundle_mrgc(as, als_idx, insn->alop.mrgc,
                  insn->alop.flags & ALOP_MRGC_INV ? 1 : 0, 0);
    }
    break;
  }

  return als_idx;
}

/* Emit a single instruction from the sched queue.  */
static int sched_emit_single_insn(ASMState *as)
{
  int als_idx, nops;
  E2kInsn *insn = as->sched.tail;

  /* Nothing to emit. */
  if (!insn)
    return 0;

  als_idx = sched_bundle_insn(as, insn);
  if (as->sched.enable & E2K_SCHED_NOPS) {
    nops = sched_cycles_to_ready(as, insn, ALC_CLUSTER(als_idx));
    if (nops < 0)
      nops = 0;
    else
      as->sched.cycle += nops;
    bundle_nop(as, nops);
  }
  bundle_finalize(as, &as->mcp);

#ifdef LUA_USE_ASSERT
  if (as->sched.dbg) {
    fprintf(stderr, "%08lx: ", (uintptr_t) as->mcp);
    sched_dump_insn(as, insn);
    fputc('\n', stderr);
    if (nops)
      fprintf(stderr, "%9s NOP %d\n", "", nops);
  }
#endif

  if (as->sched.enable & E2K_SCHED_NOPS) {
    sched_issued(as, insn);
    sched_use(as, insn, ALC_CLUSTER(als_idx));
    as->sched.cycle += 1;
  }

  sched_remove(as, insn);

  return 1;
}

static int sched_emit(ASMState *as)
{
  return sched_emit_single_insn(as);
}

static MCode *emit_serializef(ASMState *as, const char *reason)
{
  if (as->sched.enable && as->sched.tail) {
    if (reason)
      sched_dbg(as, "---- SERIALIZE START (%s)", reason);
    else
      sched_dbg(as, "---- SERIALIZE START");

    /* Write all pending insns to memory. */
    while (sched_emit(as));

    as->sched.overflow = 0;
  }
  return as->mcp;
}

#define emit_serialize(as) emit_serializef(as, __FUNCTION__)

static E2kInsn *sched_alloc(ASMState *as, int op)
{
  E2kInsn *insn;

  /* Emit one bundle if these is no free insn. */
  if (!as->sched.free) {
    if (!as->sched.overflow) {
      sched_dbg(as, "---- OVERFLOW START");
      as->sched.overflow = 1;
    }
    sched_emit(as);
  }

  /* Take insn from free list. */
  insn = as->sched.free;
  lj_assertA(insn, "expect free insn");
  if (insn->next)
    as->sched.free->prev = NULL;
  as->sched.free = insn->next;

  /* Initialize insn. */
  memset(insn, 0, sizeof(*insn));

  if (as->sched.head) {
    /* This insn is a new head. */
    insn->next = as->sched.head;
    insn->next->prev = insn;
  } else {
    /* List is empty. */
    as->sched.tail = insn;
  }
  as->sched.head = insn;
  insn->op = op;
  return insn;
}

/* -- Emit basic instructions --------------------------------------------- */

static E2kInsn *emit_alop(ASMState *as, int spec, E2kAlopFlags flags, uint32_t op,
                          int src1, int src2, int src3, int dst)
{
  E2kInsn *insn = sched_alloc(as, op);
  if (spec) flags |= ALOP_SPEC;
  insn->alop.flags = flags;
  insn->alop.src1 = src1;
  insn->alop.src2 = src2;
  insn->alop.src3 = src3;
  insn->alop.dst = dst;
  return insn;
}

static E2kInsn *emit_alop_i(ASMState *as, int spec, E2kAlopFlags flags, uint32_t op,
                            int src1, int64_t src2, int src3, int dst)
{
  E2kInsn *insn;
  if (get_const_type(src2) != E2K_CONST4) {
    flags |= ALOP_SRC2_LIT;
    insn = emit_alop(as, spec, flags, op, src1, 0, src3, dst);
    insn->alop.literal = src2;
  } else {
    insn = emit_alop(as, spec, flags, op, src1, src2 | 0xc0, src3, dst);
  }
  return insn;
}

static void emit_alopf7_rr(ASMState *as, int spec, uint32_t op, int src1,
                           int src2, int pdst)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_SRC2_GPR | ALOP_DST_REG;
    emit_alop(as, spec, flags, op, src1, src2, 0, pdst);
  } else
    bundle_alopf7_rr(as, spec, op, src1, src2, pdst, &as->mcp);
}

static void emit_alopf7_ri(ASMState *as, int spec, uint32_t op, int src1,
                           int64_t src2, int pdst)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_DST_REG;
    emit_alop_i(as, spec, flags, op, src1, src2, 0, pdst);
  } else
    bundle_alopf7_ri(as, spec, op, src1, src2, pdst, &as->mcp);
}

static void emit_alopf3_ri(ASMState *as, int spec, uint32_t op, int src1,
                           int64_t src2, int src4)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_SRC4_GPR;
    emit_alop_i(as, spec, flags, op, src1, src2, 0, src4);
  } else
    bundle_alopf3_ri(as, spec, op, src1, src2, src4, &as->mcp);
}

#define emit_alopf12_i emit_alopf2_i
#define emit_alopf12_r emit_alopf2_r

static void emit_alopf2_r_rlp(ASMState *as, int spec, uint32_t op, int src2,
                              int dst, int pred, int inv)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC2_GPR | ALOP_DST_REG;
    if (inv) flags |= ALOP_COND_INV;
    emit_alop(as, spec, flags, op, 0, src2, 0, dst)
      ->alop.cond = pred;
  } else {
    int als_idx = bundle_alopf2_r(as, spec, op, src2, dst, 0);
    bundle_rlp(as, als_idx, pred, inv, &as->mcp);
  }
}

static void emit_alopf2_i(ASMState *as, int spec, uint32_t op, int64_t src2,
                          int dst)
{
  if (as->sched.enable) {
    int flags = ALOP_DST_REG;
    emit_alop_i(as, spec, flags, op, 0, src2, 0, dst);
  } else
    bundle_alopf2_i(as, spec, op, src2, dst, &as->mcp);
}

static void emit_alopf2_r(ASMState *as, int spec, uint32_t op, int src2,
                          int dst)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC2_GPR | ALOP_DST_REG;
    emit_alop(as, spec, flags, op, 0, src2, 0, dst);
  } else
    bundle_alopf2_r(as, spec, op, src2, dst, &as->mcp);
}

static void emit_alopf21_rrr(ASMState *as, int spec, uint32_t op, int src1,
                             int src2, int src3, int dst)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_SRC2_GPR | ALOP_SRC3_GPR | ALOP_DST_REG;
    emit_alop(as, spec, flags, op, src1, src2, src3, dst);
  } else
    bundle_alopf21_rrr(as, spec, op, src1, src2, src3, dst, &as->mcp);
}

#define emit_alopf11_rr emit_alopf1_rr
#define emit_alopf11_ir emit_alopf1_ir
#define emit_alopf11_ri emit_alopf1_ri
#define emit_alopf11_ii emit_alopf1_ii

static void emit_alopf1_rr(ASMState *as, int spec, uint32_t op, int src1,
                           int src2, int dst)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_SRC2_GPR | ALOP_DST_REG;
    emit_alop(as, spec, flags, op, src1, src2, 0, dst);
  } else
    bundle_alopf1_rr(as, spec, op, src1, src2, dst, &as->mcp);
}

static void emit_alopf1_rr_mrgc(ASMState *as, int spec, uint32_t op, int src1,
                                int src2, int dst, int pred, int inv)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_SRC2_GPR | ALOP_DST_REG;
    if (inv) flags |= ALOP_MRGC_INV;
    emit_alop(as, spec, flags, op, src1, src2, 0, dst)
      ->alop.mrgc = pred;
  } else {
    int als_idx = bundle_alopf1_rr(as, spec, op, src1, src2, dst, 0);
    bundle_mrgc(as, als_idx, pred, inv, &as->mcp);
  }
}

static void emit_alopf1_ri_rlp(ASMState *as, int spec, uint32_t op, int src1,
                               int64_t src2, int dst, int pred, int inv)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_DST_REG;
    if (inv) flags |= ALOP_COND_INV;
    emit_alop_i(as, spec, flags, op, src1, src2, 0, dst)
      ->alop.cond = pred;
  } else {
    int als_idx = bundle_alopf1_ri(as, spec, op, src1, src2, dst, 0);
    bundle_rlp(as, als_idx, pred, inv, &as->mcp);
  }
}

static void emit_alopf1_ir(ASMState *as, int spec, uint32_t op, uint8_t src1,
                           int src2, int dst)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC2_GPR | ALOP_DST_REG;
    emit_alop(as, spec, flags, op, ISRC1(src1), src2, 0, dst);
  } else
    bundle_alopf1_ir(as, spec, op, src1, src2, dst, &as->mcp);
}

static void emit_alopf1_ri(ASMState *as, int spec, uint32_t op, int src1,
                           int64_t src2, int dst)
{
  if (as->sched.enable) {
    int flags = ALOP_SRC1_GPR | ALOP_DST_REG;
    emit_alop_i(as, spec, flags, op, src1, src2, 0, dst);
  } else
    bundle_alopf1_ri(as, spec, op, src1, src2, dst, &as->mcp);
}

static void emit_alopf1_ii(ASMState *as, int spec, uint32_t op, uint8_t src1,
                           int64_t src2, int dst)
{
  if (as->sched.enable) {
    int flags = ALOP_DST_REG;
    emit_alop_i(as, spec, flags, op, ISRC1(src1), src2, 0, dst);
  } else
    bundle_alopf1_ii(as, spec, op, src1, src2, dst, &as->mcp);
}

static void emit_snapno(ASMState *as, uint64_t snapno)
{
  emit_alopf1_ii(as, 0, E2K_ADDD,
                 snapno < 32 ? snapno : 31,
                 snapno < 32 ? 0 : snapno - 31, RID_TMP);
}

/* ord uimm5, uimm4, empty.hi */
#define E2K_CONTROL_OP(src1, src2) \
  (0x50000df \
   | ((uint32_t) (((src1) & 31) | 0xc0) << 16) \
   | ((uint32_t) (((src2) & 15) | 0xc0) << 8))

static void emit_control(ASMState *as, uint8_t src1, uint8_t src2)
{
  lj_assertA(src1 < 32, "invalid src1");
  lj_assertA(src2 < 16, "invalid src2");
  if (as->sched.enable)
    emit_alop(as, 0, 0, E2K_ORD, ISRC1(src1), ISRC2(src2), 0, 0xdf);
  else
    bundle_alop(as, &as->mcp, 0, E2K_ORD, ISRC1(src1), ISRC2(src2), 0, 0xdf);
}

static void emit_disp(ASMState *as, Reg ctpr, MCode *target)
{
  if (as->sched.enable) {
    E2kInsn *insn = sched_alloc(as, E2K_INSN_DISP);
    insn->disp.ctpr = ctpr;
    insn->disp.target = target;
  } else
    bundle_disp(as, ctpr, target, &as->mcp);
}

static void emit_prepcall(ASMState *as, Reg ctpr, ASMFunction target)
{
  if (as->sched.enable) {
    E2kInsn *insn = sched_alloc(as, E2K_INSN_PREPCALL);
    insn->disp.ctpr = ctpr;
    insn->disp.target = target;
  } else
    bundle_prepcall(as, ctpr, target, &as->mcp);
}

static void emit_ct(ASMState *as, Reg ctpr, Reg pred, int inverted)
{
  if (as->sched.enable) {
    E2kInsn *insn = sched_alloc(as, E2K_INSN_CT);
    insn->ct.ctpr = ctpr;
    insn->ct.pred = pred;
    insn->ct.inverted = inverted;
  } else
    bundle_ct(as, ctpr, pred, inverted, &as->mcp);
}

static void emit_ibranch(ASMState *as, MCode *target, Reg pred,
                         int inverted)
{
  if (as->sched.enable) {
    E2kInsn *insn = sched_alloc(as, E2K_INSN_IBRANCH);
    insn->ibranch.target = target;
    insn->ibranch.pred = pred;
    insn->ibranch.inverted = inverted;
  } else
    bundle_ibranch(as, target, pred, inverted, &as->mcp);
}

static void emit_call(ASMState *as, Reg ctpr, Reg pred, int inverted, int wbs)
{
  if (as->sched.enable) {
    E2kInsn *insn = sched_alloc(as, E2K_INSN_CALL);
    insn->call.ctpr = ctpr;
    insn->call.pred = pred;
    insn->call.inverted = inverted;
    insn->call.wbs = wbs;
  } else
    bundle_call(as, ctpr, pred, inverted, wbs, &as->mcp);
}

/* -- Emit loads/stores --------------------------------------------------- */

#define dispofs(as, k) \
  ((intptr_t)((uintptr_t)(k) - (uintptr_t)J2GG(as->J)->dispatch))

/* Prefer rematerialization of BASE/L from global_State over spills. */
#define emit_canremat(ref)  ((ref) <= REF_BASE)

/* Load a 64 bit constant into a GPR. */
static void emit_loadu64(ASMState *as, Reg r, uint64_t u64)
{
  emit_alopf1_ii(as, 0, E2K_ADDD, 0, u64, r);
}

static void emit_loadk64(ASMState *as, Reg r, IRIns *ir)
{
  const uint64_t *k = &ir_k64(ir)->u64;
  emit_loadu64(as, r, *k);
}

static void emit_ldd(ASMState *as, Reg dest, void *addr)
{
  emit_alopf1_ri(as, 0, E2K_LDD, RID_DISPATCH,
                 (intptr_t)dispofs(as, addr), dest);
}

static void emit_std(ASMState *as, Reg src, void *addr)
{
  emit_alopf3_ri(as, 0, E2K_STD, RID_DISPATCH, (intptr_t)dispofs(as, addr),
                 src);
}

/* Load a constant address into a GPR. */
#define emit_loada(as, r, addr)   emit_loadu64(as, (r), u64ptr((addr)))
/* Load a 32 bit constant into a GPR. */
#define emit_loadi(as, r, i)      emit_loadu64(as, (r), (intptr_t)i)

/* Get/set global_State fields. */
#define emit_getgl(as, r, field) emit_ldd(as, r, (void *)&J2G(as->J)->field)
#define emit_setgl(as, r, field) emit_std(as, r, (void *)&J2G(as->J)->field)

/* Trace number is determined from per-trace exit stubs. */
#define emit_setvmstate(as, i) UNUSED(i)

/* -- Emit control-flow instructions -------------------------------------- */

/* Label for internal jumps. */
typedef MCode *MCLabel;

/* Return label pointing to current PC. */
static MCLabel emit_label(ASMState *as)
{
  return emit_serialize(as);
}

static void emit_jmp(ASMState *as, MCode *target)
{
  emit_ibranch(as, target, 0, 0);
}

/* -- Emit generic operations --------------------------------------------- */

/* argument extension */
static void emit_ext(ASMState *as, Reg dst, Reg src, int ext) {
  emit_alopf1_ir(as, 0, E2K_SXT, ext, src, dst);
}

/* Generic move between two regs. */
static void emit_movrr(ASMState *as, IRIns *ir, Reg dst, Reg src)
{
  UNUSED(ir);
  emit_alopf1_ri(as, 0, E2K_ADDD, src, 0, dst);
}

/* Generic load of register with base and (small) offset address. */
static void emit_loadofs(ASMState *as, IRIns *ir, Reg r, Reg base, int32_t ofs)
{
  emit_alopf1_ri(as, 0, irt_is64(ir->t) ? E2K_LDD : E2K_LDW, base, ofs, r);
}

/* Generic store of register with base and (small) offset address. */
static void emit_storeofs(ASMState *as, IRIns *ir, Reg r, Reg base, int32_t ofs)
{
  emit_alopf3_ri(as, 0, irt_is64(ir->t) ? E2K_STD : E2K_STW,
                 base, ofs, r);
}

/* Add offset to pointer. */
static void emit_addptr(ASMState *as, Reg r, int32_t ofs)
{
  if (ofs) {
    emit_alopf1_ri(as, 0, E2K_ADDD, r, ofs, r);
  }
}

/* Get additional stack space. */
static void emit_spsub(ASMState *as, int32_t ofs)
{
  if (ofs) {
    emit_alopf12_i(as, 0, E2K_GETSP, -ofs, RID_SP);
  }
}
