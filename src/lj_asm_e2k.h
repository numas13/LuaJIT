/*
** E2K IR assembler (SSA IR -> machine code).
** Copyright (C) 2005-2025 Mike Pall. See Copyright Notice in luajit.h
*/

/* Environment variable names for e2k scheduler. */
#define LJ_E2K_SCHED_DISABLE  "LJ_E2K_SCHED_DISABLE"
#define LJ_E2K_SCHED_DISABLE_NOPS  "LJ_E2K_SCHED_DISABLE_NOPS"
#define LJ_E2K_SCHED_DEBUG    "LJ_E2K_SCHED_DEBUG"

/* -- Register allocator extensions --------------------------------------- */

static Reg ra_hintalloc(ASMState *as, IRRef ref, Reg hint, RegSet allow)
{
  Reg r = IR(ref)->r;
  if (ra_noreg(r)) {
    if (!ra_hashint(r) && !iscrossref(as, ref))
      ra_sethint(IR(ref)->r, hint);  /* Propagate register hint. */
    r = ra_allocref(as, ref, allow);
  }
  ra_noweak(as, r);
  return r;
}

/* -- Guard handling ------------------------------------------------------ */

/* Setup exit stub after the end of each trace. */
static void asm_exitstub_setup(ASMState *as)
{
  /* Register allocation is not started yet */
  MCode *mxp = as->mctop;
  bundle_ct(as, RID_CTPR1, 0, 0, 0);
  bundle_snapno(as, as->T->traceno, 0);
  bundle_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, RID_TMP, RID_TMP2, &mxp);
  bundle_nop(as, 4);
  bundle_disp(as, RID_CTPR1, (MCode *) lj_vm_exit_handler, &mxp);

  as->mctop = mxp;
}

/* Keep this in-sync with exitstub_trace_addr(). */
#define asm_exitstub_addr(as) ((as)->mctop)

/* Emit conditional branch to exit for guard */
static void asm_guard(ASMState *as, Reg pred, int inverted)
{
  MCode *target = asm_exitstub_addr(as);
  MCode *p = as->mcp;
  /* Do not emit inverted loop guard if sched queue is not empty. */
  if (!as->sched.tail && LJ_UNLIKELY(p == as->invmcp)) {
    as->invmcp = NULL;
    as->loopinv = 1;
    as->mcp = p + 4;
    inverted = inverted ? 0 : 1;
    target = p; /* Patch target later in asm_loop_fixup. */
    bundle_len(as, 8); /* required for asm_loop_fixup */
    bundle_ibranch(as, target, pred, inverted, 0);
    bundle_snapno(as, as->snapno, &as->mcp);
  } else {
    emit_ibranch(as, target, pred, inverted);
    emit_snapno(as, as->snapno);
  }
}

/* Marker to prevent patching the GC check exit. */
/* ord 0x0, 0x0, empty.hi */
#define E2K_NOPATCH_GC_CHECK_SRC1 0
#define E2K_NOPATCH_GC_CHECK E2K_CONTROL_OP(E2K_NOPATCH_GC_CHECK_SRC1, 0)

/* Emit conditional branch to exit for gc check guard */
static void asm_guard_gc(ASMState *as, Reg pred, int inverted)
{
  MCode *target = asm_exitstub_addr(as);
  lj_assertA(as->mcp != as->invmcp, "unexpected GC check");
  emit_ibranch(as, target, pred, inverted);
  emit_snapno(as, as->snapno);
  /* Keep in sync with E2K_NOPATCH_GC_CHECK */
  emit_control(as, E2K_NOPATCH_GC_CHECK_SRC1, 0);
}

/* -- Operand fusion ------------------------------------------------------ */

/* Limit linear search to this distance. Avoids O(n^2) behavior. */
#define CONFLICT_SEARCH_LIM 31

/* Check if there's no conflicting instruction between curins and ref. */
static int noconflict(ASMState *as, IRRef ref, IROp conflict)
{
  IRIns *ir = as->ir;
  IRRef i = as->curins;
  if (i > ref + CONFLICT_SEARCH_LIM)
    return 0; /* Give up, ref is too far away. */
  while (--i > ref)
    if (ir[i].o == conflict)
      return 0; /* Conflict found. */
  return 1;  /* Ok, no conflict. */
}

/* Fuse the array base of colocated arrays. */
static intptr_t asm_fuseabase(ASMState *as, IRRef ref)
{
  IRIns *ir = IR(ref);
  if (ir->o == IR_TNEW && ir->op1 <= LJ_MAX_COLOSIZE &&
      !neverfuse(as) && noconflict(as, ref, IR_NEWREF))
    return (intptr_t)sizeof(GCtab);
  return 0;
}

/* Fuse array/hash/upvalue reference into register+offset operand. */
static Reg asm_fuseahuref(ASMState *as, IRRef ref, intptr_t *ofsp, RegSet allow)
{
  IRIns *ir = IR(ref);
  if (ra_noreg(ir->r)) {
    if (ir->o == IR_AREF) {
      if (mayfuse(as, ref)) {
        if (irref_isk(ir->op2)) {
          IRRef tab = IR(ir->op1)->op1;
          intptr_t ofs = asm_fuseabase(as, tab);
          IRRef refa = ofs ? tab : ir->op1;
          ofs += 8*IR(ir->op2)->i;
          *ofsp = ofs;
          return ra_alloc1(as, refa, allow);
        }
      }
    } else if (ir->o == IR_HREFK) {
      if (mayfuse(as, ref)) {
        *ofsp = (intptr_t)(IR(ir->op2)->op2 * sizeof(Node));
        return ra_alloc1(as, ir->op1, allow);
      }
    } else if (ir->o == IR_UREFC) {
      if (irref_isk(ir->op1)) {
        GCfunc *fn = ir_kfunc(IR(ir->op1));
        GCupval *uv = &gcref(fn->l.uvptr[(ir->op2 >> 8)])->uv;
        *ofsp = dispofs(as, &uv->tv);
        return RID_DISPATCH;
      }
    } else if (ir->o == IR_TMPREF) {
      *ofsp = dispofs(as, &J2G(as->J)->tmptv);
      return RID_DISPATCH;
    }
  }
  *ofsp = 0;
  return ra_alloc1(as, ref, allow);
}

/* Fuse XLOAD/XSTORE reference into load/store operand. */
static IRRef asm_fusexref(ASMState *as, IRRef ref, intptr_t *ofs)
{
  IRIns *ir = IR(ref);
  if (ra_noreg(ir->r) && canfuse(as, ir)) {
    if ((ir->o == IR_ADD) && (irref_isk(ir->op2))) {
      *ofs = *ofs + get_kval(as, ir->op2);
      return ir->op1;
    } else if ((ir->o == IR_STRREF) && irref_isk(ir->op2)) {
      *ofs = (intptr_t)sizeof(GCstr) + get_kval(as, ir->op2);
      return ir->op1;
    } else if ((ir->o == IR_STRREF) && irref_isk(ir->op1)) {
      *ofs = (intptr_t)sizeof(GCstr) + get_kval(as, ir->op1);
      return ir->op2;
    }
  }
  return ref;
}

/* -- Calls --------------------------------------------------------------- */

/* Generate a call to a C function. */
static void asm_gencall(ASMState *as, const CCallInfo *ci, IRRef *args)
{
  uint32_t n, nargs = CCI_XNARGS(ci);
  uint32_t is_vararg = ci->flags & CCI_VARARG;
  int32_t ofs = is_vararg ? 0 : STACKARG_OFS;
  Reg gpr = RID_NONE;
  if (ci->func)
    emit_call(as, RID_CTPR1, 0, 0, JIT_WBS);
  for (gpr = REGARG_FIRSTGPR; gpr <= REGARG_LASTGPR; gpr++)
    as->cost[gpr] = REGCOST(~0u, ASMREF_L);
  gpr = REGARG_FIRSTGPR;

  for (n = 0; n < nargs; n++) { /* Setup args. */
    IRRef ref = args[n];
    if (ref) {
      IRIns *ir = IR(ref);
      if (gpr <= REGARG_LASTGPR) {
        lj_assertA(rset_test(as->freeset, gpr),
                   "arg%d not free", gpr-REGARG_FIRSTGPR); /*  Already evicted. */
        ra_leftov(as, gpr, ref);
        if (is_vararg) {
          emit_spstore(as, ir, gpr, ofs);
          ofs += 8;
        }
        gpr++;
      } else {
        Reg r = ra_alloc1(as, ref, RSET_GPR);
        emit_spstore(as, ir, r, ofs);
        ofs += 8;
      }
    } else {
      if (gpr <= REGARG_LASTGPR) {
        if (is_vararg) ofs += 8;
        gpr++;
      } else {
        ofs += 8;
      }
    }
    checkmclim(as);
  }
  if (ci->func) {
    emit_prepcall(as, RID_CTPR1, ci->func);
  }
}

/* Setup result reg/sp for call. Evict scratch regs. */
static void asm_setupresult(ASMState *as, IRIns *ir, const CCallInfo *ci)
{
  RegSet drop = RSET_SCRATCH;
  int hiop = ((ir+1)->o == IR_HIOP && !irt_isnil((ir+1)->t));

  if (ra_hasreg(ir->r))
    rset_clear(drop, ir->r); /* Dest reg handled below. */
  if (hiop && ra_hasreg((ir+1)->r))
    rset_clear(drop, (ir+1)->r);  /* Dest reg handled below. */
  ra_evictset(as, drop);  /* Evictions must be performed first. */
  if (ra_used(ir)) {
    lj_assertA(!irt_ispri(ir->t), "PRI dest");
    if (hiop) {
      ra_destpair(as, ir);
    } else {
      ra_destreg(as, ir, RID_RET);
    }
  }
}

static void asm_callx(ASMState *as, IRIns *ir)
{
  IRRef args[CCI_NARGS_MAX*2];
  CCallInfo ci;
  IRRef func;
  IRIns *irf;
  ci.flags = asm_callx_flags(as, ir);
  asm_collectargs(as, ir, &ci, args);
  asm_setupresult(as, ir, &ci);
  func = ir->op2; irf = IR(func);
  if (irf->o == IR_CARG) { func = irf->op1; irf = IR(func); }
  if (irref_isk(func)) {  /* Call to constant address. */
    ci.func = (ASMFunction)(void *)get_kval(as, func);
  } else {
    emit_call(as, RID_CTPR1, 0, 0, JIT_WBS);
    ci.func = (ASMFunction)(void *)0;
  }
  asm_gencall(as, &ci, args);
  if (!ci.func) {
    Reg r = ra_alloc1(as, func, RSET_GPR & ~RSET_SCRATCH);
    emit_alopf2_r(as, 0, E2K_MOVTD, r, RID_CTPR1);
  }
}

/* -- Returns ------------------------------------------------------------- */

/* Return to lower frame. Guard that it goes to the right spot. */
static void asm_retf(ASMState *as, IRIns *ir)
{
  Reg base = ra_alloc1(as, REF_BASE, RSET_GPR);
  void *pc = ir_kptr(IR(ir->op2));
  int32_t delta = 1+LJ_FR2+bc_a(*((const BCIns *)pc - 1));
  as->topslot -= (BCReg)delta;
  if ((int32_t)as->topslot < 0) as->topslot = 0;
  irt_setmark(IR(REF_BASE)->t);  /* Children must not coalesce with BASE reg. */
  emit_setgl(as, base, jit_base);
  emit_addptr(as, base, -8*delta);
  Reg tmp = ra_scratch(as, rset_exclude(RSET_GPR, base));
  asm_guard(as, RID_PRED0, 1);
  emit_alopf7_ri(as, 0, E2K_CMPEDB, tmp, (intptr_t)pc, RID_PRED0);
  emit_alopf1_ri(as, 0, E2K_LDD, base, LJ_FR2 ? -8 : -4, tmp);
}

/* -- Buffer operations --------------------------------------------------- */

#if LJ_HASBUFFER
static void asm_bufhdr_write(ASMState *as, Reg sb)
{
  RegSet allow = RSET_GPR;
  Reg tmp1 = ra_scratch(as, rset_clear(allow, sb));
  Reg tmp2 = ra_scratch(as, rset_clear(allow, tmp1));
  IRIns irgc;
  irgc.ot = IRT(0, IRT_PGC);  /* GC type. */
  emit_storeofs(as, &irgc, tmp2, sb, offsetof(SBuf, L));
  emit_alopf1_rr(as, 0, E2K_ORD, tmp2, tmp1, tmp2);
  emit_alopf1_ri(as, 0, E2K_ANDD, tmp1, SBUF_MASK_FLAG, tmp1);
  emit_getgl(as, tmp2, cur_L);
  emit_loadofs(as, &irgc, tmp1, sb, offsetof(SBuf, L));
}
#endif

/* -- Type conversions ---------------------------------------------------- */

static void asm_tointg(ASMState *as, IRIns *ir, Reg left)
{
  Reg tmp = ra_scratch(as, rset_exclude(RSET_GPR, left));
  Reg dest = ra_dest(as, ir, RSET_GPR);
  asm_guard(as, RID_PRED0, 1);
  emit_alopf7_rr(as, 0, E2K_FCMPEQDB, left, tmp, RID_PRED0);
  emit_alopf2_r(as, 0, E2K_ISTOFD, dest, tmp);
  emit_alopf2_r(as, 0, E2K_FDTOISTR, left, dest);
}

static void asm_tobit(ASMState *as, IRIns *ir)
{
  Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
  Reg dest = ra_dest(as, ir, RSET_GPR);
  emit_alopf1_rr(as, 0, E2K_FADDD, left, right, dest);
}

static void asm_conv(ASMState *as, IRIns *ir)
{
  IRType st = (IRType)(ir->op2 & IRCONV_SRCMASK);
  Reg dest = RID_NONE, left = RID_NONE;
  int stfp = (st == IRT_NUM || st == IRT_FLOAT);
  int st64 = (st == IRT_I64 || st == IRT_U64 || st == IRT_P64);
  uint32_t op = 0;
  lj_assertA(irt_type(ir->t) != st, "inconsistent types for CONV");
  if (irt_isfp(ir->t)) {
    dest = ra_dest(as, ir, RSET_GPR);
    if (stfp) { /* FP to FP conversion */
      left = ra_alloc1(as, ir->op1, RSET_GPR);
      op = (st == IRT_NUM) ? E2K_FDTOFS : E2K_FSTOFD;
      emit_alopf2_r(as, 0, op, left, dest);
    } else if (st == IRT_U32) { /* U32 to FP conversion */
      left = ra_alloc1(as, ir->op1, RSET_GPR);
      op = irt_isnum(ir->t) ? E2K_IDTOFD : E2K_IDTOFS;
      emit_alopf2_r(as, 0, op, dest, dest);
      emit_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, left, dest);
    } else if (st == IRT_U64) { /* U64 to FP conversion */
      RegSet allow = RSET_GPR;
      left = ra_alloc1(as, ir->op1, rset_clear(allow, dest));
      Reg tmp1 = ra_scratch(as, rset_clear(allow, left));
      Reg tmp2 = ra_scratch(as, rset_clear(allow, tmp1));
      op = irt_isnum(ir->t) ? E2K_IDTOFD : E2K_IDTOFS;
      emit_alopf1_rr_mrgc(as, 1, irt_isnum(ir->t) ? E2K_MERGED : E2K_MERGES,
                          tmp2, tmp1, dest, RID_PRED0, 0);
      emit_alopf1_rr(as, 1, irt_isnum(ir->t) ? E2K_FADDD : E2K_FADDS,
                     tmp1, tmp1, tmp1);
      emit_alopf2_r(as, 1, op, left, tmp2);
      emit_alopf7_ri(as, 0, E2K_CMPLDB, left, 0x0, RID_PRED0);
      emit_alopf2_r(as, 1, op, tmp1, tmp1);
      emit_alopf1_rr(as, 1, E2K_ORD, tmp2, tmp1, tmp1);
      emit_alopf1_ri(as, 1, E2K_SHRD, left, 0x1, tmp2);
      emit_alopf1_ri(as, 1, E2K_ANDD, left, 0x1, tmp1);
    } else {
      left = ra_alloc1(as, ir->op1, RSET_GPR);
      op = irt_isnum(ir->t) ? (st64 ? E2K_IDTOFD : E2K_ISTOFD) :
                              (st64 ? E2K_IDTOFS : E2K_ISTOFS);
      emit_alopf2_r(as, 0, op, left, dest);
    }
  } else if (stfp) { /* FP to INT conversion */
    left = ra_alloc1(as, ir->op1, RSET_GPR);
    if (irt_isguard(ir->t)) {
      /* Checked conversions are only supported from NUM to INT */
      lj_assertA(irt_isint(ir->t) && st == IRT_NUM,
                 "bad type for checked CONV");
      asm_tointg(as, ir, left);
    } else {
      dest = ra_dest(as, ir, RSET_GPR);
      Reg tmp = ra_scratch(as, rset_exclude(RSET_GPR, left));
      if (irt_isu64(ir->t)) { /* FP to U64 */
        intptr_t k = (st == IRT_NUM) ? 0x43e0000000000000 : 0x5f000000;
        op = (st == IRT_NUM) ? E2K_FDTOIDTR : E2K_FSTOIDTR;
        emit_alopf1_ri_rlp(as, 1, E2K_ADDD, tmp, 0x8000000000000000, dest, RID_PRED0, 1);
        emit_alopf2_r_rlp(as, 0, op, left, dest, RID_PRED0, 0);
        emit_alopf2_r(as, 1, op, tmp, tmp);
        emit_alopf7_ri(as, 0, (st == IRT_NUM) ? E2K_FCMPLTDB : E2K_FCMPLTSB,
                       left, k, RID_PRED0);
        emit_alopf1_ri(as, 1, (st == IRT_NUM) ? E2K_FSUBD : E2K_FSUBS,
                       left, k, tmp);
      } else if (irt_isu32(ir->t)) { /* FP to U32 */
        op = (st == IRT_NUM) ? E2K_FDTOIDTR : E2K_FSTOIDTR;
        emit_alopf1_ri(as, 0, E2K_GETFD, dest, 0x800, dest);
        emit_alopf2_r(as, 0, op, left, dest);
      } else {
        op = irt_is64(ir->t) ?
             (st == IRT_NUM ? E2K_FDTOIDTR : E2K_FSTOIDTR) :
             (st == IRT_NUM ? E2K_FDTOISTR : E2K_FSTOISTR);
        emit_alopf2_r(as, 0, op, left, dest);
      }
    }
  } else { /* INT to INT conversion */
    dest = ra_dest(as, ir, RSET_GPR);
    if (st >= IRT_I8 && st <= IRT_U16) { /* Extend to 32/64 bit integer. */
      left = ra_alloc1(as, ir->op1, RSET_GPR);
      emit_alopf1_ir(as, 0, E2K_SXT,
                     (ir->op2 & IRCONV_SEXT) ?
                     (st == IRT_I8 ? SXT_BS : SXT_HS) :
                     (st == IRT_U8 ? SXT_BZ : SXT_HZ), left, dest);
    } else { /* 32/64 bit integer conversions */
      if (irt_is64(ir->t)) {
        if (st64) { /* 64/64 bit no-op (cast) */
          ra_leftov(as, dest, ir->op1);
        } else {
          left = ra_alloc1(as, ir->op1, RSET_GPR);
          emit_alopf1_ir(as, 0, E2K_SXT, (ir->op2 & IRCONV_SEXT) ?
                                         SXT_WS : SXT_WZ, left, dest);
        }
      } else {
        if (st64 && !(ir->op2 & IRCONV_NONE)) {
        /* This is either a 32 bit reg/reg mov which zeroes the hiword
           or a load of the loword from a 64 bit address. */
          left = ra_alloc1(as, ir->op1, RSET_GPR);
          emit_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, left, dest);
        } else { /* 32/32 bit no-op (cast). */
          ra_leftov(as, dest, ir->op1);
        }
      }
    }
  }
}

static void asm_strto(ASMState *as, IRIns *ir)
{
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_strscan_num];
  IRRef args[2];
  int32_t ofs = 0;
  RegSet drop = RSET_SCRATCH;
  if (ra_hasreg(ir->r)) rset_set(drop, ir->r);  /* Spill dest reg (if any). */
  ra_evictset(as, drop);
  ofs = sps_scale(ir->s);
  asm_guard(as, RID_PRED0, 0); /* Test return status. */
  emit_alopf7_ri(as, 0, E2K_CMPEDB, RID_RET, 0, RID_PRED0);
  args[0] = ir->op1;      /* GCstr *str */
  args[1] = ASMREF_TMP1;  /* TValue *n  */
  asm_gencall(as, ci, args);
  /* Store the result to the spill slot or temp slots. */
  emit_alopf1_ri(as, 0, E2K_ADDD, RID_SP, ofs,
                 ra_releasetmp(as, ASMREF_TMP1));
}

/* -- Memory references --------------------------------------------------- */

/* Store tagged value for ref at base+ofs. */
static void asm_tvstore64(ASMState *as, Reg base, intptr_t ofs, IRRef ref)
{
  RegSet allow = rset_exclude(RSET_GPR, base);
  IRIns *ir = IR(ref);
  lj_assertA(irt_ispri(ir->t) || irt_isaddr(ir->t) || irt_isinteger(ir->t),
             "store of IR type %d", irt_type(ir->t));
  if (irref_isk(ref)) {
    TValue k;
    lj_ir_kvalue(as->J->L, &k, ir);
    Reg tmp = ra_allock(as, (int64_t)k.u64, allow);
    emit_alopf3_ri(as, 0, E2K_STD, base, ofs, tmp);
  } else {
    Reg src = ra_alloc1(as, ref, allow);
    allow = rset_exclude(allow, src);
    intptr_t type = (intptr_t)irt_toitype(ir->t) << 47;
    Reg tmp = ra_scratch(as, allow);
    emit_alopf3_ri(as, 0, E2K_STD, base, ofs, tmp);
    if (irt_isinteger(ir->t)) {
      emit_alopf1_ri(as, 0, E2K_ADDD, tmp, type, tmp);
      emit_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, src, tmp);
    } else {
      emit_alopf1_ri(as, 0, E2K_ADDD, src, type, tmp);
    }
  }
}

/* Get pointer to TValue. */
static void asm_tvptr(ASMState *as, Reg dest, IRRef ref, MSize mode)
{
  if ((mode & IRTMPREF_IN1)) {
    IRIns *ir = IR(ref);
    if (irt_isnum(ir->t)) {
      if (irref_isk(ref) && !(mode & IRTMPREF_OUT1)) {
        /* Use the number constant itself as a TValue. */
        emit_loada(as, dest, ir_knum(ir));
        return;
      }
      Reg src = ra_alloc1(as, ref, RSET_GPR);
      emit_alopf3_ri(as, 0, E2K_STD, dest, 0, src);
    } else {
      /* Otherwise use g->tmptv to hold the TValue. */
      asm_tvstore64(as, dest, 0, ref);
    }
  }
  emit_loada(as, dest, &J2G(as->J)->tmptv);
}

static void asm_aref(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_GPR;
  Reg dest = ra_dest(as, ir, allow);
  Reg idx, base, tmp;
  if (irref_isk(ir->op2)) {
    IRRef tab = IR(ir->op1)->op1;
    intptr_t ofs = asm_fuseabase(as, tab);
    IRRef refa = ofs ? tab : ir->op1;
    ofs += 8*IR(ir->op2)->i;
    base = ra_alloc1(as, refa, allow);
    emit_alopf1_ri(as, 0, E2K_ADDD, base, ofs, dest);
  } else {
    base = ra_alloc1(as, ir->op1, allow);
    allow = rset_exclude(allow, base);
    idx = ra_alloc1(as, ir->op2, allow);
    allow = rset_exclude(allow, idx);
    tmp = ra_scratch(as, allow);
    emit_alopf1_rr(as, 0, E2K_ADDD, base, tmp, dest);
    emit_alopf1_ri(as, 0, E2K_SHLD, idx, 3, tmp);
    /* if its integer extend first */
    if (irt_isinteger(IR(ir->op2)->t)) {
      emit_alopf1_ir(as, 0, E2K_SXT, SXT_WS, idx, idx);
    }
  }
}

/* Inlined hash lookup. Specialized for key type and for const keys.
** The equivalent C code is:
**   Node *n = hashkey(t, key);
**   do {
**     if (lj_obj_equal(&n->key, key)) return &n->val;
**   } while ((n = nextnode(n)));
**   return niltv(L);
*/
static void asm_href(ASMState *as, IRIns *ir, IROp merge)
{
  RegSet allow = RSET_GPR;
  int destused = ra_used(ir);
  Reg dest = ra_dest(as, ir, allow);
  Reg tab = ra_alloc1(as, ir->op1, rset_clear(allow, dest));
  Reg key = RID_NONE;
  Reg tmp1 = ra_scratch(as, rset_clear(allow, tab));
  Reg tmp2 = ra_scratch(as, rset_clear(allow, tmp1));
  Reg tmp3 = ra_scratch(as, rset_clear(allow, tmp2));
  IRRef refkey = ir->op2;
  IRIns *irkey = IR(refkey);
  int isk = irref_isk(ir->op2);
  IRType1 kt = irkey->t;
  uint32_t khash;
  MCLabel l_end, l_next, l_loop;
  if (!isk || irt_isnum(kt)) {
    key = ra_alloc1(as, refkey, rset_clear(allow, tmp3));
  }

  /* Key not found in chain: jump to exit (if merged or load niltv. */
  l_end = emit_label(as);
  if (merge == IR_NE) {
    asm_guard(as, 0, 0);
  } else if (destused) {
    emit_loada(as, dest, niltvg(J2G(as->J)));
  }

  /* Follow hash chain until the end. */
  emit_ct(as, RID_CTPR1, RID_PRED3, 1);
  emit_alopf1_ri(as, 0, E2K_ADDD, tmp3, 0, dest);
  l_next = emit_label(as);

  /* Type  and value comparison. */
  if (merge == IR_EQ) {
    asm_guard(as, RID_PRED2, 0);
  } else {
    emit_ibranch(as, l_end, RID_PRED2, 0);
  }

  if (irt_isnum(kt)) {
    emit_ibranch(as, l_next, RID_PRED1, 1);
    emit_alopf7_ri(as, 0, E2K_CMPBSB, tmp2, (int32_t)LJ_TISNUM, RID_PRED1);
    emit_alopf7_ri(as, 1, E2K_CMPEDB, tmp3, 0, RID_PRED3);
    emit_alopf7_rr(as, 0, E2K_FCMPEQDB, tmp1, key, RID_PRED2);
    emit_alopf1_ri(as, 0, E2K_SARD, tmp1, 47, tmp2);
  } else {
    if (isk) {
      intptr_t k = 0;
      if (irt_isaddr(kt)) {
        k = (intptr_t)irt_toitype(kt) << 47 | irkey[1].tv.u64;
      } else {
        lj_assertA(irt_ispri(kt) && !irt_isnil(kt), "bad HREF key type");
        k = ~((intptr_t)~irt_toitype(kt) << 47);
      }
      emit_alopf7_ri(as, 0, E2K_CMPEDB, tmp1, k, RID_PRED2);
    } else {
      emit_alopf7_rr(as, 0, E2K_CMPEDB, tmp1, tmp2, RID_PRED2);
    }
    emit_alopf7_ri(as, 1, E2K_CMPEDB, tmp3, 0, RID_PRED3);
  }
  emit_alopf1_ri(as, 0, E2K_LDD, dest, (intptr_t)offsetof(Node, key.u64), tmp1);
  emit_alopf1_ri(as, 1, E2K_LDD, dest, (intptr_t)offsetof(Node, next), tmp3);
  l_loop = emit_label(as);

  if (!isk && irt_isaddr(kt)) {
    emit_alopf1_ri(as, 0, E2K_ADDD, key, (intptr_t)irt_toitype(kt) << 47,
                   tmp2);
  }

  /* Load main position relative to tab->node into dest. */
  khash = isk ? ir_khash(as, irkey) : 1;
  if (khash == 0) {
    emit_alopf1_ri(as, 0, E2K_LDD, tab, (intptr_t)offsetof(GCtab, node),
                   dest);
  } else {
    emit_alopf1_rr(as, 0, E2K_ADDD, dest, tmp1, dest);
    emit_alopf1_ir(as, 0, E2K_SXT, SXT_WS, tmp1, tmp1);
    lj_assertA(sizeof(Node) == 24, "bad Node size");
    emit_alopf1_rr(as, 0, E2K_SUBS, tmp2, tmp1, tmp1);
    emit_alopf1_ri(as, 0, E2K_SHLS, tmp1, 3, tmp1);
    emit_alopf1_ri(as, 0, E2K_SHLS, tmp1, 5, tmp2);
    if (isk) {
      emit_alopf1_ri(as, 0, E2K_ANDS, tmp2, khash, tmp1);
    } else {
      emit_alopf1_rr(as, 0, E2K_ANDS, tmp2, tmp1, tmp1);
    }
    emit_alopf1_ri(as, 0, E2K_LDD, tab, (intptr_t)offsetof(GCtab, node), dest);
    emit_alopf1_ri(as, 0, E2K_LDW, tab, (intptr_t)offsetof(GCtab, hmask), tmp2);
    if (isk) {
      /* Nothing to do */
    } else if (irt_isstr(kt)) {
      emit_alopf1_ri(as, 0, E2K_LDW, key, (intptr_t)offsetof(GCstr, sid), tmp1);
    } else { /*  Must match with hash*() in lj_tab.c. */
      emit_alopf1_rr(as, 0, E2K_SUBS, tmp1, tmp2, tmp1);
      emit_alopf1_ri(as, 0, E2K_SCLS, tmp2, HASH_ROT3, tmp2);
      emit_alopf1_rr(as, 0, E2K_XORS, tmp1, tmp2, tmp1);
      emit_alopf1_ri(as, 0, E2K_SCLS, tmp1, (HASH_ROT2+HASH_ROT1), tmp1);
      emit_alopf1_rr(as, 0, E2K_SUBS, tmp2, dest, tmp2);
      emit_alopf1_rr(as, 0, E2K_XORS, tmp2, tmp1, tmp2);
      emit_alopf1_ri(as, 0, E2K_SCLS, tmp1, HASH_ROT1, dest);
      if (irt_isnum(kt)) {
        emit_alopf1_rr(as, 0, E2K_ADDS, tmp1, tmp1, tmp1);
        emit_alopf1_ri(as, 0, E2K_SHRD, key, 32, tmp1);
        emit_alopf1_ri(as, 0, E2K_ADDS, key, 0, tmp2);
      } else {
        emit_alopf1_ri(as, 0, E2K_SHRD, tmp1, 32, tmp1);
        emit_alopf1_ri(as, 0, E2K_ADDS, key, 0, tmp2);
        emit_alopf1_ri(as, 0, E2K_ADDD, key, (intptr_t)irt_toitype(kt) << 47,
                       tmp1);
      }
    }
    emit_disp(as, RID_CTPR1, l_loop);
  }
}

static void asm_hrefk(ASMState *as, IRIns *ir)
{
  IRIns *kslot = IR(ir->op2);
  IRIns *irkey = IR(kslot->op1);
  int32_t ofs = (int32_t)(kslot->op2 * sizeof(Node));
  int32_t kofs = ofs + (int32_t)offsetof(Node, key);
  intptr_t k = 0;
  RegSet allow = RSET_GPR;
  Reg dest = ra_used(ir) ? ra_dest(as, ir, allow) : RID_NONE;
  allow = rset_exclude(allow, dest);
  Reg node = ra_alloc1(as, ir->op1, RSET_GPR);
  allow = rset_exclude(allow, node);
  Reg key = ra_scratch(as, allow);
  lj_assertA(ofs % sizeof(Node) == 0, "unaligned HREFK slot");
  if (irt_ispri(irkey->t)) {
    lj_assertA(!irt_isnil(irkey->t), "bad HREFK key type");
    k = ~((int64_t)~irt_toitype(irkey->t) << 47);
  } else if (irt_isnum(irkey->t)) {
    k = (int64_t)ir_knum(irkey)->u64;
  } else {
    k = ((int64_t)irt_toitype(irkey->t) << 47) | (int64_t)ir_kgc(irkey);
  }
  if (ra_hasreg(dest)) {
    emit_alopf1_ri(as, 0, E2K_ADDD, node, ofs, dest);
  }
  asm_guard(as, RID_PRED0, 1);
  emit_alopf7_ri(as, 0, E2K_CMPEDB, key, k, RID_PRED0);
  emit_alopf1_ri(as, 0, E2K_LDD, node, kofs, key);
}

static void asm_uref(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg tmp = RID_NONE;
  int guarded = (irt_t(ir->t) & (IRT_GUARD|IRT_TYPE)) == (IRT_GUARD|IRT_PGC);
  intptr_t ofs = 0;
  if (irref_isk(ir->op1) && !guarded) {
    GCfunc *fn = ir_kfunc(IR(ir->op1));
    MRef *v = &gcref(fn->l.uvptr[(ir->op2 >> 8)])->uv.v;
    ofs = dispofs(as, v);
    emit_alopf1_ri(as, 0, E2K_LDD, RID_DISPATCH, ofs, dest);
  } else {
    if (guarded) {
      tmp = ra_scratch(as, rset_exclude(RSET_GPR, dest));
      asm_guard(as, RID_PRED0, ir->o == IR_UREFC ? 0 : 1);
      emit_alopf7_ri(as, 0, E2K_CMPEDB, tmp, 0, RID_PRED0);
    }
    ofs = ir->o == IR_UREFC ? (intptr_t)offsetof(GCupval, tv)
                             : (intptr_t)offsetof(GCupval, v);
    int op = ir->o == IR_UREFC ? E2K_ADDD : E2K_LDD;
    emit_alopf1_ri(as, 0, op, dest, ofs, dest);
    if (guarded) {
      ofs = (intptr_t)offsetof(GCupval, closed);
      emit_alopf1_ri(as, 0, E2K_LDB, dest, ofs, tmp);
    }
    if (irref_isk(ir->op1)) {
      GCfunc *fn = ir_kfunc(IR(ir->op1));
      GCobj *o = gcref(fn->l.uvptr[(ir->op2 >> 8)]);
      emit_loada(as, dest, o);
    } else {
      ofs = (intptr_t)offsetof(GCfuncL, uvptr) +
            (intptr_t)sizeof(MRef) * (intptr_t)(ir->op2 >> 8);
      emit_alopf1_ri(as, 0, E2K_LDD, ra_alloc1(as, ir->op1, RSET_GPR),
                     ofs, dest);
    }
  }
}

static void asm_fref(ASMState *as, IRIns *ir)
{
  UNUSED(as); UNUSED(ir);
  lj_assertA(!ra_used(ir), "unfused FREF");
}

static void asm_strref(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_GPR;
  Reg dest = ra_dest(as, ir, allow);
  Reg base = ra_alloc1(as, ir->op1, rset_clear(allow, dest));
  IRIns *irr = IR(ir->op2);
  intptr_t ofs = sizeof(GCstr);
  if (irref_isk(ir->op2)) {
    emit_alopf1_ri(as, 0, E2K_ADDD, base, ofs + irr->i, dest);
  } else {
    /* base + ofs + right(32-bit) */
    Reg right = ra_alloc1(as, ir->op2, rset_clear(allow, base));
    emit_alopf1_rr(as, 0, E2K_ADDD, dest, right, dest);
    emit_alopf1_ir(as, 0, E2K_SXT, SXT_WS, right, right);
    emit_alopf1_ri(as, 0, E2K_ADDD, base, ofs, dest);
  }
}

/* -- Loads and stores ---------------------------------------------------- */

static uint32_t asm_loadins(ASMState *as, IRIns *ir, Reg dest)
{
  uint32_t sxt_cop = 0, need_sxt = 0, op = 0;
  switch (irt_type(ir->t)) {
  case IRT_I8:
    need_sxt = 1;
    sxt_cop = SXT_BS;
  case IRT_U8:
    op = E2K_LDB;
    break;
  case IRT_I16:
    need_sxt = 1;
    sxt_cop = SXT_HS;
  case IRT_U16:
    op = E2K_LDH;
    break;
  default:
    op = irt_is64(ir->t) ? E2K_LDD : E2K_LDW;
    break;
  }
  /* ldb and ldh unsigned, need sign extension */
  if (need_sxt) {
    emit_alopf1_ir(as, 0, E2K_SXT, sxt_cop, dest, dest);
  }
  return op;
}

static uint32_t asm_storeins(ASMState *as, IRIns *ir)
{
  UNUSED(as);
  switch (irt_type(ir->t)) {
  case IRT_I8: case IRT_U8: return E2K_STB;
  case IRT_I16: case IRT_U16: return E2K_STH;
  default:
    if (irt_is64(ir->t)) return E2K_STD;
    else return E2K_STW;
  }
}

static void asm_fload(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg base = RID_NONE;
  uint32_t op = asm_loadins(as, ir, dest);
  intptr_t ofs = 0;
  if (ir->op1 == REF_NIL) { /* FLOAD from GG_State with offset. */
    ofs = (intptr_t)(ir->op2 << 2) - GG_OFS(dispatch);
    base = RID_DISPATCH;
  } else if (irref_isk(ir->op1)) {
    IRIns *op1 = IR(ir->op1);
    if (op1->o == IR_KPTR || op1->o == IR_KKPTR) {
      ofs = field_ofs[ir->op2] + dispofs(as, ir_kptr(op1));
      base = RID_DISPATCH;
    } else {
      ofs = field_ofs[ir->op2];
      base = ra_alloc1(as, ir->op1, RSET_GPR);
    }
  } else {
    ofs = field_ofs[ir->op2];
    base = ra_alloc1(as, ir->op1, RSET_GPR);
  }
  emit_alopf1_ri(as, 0, op, base, ofs, dest);
}

static void asm_fstore(ASMState *as, IRIns *ir)
{
  if (ir->r != RID_SINK) {
    Reg src = ra_alloc1(as, ir->op2, RSET_GPR);
    IRIns *irf = IR(ir->op1);
    Reg base = ra_alloc1(as, irf->op1, rset_exclude(RSET_GPR, src));
    intptr_t ofs = field_ofs[irf->op2];
    lj_assertA(!irt_isfp(ir->t), "bad FP FSTORE");
    emit_alopf3_ri(as, 0, asm_storeins(as, ir), base, ofs, src);
  }
}

static void asm_xload(ASMState *as, IRIns *ir)
{
  intptr_t ofs = 0;
  Reg dest = ra_dest(as, ir, RSET_GPR);
  IRRef ref = asm_fusexref(as, ir->op1, &ofs);
  Reg base = ra_alloc1(as, ref, RSET_GPR);
  emit_alopf1_ri(as, 0, asm_loadins(as, ir, dest), base, ofs, dest);
  if (!irt_is64(IR(ref)->t))
    emit_alopf1_ir(as, 0, E2K_SXT, SXT_WS, base, base);
}

static void asm_xstore(ASMState *as, IRIns *ir)
{
  if (ir->r != RID_SINK) {
    intptr_t ofs = 0;
    Reg src = ra_alloc1(as, ir->op2, RSET_GPR);
    IRRef ref = asm_fusexref(as, ir->op1, &ofs);
    Reg base = ra_alloc1(as, ref, rset_exclude(RSET_GPR, src));
    emit_alopf3_ri(as, 0, asm_storeins(as, ir), base, ofs, src);
    if (!irt_is64(IR(ref)->t))
      emit_alopf1_ir(as, 0, E2K_SXT, SXT_WS, base, base);
  }
}

static void asm_ahuvload(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_GPR;
  Reg base, dest = RID_NONE, type = RID_NONE;
  IRType1 t = ir->t;
  intptr_t ofs = 0;
  lj_assertA(irt_isnum(t) || irt_ispri(t) || irt_isaddr(t) ||
             irt_isint(t), "bad load type %d", irt_type(t));
  if (ra_used(ir)) {
    dest = ra_dest(as, ir, allow);
    allow = rset_exclude(allow, dest);
    if (irt_isaddr(t)) {
      emit_alopf1_ri(as, 0, E2K_GETFD, dest, 0xbc0, dest);
    } else if (irt_isint(t)) {
      emit_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, dest, dest);
    }
  }
  base = asm_fuseahuref(as, ir->op1, &ofs, allow);
  allow = rset_exclude(allow, base);
  if (ir->o == IR_VLOAD) ofs += 8 * ir->op2;
  type = ra_scratch(as, allow);
  intptr_t k = irt_isnum(t) ? (int32_t)LJ_TISNUM :
               (int32_t)irt_toitype(t);
  asm_guard(as, RID_PRED0, 1);
  emit_alopf7_ri(as, 0, irt_isnum(t) ? E2K_CMPBSB : E2K_CMPESB,
                 type, k, RID_PRED0);
  if (!ra_hasreg(dest)) dest = type;
  emit_alopf1_ri(as, 0, E2K_SARD, dest, 47, type);
  emit_alopf1_ri(as, 0, E2K_LDD, base, ofs, dest);
}

static void asm_ahustore(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_GPR;
  Reg base, src = RID_NONE;
  intptr_t type = 0;
  intptr_t ofs = 0;
  if (ir->r == RID_SINK)
    return;
  if (irt_isnum(ir->t)) {
    src = ra_alloc1(as, ir->op2, allow);
    allow = rset_exclude(allow, src);
    base = asm_fuseahuref(as, ir->op1, &ofs, allow);
    emit_alopf3_ri(as, 0, E2K_STD, base, ofs, src);
  } else {
    Reg tmp = RID_NONE;
    if (irt_ispri(ir->t)) {
      tmp = ra_allock(as, ~((int64_t)~irt_toitype(ir->t) << 47), allow);
      allow = rset_exclude(allow, tmp);
    } else {
      tmp = ra_scratch(as, allow);
      allow = rset_exclude(allow, tmp);
      src = ra_alloc1(as, ir->op2, allow);
      allow = rset_exclude(allow, src);
      type = (intptr_t)irt_toitype(ir->t) << 47;
    }
    base = asm_fuseahuref(as, ir->op1, &ofs, allow);
    emit_alopf3_ri(as, 0, E2K_STD, base, ofs, tmp);
    if (ra_hasreg(src)) {
      if (irt_isinteger(ir->t)) {
        emit_alopf1_ri(as, 0, E2K_ADDD, tmp, type, tmp);
        emit_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, src, tmp);
      } else {
        emit_alopf1_ri(as, 0, E2K_ADDD, src, type, tmp);
      }
    }
  }
}

static void asm_sload(ASMState *as, IRIns *ir)
{
  Reg dest = RID_NONE, base;
  RegSet allow = RSET_GPR;
  IRType1 t = ir->t;
  intptr_t ofs = 8*((intptr_t)ir->op1-2);
  int op = 0;
  lj_assertA(!(ir->op2 & IRSLOAD_PARENT),
             "bad parent SLOAD");  /* Handled by asm_head_side(). */
  lj_assertA(irt_isguard(ir->t) || !(ir->op2 & IRSLOAD_TYPECHECK),
             "inconsistent SLOAD variant");
  if ((ir->op2 & IRSLOAD_CONVERT) && irt_isguard(t) && irt_isint(t)) {
    dest = ra_scratch(as, allow);
    allow = rset_clear(allow, dest);
    asm_tointg(as, ir, dest);
    t.irt = IRT_NUM; /* Continue with a regular number type check. */
  } else if (ra_used(ir)) {
    lj_assertA(irt_isnum(ir->t) || irt_isint(ir->t) || irt_isaddr(ir->t),
               "bad SLOAD type %d", irt_type(t));
    dest = ra_dest(as, ir, allow);
    allow = rset_clear(allow, dest);
    if (ir->op2 & IRSLOAD_CONVERT) {
      emit_alopf2_r(as, 0, irt_isint(t) ? E2K_FDTOISTR : E2K_ISTOFD,
                    dest, dest);
      t.irt = irt_isint(t) ? IRT_NUM : IRT_INT;
    } else if (irt_isaddr(t)) {
      /* Clear type from pointers. */
      emit_alopf1_ri(as, 0, E2K_GETFD, dest, 0xbc0, dest);
    } else if (irt_isint(t) && (ir->op2 & IRSLOAD_TYPECHECK)) {
      /* Sign-extend integers. */
      emit_alopf1_ir(as, 0, E2K_SXT, SXT_WS, dest, dest);
    }
  }
  base = ra_alloc1(as, REF_BASE, allow);
  if (ir->op2 & IRSLOAD_TYPECHECK) {
    Reg type = ra_scratch(as, rset_clear(allow, base));
    if (!ra_hasreg(dest))
      dest = type;
    if (irt_ispri(t)) {
      asm_guard(as, RID_PRED0, 1);
      intptr_t k = ~((int64_t)~irt_toitype(t) << 47);
      emit_alopf7_ri(as, 0, E2K_CMPEDB, type, k, RID_PRED0);
    } else if (ir->op2 & IRSLOAD_KEYINDEX) {
      asm_guard(as, RID_PRED0, 1);
      intptr_t k = (int32_t)LJ_KEYINDEX;
      emit_alopf7_ri(as, 0, E2K_CMPESB, type, k, RID_PRED0);
      emit_alopf1_ri(as, 0, E2K_SHRD, dest, 32, type);
    } else {
      intptr_t k = irt_isnum(t) ? (int32_t)LJ_TISNUM :
                   (int32_t)irt_toitype(t);
      asm_guard(as, RID_PRED0, 1);
      emit_alopf7_ri(as, 0, irt_isnum(t) ? E2K_CMPBSB : E2K_CMPESB,
                     type, k, RID_PRED0);
      emit_alopf1_ri(as, 0, E2K_SARD, dest, 47, type);
    }
    op = E2K_LDD;
  } else {
    op = irt_isint(t) ? E2K_LDW : E2K_LDD;
  }
  emit_alopf1_ri(as, 0, op, base, ofs, dest);
}

/* -- Allocations --------------------------------------------------------- */
#if LJ_HASFFI
static void asm_cnew(ASMState *as, IRIns *ir)
{
  CTState *cts = ctype_ctsG(J2G(as->J));
  CTypeID id = (CTypeID)IR(ir->op1)->i;
  CTSize sz;
  CTInfo info = lj_ctype_info(cts, id, &sz);
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_mem_newgco];
  IRRef args[4];
  RegSet drop = RSET_SCRATCH;
  RegSet allow = (RSET_GPR & ~RSET_SCRATCH);
  lj_assertA(sz != CTSIZE_INVALID || (ir->o == IR_CNEW && ir->op2 != REF_NIL),
             "bad CNEW/CNEWI operands");
  as->gcsteps++;
  if (ra_hasreg(ir->r))
    rset_clear(drop, ir->r);  /* Dest reg handled below. */
  ra_evictset(as, drop);
  if (ra_used(ir))
    ra_destreg(as, ir, RID_RET);  /* GCcdata * */

  /* Initialize immutable cdata object. */
  if (ir->o == IR_CNEWI) {
    emit_alopf3_ri(as, 0, sz == 8 ? E2K_STD : E2K_STW, RID_RET,
                   (intptr_t)sizeof(GCcdata), ra_alloc1(as, ir->op2, allow));
    lj_assertA(sz == 4 || sz == 8, "bad CNEWI size %d", sz);
  } else if (ir->op2 != REF_NIL) { /* Create VLA/VLS/aligned cdata. */
    ci = &lj_ir_callinfo[IRCALL_lj_cdata_newv];
    args[0] = ASMREF_L;     /* lua_State *L */
    args[1] = ir->op1;      /* CTypeID id   */
    args[2] = ir->op2;      /* CTSize sz    */
    args[3] = ASMREF_TMP1;  /* CTSize align */
    asm_gencall(as, ci, args);
    emit_loadi(as, ra_releasetmp(as, ASMREF_TMP1), (int32_t)ctype_align(info));
    return;
  }

  /* Initialize gct and ctypeid. lj_mem_newgco() already sets marked. */
  Reg tmp1 = ra_scratch(as, allow);
  Reg tmp2 = ra_scratch(as, rset_clear(allow, tmp1));
  emit_alopf3_ri(as, 0, E2K_STB, RID_RET, (intptr_t)offsetof(GCcdata, gct),
                 tmp1);
  emit_alopf3_ri(as, 0, E2K_STH, RID_RET, (intptr_t)offsetof(GCcdata, ctypeid),
                 tmp2);
  emit_alopf1_ii(as, 0, E2K_ADDD, 0, (intptr_t)(~LJ_TCDATA), tmp1);
  emit_alopf1_ii(as, 0, E2K_ADDD, 0, (intptr_t)id, tmp2);
  args[0] = ASMREF_L;     /* lua_State *L */
  args[1] = ASMREF_TMP1;  /* MSize size   */
  asm_gencall(as, ci, args);
  emit_loadi(as, ra_releasetmp(as, ASMREF_TMP1), (int32_t)(sz+sizeof(GCcdata)));
}
#endif

/* -- Write barriers ------------------------------------------------------ */

static void asm_tbar(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_GPR;
  Reg tab = ra_alloc1(as, ir->op1, allow);
  allow = rset_exclude(allow, tab);
  Reg mark = ra_scratch(as, allow);
  Reg link = ra_scratch(as, rset_exclude(allow, mark));
  Reg tmp = link;
  MCLabel l_end = emit_label(as);
  emit_alopf3_ri(as, 0, E2K_STD, tab, (intptr_t)offsetof(GCtab, gclist),
                 link);
  emit_alopf3_ri(as, 0, E2K_STB, tab, (intptr_t)offsetof(GCtab, marked),
                 mark);
  emit_setgl(as, tab, gc.grayagain);
  emit_getgl(as, link, gc.grayagain);
  /* Clear black bit. */
  emit_alopf1_rr(as, 0, E2K_XORD, tmp, mark, mark);
  emit_ibranch(as, l_end, RID_PRED0, 0);
  emit_alopf7_ri(as, 0, E2K_CMPEDB, tmp, 0, RID_PRED0);
  emit_alopf1_ri(as, 0, E2K_ANDD, mark, LJ_GC_BLACK, tmp);
  emit_alopf1_ri(as, 0, E2K_LDB, tab, (intptr_t)offsetof(GCtab, marked), mark);
}

static void asm_obar(ASMState *as, IRIns *ir)
{
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_gc_barrieruv];
  IRRef args[2];
  MCLabel l_end;
  RegSet allow = RSET_GPR;
  Reg obj = RID_NONE, val = RID_NONE, tmp1 = RID_NONE, tmp2 = RID_NONE;
  /* No need for other object barriers (yet). */
  lj_assertA(IR(ir->op1)->o == IR_UREFC, "bad OBAR type");
  ra_evictset(as, RSET_SCRATCH);
  l_end = emit_label(as);
  args[0] = ASMREF_TMP1;  /* global_State *g */
  args[1] = ir->op1;      /* TValue *tv      */
  asm_gencall(as, ci, args);
  emit_loada(as, ra_releasetmp(as, ASMREF_TMP1), J2G(as->J));
  obj = IR(ir->op1)->r;
  val = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, obj));
  tmp1 = ra_scratch(as, rset_clear(allow, obj));
  tmp2 = ra_scratch(as, rset_clear(allow, tmp1));
  emit_ct(as, RID_CTPR1, RID_PRED1, 0);
  emit_ct(as, RID_CTPR1, RID_PRED2, 0);
  emit_alopf7_ri(as, 0, E2K_CMPESB, tmp1, 0, RID_PRED1);
  emit_alopf7_ri(as, 0, E2K_CMPESB, tmp2, 0, RID_PRED2);
  emit_alopf1_ri(as, 0, E2K_ANDS, tmp1, (intptr_t)LJ_GC_BLACK, tmp1);
  emit_alopf1_ri(as, 0, E2K_ANDS, tmp2, (intptr_t)LJ_GC_WHITES, tmp2);
  emit_alopf1_ri(as, 0, E2K_LDB, obj, (intptr_t)offsetof(GCupval, marked) -
                                      (intptr_t)offsetof(GCupval, tv), tmp1);
  emit_alopf1_ri(as, 0, E2K_LDB, val, (intptr_t)offsetof(GChead, marked), tmp2);
  emit_disp(as, RID_CTPR1, l_end);
}

/* -- FP/int arithmetic and logic operations ------------------------------ */

static void asm_alopf1(ASMState *as, IRIns *ir, int op)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
  Reg right = RID_NONE;
  if (irref_isk(ir->op2)) {
    emit_alopf1_ri(as, 0, op, left, get_kval(as, ir->op2), dest);
  } else {
    right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
    emit_alopf1_rr(as, 0, op, left, right, dest);
  }
}

static void asm_arithov(ASMState *as, IRIns *ir)
{
  RegSet allow = RSET_GPR;
  lj_assertA(!irt_is64(ir->t), "bad usage");
  Reg dest = ra_dest(as, ir, allow);
  Reg left = ra_alloc1(as, ir->op1, rset_clear(allow, dest));
  Reg right = RID_NONE, tmp1 = RID_NONE, tmp2 = RID_NONE;
  if (irref_isk(ir->op2)) {
    /* (dest < left) == (k >= 0 ? 1 : 0) */
    int k = IR(ir->op2)->i;
    if (ir->o == IR_SUBOV) k = (int)(~(unsigned int)k+1u);
    asm_guard(as, RID_PRED0, k >= 0 ? 0 : 1);
    emit_alopf7_rr(as, 0, E2K_CMPLSB, dest, left, RID_PRED0);
    emit_alopf1_ri(as, 0, E2K_ADDS, left, k, dest);
  } else {
    /* ((dest^left) & (dest^(~)right)) < 0 */
    right = ra_alloc1(as, ir->op2, rset_clear(allow, left));
    tmp1 = ra_scratch(as, rset_clear(allow, right));
    tmp2 = ra_scratch(as, rset_clear(allow, tmp1));
    asm_guard(as, RID_PRED0, 0);
    emit_alopf7_ri(as, 0, E2K_CMPLSB, tmp1, 0, RID_PRED0);
    emit_alopf1_rr(as, 0, E2K_ANDS, tmp1, tmp2, tmp1);
    emit_alopf1_rr(as, 0, E2K_XORS, dest, left, tmp1);
    emit_alopf1_rr(as, 0, ir->o == IR_ADDOV ? E2K_XORS : E2K_XORNS,
                   dest, right, tmp2);
    emit_alopf1_rr(as, 0, ir->o == IR_ADDOV ? E2K_ADDS : E2K_SUBS,
                   left, right, dest);
  }
}

static void asm_fpmath(ASMState *as, IRIns *ir)
{
  IRFPMathOp fpm = (IRFPMathOp)ir->op2;
  if (fpm == IRFPM_SQRT) {
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg left = ra_alloc1(as, ir->op1, rset_exclude(RSET_GPR, dest));
    emit_alopf11_rr(as, 0, E2K_FSQRTTD, left, dest, dest);
    emit_alopf12_r(as, 0, E2K_FSQRTID, left, dest);
  /* floor(0x1), ceil(0x2), trunc(0x3) */
  } else if (fpm <= IRFPM_TRUNC) {
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
    emit_alopf11_ir(as, 0, E2K_FDTOIFD, fpm+1, left, dest);
  } else {
    asm_callid(as, ir, IRCALL_lj_vm_floor + fpm);
  }
}

static void asm_bnot(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
  emit_alopf1_ri(as, 0, irt_is64(ir->t) ? E2K_XORD : E2K_XORS,
                 left, -1, dest);
}

static void asm_bswap(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg smask = ra_scratch(as, rset_exclude(RSET_GPR, left));
  if (irt_is64(ir->t)) {
    emit_alopf21_rrr(as, 0, E2K_PSHUFB, left, left, smask, dest);
    emit_alopf1_ii(as, 0, E2K_ADDD, 0, 0x1020304050607, smask);
  } else {
    emit_alopf21_rrr(as, 0, E2K_PSHUFB, left, left, smask, dest);
    emit_alopf1_ii(as, 0, E2K_ADDD, 0, 0x8080808000010203, smask);
    emit_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, left, left);
  }
}

static void asm_mul(ASMState *as, IRIns *ir)
{
  if (irt_isnum(ir->t)) {
    asm_alopf1(as, ir, E2K_FMULD);
  } else {
    /* alopf11 for muls/muld */
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg right = RID_NONE, left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
    uint32_t op = irt_is64(ir->t) ? E2K_MULD : E2K_MULS;
    if (irref_isk(ir->op2)) {
      emit_alopf11_ri(as, 0, op, left, get_kval(as, ir->op2), dest);
    } else {
      right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
      emit_alopf11_rr(as, 0, op, left, right, dest);
    }
  }
}

static void asm_fpdiv(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
  emit_alopf11_rr(as, 0, E2K_FDIVD, left, right, dest);
}

static void asm_neg(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
  if (irt_isnum(ir->t)) {
    emit_alopf1_ri(as, 0, E2K_XORD, left, 0x8000000000000000, dest);
  } else {
    emit_alopf1_ir(as, 0, irt_is64(ir->t) ? E2K_SUBD : E2K_SUBS,
                   0, left, dest);
  }
}

static void asm_abs(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);;
  emit_alopf1_ri(as, 0, E2K_ANDD, left, 0x7fffffffffffffff, dest);
}

static void asm_min_max(ASMState *as, IRIns *ir, int ismax)
{
  if (irt_isnum(ir->t)) {
    asm_alopf1(as, ir, ismax ? E2K_FMAXD : E2K_FMIND);
  } else {
    Reg dest = ra_dest(as, ir, RSET_GPR);
    Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
    Reg right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
    // rset can be ignored if register was already allocated
    if (left == right) {
      if (dest != left) emit_movrr(as, 0, dest, left);
    } else {
      emit_alopf1_rr_mrgc(as, 1, E2K_MERGES, left, right, dest,
                          RID_PRED0, ismax ? 0 : 1);
      emit_alopf7_rr(as, 0, E2K_CMPLSB, left, right, RID_PRED0);
    }
  }
}

#define asm_min(as, ir)   asm_min_max(as, ir, 0)
#define asm_max(as, ir)   asm_min_max(as, ir, 1)

static void asm_mulov(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
  Reg tmp = ra_scratch(as, rset_exclude(RSET_GPR, dest));
  asm_guard(as, RID_PRED0, 1);
  emit_alopf7_rr(as, 0, E2K_CMPEDB, tmp, dest, RID_PRED0);
  emit_alopf1_ir(as, 0, E2K_SXT, SXT_WS, dest, tmp);
  emit_alopf11_rr(as, 0, E2K_SMULX, left, right, dest);
}

#define asm_addov(as, ir) asm_arithov(as, ir)
#define asm_subov(as, ir) asm_arithov(as, ir)

#define asm_sub(as, ir)   asm_alopf1(as, ir, irt_isnum(ir->t) ? E2K_FSUBD : \
                                             (irt_is64(ir->t) ? E2K_SUBD : E2K_SUBS))
#define asm_add(as, ir)   asm_alopf1(as, ir, irt_isnum(ir->t) ? E2K_FADDD : \
                                             (irt_is64(ir->t) ? E2K_ADDD : E2K_ADDS))
#define asm_bor(as, ir)   asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_ORD : E2K_ORS)
#define asm_bxor(as, ir)  asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_XORD : E2K_XORS)
#define asm_band(as, ir)  asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_ANDD : E2K_ANDS)
#define asm_bshr(as, ir)  asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_SHRD : E2K_SHRS)
#define asm_bshl(as, ir)  asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_SHLD : E2K_SHLS)
#define asm_bsar(as, ir)  asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_SARD : E2K_SARS)
#define asm_bror(as, ir)  asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_SCRD : E2K_SCRS)
#define asm_brol(as, ir)  asm_alopf1(as, ir, irt_is64(ir->t) ? E2K_SCLD : E2K_SCLS)
/* -- Comparisons --------------------------------------------------------- */

static const uint32_t asm_compmap[IR_ABC+1] = {
  /* cmpop  op              invert?  */
  /* LT  */ E2K_CMPLSB,  /* inverted */
  /* GE  */ E2K_CMPLSB,
  /* LE  */ E2K_CMPLESB, /* inverted */
  /* GT  */ E2K_CMPLESB,
  /* ULT */ E2K_CMPBSB,  /* inverted */
  /* UGE */ E2K_CMPBSB,
  /* ULE */ E2K_CMPBESB, /* inverted */
  /* UGT */ E2K_CMPBESB,
  /* EQ  */ E2K_CMPESB,  /* inverted */
  /* NE  */ E2K_CMPESB,
  /* ABC */ E2K_CMPBESB, /* same as UGT */
};

static const uint32_t asm_fpcompmap[IR_ABC+1] = {
  /* cmpop  op                invert?        swap? */
  /* LT  */ E2K_FCMPLTDB,  /* inverted */
  /* GE  */ E2K_FCMPLEDB,  /* inverted */ /* swap  */
  /* LE  */ E2K_FCMPLEDB,  /* inverted */
  /* GT  */ E2K_FCMPLTDB,  /* inverted */ /* swap */
  /* ULT */ E2K_FCMPNLEDB, /* inverted */ /* swap */
  /* UGE */ E2K_FCMPNLTDB, /* inverted */
  /* ULE */ E2K_FCMPNLTDB, /* inverted */ /* swap */
  /* UGT */ E2K_FCMPNLEDB, /* inverted */
  /* EQ  */ E2K_FCMPEQDB,  /* inverted */
  /* NE  */ E2K_FCMPEQDB,
  /* ABC */ E2K_FCMPNLEDB, /* inverted */ /* same as UGT */
};

static void asm_comp(ASMState *as, IRIns *ir)
{
  IROp cmpop = ir->o;
  int inverted = 0, op = 0;
  IRRef lref = ir->op1;
  IRRef rref = ir->op2;
  if (cmpop == IR_ABC) cmpop = IR_UGT;
  if (irt_isnum(ir->t)) {
    inverted = (cmpop == IR_NE) ? 0 : 1;
    op = asm_fpcompmap[cmpop];
    if ((cmpop == IR_GE) || (cmpop == IR_GT) ||
        (cmpop == IR_ULT) || (cmpop == IR_ULE)) {
      IRRef tmp = lref; lref = rref; rref = tmp;
    }
  } else {
    inverted = (cmpop&1) ? 0 : 1;
    /* 5 = E2K_CMPBDB - E2K_CMPBSB */
    op = asm_compmap[cmpop] + (irt_is64(ir->t) ? 5 : 0);
  }
  Reg left = ra_alloc1(as, lref, RSET_GPR);
  asm_guard(as, RID_PRED0, inverted);

  if (irref_isk(rref)) {
    intptr_t k = get_kval(as, rref);
    emit_alopf7_ri(as, 0, op, left, k, RID_PRED0);
  } else {
    Reg right = ra_alloc1(as, rref, rset_exclude(RSET_GPR, left));
    emit_alopf7_rr(as, 0, op, left, right, RID_PRED0);
  }
}

#define asm_equal(as, ir) asm_comp(as, ir)

/* -- Split register ops -------------------------------------------------- */

/* Hiword op of a split 32/32 or 64/64 bit op. Previous op is the loword op. */
static void asm_hiop(ASMState *as, IRIns *ir)
{
  /* HIOP is marked as a store because it needs its own DCE logic. */
  int uselo = ra_used(ir-1), usehi = ra_used(ir);  /* Loword/hiword used? */
  if (LJ_UNLIKELY(!(as->flags & JIT_F_OPT_DCE))) uselo = usehi = 1;
  if (!usehi) return;  /* Skip unused hiword op for all remaining ops. */
  switch ((ir-1)->o) {
  case IR_CALLN: case IR_CALLL: case IR_CALLS: case IR_CALLXS:
    if (!uselo)
      ra_allocref(as, ir->op1, RID2RSET(RID_RETLO));  /* Mark lo op as used. */
    break;
  default: lj_assertA(0, "bad HIOP for op %d", (ir-1)->o); break;
  }
}

/* -- Profiling ----------------------------------------------------------- */

static void asm_prof(ASMState *as, IRIns *ir)
{
  UNUSED(ir);
  Reg tmp = ra_scratch(as, RSET_GPR);
  asm_guard(as, RID_PRED0, 1);
  emit_alopf7_ri(as, 0, E2K_CMPANDESB, tmp, HOOK_PROFILE, RID_PRED0);
  emit_alopf1_ri(as, 0, E2K_LDB, RID_DISPATCH,
                 dispofs(as, &J2G(as->J)->hookmask), tmp);
}

/* -- Stack handling ------------------------------------------------------ */

/* Check Lua stack size for overflow. Use exit handler as fallback. */
static void asm_stack_check(ASMState *as, BCReg topslot,
                            IRIns *irp, RegSet allow, ExitNo exitno)
{
  /* Try to get an unused temp register, otherwise use RID_TMP*. */
  Reg pbase = irp ? (ra_hasreg(irp->r) ? irp->r : RID_TMP2) : RID_BASE;
  ExitNo oldsnap = as->snapno;
  allow = rset_exclude(allow, pbase);
  Reg tmp = allow ? rset_pickbot(allow) : RID_TMP3;
  as->snapno = exitno;
  asm_guard(as, RID_PRED0, 0);
  as->snapno = oldsnap;
  emit_alopf7_ri(as, 0, E2K_CMPBDB, tmp, (intptr_t)(8*topslot),
                 RID_PRED0);
  if (allow != RSET_EMPTY) ra_modified(as, tmp);
  emit_alopf1_rr(as, 0, E2K_SUBD, tmp, pbase, tmp);
  emit_alopf1_ri(as, 0, E2K_LDD, tmp, (intptr_t)offsetof(lua_State, maxstack),
                 tmp);
  if (pbase == RID_TMP2)
    emit_getgl(as, RID_TMP2, jit_base);
  emit_getgl(as, tmp, cur_L);
}

/* Restore Lua stack from on-trace state. */
static void asm_stack_restore(ASMState *as, SnapShot *snap)
{
  RegSet allow = rset_exclude(RSET_GPR, RID_BASE);
  SnapEntry *map = &as->T->snapmap[snap->mapofs];
  MSize n, nent = snap->nent;
  /* Store the value of all modified slots to the Lua stack. */
  for (n = 0; n < nent; n++) {
    SnapEntry sn = map[n];
    BCReg s = snap_slot(sn);
    intptr_t ofs = 8*((intptr_t)s-1-LJ_FR2);
    IRRef ref = snap_ref(sn);
    IRIns *ir = IR(ref);
    if ((sn & SNAP_NORESTORE))
      continue;
    if ((sn & SNAP_KEYINDEX)) {
      int64_t kki = (int64_t)LJ_KEYINDEX << 32;
      if (irref_isk(ref)) {
        kki = kki | (int64_t)(uint32_t)ir->i;
        Reg rki = ra_allock(as, kki, allow);
        emit_alopf3_ri(as, 0, E2K_STD, RID_BASE, ofs, rki);
      } else {
        Reg src = ra_alloc1(as, ref, allow);
        allow = rset_exclude(allow, src);
        Reg tmp = ra_scratch(as, allow);
        emit_alopf3_ri(as, 0, E2K_STD, RID_BASE, ofs, tmp);
        emit_alopf1_ri(as, 0, E2K_ADDD, src, kki, tmp);
      }
    } else if (irt_isnum(ir->t)) {
      Reg src = ra_alloc1(as, ref, allow);
      emit_alopf3_ri(as, 0, E2K_STD, RID_BASE, ofs, src);
    } else {
      lj_assertA(irt_ispri(ir->t) || irt_isaddr(ir->t) || irt_isinteger(ir->t),
                 "store of IR type %d", irt_type(ir->t));
      if (irref_isk(ref)) {
        TValue k;
        lj_ir_kvalue(as->J->L, &k, ir);
        Reg rki = ra_allock(as, (int64_t)k.u64, allow);
        emit_alopf3_ri(as, 0, E2K_STD, RID_BASE, ofs, rki);
      } else {
        Reg src = ra_alloc1(as, ref, allow);
        allow = rset_exclude(allow, src);
        int64_t type = (int64_t)irt_toitype(ir->t) << 47;
        Reg tmp = ra_scratch(as, allow);
        emit_alopf3_ri(as, 0, E2K_STD, RID_BASE, ofs, tmp);
        if (irt_isinteger(ir->t)) {
          emit_alopf1_ri(as, 0, E2K_ADDD, tmp, type, tmp);
          emit_alopf1_ir(as, 0, E2K_SXT, SXT_WZ, src, tmp);
        } else {
          emit_alopf1_ri(as, 0, E2K_ADDD, src, type, tmp);
        }
      }
    }
    checkmclim(as);
  }
}

/* -- GC handling --------------------------------------------------------- */

/* Check GC threshold and do one or more GC steps. */
static void asm_gc_check(ASMState *as)
{
  const CCallInfo *ci = &lj_ir_callinfo[IRCALL_lj_gc_step_jit];
  IRRef args[2];
  MCLabel l_end;
  Reg tmp1, tmp2;
  ra_evictset(as, RSET_SCRATCH);
  l_end = emit_label(as);
  /* Exit trace if in GCSatomic or GCSfinalize. Avoids syncing GC objects. */
  /* Assumes asm_snap_prep() already done. */
  asm_guard_gc(as, RID_PRED0, 1);
  emit_alopf7_ri(as, 0, E2K_CMPEDB, RID_RET, 0, RID_PRED0);
  args[0] = ASMREF_TMP1;  /* global_State *g */
  args[1] = ASMREF_TMP2;  /* MSize steps     */
  asm_gencall(as, ci, args);
  tmp1 = ra_releasetmp(as, ASMREF_TMP1);
  emit_alopf1_ri(as, 0, E2K_ADDD, RID_DISPATCH, GG_DISP2G, tmp1);
  tmp2 = ra_releasetmp(as, ASMREF_TMP2);
  emit_loadi(as, tmp2, as->gcsteps);
  /* Jump around GC step if GC total < GC threshold. */
  emit_ibranch(as, l_end, RID_PRED0, 0);
  emit_alopf7_rr(as, 0, E2K_CMPBDB, tmp1, tmp2, RID_PRED0);
  emit_getgl(as, tmp1, gc.total);
  emit_getgl(as, tmp2, gc.threshold);
  as->gcsteps = 0;
  checkmclim(as);
}

/* -- Loop handling ------------------------------------------------------- */

/* Look for CS0. */
static MCode *e2k_lookup_cs0(MCode *p) {
  E2kHS hs = { p[0] };
  if ((hs.fields.c & 1)) {
    p += (hs.fields.x_s_sw & 2) ? 2 : 1;
    return p + __builtin_popcount(hs.fields.als);
  }
  return NULL;
}

/* Look for ibranch. */
static MCode *e2k_lookup_ibranch(MCode *p) {
  E2kHS hs = { p[0] };
  if (hs.fields.x_s_sw & 2) {
    /* TODO: check ct in SS syllable */
    MCode *cs0 = e2k_lookup_cs0(p);
    if (cs0 && (*cs0 >> 28) == E2K_IBRANCH) {
      return cs0;
    }
  }
  return NULL;
}

#define E2K_NOP_LOOP_PATCH 0x4e853fc9

static void asm_loop_fixup(ASMState *as)
{
  MCode *p = as->mctop;
  MCode *target = emit_serialize(as);

  p[-4] = E2K_NOP(4, 0);
  p[-3] = E2K_NOP_LOOP_PATCH;
  p[-2] = 0;
  p[-1] = 0;
  p = p - 4; /* skip nops */
  /* p[-4] - HS; p[-3] - SS; p[-2] - CS0; p[-1] - Align */
  if (as->loopinv) { /* Inverted loop branch? */
    MCode *cs0 = e2k_lookup_ibranch(p - 8);
    lj_assertA(cs0, "expect ibranch");
    /* asm_guard already inverted the cond branch. Only patch the target. */
    *cs0 &= 0xf0000000;
    *cs0 |= E2KF_DISP(target - p + 8);
  } else { /* 4(HS+SS+CS0+Align) */
    bundle_ibranch(as, target, 0, 0, &p);
  }
}

static void asm_loop_tail_fixup(ASMState *as)
{
  lj_assertA(!as->sched.tail, "expect empty sched queue");

  if (as->loopinv) as->mctop -= 4;
}

/* -- Head of trace ------------------------------------------------------- */

/* Coalesce BASE register for a root trace. */
static void asm_head_root_base(ASMState *as)
{
  IRIns *ir = IR(REF_BASE);
  Reg r = ir->r;
  if (ra_hasreg(r)) {
    ra_free(as, r);
    if (rset_test(as->modset, r) || irt_ismarked(ir->t))
      ir->r = RID_INIT; /* No inheritance for modified BASE register. */
    if (r != RID_BASE)
      emit_movrr(as, 0, r, RID_BASE);
  }

  /* TODO: sched: setwd/setbn */
  emit_serializef(as, "setwd/setbn");

  /* setup jit window */
  bundle_window(as, JIT_WSZ, JIT_NFX, JIT_DBL,
                JIT_RSZ, JIT_RBS, JIT_RCUR, &as->mcp);
}

/* Coalesce BASE register for a side trace. */
static Reg asm_head_side_base(ASMState *as, IRIns *irp)
{
  IRIns *ir = IR(REF_BASE);
  Reg r = ir->r;
  if (ra_hasreg(r)) {
    ra_free(as, r);
    if (rset_test(as->modset, r) || irt_ismarked(ir->t))
      ir->r = RID_INIT; /* No inheritance for modified BASE register. */
    if (irp->r == r) {
      return r;  /* Same BASE register already coalesced. */
    } else if (ra_hasreg(irp->r) && rset_test(as->freeset, irp->r)) {
      emit_movrr(as, 0, r, irp->r); /* Move from coalesced parent reg. */
      return irp->r;
    } else {
      emit_getgl(as, r, jit_base);  /* Otherwise reload BASE. */
    }
  }
  emit_serialize(as);
  return RID_NONE;
}


/* -- Tail of trace ------------------------------------------------------- */

/* Fixup the tail code. */
static void asm_tail_fixup(ASMState *as, TraceNo lnk)
{
  MCode *target = lnk ? traceref(as->J, lnk)->mcode : (MCode *)lj_vm_exit_interp;
  MCode *p = as->mctop;
  int32_t spadj = as->T->spadjust;

  lj_assertA(!as->sched.tail, "expect empty sched queue");

  if (spadj) {
    p[-2] = E2K_NOP(2, 0);
    p[-1] = 0;
    p -= 2; /* skip nops */
    bundle_len(as, 6);
    bundle_ibranch(as, target, 0, 0, 0);
    bundle_alopf12_i(as, 0, E2K_GETSP, spadj, RID_SP, &p);
  } else {
    p[-4] = E2K_NOP(4, 0);
    p[-3] = 0;
    p[-2] = 0;
    p[-1] = 0;
    p -= 4; /* skip nops */
    bundle_len(as, 4);
    bundle_ibranch(as, target, 0, 0, &p);
  }
}

/* Prepare tail of code. */
static void asm_tail_prep(ASMState *as)
{
  /* initialized by zero, it counts as nop */
  as->mcp = as->mctop - 8;
  as->invmcp = as->loopref ? as->mcp : NULL;

  sched_reset(as);
}

/* -- Trace setup --------------------------------------------------------- */

/* Ensure there are enough stack slots for call arguments. */
static Reg asm_setup_call_slots(ASMState *as, IRIns *ir, const CCallInfo *ci)
{
  IRRef args[CCI_NARGS_MAX*2];
  uint32_t nargs = CCI_XNARGS(ci);
  int nslots = 0, ngpr = REGARG_NUMGPR;
  int is_vararg = ci->flags & CCI_VARARG;
  asm_collectargs(as, ir, ci, args);
  /* empty slots for first 8 args or slots if vararg */
  if ((nargs > ngpr) || is_vararg)
    nslots = nargs * 2;
  if (nslots > as->evenspill) /* Leave room for args in stack slots. */
    as->evenspill = nslots;
  return REGSP_HINT(RID_RET);
}

static void asm_setup_sched_latencies(ASMState *as, int iset)
{
  static uint8_t initialized = 0;

  if (initialized)
    return;

  /* Initialize latency table with all ones. */
  for (int out = 0; out < OUT_LEN; out++)
    for (int in = 0; in < IN_LEN; in++)
      e2klat[out][in] = 1;

  /* int ops */
  sched_latency_set(OUT_INT,    IN_R,     2);

  /* int cmp ops */
  /* TODO: sched: elbrus-v6 cmp->ct|rlp latency */
  sched_latency_set(OUT_ICMP,   IN_CT,    3);
  sched_latency_set(OUT_ICMP,   IN_RLP,   2);

  /* rrd, movtd, getsp */
  sched_latency_set(OUT_SYS,    IN_I,     2);
  sched_latency_set(OUT_SYS,    IN_F,     2);
  sched_latency_set(OUT_SYS,    IN_S,     1);
  sched_latency_set(OUT_SYS,    IN_S_C,   3);
  sched_latency_set(OUT_SYS,    IN_R,     4);
  sched_latency_set(OUT_SYS,    IN_CT,    9);

  /* load ops */
  if (iset < 6) {
    sched_latency_set(OUT_LOAD,   IN_I,     3);
    sched_latency_set(OUT_LOAD,   IN_F,     3);
    sched_latency_set(OUT_LOAD,   IN_S,     4);
    sched_latency_set(OUT_LOAD,   IN_S_C,   4);
    sched_latency_set(OUT_LOAD,   IN_R,     4);
  } else {
    sched_latency_set(OUT_LOAD,   IN_I,     5);
    sched_latency_set(OUT_LOAD,   IN_F,     5);
    sched_latency_set(OUT_LOAD,   IN_S,     6);
    sched_latency_set(OUT_LOAD,   IN_S_C,   6);
    sched_latency_set(OUT_LOAD,   IN_R,     6);
  }

  /* fp ops */
  sched_latency_set(OUT_FP,     IN_I,     6);
  sched_latency_set(OUT_FP,     IN_F,     4);
  sched_latency_set(OUT_FP,     IN_S,     4);
  sched_latency_set(OUT_FP,     IN_S_C,   6);
  sched_latency_set(OUT_FP,     IN_R,     6);

  /* fp cmp ops */
  /* TODO: sched: elbrus-v6 fcmp->ct|rlp latency */
  sched_latency_set(OUT_FCMP,   IN_CT,    5);
  sched_latency_set(OUT_FCMP,   IN_RLP,   4);
  sched_latency_set(OUT_FCMP,   IN_ELP,   3);

  /* fp misc ops */
  sched_latency_set(OUT_FMISC,  IN_I,     4);
  sched_latency_set(OUT_FMISC,  IN_F,     2);
  sched_latency_set(OUT_FMISC,  IN_S,     2);
  sched_latency_set(OUT_FMISC,  IN_S_C,   3);
  sched_latency_set(OUT_FMISC,  IN_R,     4);

  /* fdiv ops */
  sched_latency_set(OUT_FDIV,   IN_I,     13);
  sched_latency_set(OUT_FDIV,   IN_F,     11);
  sched_latency_set(OUT_FDIV,   IN_S,     11);
  sched_latency_set(OUT_FDIV,   IN_S_C,   13);
  sched_latency_set(OUT_FDIV,   IN_R,     13);

  /* fsqrtt ops */
  sched_latency_set(OUT_FSQRTT, IN_I,     16);
  sched_latency_set(OUT_FSQRTT, IN_F,     14);
  sched_latency_set(OUT_FSQRTT, IN_S,     14);
  sched_latency_set(OUT_FSQRTT, IN_S_C,   16);
  sched_latency_set(OUT_FSQRTT, IN_R,     16);

  /* vec int ops */
  sched_latency_set(OUT_VINT,   IN_I,     4);
  sched_latency_set(OUT_VINT,   IN_F,     1);
  sched_latency_set(OUT_VINT,   IN_S,     1);
  sched_latency_set(OUT_VINT,   IN_S_C,   3);
  sched_latency_set(OUT_VINT,   IN_R,     4);

  /* disp ops */
  sched_latency_set(OUT_DISP,   IN_CT,    5);

  initialized = 1;
}

static void asm_setup_sched(ASMState *as)
{
  memset(&as->sched, 0, sizeof(as->sched));
  /* Do not forget to preserve across sched_reset calls. */
  if (!getenv(LJ_E2K_SCHED_DISABLE))
    as->sched.enable = E2K_SCHED;
  if (getenv(LJ_E2K_SCHED_DISABLE_NOPS))
    as->sched.enable &= ~E2K_SCHED_NOPS;
  if (getenv(LJ_E2K_SCHED_DEBUG))
    as->sched.dbg = 1;

  asm_setup_sched_latencies(as, __iset__);
}

/* Target-specific setup. */
static void asm_setup_target(ASMState *as)
{
  bundle_reset(&as->bundle);
  asm_setup_sched(as);
  asm_exitstub_setup(as);
}

/* -- Trace patching ------------------------------------------------------ */

#define E2K_OPC_ADDD 0x11
#define E2K_REG_EXITNO 0xf0 /* g16 */
#define E2K_IS_SRC1_IMM5(x)   ((x) >= 0xc0 && (x) < 0xe0)
#define E2K_IS_SRC2_IMM4(x)   ((x) >= 0xc0 && (x) < 0xd0)
#define E2K_IS_SRC2_LIT16(x)  ((x) >= 0xd0 && (x) < 0xd8)
#define E2K_IS_SRC2_LIT32(x)  ((x) >= 0xd8 && (x) < 0xdc)
#define E2K_IS_SRC2_LIT64(x)  ((x) >= 0xdc && (x) < 0xdf)
#define E2K_IS_SRC2_LIT(x)    ((x) >= 0xd0 && (x) < 0xdf)

#define E2K_ALS_NOP 0x11c0c0df

/* Extract immediate or literal value for src2. */
static uint64_t e2k_extract_lit(jit_State *J, MCode *p, uint8_t src2) {
  E2kHS hs;
  int index;
  uint32_t *lts;

  if (E2K_IS_SRC2_IMM4(src2)) {
    return src2 & 15;
  }

  /* Get pointer to the lts0. */
  hs.i = p[0];
  index = (hs.fields.lng + 1) * 2 - hs.fields.cds - hs.fields.pls;
  lts = (uint32_t *) (p + index - 1);

  if (E2K_IS_SRC2_LIT16(src2)) {
    index = src2 & 1;
    return (int16_t) (lts[-index] >> ((src2 & 4) ? 16 : 0));
  } else if (E2K_IS_SRC2_LIT32(src2)) {
    index = src2 & 3;
    return (int32_t) lts[-index];
  } else if (E2K_IS_SRC2_LIT64(src2)) {
    index = src2 & 3;
    return ((uint64_t) lts[-index - 1] << 32) | lts[-index];
  }

#ifdef LUA_USE_ASSERT
  lj_assertJ(0, "expect literal src2(%#x)", src2);
  return 0;
#else
  abort();
#endif
}

/* Look for load of exit number. */
static MCode *e2k_lookup_exitno_load(jit_State *J, MCode *p, ExitNo *exitno) {
  E2kHS hs = { p[0] };
  uint32_t als;
  uint8_t src1, src2;
  int offset;

  if (hs.fields.als == 0)
    return NULL;

  offset = hs.fields.x_s_sw & 2 ? 2 : 1;
  for (int i = 0; i < 6; i++) {
    /* ALS must be allocated. */
    if ((hs.fields.als & (1 << i)) == 0)
      continue;

    /* ALES must not be allocated. */
    if ((hs.fields.ales & (1 << i)) != 0)
      continue;

    /* Look for addd imm, imm|lit, TMP0 */
    als = p[offset];
    src2 = als >> 8;
    src1 = als >> 16;
    if (((als >> 24) & 0x7f) == E2K_OPC_ADDD
        && (als & 0xff) == E2K_REG_EXITNO
        && E2K_IS_SRC1_IMM5(src1)
        && (E2K_IS_SRC2_IMM4(src2) || E2K_IS_SRC2_LIT(src2))) {
      *exitno = (src1 & 31) + e2k_extract_lit(J, p, src2);
      return p + offset;
    }

    offset += 1;
  }

  return NULL;
}

/* Look for nopatch gc check. */
static MCode *e2k_lookup_nopatch_gc_check(jit_State *J, MCode *p) {
  E2kHS hs = { p[0] };
  MCode *als;

  if (hs.fields.als == 0)
    return NULL;

  als = p + (hs.fields.x_s_sw & 2 ? 2 : 1);
  for (int i = 0; i < 6; i++) {
    if ((hs.fields.als & (1 << i)) == 0)
      continue;

    if ((hs.fields.ales & (1 << i)) == 0
        && *als == E2K_NOPATCH_GC_CHECK) {
      return als;
    }

    als += 1;
  }

  return NULL;
}

/* Patch exit jumps of existing machine code to a new target. */
void lj_asm_patchexit(jit_State *J, GCtrace *T, ExitNo exitno, MCode *target)
{
  MCode *p = T->mcode;
  MCode *pe = (MCode *)((char *)p + T->szmcode);
  MCode *px = exitstub_trace_addr(T, exitno);
  MCode *cstart = NULL, *cstop = NULL;
  MCode *mcarea = lj_mcode_patch(J, p, 0);
  MCode *als = NULL;
  ExitNo als_exitno = 0;
  int gc_check = 0;
  while (p < pe) {
    E2kHS hs = { p[0] };
    MCode *t;
    int lng = (hs.fields.lng + 1) * 2;

    if (e2k_lookup_nopatch_gc_check(J, p)) {
      /* GC checks must not be patched. */
      als = NULL;
      gc_check = 1;
    }

    t = e2k_lookup_exitno_load(J, p, &als_exitno);
    if (t) {
      als = t;
      if (gc_check || als_exitno != exitno) {
        /* Do not patch next exitstub branch. */
        als = NULL;
        gc_check = 0;
      }
    }

    if (als) {
      int patched = 0;
      /* Look for exitstub branch. */
      MCode *cs0 = e2k_lookup_ibranch(p);
      if (cs0 && *cs0 == E2KF_DISP(px - p)) {
        patched = 1;
        *cs0 = E2KF_DISP(target - p);
        cstop = p + lng;
        if (!cstart) cstart = p;
      } else if (p + lng == pe && p[lng] == E2K_NOP(4, 0)) {
        /* Patch NOP after code for inverted loop branch. Use of J is ok. */
        lj_assertJ(p[lng + 1] == E2K_NOP_LOOP_PATCH, "expected NOP");
        patched = 1;
        /* ibranch target */
        p[lng + 0] = 0x00005012; /* HS */
        p[lng + 1] = 0xc0000020; /* SS */
        p[lng + 2] = E2KF_DISP(target - p - lng); /* CS0 */
        p[lng + 3] = 0; /* PAD */
        cstop = p + lng + 4;
        if (!cstart) cstart = p;
      }
      if (patched) {
        *als = E2K_ALS_NOP;
        als = NULL;
      }
    }

    /* Skip this bundle. */
    p += lng;
  }
  if (cstart) lj_mcode_sync(cstart, cstop);
  lj_mcode_patch(J, mcarea, 1);
}

/* -- Static data --------------------------------------------------------- */

static const E2kReservation e2kres[] = {
/*  unit        out         resources */
  { UNIT_INT,   OUT_INT,    RES_ALS_012345 },       /* TYPE_INT */
  { UNIT_FPU,   OUT_FP,     RES_ALS_0134 },         /* TYPE_IMUL */
  { UNIT_INT,   OUT_ICMP,   RES_ALS_0134 },         /* TYPE_ICMP */
  { UNIT_INT,   OUT_SYS,    RES_ALS0 },             /* TYPE_SYS */
  { UNIT_INT,   OUT_LOAD,   RES_ALS_0235 },         /* TYPE_LOAD */
  { UNIT_INT,   OUT_NONE,   RES_ALS_25 },           /* TYPE_STORE */
  { UNIT_FPU,   OUT_FP,     RES_ALS_0134 },         /* TYPE_FADD */
  { UNIT_FPU,   OUT_FP,     RES_ALS_0134 },         /* TYPE_FMUL */
  { UNIT_FPU,   OUT_FDIV,   RES_ALS5 },             /* TYPE_FDIV */
  { UNIT_FPU,   OUT_FSQRTT, RES_ALS5 },             /* TYPE_FSQRTT */
  { UNIT_FPU,   OUT_FCMP,   RES_ALS_0134 },         /* TYPE_FCMP */
  { UNIT_FPU,   OUT_FP,     RES_ALS_0134 },         /* TYPE_FMISC */
  { UNIT_FPU,   OUT_FP,     RES_ALS_0134 },         /* TYPE_FCVT */
  { UNIT_FPU,   OUT_VINT,   RES_ALS_0134 },         /* TYPE_VINT */
  { UNIT_CT,    OUT_DISP,   RES_CS0 },              /* TYPE_PREP */
  { UNIT_CT,    OUT_NONE,   RES_SS },               /* TYPE_CT */
};

static const E2kOp e2kop[] = {
/*  name          type              ales    als */
  { "ANDs",       TYPE_INT,         0x0000, 0x00000000 },
  { "ANDd",       TYPE_INT,         0x0000, 0x01000000 },
  { "ANDNs",      TYPE_INT,         0x0000, 0x02000000 },
  { "ANDNd",      TYPE_INT,         0x0000, 0x03000000 },
  { "ORs",        TYPE_INT,         0x0000, 0x04000000 },
  { "ORd",        TYPE_INT,         0x0000, 0x05000000 },
  { "ORNs",       TYPE_INT,         0x0000, 0x06000000 },
  { "ORNd",       TYPE_INT,         0x0000, 0x07000000 },
/* --------------------------------------------------- */
  { "XORs",       TYPE_INT,         0x0000, 0x08000000 },
  { "XORd",       TYPE_INT,         0x0000, 0x09000000 },
  { "XORNs",      TYPE_INT,         0x0000, 0x0a000000 },
  { "XORNd",      TYPE_INT,         0x0000, 0x0b000000 },
  { "SXT",        TYPE_INT,         0x0000, 0x0c000000 },
  { "MERGEs",     TYPE_INT,         0x0000, 0x0e000000 },
  { "MERGEd",     TYPE_INT,         0x0000, 0x0f000000 },
/* --------------------------------------------------- */
  { "ADDs",       TYPE_INT,         0x0000, 0x10000000 },
  { "ADDd",       TYPE_INT,         0x0000, 0x11000000 },
  { "SUBs",       TYPE_INT,         0x0000, 0x12000000 },
  { "SUBd",       TYPE_INT,         0x0000, 0x13000000 },
  { "SCLs",       TYPE_INT,         0x0000, 0x14000000 },
  { "SCLd",       TYPE_INT,         0x0000, 0x15000000 },
  { "SCRs",       TYPE_INT,         0x0000, 0x16000000 },
  { "SCRd",       TYPE_INT,         0x0000, 0x17000000 },
/* --------------------------------------------------- */
  { "SHLs",       TYPE_INT,         0x0000, 0x18000000 },
  { "SHLd",       TYPE_INT,         0x0000, 0x19000000 },
  { "SHRs",       TYPE_INT,         0x0000, 0x1a000000 },
  { "SHRd",       TYPE_INT,         0x0000, 0x1b000000 },
  { "SARs",       TYPE_INT,         0x0000, 0x1c000000 },
  { "SARd",       TYPE_INT,         0x0000, 0x1d000000 },
  { "GETFs",      TYPE_INT,         0x0000, 0x1e000000 },
  { "GETFd",      TYPE_INT,         0x0000, 0x1f000000 },
/* --------------------------------------------------- */
  { "CMPBsb",     TYPE_ICMP,        0x0000, 0x20000020 },
  { "CMPEsb",     TYPE_ICMP,        0x0000, 0x20000040 },
  { "CMPBEsb",    TYPE_ICMP,        0x0000, 0x20000060 },
  { "CMPLsb",     TYPE_ICMP,        0x0000, 0x200000c0 },
  { "CMPLEsb",    TYPE_ICMP,        0x0000, 0x200000e0 },
  { "CMPBdb",     TYPE_ICMP,        0x0000, 0x21000020 },
  { "CMPEdb",     TYPE_ICMP,        0x0000, 0x21000040 },
  { "CMPBEdb",    TYPE_ICMP,        0x0000, 0x21000060 },
  { "CMPLdb",     TYPE_ICMP,        0x0000, 0x210000c0 },
  { "CMPLEdb",    TYPE_ICMP,        0x0000, 0x210000e0 },
  { "CMPANDEsb",  TYPE_ICMP,        0x0000, 0x22000040 },
  { "STb",        TYPE_STORE,       0x0000, 0x24000000 },
  { "STh",        TYPE_STORE,       0x0000, 0x25000000 },
  { "STw",        TYPE_STORE,       0x0000, 0x26000000 },
  { "STd",        TYPE_STORE,       0x0000, 0x27000000 },
/* --------------------------------------------------- */
  { "FCMPEQsb",   TYPE_FCMP,        0x0000, 0x2e000000 },
  { "FCMPLTsb",   TYPE_FCMP,        0x0000, 0x2e000020 },
  { "FCMPLEsb",   TYPE_FCMP,        0x0000, 0x2e000040 },
  { "FCMPNLTsb",  TYPE_FCMP,        0x0000, 0x2e0000a0 },
  { "FCMPNLEsb",  TYPE_FCMP,        0x0000, 0x2e0000c0 },
  { "FCMPEQdb",   TYPE_FCMP,        0x0000, 0x2f000000 },
  { "FCMPLTdb",   TYPE_FCMP,        0x0000, 0x2f000020 },
  { "FCMPLEdb",   TYPE_FCMP,        0x0000, 0x2f000040 },
  { "FCMPNLTdb",  TYPE_FCMP,        0x0000, 0x2f0000a0 },
  { "FCMPNLEdb",  TYPE_FCMP,        0x0000, 0x2f0000c0 },
/* --------------------------------------------------- */
  { "FADDs",      TYPE_FADD,        0x0000, 0x30000000 },
  { "FADDd",      TYPE_FADD,        0x0000, 0x31000000 },
  { "FSUBs",      TYPE_FADD,        0x0000, 0x32000000 },
  { "FSUBd",      TYPE_FADD,        0x0000, 0x33000000 },
  { "FMINs",      TYPE_FMISC,       0x0000, 0x34000000 },
  { "FMINd",      TYPE_FMISC,       0x0000, 0x35000000 },
  { "FMAXs",      TYPE_FMISC,       0x0000, 0x36000000 },
  { "FMAXd",      TYPE_FMISC,       0x0000, 0x37000000 },
/* --------------------------------------------------- */
  { "FMULs",      TYPE_FMUL,        0x0000, 0x38000000 },
  { "FMULd",      TYPE_FMUL,        0x0000, 0x39000000 },
  { "FSTOIStr",   TYPE_FCVT,        0x0000, 0x3cc20000 },
  { "ISTOFS",     TYPE_FCVT,        0x0000, 0x3cc40000 },
  { "FDTOIDtr",   TYPE_FCVT,        0x0000, 0x3dc20000 },
  { "IDTOFD",     TYPE_FCVT,        0x0000, 0x3dc40000 },
  { "FSTOIDtr",   TYPE_FCVT,        0x0000, 0x3ec20000 },
  { "ISTOFD",     TYPE_FCVT,        0x0000, 0x3ec40000 },
  { "FSTOFD",     TYPE_FCVT,        0x0000, 0x3ec60000 },
  { "FDTOIStr",   TYPE_FCVT,        0x0000, 0x3fc20000 },
  { "IDTOFS",     TYPE_FCVT,        0x0000, 0x3fc40000 },
  { "FDTOFS",     TYPE_FCVT,        0x0000, 0x3fc60000 },
/* --------------------------------------------------- */
  { "MOVTd",      TYPE_SYS,         0x0000, 0x61c00000 },
  { "LDb",        TYPE_LOAD,        0x0000, 0x64000000 },
  { "LDh",        TYPE_LOAD,        0x0000, 0x65000000 },
  { "LDw",        TYPE_LOAD,        0x0000, 0x66000000 },
  { "LDd",        TYPE_LOAD,        0x0000, 0x67000000 },
/* --------------------------------------------------- */
  { "MULs",       TYPE_IMUL,        0x01c0, 0x20000000 },
  { "MULd",       TYPE_IMUL,        0x01c0, 0x21000000 },
  { "UMULX",      TYPE_IMUL,        0x01c0, 0x22000000 },
  { "SMULX",      TYPE_IMUL,        0x01c0, 0x23000000 },
/* --------------------------------------------------- */
  { "FDIVd",      TYPE_FDIV,        0x01c0, 0x49000000 },
  { "FSQRTId",    TYPE_FDIV,        0x01c0, 0x4dc00000 },
/* --------------------------------------------------- */
  { "FSQRTTd",    TYPE_FSQRTT,      0x01c0, 0x51000000 },
/* --------------------------------------------------- */
  { "GETSP",      TYPE_SYS,         0x01c0, 0x58ec0000 },
/* --------------------------------------------------- */
  { "FDTOIFd",    TYPE_FCVT,        0x01c0, 0x6d000000 },
/* --------------------------------------------------- */
  { "PSHUFB",     TYPE_VINT,        0x0f00, 0x4d000000 },
/* --------------------------------------------------- */
  { "DISP",       TYPE_PREP,        0x0000, 0x00000000 },
  { "PREPCALL",   TYPE_PREP,        0x0000, 0x00000000 },
  { "CT",         TYPE_CT,          0x0000, 0x00000000 },
  { "IBRANCH",    TYPE_CT,          0x0000, 0x00000000 },
  { "CALL",       TYPE_CT,          0x0000, 0x00000000 },
};
