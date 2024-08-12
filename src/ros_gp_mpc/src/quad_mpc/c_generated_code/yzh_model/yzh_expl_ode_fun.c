/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) yzh_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_c1 CASADI_PREFIX(c1)
#define casadi_c2 CASADI_PREFIX(c2)
#define casadi_c3 CASADI_PREFIX(c3)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[17] = {13, 1, 0, 13, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};

static const casadi_real casadi_c0[3] = {0., 0., 9.7947000000000006e+00};
static const casadi_real casadi_c1[4] = {-2.8284271247461905e-02, -2.8284271247461905e-02, 2.8284271247461905e-02, 2.8284271247461905e-02};
static const casadi_real casadi_c2[4] = {2.8284271247461905e-02, -2.8284271247461905e-02, -2.8284271247461905e-02, 2.8284271247461905e-02};
static const casadi_real casadi_c3[4] = {-5.0000000000000003e-02, 5.0000000000000003e-02, -5.0000000000000003e-02, 5.0000000000000003e-02};

/* yzh_expl_ode_fun:(i0[13],i1[4],i2[])->(o0[13]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real *w0=w+4, w1, *w2=w+8, w3, w4, w5, w6, *w7=w+16, *w8=w+20, *w9=w+24, *w10=w+28, *w11=w+32, *w12=w+48, w13, w14, w15, *w16=w+67, *w17=w+70, *w18=w+73, *w19=w+76, *w20=w+85;
  /* #0: @0 = input[0][2] */
  casadi_copy(arg[0] ? arg[0]+7 : 0, 3, w0);
  /* #1: output[0][0] = @0 */
  casadi_copy(w0, 3, res[0]);
  /* #2: @1 = 0.5 */
  w1 = 5.0000000000000000e-01;
  /* #3: @2 = zeros(4x1) */
  casadi_clear(w2, 4);
  /* #4: @3 = 0 */
  w3 = 0.;
  /* #5: @0 = input[0][3] */
  casadi_copy(arg[0] ? arg[0]+10 : 0, 3, w0);
  /* #6: @4 = @0[0] */
  for (rr=(&w4), ss=w0+0; ss!=w0+1; ss+=1) *rr++ = *ss;
  /* #7: @4 = (-@4) */
  w4 = (- w4 );
  /* #8: @5 = @0[1] */
  for (rr=(&w5), ss=w0+1; ss!=w0+2; ss+=1) *rr++ = *ss;
  /* #9: @5 = (-@5) */
  w5 = (- w5 );
  /* #10: @6 = @0[2] */
  for (rr=(&w6), ss=w0+2; ss!=w0+3; ss+=1) *rr++ = *ss;
  /* #11: @6 = (-@6) */
  w6 = (- w6 );
  /* #12: @7 = horzcat(@3, @4, @5, @6) */
  rr=w7;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #13: @7 = @7' */
  /* #14: @3 = @0[0] */
  for (rr=(&w3), ss=w0+0; ss!=w0+1; ss+=1) *rr++ = *ss;
  /* #15: @4 = 0 */
  w4 = 0.;
  /* #16: @5 = @0[2] */
  for (rr=(&w5), ss=w0+2; ss!=w0+3; ss+=1) *rr++ = *ss;
  /* #17: @6 = @0[1] */
  for (rr=(&w6), ss=w0+1; ss!=w0+2; ss+=1) *rr++ = *ss;
  /* #18: @6 = (-@6) */
  w6 = (- w6 );
  /* #19: @8 = horzcat(@3, @4, @5, @6) */
  rr=w8;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #20: @8 = @8' */
  /* #21: @3 = @0[1] */
  for (rr=(&w3), ss=w0+1; ss!=w0+2; ss+=1) *rr++ = *ss;
  /* #22: @4 = @0[2] */
  for (rr=(&w4), ss=w0+2; ss!=w0+3; ss+=1) *rr++ = *ss;
  /* #23: @4 = (-@4) */
  w4 = (- w4 );
  /* #24: @5 = 0 */
  w5 = 0.;
  /* #25: @6 = @0[0] */
  for (rr=(&w6), ss=w0+0; ss!=w0+1; ss+=1) *rr++ = *ss;
  /* #26: @9 = horzcat(@3, @4, @5, @6) */
  rr=w9;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #27: @9 = @9' */
  /* #28: @3 = @0[2] */
  for (rr=(&w3), ss=w0+2; ss!=w0+3; ss+=1) *rr++ = *ss;
  /* #29: @4 = @0[1] */
  for (rr=(&w4), ss=w0+1; ss!=w0+2; ss+=1) *rr++ = *ss;
  /* #30: @5 = @0[0] */
  for (rr=(&w5), ss=w0+0; ss!=w0+1; ss+=1) *rr++ = *ss;
  /* #31: @5 = (-@5) */
  w5 = (- w5 );
  /* #32: @6 = 0 */
  w6 = 0.;
  /* #33: @10 = horzcat(@3, @4, @5, @6) */
  rr=w10;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w6;
  /* #34: @10 = @10' */
  /* #35: @11 = horzcat(@7, @8, @9, @10) */
  rr=w11;
  for (i=0, cs=w7; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w8; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w9; i<4; ++i) *rr++ = *cs++;
  for (i=0, cs=w10; i<4; ++i) *rr++ = *cs++;
  /* #36: @12 = @11' */
  for (i=0, rr=w12, cs=w11; i<4; ++i) for (j=0; j<4; ++j) rr[i+j*4] = *cs++;
  /* #37: @7 = input[0][1] */
  casadi_copy(arg[0] ? arg[0]+3 : 0, 4, w7);
  /* #38: @2 = mac(@12,@7,@2) */
  for (i=0, rr=w2; i<1; ++i) for (j=0; j<4; ++j, ++rr) for (k=0, ss=w12+j, tt=w7+i*4; k<4; ++k) *rr += ss[k*4]**tt++;
  /* #39: @2 = (@1*@2) */
  for (i=0, rr=w2, cs=w2; i<4; ++i) (*rr++)  = (w1*(*cs++));
  /* #40: output[0][1] = @2 */
  if (res[0]) casadi_copy(w2, 4, res[0]+3);
  /* #41: @0 = zeros(3x1) */
  casadi_clear(w0, 3);
  /* #42: @1 = 1 */
  w1 = 1.;
  /* #43: @3 = @7[2] */
  for (rr=(&w3), ss=w7+2; ss!=w7+3; ss+=1) *rr++ = *ss;
  /* #44: @4 = sq(@3) */
  w4 = casadi_sq( w3 );
  /* #45: @5 = @7[3] */
  for (rr=(&w5), ss=w7+3; ss!=w7+4; ss+=1) *rr++ = *ss;
  /* #46: @6 = sq(@5) */
  w6 = casadi_sq( w5 );
  /* #47: @4 = (@4+@6) */
  w4 += w6;
  /* #48: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #49: @1 = (@1-@4) */
  w1 -= w4;
  /* #50: @4 = @7[1] */
  for (rr=(&w4), ss=w7+1; ss!=w7+2; ss+=1) *rr++ = *ss;
  /* #51: @6 = (@4*@3) */
  w6  = (w4*w3);
  /* #52: @13 = @7[0] */
  for (rr=(&w13), ss=w7+0; ss!=w7+1; ss+=1) *rr++ = *ss;
  /* #53: @14 = (@13*@5) */
  w14  = (w13*w5);
  /* #54: @6 = (@6-@14) */
  w6 -= w14;
  /* #55: @6 = (2.*@6) */
  w6 = (2.* w6 );
  /* #56: @14 = (@4*@5) */
  w14  = (w4*w5);
  /* #57: @15 = (@13*@3) */
  w15  = (w13*w3);
  /* #58: @14 = (@14+@15) */
  w14 += w15;
  /* #59: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #60: @16 = horzcat(@1, @6, @14) */
  rr=w16;
  *rr++ = w1;
  *rr++ = w6;
  *rr++ = w14;
  /* #61: @16 = @16' */
  /* #62: @1 = (@4*@3) */
  w1  = (w4*w3);
  /* #63: @6 = (@13*@5) */
  w6  = (w13*w5);
  /* #64: @1 = (@1+@6) */
  w1 += w6;
  /* #65: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #66: @6 = 1 */
  w6 = 1.;
  /* #67: @14 = sq(@4) */
  w14 = casadi_sq( w4 );
  /* #68: @15 = sq(@5) */
  w15 = casadi_sq( w5 );
  /* #69: @14 = (@14+@15) */
  w14 += w15;
  /* #70: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #71: @6 = (@6-@14) */
  w6 -= w14;
  /* #72: @14 = (@3*@5) */
  w14  = (w3*w5);
  /* #73: @15 = (@13*@4) */
  w15  = (w13*w4);
  /* #74: @14 = (@14-@15) */
  w14 -= w15;
  /* #75: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #76: @17 = horzcat(@1, @6, @14) */
  rr=w17;
  *rr++ = w1;
  *rr++ = w6;
  *rr++ = w14;
  /* #77: @17 = @17' */
  /* #78: @1 = (@4*@5) */
  w1  = (w4*w5);
  /* #79: @6 = (@13*@3) */
  w6  = (w13*w3);
  /* #80: @1 = (@1-@6) */
  w1 -= w6;
  /* #81: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #82: @5 = (@3*@5) */
  w5  = (w3*w5);
  /* #83: @13 = (@13*@4) */
  w13 *= w4;
  /* #84: @5 = (@5+@13) */
  w5 += w13;
  /* #85: @5 = (2.*@5) */
  w5 = (2.* w5 );
  /* #86: @13 = 1 */
  w13 = 1.;
  /* #87: @4 = sq(@4) */
  w4 = casadi_sq( w4 );
  /* #88: @3 = sq(@3) */
  w3 = casadi_sq( w3 );
  /* #89: @4 = (@4+@3) */
  w4 += w3;
  /* #90: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #91: @13 = (@13-@4) */
  w13 -= w4;
  /* #92: @18 = horzcat(@1, @5, @13) */
  rr=w18;
  *rr++ = w1;
  *rr++ = w5;
  *rr++ = w13;
  /* #93: @18 = @18' */
  /* #94: @19 = horzcat(@16, @17, @18) */
  rr=w19;
  for (i=0, cs=w16; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w17; i<3; ++i) *rr++ = *cs++;
  for (i=0, cs=w18; i<3; ++i) *rr++ = *cs++;
  /* #95: @20 = @19' */
  for (i=0, rr=w20, cs=w19; i<3; ++i) for (j=0; j<3; ++j) rr[i+j*3] = *cs++;
  /* #96: @1 = 0 */
  w1 = 0.;
  /* #97: @5 = 0 */
  w5 = 0.;
  /* #98: @13 = 0.4 */
  w13 = 4.0000000000000002e-01;
  /* #99: @4 = input[1][0] */
  w4 = arg[1] ? arg[1][0] : 0;
  /* #100: @3 = input[1][1] */
  w3 = arg[1] ? arg[1][1] : 0;
  /* #101: @6 = input[1][2] */
  w6 = arg[1] ? arg[1][2] : 0;
  /* #102: @14 = input[1][3] */
  w14 = arg[1] ? arg[1][3] : 0;
  /* #103: @7 = vertcat(@4, @3, @6, @14) */
  rr=w7;
  *rr++ = w4;
  *rr++ = w3;
  *rr++ = w6;
  *rr++ = w14;
  /* #104: @2 = (@13*@7) */
  for (i=0, rr=w2, cs=w7; i<4; ++i) (*rr++)  = (w13*(*cs++));
  /* #105: @13 = @2[0] */
  for (rr=(&w13), ss=w2+0; ss!=w2+1; ss+=1) *rr++ = *ss;
  /* #106: @4 = @2[1] */
  for (rr=(&w4), ss=w2+1; ss!=w2+2; ss+=1) *rr++ = *ss;
  /* #107: @13 = (@13+@4) */
  w13 += w4;
  /* #108: @4 = @2[2] */
  for (rr=(&w4), ss=w2+2; ss!=w2+3; ss+=1) *rr++ = *ss;
  /* #109: @13 = (@13+@4) */
  w13 += w4;
  /* #110: @4 = @2[3] */
  for (rr=(&w4), ss=w2+3; ss!=w2+4; ss+=1) *rr++ = *ss;
  /* #111: @13 = (@13+@4) */
  w13 += w4;
  /* #112: @16 = vertcat(@1, @5, @13) */
  rr=w16;
  *rr++ = w1;
  *rr++ = w5;
  *rr++ = w13;
  /* #113: @1 = 0.083 */
  w1 = 8.3000000000000004e-02;
  /* #114: @16 = (@16/@1) */
  for (i=0, rr=w16; i<3; ++i) (*rr++) /= w1;
  /* #115: @0 = mac(@20,@16,@0) */
  for (i=0, rr=w0; i<1; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w20+j, tt=w16+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #116: @16 = [0, 0, 9.7947] */
  casadi_copy(casadi_c0, 3, w16);
  /* #117: @0 = (@0-@16) */
  for (i=0, rr=w0, cs=w16; i<3; ++i) (*rr++) -= (*cs++);
  /* #118: output[0][2] = @0 */
  if (res[0]) casadi_copy(w0, 3, res[0]+7);
  /* #119: @1 = 0 */
  w1 = 0.;
  /* #120: @5 = 0.4 */
  w5 = 4.0000000000000002e-01;
  /* #121: @7 = (@5*@7) */
  for (i=0, rr=w7, cs=w7; i<4; ++i) (*rr++)  = (w5*(*cs++));
  /* #122: @2 = @7' */
  casadi_copy(w7, 4, w2);
  /* #123: @8 = [-0.0282843, -0.0282843, 0.0282843, 0.0282843] */
  casadi_copy(casadi_c1, 4, w8);
  /* #124: @1 = mac(@2,@8,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w8+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #125: @5 = 0.00059 */
  w5 = 5.9000000000000003e-04;
  /* #126: @1 = (@1/@5) */
  w1 /= w5;
  /* #127: output[0][3] = @1 */
  if (res[0]) res[0][10] = w1;
  /* #128: @1 = 0 */
  w1 = 0.;
  /* #129: @2 = @7' */
  casadi_copy(w7, 4, w2);
  /* #130: @8 = [0.0282843, -0.0282843, -0.0282843, 0.0282843] */
  casadi_copy(casadi_c2, 4, w8);
  /* #131: @1 = mac(@2,@8,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w2+j, tt=w8+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #132: @5 = 0.00059 */
  w5 = 5.9000000000000003e-04;
  /* #133: @1 = (@1/@5) */
  w1 /= w5;
  /* #134: @1 = (-@1) */
  w1 = (- w1 );
  /* #135: output[0][4] = @1 */
  if (res[0]) res[0][11] = w1;
  /* #136: @1 = 0 */
  w1 = 0.;
  /* #137: @7 = @7' */
  /* #138: @2 = [-0.05, 0.05, -0.05, 0.05] */
  casadi_copy(casadi_c3, 4, w2);
  /* #139: @1 = mac(@7,@2,@1) */
  for (i=0, rr=(&w1); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w7+j, tt=w2+i*4; k<4; ++k) *rr += ss[k*1]**tt++;
  /* #140: @5 = 0.00059 */
  w5 = 5.9000000000000003e-04;
  /* #141: @1 = (@1/@5) */
  w1 /= w5;
  /* #142: output[0][5] = @1 */
  if (res[0]) res[0][12] = w1;
  return 0;
}

CASADI_SYMBOL_EXPORT int yzh_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int yzh_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int yzh_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void yzh_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int yzh_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void yzh_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void yzh_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void yzh_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int yzh_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int yzh_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real yzh_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* yzh_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* yzh_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* yzh_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* yzh_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int yzh_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 94;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
