#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define FMI2_FUNCTION_PREFIX VIRYA_AEB_
#include "fmi2Functions.h"

#define GUID "{7f3c9b21-6a42-4e85-91d7-000000000001}"
static void* al(size_t n, size_t s){ return calloc(n,s); }
static void fr(void* p){ free(p); }
static void lg(fmi2ComponentEnvironment e, fmi2String n, fmi2Status s, fmi2String c, fmi2String m, ...){(void)e;(void)n;(void)s;(void)c;(void)m;}
static int fails=0;
#define CHECK(c,msg) do{ if(!(c)){printf("FAIL: %s\n",msg);fails++;} else printf("ok:   %s\n",msg);}while(0)

static fmi2Component mk(const fmi2CallbackFunctions* cb){
  fmi2Component c = fmi2Instantiate("t", fmi2CoSimulation, GUID, "", cb, 0, 0);
  fmi2SetupExperiment(c,0,0,0,0,0); fmi2EnterInitializationMode(c); fmi2ExitInitializationMode(c);
  { fmi2ValueReference vr=3000; fmi2Real value=0.0; fmi2SetReal(c,&vr,1,&value); }
  return c;
}
static void setin(fmi2Component c,double e,double t,double d){
  fmi2ValueReference vr[3]={1000,1001,1002}; double v[3]={e,t,d}; fmi2SetReal(c,vr,3,v);
}
static void out(fmi2Component c,double*vrel,double*ttc,int*st,double*dec,double*stime){
  fmi2ValueReference r[4]={2000,2001,2003,4000}; double v[4]; fmi2ValueReference ri=2002; fmi2Integer s;
  fmi2GetReal(c,r,4,v); fmi2GetInteger(c,&ri,1,&s);
  *vrel=v[0];*ttc=v[1];*dec=v[2];*stime=v[3];*st=s;
}
int main(void){
  fmi2CallbackFunctions cb={ (fmi2CallbackLogger)lg, al, fr, NULL, NULL };
  double vrel,ttc,dec,stime; int st; int i;
  printf("version=%s platform=%s\n", fmi2GetVersion(), fmi2GetTypesPlatform());

  /* 1 defaults = XML start values 1.0/0.0/5.0 */
  fmi2Component c = mk(&cb);
  CHECK(fmi2DoStep(c,0,0.05,1)==fmi2OK,"DoStep OK");
  out(c,&vrel,&ttc,&st,&dec,&stime);
  printf("   start values: vrel=%g ttc=%g state=%d decel=%g\n",vrel,ttc,st,dec);
  CHECK(vrel==1.0 && ttc==5.0 && st==0 && dec==0.0,"start values -> vrel 1, TTC 5, SAFE, 0");

  /* 2 aggressive */
  setin(c,10,0,15); fmi2DoStep(c,0.05,0.05,1); out(c,&vrel,&ttc,&st,&dec,&stime);
  printf("   15/10/0: vrel=%g ttc=%g state=%d decel=%g\n",vrel,ttc,st,dec);
  CHECK(vrel==10.0 && ttc==1.5 && st==4 && dec==-6.0,"15/10/0 -> vrel 10, TTC 1.5, LEVEL_3, -6");

  /* 3 min brake duration: release distance, count steps until SAFE */
  setin(c,10,0,100); int n=0;
  for(i=0;i<10;i++){ fmi2DoStep(c,0,0.05,1); n++; out(c,&vrel,&ttc,&st,&dec,&stime); printf("   step %d state=%d decel=%g state_time=%g\n",n,st,dec,stime); if(st==0)break; }
  CHECK(n==4,"LEVEL_3 held 0.20 s (releases on 4th step at dt=0.05)");

  /* 4 distance 0 while closing -> must brake */
  fmi2Reset(c); fmi2EnterInitializationMode(c); fmi2ExitInitializationMode(c); { fmi2ValueReference vr=3000; fmi2Real value=0.0; fmi2SetReal(c,&vr,1,&value); }
  setin(c,5,0,0); fmi2DoStep(c,0,0.05,1); out(c,&vrel,&ttc,&st,&dec,&stime);
  CHECK(ttc==0.0 && st==4,"distance 0 while closing -> TTC 0, LEVEL_3");

  /* 5 not closing */
  setin(c,0,5,3); fmi2DoStep(c,0,0.05,1); out(c,&vrel,&ttc,&st,&dec,&stime);
  CHECK(vrel==-5.0 && st==0 && dec==0.0,"target faster than ego -> SAFE");

  /* 6 threshold walk: warn/L1/L2/L3 entry from SAFE */
  { double d[4]={39.9,29.9,22.4,17.4}; int exp[4]={1,2,3,4}; int k;
    for(k=0;k<4;k++){ fmi2Reset(c); fmi2EnterInitializationMode(c); fmi2ExitInitializationMode(c); { fmi2ValueReference vr=3000; fmi2Real value=0.0; fmi2SetReal(c,&vr,1,&value); }
      setin(c,10,0,d[k]); fmi2DoStep(c,0,0.05,1); out(c,&vrel,&ttc,&st,&dec,&stime);
      printf("   dist %.1f -> TTC %.2f state %d\n",d[k],ttc,st);
      CHECK(st==exp[k],"threshold entry state"); } }

  /* 7 argument / error handling */
  { fmi2ValueReference bad=9999; double v; fmi2Integer iv;
    CHECK(fmi2GetReal(c,&bad,1,&v)==fmi2Error,"unknown real vr -> error");
    CHECK(fmi2SetReal(c,&bad,1,&v)==fmi2Error,"set unknown vr -> error");
    bad=2000; CHECK(fmi2SetReal(c,&bad,1,&v)==fmi2Error,"set output vr -> error");
    CHECK(fmi2GetInteger(c,&bad,1,&iv)==fmi2Error,"integer query on real vr -> error");
    CHECK(fmi2GetReal(NULL,&bad,1,&v)==fmi2Error,"NULL component -> error");
    CHECK(fmi2GetReal(c,NULL,1,&v)==fmi2Error,"NULL vr array -> error");
    CHECK(fmi2DoStep(c,0,0.0,1)==fmi2Error,"step size 0 -> error");
    CHECK(fmi2DoStep(c,0,-0.05,1)==fmi2Error,"negative step -> error");
    setin(c,NAN,0,5); CHECK(fmi2DoStep(c,0,0.05,1)==fmi2Error,"NaN input -> error"); }
  CHECK(fmi2Instantiate("t",fmi2CoSimulation,"{wrong}","",&cb,0,0)==NULL,"wrong GUID -> NULL");
  CHECK(fmi2Instantiate("t",fmi2ModelExchange,GUID,"",&cb,0,0)==NULL,"ModelExchange -> NULL");
  CHECK(fmi2Instantiate("t",fmi2CoSimulation,NULL,"",&cb,0,0)==NULL,"NULL GUID -> NULL");
  CHECK(fmi2Instantiate("t",fmi2CoSimulation,GUID,"",NULL,0,0)==NULL,"NULL callbacks -> NULL");

  /* 8 different step size: 0.01 s -> 20 steps for 0.20 s */
  fmi2Reset(c); fmi2EnterInitializationMode(c); fmi2ExitInitializationMode(c); { fmi2ValueReference vr=3000; fmi2Real value=0.0; fmi2SetReal(c,&vr,1,&value); }
  setin(c,10,0,15); fmi2DoStep(c,0,0.01,1); setin(c,10,0,100); n=0;
  for(i=0;i<60;i++){ fmi2DoStep(c,0,0.01,1); n++; out(c,&vrel,&ttc,&st,&dec,&stime); if(st==0)break; }
  printf("   dt=0.01: released after %d steps\n",n);
  CHECK(n==20,"dt=0.01 -> hold time still 0.20 s (20 steps)");

  fmi2Terminate(c);
  CHECK(fmi2DoStep(c,0,0.05,1)==fmi2Error,"DoStep after Terminate -> error");
  fmi2FreeInstance(c); fmi2FreeInstance(NULL);
  printf("\n%s (%d failures)\n", fails?"SOME TESTS FAILED":"ALL TESTS PASSED", fails);
  return fails;
}