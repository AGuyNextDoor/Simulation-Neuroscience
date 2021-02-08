/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__StochasticTsodyksMarkram_AMPA_NMDA
#define _nrn_initial _nrn_initial__StochasticTsodyksMarkram_AMPA_NMDA
#define nrn_cur _nrn_cur__StochasticTsodyksMarkram_AMPA_NMDA
#define _nrn_current _nrn_current__StochasticTsodyksMarkram_AMPA_NMDA
#define nrn_jacob _nrn_jacob__StochasticTsodyksMarkram_AMPA_NMDA
#define nrn_state _nrn_state__StochasticTsodyksMarkram_AMPA_NMDA
#define _net_receive _net_receive__StochasticTsodyksMarkram_AMPA_NMDA 
#define odes odes__StochasticTsodyksMarkram_AMPA_NMDA 
#define setRNG setRNG__StochasticTsodyksMarkram_AMPA_NMDA 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define tau_r_AMPA _p[0]
#define tau_d_AMPA _p[1]
#define tau_r_NMDA _p[2]
#define tau_d_NMDA _p[3]
#define e _p[4]
#define mg _p[5]
#define gmax_AMPA _p[6]
#define gmax_NMDA _p[7]
#define tau_rec _p[8]
#define tau_facil _p[9]
#define U1 _p[10]
#define i _p[11]
#define i_AMPA _p[12]
#define i_NMDA _p[13]
#define g_AMPA _p[14]
#define g_NMDA _p[15]
#define g _p[16]
#define A_AMPA _p[17]
#define B_AMPA _p[18]
#define A_NMDA _p[19]
#define B_NMDA _p[20]
#define R _p[21]
#define Use _p[22]
#define DA_AMPA _p[23]
#define DB_AMPA _p[24]
#define DA_NMDA _p[25]
#define DB_NMDA _p[26]
#define DR _p[27]
#define DUse _p[28]
#define factor_AMPA _p[29]
#define factor_NMDA _p[30]
#define v _p[31]
#define _g _p[32]
#define _tsav _p[33]
#define _nd_area  *_ppvar[0]._pval
#define rng	*_ppvar[2]._pval
#define _p_rng	_ppvar[2]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  2;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_setRNG();
 static double _hoc_urand();
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "setRNG", _hoc_setRNG,
 "urand", _hoc_urand,
 0, 0
};
#define urand urand_StochasticTsodyksMarkram_AMPA_NMDA
 extern double urand( _threadargsproto_ );
 /* declare global and static user variables */
 static int _thread1data_inuse = 0;
static double _thread1data[1];
#define _gth 0
#define mggate_StochasticTsodyksMarkram_AMPA_NMDA _thread1data[0]
#define mggate _thread[_gth]._pval[0]
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "tau_r_AMPA", "ms",
 "tau_d_AMPA", "ms",
 "tau_r_NMDA", "ms",
 "tau_d_NMDA", "ms",
 "e", "mV",
 "mg", "mM",
 "gmax_AMPA", "uS",
 "gmax_NMDA", "uS",
 "tau_rec", "ms",
 "tau_facil", "ms",
 "U1", "1",
 "i", "nA",
 "i_AMPA", "nA",
 "i_NMDA", "nA",
 "g_AMPA", "uS",
 "g_NMDA", "uS",
 "g", "uS",
 0,0
};
 static double A_NMDA0 = 0;
 static double A_AMPA0 = 0;
 static double B_NMDA0 = 0;
 static double B_AMPA0 = 0;
 static double R0 = 0;
 static double Use0 = 0;
 static double delta_t = 0.01;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "mggate_StochasticTsodyksMarkram_AMPA_NMDA", &mggate_StochasticTsodyksMarkram_AMPA_NMDA,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[3]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"StochasticTsodyksMarkram_AMPA_NMDA",
 "tau_r_AMPA",
 "tau_d_AMPA",
 "tau_r_NMDA",
 "tau_d_NMDA",
 "e",
 "mg",
 "gmax_AMPA",
 "gmax_NMDA",
 "tau_rec",
 "tau_facil",
 "U1",
 0,
 "i",
 "i_AMPA",
 "i_NMDA",
 "g_AMPA",
 "g_NMDA",
 "g",
 0,
 "A_AMPA",
 "B_AMPA",
 "A_NMDA",
 "B_NMDA",
 "R",
 "Use",
 0,
 "rng",
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 34, _prop);
 	/*initialize range parameters*/
 	tau_r_AMPA = 0.2;
 	tau_d_AMPA = 1.7;
 	tau_r_NMDA = 0.29;
 	tau_d_NMDA = 43;
 	e = 0;
 	mg = 1;
 	gmax_AMPA = 0.001;
 	gmax_NMDA = 0.001;
 	tau_rec = 600;
 	tau_facil = 10;
 	U1 = 0.5;
  }
 	_prop->param = _p;
 	_prop->param_size = 34;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 static void _net_init(Point_process*, double*, double);
 static void _thread_mem_init(Datum*);
 static void _thread_cleanup(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _StochasticTsodyksMarkram_AMPA_NMDA_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 2,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
  _extcall_thread = (Datum*)ecalloc(1, sizeof(Datum));
  _thread_mem_init(_extcall_thread);
  _thread1data_inuse = 0;
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 1, _thread_mem_init);
     _nrn_thread_reg(_mechtype, 0, _thread_cleanup);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 34, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "pointer");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 2;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 StochasticTsodyksMarkram_AMPA_NMDA /Users/martinvielvoye/Simulation-Neuroscience/Week 4/MOOC-neurons-and-synapses-2017/ModelingSynapses/mechanisms/StochasticTsodyksMarkram_AMPA_NMDA.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "AMPA and NMDA glutamate receptor with Stochastic Tsodyks-Markram dynamics";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int setRNG(_threadargsproto_);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[6], _dlist1[6];
 static int odes(_threadargsproto_);
 
/*VERBATIM*/

#include<stdlib.h>
#include<stdio.h>
#include<math.h>

double nrn_random_pick(void* r);
void* nrn_random_arg(int argpos);

 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   DA_AMPA = - A_AMPA / tau_r_AMPA ;
   DB_AMPA = - B_AMPA / tau_d_AMPA ;
   DA_NMDA = - A_NMDA / tau_r_NMDA ;
   DB_NMDA = - B_NMDA / tau_d_NMDA ;
   DR = ( 1.0 - R ) / tau_rec ;
   DUse = - Use / tau_facil ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 DA_AMPA = DA_AMPA  / (1. - dt*( ( - 1.0 ) / tau_r_AMPA )) ;
 DB_AMPA = DB_AMPA  / (1. - dt*( ( - 1.0 ) / tau_d_AMPA )) ;
 DA_NMDA = DA_NMDA  / (1. - dt*( ( - 1.0 ) / tau_r_NMDA )) ;
 DB_NMDA = DB_NMDA  / (1. - dt*( ( - 1.0 ) / tau_d_NMDA )) ;
 DR = DR  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_rec )) ;
 DUse = DUse  / (1. - dt*( ( - 1.0 ) / tau_facil )) ;
  return 0;
}
 /*END CVODE*/
 static int odes (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
    A_AMPA = A_AMPA + (1. - exp(dt*(( - 1.0 ) / tau_r_AMPA)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_r_AMPA ) - A_AMPA) ;
    B_AMPA = B_AMPA + (1. - exp(dt*(( - 1.0 ) / tau_d_AMPA)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_d_AMPA ) - B_AMPA) ;
    A_NMDA = A_NMDA + (1. - exp(dt*(( - 1.0 ) / tau_r_NMDA)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_r_NMDA ) - A_NMDA) ;
    B_NMDA = B_NMDA + (1. - exp(dt*(( - 1.0 ) / tau_d_NMDA)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_d_NMDA ) - B_NMDA) ;
    R = R + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_rec)))*(- ( ( ( 1.0 ) ) / tau_rec ) / ( ( ( ( - 1.0 ) ) ) / tau_rec ) - R) ;
    Use = Use + (1. - exp(dt*(( - 1.0 ) / tau_facil)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau_facil ) - Use) ;
   }
  return 0;
}
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{  double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _thread = (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
   double _lA , _lresult , _lPsurv ;
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = Use;
    double __primary = (Use + U1 * ( 1.0 - Use )) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_facil ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_facil ) - __primary );
    Use += __primary;
  } else {
 Use = Use + U1 * ( 1.0 - Use ) ;
     }
 if ( R  == 0.0 ) {
     _lPsurv = exp ( - ( t - _args[1] ) / tau_rec ) ;
     _lresult = urand ( _threadargs_ ) ;
     if ( _lresult > _lPsurv ) {
         if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = R;
    double __primary = (1.0) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( ( - 1.0 ) ) ) / tau_rec ) ) )*( - ( ( ( 1.0 ) ) / tau_rec ) / ( ( ( ( - 1.0 ) ) ) / tau_rec ) - __primary );
    R += __primary;
  } else {
 R = 1.0 ;
         }
 }
     else {
       _args[1] = t ;
       }
     }
   if ( R  == 1.0 ) {
     _lresult = urand ( _threadargs_ ) ;
     if ( _lresult < Use ) {
       _args[1] = t ;
         if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = R;
    double __primary = (0.0) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( ( ( - 1.0 ) ) ) / tau_rec ) ) )*( - ( ( ( 1.0 ) ) / tau_rec ) / ( ( ( ( - 1.0 ) ) ) / tau_rec ) - __primary );
    R += __primary;
  } else {
 R = 0.0 ;
         }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A_AMPA;
    double __primary = (A_AMPA + _args[0] * factor_AMPA) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_r_AMPA ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_r_AMPA ) - __primary );
    A_AMPA += __primary;
  } else {
 A_AMPA = A_AMPA + _args[0] * factor_AMPA ;
         }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B_AMPA;
    double __primary = (B_AMPA + _args[0] * factor_AMPA) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_d_AMPA ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_d_AMPA ) - __primary );
    B_AMPA += __primary;
  } else {
 B_AMPA = B_AMPA + _args[0] * factor_AMPA ;
         }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = A_NMDA;
    double __primary = (A_NMDA + _args[0] * factor_NMDA) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_r_NMDA ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_r_NMDA ) - __primary );
    A_NMDA += __primary;
  } else {
 A_NMDA = A_NMDA + _args[0] * factor_NMDA ;
         }
   if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for cnexp case (rate uses no local variable) */
    double __state = B_NMDA;
    double __primary = (B_NMDA + _args[0] * factor_NMDA) - __state;
     __primary += ( 1. - exp( 0.5*dt*( ( - 1.0 ) / tau_d_NMDA ) ) )*( - ( 0.0 ) / ( ( - 1.0 ) / tau_d_NMDA ) - __primary );
    B_NMDA += __primary;
  } else {
 B_NMDA = B_NMDA + _args[0] * factor_NMDA ;
         }
 }
     }
   } }
 
static void _net_init(Point_process* _pnt, double* _args, double _lflag) {
       double* _p = _pnt->_prop->param;
    Datum* _ppvar = _pnt->_prop->dparam;
    Datum* _thread = (Datum*)0;
    _NrnThread* _nt = (_NrnThread*)_pnt->_vnt;
 _args[1] = t ;
   }
 
static int  setRNG ( _threadargsproto_ ) {
   
/*VERBATIM*/
    {
        /**
         * This function takes a NEURON Random object declared in hoc and makes it usable by this mod file.
         * Note that this method is taken from Brett paper as used by netstim.hoc and netstim.mod
         * which points out that the Random must be in uniform(1) mode
         */
        void** pv = (void**)(&_p_rng);
        if( ifarg(1)) {
            *pv = nrn_random_arg(1);
        } else {
            *pv = (void*)0;
        }
    }
  return 0; }
 
static double _hoc_setRNG(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 setRNG ( _p, _ppvar, _thread, _nt );
 return(_r);
}
 
double urand ( _threadargsproto_ ) {
   double _lurand;
 
/*VERBATIM*/
        double value;
        if (_p_rng) {
                /*
                :Supports separate independent but reproducible streams for
                : each instance. However, the corresponding hoc Random
                : distribution MUST be set to Random.uniform(0,1)
                */
                value = nrn_random_pick(_p_rng);
                //printf("random stream for this simulation = %lf\n",value);
                return value;
        }else{
 value = scop_random ( 1.0 ) ;
   
/*VERBATIM*/
        }
 _lurand = value ;
   
return _lurand;
 }
 
static double _hoc_urand(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  urand ( _p, _ppvar, _thread, _nt );
 return(_r);
}
 
static int _ode_count(int _type){ return 6;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 6; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 
static void _thread_mem_init(Datum* _thread) {
  if (_thread1data_inuse) {_thread[_gth]._pval = (double*)ecalloc(1, sizeof(double));
 }else{
 _thread[_gth]._pval = _thread1data; _thread1data_inuse = 1;
 }
 }
 
static void _thread_cleanup(Datum* _thread) {
  if (_thread[_gth]._pval == _thread1data) {
   _thread1data_inuse = 0;
  }else{
   free((void*)_thread[_gth]._pval);
  }
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  A_NMDA = A_NMDA0;
  A_AMPA = A_AMPA0;
  B_NMDA = B_NMDA0;
  B_AMPA = B_AMPA0;
  R = R0;
  Use = Use0;
 {
   double _ltp_AMPA , _ltp_NMDA ;
 A_AMPA = 0.0 ;
   B_AMPA = 0.0 ;
   A_NMDA = 0.0 ;
   B_NMDA = 0.0 ;
   _ltp_AMPA = ( tau_r_AMPA * tau_d_AMPA ) / ( tau_d_AMPA - tau_r_AMPA ) * log ( tau_d_AMPA / tau_r_AMPA ) ;
   _ltp_NMDA = ( tau_r_NMDA * tau_d_NMDA ) / ( tau_d_NMDA - tau_r_NMDA ) * log ( tau_d_NMDA / tau_r_NMDA ) ;
   factor_AMPA = - exp ( - _ltp_AMPA / tau_r_AMPA ) + exp ( - _ltp_AMPA / tau_d_AMPA ) ;
   factor_AMPA = 1.0 / factor_AMPA ;
   factor_NMDA = - exp ( - _ltp_NMDA / tau_r_NMDA ) + exp ( - _ltp_NMDA / tau_d_NMDA ) ;
   factor_NMDA = 1.0 / factor_NMDA ;
   R = 1.0 ;
   Use = 0.0 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   g_AMPA = gmax_AMPA * ( B_AMPA - A_AMPA ) ;
   mggate = 1.0 / ( 1.0 + exp ( 0.062 * - ( v ) ) * ( mg / 3.57 ) ) ;
   g_NMDA = mggate * gmax_NMDA * ( B_NMDA - A_NMDA ) ;
   g = g_AMPA + g_NMDA ;
   i_AMPA = g_AMPA * ( v - e ) ;
   i_NMDA = g_NMDA * ( v - e ) ;
   i = i_AMPA + i_NMDA ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   odes(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(A_AMPA) - _p;  _dlist1[0] = &(DA_AMPA) - _p;
 _slist1[1] = &(B_AMPA) - _p;  _dlist1[1] = &(DB_AMPA) - _p;
 _slist1[2] = &(A_NMDA) - _p;  _dlist1[2] = &(DA_NMDA) - _p;
 _slist1[3] = &(B_NMDA) - _p;  _dlist1[3] = &(DB_NMDA) - _p;
 _slist1[4] = &(R) - _p;  _dlist1[4] = &(DR) - _p;
 _slist1[5] = &(Use) - _p;  _dlist1[5] = &(DUse) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/Users/martinvielvoye/Simulation-Neuroscience/Week 4/MOOC-neurons-and-synapses-2017/ModelingSynapses/mechanisms/StochasticTsodyksMarkram_AMPA_NMDA.mod";
static const char* nmodl_file_text = 
  "COMMENT\n"
  "/**\n"
  " * @file TsodyksMarkram_AMPA_NMDA.mod\n"
  " * @brief An AMPA and NMDA glutamate receptor model with Tsodyks-Markram dynamics of the releasible pool\n"
  " * @author emuller\n"
  " * @date 2017-05-11\n"
  " * @remark Copyright \n"
  "\n"
  " BBP/EPFL 2005-2014; All rights reserved. \n"
  " */\n"
  "ENDCOMMENT\n"
  "\n"
  "\n"
  "TITLE AMPA and NMDA glutamate receptor with Stochastic Tsodyks-Markram dynamics\n"
  "\n"
  "COMMENT\n"
  "AMPA and NMDA glutamate receptor conductance using a dual-exponential profile\n"
  "and with stochastic Tsodyks-Markram dynamics of the releasible pool.\n"
  "\n"
  "This new model is based on Fuhrmann et al. 2002, and has the following properties:\n"
  "\n"
  "1) No consumption on failure.  \n"
  "2) No release just after release until recovery.\n"
  "3) Same ensemble averaged trace as deterministic/canonical Tsodyks-Markram \n"
  "   using same parameters determined from experiment.\n"
  "\n"
  "The synapse is implemented as a uni-vesicular (generalization to\n"
  "multi-vesicular should be straight-forward) 2-state Markov process.\n"
  "The states are {1=recovered, 0=unrecovered}.\n"
  "\n"
  "For a pre-synaptic spike or external spontaneous release trigger\n"
  "event, the synapse will only release if it is in the recovered state,\n"
  "and with probability Use (which follows facilitation dynamics).  If it\n"
  "releases, it will transition to the unrecovered state.  Recovery is as\n"
  "a Poisson process with rate 1/tau_rec.\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  ": Definition of variables which may be accesses by the user \n"
  "NEURON {\n"
  "    THREADSAFE\n"
  "\n"
  "    POINT_PROCESS StochasticTsodyksMarkram_AMPA_NMDA\n"
  "    RANGE tau_r_AMPA, tau_d_AMPA\n"
  "    RANGE tau_r_NMDA, tau_d_NMDA\n"
  "    RANGE mg, gmax_AMPA, gmax_NMDA\n"
  "    RANGE i, i_AMPA, i_NMDA, g_AMPA, g_NMDA, g, e\n"
  "    RANGE tau_rec, tau_facil, U1\n"
  "    NONSPECIFIC_CURRENT i\n"
  "\n"
  "    : For user defined random number generator\n"
  "    POINTER rng\n"
  "}\n"
  "\n"
  ": Definition of constants which may be set by the user\n"
  "PARAMETER {\n"
  "\n"
  "    tau_r_AMPA = 0.2   (ms)  : Dual-exponential conductance profile\n"
  "    tau_d_AMPA = 1.7   (ms)  : IMPORTANT: tau_r < tau_d\n"
  "    tau_r_NMDA = 0.29  (ms)  : Dual-exponential conductance profile\n"
  "    tau_d_NMDA = 43    (ms)  : IMPORTANT: tau_r < tau_d\n"
  "\n"
  "    e = 0              (mV)  : AMPA and NMDA reversal potential\n"
  "    mg = 1             (mM)  : Initial concentration of mg2+\n"
  "    mggate\n"
  "    \n"
  "    gmax_AMPA = .001   (uS)  : Weight conversion factor (from nS to uS)\n"
  "    gmax_NMDA = .001   (uS)  : Weight conversion factor (from nS to uS)\n"
  "\n"
  "    : Parameters for Tsodyks-Markram (TM) model of vesicle pool dynamics\n"
  "    tau_rec = 600      (ms)  : time constant of vesicle pool recovery\n"
  "    tau_facil = 10     (ms)  : time constant of release facilitation relaxation\n"
  "    U1 = 0.5           (1)   : release probability in the absence of facilitation\n"
  "\n"
  "}\n"
  "\n"
  ": Declaration of state variables \n"
  "STATE {\n"
  "\n"
  "    A_AMPA       : AMPA state variable to construct the dual-exponential profile\n"
  "                 : rise kinetics with tau_r_AMPA\n"
  "\n"
  "    B_AMPA       : AMPA state variable to construct the dual-exponential profile\n"
  "                 : decay kinetics with tau_d_AMPA\n"
  "\n"
  "    A_NMDA       : NMDA state variable to construct the dual-exponential profile\n"
  "                 : rise kinetics with tau_r_NMDA\n"
  "\n"
  "    B_NMDA       : NMDA state variable to construct the dual-exponential profile\n"
  "                 : decay kinetics with tau_d_NMDA\n"
  "\n"
  "    : State variables for TM model\n"
  "    R            : running fraction of vesicle pool\n"
  "    Use          : running release probability\n"
  "\n"
  "}\n"
  "\n"
  ": This is \"inline\" code in the C programming language \n"
  ": needed to manage user defined random number generators\n"
  "VERBATIM\n"
  "\n"
  "#include<stdlib.h>\n"
  "#include<stdio.h>\n"
  "#include<math.h>\n"
  "\n"
  "double nrn_random_pick(void* r);\n"
  "void* nrn_random_arg(int argpos);\n"
  "\n"
  "ENDVERBATIM\n"
  "\n"
  "\n"
  ": Declaration of variables that are computed, e.g. in the BREAKPOINT block\n"
  "ASSIGNED {\n"
  "    v (mV)\n"
  "    i (nA)\n"
  "    i_AMPA (nA)\n"
  "    i_NMDA (nA)\n"
  "    g_AMPA (uS)\n"
  "    g_NMDA (uS)\n"
  "    g (uS)\n"
  "    factor_AMPA\n"
  "    factor_NMDA\n"
  "    rng\n"
  "}\n"
  "\n"
  ": Definition of initial conditions\n"
  "INITIAL{\n"
  "    LOCAL tp_AMPA, tp_NMDA     : Declaration of some local variables\n"
  "\n"
  "    : Zero receptor rise and fall kinetics variables\n"
  "    A_AMPA = 0\n"
  "    B_AMPA = 0\n"
  "\n"
  "    A_NMDA = 0\n"
  "    B_NMDA = 0\n"
  "\n"
  "    : Compute constants needed to normalize the dual-exponential receptor dynamics\n"
  "\n"
  "    : Expression for time to peak of the AMPA dual-exponential conductance\n"
  "    tp_AMPA = (tau_r_AMPA*tau_d_AMPA)/(tau_d_AMPA-tau_r_AMPA)*log(tau_d_AMPA/tau_r_AMPA)\n"
  "    : Expression for time to peak of the NMDA dual-exponential conductance\n"
  "    tp_NMDA = (tau_r_NMDA*tau_d_NMDA)/(tau_d_NMDA-tau_r_NMDA)*log(tau_d_NMDA/tau_r_NMDA)\n"
  "\n"
  "    : AMPA Normalization factor - so that when t = tp_AMPA, gsyn = gpeak\n"
  "    factor_AMPA = -exp(-tp_AMPA/tau_r_AMPA)+exp(-tp_AMPA/tau_d_AMPA) \n"
  "    factor_AMPA = 1/factor_AMPA\n"
  "    : NMDA Normalization factor - so that when t = tp_NMDA, gsyn = gpeak\n"
  "    factor_NMDA = -exp(-tp_NMDA/tau_r_NMDA)+exp(-tp_NMDA/tau_d_NMDA) \n"
  "    factor_NMDA = 1/factor_NMDA\n"
  "\n"
  "    R = 1\n"
  "    Use = 0\n"
  "\n"
  "}\n"
  "\n"
  ": Declare method to propagate the state variables in time\n"
  "BREAKPOINT {\n"
  "\n"
  "    : Specify to solve system of equations \"odes\", declared below (DERIVATIVE block)\n"
  "    : \"cnexp\" specifies the intergration method, it is\n"
  "    : an implicit integration method that is stable even for stiff systems\n"
  "    SOLVE odes METHOD cnexp\n"
  "\n"
  "    : Compute and assign quantities which depend on the state variables\n"
  "\n"
  "    : Compute the time varying AMPA receptor conductance as \n"
  "    : the difference of state variables B_AMPA and A_AMPA\n"
  "    g_AMPA = gmax_AMPA*(B_AMPA-A_AMPA) \n"
  "\n"
  "    : NMDA is similar, but with a Magnesium block term: mggate\n"
  "    : Magneisum block kinetics due to Jahr & Stevens 1990\n"
  "    mggate = 1 / (1 + exp(0.062 (/mV) * -(v)) * (mg / 3.57 (mM))) \n"
  "    g_NMDA = mggate*gmax_NMDA*(B_NMDA-A_NMDA)  \n"
  "\n"
  "    : Total conductance\n"
  "    g = g_AMPA + g_NMDA\n"
  "\n"
  "    : Compute the AMPA and NMDA specific currents\n"
  "    i_AMPA = g_AMPA*(v-e) \n"
  "    i_NMDA = g_NMDA*(v-e) \n"
  "\n"
  "    : Compute the total current\n"
  "    i = i_AMPA + i_NMDA\n"
  "}\n"
  "\n"
  ": Declaration of ODEs solved for in the BREAKPOINT block\n"
  "DERIVATIVE odes {\n"
  "    A_AMPA' = -A_AMPA/tau_r_AMPA\n"
  "    B_AMPA' = -B_AMPA/tau_d_AMPA\n"
  "    A_NMDA' = -A_NMDA/tau_r_NMDA\n"
  "    B_NMDA' = -B_NMDA/tau_d_NMDA\n"
  "    \n"
  "    R' = (1-R)/tau_rec\n"
  "    Use' = -Use/tau_facil\n"
  "}\n"
  "\n"
  ": Block to be executed for a pre-synaptic spike event\n"
  "NET_RECEIVE (weight, tsyn (ms)) {\n"
  "    LOCAL A, result, Psurv\n"
  "\n"
  "    INITIAL{\n"
  "        tsyn=t\n"
  "    }\n"
  "\n"
  "    Use = Use + U1*(1-Use) : Update of release probability \n"
  "\n"
  "    if (R == 0) {\n"
  "        : probability of survival of unrecovered state based on Poisson recovery with rate 1/tau_rec\n"
  "        Psurv = exp(-(t-tsyn)/tau_rec)\n"
  "        result = urand()\n"
  "        if (result>Psurv) {\n"
  "            : recover      \n"
  "            R = 1     \n"
  "        }\n"
  "        else {\n"
  "            : probability of survival must now be from this interval\n"
  "	    tsyn = t\n"
  "        }\n"
  "    }	   \n"
  "	   \n"
  "    if (R == 1) {\n"
  "        result = urand()\n"
  "        if (result<Use) {\n"
  "	    : release!\n"
  "   	    tsyn = t\n"
  "	    R = 0\n"
  "            A_AMPA = A_AMPA + weight*factor_AMPA\n"
  "            B_AMPA = B_AMPA + weight*factor_AMPA\n"
  "            A_NMDA = A_NMDA + weight*factor_NMDA\n"
  "            B_NMDA = B_NMDA + weight*factor_NMDA\n"
  "        }\n"
  "\n"
  "    }\n"
  "\n"
  "}\n"
  "\n"
  ": Black magic to support use defined random number generators\n"
  ": No user serviceable code below this point\n"
  "PROCEDURE setRNG() {\n"
  "VERBATIM\n"
  "    {\n"
  "        /**\n"
  "         * This function takes a NEURON Random object declared in hoc and makes it usable by this mod file.\n"
  "         * Note that this method is taken from Brett paper as used by netstim.hoc and netstim.mod\n"
  "         * which points out that the Random must be in uniform(1) mode\n"
  "         */\n"
  "        void** pv = (void**)(&_p_rng);\n"
  "        if( ifarg(1)) {\n"
  "            *pv = nrn_random_arg(1);\n"
  "        } else {\n"
  "            *pv = (void*)0;\n"
  "        }\n"
  "    }\n"
  "ENDVERBATIM\n"
  "}\n"
  "\n"
  "FUNCTION urand() {\n"
  "VERBATIM\n"
  "        double value;\n"
  "        if (_p_rng) {\n"
  "                /*\n"
  "                :Supports separate independent but reproducible streams for\n"
  "                : each instance. However, the corresponding hoc Random\n"
  "                : distribution MUST be set to Random.uniform(0,1)\n"
  "                */\n"
  "                value = nrn_random_pick(_p_rng);\n"
  "                //printf(\"random stream for this simulation = %lf\\n\",value);\n"
  "                return value;\n"
  "        }else{\n"
  "ENDVERBATIM\n"
  "                : the old standby. Cannot use if reproducible parallel sim\n"
  "                : independent of nhost or which host this instance is on\n"
  "                : is desired, since each instance on this cpu draws from\n"
  "                : the same stream\n"
  "                value = scop_random(1)\n"
  "VERBATIM\n"
  "        }\n"
  "ENDVERBATIM\n"
  "        urand = value\n"
  "}\n"
  "\n"
  "\n"
  "\n"
  ;
#endif
