#ifndef __YQg0KPp9US1iRXoVC2Rf7F_h__
#define __YQg0KPp9US1iRXoVC2Rf7F_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_simstruct_bridge.h"
#include "sl_sfcn_cov/sl_sfcn_cov_bridge.h"

/* Type Definitions */
#ifndef typedef_InstanceStruct_YQg0KPp9US1iRXoVC2Rf7F
#define typedef_InstanceStruct_YQg0KPp9US1iRXoVC2Rf7F

typedef struct {
  SimStruct *S;
  PyObject *namespaceDict;
  PyGILState_STATE GIL;
  void *emlrtRootTLSGlobal;
  real_T *b_y0;
  real_T *b_y1;
} InstanceStruct_YQg0KPp9US1iRXoVC2Rf7F;

#endif                                 /* typedef_InstanceStruct_YQg0KPp9US1iRXoVC2Rf7F */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_YQg0KPp9US1iRXoVC2Rf7F(SimStruct *S, int_T method,
  void* data);

#endif
