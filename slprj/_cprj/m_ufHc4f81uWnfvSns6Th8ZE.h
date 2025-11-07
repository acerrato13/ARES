#ifndef __ufHc4f81uWnfvSns6Th8ZE_h__
#define __ufHc4f81uWnfvSns6Th8ZE_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_simstruct_bridge.h"
#include "sl_sfcn_cov/sl_sfcn_cov_bridge.h"

/* Type Definitions */
#ifndef typedef_InstanceStruct_ufHc4f81uWnfvSns6Th8ZE
#define typedef_InstanceStruct_ufHc4f81uWnfvSns6Th8ZE

typedef struct {
  SimStruct *S;
  PyObject *namespaceDict;
  PyGILState_STATE GIL;
  void *emlrtRootTLSGlobal;
  real_T *b_y0;
  real_T *b_y1;
} InstanceStruct_ufHc4f81uWnfvSns6Th8ZE;

#endif                                 /* typedef_InstanceStruct_ufHc4f81uWnfvSns6Th8ZE */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_ufHc4f81uWnfvSns6Th8ZE(SimStruct *S, int_T method,
  void* data);

#endif
