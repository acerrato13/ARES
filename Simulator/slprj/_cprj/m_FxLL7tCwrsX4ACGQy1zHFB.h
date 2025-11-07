#ifndef __FxLL7tCwrsX4ACGQy1zHFB_h__
#define __FxLL7tCwrsX4ACGQy1zHFB_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_simstruct_bridge.h"
#include "sl_sfcn_cov/sl_sfcn_cov_bridge.h"

/* Type Definitions */
#ifndef typedef_InstanceStruct_FxLL7tCwrsX4ACGQy1zHFB
#define typedef_InstanceStruct_FxLL7tCwrsX4ACGQy1zHFB

typedef struct {
  SimStruct *S;
  PyObject *namespaceDict;
  PyGILState_STATE GIL;
  void *emlrtRootTLSGlobal;
  real_T *b_y0;
  real_T *b_y1;
} InstanceStruct_FxLL7tCwrsX4ACGQy1zHFB;

#endif                                 /* typedef_InstanceStruct_FxLL7tCwrsX4ACGQy1zHFB */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_FxLL7tCwrsX4ACGQy1zHFB(SimStruct *S, int_T method,
  void* data);

#endif
