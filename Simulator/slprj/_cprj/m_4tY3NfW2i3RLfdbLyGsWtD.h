#ifndef __4tY3NfW2i3RLfdbLyGsWtD_h__
#define __4tY3NfW2i3RLfdbLyGsWtD_h__

/* Include files */
#include "simstruc.h"
#include "rtwtypes.h"
#include "multiword_types.h"
#include "slexec_vm_zc_functions.h"
#include "slexec_vm_simstruct_bridge.h"
#include "sl_sfcn_cov/sl_sfcn_cov_bridge.h"

/* Type Definitions */
#ifndef typedef_InstanceStruct_4tY3NfW2i3RLfdbLyGsWtD
#define typedef_InstanceStruct_4tY3NfW2i3RLfdbLyGsWtD

typedef struct {
  SimStruct *S;
  PyObject *namespaceDict;
  PyGILState_STATE GIL;
  void *emlrtRootTLSGlobal;
  real_T *b_y0;
  real_T *b_y1;
} InstanceStruct_4tY3NfW2i3RLfdbLyGsWtD;

#endif                                 /* typedef_InstanceStruct_4tY3NfW2i3RLfdbLyGsWtD */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
extern void method_dispatcher_4tY3NfW2i3RLfdbLyGsWtD(SimStruct *S, int_T method,
  void* data);

#endif
