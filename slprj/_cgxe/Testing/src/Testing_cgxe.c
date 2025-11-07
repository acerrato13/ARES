/* Include files */

#include "Testing_cgxe.h"
#include "m_YQg0KPp9US1iRXoVC2Rf7F.h"

unsigned int cgxe_Testing_method_dispatcher(SimStruct* S, int_T method, void
  * data)
{
  if (ssGetChecksum0(S) == 2352341616 &&
      ssGetChecksum1(S) == 631446264 &&
      ssGetChecksum2(S) == 3833566692 &&
      ssGetChecksum3(S) == 1960354682) {
    method_dispatcher_YQg0KPp9US1iRXoVC2Rf7F(S, method, data);
    return 1;
  }

  return 0;
}
