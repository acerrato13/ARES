/* Include files */

#include "Testing_cgxe.h"
#include "m_4tY3NfW2i3RLfdbLyGsWtD.h"

unsigned int cgxe_Testing_method_dispatcher(SimStruct* S, int_T method, void
  * data)
{
  if (ssGetChecksum0(S) == 3158694247 &&
      ssGetChecksum1(S) == 92789891 &&
      ssGetChecksum2(S) == 3407822544 &&
      ssGetChecksum3(S) == 956572768) {
    method_dispatcher_4tY3NfW2i3RLfdbLyGsWtD(S, method, data);
    return 1;
  }

  return 0;
}
