#include "pythreadstatelock.h"


PyThreadStateLock::PyThreadStateLock()
{  
    gstate = PyGILState_Ensure();
}

PyThreadStateLock::~PyThreadStateLock()
{
    PyGILState_Release(gstate);
}
