#include "rundongle.hpp"

#include <windows.h>

#include <csignal>
#include <cstdlib>

extern "C" {
void serviceMain (int argc, char** argv);
void ControlHandler (DWORD request);
}

int main (int argc, char** argv)
{
    // If there's a bug in the rest of the Windows service code, it's useful
    // to be able to start the daemon manually for testing.
    if (argc > 1) {
        return runDaemon();
    }
    SERVICE_TABLE_ENTRY serviceTable[2];
    serviceTable[0].lpServiceName = "baromeshd";
    serviceTable[0].lpServiceProc = (LPSERVICE_MAIN_FUNCTION)serviceMain;

    serviceTable[1].lpServiceName = NULL;
    serviceTable[1].lpServiceProc = NULL;
    // Start the control dispatcher thread for our service
    StartServiceCtrlDispatcher(serviceTable);
    return 0;
}

SERVICE_STATUS& serviceStatus () {
    static SERVICE_STATUS s;
    return s;
}

SERVICE_STATUS_HANDLE& serviceStatusHandle () {
    static SERVICE_STATUS_HANDLE h;
    return h;
}

void updateServiceStatus () {
    auto rc = SetServiceStatus(serviceStatusHandle(), &serviceStatus());
    if (!rc) {
        // Report an error somehow?
    }
}


void serviceMain (int argc, char** argv)
{
    serviceStatusHandle() = RegisterServiceCtrlHandler(
        "baromeshd",
        (LPHANDLER_FUNCTION)ControlHandler);
    if (serviceStatusHandle() == (SERVICE_STATUS_HANDLE)0)
    {
        // Registering Control Handler failed
        return;
    }

    // We report the running status to SCM. 
    serviceStatus().dwServiceType        = SERVICE_WIN32_OWN_PROCESS;
    serviceStatus().dwCurrentState       = SERVICE_RUNNING;
    serviceStatus().dwControlsAccepted   = SERVICE_ACCEPT_STOP | SERVICE_ACCEPT_SHUTDOWN;
    serviceStatus().dwWin32ExitCode      = 0;
    serviceStatus().dwServiceSpecificExitCode = 0;
    serviceStatus().dwCheckPoint         = 0;
    serviceStatus().dwWaitHint           = 0;
    updateServiceStatus();

    auto code = runDaemon();

    if (code) {
        serviceStatus().dwWin32ExitCode = ERROR_SERVICE_SPECIFIC_ERROR;
        serviceStatus().dwServiceSpecificExitCode = code;
    }
    else {
        serviceStatus().dwWin32ExitCode = 0;
        serviceStatus().dwServiceSpecificExitCode = 0;
    }

    serviceStatus().dwCurrentState = SERVICE_STOPPED;
    serviceStatus().dwCheckPoint = 0;
    serviceStatus().dwWaitHint = 0;
    updateServiceStatus();
}
 
// Control handler function
void ControlHandler(DWORD request) 
{ 
    switch(request) 
    { 
        case SERVICE_CONTROL_STOP:
            // fall-through
        case SERVICE_CONTROL_SHUTDOWN:
            serviceStatus().dwCurrentState  = SERVICE_STOP_PENDING;
            serviceStatus().dwWaitHint = 3000; // We should be stopped w/i 3 seconds
            serviceStatus().dwCheckPoint = 1;
            std::raise(SIGTERM);
            break;
        default:
            break;
    }
    updateServiceStatus();
} 

