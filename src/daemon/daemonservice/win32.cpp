#include <windows.h>

extern "C" {
void ServiceMain(int argc, char** argv);
void ControlHandler(DWORD request);
}

int startService() 
{ 
    SERVICE_TABLE_ENTRY ServiceTable[2];
    ServiceTable[0].lpServiceName = "baromeshd";
    ServiceTable[0].lpServiceProc = (LPSERVICE_MAIN_FUNCTION)ServiceMain;

    ServiceTable[1].lpServiceName = NULL;
    ServiceTable[1].lpServiceProc = NULL;
    // Start the control dispatcher thread for our service
    StartServiceCtrlDispatcher(ServiceTable);  
}

void ServiceMain(int argc, char** argv) 
{ 
    int error; 
 
    ServiceStatus.dwServiceType        = SERVICE_WIN32; 
    ServiceStatus.dwCurrentState       = SERVICE_START_PENDING; 
    ServiceStatus.dwControlsAccepted   = SERVICE_ACCEPT_STOP | SERVICE_ACCEPT_SHUTDOWN;
    ServiceStatus.dwWin32ExitCode      = 0; 
    ServiceStatus.dwServiceSpecificExitCode = 0; 
    ServiceStatus.dwCheckPoint         = 0; 
    ServiceStatus.dwWaitHint           = 0; 
 
    hStatus = RegisterServiceCtrlHandler(
		"baromeshd", 
		(LPHANDLER_FUNCTION)ControlHandler); 
    if (hStatus == (SERVICE_STATUS_HANDLE)0) 
    { 
        // Registering Control Handler failed
        return; 
    }  

    // We report the running status to SCM. 
    ServiceStatus.dwCurrentState = SERVICE_RUNNING; 
    SetServiceStatus (hStatus, &ServiceStatus);
 
    MEMORYSTATUS memory;
    // The worker loop of a service
    while (ServiceStatus.dwCurrentState == SERVICE_RUNNING)
	{
		char buffer[16];
		GlobalMemoryStatus(&memory);
		sprintf(buffer, "%d", memory.dwAvailPhys);
		int result = WriteToLog(buffer);
		if (result)
		{
			ServiceStatus.dwCurrentState       = SERVICE_STOPPED; 
			ServiceStatus.dwWin32ExitCode      = -1; 
			SetServiceStatus(hStatus, &ServiceStatus);
			return;
		}

		Sleep(SLEEP_TIME);
	}
    return; 
}
 
// Control handler function
void ControlHandler(DWORD request) 
{ 
    switch(request) 
    { 
        case SERVICE_CONTROL_STOP: 
            //WriteToLog("Monitoring stopped.");

            ServiceStatus.dwWin32ExitCode = 0; 
            ServiceStatus.dwCurrentState  = SERVICE_STOPPED; 
            SetServiceStatus (hStatus, &ServiceStatus);
            return; 
 
        case SERVICE_CONTROL_SHUTDOWN: 
            //WriteToLog("Monitoring stopped.");

            ServiceStatus.dwWin32ExitCode = 0; 
            ServiceStatus.dwCurrentState  = SERVICE_STOPPED; 
            SetServiceStatus (hStatus, &ServiceStatus);
            return; 
        
        default:
            break;
    } 
 
    // Report current status
    SetServiceStatus (hStatus,  &ServiceStatus);
 
    return; 
} 

