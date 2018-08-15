//============ Copyright (c) Valve Corporation, All rights reserved. ============

#define WIN32_LEAN_AND_MEAN

#include <openvr_driver.h>
#include "driverlog.h"
#include <vector>
#include <thread>
#include <chrono>
#include <string>
#include <mutex>
#include <atomic>
#include <sstream>
#include <SDKDDKVer.h>
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include <sixense.h>
#include <sixense_math.hpp>
#include <sixense_utils/derivatives.hpp>

#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h> // for timeBeginPeriod()
#pragma comment(lib, "winmm.lib")
#endif

using namespace vr;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

// keys for use with the settings API
static const char * const k_pch_Hydra_Section = "hydra";
static const char * const k_pch_Hydra_RenderModel_String = "rendermodel";
static const char * const k_pch_Hydra_EnableIMU_Bool = "enableimu";
static const char * const k_pch_Hydra_EnableDeveloperMode_Bool = "enabledevelopermode";
static const char * const k_pch_Hydra_JoystickDeadzone_Float = "joystickdeadzone";

//-----------------------------------------------------------------------------
// Purpose: Watchdog
//-----------------------------------------------------------------------------

class CWatchdogDriver_Hydra : public IVRWatchdogProvider
{
public:
    CWatchdogDriver_Hydra()
    {
        m_pWatchdogThread = nullptr;
    }

    virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext);
    virtual void Cleanup();

private:
    std::thread *m_pWatchdogThread;
};

CWatchdogDriver_Hydra g_watchdogDriverHydra;

bool g_bExiting = false;

void WatchdogThreadFunction()
{
    while (!g_bExiting)
    {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        vr::VRWatchdogHost()->WatchdogWakeUp();
    }
}

EVRInitError CWatchdogDriver_Hydra::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());

    // Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to 
    // be pressed. A real driver should wait for a system button event or something else from the 
    // the hardware that signals that the VR system should start up.
    g_bExiting = false;
    m_pWatchdogThread = new std::thread(WatchdogThreadFunction);
    if (!m_pWatchdogThread)
    {
        DriverLog("Unable to create watchdog thread\n");
        return VRInitError_Driver_Failed;
    }

    return VRInitError_None;
}

void CWatchdogDriver_Hydra::Cleanup()
{
    g_bExiting = true;
    if (m_pWatchdogThread)
    {
        m_pWatchdogThread->join();
        delete m_pWatchdogThread;
        m_pWatchdogThread = nullptr;
    }

    CleanupDriverLog();
}

//-----------------------------------------------------------------------------
// Purpose: Controller Driver
//-----------------------------------------------------------------------------
class CHydraControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
    CHydraControllerDriver()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

        m_sSerialNumber = "CTRL_1234";
        m_sModelNumber = "MyController";
    }

    virtual ~CHydraControllerDriver()
    {
    }


    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

        // avoid "not fullscreen" warnings from vrmonitor
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

        // our hydra device isn't actually tracked, so set this property to avoid having the icon blink in the status window
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_NeverTracked_Bool, true);

        // even though we won't ever track we want to pretend to be the right hand so binding will work as expected
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_RightHand);

        // this file tells the UI what to show the user for binding this controller as well as what default bindings should
        // be for legacy or other apps
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{hydra}/input/mycontroller_profile.json");

        // create all the input components
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/a/click", &m_compA);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/b/click", &m_compB);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/c/click", &m_compC);

        // create our haptic component
        vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

        return VRInitError_None;
    }

    virtual void Deactivate()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    virtual void EnterStandby()
    {
    }

    void *GetComponent(const char *pchComponentNameAndVersion)
    {
        // override this to add a component to a driver
        return NULL;
    }

    virtual void PowerOff()
    {
    }

    /** debug request from a client */
    virtual void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
    }

    virtual DriverPose_t GetPose()
    {
        DriverPose_t pose = { 0 };
        pose.poseIsValid = false;
        pose.result = TrackingResult_Calibrating_OutOfRange;
        pose.deviceIsConnected = true;

        //pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        //pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        return pose;
    }


    void RunFrame()
    {
#if defined( _WINDOWS )
        // Your driver would read whatever hardware state is associated with its input components and pass that
        // in to UpdateBooleanComponent. This could happen in RunFrame or on a thread of your own that's reading USB
        // state. There's no need to update input state unless it changes, but it doesn't do any harm to do so.

        vr::VRDriverInput()->UpdateBooleanComponent(m_compA, (0x8000 & GetAsyncKeyState('A')) != 0, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compB, (0x8000 & GetAsyncKeyState('B')) != 0, 0);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compC, (0x8000 & GetAsyncKeyState('C')) != 0, 0);
#endif
    }

    void ProcessEvent(const vr::VREvent_t & vrEvent)
    {
        switch (vrEvent.eventType)
        {
        case vr::VREvent_Input_HapticVibration:
        {
            if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
            {
                // This is where you would send a signal to your hardware to trigger actual haptic feedback
                DriverLog("BUZZ!\n");
            }
        }
        break;
        }
    }


    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    vr::VRInputComponentHandle_t m_compA;
    vr::VRInputComponentHandle_t m_compB;
    vr::VRInputComponentHandle_t m_compC;
    vr::VRInputComponentHandle_t m_compHaptic;

    std::string m_sSerialNumber;
    std::string m_sModelNumber;


};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_Hydra : public IServerTrackedDeviceProvider
{
public:
    virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext);
    virtual void Cleanup();
    virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame();
    virtual bool ShouldBlockStandbyMode() { return false; }
    virtual void EnterStandby() {}
    virtual void LeaveStandby() {}

private:
    CHydraControllerDriver *m_pController = nullptr;
};

CServerDriver_Hydra g_serverDriverHydra;


EVRInitError CServerDriver_Hydra::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());

    m_pController = new CHydraControllerDriver();
    vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);

    return VRInitError_None;
}

void CServerDriver_Hydra::Cleanup()
{
    CleanupDriverLog();
    delete m_pController;
    m_pController = NULL;
}


void CServerDriver_Hydra::RunFrame()
{
    if (m_pController)
    {
        m_pController->RunFrame();
    }

    vr::VREvent_t vrEvent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
    {
        if (m_pController)
        {
            m_pController->ProcessEvent(vrEvent);
        }
    }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriverHydra;
    }
    if (0 == strcmp(IVRWatchdogProvider_Version, pInterfaceName))
    {
        return &g_watchdogDriverHydra;
    }

    if (pReturnCode)
        *pReturnCode = VRInitError_Init_InterfaceNotFound;

    return NULL;
}
