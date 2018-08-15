//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>

#include "driverlog.h"

#include <string>
#include <mutex>
#include <atomic>
#include <vector>
#include <thread>
#include <chrono>

// sixense sdk
#include <sixense.h>
#include <sixense_math.hpp>
#include <sixense_utils/derivatives.hpp>

#ifdef _WIN32
#include <mmsystem.h> // for timeBeginPeriod()
#pragma comment(lib, "winmm.lib")
#endif

#if defined( _WINDOWS )
#include <windows.h>
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
// Purpose:
//-----------------------------------------------------------------------------
class CHydraDeviceDriver : public vr::ITrackedDeviceServerDriver
{
public:
    CHydraDeviceDriver()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

        DriverLog("Using settings values\n");
        m_flIPD = vr::VRSettings()->GetFloat(k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float);

        char buf[1024];
        vr::VRSettings()->GetString(k_pch_Hydra_Section, k_pch_Hydra_SerialNumber_String, buf, sizeof(buf));
        m_sSerialNumber = buf;

        vr::VRSettings()->GetString(k_pch_Hydra_Section, k_pch_Hydra_ModelNumber_String, buf, sizeof(buf));
        m_sModelNumber = buf;

        m_nWindowX = vr::VRSettings()->GetInt32(k_pch_Hydra_Section, k_pch_Hydra_WindowX_Int32);
        m_nWindowY = vr::VRSettings()->GetInt32(k_pch_Hydra_Section, k_pch_Hydra_WindowY_Int32);
        m_nWindowWidth = vr::VRSettings()->GetInt32(k_pch_Hydra_Section, k_pch_Hydra_WindowWidth_Int32);
        m_nWindowHeight = vr::VRSettings()->GetInt32(k_pch_Hydra_Section, k_pch_Hydra_WindowHeight_Int32);
        m_nRenderWidth = vr::VRSettings()->GetInt32(k_pch_Hydra_Section, k_pch_Hydra_RenderWidth_Int32);
        m_nRenderHeight = vr::VRSettings()->GetInt32(k_pch_Hydra_Section, k_pch_Hydra_RenderHeight_Int32);
        m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(k_pch_Hydra_Section, k_pch_Hydra_SecondsFromVsyncToPhotons_Float);
        m_flDisplayFrequency = vr::VRSettings()->GetFloat(k_pch_Hydra_Section, k_pch_Hydra_DisplayFrequency_Float);

        DriverLog("driver_hydra: Serial Number: %s\n", m_sSerialNumber.c_str());
        DriverLog("driver_hydra: Model Number: %s\n", m_sModelNumber.c_str());
        DriverLog("driver_hydra: Window: %d %d %d %d\n", m_nWindowX, m_nWindowY, m_nWindowWidth, m_nWindowHeight);
        DriverLog("driver_hydra: Render Target: %d %d\n", m_nRenderWidth, m_nRenderHeight);
        DriverLog("driver_hydra: Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons);
        DriverLog("driver_hydra: Display Frequency: %f\n", m_flDisplayFrequency);
        DriverLog("driver_hydra: IPD: %f\n", m_flIPD);
    }

    virtual ~CHydraDeviceDriver()
    {
    }


    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);


        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD);
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f);
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency);
        vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);

        // return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

        // avoid "not fullscreen" warnings from vrmonitor
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

        // Icons can be configured in code or automatically configured by an external file "drivername\resources\driver.vrresources".
        // Icon properties NOT configured in code (post Activate) are then auto-configured by the optional presence of a driver's "drivername\resources\driver.vrresources".
        // In this manner a driver can configure their icons in a flexible data driven fashion by using an external file.
        //
        // The structure of the driver.vrresources file allows a driver to specialize their icons based on their HW.
        // Keys matching the value in "Prop_ModelNumber_String" are considered first, since the driver may have model specific icons.
        // An absence of a matching "Prop_ModelNumber_String" then considers the ETrackedDeviceClass ("HMD", "Controller", "GenericTracker", "TrackingReference")
        // since the driver may have specialized icons based on those device class names.
        //
        // An absence of either then falls back to the "system.vrresources" where generic device class icons are then supplied.
        //
        // Please refer to "bin\drivers\hydra\resources\driver.vrresources" which contains this hydra configuration.
        //
        // "Alias" is a reserved key and specifies chaining to another json block.
        //
        // In this hydra configuration file (overly complex FOR EXAMPLE PURPOSES ONLY)....
        //
        // "Model-v2.0" chains through the alias to "Model-v1.0" which chains through the alias to "Model-v Defaults".
        //
        // Keys NOT found in "Model-v2.0" would then chase through the "Alias" to be resolved in "Model-v1.0" and either resolve their or continue through the alias.
        // Thus "Prop_NamedIconPathDeviceAlertLow_String" in each model's block represent a specialization specific for that "model".
        // Keys in "Model-v Defaults" are an example of mapping to the same states, and here all map to "Prop_NamedIconPathDeviceOff_String".
        //
        bool bSetupIconUsingExternalResourceFile = true;
        if (!bSetupIconUsingExternalResourceFile)
        {
            // Setup properties directly in code.
            // Path values are of the form {drivername}\icons\some_icon_filename.png
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{hydra}/icons/headset_hydra_status_off.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{hydra}/icons/headset_hydra_status_searching.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{hydra}/icons/headset_hydra_status_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{hydra}/icons/headset_hydra_status_ready.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{hydra}/icons/headset_hydra_status_ready_alert.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{hydra}/icons/headset_hydra_status_error.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{hydra}/icons/headset_hydra_status_standby.png");
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{hydra}/icons/headset_hydra_status_ready_low.png");
        }

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

    virtual bool IsDisplayOnDesktop()
    {
        return true;
    }

    virtual bool IsDisplayRealDisplay()
    {
        return false;
    }

    virtual void GetRecommendedRenderTargetSize(uint32_t *pnWidth, uint32_t *pnHeight)
    {
        *pnWidth = m_nRenderWidth;
        *pnHeight = m_nRenderHeight;
    }

    virtual void GetEyeOutputViewport(EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight)
    {
        *pnY = 0;
        *pnWidth = m_nWindowWidth / 2;
        *pnHeight = m_nWindowHeight;

        if (eEye == Eye_Left)
        {
            *pnX = 0;
        }
        else
        {
            *pnX = m_nWindowWidth / 2;
        }
    }

    virtual void GetProjectionRaw(EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom)
    {
        *pfLeft = -1.0;
        *pfRight = 1.0;
        *pfTop = -1.0;
        *pfBottom = 1.0;
    }

    virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV)
    {
        DistortionCoordinates_t coordinates;
        coordinates.rfBlue[0] = fU;
        coordinates.rfBlue[1] = fV;
        coordinates.rfGreen[0] = fU;
        coordinates.rfGreen[1] = fV;
        coordinates.rfRed[0] = fU;
        coordinates.rfRed[1] = fV;
        return coordinates;
    }

    virtual DriverPose_t GetPose()
    {
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;

        //pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        //pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);


        return pose;
    }


    void RunFrame()
    {
        // In a real driver, this should happen from some pose tracking thread.
        // The RunFrame interval is unspecified and can be very irregular if some other
        // driver blocks it for some periodic task.
        if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
        {
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(), sizeof(DriverPose_t));
        }
    }

    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    std::string m_sSerialNumber;
    std::string m_sModelNumber;

    std::string m_strRenderModel;

    int32_t m_nWindowX;
    int32_t m_nWindowY;
    int32_t m_nWindowWidth;
    int32_t m_nWindowHeight;
    int32_t m_nRenderWidth;
    int32_t m_nRenderHeight;
    float m_flSecondsFromVsyncToPhotons;
    float m_flDisplayFrequency;
    float m_flIPD;
};

//-----------------------------------------------------------------------------
// Purpose:
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
    CHydraDeviceDriver *m_pHydraDeviceLatest = nullptr;
    CHydraControllerDriver *m_pController = nullptr;
};

CServerDriver_Hydra g_serverDriverHydra;


EVRInitError CServerDriver_Hydra::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());

    m_pHydraDeviceLatest = new CHydraDeviceDriver();
    vr::VRServerDriverHost()->TrackedDeviceAdded(m_pHydraDeviceLatest->GetSerialNumber().c_str(), vr::TrackedDeviceClass_HMD, m_pHydraDeviceLatest);

    m_pController = new CHydraControllerDriver();
    vr::VRServerDriverHost()->TrackedDeviceAdded(m_pController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pController);

    return VRInitError_None;
}

void CServerDriver_Hydra::Cleanup()
{
    CleanupDriverLog();
    delete m_pHydraDeviceLatest;
    m_pHydraDeviceLatest = NULL;
    delete m_pController;
    m_pController = NULL;
}


void CServerDriver_Hydra::RunFrame()
{
    if (m_pHydraDeviceLatest)
    {
        m_pHydraDeviceLatest->RunFrame();
    }
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
