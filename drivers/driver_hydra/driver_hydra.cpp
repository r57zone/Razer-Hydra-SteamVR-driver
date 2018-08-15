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

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
    HmdQuaternion_t quat;
    quat.w = w;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    return quat;
}

// keys for use with the settings API
static const char * const k_pch_Hydra_Section = "hydra";
static const char * const k_pch_Hydra_RenderModel_String = "rendermodel";
static const char * const k_pch_Hydra_EnableIMU_Bool = "enableimu";
static const char * const k_pch_Hydra_EnableDeveloperMode_Bool = "enabledevelopermode";
static const char * const k_pch_Hydra_JoystickDeadzone_Float = "joystickdeadzone";

static void GenerateSerialNumber(char *p, int psize, int base, int controller)
{
    _snprintf(p, psize, "hydra%d_controller%d", base, controller);
}

//-----------------------------------------------------------------------------
// Purpose: Watchdog
//-----------------------------------------------------------------------------
/*
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
*/

//-----------------------------------------------------------------------------
// Purpose: Controller Driver
//-----------------------------------------------------------------------------
class CHydraControllerDriver : public vr::ITrackedDeviceServerDriver
{
public:
    bool IsActivated() const
    {
        return m_unObjectId != vr::k_unTrackedDeviceIndexInvalid;
    }

    bool HasControllerId(int nBase, int nId)
    {
        return nBase == m_nBase && nId == m_nId;
    }

    /** Process sixenseControllerData.  Return true if it's new to help caller manage sleep durations */
    bool Update(sixenseControllerData & cd)
    {
        if (m_ucPoseSequenceNumber == cd.sequence_number || !IsActivated())
            return false;
        m_ucPoseSequenceNumber = cd.sequence_number;

        //UpdateTrackingState(cd); // TODO

        // Block all buttons until initial press confirms hemisphere
        if (WaitingForHemisphereTracking(cd))
            return true;

        DelaySystemButtonForChording(cd);

        //UpdateControllerState(cd); // TODO

        return true;
    }

    bool IsHoldingSystemButton() const
    {
        return m_eSystemButtonState == k_eWaiting;
    }

    void ConsumeSystemButtonPress()
    {
        if (m_eSystemButtonState == k_eWaiting)
        {
            m_eSystemButtonState = k_eBlocked;
        }
    }

    void DelaySystemButtonForChording(sixenseControllerData & cd)
    {
        // Delay sending system button to vrserver while we see if it is being
        // chorded with the other system button to reset the coordinate system
        if (cd.buttons & SIXENSE_BUTTON_START)
        {
            switch (m_eSystemButtonState)
            {
            case k_eIdle:
                m_eSystemButtonState = k_eWaiting;
                m_SystemButtonDelay = std::chrono::steady_clock::now() + k_SystemButtonChordingDelay;
                cd.buttons &= ~SIXENSE_BUTTON_START;
                break;

            case k_eWaiting:
                if (std::chrono::steady_clock::now() >= m_SystemButtonDelay)
                {
                    m_eSystemButtonState = k_eSent;
                    // leave button state set, will reach vrserver
                }
                else
                {
                    cd.buttons &= ~SIXENSE_BUTTON_START;
                }
                break;

            case k_eSent:
                // still held down, nothing to do
                break;

            case k_ePulsed:
                // user re-pressed within 1 frame, just ignore lift
                m_eSystemButtonState = k_eSent;
                break;

            case k_eBlocked:
                // was consumed by chording gesture -- never send until released
                cd.buttons &= ~SIXENSE_BUTTON_START;
                break;
            }
        }
        else
        {
            switch (m_eSystemButtonState)
            {
            case k_eIdle:
            case k_eSent:
            case k_eBlocked:
                m_eSystemButtonState = k_eIdle;
                break;

            case k_eWaiting:
                // user pressed and released the button within the timeout, so
                // send a quick pulse to the application
                m_eSystemButtonState = k_ePulsed;
                m_SystemButtonDelay = std::chrono::steady_clock::now() + k_SystemButtonPulsingDuration;
                cd.buttons |= SIXENSE_BUTTON_START;
                break;

            case k_ePulsed:
                // stretch fake pulse so client sees it
                if (std::chrono::steady_clock::now() >= m_SystemButtonDelay)
                {
                    m_eSystemButtonState = k_eIdle;
                }
                cd.buttons |= SIXENSE_BUTTON_START;
                break;
            }
        }
    }

    // Initially block all button presses, stealing the first one to mean
    // that the controller is pointing at the base and we should tell the Sixense SDK
    bool WaitingForHemisphereTracking(sixenseControllerData & cd)
    {
        switch (m_eHemisphereTrackingState)
        {
        case k_eHemisphereTrackingDisabled:
            if (cd.buttons || cd.trigger > 0.8f)
            {
                // First button press
                m_eHemisphereTrackingState = k_eHemisphereTrackingButtonDown;
            }
            return true;

        case k_eHemisphereTrackingButtonDown:
            if (!cd.buttons && cd.trigger < 0.1f)
            {
                // Buttons released (so they won't leak into application), go!
                sixenseAutoEnableHemisphereTracking(m_nId);
                m_eHemisphereTrackingState = k_eHemisphereTrackingEnabled;
            }
            return true;

        case k_eHemisphereTrackingEnabled:
        default:
            return false;
        }
    }

    // User initiated manual alignment of the coordinate system of driver_hydra with the HMD:
    //
    // The user has put two controllers on either side of her head, near the shoulders.  We
    // assume that the HMD is roughly in between them (so the exact distance apart doesn't
    // matter as long as the pose is symmetrical) and we align the HMD's coordinate system
    // using the line between the controllers (again, exact position is not important, only
    // symmetry).
    static void RealignCoordinates(CHydraControllerDriver * pHydraA, CHydraControllerDriver * pHydraB)
    {
        if (pHydraA->m_unObjectId == vr::k_unTrackedDeviceIndexInvalid)
            return;

        pHydraA->m_pAlignmentPartner = pHydraB;
        pHydraB->m_pAlignmentPartner = pHydraA;

        // Ask hydra_monitor to tell us HMD pose
        static vr::VREvent_Data_t nodata = { 0 };
        vr::VRServerDriverHost()->VendorSpecificEvent(pHydraA->m_unObjectId,
            (vr::EVREventType) (vr::VREvent_VendorSpecific_Reserved_Start + 0), nodata,
            -std::chrono::duration_cast<std::chrono::seconds>(k_SystemButtonChordingDelay).count());
    }


    CHydraControllerDriver(int base, int n)
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

        char buf[256];
        GenerateSerialNumber(buf, sizeof(buf), base, n);
        m_sSerialNumber = buf;

        m_sModelNumber = "Razer Hydra";
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

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        return pose;
    }


    void RunFrame()
    {
#if defined( _WIN32 )
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
            }
        }
        break;
        }
    }


    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;
    vr::ETrackedControllerRole m_eControllerRole;
    vr::VRInputComponentHandle_t m_compA;
    vr::VRInputComponentHandle_t m_compB;
    vr::VRInputComponentHandle_t m_compC;
    vr::VRInputComponentHandle_t m_compHaptic;

    std::string m_sSerialNumber;
    std::string m_sModelNumber;

    // Which Hydra controller
    int m_nBase;
    int m_nId;

    // Used to deduplicate state data from the sixense driver
    uint8_t m_ucPoseSequenceNumber;

    // Ancillary tracking state
    sixenseMath::Vector3 m_WorldFromDriverTranslation;
    sixenseMath::Quat m_WorldFromDriverRotation;
    sixenseUtils::Derivatives m_Deriv;
    enum { k_eHemisphereTrackingDisabled, k_eHemisphereTrackingButtonDown, k_eHemisphereTrackingEnabled } m_eHemisphereTrackingState;
    bool m_bCalibrated;

    // Other controller with from the last realignment
    CHydraControllerDriver *m_pAlignmentPartner;

    // Timeout for system button chording
    std::chrono::steady_clock::time_point m_SystemButtonDelay;
    enum { k_eIdle, k_eWaiting, k_eSent, k_ePulsed, k_eBlocked } m_eSystemButtonState;

    static const float k_fScaleSixenseToMeters;
    static const std::chrono::milliseconds k_SystemButtonChordingDelay;
    static const std::chrono::milliseconds k_SystemButtonPulsingDuration;

};

const std::chrono::milliseconds CHydraControllerDriver::k_SystemButtonChordingDelay(150);
const std::chrono::milliseconds CHydraControllerDriver::k_SystemButtonPulsingDuration(100);
const float CHydraControllerDriver::k_fScaleSixenseToMeters = 0.001;  // sixense driver in mm


//-----------------------------------------------------------------------------
// Purpose: IServerTrackedDeviceProvider
//-----------------------------------------------------------------------------
class CServerDriver_Hydra : public IServerTrackedDeviceProvider
{
public:
    CServerDriver_Hydra();
    virtual ~CServerDriver_Hydra();
    virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext);
    virtual void Cleanup();
    virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame();
    virtual bool ShouldBlockStandbyMode() { return false; }
    virtual void EnterStandby() {}
    virtual void LeaveStandby() {}

private:
    CHydraControllerDriver *m_pControllerA = nullptr;
    CHydraControllerDriver *m_pControllerB = nullptr;
    std::atomic<bool> m_bStopRequested;
    bool m_bLaunchedHydraMonitor;
    std::thread *m_Thread;
    std::recursive_mutex m_Mutex;
    typedef std::lock_guard<std::recursive_mutex> scope_lock;
    std::vector< CHydraControllerDriver * > m_vecControllers;
    static void ThreadEntry(CServerDriver_Hydra *pDriver);
    void ThreadFunc();
    void ScanForNewControllers(bool bNotifyServer);
    void CheckForChordedSystemButtons();
    virtual uint32_t GetTrackedDeviceCount();
    virtual vr::ITrackedDeviceServerDriver * FindTrackedDeviceDriver(const char * pchId);
};

CServerDriver_Hydra g_serverDriverHydra;

CServerDriver_Hydra::CServerDriver_Hydra():
    m_Thread(NULL), // initialize m_Thread
    m_bStopRequested(false),
    m_bLaunchedHydraMonitor(false)
{
}

CServerDriver_Hydra::~CServerDriver_Hydra()
{
    // 10/10/2015 benj:  vrserver is exiting without calling Cleanup() to balance Init()
    // causing std::thread to call std::terminate
    Cleanup();
}

EVRInitError CServerDriver_Hydra::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());

    if (sixenseInit() != SIXENSE_SUCCESS)
        return vr::VRInitError_Driver_Failed;

    // Will not immediately detect controllers at this point.  Sixense driver must be initializing
    // in its own thread...  It's okay to dynamically detect devices later, but if controllers are
    // the only devices (e.g. requireHmd=false) we must have GetTrackedDeviceCount() != 0 before returning.
    for (int i = 0; i < 20; ++i)
    {
        ScanForNewControllers(false);
        if (GetTrackedDeviceCount())
            break;
        Sleep(100);
    }

    m_Thread = new std::thread(ThreadEntry, this);   // use new operator now

    //m_pControllerA = new CHydraControllerDriver(0);
    //vr::VRServerDriverHost()->TrackedDeviceAdded(m_pControllerA->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pControllerA);
    //m_pControllerB = new CHydraControllerDriver(1);
    //vr::VRServerDriverHost()->TrackedDeviceAdded(m_pControllerB->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_pControllerB);

    return VRInitError_None;
}

void CServerDriver_Hydra::ThreadEntry(CServerDriver_Hydra * pDriver)
{
    pDriver->ThreadFunc();
}

void CServerDriver_Hydra::ThreadFunc()
{
    // We know the sixense SDK thread is running at "60 FPS", but we don't know when
    // those frames are.  To minimize latency, we sleep for slightly less than the
    // target rate, and detect when the frame has not advanced to wait a bit longer.
    auto longInterval = std::chrono::milliseconds(16);
    auto retryInterval = std::chrono::milliseconds(2);
    auto scanInterval = std::chrono::seconds(1);
    auto pollDeadline = std::chrono::steady_clock::now();
    auto scanDeadline = std::chrono::steady_clock::now() + scanInterval;

#ifdef _WIN32
    // Request at least 2ms timing granularity for the life of this process
    timeBeginPeriod(2);
#endif

    while (!m_bStopRequested)
    {
        // Check for new controllers here because sixense API is modal
        // (e.g. sixenseSetActiveBase()) so it can't happen in parallel with pose updates
        if (pollDeadline > scanDeadline)
        {
            ScanForNewControllers(true);
            scanDeadline += scanInterval;
        }

        bool bAnyActivated = false;
        bool bAllUpdated = true;
        for (int base = 0; base < sixenseGetMaxBases(); ++base)
        {
            if (!sixenseIsBaseConnected(base))
                continue;

            sixenseAllControllerData acd;

            sixenseSetActiveBase(base);
            if (sixenseGetAllNewestData(&acd) != SIXENSE_SUCCESS)
                continue;
            for (int id = 0; id < sixenseGetMaxControllers(); ++id)
            {
                for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
                {
                    CHydraControllerDriver *pHydra = *it;
                    if (pHydra->IsActivated() && pHydra->HasControllerId(base, id))
                    {
                        bAnyActivated = true;
                        // Returns true if this is new data (so we can sleep for long interval)
                        if (!pHydra->Update(acd.controllers[id]))
                        {
                            bAllUpdated = false;
                        }
                        break;
                    }
                }
            }
        }

        CheckForChordedSystemButtons();

        // If everyone just got new data, we can wait about 1/60s, else try again soon
        pollDeadline += !bAnyActivated ? scanInterval :
            bAllUpdated ? longInterval : retryInterval;
        std::this_thread::sleep_until(pollDeadline);
    }

#ifdef _WIN32
    timeEndPeriod(2);
#endif
}

void CServerDriver_Hydra::CheckForChordedSystemButtons()
{
    std::vector<CHydraControllerDriver *> vecHeldSystemButtons;

    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        CHydraControllerDriver *pHydra = *it;

        if (pHydra->IsHoldingSystemButton())
        {
            vecHeldSystemButtons.push_back(pHydra);
        }
    }
    // If two or more system buttons are pressed together, treat them as a chord
    // requesting a realignment of the coordinate system
    if (vecHeldSystemButtons.size() >= 2)
    {
        if (vecHeldSystemButtons.size() == 2)
        {
            CHydraControllerDriver::RealignCoordinates(vecHeldSystemButtons[0], vecHeldSystemButtons[1]);
        }

        for (auto it = vecHeldSystemButtons.begin(); it != vecHeldSystemButtons.end(); ++it)
        {
            (*it)->ConsumeSystemButtonPress();
        }
    }
}


uint32_t CServerDriver_Hydra::GetTrackedDeviceCount()
{
    scope_lock lock(m_Mutex);

    return m_vecControllers.size();
}

vr::ITrackedDeviceServerDriver * CServerDriver_Hydra::FindTrackedDeviceDriver(const char * pchId)
{
    scope_lock lock(m_Mutex);

    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        if (0 == strcmp((*it)->GetSerialNumber().c_str(), pchId))
        {
            return *it;
        }
    }
    return nullptr;
}

void CServerDriver_Hydra::ScanForNewControllers(bool bNotifyServer)
{
    for (int base = 0; base < sixenseGetMaxBases(); ++base)
    {
        if (sixenseIsBaseConnected(base))
        {
            sixenseSetActiveBase(base);
            for (int i = 0; i < sixenseGetMaxControllers(); ++i)
            {
                if (sixenseIsControllerEnabled(i))
                {
                    char buf[256];
                    GenerateSerialNumber(buf, sizeof(buf), base, i);
                    scope_lock lock(m_Mutex);
                    if (!FindTrackedDeviceDriver(buf))
                    {
                        DriverLog("added new device %s\n", buf);
                        m_vecControllers.push_back(new CHydraControllerDriver(base, i));
                        if (bNotifyServer)
                        {
                            vr::VRServerDriverHost()->TrackedDeviceAdded(m_vecControllers.back()->GetSerialNumber().c_str(), vr::TrackedDeviceClass_Controller, m_vecControllers.back());
                        }
                    }
                }
            }
        }
    }
}


void CServerDriver_Hydra::Cleanup()
{
    CleanupDriverLog();

    if (m_Thread && m_Thread->joinable())  // test for NULL m_Thread
    {
        m_bStopRequested = true;
        m_Thread->join();
        m_Thread = NULL;  // force to NULL after thread join()
        sixenseExit();
    }

    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        delete *it;
        *it = NULL;
    }
}


void CServerDriver_Hydra::RunFrame()
{
    for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
    {
        (*it)->RunFrame();
    }

    vr::VREvent_t vrEvent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
    {
        for (auto it = m_vecControllers.begin(); it != m_vecControllers.end(); ++it)
        {
            (*it)->ProcessEvent(vrEvent);
        }
    }
}

//-----------------------------------------------------------------------------
// Purpose: HmdDriverFactory
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode)
{
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriverHydra;
    }

    // watchdog causes problems with steamvr (endless startup loop), so it's disabled for now
    /*
    if (0 == strcmp(IVRWatchdogProvider_Version, pInterfaceName))
    {
        return &g_watchdogDriverHydra;
    }
    */

    if (pReturnCode)
        *pReturnCode = VRInitError_Init_InterfaceNotFound;

    return NULL;
}
