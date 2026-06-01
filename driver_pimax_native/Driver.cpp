// MIT License
//
// Copyright(c) 2026 Matthieu Bucchianeri
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "pch.h"

#include "CompositorDriver.h"
#include "HmdDriver.h"
#include "ErrorHandling.h"
#include "Utilities.h"
#include "Tracing.h"

#include "version.h"
#include "commit.h"

using namespace util;

namespace {
    std::unique_ptr<vr::IServerTrackedDeviceProvider> thisDriver;
    std::unique_ptr<driver::IHmdDriver> hmdDriver;

    class Driver : public vr::IServerTrackedDeviceProvider {
      public:
        Driver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Driver_Ctor");

            TraceLoggingWriteStop(local, "Driver_Ctor");
        }

        virtual ~Driver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Driver_Dtor");

            Cleanup();

            TraceLoggingWriteStop(local, "Driver_Dtor");
        };

        vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Driver_Init");

            VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

            if (!m_isLoaded) {
                DriverLog("Driver version: %d.%d.%d (%s)",
                          DriverVersionMajor,
                          DriverVersionMinor,
                          DriverVersionPatch,
                          DriverCommitHash);

                // Attempt to create a session with the headset.
                pvrEnvHandle pvr = {};
                pvrSessionHandle pvrSession = {};
                pvr_initialise(&pvr);
                pvrResult status = pvr_createSession(pvr, &pvrSession);
                if (status == pvr_success) {
                    DriverLog("Connected to pi_server");

                    const bool isOpenPortEnabled = pvr_getIntConfig(pvrSession, "no_render", 0);

                    // We only attach to headsets with inside-out tracking, like the Pimax Crystal without Lighthouse
                    // faceplate.
                    pvrHmdTrackingStyle trackingStyle = pvrHmdTrackingStyle_Unknown;
                    trackingStyle = (pvrHmdTrackingStyle)pvr_getTrackedDeviceIntProperty(
                        pvrSession,
                        pvrTrackedDevice_HMD,
                        pvrTrackedDeviceProp_Prop_HmdTrackingStyle_Int,
                        pvrHmdTrackingStyle_Unknown);

                    if (isOpenPortEnabled && trackingStyle == pvrHmdTrackingStyle_InsideOutCameras) {
                        try {
                            hmdDriver = driver::CreateHmdDriver(pvr, pvrSession);
                            vr::VRServerDriverHost()->TrackedDeviceAdded(
                                hmdDriver->GetSerialNumber(), vr::TrackedDeviceClass_HMD, hmdDriver.get());
                            m_isLoaded = true;
                        } catch (std::exception& ex) {
                            DriverLog("Failed to initialize HMD: %s", ex.what());
                        }
                    } else {
                        if (!isOpenPortEnabled) {
                            DriverLog("Found a Pimax headset, but Pimax OpenPort is not enabled!");
                        } else {
                            DriverLog("Found a Pimax headset, but it does not appear to use inside-out tracking! (%s)",
                                      ToString(trackingStyle).c_str());
                        }
                        pvr_destroySession(pvrSession);
                    }
                } else {
                    DriverLog("Error creating session with pi_server: %s", ToString(status).c_str());
                }
            }

            TraceLoggingWriteStop(local, "Driver_Init", TLArg(m_isLoaded, "Loaded"));

            return m_isLoaded ? vr::VRInitError_None : vr::VRInitError_Init_HmdNotFound;
        }

        void Cleanup() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Driver_Cleanup");

            hmdDriver.reset();

            VR_CLEANUP_SERVER_DRIVER_CONTEXT();

            TraceLoggingWriteStop(local, "Driver_Cleanup");
        }

        const char* const* GetInterfaceVersions() override {
            return vr::k_InterfaceVersions;
        }

        void RunFrame() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Driver_RunFrame");

            if (hmdDriver) {
                vr::VREvent_t event;
                while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event))) {
                    switch (event.eventType) {
                    case vr::VREvent_Input_HapticVibration:
                        hmdDriver->SendHapticEvent(event.data.hapticVibration);
                        break;
                    case vr::VREvent_AnyDriverSettingsChanged:
                        hmdDriver->ApplySettingsChanges();
                        break;
#ifdef _DEBUG
                    case vr::VREvent_ProcessConnected:
                        {
                            PROCESSENTRY32 entry;
                            entry.dwSize = sizeof(PROCESSENTRY32);

                            HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
                            if (Process32First(snapshot, &entry) == TRUE) {
                                while (Process32Next(snapshot, &entry) == TRUE) {
                                    if (entry.th32ProcessID == event.data.process.pid) {
                                        const auto processExe = std::wstring_view(entry.szExeFile);
                                        if (event.eventType == vr::VREvent_ProcessConnected &&
                                            processExe == L"vrcompositor.exe") {
                                            driver::InjectVrCompositor(event.data.process.pid);
                                        }
                                        break;
                                    }
                                }
                            }
                            CloseHandle(snapshot);
                        }
                        break;
#endif
                    }
                }

                hmdDriver->RunFrame();
            }

            TraceLoggingWriteStop(local, "Driver_RunFrame");
        };

        bool ShouldBlockStandbyMode() override {
            return false;
        }

        void EnterStandby() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Driver_EnterStandby");

            TraceLoggingWriteStop(local, "Driver_EnterStandby");
        };

        void LeaveStandby() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "Driver_LeaveStandby");

            if (hmdDriver) {
                hmdDriver->LeaveStandby();
            }

            TraceLoggingWriteStop(local, "Driver_LeaveStandby");
        };

      private:
        bool m_isLoaded = false;
    };
} // namespace

// Entry point for vrserver.
extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0) {
        if (!thisDriver) {
            thisDriver = std::make_unique<Driver>();
        }
        return thisDriver.get();
    }
    if (pReturnCode) {
        *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    }
    return nullptr;
}
