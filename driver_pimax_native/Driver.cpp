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

                // Start pi_server with our preload.
                {
                    HMODULE thisDll = nullptr;
                    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                           GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                       (LPCWSTR)&driver::CreateHmdDriver,
                                       &thisDll);
                    WCHAR path[MAX_PATH]{};
                    GetModuleFileNameW(thisDll, path, MAX_PATH);
                    const auto driverRoot = std::filesystem::path(path).parent_path();

                    const auto nvapiNullOutputDll = driverRoot / "nvapi_null_output.dll";
                    const auto nvapiNullOutputDllStr = nvapiNullOutputDll.string();
                    const auto nvapiNullOutputDllC = nvapiNullOutputDllStr.c_str();
                    const auto distortionExtractorDll = driverRoot / "distortion_extractor.dll";
                    const auto distortionExtractorDllStr = distortionExtractorDll.string();
                    const auto distortionExtractorDllC = distortionExtractorDllStr.c_str();

                    const std::filesystem::path pimaxRoot =
                        RegGetString(HKEY_LOCAL_MACHINE, L"SOFTWARE\\Pimax", L"RuntimePath")
                            .value_or(L"C:\\Program Files\\Pimax\\Runtime");

                    const bool disableWithDll = vr::VRSettings()->GetBool("driver_pimax_native", "disable_withdll");

                    TraceLoggingWriteTagged(local,
                                            "Driver_Init_StartPiServer",
                                            TLArg(pimaxRoot.c_str(), "PimaxRoot"),
                                            TLArg(nvapiNullOutputDllC, "NullOutputDll"),
                                            TLArg(distortionExtractorDllC, "DistortionExtractor"),
                                            TLArg(driver::k_distortionMapSize, "DistortionMapSize"),
                                            TLArg(disableWithDll, "DisableWithDll"));
                    SetEnvironmentVariableA("DISTORTION_EXTRACTOR_SIZE",
                                            error::_Fmt("%d", driver::k_distortionMapSize).c_str());

                    LPCSTR withDlls[] = {nvapiNullOutputDllC, distortionExtractorDllC};
                    STARTUPINFO info = {sizeof(info)};
                    m_piServer = {};
                    if (!DetourCreateProcessWithDlls((pimaxRoot / L"pi_server.exe").wstring().c_str(),
                                                     nullptr,
                                                     nullptr,
                                                     nullptr,
                                                     FALSE,
                                                     0,
                                                     nullptr,
                                                     pimaxRoot.c_str(),
                                                     &info,
                                                     &m_piServer,
                                                     !disableWithDll ? 2 : 0,
                                                     withDlls,
                                                     nullptr)) {
                        DriverLog("Failed to start pi_server: %d", GetLastError());
                    }
                    CloseHandle(m_piServer.hThread);
                    TraceLoggingWriteTagged(
                        local, "Driver_Init_StartPiServer", TLArg(m_piServer.dwProcessId, "ProcessId"));
                }

                // Attempt to create a session with the headset.
                CHECK_PVRCMD(pvr_initialise(&m_pvr));
                pvrResult status = pvr_failed;
                if (m_piServer.dwProcessId) {
                    uint32_t retries = 5;
                    do {
                        status = pvr_createSession(m_pvr, &m_pvrSession);
                        TraceLoggingWriteTagged(
                            local, "Driver_Init_CreateSession", TLArg(ToString(status).c_str(), "Status"));
                    } while (status != pvr_success && --retries &&
                             WaitForSingleObject(m_piServer.hProcess, 1000) == WAIT_TIMEOUT);
                }
                if (status == pvr_success) {
                    DriverLog("Connected to pi_server");

                    // WORKAROUND: For some reasons, pi_server returns junk data for a while. Retry.
                    pvrHmdTrackingStyle trackingStyle = pvrHmdTrackingStyle_Unknown;
                    pvrDisplayInfo displayInfo = {};
                    uint32_t retries = 10;
                    do {
                        DriverLog("Waiting for valid headset info...");
                        trackingStyle = (pvrHmdTrackingStyle)pvr_getTrackedDeviceIntProperty(
                            m_pvrSession,
                            pvrTrackedDevice_HMD,
                            pvrTrackedDeviceProp_Prop_HmdTrackingStyle_Int,
                            pvrHmdTrackingStyle_Unknown);
                        pvr_getEyeDisplayInfo(m_pvrSession, pvrEye_Left, &displayInfo);
                        TraceLoggingWriteTagged(local,
                                                "Driver_Init_Poll",
                                                TLArg(ToString(trackingStyle).c_str(), "TrackingStyle"),
                                                TLArg(displayInfo.width, "DisplayWidth"));
                    } while ((trackingStyle == pvrHmdTrackingStyle_Unknown || displayInfo.width == 0) && --retries &&
                             (Sleep(1000), true));

                    // We only attach to headsets with inside-out tracking, like the Pimax Crystal without Lighthouse
                    // faceplate.
                    if (displayInfo.width && trackingStyle == pvrHmdTrackingStyle_InsideOutCameras) {
                        DriverLog("Ready!");

                        // Turn off smart smoothing to avoid consuming unnecessarily resources.
                        CHECK_PVRCMD(pvr_setIntConfig(m_pvrSession, "dbg_asw_enable", 0));

                        try {
                            hmdDriver = driver::CreateHmdDriver(m_pvr, m_pvrSession);
                        } catch (std::exception& ex) {
                            DriverLog("Failed to initialize HMD: %s", ex.what());
                        }
                    } else {
                        if (!displayInfo.width) {
                            DriverLog("Failed to get headset information after retries");
                        } else {
                            DriverLog("Found a Pimax headset, but it does not appear to use inside-out tracking! (%s)",
                                      ToString(trackingStyle).c_str());
                        }
                    }

                    if (hmdDriver) {
                        vr::VRServerDriverHost()->TrackedDeviceAdded(
                            hmdDriver->GetSerialNumber(), vr::TrackedDeviceClass_HMD, hmdDriver.get());
                        m_isLoaded = true;
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
            if (m_pvrSession->hmdh) {
                pvr_destroySession(m_pvrSession);
                m_pvrSession = {};
            }

            if (m_piServer.dwProcessId) {
                GenerateConsoleCtrlEvent(CTRL_C_EVENT, m_piServer.dwProcessId);
                if (WaitForSingleObject(m_piServer.hProcess, 5000) == WAIT_TIMEOUT) {
                    DriverLog("Timeout waiting for pi_server, terminating anyway");
                }
                TerminateProcess(m_piServer.hProcess, 255);
                CloseHandle(m_piServer.hProcess);
                m_piServer = {};
            }

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
        PROCESS_INFORMATION m_piServer = {};
        pvrEnvHandle m_pvr = {};
        pvrSessionHandle m_pvrSession = {};

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
