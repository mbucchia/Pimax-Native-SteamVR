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

#include "CameraDriver.h"
#include "ControllerDriver.h"
#include "ErrorHandling.h"
#include "HmdDriver.h"
#include "SharedMemory.h"
#include "Tracing.h"
#include "Utilities.h"

using namespace driver;
using namespace util;

namespace {
    enum Components {
        ComponentSystemClick,
        ComponentPresence,

        ComponentCount,
    };

    class HmdDriver : public IHmdDriver, public vr::IVRDisplayComponent {
      public:
        HmdDriver(pvrEnvHandle pvr, pvrSessionHandle pvrSession) : m_pvr(pvr), m_pvrSession(pvrSession) {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_Ctor");

            // Enable parallel projections.
            CHECK_PVRCMD(pvr_setIntConfig(m_pvrSession, "view_rotation_fix", 1));

            // Cache all the useful state.
            CHECK_PVRCMD(pvr_getHmdInfo(m_pvrSession, &m_pvrHmdInfo));
            TraceLoggingWriteTagged(local,
                                    "HmdDriver_Ctor_HmdInfo",
                                    TLArg(m_pvrHmdInfo.VendorId, "VendorId"),
                                    TLArg(m_pvrHmdInfo.ProductId, "ProductId"),
                                    TLArg(m_pvrHmdInfo.Manufacturer, "Manufacturer"),
                                    TLArg(m_pvrHmdInfo.ProductName, "ProductName"),
                                    TLArg(m_pvrHmdInfo.SerialNumber, "SerialNumber"),
                                    TLArg(m_pvrHmdInfo.FirmwareMinor, "FirmwareMinor"),
                                    TLArg(m_pvrHmdInfo.FirmwareMajor, "FirmwareMajor"),
                                    TLArg(m_pvrHmdInfo.Resolution.w, "ResolutionWidth"),
                                    TLArg(m_pvrHmdInfo.Resolution.h, "ResolutionHeight"));

            if (vr::VRSettings()->GetBool("driver_pimax_native", "use_eye_tracking")) {
                // Crystal OG, Crystal Super, Dream Air SE, Dream Air.
                m_hasEyeTracking = m_pvrHmdInfo.ProductId == 0x0012 || m_pvrHmdInfo.ProductId == 0x0040 ||
                                   m_pvrHmdInfo.ProductId == 0x0042 || m_pvrHmdInfo.ProductId == 0x0044;
            }

            CHECK_PVRCMD(pvr_getEyeDisplayInfo(m_pvrSession, pvrEye_Left, &m_pvrDisplayInfo));
            TraceLoggingWriteTagged(local,
                                    "HmdDriver_Ctor_EyeDisplayInfo",
                                    TLArg(m_pvrDisplayInfo.edid_vid, "EdidVid"),
                                    TLArg(m_pvrDisplayInfo.edid_pid, "EdidPid"),
                                    TLArg(m_pvrDisplayInfo.pos_x, "PosX"),
                                    TLArg(m_pvrDisplayInfo.pos_y, "PosY"),
                                    TLArg(m_pvrDisplayInfo.width, "Width"),
                                    TLArg(m_pvrDisplayInfo.height, "Height"),
                                    TLArg(m_pvrDisplayInfo.refresh_rate, "RefreshRate"),
                                    TLArg((int)m_pvrDisplayInfo.disp_state, "DispState"),
                                    TLArg((int)m_pvrDisplayInfo.eye_display, "EyeDisplay"),
                                    TLArg((int)m_pvrDisplayInfo.eye_rotate, "EyeRotate"),
                                    TraceLoggingCharArray((char*)&m_pvrDisplayInfo.luid, sizeof(LUID), "Luid"));

            m_displayResolutionWidth = m_pvrDisplayInfo.width / 2;
            m_displayResolutionHeight = m_pvrDisplayInfo.height;

            CHECK_PVRCMD(pvr_getEyeRenderInfo(m_pvrSession, pvrEye_Left, &m_pvrEyeRenderInfo[0]));
            CHECK_PVRCMD(pvr_getEyeRenderInfo(m_pvrSession, pvrEye_Right, &m_pvrEyeRenderInfo[1]));

            // Initial pose fields.
            m_latestPose.qWorldFromDriverRotation.w = m_latestPose.qDriverFromHeadRotation.w =
                m_latestPose.qRotation.w = 1.f;
            m_latestPose.deviceIsConnected = true;
            m_latestPose.result = vr::TrackingResult_Running_OutOfRange;

            TraceLoggingWriteStop(local, "HmdDriver_Ctor");
        }

        ~HmdDriver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_Dtor");

            TraceLoggingWriteStop(local, "HmdDriver_Dtor");
        }

        vr::EVRInitError Activate(uint32_t unObjectId) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_Activate", TLArg(unObjectId, "ObjectId"));

            // WORKAROUND: Always start with 72Hz for now. There seems to be an issue with vrcompositor initializing
            // other modes. This is not great at all, because it changes a global setting :(
            vr::VRSettings()->SetFloat(vr::k_pch_SteamVR_Section, vr::k_pch_SteamVR_PreferredRefreshRate, 72.f);

            m_deviceIndex = unObjectId;

            const vr::PropertyContainerHandle_t container =
                vr::VRProperties()->TrackedDeviceToPropertyContainer(m_deviceIndex);

            DriverLog("Product: %s (%04x)", m_pvrHmdInfo.ProductName, m_pvrHmdInfo.ProductId);
            vr::VRProperties()->SetStringProperty(container, vr::Prop_TrackingSystemName_String, "Pimax");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, m_pvrHmdInfo.ProductName);
            vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "Pimax");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_SerialNumber_String, m_pvrHmdInfo.SerialNumber);

            vr::VRProperties()->SetStringProperty(container, vr::Prop_RenderModelName_String, "generic_hmd");
            vr::VRProperties()->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, 1);

            const auto photonTime = pvr_getTrackedDeviceFloatProperty(
                m_pvrSession, pvrTrackedDevice_HMD, pvrTrackedDeviceProp_SecondsFromVsyncToPhotons_Float, 0.01f);
            DriverLog("Photon Time: %.1fms", photonTime * 1000.f);
            vr::VRProperties()->SetFloatProperty(container, vr::Prop_SecondsFromVsyncToPhotons_Float, photonTime);

            vr::VRProperties()->SetInt32Property(container, vr::Prop_EdidVendorID_Int32, m_pvrDisplayInfo.edid_vid);
            vr::VRProperties()->SetInt32Property(container, vr::Prop_EdidProductID_Int32, m_pvrDisplayInfo.edid_pid);

            vr::VRProperties()->SetBoolProperty(container, vr::Prop_DisplaySupportsRuntimeFramerateChange_Bool, true);
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_IsOnDesktop_Bool, false);
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_DisplayAllowNightMode_Bool, true);

            m_currentIpd = pvr_getFloatConfig(m_pvrSession, CONFIG_KEY_IPD, m_currentIpd);
            {
                const PVR::Matrix4f eyeToHead[] = {
                    PVR::Matrix4(PVR::Posef(m_pvrEyeRenderInfo[pvrEye_Left].HmdToEyePose)),
                    PVR::Matrix4(PVR::Posef(m_pvrEyeRenderInfo[pvrEye_Right].HmdToEyePose))};
                vr::VRServerDriverHost()->SetDisplayEyeToHead(
                    m_deviceIndex, StoreHmdMatrix34(eyeToHead[pvrEye_Left]), StoreHmdMatrix34(eyeToHead[pvrEye_Right]));
            }
            vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float, m_currentIpd);

            m_horizontalFovTangent = vr::VRSettings()->GetFloat("driver_pimax_native", "horizontal_fov_tangent");
            m_verticalFovTangent = vr::VRSettings()->GetFloat("driver_pimax_native", "vertical_fov_tangent");
            TraceLoggingWriteTagged(local,
                                    "HmdDriver_Activate",
                                    TLArg(m_horizontalFovTangent, "HorizontalFovTangent"),
                                    TLArg(m_verticalFovTangent, "VerticalFovTangent"));

            bool hasHiddenAreaMesh = false;
            const bool useFovTangent = m_horizontalFovTangent < 1 || m_verticalFovTangent < 1;
            if (vr::VRSettings()->GetBool("driver_pimax_native", "use_hidden_area_mesh") && !useFovTangent) {
                vr::CVRHiddenAreaHelpers helpers = {vr::VRPropertiesRaw()};
                for (int eye = 0; eye < pvrEye_Count; eye++) {
                    std::vector<vr::HmdVector2_t> vertices;
                    size_t count;

                    count = pvr_getEyeHiddenAreaMesh(
                        m_pvrSession, (pvrEyeType)eye, pvrHiddenAreaMesh_HiddenArea, nullptr, 0);
                    vertices.resize(count);
                    pvr_getEyeHiddenAreaMesh(m_pvrSession,
                                             (pvrEyeType)eye,
                                             pvrHiddenAreaMesh_HiddenArea,
                                             (pvrVector2f*)vertices.data(),
                                             (unsigned int)vertices.size());
                    helpers.SetHiddenArea(
                        (vr::EVREye)eye, vr::k_eHiddenAreaMesh_Standard, vertices.data(), (uint32_t)vertices.size());

                    count = pvr_getEyeHiddenAreaMesh(
                        m_pvrSession, (pvrEyeType)eye, pvrHiddenAreaMesh_VisibleArea, nullptr, 0);
                    vertices.resize(count);
                    pvr_getEyeHiddenAreaMesh(m_pvrSession,
                                             (pvrEyeType)eye,
                                             pvrHiddenAreaMesh_VisibleArea,
                                             (pvrVector2f*)vertices.data(),
                                             (unsigned int)vertices.size());
                    helpers.SetHiddenArea(
                        (vr::EVREye)eye, vr::k_eHiddenAreaMesh_Inverse, vertices.data(), (uint32_t)vertices.size());

                    hasHiddenAreaMesh = hasHiddenAreaMesh || vertices.size();
                }
            }
            vr::VRProperties()->SetStringProperty(
                container, vr::Prop_InputProfilePath_String, "{pimax_native}/input/pimaxhmd_profile.json");
            vr::VRDriverInput()->CreateBooleanComponent(
                container, "/input/system/click", &m_components[ComponentSystemClick]);
            vr::VRDriverInput()->CreateBooleanComponent(container, "/proximity", &m_components[ComponentPresence]);

            vr::VRProperties()->SetStringProperty(container, vr::Prop_ExpectedControllerType_String, "oculus_touch");

            const auto vstType = pvr_getVSTType(m_pvrSession);
            if (vstType != pvrVSTTypeNone) {
                const auto format = pvr_getVSTStreamFormat(m_pvrSession);
                DriverLog("Supports VST: %d, %d", vstType, format);

                if (format == pvrVST_FORMAT_NV12 || format == pvrVST_FORMAT_RAW8) {
                    // TODO: For now we disable the camera since it causes hang.
#if 0
                    m_cameraDriver = CreateCameraDriver(m_pvr, m_pvrSession);
                    m_cameraDriver->Activate(m_deviceIndex);
#endif
                } else {
                    DriverLog("Unsupported stream format, VST will be unavailable");
                }
            }

            if (m_hasEyeTracking) {
                DriverLog("Supports eye tracking");
                vr::VRProperties()->SetBoolProperty(container, vr::Prop_SupportsXrEyeGazeInteraction_Bool, true);
                vr::VRDriverInput()->CreateEyeTrackingComponent(container, "/eyetracking", &m_eyeTrackingComponent);
            }

            const auto playbackDevice = pvr_getTrackedDeviceStringPropertyHelper(
                m_pvrSession, pvrTrackedDevice_HMD, pvrTrackedDeviceProp_Audio_PlaybackDeviceId_String);
            const auto recordingDevice = pvr_getTrackedDeviceStringPropertyHelper(
                m_pvrSession, pvrTrackedDevice_HMD, pvrTrackedDeviceProp_Audio_RecordingDeviceId_String);
            vr::VRProperties()->SetStringProperty(
                container, vr::Prop_Audio_DefaultPlaybackDeviceId_String, playbackDevice.c_str());
            vr::VRProperties()->SetStringProperty(
                container, vr::Prop_Audio_DefaultRecordingDeviceId_String, recordingDevice.c_str());

            // clang-format off
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceOff_String, "{pimax_native}/icons/headset_status_off.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceSearching_String, "{pimax_native}/icons/headset_status_searching.gif");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{pimax_native}/icons/headset_status_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceReady_String, "{pimax_native}/icons/headset_status_ready.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{pimax_native}/icons/headset_status_ready_alert.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceNotReady_String, "{pimax_native}/icons/headset_status_error.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceStandby_String, "{pimax_native}/icons/headset_status_standby.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceAlertLow_String, "{pimax_native}/icons/headset_standby_alert.png");
            // clang-format on

            vr::VRProperties()->SetStringProperty(container, vr::Prop_ResourceRoot_String, "pimax_native");
            vr::VRProperties()->SetStringProperty(container,
                                                  vr::Prop_AdditionalDeviceSettingsPath_String,
                                                  "{pimax_native}/settings/settingsschema.vrsettings");

            TraceLoggingWriteTagged(local,
                                    "HmdDriver_Activate",
                                    TLArg(m_pvrHmdInfo.ProductName, "ProductName"),
                                    TLArg(m_pvrHmdInfo.ProductId, "ProductId"),
                                    TLArg(photonTime, "VsyncToPhotons"),
                                    TLArg(m_pvrDisplayInfo.edid_vid, "EdidVid"),
                                    TLArg(m_pvrDisplayInfo.edid_pid, "EdidPid"),
                                    TLArg(m_currentIpd, "Ipd"),
                                    TLArg(hasHiddenAreaMesh, "HasHiddenAreaMesh"),
                                    TLArg((int)vstType, "VSTType"),
                                    TLArg(playbackDevice.c_str(), "PlaybackDevice"),
                                    TLArg(recordingDevice.c_str(), "RecordingDevice"));

            // Setup IPC with the client utility.
            *m_sharedFileHandle.put() = CreateFileMapping(
                INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(*m_sharedMemory), L"PimaxNative.SharedMemory");
            if (m_sharedFileHandle) {
                m_sharedMemory = (shared::SharedMemory*)MapViewOfFile(
                    m_sharedFileHandle.get(), FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, sizeof(*m_sharedMemory));
                memset(m_sharedMemory, 0, sizeof(*m_sharedMemory));
            }

            // This must be done after initializing the IPC to the client utility.
            ApplySettingsChanges();

            // Prepare to retrieve the distortion map.
            const auto openDistortionShm = [&](const WCHAR* name, UINT channel) {
                constexpr UINT size = k_distortionMapSize * k_distortionMapSize * sizeof(float) * 4;
                *m_sharedDistortionMapHandle[channel].put() =
                    OpenFileMapping(FILE_MAP_READ | FILE_MAP_WRITE, false, name);
                if (m_sharedDistortionMapHandle[channel]) {
                    m_sharedDistortionMap[channel] = (float*)MapViewOfFile(
                        m_sharedDistortionMapHandle[channel].get(), FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, size);
                } else {
                    DriverLog("Failed to open '%ws'", name);
                }
            };
            openDistortionShm(L"DistortionExtractor.Red", 0);
            openDistortionShm(L"DistortionExtractor.Green", 1);
            openDistortionShm(L"DistortionExtractor.Blue", 2);

            if (m_sharedDistortionMap[0] && m_sharedDistortionMap[1] && m_sharedDistortionMap[2]) {
                vr::VRProperties()->SetInt32Property(
                    container, vr::Prop_DistortionMeshResolution_Int32, k_distortionMapSize);
            }

#if 0
            std::ofstream outFile("d:\\dist.txt");
            for (uint32_t v = 0; v < k_distortionMapSize; v++) {
                for (uint32_t u = 0; u < k_distortionMapSize; u++) {
                    const uint32_t rowPitch = k_distortionMapSize * 4;
                    if (m_sharedDistortionMap && m_sharedDistortionMap[0][v * rowPitch + u * 4 + 3]) {
                        outFile << "(" << m_sharedDistortionMap[0][v * rowPitch + u * 4 + 0] << ","
                                << m_sharedDistortionMap[0][v * rowPitch + u * 4 + 1] << "), ";
                    } else {
                        outFile << "(NONE),";
                    }
                }
                outFile << "\n";
            }
#endif

            // WORKAROUND: To make PVR happy and receive controller/inputs, we must call pvr_beginFrame() from
            // RunFrame(), which requires to have bound a D3D device via a dummy swapchain.
            {
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d11.lib")
                ComPtr<IDXGIFactory1> dxgiFactory;
                CHECK_HRCMD(CreateDXGIFactory1(IID_PPV_ARGS(dxgiFactory.ReleaseAndGetAddressOf())));

                ComPtr<IDXGIAdapter1> dxgiAdapter;
                for (UINT adapterIndex = 0;; adapterIndex++) {
                    // EnumAdapters1 will fail with DXGI_ERROR_NOT_FOUND when there are no more adapters to
                    // enumerate.
                    CHECK_HRCMD(dxgiFactory->EnumAdapters1(adapterIndex, dxgiAdapter.ReleaseAndGetAddressOf()));

                    DXGI_ADAPTER_DESC1 desc;
                    CHECK_HRCMD(dxgiAdapter->GetDesc1(&desc));
                    if (!memcmp(&desc.AdapterLuid, &m_pvrDisplayInfo.luid, sizeof(LUID))) {
                        break;
                    }
                }

                const D3D_FEATURE_LEVEL featureLevel = D3D_FEATURE_LEVEL_11_0;
                CHECK_HRCMD(D3D11CreateDevice(dxgiAdapter.Get(),
                                              D3D_DRIVER_TYPE_UNKNOWN,
                                              0,
                                              D3D11_CREATE_DEVICE_BGRA_SUPPORT,
                                              &featureLevel,
                                              1,
                                              D3D11_SDK_VERSION,
                                              m_d3dDummyDevice.ReleaseAndGetAddressOf(),
                                              nullptr,
                                              nullptr));

                pvrTextureSwapChainDesc desc = {};
                desc.Width = desc.Height = 128;
                desc.ArraySize = desc.MipLevels = desc.SampleCount = 1;
                desc.Format = PVR_FORMAT_R8G8B8A8_UNORM_SRGB;
                desc.StaticImage = true;
                pvrTextureSwapChain swapchain = {};
                CHECK_PVRCMD(pvr_createTextureSwapChainDX(m_pvrSession, m_d3dDummyDevice.Get(), &desc, &swapchain));
            }

            TraceLoggingWriteStop(local, "HmdDriver_Activate");

            return vr::VRInitError_None;
        }

        void Deactivate() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_Deactivate", TLArg(m_deviceIndex, "ObjectId"));

            if (m_updateThreadActive.exchange(false) && m_updateThread.joinable()) {
                m_updateThread.join();
            }

            for (uint32_t channel = 0; channel < 3; channel++) {
                if (m_sharedDistortionMapHandle[channel]) {
                    UnmapViewOfFile(m_sharedDistortionMap[channel]);
                    m_sharedDistortionMap[channel] = nullptr;
                }
            }
            if (m_sharedMemory) {
                UnmapViewOfFile(m_sharedMemory);
                m_sharedMemory = nullptr;
            }
            if (m_sharedFileHandle) {
                m_sharedFileHandle.reset();
            }

            if (m_cameraDriver) {
                m_cameraDriver->Deactivate(m_deviceIndex);
                m_cameraDriver.reset();
            }

            m_deviceIndex = vr::k_unTrackedDeviceIndexInvalid;

            TraceLoggingWriteStop(local, "HmdDriver_Deactivate");
        }

        void EnterStandby() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_EnterStandby", TLArg(m_deviceIndex, "ObjectId"));

            TraceLoggingWriteStop(local, "HmdDriver_EnterStandby");
        }

        void LeaveStandby() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_LeaveStandby", TLArg(m_deviceIndex, "ObjectId"));

            TraceLoggingWriteStop(local, "HmdDriver_LeaveStandby");
        }

        void* GetComponent(const char* pchComponentNameAndVersion) override {
            if (strcmp(vr::IVRDisplayComponent_Version, pchComponentNameAndVersion) == 0) {
                return (vr::IVRDisplayComponent*)this;
            } else if (strcmp(vr::IVRCameraComponent_Version, pchComponentNameAndVersion) == 0) {
                return m_cameraDriver.get();
            }
            return nullptr;
        }

        vr::DriverPose_t GetPose() override {
            std::shared_lock lock(m_poseMutex);
            return m_latestPose;
        }

        void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {
            if (unResponseBufferSize >= 1) {
                pchResponseBuffer[0] = 0;
            }
        }

        bool IsDisplayOnDesktop() override {
            return false;
        }

        bool IsDisplayRealDisplay() override {
            return true;
        }

        void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_GetRecommendedRenderTargetSize", TLArg(m_deviceIndex, "ObjectId"));

            pvrSizei viewportSize;
            CHECK_PVRCMD(pvr_getFovTextureSize(
                m_pvrSession, pvrEye_Left, m_pvrEyeRenderInfo[pvrEye_Left].Fov, 1.f, &viewportSize));

            *pnWidth = (uint32_t)(viewportSize.w * m_horizontalFovTangent);
            *pnWidth = (*pnWidth + 3) / 4 * 4;
            *pnHeight = (uint32_t)(viewportSize.h * m_verticalFovTangent);
            *pnHeight = (*pnHeight + 3) / 4 * 4;

            TraceLoggingWriteStop(local,
                                  "HmdDriver_GetRecommendedRenderTargetSize",
                                  TLArg(*pnWidth, "RecommendedWidth"),
                                  TLArg(*pnHeight, "RecommendedHeight"));
        }

        void GetEyeOutputViewport(
            vr::EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "HmdDriver_GetEyeOutputViewport",
                                   TLArg(m_deviceIndex, "ObjectId"),
                                   TLArg(eEye == vr::Eye_Left ? "Left" : "Right", "Eye"));

            *pnX = eEye == vr::Eye_Left ? 0 : m_displayResolutionWidth;
            *pnY = 0;
            *pnWidth = m_displayResolutionWidth;
            *pnHeight = m_displayResolutionHeight;

            TraceLoggingWriteStop(local,
                                  "HmdDriver_GetEyeOutputViewport",
                                  TLArg(*pnX, "X"),
                                  TLArg(*pnY, "Y"),
                                  TLArg(*pnWidth, "Width"),
                                  TLArg(*pnHeight, "Height"));
        }

        void GetProjectionRaw(vr::EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "HmdDriver_GetProjectionRaw",
                                   TLArg(m_deviceIndex, "ObjectId"),
                                   TLArg(eEye == vr::Eye_Left ? "Left" : "Right", "Eye"));

            *pfLeft = -m_pvrEyeRenderInfo[eEye].Fov.LeftTan * m_horizontalFovTangent;
            *pfRight = m_pvrEyeRenderInfo[eEye].Fov.RightTan * m_horizontalFovTangent;
            // Top and bottom are backwards per SteamVR documentation.
            *pfTop = -m_pvrEyeRenderInfo[eEye].Fov.DownTan * m_verticalFovTangent;
            *pfBottom = m_pvrEyeRenderInfo[eEye].Fov.UpTan * m_verticalFovTangent;

            TraceLoggingWriteStop(local,
                                  "HmdDriver_GetProjectionRaw",
                                  TLArg(*pfLeft, "Left"),
                                  TLArg(*pfRight, "Right"),
                                  TLArg(*pfBottom, "Bottom"),
                                  TLArg(*pfTop, "Top"));
        }

        vr::DistortionCoordinates_t ComputeDistortion(vr::EVREye eEye, float fU, float fV) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "HmdDriver_ComputeDistortion",
                                   TLArg(m_deviceIndex, "ObjectId"),
                                   TLArg(eEye == vr::Eye_Left ? "Left" : "Right", "Eye"),
                                   TLArg(fU, "U"),
                                   TLArg(fV, "V"));

            vr::DistortionCoordinates_t result;
            // TODO: PVR does not give the correct map, only gives identity.
            // For now, we use a distortion extraction process from pi_server, and convert the mesh to the format
            // expected here.
            if (m_sharedDistortionMap[0] && m_sharedDistortionMap[1] && m_sharedDistortionMap[2]) {
                // Convert from UV coordinates (in increment of Prop_DistortionMeshResolution_Int32 =
                // k_distortionMapSize) back to array indices.
                // - Don't forget to round the values to nearest to counteract the imprecision of floats when converting
                //   back to integer.
                // - Don't forget to mirror the right eye (we only have one mesh for both eyes).
                // - Clamp to avoid any buffer over/underflow.
                //
                // clang-format off
                const int32_t u = std::clamp((int32_t)std::round((eEye == vr::Eye_Left ? fU : (1.f - fU)) * (k_distortionMapSize - 1)), 0, (int32_t)k_distortionMapSize - 1);
                const int32_t v = std::clamp((int32_t)std::round(                                    fV   * (k_distortionMapSize - 1)), 0, (int32_t)k_distortionMapSize - 1);
                // clang-format on

                // Distortion SHM contains RGBA samples, with R = U, G = V, B = unused, and A = validity
                const uint32_t rowPitch = k_distortionMapSize * 4;
                const auto getIndex = [](uint32_t u, uint32_t v) { return v * rowPitch + u * 4; };

                const uint32_t index = getIndex(u, v);
                const auto getDistortion = [&](float* uv, uint32_t channel) {
                    if (m_sharedDistortionMap[channel][index + 3]) {
                        uv[0] = m_sharedDistortionMap[channel][index + 0];
                        uv[1] = m_sharedDistortionMap[channel][index + 1];

                    } else {
                        uv[0] = uv[1] = NAN;
                    }
                };

                // TODO: Add FOV tangents calculation.
                getDistortion(result.rfRed, 0);
                getDistortion(result.rfGreen, 1);
                getDistortion(result.rfBlue, 2);

#ifdef _DEBUG
                if (u == k_distortionMapSize / 2 && v == 0 || v == k_distortionMapSize / 2 && u == 0 ||
                    u == k_distortionMapSize / 2 && v == k_distortionMapSize - 1 ||
                    v == k_distortionMapSize / 2 && u == k_distortionMapSize - 1) {
                    DriverLog("Distortion Eye%d at (%d, %d) = r:(%.3f, %.3f), g:(%.3f, %.3f), b:(%.3f, %.3f)",
                              eEye,
                              u,
                              v,
                              result.rfRed[0],
                              result.rfRed[1],
                              result.rfGreen[0],
                              result.rfGreen[1],
                              result.rfBlue[0],
                              result.rfBlue[1]);
                }
#endif
            } else {
                CHECK_PVRCMD(pvr_getHmdDistortedUV(m_pvrSession, (pvrEyeType)eEye, {fU, fV}, (pvrVector2f*)&result));
            }

            TraceLoggingWriteStop(local,
                                  "HmdDriver_ComputeDistortion",
                                  TLArg(result.rfRed[0], "RedX"),
                                  TLArg(result.rfRed[1], "RedY"),
                                  TLArg(result.rfGreen[0], "GreenX"),
                                  TLArg(result.rfGreen[1], "GreenY"),
                                  TLArg(result.rfBlue[0], "BlueX"),
                                  TLArg(result.rfBlue[1], "BlueY"));

            return result;
        }

        void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight) override {
            *pnX = 0;
            *pnY = 0;
            *pnWidth = m_displayResolutionWidth * 2;
            *pnHeight = m_displayResolutionHeight;
        }

        bool ComputeInverseDistortion(
            vr::HmdVector2_t* pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV) override {
            // Not supported.
            return false;
        }

        void SendHapticEvent(const vr::VREvent_HapticVibration_t& data) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "HmdDriver_SendHapticEvent",
                                   TLArg(m_deviceIndex, "ObjectId"),
                                   TLArg(data.containerHandle, "Container"));

            // Dispatch events to the comtrollers.
            for (uint32_t side = 0; side < 2; side++) {
                if (m_controllerDriver[side]) {
                    if (vr::VRProperties()->TrackedDeviceToPropertyContainer(
                            m_controllerDriver[side]->GetDeviceIndex()) == data.containerHandle) {
                        m_controllerDriver[side]->SendHapticEvent(data);
                        break;
                    }
                }
            }

            TraceLoggingWriteStop(local, "HmdDriver_SendHapticEvent");
        }

        void ApplySettingsChanges() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_ApplySettingsChanges", TLArg(m_deviceIndex, "ObjectId"));

            const vr::PropertyContainerHandle_t container =
                vr::VRProperties()->TrackedDeviceToPropertyContainer(m_deviceIndex);

            const bool gammaMode = vr::VRSettings()->GetBool("driver_pimax_native", "gamma_mode");
            const float redGain = gammaMode ? vr::VRSettings()->GetFloat("driver_pimax_native", "red_gain")
                                            : vr::VRSettings()->GetFloat("driver_pimax_native", "rgb_gain");
            const float greenGain =
                gammaMode ? vr::VRSettings()->GetFloat("driver_pimax_native", "green_gain") : redGain;
            const float blueGain = gammaMode ? vr::VRSettings()->GetFloat("driver_pimax_native", "blue_gain") : redGain;
            TraceLoggingWriteTagged(local,
                                    "HmdDriver_ApplySettingsChanges",
                                    TLArg(redGain, "Red"),
                                    TLArg(greenGain, "Green"),
                                    TLArg(blueGain, "Blue"));

            vr::VRProperties()->SetVec3Property(
                container, vr::Prop_DisplayColorMultLeft_Vector3, {redGain, greenGain, blueGain});
            vr::VRProperties()->SetVec3Property(
                container, vr::Prop_DisplayColorMultRight_Vector3, {redGain, greenGain, blueGain});

            // Dispatch settings update to the controllers.
            for (uint32_t side = 0; side < 2; side++) {
                if (m_controllerDriver[side]) {
                    m_controllerDriver[side]->ApplySettingsChanges();
                }
            }

            // Dispatch settings to the client process.
            if (m_sharedMemory) {
                m_sharedMemory->allowOpenDashboard =
                    vr::VRSettings()->GetInt32("driver_pimax_native", "use_windows_key") >= 2;
            }

            TraceLoggingWriteStop(local, "HmdDriver_ApplySettingsChanges");
        }

        void RunFrame() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_RunFrame", TLArg(m_deviceIndex, "ObjectId"));

            const vr::PropertyContainerHandle_t container =
                vr::VRProperties()->TrackedDeviceToPropertyContainer(m_deviceIndex);

            if (m_d3dDummyDevice) {
                // Poke the bear.
                static long long frameId = 0;
                pvr_beginFrame(m_pvrSession, frameId++);
            }

            pvrHmdStatus hmdStatus = {};
            CHECK_PVRCMD(pvr_getHmdStatus(m_pvrSession, &hmdStatus));
            TraceLoggingWriteTagged(local,
                                    "HmdDriver_RunFrame_HmdStatus",
                                    TLArg(!!hmdStatus.ServiceReady, "ServiceReady"),
                                    TLArg(!!hmdStatus.HmdPresent, "HmdPresent"),
                                    TLArg(!!hmdStatus.HmdMounted, "HmdMounted"),
                                    TLArg(!!hmdStatus.IsVisible, "IsVisible"),
                                    TLArg(!!hmdStatus.DisplayLost, "DisplayLost"),
                                    TLArg(!!hmdStatus.ShouldQuit, "ShouldQuit"));

            vr::VRDriverInput()->UpdateBooleanComponent(m_components[ComponentPresence], hmdStatus.HmdMounted, 0);

            m_useAsyncTracking = vr::VRSettings()->GetBool("driver_pimax_native", "use_async_tracking");
            if (!m_useAsyncTracking) {
                // Halt the thread if needed.
                if (m_updateThreadActive.exchange(false) && m_updateThread.joinable()) {
                    DriverLog("Stopping async tracking thread");
                    m_updateThread.join();
                }

                // Do the update synchronously.
                UpdatePvrState(pvr_getTimeSeconds(m_pvr));
            } else {
                if (!m_updateThreadActive) {
                    // Spin up the thread if needed.
                    DriverLog("Starting async tracking thread");
                    m_updateThreadActive = true;
                    m_updateThread = std::thread(&HmdDriver::UpdateThread, this);
                }
            }

            // Update the battery level.
            const int batteryPercentage = pvr_getTrackedDeviceIntProperty(
                m_pvrSession, pvrTrackedDevice_HMD, pvrTrackedDeviceProp_BatteryPercent_int, -1);
            if (batteryPercentage > 0) {
                vr::VRProperties()->SetFloatProperty(
                    container, vr::Prop_DeviceBatteryPercentage_Float, batteryPercentage / 100.f);
                vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
            }

            // Adjust IPD based on slider.
            const auto currentIpd = pvr_getFloatConfig(m_pvrSession, CONFIG_KEY_IPD, m_currentIpd);
            if (fabsf(currentIpd - m_currentIpd) >= 0.0001f) {
                vr::VRProperties()->SetFloatProperty(container, vr::Prop_UserIpdMeters_Float, currentIpd);
                m_currentIpd = currentIpd;
            }

            TraceLoggingWriteTagged(local,
                                    "HmdDriver_RunFrame_HmdProps",
                                    TLArg(batteryPercentage, "BatteryPercentage"),
                                    TLArg(currentIpd, "Ipd"));

            for (uint32_t side = 0; side < 2; side++) {
                // Detect controllers.
                const auto controllerType = pvr_getTrackedDeviceStringPropertyHelper(
                    m_pvrSession,
                    side == 0 ? pvrTrackedDevice_LeftController : pvrTrackedDevice_RightController,
                    pvrTrackedDeviceProp_ControllerType_String);
                const bool isActive = !controllerType.empty();
                if (isActive) {
                    TraceLoggingWriteTagged(local,
                                            "HmdDriver_RunFrame",
                                            TLArg(controllerType.c_str(), "DetectedController"),
                                            TLArg(side == 0 ? "Left" : "Right", "Side"));
                    if (!m_controllerDriver[side]) {
                        const bool isPimaxController = controllerType == "pimax_crystal";
                        if (isPimaxController) {
                            m_controllerDriver[side] = CreateControllerDriver(
                                m_pvr,
                                m_pvrSession,
                                side == 0 ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_RightHand);

                            vr::VRServerDriverHost()->TrackedDeviceAdded(m_controllerDriver[side]->GetSerialNumber(),
                                                                         vr::TrackedDeviceClass_Controller,
                                                                         m_controllerDriver[side].get());
                        }
                    }
                } else {
                    if (m_controllerDriver[side]) {
                        // We never remove the controller driver, we just mark it as disconnected.
                        m_controllerDriver[side]->Disconnect();
                    }
                }

                // Dispatch RunFrame() to the controllers.
                if (m_controllerDriver[side]) {
                    m_controllerDriver[side]->RunFrame();
                }
            }

            // Dispatch RunFrame() to the camera.
            if (m_cameraDriver) {
                m_cameraDriver->RunFrame();
            }

            // Handle client utility events.
            if (m_sharedMemory) {
                // See if the client process is already started.
                if (m_clientProcessInfo.dwProcessId) {
                    if (!WaitForSingleObject(m_clientProcessInfo.hProcess, 0)) {
                        CloseHandle(m_clientProcessInfo.hProcess);

                        // Mark as finished.
                        m_clientProcessInfo = {};
                    }
                }

                // Start the client process if needed.
                if (!m_clientProcessInfo.dwProcessId) {
                    HMODULE thisDll = nullptr;
                    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                           GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                       (LPCWSTR)&CreateHmdDriver,
                                       &thisDll);
                    WCHAR path[MAX_PATH]{};
                    GetModuleFileNameW(thisDll, path, MAX_PATH);
                    const auto root = std::filesystem::path(path).parent_path();

                    STARTUPINFO info = {sizeof(info)};
                    if (!CreateProcess((root / L"client_utility.exe").wstring().c_str(),
                                       nullptr,
                                       nullptr,
                                       nullptr,
                                       FALSE,
                                       0,
                                       nullptr,
                                       root.parent_path().parent_path().c_str(),
                                       &info,
                                       &m_clientProcessInfo)) {
                        DriverLog("Failed to start client utility: %d", GetLastError());
                    }
                    CloseHandle(m_clientProcessInfo.hThread);
                }

                if (InterlockedExchange(&m_sharedMemory->sendClickEvent, 0)) {
                    vr::VRDriverInput()->UpdateBooleanComponent(m_components[ComponentSystemClick], true, 0);
                    m_inClickEvent = true;
                } else if (m_inClickEvent) {
                    vr::VRDriverInput()->UpdateBooleanComponent(m_components[ComponentSystemClick], false, 0);
                    m_inClickEvent = false;
                }
            }

            TraceLoggingWriteStop(local, "HmdDriver_RunFrame");
        }

        void UpdateTrackingState(const pvrPoseStatef& state) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_UpdateTrackingState", TLArg(m_deviceIndex, "ObjectId"));

            vr::DriverPose_t pose = {};
            pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = pose.qRotation.w = 1.0;
            pose.deviceIsConnected = true;
            pose.result = vr::TrackingResult_Running_OutOfRange;

            if (state.StatusFlags & pvrStatus_OrientationTracked) {
                const bool force3Dof = vr::VRSettings()->GetBool("driver_pimax_native", "force_3dof");

                if (!force3Dof) {
                    pose.vecPosition[0] = state.ThePose.Position.x;
                    pose.vecPosition[1] = state.ThePose.Position.y;
                    pose.vecPosition[2] = state.ThePose.Position.z;
                }
                pose.qRotation.x = state.ThePose.Orientation.x;
                pose.qRotation.y = state.ThePose.Orientation.y;
                pose.qRotation.z = state.ThePose.Orientation.z;
                pose.qRotation.w = state.ThePose.Orientation.w;

                if (!force3Dof) {
                    pose.vecVelocity[0] = state.LinearVelocity.x;
                    pose.vecVelocity[1] = state.LinearVelocity.y;
                    pose.vecVelocity[2] = state.LinearVelocity.z;
                }

                pose.vecAngularVelocity[0] = state.AngularVelocity.x;
                pose.vecAngularVelocity[1] = state.AngularVelocity.y;
                pose.vecAngularVelocity[2] = state.AngularVelocity.z;

                const auto pvrNow = pvr_getTimeSeconds(m_pvr);
                pose.poseTimeOffset = state.TimeInSeconds - pvrNow;

                pose.poseIsValid = true;
                pose.result = vr::TrackingResult_Running_OK;
            }

            {
                std::unique_lock lock(m_poseMutex);
                m_latestPose = pose;
            }
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));

            TraceLoggingWriteStop(local, "HmdDriver_UpdateTrackingState", TLArg(pose.poseTimeOffset, "PoseTimeOffset"));
        }

        void UpdateEyeTrackingState(const pvrEyeTrackingInfo& state) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_UpdateEyeTrackingState", TLArg(m_deviceIndex, "ObjectId"));

            vr::VREyeTrackingData_t data = {};
            data.bActive = true;
            data.bValid = data.bTracked = state.TimeInSeconds > 0;
            if (data.bValid) {
                // This logic is taken from PimaxXR. Seems to have worked well over the years.
                const float angleHorizontal =
                    atanf((state.GazeTan[pvrEye_Left].x + state.GazeTan[pvrEye_Right].x) / 2.f);
                const float angleVertical = atanf((state.GazeTan[pvrEye_Left].y + state.GazeTan[pvrEye_Right].y) / 2.f);

                // Use polar coordinates to create a unit vector.
                data.vGazeTarget.v[0] = sinf(angleHorizontal) * cosf(angleVertical);
                data.vGazeTarget.v[1] = sinf(angleVertical);
                data.vGazeTarget.v[2] = -cosf(angleHorizontal) * cosf(angleVertical);
            }
            vr::VRDriverInput()->UpdateEyeTrackingComponent(m_eyeTrackingComponent, &data, 0.f);

            TraceLoggingWriteStop(local, "HmdDriver_UpdateEyeTrackingState");
        }

        const char* GetSerialNumber() const override {
            return m_pvrHmdInfo.SerialNumber;
        }

        vr::TrackedDeviceIndex_t GetDeviceIndex() const override {
            return m_deviceIndex;
        }

      private:
        void UpdatePvrState(double pvrTime) {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(
                local, "HmdDriver_UpdatePvrState", TLArg(m_deviceIndex, "ObjectId"), TLArg(pvrTime, "Time"));

            // Update HMD, controllers, and eye tracking.
            pvrTrackingState state = {};
            CHECK_PVRCMD(pvr_getTrackingState(m_pvrSession, pvrTime, &state));
            TraceLoggingWriteTagged(local,
                                    "HmdDriver_UpdatePvrState_HmdTrackingState",
                                    TLArg(state.HeadPose.StatusFlags, "StatusFlags"),
                                    TLArg(ToString(state.HeadPose.ThePose).c_str(), "Pose"),
                                    TLArg(ToString(state.HeadPose.AngularVelocity).c_str(), "AngularVelocity"),
                                    TLArg(ToString(state.HeadPose.LinearVelocity).c_str(), "LinearVelocity"),
                                    TLArg(state.HeadPose.TimeInSeconds, "TimeInSeconds"));

            UpdateTrackingState(state.HeadPose);

            for (uint32_t side = 0; side < 2; side++) {
                if (m_controllerDriver[side]) {
                    TraceLoggingWriteTagged(
                        local,
                        "HmdDriver_UpdatePvrState_ControllerTrackingState",
                        TLArg(side == 0 ? "Left" : "Right", "Side"),
                        TLArg(state.HandPoses[side].StatusFlags, "StatusFlags"),
                        TLArg(ToString(state.HandPoses[side].ThePose).c_str(), "Pose"),
                        TLArg(ToString(state.HandPoses[side].AngularVelocity).c_str(), "AngularVelocity"),
                        TLArg(ToString(state.HandPoses[side].LinearVelocity).c_str(), "LinearVelocity"),
                        TLArg(state.HandPoses[side].TimeInSeconds, "TimeInSeconds"));

                    m_controllerDriver[side]->UpdateTrackingState(state.HandPoses[side]);
                }
            }

            if (m_hasEyeTracking) {
                pvrEyeTrackingInfo state = {};
                CHECK_PVRCMD(pvr_getEyeTrackingInfo(m_pvrSession, pvrTime, &state));
                TraceLoggingWriteTagged(local,
                                        "HmdDriver_UpdatePvrState_EyeTrackingInfo",
                                        TLArg(ToString(state.GazeTan[pvrEye_Left]).c_str(), "LeftGaze"),
                                        TLArg(ToString(state.GazeTan[pvrEye_Right]).c_str(), "RightGaze"),
                                        TLArg(state.TimeInSeconds, "TimeInSeconds"));

                UpdateEyeTrackingState(state);
            }

            TraceLoggingWriteStop(local, "HmdDriver_UpdatePvrState");
        }

        void UpdateThread() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HmdDriver_UpdateThread", TLArg(m_deviceIndex, "ObjectId"));

            SetThreadDescription(GetCurrentThread(), L"HmdDriver_UpdateThread");
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

            wil::unique_handle timer;
            *timer.put() = CreateWaitableTimer(nullptr, FALSE, nullptr);
            LARGE_INTEGER noDelay = {};

            uint32_t periodMs = 0;
            while (m_updateThreadActive) {
                const auto newPeriodMs = vr::VRSettings()->GetInt32("driver_pimax_native", "async_tracking_period");
                if (newPeriodMs != periodMs) {
                    DriverLog("Setting async tracking period to %dms", newPeriodMs);
                    SetWaitableTimer(timer.get(), &noDelay, newPeriodMs, nullptr, nullptr, TRUE);
                    periodMs = newPeriodMs;
                }

                const bool waited = WaitForSingleObject(timer.get(), 100) == WAIT_OBJECT_0;
                const auto pvrNow = pvr_getTimeSeconds(m_pvr);
                TraceLoggingWriteTagged(local,
                                        "HmdDriver_UpdateThread",
                                        TLArg(waited, "Waited"),
                                        TLArg(periodMs, "PeriodMs"),
                                        TLArg(pvrNow, "PvrTime"));

                UpdatePvrState(pvrNow);
            }

            TraceLoggingWriteStop(local, "HmdDriver_UpdateThread");
        }

        vr::TrackedDeviceIndex_t m_deviceIndex = vr::k_unTrackedDeviceIndexInvalid;
        vr::VRInputComponentHandle_t m_components[ComponentCount] = {};

        pvrEnvHandle m_pvr = {};
        pvrHmdInfo m_pvrHmdInfo = {};
        pvrDisplayInfo m_pvrDisplayInfo = {};
        pvrEyeRenderInfo m_pvrEyeRenderInfo[pvrEye_Count] = {};
        pvrSessionHandle m_pvrSession = {};

        bool m_hasEyeTracking = false;

        uint32_t m_displayResolutionWidth = 0;
        uint32_t m_displayResolutionHeight = 0;
        float m_horizontalFovTangent = 1.f;
        float m_verticalFovTangent = 1.f;

        float m_currentIpd = 0.0633f;

        std::shared_mutex m_poseMutex;
        vr::DriverPose_t m_latestPose = {};

        std::unique_ptr<IControllerDriver> m_controllerDriver[2];

        std::unique_ptr<ICameraDriver> m_cameraDriver;

        vr::VRInputComponentHandle_t m_eyeTrackingComponent = 0;

        std::atomic_bool m_updateThreadActive;
        std::thread m_updateThread;
        std::atomic_bool m_useAsyncTracking;

        wil::unique_handle m_sharedFileHandle;
        shared::SharedMemory* m_sharedMemory = nullptr;
        bool m_inClickEvent = false;
        PROCESS_INFORMATION m_clientProcessInfo = {};

        wil::unique_handle m_sharedDistortionMapHandle[3];
        float* m_sharedDistortionMap[3] = {};

        ComPtr<ID3D11Device> m_d3dDummyDevice;
    };

} // namespace

namespace driver {
    std::unique_ptr<IHmdDriver> CreateHmdDriver(pvrEnvHandle pvr, pvrSessionHandle pvrSession) {
        return std::make_unique<HmdDriver>(pvr, pvrSession);
    }
} // namespace driver
