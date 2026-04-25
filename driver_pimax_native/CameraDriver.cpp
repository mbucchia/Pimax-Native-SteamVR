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
#include "ErrorHandling.h"
#include "Tracing.h"
#include "Utilities.h"

#include "vr_blockqueue.h"

using namespace driver;
using namespace util;

namespace {

    // Most API not officially documented.
    // See https://github.com/Rectus/openvr_camera_sim/blob/main/camera_component.cpp
    class CameraDriver : public ICameraDriver {
      public:
        CameraDriver(pvrEnvHandle pvr, pvrSessionHandle pvrSession) : m_pvr(pvr), m_pvrSession(pvrSession) {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_Ctor");

            QueryPerformanceFrequency(&m_qpcFrequency);

            m_numCameras = pvr_getVSTType(m_pvrSession) == pvrVSTTypeStereo ? 2 : 1;
            for (uint32_t i = 0; i < m_numCameras; i++) {
                CHECK_PVRCMD(pvr_getVSTCameraIntrinsics(m_pvrSession,
                                                        i,
                                                        &m_cameraResolutionWidth,
                                                        &m_cameraResolutionHeight,
                                                        &m_focalLength[i],
                                                        &m_principalPoint[i]));
                TraceLoggingWriteTagged(local,
                                        "CameraDriver_Ctor_Intrinsics",
                                        TLArg(i, "Index"),
                                        TLArg(m_cameraResolutionWidth, "Width"),
                                        TLArg(m_cameraResolutionHeight, "Height"),
                                        TLArg(ToString(m_focalLength[i]).c_str(), "FocalLength"),
                                        TLArg(ToString(m_principalPoint[i]).c_str(), "PricipalPoint"));

                CHECK_PVRCMD(pvr_getVSTCameraExtrinsics(m_pvrSession, i, &m_cameraToHmd[i]));
                TraceLoggingWriteTagged(local,
                                        "CameraDriver_Ctor_Extrinsics",
                                        TLArg(i, "Index"),
                                        TLArg(ToString(m_cameraToHmd[i]).c_str(), "Pose"));

                pvrVSTDistortionType distortionType = {};
                CHECK_PVRCMD(pvr_getVSTCameraDistortionParams(m_pvrSession, i, &distortionType, m_distortionParams[i]));
                TraceLoggingWriteTagged(
                    local,
                    "CameraDriver_Ctor_DistortionParams",
                    TLArg(i, "Index"),
                    TLArg((int)distortionType, "Type"),
                    TraceLoggingFloat32FixedArray(m_distortionParams[i], std::size(m_distortionParams[0]), "Params"));
            }

            TraceLoggingWriteStop(local, "CameraDriver_Ctor");
        }

        ~CameraDriver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_Dtor");

            TraceLoggingWriteStop(local, "CameraDriver_Dtor");
        }

        void Activate(vr::TrackedDeviceIndex_t deviceIndex) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_Activate", TLArg(deviceIndex, "ObjectId"));

            const vr::PropertyContainerHandle_t container =
                vr::VRProperties()->TrackedDeviceToPropertyContainer(deviceIndex);

            vr::VRProperties()->SetBoolProperty(container, vr::Prop_HasCamera_Bool, true);
            vr::VRProperties()->SetStringProperty(container, vr::Prop_CameraFirmwareDescription_String, "Pimax VST");
            vr::VRProperties()->SetInt32Property(container, vr::Prop_NumCameras_Int32, m_numCameras);
            vr::VRProperties()->SetInt32Property(
                container,
                vr::Prop_CameraFrameLayout_Int32,
                m_numCameras == 1
                    ? vr::EVRTrackedCameraFrameLayout_Mono
                    : (vr::EVRTrackedCameraFrameLayout_Stereo | vr::EVRTrackedCameraFrameLayout_HorizontalLayout));
            vr::VRProperties()->SetInt32Property(container, vr::Prop_CameraStreamFormat_Int32, vr::CVS_FORMAT_NV12);
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_CameraSupportsCompatibilityModes_Bool, false);
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_AllowCameraToggle_Bool, true);

            std::vector<vr::HmdMatrix34_t> transforms;
            for (uint32_t i = 0; i < m_numCameras; i++) {
                const PVR::Matrix4f cameraToHmd = PVR::Matrix4(PVR::Posef(m_cameraToHmd[i]));
                transforms.push_back(StoreHmdMatrix34(cameraToHmd));
            }
            vr::VRProperties()->SetProperty(container,
                                            vr::Prop_CameraToHeadTransform_Matrix34,
                                            &transforms[0],
                                            sizeof(vr::HmdMatrix34_t),
                                            vr::k_unHmdMatrix34PropertyTag);
            vr::VRProperties()->SetPropertyVector(
                container, vr::Prop_CameraToHeadTransforms_Matrix34_Array, vr::k_unHmdMatrix34PropertyTag, &transforms);

            std::vector<int32_t> distortionFunction;
            distortionFunction.push_back((int32_t)vr::VRDistortionFunctionType_Extended_FTheta);
            distortionFunction.push_back((int32_t)vr::VRDistortionFunctionType_Extended_FTheta);
            vr::VRProperties()->SetPropertyVector(container,
                                                  vr::Prop_CameraDistortionFunction_Int32_Array,
                                                  vr::k_unInt32PropertyTag,
                                                  &distortionFunction);
            std::vector<double> distortionCoeff;
            for (uint32_t i = 0; i < m_numCameras; i++) {
                for (uint32_t j = 0; j < std::size(m_distortionParams[0]); j++) {
                    distortionCoeff.push_back(m_distortionParams[i][j]);
                }
            }
            vr::VRProperties()->SetPropertyVector(container,
                                                  vr::Prop_CameraDistortionCoefficients_Float_Array,
                                                  vr::k_unFloatPropertyTag,
                                                  &distortionCoeff);

            {
                uint32_t nFrameBufferDataSize = 0;
                int nDefaultFrameQueueSize = 0;
                vr::ECameraVideoStreamFormat nVideoStreamFormat = GetCameraVideoStreamFormat();
                GetCameraFrameBufferingRequirements(&nDefaultFrameQueueSize, &nFrameBufferDataSize);
                vr::VRBlockQueue()->Create(&m_cameraBlockQueue,
                                           "/lighthouse/camera/raw_frames",
                                           nFrameBufferDataSize,
                                           512,
                                           nDefaultFrameQueueSize,
                                           0);
                DriverLog("Camera Block Queue: %lld", m_cameraBlockQueue);

                vr::VRPathsSet<UINT32>(m_cameraBlockQueue,
                                       vr::k_pathCameraWidth,
                                       m_cameraResolutionWidth * m_numCameras,
                                       vr::k_unInt32PropertyTag);
                vr::VRPathsSet<UINT32>(
                    m_cameraBlockQueue, vr::k_pathCameraHeight, m_cameraResolutionHeight, vr::k_unInt32PropertyTag);
                vr::VRPathsSet<UINT32>(
                    m_cameraBlockQueue, vr::k_pathCameraFormat, nVideoStreamFormat, vr::k_unInt32PropertyTag);
            }

            // Force RoomView by faking firmware versions that will pass the checks in vrcompositor.
            vr::VRProperties()->SetUint64Property(container, vr::Prop_FPGAVersion_Uint64, UINT64_MAX);
            vr::VRProperties()->SetUint64Property(container, vr::Prop_FirmwareVersion_Uint64, UINT64_MAX);
            vr::VRProperties()->SetUint64Property(container, vr::Prop_CameraFirmwareVersion_Uint64, UINT64_MAX);

            vr::VRProperties()->SetBoolProperty(container, vr::Prop_SupportsRoomViewDepthProjection_Bool, false);
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_AllowLightSourceFrequency_Bool, false);
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_Hmd_SupportsRoomViewDirect_Bool, false);
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_CameraSupportsCompatibilityModes_Bool, false);
            vr::VRProperties()->SetInt32Property(container, vr::Prop_CameraCompatibilityMode_Int32, 0);
            {
                std::vector<float> CameraWhiteBalance;
                CameraWhiteBalance.resize(8);
                CameraWhiteBalance[0] = 1.0f;
                CameraWhiteBalance[1] = 1.0f;
                CameraWhiteBalance[2] = 1.0f;
                CameraWhiteBalance[4] = 1.0f;
                CameraWhiteBalance[5] = 1.0f;
                CameraWhiteBalance[6] = 1.0f;
                vr::VRProperties()->SetPropertyVector(container,
                                                      vr::Prop_CameraWhiteBalance_Vector4_Array,
                                                      vr::k_unHmdVector4PropertyTag,
                                                      &CameraWhiteBalance);
            }

            TraceLoggingWriteStop(local, "CameraDriver_Activate");
        }

        void Deactivate(vr::TrackedDeviceIndex_t deviceIndex) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_Deactivate", TLArg(deviceIndex, "ObjectId"));

            TraceLoggingWriteStop(local, "CameraDriver_CameraDriver_Deactivate");
        }

        bool GetCameraFrameDimensions(vr::ECameraVideoStreamFormat nVideoStreamFormat,
                                      uint32_t* pWidth,
                                      uint32_t* pHeight) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "CameraDriver_GetCameraFrameDimensions",
                                   TLArg((uint32_t)nVideoStreamFormat, "VideoStreamFormat"));

            bool status = false;
            if (nVideoStreamFormat == vr::CVS_FORMAT_NV12 || nVideoStreamFormat == vr::CVS_FORMAT_UNKNOWN) {
                *pWidth = m_cameraResolutionWidth * m_numCameras;
                *pHeight = m_cameraResolutionHeight;
                status = true;
            }

            TraceLoggingWriteStop(local,
                                  "CameraDriver_GetCameraFrameDimensions",
                                  TLArg(status, "IsFormatSupported"),
                                  TLArg(*pWidth, "Width"),
                                  TLArg(*pHeight, "Height"));

            return status;
        }

        bool GetCameraFrameBufferingRequirements(int* pDefaultFrameQueueSize, uint32_t* pFrameBufferDataSize) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_GetCameraFrameBufferingRequirements");

            // Assume NV12.
            uint32_t nFrameBufferDataSize = (m_cameraResolutionWidth * m_cameraResolutionHeight * m_numCameras * 3) / 2;
            nFrameBufferDataSize = sizeof(vr::CameraVideoStreamFrame_t) + nFrameBufferDataSize;
            nFrameBufferDataSize = ((nFrameBufferDataSize + 15) / 16) * 16;
            if (pDefaultFrameQueueSize) {
                *pDefaultFrameQueueSize = 2;
            }
            if (pFrameBufferDataSize) {
                *pFrameBufferDataSize = nFrameBufferDataSize;
            }

            TraceLoggingWriteStop(local,
                                  "CameraDriver_GetCameraFrameBufferingRequirements",
                                  TLArg(2, "DefaultFrameQueueSize"),
                                  TLArg(nFrameBufferDataSize, "FrameBufferDataSize"));

            return true;
        }

        bool SetCameraFrameBuffering(int nFrameBufferCount,
                                     void** ppFrameBuffers,
                                     uint32_t nFrameBufferDataSize) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "CameraDriver_SetCameraFrameBuffering",
                                   TLArg(nFrameBufferCount, "FrameBufferCount"),
                                   TLArg(nFrameBufferDataSize, "FrameBufferDataSize"));

            bool status = false;
            if (nFrameBufferCount == 0) {
                m_cameraBuffer[0] = m_cameraBuffer[1] = nullptr;
                status = true;
            } else if (nFrameBufferCount > 1) {
                for (uint32_t i = 0; i < 2; i++) {
                    m_cameraBuffer[i] = (vr::CameraVideoStreamFrame_t*)ppFrameBuffers[i];
                    *m_cameraBuffer[i] = {};
                    m_cameraBuffer[i]->m_flFrameDeliveryRate = 1 / 30.0;
                    m_cameraBuffer[i]->m_nBufferCount = 2;
                    m_cameraBuffer[i]->m_nBufferIndex = (uint32_t)i;
                    m_cameraBuffer[i]->m_nWidth = m_cameraResolutionWidth * m_numCameras;
                    m_cameraBuffer[i]->m_nHeight = m_cameraResolutionHeight;
                    m_cameraBuffer[i]->m_nStreamFormat = vr::CVS_FORMAT_NV12;
                    m_cameraBuffer[i]->m_nImageDataSize =
                        (m_cameraResolutionWidth * m_cameraResolutionHeight * m_numCameras * 3) / 2;
                    m_cameraBuffer[i]->m_pImageData = (uint64_t)(m_cameraBuffer[i] + 1);
                    m_cameraBuffer[i]->m_RawTrackedDevicePose.bDeviceIsConnected = true;
                    m_cameraBuffer[i]->m_RawTrackedDevicePose.bPoseIsValid = false;
                    m_cameraBuffer[i]->m_RawTrackedDevicePose.mDeviceToAbsoluteTracking =
                        StoreHmdMatrix34(DirectX::XMMatrixIdentity());
                    m_cameraBuffer[i]->m_RawTrackedDevicePose.eTrackingResult = vr::TrackingResult_Running_OK;
                }
                status = true;
            }

            TraceLoggingWriteStop(local, "CameraDriver_SetCameraFrameBuffering");

            return status;
        }

        bool SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat nVideoStreamFormat) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(
                local, "CameraDriver_SetCameraVideoStreamFormat", TLArg((int)nVideoStreamFormat, "VideoStreamFormat"));

            bool status = false;
            if (nVideoStreamFormat == vr::CVS_FORMAT_NV12) {
                status = true;
            }

            TraceLoggingWriteStop(local, "CameraDriver_SetCameraVideoStreamFormat", TLArg(status, "IsFormatSupported"));

            return status;
        }

        vr::ECameraVideoStreamFormat GetCameraVideoStreamFormat() override {
            return vr::CVS_FORMAT_NV12;
        }

        bool StartVideoStream() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_StartVideoStream");

            QueryPerformanceCounter(&m_cameraStartTime);
            m_cameraFrameIndex = 0;
            m_cameraActive = true;
            m_cameraPaused = false;

            TraceLoggingWriteStop(local, "CameraDriver_StartVideoStream");

            return true;
        }

        void StopVideoStream() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_StopVideoStream");

            m_cameraActive = m_cameraPaused = false;

            TraceLoggingWriteStop(local, "CameraDriver_StopVideoStream");
        }

        bool IsVideoStreamActive(bool* pbPaused, float* pflElapsedTime) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_IsVideoStreamActive");

            *pbPaused = m_cameraActive && m_cameraPaused;
            LARGE_INTEGER now;
            QueryPerformanceCounter(&now);
            *pflElapsedTime =
                m_cameraActive ? (now.QuadPart - m_cameraStartTime.QuadPart) / (float)m_qpcFrequency.QuadPart : 0.f;

            TraceLoggingWriteStop(local,
                                  "CameraDriver_IsVideoStreamActive",
                                  TLArg(m_cameraActive, "IsActive"),
                                  TLArg(*pbPaused, "IsPaused"),
                                  TLArg(*pflElapsedTime, "ElapsedTime"));

            return m_cameraActive;
        }

        const vr::CameraVideoStreamFrame_t* GetVideoStreamFrame() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_GetVideoStreamFrame");

            // Not supported.

            TraceLoggingWriteStop(local, "CameraDriver_GetVideoStreamFrame");

            return nullptr;
        }

        void ReleaseVideoStreamFrame(const vr::CameraVideoStreamFrame_t* pFrameImage) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_ReleaseVideoStreamFrame");

            TraceLoggingWriteStop(local, "CameraDriver_ReleaseVideoStreamFrame");
        }

        bool SetAutoExposure(bool bEnable) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_SetAutoExposure", TLArg(bEnable, "Enable"));

            // Not supported.

            TraceLoggingWriteStop(local, "CameraDriver_SetAutoExposure");

            return false;
        }

        bool PauseVideoStream() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_PauseVideoStream");

            m_cameraPaused = true;

            TraceLoggingWriteStop(local, "CameraDriver_PauseVideoStream");

            return true;
        }

        bool ResumeVideoStream() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_ResumeVideoStream");

            m_cameraPaused = false;

            TraceLoggingWriteStop(local, "CameraDriver_ResumeVideoStream");

            return true;
        }

        bool GetCameraDistortion(
            uint32_t nCameraIndex, float flInputU, float flInputV, float* pflOutputU, float* pflOutputV) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "CameraDriver_GetCameraDistortion",
                                   TLArg(nCameraIndex, "CameraIndex"),
                                   TLArg(flInputU, "U"),
                                   TLArg(flInputV, "V"));

            // Taken as-is from https://github.com/Rectus/openvr_camera_sim/blob/main/camera_component.cpp

            const double focalX = m_focalLength[nCameraIndex].x / (double)m_cameraResolutionWidth;
            const double focalY = m_focalLength[nCameraIndex].y / (double)m_cameraResolutionHeight;

            const double centerX = m_principalPoint[nCameraIndex].x / (double)m_cameraResolutionWidth - 0.5;
            const double centerY = m_principalPoint[nCameraIndex].y / (double)m_cameraResolutionHeight - 0.5;

            double UScaled = (flInputU - 0.5) * 2.0 / focalX;
            double VScaled = (flInputV - 0.5) * 2.0 / focalY;

            double radius = sqrt(UScaled * UScaled + VScaled * VScaled);

            double theta = atan(radius);

            double thetaD = theta + m_distortionParams[nCameraIndex][0] * pow(theta, 3) +
                            m_distortionParams[nCameraIndex][1] * pow(theta, 5) +
                            m_distortionParams[nCameraIndex][2] * pow(theta, 7) +
                            m_distortionParams[nCameraIndex][3] * pow(theta, 9);

            double radialFactor = thetaD / radius;

            *pflOutputU = (float)(UScaled * radialFactor * focalX + centerX + 0.5);
            *pflOutputV = (float)(VScaled * radialFactor * focalY + centerY + 0.5);

            TraceLoggingWriteStop(
                local, "CameraDriver_GetCameraDistortion", TLArg(*pflOutputU, "U"), TLArg(*pflOutputV, "V"));

            return true;
        }

        bool GetCameraProjection(uint32_t nCameraIndex,
                                 vr::EVRTrackedCameraFrameType eFrameType,
                                 float flZNear,
                                 float flZFar,
                                 vr::HmdMatrix44_t* pProjection) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "CameraDriver_GetCameraProjection",
                                   TLArg(nCameraIndex, "CameraIndex"),
                                   TLArg((uint32_t)eFrameType, "FrameType"),
                                   TLArg(flZNear, "NearZ"),
                                   TLArg(flZFar, "FarZ"));

            // Taken as-is from https://github.com/Rectus/openvr_camera_sim/blob/main/camera_component.cpp

            *pProjection = {};
            pProjection->m[0][0] = m_focalLength[nCameraIndex].x / (float)m_cameraResolutionWidth / 2.0f;
            pProjection->m[1][1] = m_focalLength[nCameraIndex].y / (float)m_cameraResolutionHeight;
            pProjection->m[0][2] = m_principalPoint[nCameraIndex].x / (float)m_cameraResolutionWidth;
            pProjection->m[1][2] = m_principalPoint[nCameraIndex].y / (float)m_cameraResolutionHeight - 0.5f;
            pProjection->m[2][2] = -flZFar / (flZFar - flZNear);
            pProjection->m[2][3] = -flZFar * flZNear / (flZFar - flZNear);
            pProjection->m[3][2] = -1;

            TraceLoggingWriteStop(local, "CameraDriver_GetCameraProjection");

            return true;
        }

        bool SetFrameRate(int nISPFrameRate, int nSensorFrameRate) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "CameraDriver_SetFrameRate",
                                   TLArg(nISPFrameRate, "IspFrameRate"),
                                   TLArg(nSensorFrameRate, "SensorFrameRate"));

            // We cannot service this request. Pretend success.

            TraceLoggingWriteStop(local, "CameraDriver_SetFrameRate");

            return true;
        }

        bool SetCameraVideoSinkCallback(vr::ICameraVideoSinkCallback* pCameraVideoSinkCallback) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "CameraDriver_SetCameraVideoSinkCallback",
                                   TLPArg(pCameraVideoSinkCallback, "CameraVideoSinkCallback"));

            m_cameraSinkCallback = pCameraVideoSinkCallback;

            TraceLoggingWriteStop(local, "CameraDriver_SetCameraVideoSinkCallback");

            return true;
        }

        bool GetCameraCompatibilityMode(vr::ECameraCompatibilityMode* pCameraCompatibilityMode) override {
            *pCameraCompatibilityMode = vr::CAMERA_COMPAT_MODE_ISO_30FPS;
            return true;
        }

        bool SetCameraCompatibilityMode(vr::ECameraCompatibilityMode nCameraCompatibilityMode) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_SetCameraCompatibilityMode");

            bool status = false;
            if (nCameraCompatibilityMode == vr::CAMERA_COMPAT_MODE_ISO_30FPS) {
                status = true;
            }

            TraceLoggingWriteStop(local, "CameraDriver_SetCameraCompatibilityMode", TLArg(status, "IsModeSupported"));

            return status;
        }

        bool GetCameraFrameBounds(vr::EVRTrackedCameraFrameType eFrameType,
                                  uint32_t* pLeft,
                                  uint32_t* pTop,
                                  uint32_t* pWidth,
                                  uint32_t* pHeight) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(
                local, "CameraDriver_GetCameraFrameBounds", TLArg((uint32_t)eFrameType, "FrameType"));

            *pLeft = *pTop = 0;
            *pWidth = m_cameraResolutionWidth * m_numCameras;
            *pHeight = m_cameraResolutionHeight;

            TraceLoggingWriteStop(
                local, "CameraDriver_GetCameraFrameBounds", TLArg(*pWidth, "Width"), TLArg(*pHeight, "Height"));

            return true;
        }

        bool GetCameraIntrinsics(uint32_t nCameraIndex,
                                 vr::EVRTrackedCameraFrameType eFrameType,
                                 vr::HmdVector2_t* pFocalLength,
                                 vr::HmdVector2_t* pCenter,
                                 vr::EVRDistortionFunctionType* peDistortionType,
                                 double rCoefficients[vr::k_unMaxDistortionFunctionParameters]) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_GetCameraIntrinsics");

            (*pFocalLength).v[0] = m_focalLength[nCameraIndex].x;
            (*pFocalLength).v[1] = m_focalLength[nCameraIndex].y;

            (*pCenter).v[0] = m_principalPoint[nCameraIndex].x;
            (*pCenter).v[1] = m_principalPoint[nCameraIndex].y;

            *peDistortionType = vr::EVRDistortionFunctionType::VRDistortionFunctionType_FTheta;
            for (uint32_t i = 0; i < std::size(m_distortionParams[0]); i++) {
                rCoefficients[i] = m_distortionParams[nCameraIndex][i];
            }

            TraceLoggingWriteStop(local, "CameraDriver_GetCameraIntrinsics");

            return true;
        }

        void RunFrame() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CameraDriver_RunFrame");

            if (m_cameraActive && !m_cameraPaused && m_cameraBuffer[m_cameraBufferIndex]) {
                // Flip buffer.
                m_cameraBufferIndex ^= 1;

                // Retrieve and convert the image from PVR.
                const uint32_t frameSize = (m_cameraResolutionWidth * m_cameraResolutionHeight * m_numCameras * 3) / 2;
                pvrVSTStreamFrame frame = {};
                CHECK_PVRCMD(pvr_getVSTStreamFrame(m_pvrSession, m_pvrFrameIndex, &frame));
                TraceLoggingWriteTagged(local,
                                        "CameraDriver_RunFrame_StreamFrame",
                                        TLArg(frame.frameIdx, "FrameIndex"),
                                        TLArg(frame.width, "Width"),
                                        TLArg(frame.height, "Height"),
                                        TLArg(frame.stride, "Stride"),
                                        TLArg(frame.exposureTime, "Exporsure"));
                m_pvrFrameIndex = frame.frameIdx + 1;

                switch (pvr_getVSTStreamFormat(m_pvrSession)) {
                case pvrVST_FORMAT_RAW8:
                    {
                        uint8_t* y_plane = (uint8_t*)m_cameraBuffer[m_cameraBufferIndex]->m_pImageData;
                        memcpy((uint8_t*)m_cameraBuffer[m_cameraBufferIndex]->m_pImageData,
                               frame.buffer,
                               m_cameraResolutionWidth * m_cameraResolutionHeight * m_numCameras);
                        uint8_t* uv_plane = (uint8_t*)m_cameraBuffer[m_cameraBufferIndex]->m_pImageData +
                                            m_cameraResolutionWidth * m_cameraResolutionHeight * m_numCameras;
                        memset(uv_plane, 128, (m_cameraResolutionWidth * m_cameraResolutionHeight * m_numCameras) / 2);
                    }
                    break;
                case pvrVST_FORMAT_NV12:
                    memcpy((uint8_t*)m_cameraBuffer[m_cameraBufferIndex]->m_pImageData, frame.buffer, frameSize);
                    break;
                }

                // Push the frame to SteamVR.
                LARGE_INTEGER now;
                QueryPerformanceCounter(&now);
                m_cameraBuffer[m_cameraBufferIndex]->m_flFrameElapsedTime =
                    (now.QuadPart - m_cameraStartTime.QuadPart) / (double)m_qpcFrequency.QuadPart;
                m_cameraBuffer[m_cameraBufferIndex]->m_nFrameSequence = (uint32_t)m_cameraFrameIndex;

                void* pvBuffer = nullptr;
                vr::PropertyContainerHandle_t ulBlockContainer = 0;
                bool acquired;
                {
                    TraceLocalActivity(acquireWriteBlock);
                    TraceLoggingWriteStart(acquireWriteBlock, "CameraDriver_RunFrame_AcquireWriteOnlyBlock");
                    acquired =
                        vr::VRBlockQueue()->AcquireWriteOnlyBlock(m_cameraBlockQueue, &ulBlockContainer, &pvBuffer) ==
                        vr::EBlockQueueError_BlockQueueError_None;
                    TraceLoggingWriteStop(
                        acquireWriteBlock, "CameraDriver_RunFrame_AcquireWriteOnlyBlock", TLArg(acquired, "Acquired"));
                }
                if (acquired) {
                    memcpy(pvBuffer, (void*)m_cameraBuffer[m_cameraBufferIndex]->m_pImageData, frameSize);
                    vr::VRPathsSet<UINT32>(
                        ulBlockContainer, vr::k_pathCameraFrameSize, frameSize, vr::k_unInt32PropertyTag);

                    vr::VRPathsSet<UINT64>(
                        ulBlockContainer, vr::k_pathCameraFrameSequence, m_cameraFrameIndex, vr::k_unUint64PropertyTag);

                    vr::VRPathsSet<double>(ulBlockContainer,
                                           vr::k_pathCameraFrameTimeMonotonic,
                                           now.QuadPart / (double)m_qpcFrequency.QuadPart,
                                           vr::k_unDoublePropertyTag);
                    vr::VRPathsSet<UINT64>(ulBlockContainer,
                                           vr::k_pathCameraFrameServerTimeTicks,
                                           now.QuadPart,
                                           vr::k_unUint64PropertyTag);
                    vr::VRPathsSet<double>(
                        ulBlockContainer, vr::k_pathCameraFrameDeliveryRate, 1 / 30.0, vr::k_unDoublePropertyTag);
                    vr::VRPathsSet<double>(ulBlockContainer,
                                           vr::k_pathCameraFrameElapsedTime,
                                           m_cameraBuffer[m_cameraBufferIndex]->m_flFrameElapsedTime,
                                           vr::k_unDoublePropertyTag);

                    vr::VRBlockQueue()->ReleaseWriteOnlyBlock(m_cameraBlockQueue, ulBlockContainer);
                }

                m_cameraFrameIndex++;

                if (m_cameraSinkCallback) {
                    m_cameraSinkCallback->OnCameraVideoSinkCallback();
                }
            }

            TraceLoggingWriteStop(local, "CameraDriver_RunFrame");
        }

      private:
        pvrEnvHandle m_pvr = {};
        pvrSessionHandle m_pvrSession = {};

        LARGE_INTEGER m_qpcFrequency = {};

        uint32_t m_numCameras = 0;
        uint32_t m_cameraResolutionWidth = 0;
        uint32_t m_cameraResolutionHeight = 0;
        pvrVector2f m_focalLength[2] = {};
        pvrVector2f m_principalPoint[2] = {};
        pvrPosef m_cameraToHmd[2] = {};
        float m_distortionParams[2][8] = {};

        uint32_t m_pvrFrameIndex = 0;

        vr::PropertyContainerHandle_t m_cameraBlockQueue = vr::k_ulInvalidPropertyContainer;
        bool m_cameraActive = false;
        bool m_cameraPaused = false;
        LARGE_INTEGER m_cameraStartTime = {};
        uint64_t m_cameraFrameIndex = 0;

        vr::CameraVideoStreamFrame_t* m_cameraBuffer[2] = {nullptr, nullptr};
        std::atomic<size_t> m_cameraBufferIndex = 1;
        vr::ICameraVideoSinkCallback* m_cameraSinkCallback = nullptr;
    };

} // namespace

namespace driver {
    std::unique_ptr<ICameraDriver> CreateCameraDriver(pvrEnvHandle pvr, pvrSessionHandle pvrSession) {
        return std::make_unique<CameraDriver>(pvr, pvrSession);
    }
} // namespace driver
