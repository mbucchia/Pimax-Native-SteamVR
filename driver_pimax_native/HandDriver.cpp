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

#include "HandDriver.h"
#include "ErrorHandling.h"
#include "Tracing.h"
#include "Utilities.h"

using namespace driver;
using namespace util;

namespace {
    enum HandSkeletonBone {
        BoneRoot = 0,
        BoneWrist,
        BoneThumb0,
        BoneThumb1,
        BoneThumb2,
        BoneThumb3,
        BoneIndexFinger0,
        BoneIndexFinger1,
        BoneIndexFinger2,
        BoneIndexFinger3,
        BoneIndexFinger4,
        BoneMiddleFinger0,
        BoneMiddleFinger1,
        BoneMiddleFinger2,
        BoneMiddleFinger3,
        BoneMiddleFinger4,
        BoneRingFinger0,
        BoneRingFinger1,
        BoneRingFinger2,
        BoneRingFinger3,
        BoneRingFinger4,
        BonePinkyFinger0,
        BonePinkyFinger1,
        BonePinkyFinger2,
        BonePinkyFinger3,
        BonePinkyFinger4,
        BoneAux_Thumb,
        BoneAux_IndexFinger,
        BoneAux_MiddleFinger,
        BoneAux_RingFinger,
        BoneAux_PinkyFinger,

        BoneCount
    };

    enum Component {
        ComponentSkeleton,
        ComponentIndexPinch,
        ComponentMenu,

        ComponentCount,
    };

    class HandDriver : public IHandDriver {
      public:
        HandDriver(pvrEnvHandle pvr, pvrSessionHandle pvrSession, vr::ETrackedControllerRole role)
            : m_role(role), m_pvr(pvr), m_pvrSession(pvrSession) {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "HandDriver_Ctor",
                                   TLArg(m_role == vr::TrackedControllerRole_LeftHand ? "Left" : "Right", "Role"));

            m_serialNumber = m_role == vr::TrackedControllerRole_LeftHand ? "HAND_LEFT" : "HAND_RIGHT";

            TraceLoggingWriteStop(local, "HandDriver_Ctor");
        }

        ~HandDriver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HandDriver_Dtor", TLArg(m_serialNumber.c_str(), "SerialNumber"));

            TraceLoggingWriteStop(local, "HandDriver_Dtor");
        }

        vr::EVRInitError Activate(uint32_t unObjectId) override {
            TraceLocalActivity(local);
            const bool isLeft = m_role == vr::TrackedControllerRole_LeftHand;
            TraceLoggingWriteStart(
                local, "HandDriver_Activate", TLArg(unObjectId, "ObjectId"), TLArg(isLeft ? "Left" : "Right", "Role"));

            m_deviceIndex = unObjectId;

            const vr::PropertyContainerHandle_t container =
                vr::VRProperties()->TrackedDeviceToPropertyContainer(m_deviceIndex);

            // Fill out all the properties to use SteamLink's hand tracking profile.
            vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, m_role);
            vr::VRProperties()->SetStringProperty(container, vr::Prop_TrackingSystemName_String, "vrlink");

            vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "Hand Tracking");
            vr::VRProperties()->SetStringProperty(
                container, vr::Prop_ControllerType_String, "svl_hand_interaction_augmented");
            vr::VRProperties()->SetStringProperty(container,
                                                  vr::Prop_ModelNumber_String,
                                                  isLeft ? "VRLink Hand Tracker (Left Hand)"
                                                         : "VRLink Hand Tracker (Right Hand)");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_SerialNumber_String, GetSerialNumber());
            vr::VRProperties()->SetStringProperty(container,
                                                  vr::Prop_InputProfilePath_String,
                                                  "{vrlink}/input/svl_hand_interaction_augmented_input_profile.json");
            vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceHasNoIMU_Bool, true);

            vr::VRProperties()->SetStringProperty(
                container, vr::Prop_RenderModelName_String, "{vrlink}/rendermodels/shuttlecock");

            vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerHandSelectionPriority_Int32, 100);

            // clang-format off
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceOff_String, isLeft ? "{vrlink}/icons/left_handtracking_off.png" : "{vrlink}/icons/right_handtracking_off.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceSearching_String, isLeft ? "{vrlink}/icons/left_handtracking_searching.png" : "{vrlink}/icons/right_handtracking_searching.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceReady_String, isLeft ? "{vrlink}/icons/left_handtracking_ready.png" : "{vrlink}/icons/right_handtracking_ready.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceReadyAlert_String, isLeft ? "{vrlink}/icons/left_handtracking_ready_info.png" : "{vrlink}/icons/right_handtracking_ready_info.png");
            // clang-format on

            // Create all the input components.
            vr::VRDriverInput()->CreateSkeletonComponent(container,
                                                         isLeft ? "/input/skeleton/left" : "/input/skeleton/right",
                                                         isLeft ? "/skeleton/hand/left" : "/skeleton/hand/right",
                                                         "/pose/raw",
                                                         vr::VRSkeletalTracking_Full,
                                                         nullptr,
                                                         0,
                                                         &m_components[ComponentSkeleton]);
            vr::VRDriverInput()->CreateScalarComponent(container,
                                                       "/input/index_pinch/value",
                                                       &m_components[ComponentIndexPinch],
                                                       vr::VRScalarType_Absolute,
                                                       vr::VRScalarUnits_NormalizedOneSided);
            vr::VRDriverInput()->CreateBooleanComponent(container, "/input/system/click", &m_components[ComponentMenu]);

            ApplySettingsChanges();

            TraceLoggingWriteStop(local, "HandDriver_Activate");

            return vr::VRInitError_None;
        }

        void Deactivate() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HandDriver_Deactivate", TLArg(m_deviceIndex, "ObjectId"));

            m_deviceIndex = vr::k_unTrackedDeviceIndexInvalid;

            TraceLoggingWriteStop(local, "HandDriver_Deactivate");
        }

        void EnterStandby() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HandDriver_EnterStandby", TLArg(m_deviceIndex, "ObjectId"));

            TraceLoggingWriteStop(local, "HandDriver_EnterStandby");
        }

        void* GetComponent(const char* pchComponentNameAndVersion) override {
            return nullptr;
        }

        vr::DriverPose_t GetPose() override {
            // This method is not used by SteamVR.
            return {};
        }

        void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {
            if (unResponseBufferSize >= 1) {
                pchResponseBuffer[0] = 0;
            }
        }

        void ApplySettingsChanges() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HandDriver_ApplySettingsChanges", TLArg(m_deviceIndex, "ObjectId"));

            m_useMenuGesture = vr::VRSettings()->GetBool("driver_pimax_native", "use_tap_wrist");

            TraceLoggingWriteStop(local, "HandDriver_ApplySettingsChanges");
        }

        void RunFrame() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HandDriver_RunFrame", TLArg(m_deviceIndex, "ObjectId"));

            TraceLoggingWriteStop(local, "HandDriver_RunFrame", TLArg(m_deviceIndex, "ObjectId"));
        }

        void UpdateTrackingState(const pvrHandTrackingInputState& state) override {
            TraceLocalActivity(local);
            const int side = m_role == vr::TrackedControllerRole_LeftHand ? 0 : 1;
            TraceLoggingWriteStart(local, "HandDriver_UpdateTrackingState", TLArg(m_deviceIndex, "ObjectId"));

            vr::DriverPose_t pose = {};
            pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = pose.qRotation.w = 1.0;

            if (m_deviceIndex != vr::k_unTrackedDeviceIndexInvalid) {
                pose.deviceIsConnected = m_isConnected;
                pose.result = vr::TrackingResult_Running_OutOfRange;

                const bool force3Dof = vr::VRSettings()->GetBool("driver_pimax_native", "force_3dof");
                if (m_isConnected && state.IsValid[side] && (state.HandStatus[side] & pvrStatus_OrientationTracked) &&
                    !force3Dof) {
                    pose.vecPosition[0] = state.PointerPose[side].Position.x;
                    pose.vecPosition[1] = state.PointerPose[side].Position.y;
                    pose.vecPosition[2] = state.PointerPose[side].Position.z;
                    pose.qRotation.x = state.PointerPose[side].Orientation.x;
                    pose.qRotation.y = state.PointerPose[side].Orientation.y;
                    pose.qRotation.z = state.PointerPose[side].Orientation.z;
                    pose.qRotation.w = state.PointerPose[side].Orientation.w;

                    DirectX::XMVECTOR position, orientation, scale;
                    DirectX::XMMatrixDecompose(&scale, &orientation, &position, m_poseOffset);
                    pose.vecDriverFromHeadTranslation[0] = DirectX::XMVectorGetX(position);
                    pose.vecDriverFromHeadTranslation[1] = DirectX::XMVectorGetY(position);
                    pose.vecDriverFromHeadTranslation[2] = DirectX::XMVectorGetZ(position);
                    pose.qDriverFromHeadRotation.x = DirectX::XMVectorGetX(orientation);
                    pose.qDriverFromHeadRotation.y = DirectX::XMVectorGetY(orientation);
                    pose.qDriverFromHeadRotation.z = DirectX::XMVectorGetZ(orientation);
                    pose.qDriverFromHeadRotation.w = DirectX::XMVectorGetW(orientation);

                    const auto pvrNow = pvr_getTimeSeconds(m_pvr);
                    pose.poseTimeOffset = state.TimeInSeconds - pvrNow;

                    pose.poseIsValid = true;
                    pose.result = vr::TrackingResult_Running_OK;
                }

                vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));

                // Update the skeleton.
                if (pose.poseIsValid) {
                    pvrSkeletalData skeletalData = {};
                    CHECK_PVRCMD(pvr_getHandTrackingSkeletalData(
                        m_pvrSession, side == 0 ? pvrHand_Left : pvrHand_Right, state.TimeInSeconds, &skeletalData));

                    vr::VRBoneTransform_t transforms[BoneCount]{};
                    for (uint32_t i = 0; i < BoneCount; i++) {
                        transforms[i].position.v[0] = skeletalData.boneTransforms[i].Position.x;
                        transforms[i].position.v[1] = skeletalData.boneTransforms[i].Position.y;
                        transforms[i].position.v[2] = skeletalData.boneTransforms[i].Position.z;
                        transforms[i].orientation.x = skeletalData.boneTransforms[i].Orientation.x;
                        transforms[i].orientation.y = skeletalData.boneTransforms[i].Orientation.y;
                        transforms[i].orientation.z = skeletalData.boneTransforms[i].Orientation.z;
                        transforms[i].orientation.w = skeletalData.boneTransforms[i].Orientation.w;
                    }

                    vr::VRDriverInput()->UpdateSkeletonComponent(m_components[ComponentSkeleton],
                                                                 vr::VRSkeletalMotionRange_WithController,
                                                                 transforms,
                                                                 BoneCount);
                    vr::VRDriverInput()->UpdateSkeletonComponent(m_components[ComponentSkeleton],
                                                                 vr::VRSkeletalMotionRange_WithoutController,
                                                                 transforms,
                                                                 BoneCount);

                    // Basic gesture detection.
                    const auto jointActionValue = [](const PVR::Posef& joint1, const PVR::Posef& joint2) {
                        static constexpr float NearDistance = 0.01f;
                        static constexpr float FarDistance = 0.02f;

                        // Compute the distance between the two joints.
                        const float distance = std::max((joint1.Translation - joint2.Translation).Length(), 0.f);

                        return 1.f - (std::clamp(distance, NearDistance, FarDistance) - NearDistance) /
                                         (FarDistance - NearDistance);
                    };
                    const PVR::Posef wrist(skeletalData.boneTransforms[BoneWrist]);
                    const PVR::Posef otherHandPointer(state.PointerPose[side ^ 1]);
                    const PVR::Posef thumbTip(skeletalData.boneTransforms[BoneThumb3]);
                    const PVR::Posef indexTip(skeletalData.boneTransforms[BoneIndexFinger4]);

                    vr ::VRDriverInput()->UpdateScalarComponent(
                        m_components[ComponentIndexPinch], jointActionValue(thumbTip, indexTip), 0.0);
                    if (m_useMenuGesture) {
                        vr ::VRDriverInput()->UpdateBooleanComponent(
                            m_components[ComponentMenu], jointActionValue(wrist, otherHandPointer) > 0.75f, 0.0);
                    }
                }
            }

            TraceLoggingWriteStop(
                local, "HandDriver_UpdateTrackingState", TLArg(pose.poseTimeOffset, "PoseTimeOffset"));
        }

        void Connect() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HandDriver_Connect", TLArg(m_deviceIndex, "ObjectId"));

            m_isConnected = true;

            TraceLoggingWriteStop(local, "HandDriver_Connect");
        }

        void Disconnect() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "HandDriver_Disconnect", TLArg(m_deviceIndex, "ObjectId"));

            vr::DriverPose_t pose = {};
            pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = pose.qRotation.w = 1.0;
            pose.result = vr::TrackingResult_Running_OutOfRange;
            m_isConnected = false;

            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));

            TraceLoggingWriteStop(local, "HandDriver_Disconnect");
        }

        bool IsConnected() const override {
            return m_isConnected;
        }

        vr::TrackedDeviceIndex_t GetDeviceIndex() const {
            return m_deviceIndex;
        }

        const char* GetSerialNumber() const {
            return m_serialNumber.c_str();
        }

      private:
        const vr::ETrackedControllerRole m_role;
        pvrEnvHandle m_pvr = {};
        pvrSessionHandle m_pvrSession = {};

        vr::TrackedDeviceIndex_t m_deviceIndex = vr::k_unTrackedDeviceIndexInvalid;
        vr::VRInputComponentHandle_t m_components[ComponentCount] = {};

        std::string m_serialNumber;
        bool m_isConnected = false;

        DirectX::XMMATRIX m_poseOffset = DirectX::XMMatrixIdentity();

        bool m_useMenuGesture = false;
    };
} // namespace

namespace driver {
    std::unique_ptr<IHandDriver> CreateHandDriver(pvrEnvHandle pvr,
                                                  pvrSessionHandle pvrSession,
                                                  vr::ETrackedControllerRole role) {
        return std::make_unique<HandDriver>(pvr, pvrSession, role);
    }
} // namespace driver
