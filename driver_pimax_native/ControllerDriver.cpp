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

#include "ControllerDriver.h"
#include "ErrorHandling.h"
#include "Tracing.h"
#include "Utilities.h"

using namespace driver;
using namespace util;

namespace {
    enum Components {
        ComponentTrigger,
        ComponentTriggerClick,
        ComponentTriggerTouch,
        ComponentGrip,
        ComponentGripClick,
        ComponentThumbstickX,
        ComponentThumbstickY,
        ComponentThumbstickClick,
        ComponentThumbstickTouch,
        ComponentThumbrestTouch,
        ComponentButton1,
        ComponentButton1Touch,
        ComponentButton2,
        ComponentButton2Touch,
        ComponentMenu,

        ComponentCount,
    };

    class ControllerDriver : public IControllerDriver {
      public:
        ControllerDriver(pvrEnvHandle pvr, pvrSessionHandle pvrSession, vr::ETrackedControllerRole role)
            : m_role(role), m_pvr(pvr), m_pvrSession(pvrSession) {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "ControllerDriver_Ctor",
                                   TLArg(m_role == vr::TrackedControllerRole_LeftHand ? "Left" : "Right", "Role"));

            m_serialNumber = pvr_getTrackedDeviceStringPropertyHelper(m_pvrSession,
                                                                      m_role == vr::TrackedControllerRole_LeftHand == 0
                                                                          ? pvrTrackedDevice_LeftController
                                                                          : pvrTrackedDevice_RightController,
                                                                      pvrTrackedDeviceProp_Serial_String);
            // TODO: Troubleshoot this - doesn't seem to work everytime. Sometimes, the value is empty.
            // We fallback to a dummy name when it happens.
            if (m_serialNumber.empty()) {
                m_serialNumber = m_role == vr::TrackedControllerRole_LeftHand ? "DEFAULTLEFT" : "DEFAULTRIGHT";
            }

            // Initial pose fields.
            m_latestPose.qWorldFromDriverRotation.w = m_latestPose.qDriverFromHeadRotation.w =
                m_latestPose.qRotation.w = 1.f;
            m_latestPose.deviceIsConnected = true;
            m_latestPose.result = vr::TrackingResult_Running_OutOfRange;

            TraceLoggingWriteStop(local, "ControllerDriver_Ctor");
        }

        ~ControllerDriver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "ControllerDriver_Dtor", TLArg(m_serialNumber.c_str(), "SerialNumber"));

            TraceLoggingWriteStop(local, "ControllerDriver_Dtor");
        }

        vr::EVRInitError Activate(uint32_t unObjectId) override {
            TraceLocalActivity(local);
            const bool isLeft = m_role == vr::TrackedControllerRole_LeftHand;
            TraceLoggingWriteStart(local,
                                   "ControllerDriver_Activate",
                                   TLArg(unObjectId, "ObjectId"),
                                   TLArg(isLeft ? "Left" : "Right", "Role"));

            m_deviceIndex = unObjectId;

            const vr::PropertyContainerHandle_t container =
                vr::VRProperties()->TrackedDeviceToPropertyContainer(m_deviceIndex);

            vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, m_role);

            // Purely emulate an Oculus Touch controller for compatibility.
            vr::VRProperties()->SetStringProperty(container, vr::Prop_TrackingSystemName_String, "oculus");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_ManufacturerName_String, "Oculus");
            vr::VRProperties()->SetStringProperty(container,
                                                  vr::Prop_ModelNumber_String,
                                                  isLeft ? "Oculus Quest2 (Left Controller)"
                                                         : "Oculus Quest2 (Right Controller)");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_SerialNumber_String, GetSerialNumber());
            vr::VRProperties()->SetStringProperty(
                container, vr::Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_ControllerType_String, "oculus_touch");

            vr::VRProperties()->SetUint64Property(container, vr::Prop_CurrentUniverseId_Uint64, 1);

            {
                using namespace DirectX;

                std::filesystem::path renderModelPath = "{pimax_native}/rendermodels/";
                std::string renderModel = isLeft ? "crystal_controller_left" : "crystal_controller_right";
                renderModelPath /= renderModel;

                vr::VRProperties()->SetStringProperty(
                    container, vr::Prop_RenderModelName_String, renderModelPath.string().c_str());

                XMMATRIX originToGrip = XMMatrixIdentity();

                // SteamVR expects the pose at the origin of the render model.
                // Retrieve the grip pose transform from the rendermodel.
                const uint32_t length = vr::VRResources()->LoadSharedResource(
                    (renderModelPath / (renderModel + ".json")).string().c_str(), nullptr, 0);
                std::string content;
                content.resize(length);
                vr::VRResources()->LoadSharedResource(
                    (renderModelPath / (renderModel + ".json")).string().c_str(), content.data(), length);

                cJSON* json = nullptr;
                try {
                    json = cJSON_ParseWithLength(content.c_str(), length);
                    if (!json) {
                        throw std::runtime_error("Failed to parse JSON");
                    }

                    const cJSON* components = cJSON_GetObjectItemCaseSensitive(json, "components");
                    if (!components) {
                        throw std::runtime_error("Failed to get components");
                    }

                    const auto getTransform = [&](const char* component) {
                        const cJSON* entry = cJSON_GetObjectItemCaseSensitive(components, component);
                        if (!entry) {
                            throw std::runtime_error("Failed to get component");
                        }

                        const cJSON* component_local = cJSON_GetObjectItemCaseSensitive(entry, "component_local");
                        if (!component_local) {
                            throw std::runtime_error("Failed to get component_local");
                        }
                        const cJSON* origin = cJSON_GetObjectItemCaseSensitive(component_local, "origin");
                        const cJSON* rotation = cJSON_GetObjectItemCaseSensitive(component_local, "rotate_xyz");

                        const auto getVector = [](const cJSON* values) {
                            XMVECTOR vec = {};
                            for (int i = 0; i < 3; i++) {
                                const cJSON* value = nullptr;
                                value = cJSON_GetArrayItem(values, i);
                                if (value) {
                                    vec = XMVectorSetByIndex(vec, (float)value->valuedouble, i);
                                }
                            }
                            return vec;
                        };

                        XMMATRIX transform = XMMatrixIdentity();

                        if (rotation) {
                            const XMVECTOR v = getVector(rotation);
                            transform = XMMatrixRotationRollPitchYaw((float)(XMVectorGetX(v) * M_PI / 180),
                                                                     (float)(XMVectorGetY(v) * M_PI / 180),
                                                                     (float)(XMVectorGetZ(v) * M_PI / 180));
                        }
                        if (origin) {
                            XMVECTOR v = getVector(origin);
                            v = XMVectorSetW(v, 1.f);
                            transform.r[3] = v;
                        }

                        return transform;
                    };

                    m_poseOffset = XMMatrixInverse(nullptr, getTransform("tip"));

                } catch (std::runtime_error& exc) {
                    DriverLog("Error parsing render model %s: %s", renderModel.c_str(), exc.what());
                }
                cJSON_Delete(json);
            }

            // clang-format off
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceOff_String, "{pimax_native}/icons/crystal_controller_status_off.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceSearching_String, "{pimax_native}/icons/crystal_controller_status_searching.gif");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{pimax_native}/icons/crystal_controller_status_searching_alert.gif");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceReady_String, "{pimax_native}/icons/crystal_controller_status_ready.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{pimax_native}/icons/crystal_controller_status_ready_alert.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceNotReady_String, "{pimax_native}/icons/crystal_controller_status_error.png");
            vr::VRProperties()->SetStringProperty(container, vr::Prop_NamedIconPathDeviceAlertLow_String, "{pimax_native}/icons/crystal_controller_status_ready_low.png");
            // clang-format on

            vr::VRDriverInput()->CreateScalarComponent(container,
                                                       "/input/trigger/value",
                                                       &m_components[ComponentTrigger],
                                                       vr::VRScalarType_Absolute,
                                                       vr::VRScalarUnits_NormalizedOneSided);
            vr::VRProperties()->SetInt32Property(
                container, vr::Prop_Axis1Type_Int32, vr::EVRControllerAxisType::k_eControllerAxis_Trigger);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, "/input/trigger/click", &m_components[ComponentTriggerClick]);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, "/input/trigger/touch", &m_components[ComponentTriggerTouch]);
            vr::VRDriverInput()->CreateScalarComponent(container,
                                                       "/input/grip/value",
                                                       &m_components[ComponentGrip],
                                                       vr::VRScalarType_Absolute,
                                                       vr::VRScalarUnits_NormalizedOneSided);
            vr::VRProperties()->SetInt32Property(
                container, vr::Prop_Axis3Type_Int32, vr::EVRControllerAxisType::k_eControllerAxis_Trigger);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, "/input/grip/click", &m_components[ComponentGripClick]);

            vr::VRDriverInput()->CreateScalarComponent(container,
                                                       "/input/joystick/x",
                                                       &m_components[ComponentThumbstickX],
                                                       vr::VRScalarType_Absolute,
                                                       vr::VRScalarUnits_NormalizedTwoSided);
            vr::VRDriverInput()->CreateScalarComponent(container,
                                                       "/input/joystick/y",
                                                       &m_components[ComponentThumbstickY],
                                                       vr::VRScalarType_Absolute,
                                                       vr::VRScalarUnits_NormalizedTwoSided);
            vr::VRProperties()->SetInt32Property(
                container, vr::Prop_Axis2Type_Int32, vr::EVRControllerAxisType::k_eControllerAxis_Joystick);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, "/input/joystick/click", &m_components[ComponentThumbstickClick]);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, "/input/joystick/touch", &m_components[ComponentThumbstickTouch]);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, "/input/thumbrest/touch", &m_components[ComponentThumbrestTouch]);

            if (isLeft) {
                vr::VRDriverInput()->CreateBooleanComponent(
                    container, "/input/system/click", &m_components[ComponentMenu]);
            }
            vr::VRDriverInput()->CreateBooleanComponent(
                container, isLeft ? "/input/y/click" : "/input/b/click", &m_components[ComponentButton1]);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, isLeft ? "/input/y/touch" : "/input/b/touch", &m_components[ComponentButton1Touch]);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, isLeft ? "/input/x/click" : "/input/a/click", &m_components[ComponentButton2]);
            vr::VRDriverInput()->CreateBooleanComponent(
                container, isLeft ? "/input/x/touch" : "/input/a/touch", &m_components[ComponentButton2Touch]);

            vr::VRInputComponentHandle_t componentHaptics;
            vr::VRDriverInput()->CreateHapticComponent(container, "/output/haptic", &componentHaptics);

            ApplySettingsChanges();

            TraceLoggingWriteStop(local, "ControllerDriver_Activate");

            return vr::VRInitError_None;
        }

        void Deactivate() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "ControllerDriver_Deactivate", TLArg(m_deviceIndex, "ObjectId"));

            m_deviceIndex = vr::k_unTrackedDeviceIndexInvalid;

            TraceLoggingWriteStop(local, "ControllerDriver_Deactivate");
        }

        void EnterStandby() override {
        }

        void* GetComponent(const char* pchComponentNameAndVersion) override {
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

        void SendHapticEvent(const vr::VREvent_HapticVibration_t& data) {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local,
                                   "ControllerDriver_SendHapticEvent",
                                   TLArg(m_deviceIndex, "ObjectId"),
                                   TLArg(data.fAmplitude, "Amplitude"),
                                   TLArg(data.fFrequency, "Frequency"),
                                   TLArg(data.fDurationSeconds, "Duration"));

            if (m_deviceIndex != vr::k_unTrackedDeviceIndexInvalid) {
                const float multiplier = vr::VRSettings()->GetFloat("driver_pimax_native", "haptics_strength");

                CHECK_PVRCMD(pvr_triggerHapticPulse(m_pvrSession,
                                                    m_role == vr::TrackedControllerRole_LeftHand
                                                        ? pvrTrackedDevice_LeftController
                                                        : pvrTrackedDevice_RightController,
                                                    data.fAmplitude * multiplier,
                                                    std::max(data.fDurationSeconds, 0.02f),
                                                    data.fFrequency));
            }

            TraceLoggingWriteStop(local, "ControllerDriver_SendHapticEvent");
        }

        void ApplySettingsChanges() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "ControllerDriver_ApplySettingsChanges", TLArg(m_deviceIndex, "ObjectId"));

            const auto usePimaxButton = vr::VRSettings()->GetInt32("driver_pimax_native", "use_windows_key");
            m_usePimaxButton = usePimaxButton == 1 || usePimaxButton == 3;

            TraceLoggingWriteStop(local, "ControllerDriver_ApplySettingsChanges");
        }

        void RunFrame() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "ControllerDriver_RunFrame", TLArg(m_deviceIndex, "ObjectId"));

            const auto side = m_role == vr::TrackedControllerRole_LeftHand ? 0 : 1;

            if (m_deviceIndex != vr::k_unTrackedDeviceIndexInvalid) {
                const vr::PropertyContainerHandle_t container =
                    vr::VRProperties()->TrackedDeviceToPropertyContainer(m_deviceIndex);

                // Update the state of the buttons, triggers and joystick.
                pvrInputState state = {};
                CHECK_PVRCMD(pvr_getInputState(m_pvrSession, &state));
                TraceLoggingWriteTagged(local,
                                        "ControllerDriver_RunFrame_InputState",
                                        TLArg(side == 0 ? "Left" : "Right", "Side"),
                                        TLArg(state.HandButtons[side], "ButtonPress"),
                                        TLArg(state.HandTouches[side], "ButtonTouches"),
                                        TLArg(state.Trigger[side], "Trigger"),
                                        TLArg(state.Grip[side], "Grip"),
                                        TLArg(state.GripForce[side], "GripForce"),
                                        TLArg(ToString(state.JoyStick[side]).c_str(), "Joystick"),
                                        TLArg(ToString(state.TouchPad[side]).c_str(), "Touchpad"),
                                        TLArg(state.TouchPadForce[side], "TouchpadForce"),
                                        TLArg(state.fingerIndex[side], "IndexFinger"),
                                        TLArg(state.fingerMiddle[side], "MiddleFinger"),
                                        TLArg(state.fingerRing[side], "RingFinger"),
                                        TLArg(state.fingerPinky[side], "PinkyFinger"),
                                        TLArg(state.TimeInSeconds, "TimeInSeconds"));

                const auto pvrNow = pvr_getTimeSeconds(m_pvr);
                const auto timeOffset = state.TimeInSeconds - pvrNow;

                vr::VRDriverInput()->UpdateScalarComponent(
                    m_components[ComponentTrigger], state.Trigger[side], timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentTriggerClick], state.HandButtons[side] & pvrButton_Trigger, timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentTriggerTouch], state.HandTouches[side] & pvrButton_Trigger, timeOffset);
                vr::VRDriverInput()->UpdateScalarComponent(m_components[ComponentGrip], state.Grip[side], timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentGripClick], state.HandButtons[side] & pvrButton_Grip, timeOffset);

                vr::VRDriverInput()->UpdateScalarComponent(
                    m_components[ComponentThumbstickX], state.JoyStick[side].x, timeOffset);
                vr::VRDriverInput()->UpdateScalarComponent(
                    m_components[ComponentThumbstickY], state.JoyStick[side].y, timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentThumbstickClick], state.HandButtons[side] & pvrButton_JoyStick, timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentThumbstickTouch], state.HandTouches[side] & pvrButton_JoyStick, timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentThumbrestTouch], state.HandTouches[side] & pvrButton_TouchPad, timeOffset);

                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentButton1], state.HandButtons[side] & pvrButton_B, timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentButton2], state.HandButtons[side] & pvrButton_A, timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentButton1Touch], state.HandTouches[side] & pvrButton_B, timeOffset);
                vr::VRDriverInput()->UpdateBooleanComponent(
                    m_components[ComponentButton2Touch], state.HandTouches[side] & pvrButton_A, timeOffset);
                if (side == 0 && m_usePimaxButton) {
                    vr::VRDriverInput()->UpdateBooleanComponent(
                        m_components[ComponentMenu], state.HandButtons[side] & pvrButton_ApplicationMenu, timeOffset);
                }

                // Update the battery level.
                const int batteryPercentage = pvr_getTrackedDeviceIntProperty(
                    m_pvrSession,
                    m_role == vr::TrackedControllerRole_LeftHand ? pvrTrackedDevice_LeftController
                                                                 : pvrTrackedDevice_RightController,
                    pvrTrackedDeviceProp_BatteryPercent_int,
                    -1);
                if (batteryPercentage > 0) {
                    vr::VRProperties()->SetFloatProperty(
                        container, vr::Prop_DeviceBatteryPercentage_Float, batteryPercentage / 100.f);
                    vr::VRProperties()->SetBoolProperty(container, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
                }

                TraceLoggingWriteTagged(
                    local, "ControllerDriver_RunFrame_ControllerProps", TLArg(batteryPercentage, "BatteryPercentage"));
            }

            TraceLoggingWriteStop(local, "ControllerDriver_RunFrame");
        }

        void UpdateTrackingState(const pvrPoseStatef& state) override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "ControllerDriver_UpdateTrackingState", TLArg(m_deviceIndex, "ObjectId"));

            vr::DriverPose_t pose = {};
            pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = pose.qRotation.w = 1.0;

            if (m_deviceIndex != vr::k_unTrackedDeviceIndexInvalid) {
                pose.deviceIsConnected = true;
                pose.result = vr::TrackingResult_Running_OutOfRange;

                const bool force3Dof = vr::VRSettings()->GetBool("driver_pimax_native", "force_3dof");
                if ((state.StatusFlags & pvrStatus_OrientationTracked) && !force3Dof) {
                    pose.vecPosition[0] = state.ThePose.Position.x;
                    pose.vecPosition[1] = state.ThePose.Position.y;
                    pose.vecPosition[2] = state.ThePose.Position.z;
                    pose.qRotation.x = state.ThePose.Orientation.x;
                    pose.qRotation.y = state.ThePose.Orientation.y;
                    pose.qRotation.z = state.ThePose.Orientation.z;
                    pose.qRotation.w = state.ThePose.Orientation.w;

                    pose.vecVelocity[0] = state.LinearVelocity.x;
                    pose.vecVelocity[1] = state.LinearVelocity.y;
                    pose.vecVelocity[2] = state.LinearVelocity.z;
                    pose.vecAngularVelocity[0] = state.AngularVelocity.x;
                    pose.vecAngularVelocity[1] = state.AngularVelocity.y;
                    pose.vecAngularVelocity[2] = state.AngularVelocity.z;

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

                {
                    std::unique_lock lock(m_poseMutex);
                    m_latestPose = pose;
                }
                vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));
            }

            TraceLoggingWriteStop(
                local, "ControllerDriver_UpdateTrackingState", TLArg(pose.poseTimeOffset, "PoseTimeOffset"));
        }

        void Disconnect() override {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "ControllerDriver_Disconnect", TLArg(m_deviceIndex, "ObjectId"));

            vr::DriverPose_t pose = {};
            pose.qWorldFromDriverRotation.w = pose.qDriverFromHeadRotation.w = pose.qRotation.w = 1.0;
            pose.result = vr::TrackingResult_Running_OutOfRange;

            {
                std::unique_lock lock(m_poseMutex);
                m_latestPose = pose;
            }
            vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_deviceIndex, pose, sizeof(pose));

            TraceLoggingWriteStop(local, "ControllerDriver_Disconnect");
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

        DirectX::XMMATRIX m_poseOffset = DirectX::XMMatrixIdentity();

        std::shared_mutex m_poseMutex;
        vr::DriverPose_t m_latestPose = {};

        bool m_usePimaxButton = true;
    };
} // namespace

namespace driver {
    std::unique_ptr<IControllerDriver> CreateControllerDriver(pvrEnvHandle pvr,
                                                              pvrSessionHandle pvrSession,
                                                              vr::ETrackedControllerRole role) {
        return std::make_unique<ControllerDriver>(pvr, pvrSession, role);
    }
} // namespace driver
