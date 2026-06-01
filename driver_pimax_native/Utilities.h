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

#pragma once

namespace util {

    static std::optional<std::wstring> RegGetString(HKEY hKey, const std::wstring& subKey, const std::wstring& value) {
        DWORD dataSize = 0;
        LONG retCode = ::RegGetValue(
            hKey, subKey.c_str(), value.c_str(), RRF_SUBKEY_WOW6464KEY | RRF_RT_REG_SZ, nullptr, nullptr, &dataSize);
        if (retCode != ERROR_SUCCESS || !dataSize) {
            return {};
        }

        std::wstring data(dataSize / sizeof(wchar_t), 0);
        retCode = ::RegGetValue(hKey,
                                subKey.c_str(),
                                value.c_str(),
                                RRF_SUBKEY_WOW6464KEY | RRF_RT_REG_SZ,
                                nullptr,
                                data.data(),
                                &dataSize);
        if (retCode != ERROR_SUCCESS) {
            return {};
        }

        return data.substr(0, dataSize / sizeof(wchar_t) - 1);
    }

    inline std::string pvr_getTrackedDeviceStringPropertyHelper(pvrSessionHandle sessionHandle,
                                                                pvrTrackedDeviceType device,
                                                                pvrTrackedDeviceProp prop) {
        const int size = pvr_getTrackedDeviceStringProperty(sessionHandle, device, prop, nullptr, 0);
        if (size > 0) {
            std::string value(size, 0);
            pvr_getTrackedDeviceStringProperty(sessionHandle, device, prop, value.data(), (int)value.size() + 1);
            // Remove trailing 0.
            value.resize(size - 1, 0);
            return value;
        }
        return {};
    }

    inline vr::HmdMatrix34_t StoreHmdMatrix34(const PVR::Matrix4f& matrix) {
        return {matrix.M[0][0],
                matrix.M[0][1],
                matrix.M[0][2],
                matrix.M[0][3],
                matrix.M[1][0],
                matrix.M[1][1],
                matrix.M[1][2],
                matrix.M[1][3],
                matrix.M[2][0],
                matrix.M[2][1],
                matrix.M[2][2],
                matrix.M[2][3]};
    }

    static inline vr::HmdMatrix34_t StoreHmdMatrix34(const DirectX::XMMATRIX& matrix) {
        DirectX::XMFLOAT4X3 temp;
        DirectX::XMStoreFloat4x3(&temp, matrix);
        return {temp.m[0][0],
                temp.m[1][0],
                temp.m[2][0],
                temp.m[3][0],
                temp.m[0][1],
                temp.m[1][1],
                temp.m[2][1],
                temp.m[3][1],
                temp.m[0][2],
                temp.m[1][2],
                temp.m[2][2],
                temp.m[3][2]};
    }

    static inline std::string ToString(pvrResult result) {
        switch (result) {
        case pvr_success:
            return "Success";
        case pvr_failed:
            return "Failed";
        case pvr_dll_failed:
            return "DLL Failed";
        case pvr_dll_wrong:
            return "DLL Wrong";
        case pvr_interface_not_found:
            return "Interface not found";
        case pvr_invalid_param:
            return "Invalid Parameter";
        case pvr_rpc_failed:
            return "RPC Failed";
        case pvr_share_mem_failed:
            return "Share Memory Failed";
        case pvr_unsupport_render_name:
            return "Unsupported Render Name";
        case pvr_no_display:
            return "No Display";
        case pvr_no_render_device:
            return "No Render Device";
        case pvr_app_not_visible:
            return "App Not Visible";
        case pvr_srv_not_ready:
            return "Service Not Ready";
        case pvr_dll_srv_mismatch:
            return "DLL Mismatch";
        case pvr_app_adapter_mismatch:
            return "App Adapter Mismatch";
        case pvr_not_support:
            return "Not Supported";

        default:
            return error::_Fmt("pvrResult_%d", (int)result);
        }
    }

    static inline std::string ToString(pvrHmdTrackingStyle result) {
        switch (result) {
        case pvrHmdTrackingStyle_Imu:
            return "Imu";
        case pvrHmdTrackingStyle_Lighthouse:
            return "Lighthouse";
        case pvrHmdTrackingStyle_InsideOutCameras:
            return "Inside-Out";

        default:
            return error::_Fmt("pvrResult_%d", (int)result);
        }
    }

    static inline std::string ToString(const pvrPosef& pose) {
        return error::_Fmt("p: (%.3f, %.3f, %.3f), o:(%.3f, %.3f, %.3f, %.3f)",
                           pose.Position.x,
                           pose.Position.y,
                           pose.Position.z,
                           pose.Orientation.x,
                           pose.Orientation.y,
                           pose.Orientation.z,
                           pose.Orientation.w);
    }

    static inline std::string ToString(const pvrVector3f& vec) {
        return error::_Fmt("(%.3f, %.3f, %.3f)", vec.x, vec.y, vec.z);
    }

    static inline std::string ToString(const pvrVector2f& vec) {
        return error::_Fmt("(%.3f, %.3f)", vec.x, vec.y);
    }

} // namespace util
