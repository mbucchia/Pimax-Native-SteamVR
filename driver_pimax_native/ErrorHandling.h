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

#include "Tracing.h"

#define CHK_STRINGIFY(x) #x
#define TOSTRING(x) CHK_STRINGIFY(x)
#define FILE_AND_LINE __FILE__ ":" TOSTRING(__LINE__)

#define CHECK_HRCMD(cmd) ::error::_CheckHResult(cmd, #cmd, FILE_AND_LINE)
#define CHECK_PVRCMD(cmd) ::error::_CheckPVRResult(cmd, #cmd, FILE_AND_LINE)
#define CHECK_HRESULT(res, cmdStr) ::error::_CheckHResult(res, cmdStr, FILE_AND_LINE)
#define CHECK(exp)                                                                                                     \
    {                                                                                                                  \
        if (!(exp)) {                                                                                                  \
            ::error::_Throw("Check failed ", #exp, FILE_AND_LINE);                                                     \
        }                                                                                                              \
    }
#define CHECK_MSG(exp, msg, ...)                                                                                       \
    {                                                                                                                  \
        if (!(exp)) {                                                                                                  \
            ::error::_Throw(::error::_Fmt(msg " ", ##__VA_ARGS__), #exp, FILE_AND_LINE);                               \
        }                                                                                                              \
    }

namespace error {
    inline std::string _Fmt(const char* fmt, ...) {
        va_list vl;
        va_start(vl, fmt);
        int size = std::vsnprintf(nullptr, 0, fmt, vl);
        va_end(vl);

        if (size != -1) {
            std::unique_ptr<char[]> buffer(new char[size + 1]);

            va_start(vl, fmt);
            size = std::vsnprintf(buffer.get(), size + 1, fmt, vl);
            va_end(vl);
            if (size != -1) {
                return std::string(buffer.get(), size);
            }
        }

        throw std::runtime_error("Unexpected vsnprintf failure");
    }

    [[noreturn]] inline void _Throw(std::string failureMessage,
                                    const char* originator = nullptr,
                                    const char* sourceLocation = nullptr) {
        std::string failureMessageNoNewline = failureMessage;
        if (originator != nullptr) {
            failureMessage += _Fmt("\n    Origin: %s", originator);
            failureMessageNoNewline += _Fmt(" Origin: %s ", originator);
        }
        if (sourceLocation != nullptr) {
            failureMessage += _Fmt("\n    Source: %s", sourceLocation);
            failureMessageNoNewline += _Fmt(" Source: %s ", sourceLocation);
        }

        TraceLoggingWrite(TraceProvider, "Error", TLArg(failureMessageNoNewline.c_str(), "Message"));
        DriverLog("Fatal error: %s", failureMessageNoNewline.c_str());
        throw std::logic_error(failureMessage);
    }

    [[noreturn]] inline void _ThrowHResult(HRESULT hr,
                                           const char* originator = nullptr,
                                           const char* sourceLocation = nullptr) {
        _Throw(_Fmt("HRESULT failure [%x]", hr), originator, sourceLocation);
    }

    inline HRESULT _CheckHResult(HRESULT hr, const char* originator = nullptr, const char* sourceLocation = nullptr) {
        if (FAILED(hr)) {
            _ThrowHResult(hr, originator, sourceLocation);
        }

        return hr;
    }

    [[noreturn]] static inline void _ThrowPVRResult(pvrResult pvr,
                                                    const char* originator = nullptr,
                                                    const char* sourceLocation = nullptr) {
        _Throw(_Fmt("pvrResult failure [%d]", pvr), originator, sourceLocation);
    }

    static inline pvrResult _CheckPVRResult(pvrResult pvr,
                                            const char* originator = nullptr,
                                            const char* sourceLocation = nullptr) {
        if (pvr != pvr_success) {
            _ThrowPVRResult(pvr, originator, sourceLocation);
        }

        return pvr;
    }
} // namespace error
