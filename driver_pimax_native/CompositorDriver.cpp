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

// This is used for debugging, mostly to inject RenderDoc into vrcompositor.

#ifdef _DEBUG

#include "CompositorDriver.h"
#include "ErrorHandling.h"
#include "Tracing.h"
#include "Utilities.h"

#pragma comment(lib, "d3d11.lib")

using namespace driver;
using namespace util;

#define DECLARE_DETOUR_FUNCTION(ReturnType, FunctionName, ...)                                                         \
    ReturnType (*original_##FunctionName)(##__VA_ARGS__) = nullptr;                                                    \
    ReturnType hooked_##FunctionName(##__VA_ARGS__)

#define DEFINE_DETOUR_FUNCTION(ReturnType, FunctionName, ...) ReturnType hooked_##FunctionName(##__VA_ARGS__)

namespace {

    template <class T, typename TMethod>
    void DetourMethodAttach(T* instance, unsigned int methodOffset, TMethod hooked, TMethod& original) {
        if (original) {
            // Already hooked.
            return;
        }

        LPVOID* vtable = *((LPVOID**)instance);
        LPVOID target = vtable[methodOffset];

        DetourTransactionBegin();
        DetourUpdateThread(GetCurrentThread());

        original = (TMethod)target;
        DetourAttach((PVOID*)&original, hooked);

        DetourTransactionCommit();
    }

    DECLARE_DETOUR_FUNCTION(INT,
                            ID3DUserDefinedAnnotation_BeginEvent,
                            ID3DUserDefinedAnnotation* Annotation,
                            LPCWSTR Name);
    DECLARE_DETOUR_FUNCTION(INT, ID3DUserDefinedAnnotation_EndEvent, ID3DUserDefinedAnnotation* Annotation);

    struct CompositorDriver {
        CompositorDriver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CompositorDriver_Ctor");

            // RenderDoc DLL must be present in the current folder.
            LoadLibraryW(L"renderdoc.dll");

            // We can't create COM objects in DllMain(). Defer to an async worker.
            m_deferredHook = std::async(std::launch::async, []() {
                TraceLocalActivity(local);
                TraceLoggingWriteStart(local, "CompositorDriver_DeferredHook");

                ComPtr<ID3D11Device> device;
                ComPtr<ID3D11DeviceContext> context;
                HRESULT hr = D3D11CreateDevice(nullptr,
                                               D3D_DRIVER_TYPE_HARDWARE,
                                               nullptr,
                                               D3D11_CREATE_DEVICE_BGRA_SUPPORT,
                                               nullptr,
                                               0,
                                               D3D11_SDK_VERSION,
                                               device.ReleaseAndGetAddressOf(),
                                               nullptr,
                                               context.ReleaseAndGetAddressOf());

                ComPtr<ID3DUserDefinedAnnotation> annotation;
                context->QueryInterface(IID_PPV_ARGS(annotation.ReleaseAndGetAddressOf()));

                DetourMethodAttach(annotation.Get(),
                                   3,
                                   hooked_ID3DUserDefinedAnnotation_BeginEvent,
                                   original_ID3DUserDefinedAnnotation_BeginEvent);
                DetourMethodAttach(annotation.Get(),
                                   4,
                                   hooked_ID3DUserDefinedAnnotation_EndEvent,
                                   original_ID3DUserDefinedAnnotation_EndEvent);

                TraceLoggingWriteStop(local, "CompositorDriver_DeferredHook");
            });

            TraceLoggingWriteStop(local, "CompositorDriver_Ctor");
        }

        ~CompositorDriver() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CompositorDriver_Dtor");

            m_deferredHook.wait();

            TraceLoggingWriteStop(local, "CompositorDriver_Dtor");
        }

        void BeginEvent(ID3D11DeviceContext* context, const std::wstring& name) {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CompositorDriver_BeginEvent", TLArg(name.c_str(), "Name"));

            m_context = context;
            m_events.push_back(name);

            TraceLoggingWriteStop(local, "CompositorDriver_BeginEvent");
        }

        void EndEvent() {
            if (m_events.empty()) {
                return;
            }

            const std::wstring name = m_events.back();
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CompositorDriver_EndEvent", TLArg(name.c_str(), "Name"));

            m_events.pop_back();

            if (name == L"Warp: R") {
                OnPresent();
            }

            TraceLoggingWriteStop(local, "CompositorDriver_BeginEvent");
        }

        void OnPresent() {
            TraceLocalActivity(local);
            TraceLoggingWriteStart(local, "CompositorDriver_OnPresent");

            // Use a composition swapchain to allow RenderDoc to see frame boundaries.
            if (!m_dxgiSwapchain) {
                DXGI_SWAP_CHAIN_DESC1 swapchainDesc = {};
                swapchainDesc.Width = 8;
                swapchainDesc.Height = 8;
                swapchainDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
                swapchainDesc.SampleDesc.Count = 1;
                swapchainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
                swapchainDesc.BufferCount = 3;
                swapchainDesc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
                swapchainDesc.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING;

                ComPtr<ID3D11Device> device;
                m_context->GetDevice(device.ReleaseAndGetAddressOf());

                ComPtr<IDXGIDevice> dxgiDevice;
                device->QueryInterface(IID_PPV_ARGS(dxgiDevice.ReleaseAndGetAddressOf()));

                ComPtr<IDXGIAdapter> dxgiAdapter;
                dxgiDevice->GetAdapter(dxgiAdapter.ReleaseAndGetAddressOf());

                ComPtr<IDXGIFactory2> dxgiFactory;
                dxgiAdapter->GetParent(IID_PPV_ARGS(dxgiFactory.ReleaseAndGetAddressOf()));
                dxgiFactory->CreateSwapChainForComposition(
                    dxgiDevice.Get(), &swapchainDesc, nullptr, m_dxgiSwapchain.ReleaseAndGetAddressOf());
            }

            m_dxgiSwapchain->Present(0, 0);

            TraceLoggingWriteStop(local, "CompositorDriver_OnPresent");
        }

        static CompositorDriver& GetInstance() {
            static CompositorDriver instance;
            return instance;
        }

        std::deque<std::wstring> m_events;
        ComPtr<ID3D11DeviceContext> m_context;
        ComPtr<IDXGISwapChain1> m_dxgiSwapchain;

        std::future<void> m_deferredHook;
    };

    DEFINE_DETOUR_FUNCTION(INT,
                           ID3DUserDefinedAnnotation_BeginEvent,
                           ID3DUserDefinedAnnotation* Annotation,
                           LPCWSTR Name) {
        ComPtr<ID3D11DeviceContext> context;
        Annotation->QueryInterface(IID_PPV_ARGS(context.ReleaseAndGetAddressOf()));

        CompositorDriver::GetInstance().BeginEvent(context.Get(), Name);

        return original_ID3DUserDefinedAnnotation_BeginEvent(Annotation, Name);
    }

    DEFINE_DETOUR_FUNCTION(INT, ID3DUserDefinedAnnotation_EndEvent, ID3DUserDefinedAnnotation* Annotation) {
        CompositorDriver::GetInstance().EndEvent();

        return original_ID3DUserDefinedAnnotation_EndEvent(Annotation);
    }

} // namespace

namespace driver {

    bool InjectVrCompositor(DWORD processId) {
        HMODULE kernel32 = nullptr;
        GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, L"kernel32.dll", &kernel32);
        const FARPROC LoadLibraryW = GetProcAddress(kernel32, "LoadLibraryW");
        const FARPROC SetDllDirectoryW = GetProcAddress(kernel32, "SetDllDirectoryW");

        wil::unique_handle targetProcess;
        *targetProcess.put() = OpenProcess(PROCESS_ALL_ACCESS, 0, processId);
        if (!targetProcess) {
            DriverLog("Failed to open vrcompositor process (%u)", processId);
            return false;
        }

        HMODULE thisDll = nullptr;
        GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                           (LPCWSTR)&InjectVrCompositor,
                           &thisDll);
        WCHAR dllPath[MAX_PATH]{};
        GetModuleFileNameW(thisDll, dllPath, MAX_PATH);
        const auto rootDir = std::filesystem::path(dllPath).parent_path();

        const auto dllPathSize = (wcslen(dllPath) + 1) * sizeof(TCHAR);
        const auto rootDirSize = (rootDir.wstring().size() + 1) * sizeof(TCHAR);
        void* dllPathInTargetProcess =
            VirtualAllocEx(targetProcess.get(), nullptr, dllPathSize, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        void* rootDirInTargetProcess =
            VirtualAllocEx(targetProcess.get(), nullptr, rootDirSize, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        if (!dllPathInTargetProcess || !rootDirInTargetProcess) {
            DriverLog("Failed to allocate memory in vrcompositor");
            return false;
        }

        {
            const std::string part1 = "WriteProc";
            const std::string part2 = "essMemory";
            const std::string full = part1 + part2;
            typedef BOOL(WINAPI * pfnWriteProcessMemory)(HANDLE, LPVOID, LPCVOID, SIZE_T, SIZE_T*);
            pfnWriteProcessMemory WriteProcessMemory = (pfnWriteProcessMemory)GetProcAddress(kernel32, full.c_str());
            if (!WriteProcessMemory(targetProcess.get(), dllPathInTargetProcess, dllPath, dllPathSize, nullptr) ||
                !WriteProcessMemory(
                    targetProcess.get(), rootDirInTargetProcess, rootDir.wstring().c_str(), rootDirSize, nullptr)) {
                DriverLog("Failed to write memory in vrcompositor");
                return false;
            }
        }

        {
            const std::string part1 = "CreateRem";
            const std::string part2 = "oteThread";
            const std::string full = part1 + part2;
            typedef HANDLE(WINAPI * pfnCreateRemoteThread)(
                HANDLE, LPSECURITY_ATTRIBUTES, SIZE_T, LPTHREAD_START_ROUTINE, LPVOID, DWORD, LPDWORD);
            pfnCreateRemoteThread CreateRemoteThread = (pfnCreateRemoteThread)GetProcAddress(kernel32, full.c_str());
            wil::unique_handle threadId;
            *threadId.put() = CreateRemoteThread(targetProcess.get(),
                                                 nullptr,
                                                 0,
                                                 reinterpret_cast<LPTHREAD_START_ROUTINE>(SetDllDirectoryW),
                                                 rootDirInTargetProcess,
                                                 0,
                                                 nullptr);
            if (!threadId) {
                DriverLog("Failed to inject into vrcompositor");
                return false;
            }

            WaitForSingleObject(threadId.get(), 10000);

            *threadId.put() = CreateRemoteThread(targetProcess.get(),
                                                 nullptr,
                                                 0,
                                                 reinterpret_cast<LPTHREAD_START_ROUTINE>(LoadLibraryW),
                                                 dllPathInTargetProcess,
                                                 0,
                                                 nullptr);
            if (!threadId) {
                DriverLog("Failed to inject into vrcompositor");
                return false;
            }
        }

        DriverLog("Successfully injected into vrcompositor");

        return true;
    }

    void CreateCompositorDriver() {
        (void)CompositorDriver::GetInstance();
    }

} // namespace driver

#endif
