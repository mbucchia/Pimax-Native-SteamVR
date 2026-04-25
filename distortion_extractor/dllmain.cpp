#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winrt/base.h>

#include <future>
#include <iostream>
#include <string>

#include <d3d11_1.h>
#include <D3DX11tex.h>

#include <detours.h>

#include "VertexShader.h"
#include "PixelShader.h"

#pragma comment(lib, "d3d11.lib")

#pragma region Detours helpers

#define DECLARE_DETOUR_FUNCTION(ReturnType, FunctionName, ...)                                                         \
    ReturnType (*original_##FunctionName)(##__VA_ARGS__) = nullptr;                                                    \
    ReturnType hooked_##FunctionName(##__VA_ARGS__)

#define DEFINE_DETOUR_FUNCTION(ReturnType, FunctionName, ...) ReturnType hooked_##FunctionName(##__VA_ARGS__)

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

#pragma endregion

int GetEnvironmentInt(const LPCSTR name, const int defaultValue) {
    char buffer[32];
    DWORD result = GetEnvironmentVariableA(name, buffer, sizeof(buffer));

    if (result == 0 || result >= sizeof(buffer)) {
        return defaultValue;
    }

    try {
        return std::stoi(buffer);
    } catch (...) {
        return defaultValue;
    }
}

winrt::handle s_shm[3];
WCHAR s_mapPath[MAX_PATH] = {};
UINT s_mapSize = 0;

DECLARE_DETOUR_FUNCTION(void,
                        ID3D11DeviceContext_DrawIndexed,
                        ID3D11DeviceContext* pContext,
                        UINT IndexCount,
                        UINT StartIndexLocation,
                        INT BaseVertexLocation) {
    static bool captured = false;

    // PCS mesh is 24576. Pimax Play also draws a few more things, but with a lot less triangles.
    if (!captured && s_mapSize && IndexCount > 10000) {
        winrt::com_ptr<ID3D11Device> device;
        pContext->GetDevice(device.put());
        winrt::com_ptr<ID3D11Device1> device1 = device.as<ID3D11Device1>();
        winrt::com_ptr<ID3D11DeviceContext1> context1;
        winrt::check_hresult(pContext->QueryInterface(IID_PPV_ARGS(context1.put())));

        // Retrieve the EyeToSourceUVParams, for some reason they are invalid for several frames...
        winrt::com_ptr<ID3D11Buffer> cb;
        context1->VSGetConstantBuffers(0, 1, cb.put());
        winrt::check_pointer(cb.get());
        float EyeToSourceUVParams[4] = {};
        {
            winrt::com_ptr<ID3D11Buffer> staging;
            {
                D3D11_BUFFER_DESC desc = {};
                cb->GetDesc(&desc);
                desc.BindFlags = desc.MiscFlags = 0;
                desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
                desc.Usage = D3D11_USAGE_STAGING;
                winrt::check_hresult(device1->CreateBuffer(&desc, nullptr, staging.put()));
            }
            context1->CopyResource(staging.get(), cb.get());

            D3D11_MAPPED_SUBRESOURCE mapping = {};
            winrt::check_hresult(context1->Map(staging.get(), 0, D3D11_MAP_READ, 0, &mapping));
            memcpy(EyeToSourceUVParams, mapping.pData, sizeof(EyeToSourceUVParams));
            context1->Unmap(staging.get(), 0);
        }

        // clang-format off
        const bool isSnapshotValid = -2.f <= EyeToSourceUVParams[0] && EyeToSourceUVParams[0] <= 2.f &&
                                     -2.f <= EyeToSourceUVParams[1] && EyeToSourceUVParams[1] <= 2.f &&
                                     -2.f <= EyeToSourceUVParams[2] && EyeToSourceUVParams[2] <= 2.f &&
                                     -2.f <= EyeToSourceUVParams[3] && EyeToSourceUVParams[3] <= 2.f &&
                                     std::abs(EyeToSourceUVParams[0]) > 0.00001f &&
                                     std::abs(EyeToSourceUVParams[1]) > 0.00001f;
        // clang-format on
        if (isSnapshotValid) {
            printf("EyeToSourceUVParams = %.3f, %.3f, %.3f, %.3f\n",
                   EyeToSourceUVParams[0],
                   EyeToSourceUVParams[1],
                   EyeToSourceUVParams[2],
                   EyeToSourceUVParams[3]);

            // Latch the state of the vertex stage. We will forward it to our draw call.
            winrt::com_ptr<ID3D11Buffer> vb, ib;
            DXGI_FORMAT ibFormat = DXGI_FORMAT_UNKNOWN;
            UINT vbStride = 0, vbOffset = 0, ibOffset = 0;
            context1->IAGetVertexBuffers(0, 1, vb.put(), &vbStride, &vbOffset);
            context1->IAGetIndexBuffer(ib.put(), &ibFormat, &ibOffset);
            D3D11_PRIMITIVE_TOPOLOGY topology = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
            context1->IAGetPrimitiveTopology(&topology);
            winrt::com_ptr<ID3D11InputLayout> layout;
            context1->IAGetInputLayout(layout.put());
            winrt::com_ptr<ID3D11VertexShader> vs;
            context1->VSGetShader(vs.put(), nullptr, nullptr);
            winrt::check_pointer(ib.get());
            winrt::check_pointer(layout.get());
            winrt::check_pointer(vs.get());

            // Set-up context switching.
            const auto featureLevel = D3D_FEATURE_LEVEL_11_0;
            winrt::com_ptr<ID3DDeviceContextState> cleanContext;
            winrt::check_hresult(device1->CreateDeviceContextState(
                0, &featureLevel, 1, D3D11_SDK_VERSION, __uuidof(ID3D11Device), nullptr, cleanContext.put()));
            winrt::com_ptr<ID3DDeviceContextState> savedContext;
            context1->SwapDeviceContextState(cleanContext.get(), savedContext.put());

            // Create render targets.
            winrt::com_ptr<ID3D11Texture2D> texture;
            {
                D3D11_TEXTURE2D_DESC desc = {};
                desc.Width = desc.Height = s_mapSize;
                desc.ArraySize = 3;
                desc.MipLevels = desc.SampleDesc.Count = 1;
                desc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT; // UV + unused + alpha (validity)
                desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
                winrt::check_hresult(device1->CreateTexture2D(&desc, nullptr, texture.put()));
            }
            winrt::com_ptr<ID3D11RenderTargetView> rtv[3];
            for (uint32_t channel = 0; channel < 3; channel++) {
                {
                    D3D11_RENDER_TARGET_VIEW_DESC desc = {};
                    desc.ViewDimension = D3D11_RTV_DIMENSION_TEXTURE2DARRAY;
                    desc.Texture2DArray.FirstArraySlice = channel;
                    desc.Texture2DArray.ArraySize = 1;
                    desc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT;
                    winrt::check_hresult(device1->CreateRenderTargetView(texture.get(), &desc, rtv[channel].put()));
                    const float null[] = {0, 0, 0, 0};
                    context1->ClearRenderTargetView(rtv[channel].get(), null);
                }
            }

            // Forward vertex stage from the original call.
            {
                ID3D11Buffer* vbs[] = {vb.get()};
                context1->IASetVertexBuffers(0, 1, vbs, &vbStride, &vbOffset);
            }
            context1->IASetIndexBuffer(ib.get(), ibFormat, ibOffset);
            context1->IASetPrimitiveTopology(topology);
            context1->IASetInputLayout(layout.get());
            {
                ID3D11Buffer* cbs[] = {cb.get()};
                context1->VSSetConstantBuffers(0, 1, cbs);
                context1->PSSetConstantBuffers(0, 1, cbs);
            }

#if 1
            // Setup our shaders.
            winrt::check_hresult(
                device1->CreateVertexShader(k_VertexShader, sizeof(k_VertexShader), nullptr, vs.put()));
#endif
            context1->VSSetShader(vs.get(), nullptr, 0);
            winrt::com_ptr<ID3D11PixelShader> ps;
            winrt::check_hresult(device1->CreatePixelShader(k_PixelShader, sizeof(k_PixelShader), nullptr, ps.put()));
            context1->PSSetShader(ps.get(), nullptr, 0);

            // Setup our draw.
            D3D11_VIEWPORT viewport = {};
            viewport.Width = viewport.Height = (float)s_mapSize;
            viewport.MaxDepth = 1.f;
            context1->RSSetViewports(1, &viewport);
            winrt::com_ptr<ID3D11DepthStencilState> dss;
            {
                // Disable depth testing.
                D3D11_DEPTH_STENCIL_DESC desc = {};
                desc.DepthEnable = FALSE;
                desc.DepthFunc = D3D11_COMPARISON_ALWAYS;
                desc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ZERO;
                winrt::check_hresult(device1->CreateDepthStencilState(&desc, dss.put()));
            }
            context1->OMSetDepthStencilState(dss.get(), 0xff);
            winrt::com_ptr<ID3D11RasterizerState> rs;
            {
                // Disable backface culling.
                D3D11_RASTERIZER_DESC desc = {};
                desc.CullMode = D3D11_CULL_NONE;
                desc.FillMode = D3D11_FILL_SOLID;
                winrt::check_hresult(device1->CreateRasterizerState(&desc, rs.put()));
            }
            context1->RSSetState(rs.get());
            {
                ID3D11RenderTargetView* rtvs[] = {rtv[0].get(), rtv[1].get(), rtv[2].get()};
                context1->OMSetRenderTargets(3, rtvs, nullptr);
            }

            // Go!
            original_ID3D11DeviceContext_DrawIndexed(pContext, IndexCount, StartIndexLocation, BaseVertexLocation);

            // Unbind.
            {
                ID3D11RenderTargetView* rtvs[] = {nullptr, nullptr, nullptr};
                context1->OMSetRenderTargets(3, rtvs, nullptr);
            }

            if (s_mapPath[0]) {
                // Save the output to DDS.
                D3DX11SaveTextureToFileW(context1.get(), texture.get(), D3DX11_IFF_DDS, s_mapPath);
            } else {
                // Shove into a SHM instead.
                winrt::com_ptr<ID3D11Texture2D> staging;
                {
                    D3D11_TEXTURE2D_DESC desc = {};
                    desc.Width = desc.Height = s_mapSize;
                    desc.ArraySize = 3;
                    desc.MipLevels = desc.SampleDesc.Count = 1;
                    desc.Format = DXGI_FORMAT_R32G32B32A32_FLOAT; // UV + unused + alpha (validity)
                    desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
                    desc.Usage = D3D11_USAGE_STAGING;
                    winrt::check_hresult(device1->CreateTexture2D(&desc, nullptr, staging.put()));
                }
                context1->CopyResource(staging.get(), texture.get());

                const UINT size = s_mapSize * s_mapSize * sizeof(float) * 4;
                const auto createShm = [&](const WCHAR* name, UINT subresourceIndex) {
                    // Create the SHM.
                    *s_shm[subresourceIndex].put() =
                        CreateFileMapping(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, size, name);
                    winrt::check_pointer(s_shm[subresourceIndex].get());
                    void* memory =
                        MapViewOfFile(s_shm[subresourceIndex].get(), FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, size);
                    winrt::check_pointer(memory);

                    // Copy the map.
                    D3D11_MAPPED_SUBRESOURCE mapping = {};
                    winrt::check_hresult(context1->Map(staging.get(), subresourceIndex, D3D11_MAP_READ, 0, &mapping));
                    memcpy(memory, mapping.pData, size);
                    context1->Unmap(staging.get(), subresourceIndex);
                };
                createShm(L"DistortionExtractor.Red", 0);
                createShm(L"DistortionExtractor.Green", 1);
                createShm(L"DistortionExtractor.Blue", 2);
            }

            printf("\n=======================================================================\n"
                   "Capture completed!"
                   "\n=======================================================================\n");

            // Restore context.
            context1->SwapDeviceContextState(savedContext.get(), nullptr);

            captured = true;
        }
    }

    original_ID3D11DeviceContext_DrawIndexed(pContext, IndexCount, StartIndexLocation, BaseVertexLocation);
}

// Detours need at least one export.
extern "C" void dummy() {
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    static std::future<void> deferredHook;

    switch (ul_reason_for_call) {
    case DLL_PROCESS_ATTACH:
        s_mapSize = GetEnvironmentInt("DISTORTION_EXTRACTOR_SIZE", 256);
        GetEnvironmentVariableW(L"DISTORTION_EXTRACTOR_PATH", s_mapPath, MAX_PATH);

        printf("\n=======================================================================\n"
               "Enabling distortion extractor to '%ws' with resolution %u"
               "\n=======================================================================\n",
               s_mapPath,
               s_mapSize);

        // We can't create COM objects in DllMain(). Defer to an async worker.
        deferredHook = std::async(std::launch::async, []() {
            winrt::com_ptr<ID3D11Device> device;
            winrt::com_ptr<ID3D11DeviceContext> context;
            HRESULT hr = D3D11CreateDevice(nullptr,
                                           D3D_DRIVER_TYPE_HARDWARE,
                                           nullptr,
                                           D3D11_CREATE_DEVICE_BGRA_SUPPORT,
                                           nullptr,
                                           0,
                                           D3D11_SDK_VERSION,
                                           device.put(),
                                           nullptr,
                                           context.put());
            DetourMethodAttach(
                context.get(), 12, hooked_ID3D11DeviceContext_DrawIndexed, original_ID3D11DeviceContext_DrawIndexed);
        });
        break;
    case DLL_PROCESS_DETACH:
        deferredHook.wait();
        break;
    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
        break;
    }
    return TRUE;
}
