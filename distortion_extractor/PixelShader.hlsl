cbuffer CB0 : register(b0)
{
    float2 EyeToSourceUVScale;
    float2 EyeToSourceUVOffset;
    float4x4 EyeRotationStart;
    float4x4 EyeRotationEnd;
    float2 LensCenter;
};

struct PS_INPUT
{
    float4 pos : SV_Position;
    float4 color : COLOR0;
    float3 red : TEXCOORD0;
    float3 green : TEXCOORD1;
    float3 blue : TEXCOORD2;
};

struct PS_OUTPUT
{
    float4 red : SV_Target0;
    float4 green : SV_Target1;
    float4 blue : SV_Target2;
};

PS_OUTPUT main(PS_INPUT input)
{
    PS_OUTPUT output;
    output.red = float4(input.red.xy, input.color.x * 0.000001, 1);
    output.green = float4(input.green.xy, input.color.x * 0.000001, 1);
    output.blue = float4(input.blue.xy, input.color.x * 0.000001, 1);
    return output;
}
