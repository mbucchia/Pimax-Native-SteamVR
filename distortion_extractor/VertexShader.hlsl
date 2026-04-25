cbuffer CB0 : register(b0)
{
    float2 EyeToSourceUVScale;
    float2 EyeToSourceUVOffset;
    float4x4 EyeRotationStart;
    float4x4 EyeRotationEnd;
    float2 LensCenter;
};

struct VS_INPUT
{
    float2 pos : POSITION;
    float4 color : COLOR;
    float2 red : TEXCOORD0;
    float2 green : TEXCOORD1;
    float2 blue : TEXCOORD2;
};

struct PS_INPUT
{
    float4 pos : SV_Position;
    float4 color : COLOR0;
    float3 red : TEXCOORD0;
    float3 green : TEXCOORD1;
    float3 blue : TEXCOORD2;
};

PS_INPUT main(VS_INPUT input)
{
    PS_INPUT output;

    output.pos = float4(LensCenter + input.pos, 0, 1);
    output.color = input.color;
    output.red = float3(mad(input.red, EyeToSourceUVScale, EyeToSourceUVOffset), 0);
    output.green = float3(mad(input.green, EyeToSourceUVScale, EyeToSourceUVOffset), 0);
    output.blue = float3(mad(input.blue, EyeToSourceUVScale, EyeToSourceUVOffset), 0);

    return output;
}
