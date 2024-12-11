using System;
using System.Numerics;

namespace Genbox.VelcroPhysics;

public static class ExtensionsMatrix4X4
{
    public static void CreateRotationZ(float radians, out Matrix4x4 result)
    {
        result = Identity;

        float val1 = (float)Math.Cos(radians);
        float val2 = (float)Math.Sin(radians);

        result.M11 = val1;
        result.M12 = val2;
        result.M21 = -val2;
        result.M22 = val1;
    }

    private static Matrix4x4 Identity { get; } = new Matrix4x4(1f, 0f, 0f, 0f,
        0f, 1f, 0f, 0f,
        0f, 0f, 1f, 0f,
        0f, 0f, 0f, 1f);
}