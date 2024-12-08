using System;
using System.Numerics;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics;

public static class ExtensionsVector2
{
    public static void Normalize(this ref Vector2 vector)
    {
        float val = 1.0f / (float)Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y);
        vector.X *= val;
        vector.Y *= val;
    }
    
    public static void Normalize(ref Vector2 value, out Vector2 result)
    {
        float val = 1.0f / (float)Math.Sqrt(value.X * value.X + value.Y * value.Y);
        result.X = value.X * val;
        result.Y = value.Y * val;
    }
    
    public static void Subtract(ref Vector2 value1, ref Vector2 value2, out Vector2 result)
    {
        result.X = value1.X - value2.X;
        result.Y = value1.Y - value2.Y;
    }
    
    public static void Transform(
        Vector2[] sourceArray,
        ref Matrix4x4 matrix,
        Vector2[] destinationArray)
    {
        Transform(sourceArray, 0, ref matrix, destinationArray, 0, sourceArray.Length);
    }
    
    public static void Transform(
        Vector2[] sourceArray,
        int sourceIndex,
        ref Matrix4x4 matrix,
        Vector2[] destinationArray,
        int destinationIndex,
        int length)
    {
        if (sourceArray == null)
            throw new ArgumentNullException(nameof(sourceArray));
        if (destinationArray == null)
            throw new ArgumentNullException(nameof(destinationArray));
        if (sourceArray.Length < sourceIndex + length)
            throw new ArgumentException("Source array length is lesser than sourceIndex + length");
        if (destinationArray.Length < destinationIndex + length)
            throw new ArgumentException("Destination array length is lesser than destinationIndex + length");

        for (int x = 0; x < length; x++)
        {
            Vector2 position = sourceArray[sourceIndex + x];
            Vector2 destination = destinationArray[destinationIndex + x];
            destination.X = position.X * matrix.M11 + position.Y * matrix.M21 + matrix.M41;
            destination.Y = position.X * matrix.M12 + position.Y * matrix.M22 + matrix.M42;
            destinationArray[destinationIndex + x] = destination;
        }
    }
    
    public static Vector2 CatmullRom(Vector2 value1, Vector2 value2, Vector2 value3, Vector2 value4, float amount)
    {
        return new Vector2(
            MathHelper.CatmullRom(value1.X, value2.X, value3.X, value4.X, amount),
            MathHelper.CatmullRom(value1.Y, value2.Y, value3.Y, value4.Y, amount));
    }
    
    public static void Min(ref Vector2 value1, ref Vector2 value2, out Vector2 result)
    {
        result.X = value1.X < value2.X ? value1.X : value2.X;
        result.Y = value1.Y < value2.Y ? value1.Y : value2.Y;
    }
    
    public static void Max(ref Vector2 value1, ref Vector2 value2, out Vector2 result)
    {
        result.X = value1.X > value2.X ? value1.X : value2.X;
        result.Y = value1.Y > value2.Y ? value1.Y : value2.Y;
    }
    
    public static void Multiply(ref Vector2 value1, float scaleFactor, out Vector2 result)
    {
        result.X = value1.X * scaleFactor;
        result.Y = value1.Y * scaleFactor;
    }
}