using System.Diagnostics;
using Genbox.VelcroPhysics.Collision.Distance;
using Genbox.VelcroPhysics.Collision.Narrowphase;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Collision.TOI;

public static class SeparationFunction
{
    public static void Initialize(ref SimplexCache cache, DistanceProxy proxyA, ref Sweep sweepA, DistanceProxy proxyB,
        ref Sweep sweepB, float t1, out Vector2 axis, out Vector2 localPoint, out SeparationFunctionType type)
    {
        int count = cache.Count;
        Debug.Assert(0 < count && count < 3);

        sweepA.GetTransform(out var xfA, t1);
        sweepB.GetTransform(out var xfB, t1);

        if (count == 1)
        {
            localPoint = Vector2.Zero;
            type = SeparationFunctionType.Points;
            var localPointA = proxyA._vertices[cache.IndexA[0]];
            var localPointB = proxyB._vertices[cache.IndexB[0]];
            var pointA = MathUtils.Mul(ref xfA, localPointA);
            var pointB = MathUtils.Mul(ref xfB, localPointB);
            axis = pointB - pointA;
            axis.Normalize();
        }
        else if (cache.IndexA[0] == cache.IndexA[1])
        {
            // Two points on B and one on A.
            type = SeparationFunctionType.FaceB;
            var localPointB1 = proxyB._vertices[cache.IndexB[0]];
            var localPointB2 = proxyB._vertices[cache.IndexB[1]];

            var a = localPointB2 - localPointB1;
            axis = new Vector2(a.Y, -a.X);
            axis.Normalize();
            var normal = MathUtils.Mul(ref xfB.q, axis);

            localPoint = 0.5f * (localPointB1 + localPointB2);
            var pointB = MathUtils.Mul(ref xfB, localPoint);

            var localPointA = proxyA._vertices[cache.IndexA[0]];
            var pointA = MathUtils.Mul(ref xfA, localPointA);

            var s = Vector2.Dot(pointA - pointB, normal);
            if (s < 0.0f)
                axis = -axis;
        }
        else
        {
            // Two points on A and one or two points on B.
            type = SeparationFunctionType.FaceA;
            var localPointA1 = proxyA._vertices[cache.IndexA[0]];
            var localPointA2 = proxyA._vertices[cache.IndexA[1]];

            var a = localPointA2 - localPointA1;
            axis = new Vector2(a.Y, -a.X);
            axis.Normalize();
            var normal = MathUtils.Mul(ref xfA.q, axis);

            localPoint = 0.5f * (localPointA1 + localPointA2);
            var pointA = MathUtils.Mul(ref xfA, localPoint);

            var localPointB = proxyB._vertices[cache.IndexB[0]];
            var pointB = MathUtils.Mul(ref xfB, localPointB);

            var s = Vector2.Dot(pointB - pointA, normal);
            if (s < 0.0f)
                axis = -axis;
        }

        //Velcro note: the returned value that used to be here has been removed, as it was not used.
    }

    public static float FindMinSeparation(out int indexA, out int indexB, float t, DistanceProxy proxyA,
        ref Sweep sweepA, DistanceProxy proxyB, ref Sweep sweepB, ref Vector2 axis, ref Vector2 localPoint,
        SeparationFunctionType type)
    {
        sweepA.GetTransform(out var xfA, t);
        sweepB.GetTransform(out var xfB, t);

        switch (type)
        {
            case SeparationFunctionType.Points:
            {
                var axisA = MathUtils.MulT(ref xfA.q, axis);
                var axisB = MathUtils.MulT(ref xfB.q, -axis);

                indexA = proxyA.GetSupport(axisA);
                indexB = proxyB.GetSupport(axisB);

                var localPointA = proxyA._vertices[indexA];
                var localPointB = proxyB._vertices[indexB];

                var pointA = MathUtils.Mul(ref xfA, localPointA);
                var pointB = MathUtils.Mul(ref xfB, localPointB);

                var separation = Vector2.Dot(pointB - pointA, axis);
                return separation;
            }

            case SeparationFunctionType.FaceA:
            {
                var normal = MathUtils.Mul(ref xfA.q, axis);
                var pointA = MathUtils.Mul(ref xfA, localPoint);

                var axisB = MathUtils.MulT(ref xfB.q, -normal);

                indexA = -1;
                indexB = proxyB.GetSupport(axisB);

                var localPointB = proxyB._vertices[indexB];
                var pointB = MathUtils.Mul(ref xfB, localPointB);

                var separation = Vector2.Dot(pointB - pointA, normal);
                return separation;
            }

            case SeparationFunctionType.FaceB:
            {
                var normal = MathUtils.Mul(ref xfB.q, axis);
                var pointB = MathUtils.Mul(ref xfB, localPoint);

                var axisA = MathUtils.MulT(ref xfA.q, -normal);

                indexB = -1;
                indexA = proxyA.GetSupport(axisA);

                var localPointA = proxyA._vertices[indexA];
                var pointA = MathUtils.Mul(ref xfA, localPointA);

                var separation = Vector2.Dot(pointA - pointB, normal);
                return separation;
            }

            default:
                Debug.Assert(false);
                indexA = -1;
                indexB = -1;
                return 0.0f;
        }
    }

    public static float Evaluate(int indexA, int indexB, float t, DistanceProxy proxyA, ref Sweep sweepA,
        DistanceProxy proxyB, ref Sweep sweepB, ref Vector2 axis, ref Vector2 localPoint, SeparationFunctionType type)
    {
        sweepA.GetTransform(out var xfA, t);
        sweepB.GetTransform(out var xfB, t);

        switch (type)
        {
            case SeparationFunctionType.Points:
            {
                var localPointA = proxyA._vertices[indexA];
                var localPointB = proxyB._vertices[indexB];

                var pointA = MathUtils.Mul(ref xfA, localPointA);
                var pointB = MathUtils.Mul(ref xfB, localPointB);
                var separation = Vector2.Dot(pointB - pointA, axis);

                return separation;
            }
            case SeparationFunctionType.FaceA:
            {
                var normal = MathUtils.Mul(ref xfA.q, axis);
                var pointA = MathUtils.Mul(ref xfA, localPoint);

                var localPointB = proxyB._vertices[indexB];
                var pointB = MathUtils.Mul(ref xfB, localPointB);

                var separation = Vector2.Dot(pointB - pointA, normal);
                return separation;
            }
            case SeparationFunctionType.FaceB:
            {
                var normal = MathUtils.Mul(ref xfB.q, axis);
                var pointB = MathUtils.Mul(ref xfB, localPoint);

                var localPointA = proxyA._vertices[indexA];
                var pointA = MathUtils.Mul(ref xfA, localPointA);

                var separation = Vector2.Dot(pointA - pointB, normal);
                return separation;
            }
            default:
                Debug.Assert(false);
                return 0.0f;
        }
    }
}