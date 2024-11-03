using Genbox.VelcroPhysics.Collision.Shapes;
using Genbox.VelcroPhysics.Shared;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Collision.Narrowphase;

public static class CollideCircle
{
    /// <summary>Compute the collision manifold between two circles.</summary>
    public static void CollideCircles(ref Manifold manifold, CircleShape circleA, ref Transform xfA,
        CircleShape circleB, ref Transform xfB)
    {
        manifold.PointCount = 0;

        var pA = MathUtils.Mul(ref xfA, circleA.Position);
        var pB = MathUtils.Mul(ref xfB, circleB.Position);

        var d = pB - pA;
        var distSqr = Vector2.Dot(d, d);
        float rA = circleA._radius, rB = circleB._radius;
        var radius = rA + rB;
        if (distSqr > radius * radius)
            return;

        manifold.Type = ManifoldType.Circles;
        manifold.LocalPoint = circleA.Position;
        manifold.LocalNormal = Vector2.Zero;
        manifold.PointCount = 1;

        var p0 = manifold.Points[0];
        p0.LocalPoint = circleB.Position;
        p0.Id.Key = 0;
        manifold.Points[0] = p0;
    }

    /// <summary>Compute the collision manifold between a polygon and a circle.</summary>
    /// <param name="manifold">The manifold.</param>
    /// <param name="polygonA">The polygon A.</param>
    /// <param name="xfA">The transform of A.</param>
    /// <param name="circleB">The circle B.</param>
    /// <param name="xfB">The transform of B.</param>
    public static void CollidePolygonAndCircle(ref Manifold manifold, PolygonShape polygonA, ref Transform xfA,
        CircleShape circleB, ref Transform xfB)
    {
        manifold.PointCount = 0;

        // Compute circle position in the frame of the polygon.
        var c = MathUtils.Mul(ref xfB, circleB.Position);
        var cLocal = MathUtils.MulT(ref xfA, c);

        // Find the min separating edge.
        var normalIndex = 0;
        var separation = -MathConstants.MaxFloat;
        var radius = polygonA._radius + circleB._radius;
        var vertexCount = polygonA._vertices.Count;
        var vertices = polygonA._vertices;
        var normals = polygonA._normals;

        for (var i = 0; i < vertexCount; ++i)
        {
            var s = Vector2.Dot(normals[i], cLocal - vertices[i]);

            if (s > radius)
                // Early out.
                return;

            if (s > separation)
            {
                separation = s;
                normalIndex = i;
            }
        }

        // Vertices that subtend the incident face.
        var vertIndex1 = normalIndex;
        var vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
        var v1 = vertices[vertIndex1];
        var v2 = vertices[vertIndex2];

        // If the center is inside the polygon ...
        if (separation < MathConstants.Epsilon)
        {
            manifold.PointCount = 1;
            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = normals[normalIndex];
            manifold.LocalPoint = 0.5f * (v1 + v2);
            manifold.Points.Value0.LocalPoint = circleB.Position;
            manifold.Points.Value0.Id.Key = 0;
            return;
        }

        // Compute barycentric coordinates
        var u1 = Vector2.Dot(cLocal - v1, v2 - v1);
        var u2 = Vector2.Dot(cLocal - v2, v1 - v2);

        if (u1 <= 0.0f)
        {
            if (Vector2.DistanceSquared(cLocal, v1) > radius * radius)
                return;

            manifold.PointCount = 1;
            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = cLocal - v1;
            manifold.LocalNormal.Normalize();
            manifold.LocalPoint = v1;
            manifold.Points.Value0.LocalPoint = circleB.Position;
            manifold.Points.Value0.Id.Key = 0;
        }
        else if (u2 <= 0.0f)
        {
            if (Vector2.DistanceSquared(cLocal, v2) > radius * radius)
                return;

            manifold.PointCount = 1;
            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = cLocal - v2;
            manifold.LocalNormal.Normalize();
            manifold.LocalPoint = v2;
            manifold.Points.Value0.LocalPoint = circleB.Position;
            manifold.Points.Value0.Id.Key = 0;
        }
        else
        {
            var faceCenter = 0.5f * (v1 + v2);
            var s = Vector2.Dot(cLocal - faceCenter, normals[vertIndex1]);
            if (s > radius)
                return;

            manifold.PointCount = 1;
            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = normals[vertIndex1];
            manifold.LocalPoint = faceCenter;
            manifold.Points.Value0.LocalPoint = circleB.Position;
            manifold.Points.Value0.Id.Key = 0;
        }
    }
}