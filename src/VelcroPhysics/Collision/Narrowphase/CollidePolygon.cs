using System.Diagnostics;
using System.Numerics;
using Genbox.VelcroPhysics.Collision.ContactSystem;
using Genbox.VelcroPhysics.Collision.Shapes;
using Genbox.VelcroPhysics.Shared;
using Genbox.VelcroPhysics.Shared.Optimization;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Collision.Narrowphase;

public static class CollidePolygon
{
    /// <summary>Compute the collision manifold between two polygons.</summary>
    public static void CollidePolygons(ref Manifold manifold, PolygonShape polyA, ref Transform xfA, PolygonShape polyB,
        ref Transform xfB)
    {
        // Find edge normal of max separation on A - return if separating axis is found
        // Find edge normal of max separation on B - return if separation axis is found
        // Choose reference edge as min(minA, minB)
        // Find incident edge
        // Clip

        manifold.PointCount = 0;
        var totalRadius = polyA._radius + polyB._radius;

        var separationA = FindMaxSeparation(out var edgeA, polyA, ref xfA, polyB, ref xfB);
        if (separationA > totalRadius)
            return;

        var separationB = FindMaxSeparation(out var edgeB, polyB, ref xfB, polyA, ref xfA);
        if (separationB > totalRadius)
            return;

        PolygonShape poly1; // reference polygon
        PolygonShape poly2; // incident polygon
        Transform xf1, xf2;
        int edge1; // reference edge
        bool flip;
        var k_tol = 0.1f * Settings.LinearSlop;

        if (separationB > separationA + k_tol)
        {
            poly1 = polyB;
            poly2 = polyA;
            xf1 = xfB;
            xf2 = xfA;
            edge1 = edgeB;
            manifold.Type = ManifoldType.FaceB;
            flip = true;
        }
        else
        {
            poly1 = polyA;
            poly2 = polyB;
            xf1 = xfA;
            xf2 = xfB;
            edge1 = edgeA;
            manifold.Type = ManifoldType.FaceA;
            flip = false;
        }

        FindIncidentEdge(out var incidentEdge, poly1, ref xf1, edge1, poly2, ref xf2);

        var count1 = poly1._vertices.Count;
        var vertices1 = poly1._vertices;

        var iv1 = edge1;
        var iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

        var v11 = vertices1[iv1];
        var v12 = vertices1[iv2];

        var localTangent = v12 - v11;
        localTangent.Normalize();

        var localNormal = MathUtils.Cross(localTangent, 1.0f);
        var planePoint = 0.5f * (v11 + v12);

        var tangent = MathUtils.Mul(ref xf1.q, localTangent);
        var normal = MathUtils.Cross(tangent, 1.0f);

        v11 = MathUtils.Mul(ref xf1, v11);
        v12 = MathUtils.Mul(ref xf1, v12);

        // Face offset.
        var frontOffset = Vector2.Dot(normal, v11);

        // Side offsets, extended by polytope skin thickness.
        var sideOffset1 = -Vector2.Dot(tangent, v11) + totalRadius;
        var sideOffset2 = Vector2.Dot(tangent, v12) + totalRadius;

        // Clip incident edge against extruded edge1 side edges.

        // Clip to box side 1
        var np = Collision.ClipSegmentToLine(out var clipPoints1, ref incidentEdge, -tangent, sideOffset1, iv1);

        if (np < 2)
            return;

        // Clip to negative box side 1
        np = Collision.ClipSegmentToLine(out var clipPoints2, ref clipPoints1, tangent, sideOffset2, iv2);

        if (np < 2)
            return;

        // Now clipPoints2 contains the clipped points.
        manifold.LocalNormal = localNormal;
        manifold.LocalPoint = planePoint;

        var pointCount = 0;
        for (var i = 0; i < Settings.MaxManifoldPoints; ++i)
        {
            var separation = Vector2.Dot(normal, clipPoints2[i].V) - frontOffset;

            if (separation <= totalRadius)
            {
                var cp = manifold.Points[pointCount];
                cp.LocalPoint = MathUtils.MulT(ref xf2, clipPoints2[i].V);
                cp.Id = clipPoints2[i].Id;

                if (flip)
                {
                    // Swap features
                    var cf = cp.Id.ContactFeature;
                    cp.Id.ContactFeature.IndexA = cf.IndexB;
                    cp.Id.ContactFeature.IndexB = cf.IndexA;
                    cp.Id.ContactFeature.TypeA = cf.TypeB;
                    cp.Id.ContactFeature.TypeB = cf.TypeA;
                }

                manifold.Points[pointCount] = cp;

                ++pointCount;
            }
        }

        manifold.PointCount = pointCount;
    }

    /// <summary>Find the max separation between poly1 and poly2 using edge normals from poly1.</summary>
    private static float FindMaxSeparation(out int edgeIndex, PolygonShape poly1, ref Transform xf1, PolygonShape poly2,
        ref Transform xf2)
    {
        var count1 = poly1._vertices.Count;
        var count2 = poly2._vertices.Count;
        var n1s = poly1._normals;
        var v1s = poly1._vertices;
        var v2s = poly2._vertices;
        var xf = MathUtils.MulT(xf2, xf1);

        var bestIndex = 0;
        var maxSeparation = -MathConstants.MaxFloat;
        for (var i = 0; i < count1; ++i)
        {
            // Get poly1 normal in frame2.
            var n = MathUtils.Mul(ref xf.q, n1s[i]);
            var v1 = MathUtils.Mul(ref xf, v1s[i]);

            // Find deepest point for normal i.
            var si = MathConstants.MaxFloat;
            for (var j = 0; j < count2; ++j)
            {
                var sij = Vector2.Dot(n, v2s[j] - v1);
                if (sij < si)
                    si = sij;
            }

            if (si > maxSeparation)
            {
                maxSeparation = si;
                bestIndex = i;
            }
        }

        edgeIndex = bestIndex;
        return maxSeparation;
    }

    private static void FindIncidentEdge(out FixedArray2<ClipVertex> c, PolygonShape poly1, ref Transform xf1,
        int edge1, PolygonShape poly2, ref Transform xf2)
    {
        var normals1 = poly1._normals;

        var count2 = poly2._vertices.Count;
        var vertices2 = poly2._vertices;
        var normals2 = poly2._normals;

        Debug.Assert(0 <= edge1 && edge1 < poly1._vertices.Count);

        // Get the normal of the reference edge in poly2's frame.
        var normal1 = MathUtils.MulT(ref xf2.q, MathUtils.Mul(ref xf1.q, normals1[edge1]));

        // Find the incident edge on poly2.
        var index = 0;
        var minDot = MathConstants.MaxFloat;
        for (var i = 0; i < count2; ++i)
        {
            var dot = Vector2.Dot(normal1, normals2[i]);
            if (dot < minDot)
            {
                minDot = dot;
                index = i;
            }
        }

        // Build the clip vertices for the incident edge.
        var i1 = index;
        var i2 = i1 + 1 < count2 ? i1 + 1 : 0;

        c = new FixedArray2<ClipVertex>();
        c.Value0.V = MathUtils.Mul(ref xf2, vertices2[i1]);
        c.Value0.Id.ContactFeature.IndexA = (byte)edge1;
        c.Value0.Id.ContactFeature.IndexB = (byte)i1;
        c.Value0.Id.ContactFeature.TypeA = ContactFeatureType.Face;
        c.Value0.Id.ContactFeature.TypeB = ContactFeatureType.Vertex;

        c.Value1.V = MathUtils.Mul(ref xf2, vertices2[i2]);
        c.Value1.Id.ContactFeature.IndexA = (byte)edge1;
        c.Value1.Id.ContactFeature.IndexB = (byte)i2;
        c.Value1.Id.ContactFeature.TypeA = ContactFeatureType.Face;
        c.Value1.Id.ContactFeature.TypeB = ContactFeatureType.Vertex;
    }
}