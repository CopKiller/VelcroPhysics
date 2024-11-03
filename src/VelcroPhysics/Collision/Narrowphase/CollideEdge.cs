using System.Diagnostics;
using Genbox.VelcroPhysics.Collision.ContactSystem;
using Genbox.VelcroPhysics.Collision.Shapes;
using Genbox.VelcroPhysics.Shared;
using Genbox.VelcroPhysics.Shared.Optimization;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Collision.Narrowphase;

public static class CollideEdge
{
    /// <summary>Compute contact points for edge versus circle. This accounts for edge connectivity.</summary>
    /// <param name="manifold">The manifold.</param>
    /// <param name="edgeA">The edge A.</param>
    /// <param name="transformA">The transform A.</param>
    /// <param name="circleB">The circle B.</param>
    /// <param name="transformB">The transform B.</param>
    public static void CollideEdgeAndCircle(ref Manifold manifold, EdgeShape edgeA, ref Transform transformA,
        CircleShape circleB, ref Transform transformB)
    {
        manifold.PointCount = 0;

        // Compute circle in frame of edge
        var Q = MathUtils.MulT(ref transformA, MathUtils.Mul(ref transformB, ref circleB._position));

        Vector2 A = edgeA.Vertex1, B = edgeA.Vertex2;
        var e = B - A;

        // Normal points to the right for a CCW winding
        var n = new Vector2(e.Y, -e.X);
        var offset = MathUtils.Dot(n, Q - A);

        var oneSided = edgeA.OneSided;
        if (oneSided && offset < 0.0f)
            return;

        // Barycentric coordinates
        var u = Vector2.Dot(e, B - Q);
        var v = Vector2.Dot(e, Q - A);

        var radius = edgeA._radius + circleB._radius;

        ContactFeature cf;
        cf.IndexB = 0;
        cf.TypeB = ContactFeatureType.Vertex;

        // Region A
        if (v <= 0.0f)
        {
            var P1 = A;
            var d1 = Q - P1;
            var dd1 = Vector2.Dot(d1, d1);
            if (dd1 > radius * radius)
                return;

            // Is there an edge connected to A?
            if (edgeA.OneSided)
            {
                var A1 = edgeA.Vertex0;
                var B1 = A;
                var e1 = B1 - A1;
                var u1 = Vector2.Dot(e1, B1 - Q);

                // Is the circle in Region AB of the previous edge?
                if (u1 > 0.0f)
                    return;
            }

            cf.IndexA = 0;
            cf.TypeA = ContactFeatureType.Vertex;
            manifold.PointCount = 1;
            manifold.Type = ManifoldType.Circles;
            manifold.LocalNormal = Vector2.Zero;
            manifold.LocalPoint = P1;
            manifold.Points.Value0.Id.Key = 0;
            manifold.Points.Value0.Id.ContactFeature = cf;
            manifold.Points.Value0.LocalPoint = circleB.Position;
            return;
        }

        // Region B
        if (u <= 0.0f)
        {
            var P2 = B;
            var d2 = Q - P2;
            var dd2 = Vector2.Dot(d2, d2);
            if (dd2 > radius * radius)
                return;

            // Is there an edge connected to B?
            if (edgeA.OneSided)
            {
                var B2 = edgeA.Vertex3;
                var A2 = B;
                var e2 = B2 - A2;
                var v2 = Vector2.Dot(e2, Q - A2);

                // Is the circle in Region AB of the next edge?
                if (v2 > 0.0f)
                    return;
            }

            cf.IndexA = 1;
            cf.TypeA = (byte)ContactFeatureType.Vertex;
            manifold.PointCount = 1;
            manifold.Type = ManifoldType.Circles;
            manifold.LocalNormal = Vector2.Zero;
            manifold.LocalPoint = P2;
            manifold.Points.Value0.Id.Key = 0;
            manifold.Points.Value0.Id.ContactFeature = cf;
            manifold.Points.Value0.LocalPoint = circleB.Position;
            return;
        }

        // Region AB
        var den = Vector2.Dot(e, e);
        Debug.Assert(den > 0.0f);
        var P = 1.0f / den * (u * A + v * B);
        var d = Q - P;
        var dd = Vector2.Dot(d, d);
        if (dd > radius * radius)
            return;

        if (offset < 0.0f)
            n = new Vector2(-n.X, -n.Y);

        n.Normalize();

        cf.IndexA = 0;
        cf.TypeA = ContactFeatureType.Face;
        manifold.PointCount = 1;
        manifold.Type = ManifoldType.FaceA;
        manifold.LocalNormal = n;
        manifold.LocalPoint = A;
        manifold.Points.Value0.Id.Key = 0;
        manifold.Points.Value0.Id.ContactFeature = cf;
        manifold.Points.Value0.LocalPoint = circleB.Position;
    }

    public static void CollideEdgeAndPolygon(ref Manifold manifold, EdgeShape edgeA, ref Transform xfA,
        PolygonShape polygonB, ref Transform xfB)
    {
        manifold.PointCount = 0;

        var xf = MathUtils.MulT(xfA, xfB);

        var centroidB = MathUtils.Mul(ref xf, polygonB._massData._centroid);

        var v1 = edgeA._vertex1;
        var v2 = edgeA._vertex2;

        var edge1 = v2 - v1;
        edge1.Normalize();

        // Normal points to the right for a CCW winding
        var normal1 = new Vector2(edge1.Y, -edge1.X);
        var offset1 = MathUtils.Dot(normal1, centroidB - v1);

        var oneSided = edgeA._oneSided;
        if (oneSided && offset1 < 0.0f) return;

        // Get polygonB in frameA
        var tempPolygonB = new TempPolygon(polygonB._vertices.Count);
        for (var i = 0; i < polygonB._vertices.Count; ++i)
        {
            tempPolygonB.Vertices[i] = MathUtils.Mul(ref xf, polygonB._vertices[i]);
            tempPolygonB.Normals[i] = MathUtils.Mul(xf.q, polygonB._normals[i]);
        }

        var radius = polygonB._radius + edgeA._radius;

        var edgeAxis = ComputeEdgeSeparation(ref tempPolygonB, v1, normal1);
        if (edgeAxis.Separation > radius) return;

        var polygonAxis = ComputePolygonSeparation(ref tempPolygonB, v1, v2);
        if (polygonAxis.Separation > radius) return;

        // Use hysteresis for jitter reduction.
        const float k_relativeTol = 0.98f;
        const float k_absoluteTol = 0.001f;

        EPAxis primaryAxis;
        if (polygonAxis.Separation - radius > k_relativeTol * (edgeAxis.Separation - radius) + k_absoluteTol)
            primaryAxis = polygonAxis;
        else
            primaryAxis = edgeAxis;

        if (oneSided)
        {
            // Smooth collision
            // See https://box2d.org/posts/2020/06/ghost-collisions/

            var edge0 = v1 - edgeA._vertex0;
            edge0.Normalize();
            var normal0 = new Vector2(edge0.Y, -edge0.X);
            var convex1 = MathUtils.Cross(edge0, edge1) >= 0.0f;

            var edge2 = edgeA._vertex3 - v2;
            edge2.Normalize();
            var normal2 = new Vector2(edge2.Y, -edge2.X);
            var convex2 = MathUtils.Cross(edge1, edge2) >= 0.0f;

            const float sinTol = 0.1f;
            var side1 = MathUtils.Dot(primaryAxis.Normal, edge1) <= 0.0f;

            // Check Gauss Map
            if (side1)
            {
                if (convex1)
                {
                    if (MathUtils.Cross(primaryAxis.Normal, normal0) > sinTol)
                        // Skip region
                        return;

                    // Admit region
                }
                else
                {
                    // Snap region
                    primaryAxis = edgeAxis;
                }
            }
            else
            {
                if (convex2)
                {
                    if (MathUtils.Cross(normal2, primaryAxis.Normal) > sinTol)
                        // Skip region
                        return;

                    // Admit region
                }
                else
                {
                    // Snap region
                    primaryAxis = edgeAxis;
                }
            }
        }

        var clipPoints = new FixedArray2<ClipVertex>();
        ReferenceFace ref1;
        if (primaryAxis.Type == EPAxisType.EdgeA)
        {
            manifold.Type = ManifoldType.FaceA;

            // Search for the polygon normal that is most anti-parallel to the edge normal.
            var bestIndex = 0;
            var bestValue = MathUtils.Dot(primaryAxis.Normal, tempPolygonB.Normals[0]);
            for (var i = 1; i < tempPolygonB.Count; ++i)
            {
                var value = MathUtils.Dot(primaryAxis.Normal, tempPolygonB.Normals[i]);
                if (value < bestValue)
                {
                    bestValue = value;
                    bestIndex = i;
                }
            }

            var i1 = bestIndex;
            var i2 = i1 + 1 < tempPolygonB.Count ? i1 + 1 : 0;

            clipPoints.Value0.V = tempPolygonB.Vertices[i1];
            clipPoints.Value0.Id.ContactFeature.IndexA = 0;
            clipPoints.Value0.Id.ContactFeature.IndexB = (byte)i1;
            clipPoints.Value0.Id.ContactFeature.TypeA = ContactFeatureType.Face;
            clipPoints.Value0.Id.ContactFeature.TypeB = ContactFeatureType.Vertex;

            clipPoints.Value1.V = tempPolygonB.Vertices[i2];
            clipPoints.Value1.Id.ContactFeature.IndexA = 0;
            clipPoints.Value1.Id.ContactFeature.IndexB = (byte)i2;
            clipPoints.Value1.Id.ContactFeature.TypeA = ContactFeatureType.Face;
            clipPoints.Value1.Id.ContactFeature.TypeB = ContactFeatureType.Vertex;

            ref1.i1 = 0;
            ref1.i2 = 1;
            ref1.v1 = v1;
            ref1.v2 = v2;
            ref1.Normal = primaryAxis.Normal;
            ref1.SideNormal1 = -edge1;
            ref1.SideNormal2 = edge1;
        }
        else
        {
            manifold.Type = ManifoldType.FaceB;

            clipPoints.Value0.V = v2;
            clipPoints.Value0.Id.ContactFeature.IndexA = 1;
            clipPoints.Value0.Id.ContactFeature.IndexB = (byte)primaryAxis.Index;
            clipPoints.Value0.Id.ContactFeature.TypeA = ContactFeatureType.Vertex;
            clipPoints.Value0.Id.ContactFeature.TypeB = ContactFeatureType.Face;

            clipPoints.Value1.V = v1;
            clipPoints.Value1.Id.ContactFeature.IndexA = 0;
            clipPoints.Value1.Id.ContactFeature.IndexB = (byte)primaryAxis.Index;
            clipPoints.Value1.Id.ContactFeature.TypeA = ContactFeatureType.Vertex;
            clipPoints.Value1.Id.ContactFeature.TypeB = ContactFeatureType.Face;

            ref1.i1 = primaryAxis.Index;
            ref1.i2 = ref1.i1 + 1 < tempPolygonB.Count ? ref1.i1 + 1 : 0;
            ref1.v1 = tempPolygonB.Vertices[ref1.i1];
            ref1.v2 = tempPolygonB.Vertices[ref1.i2];
            ref1.Normal = tempPolygonB.Normals[ref1.i1];

            // CCW winding
            ref1.SideNormal1 = new Vector2(ref1.Normal.Y, -ref1.Normal.X);
            ref1.SideNormal2 = -ref1.SideNormal1;
        }

        ref1.SideOffset1 = MathUtils.Dot(ref1.SideNormal1, ref1.v1);
        ref1.SideOffset2 = MathUtils.Dot(ref1.SideNormal2, ref1.v2);

        // Clip incident edge against reference face side planes
        FixedArray2<ClipVertex> clipPoints1;
        FixedArray2<ClipVertex> clipPoints2;
        int np;

        // Clip to side 1
        np = Collision.ClipSegmentToLine(out clipPoints1, ref clipPoints, ref1.SideNormal1, ref1.SideOffset1, ref1.i1);

        if (np < Settings.MaxManifoldPoints) return;

        // Clip to side 2
        np = Collision.ClipSegmentToLine(out clipPoints2, ref clipPoints1, ref1.SideNormal2, ref1.SideOffset2, ref1.i2);

        if (np < Settings.MaxManifoldPoints) return;

        // Now clipPoints2 contains the clipped points.
        if (primaryAxis.Type == EPAxisType.EdgeA)
        {
            manifold.LocalNormal = ref1.Normal;
            manifold.LocalPoint = ref1.v1;
        }
        else
        {
            manifold.LocalNormal = polygonB._normals[ref1.i1];
            manifold.LocalPoint = polygonB._vertices[ref1.i1];
        }

        var pointCount = 0;
        for (var i = 0; i < Settings.MaxManifoldPoints; ++i)
        {
            var separation = MathUtils.Dot(ref1.Normal, clipPoints2[i].V - ref1.v1);

            if (separation <= radius)
            {
                var cp = manifold.Points[pointCount];

                if (primaryAxis.Type == EPAxisType.EdgeA)
                {
                    cp.LocalPoint = MathUtils.MulT(xf, clipPoints2[i].V);
                    cp.Id = clipPoints2[i].Id;
                }
                else
                {
                    cp.LocalPoint = clipPoints2[i].V;
                    cp.Id.ContactFeature.TypeA = clipPoints2[i].Id.ContactFeature.TypeB;
                    cp.Id.ContactFeature.TypeB = clipPoints2[i].Id.ContactFeature.TypeA;
                    cp.Id.ContactFeature.IndexA = clipPoints2[i].Id.ContactFeature.IndexB;
                    cp.Id.ContactFeature.IndexB = clipPoints2[i].Id.ContactFeature.IndexA;
                }

                manifold.Points[pointCount] = cp;
                ++pointCount;
            }
        }

        manifold.PointCount = pointCount;
    }

    private static EPAxis ComputeEdgeSeparation(ref TempPolygon polygonB, Vector2 v1, Vector2 normal1)
    {
        EPAxis axis;
        axis.Type = EPAxisType.EdgeA;
        axis.Index = -1;
        axis.Separation = -MathConstants.MaxFloat;
        axis.Normal = Vector2.Zero;

        Vector2[] axes = { normal1, -normal1 };

        // Find axis with least overlap (min-max problem)
        for (var j = 0; j < 2; ++j)
        {
            var sj = MathConstants.MaxFloat;

            // Find deepest polygon vertex along axis j
            for (var i = 0; i < polygonB.Count; ++i)
            {
                var si = MathUtils.Dot(axes[j], polygonB.Vertices[i] - v1);
                if (si < sj) sj = si;
            }

            if (sj > axis.Separation)
            {
                axis.Index = j;
                axis.Separation = sj;
                axis.Normal = axes[j];
            }
        }

        return axis;
    }

    private static EPAxis ComputePolygonSeparation(ref TempPolygon polygonB, Vector2 v1, Vector2 v2)
    {
        EPAxis axis;
        axis.Type = EPAxisType.Unknown;
        axis.Index = -1;
        axis.Separation = -MathConstants.MaxFloat;
        axis.Normal = Vector2.Zero;

        for (var i = 0; i < polygonB.Count; ++i)
        {
            var n = -polygonB.Normals[i];

            var s1 = MathUtils.Dot(n, polygonB.Vertices[i] - v1);
            var s2 = MathUtils.Dot(n, polygonB.Vertices[i] - v2);
            var s = MathUtils.Min(s1, s2);

            if (s > axis.Separation)
            {
                axis.Type = EPAxisType.EdgeB;
                axis.Index = i;
                axis.Separation = s;
                axis.Normal = n;
            }
        }

        return axis;
    }

    private struct TempPolygon
    {
        public TempPolygon(int count)
        {
            Count = count;
            Vertices = new Vector2[count];
            Normals = new Vector2[count];
        }

        public readonly Vector2[] Vertices;
        public readonly Vector2[] Normals;
        public readonly int Count;
    }
}