using System;
using System.Collections.Generic;

namespace Genbox.VelcroPhysics.Tools.Triangulation.Delaunay.Util;

internal class PointGenerator
{
    private static readonly Random RNG = new();

    public static List<TriangulationPoint> UniformDistribution(int n, double scale)
    {
        var points = new List<TriangulationPoint>();
        for (var i = 0; i < n; i++)
            points.Add(new TriangulationPoint(scale * (0.5 - RNG.NextDouble()), scale * (0.5 - RNG.NextDouble())));
        return points;
    }

    public static List<TriangulationPoint> UniformGrid(int n, double scale)
    {
        double x = 0;
        var size = scale / n;
        var halfScale = 0.5 * scale;

        var points = new List<TriangulationPoint>();
        for (var i = 0; i < n + 1; i++)
        {
            x = halfScale - i * size;
            for (var j = 0; j < n + 1; j++) points.Add(new TriangulationPoint(x, halfScale - j * size));
        }

        return points;
    }
}