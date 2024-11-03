using Genbox.VelcroPhysics.Shared;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Tools.Triangulation.Earclip;

public class Triangle : Vertices
{
    //Constructor automatically fixes orientation to ccw
    public Triangle(float x1, float y1, float x2, float y2, float x3, float y3)
    {
        var cross = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
        if (cross > 0)
        {
            Add(new Vector2(x1, y1));
            Add(new Vector2(x2, y2));
            Add(new Vector2(x3, y3));
        }
        else
        {
            Add(new Vector2(x1, y1));
            Add(new Vector2(x3, y3));
            Add(new Vector2(x2, y2));
        }
    }

    public bool IsInside(float x, float y)
    {
        var a = this[0];
        var b = this[1];
        var c = this[2];

        if (x < a.X && x < b.X && x < c.X) return false;
        if (x > a.X && x > b.X && x > c.X) return false;
        if (y < a.Y && y < b.Y && y < c.Y) return false;
        if (y > a.Y && y > b.Y && y > c.Y) return false;

        var vx2 = x - a.X;
        var vy2 = y - a.Y;
        var vx1 = b.X - a.X;
        var vy1 = b.Y - a.Y;
        var vx0 = c.X - a.X;
        var vy0 = c.Y - a.Y;

        var dot00 = vx0 * vx0 + vy0 * vy0;
        var dot01 = vx0 * vx1 + vy0 * vy1;
        var dot02 = vx0 * vx2 + vy0 * vy2;
        var dot11 = vx1 * vx1 + vy1 * vy1;
        var dot12 = vx1 * vx2 + vy1 * vy2;
        var invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return u > 0 && v > 0 && u + v < 1;
    }
}