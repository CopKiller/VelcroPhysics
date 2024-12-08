using System;
using System.Collections.Generic;
using System.Numerics;
using Genbox.VelcroPhysics.Dynamics;
using Genbox.VelcroPhysics.Extensions.Controllers.ControllerBase;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Extensions.Controllers.Gravity;

public class GravityController : Controller
{
    public GravityController(float strength)
        : base(ControllerType.GravityController)
    {
        Strength = strength;
        MaxRadius = float.MaxValue;
        GravityType = GravityType.DistanceSquared;
        Points = new List<Vector2>();
        Bodies = new List<Body>();
    }

    public GravityController(float strength, float maxRadius, float minRadius)
        : base(ControllerType.GravityController)
    {
        MinRadius = minRadius;
        MaxRadius = maxRadius;
        Strength = strength;
        GravityType = GravityType.DistanceSquared;
        Points = new List<Vector2>();
        Bodies = new List<Body>();
    }

    public float MinRadius { get; set; }
    public float MaxRadius { get; set; }
    public float Strength { get; set; }
    public GravityType GravityType { get; set; }
    public List<Body> Bodies { get; set; }
    public List<Vector2> Points { get; set; }

    public override void Update(float dt)
    {
        var f = Vector2.Zero;

        foreach (var worldBody in World.BodyList)
        {
            if (!IsActiveOn(worldBody))
                continue;

            foreach (var controllerBody in Bodies)
            {
                if (worldBody == controllerBody || (worldBody.IsStatic && controllerBody.IsStatic) ||
                    !controllerBody.Enabled)
                    continue;

                var d = controllerBody.Position - worldBody.Position;
                var r2 = d.LengthSquared();

                if (r2 <= MathConstants.Epsilon || r2 > MaxRadius * MaxRadius || r2 < MinRadius * MinRadius)
                    continue;

                switch (GravityType)
                {
                    case GravityType.DistanceSquared:
                        f = Strength / r2 * worldBody.Mass * controllerBody.Mass * d;
                        break;
                    case GravityType.Linear:
                        f = Strength / (float)Math.Sqrt(r2) * worldBody.Mass * controllerBody.Mass * d;
                        break;
                }

                worldBody.ApplyForce(ref f);
            }

            foreach (var point in Points)
            {
                var d = point - worldBody.Position;
                var r2 = d.LengthSquared();

                if (r2 <= MathConstants.Epsilon || r2 > MaxRadius * MaxRadius || r2 < MinRadius * MinRadius)
                    continue;

                switch (GravityType)
                {
                    case GravityType.DistanceSquared:
                        f = Strength / r2 * worldBody.Mass * d;
                        break;
                    case GravityType.Linear:
                        f = Strength / (float)Math.Sqrt(r2) * worldBody.Mass * d;
                        break;
                }

                worldBody.ApplyForce(ref f);
            }
        }
    }

    public void AddBody(Body body)
    {
        Bodies.Add(body);
    }

    public void AddPoint(Vector2 point)
    {
        Points.Add(point);
    }
}