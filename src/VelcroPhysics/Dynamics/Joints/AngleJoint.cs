/*
 * Velcro Physics:
 * Copyright (c) 2017 Ian Qvist
 */

using System.Diagnostics;
using System.Numerics;
using Genbox.VelcroPhysics.Dynamics.Joints.Misc;
using Genbox.VelcroPhysics.Dynamics.Solver;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Dynamics.Joints;

/// <summary>Maintains a fixed angle between two bodies</summary>
public class AngleJoint : Joint
{
    private float _bias;
    private float _jointError;
    private float _massFactor;
    private float _targetAngle;

    public AngleJoint(Body bodyA, Body bodyB)
        : base(bodyA, bodyB, JointType.Angle)
    {
        BiasFactor = .2f;
        MaxImpulse = float.MaxValue;
    }

    public override Vector2 WorldAnchorA
    {
        get => _bodyA.Position;
        set => Debug.Assert(false, "You can't set the world anchor on this joint type.");
    }

    public override Vector2 WorldAnchorB
    {
        get => _bodyB.Position;
        set => Debug.Assert(false, "You can't set the world anchor on this joint type.");
    }

    /// <summary>The desired angle between BodyA and BodyB</summary>
    public float TargetAngle
    {
        get => _targetAngle;
        set
        {
            if (_targetAngle != value)
            {
                _targetAngle = value;
                WakeBodies();
            }
        }
    }

    /// <summary>Gets or sets the bias factor. Defaults to 0.2</summary>
    public float BiasFactor { get; set; }

    /// <summary>Gets or sets the maximum impulse. Defaults to float.MaxValue</summary>
    public float MaxImpulse { get; set; }

    /// <summary>Gets or sets the softness of the joint. Defaults to 0</summary>
    public float Softness { get; set; }

    public override Vector2 GetReactionForce(float invDt)
    {
        return Vector2.Zero;
    }

    public override float GetReactionTorque(float invDt)
    {
        return 0;
    }

    internal override void InitVelocityConstraints(ref SolverData data)
    {
        var indexA = _bodyA.IslandIndex;
        var indexB = _bodyB.IslandIndex;

        var aW = data.Positions[indexA].A;
        var bW = data.Positions[indexB].A;

        _jointError = bW - aW - _targetAngle;
        _bias = -BiasFactor * data.Step.InvertedDeltaTime * _jointError;
        _massFactor = (1 - Softness) / (_bodyA._invI + _bodyB._invI);
    }

    internal override void SolveVelocityConstraints(ref SolverData data)
    {
        var indexA = _bodyA.IslandIndex;
        var indexB = _bodyB.IslandIndex;

        var p = (_bias - data.Velocities[indexB].W + data.Velocities[indexA].W) * _massFactor;

        data.Velocities[indexA].W -= _bodyA._invI * MathUtils.Sign(p) * MathUtils.Min(MathUtils.Abs(p), MaxImpulse);
        data.Velocities[indexB].W += _bodyB._invI * MathUtils.Sign(p) * MathUtils.Min(MathUtils.Abs(p), MaxImpulse);
    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        //no position solving for this joint
        return true;
    }
}