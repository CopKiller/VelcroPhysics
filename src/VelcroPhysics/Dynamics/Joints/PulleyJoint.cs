/*
 * Velcro Physics:
 * Copyright (c) 2017 Ian Qvist
 *
 * Original source Box2D:
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

using System;
using System.Diagnostics;
using Genbox.VelcroPhysics.Definitions.Joints;
using Genbox.VelcroPhysics.Dynamics.Joints.Misc;
using Genbox.VelcroPhysics.Dynamics.Solver;
using Genbox.VelcroPhysics.Shared;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Dynamics.Joints;
// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

/// <summary>
///     The pulley joint is connected to two bodies and two fixed world points. The pulley supports a ratio such that:
///     <![CDATA[length1 + ratio * length2 <= constant]]>
///     Yes, the force transmitted is scaled by the ratio. Warning: the pulley joint can get a bit squirrelly by itself.
///     They
///     often work better when combined with prismatic joints. You should also cover the the anchor points with static
///     shapes
///     to prevent one side from going to zero length.
/// </summary>
public class PulleyJoint : Joint
{
    private readonly float _constant;
    private float _impulse;

    // Solver temp
    private int _indexA;
    private int _indexB;
    private float _invIA;
    private float _invIB;
    private float _invMassA;
    private float _invMassB;

    // Solver shared
    private Vector2 _localCenterA;
    private Vector2 _localCenterB;
    private float _mass;
    private Vector2 _rA;
    private Vector2 _rB;
    private Vector2 _uA;
    private Vector2 _uB;

    public PulleyJoint(PulleyJointDef def)
        : base(def)
    {
        WorldAnchorA = def.GroundAnchorA;
        WorldAnchorB = def.GroundAnchorB;
        LocalAnchorA = def.LocalAnchorA;
        LocalAnchorB = def.LocalAnchorB;

        LengthA = def.LengthA;
        LengthB = def.LengthB;

        Debug.Assert(def.Ratio != 0.0f);
        Ratio = def.Ratio;

        _constant = def.LengthA + Ratio * def.LengthB;
    }

    /// <summary>Constructor for PulleyJoint.</summary>
    /// <param name="bodyA">The first body.</param>
    /// <param name="bodyB">The second body.</param>
    /// <param name="anchorA">The anchor on the first body.</param>
    /// <param name="anchorB">The anchor on the second body.</param>
    /// <param name="worldAnchorA">The world anchor for the first body.</param>
    /// <param name="worldAnchorB">The world anchor for the second body.</param>
    /// <param name="ratio">The ratio.</param>
    /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
    public PulleyJoint(Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, Vector2 worldAnchorA,
        Vector2 worldAnchorB, float ratio, bool useWorldCoordinates = false)
        : base(bodyA, bodyB, JointType.Pulley)
    {
        WorldAnchorA = worldAnchorA;
        WorldAnchorB = worldAnchorB;

        if (useWorldCoordinates)
        {
            LocalAnchorA = bodyA.GetLocalPoint(anchorA);
            LocalAnchorB = bodyB.GetLocalPoint(anchorB);

            var dA = anchorA - worldAnchorA;
            LengthA = dA.Length();
            var dB = anchorB - worldAnchorB;
            LengthB = dB.Length();
        }
        else
        {
            LocalAnchorA = anchorA;
            LocalAnchorB = anchorB;

            var dA = anchorA - bodyA.GetLocalPoint(worldAnchorA);
            LengthA = dA.Length();
            var dB = anchorB - bodyB.GetLocalPoint(worldAnchorB);
            LengthB = dB.Length();
        }

        Debug.Assert(ratio != 0.0f);
        Debug.Assert(ratio > MathConstants.Epsilon);

        Ratio = ratio;
        _constant = LengthA + ratio * LengthB;
        _impulse = 0.0f;
    }

    /// <summary>The local anchor point on BodyA</summary>
    public Vector2 LocalAnchorA { get; set; }

    /// <summary>The local anchor point on BodyB</summary>
    public Vector2 LocalAnchorB { get; set; }

    /// <summary>Get the first world anchor.</summary>
    public sealed override Vector2 WorldAnchorA { get; set; }

    /// <summary>Get the second world anchor.</summary>
    public sealed override Vector2 WorldAnchorB { get; set; }

    /// <summary>Get the current length of the segment attached to BodyA.</summary>
    public float LengthA { get; set; }

    /// <summary>Get the current length of the segment attached to BodyB.</summary>
    public float LengthB { get; set; }

    /// <summary>The current length between the anchor point on BodyA and WorldAnchorA</summary>
    public float CurrentLengthA
    {
        get
        {
            var p = _bodyA.GetWorldPoint(LocalAnchorA);
            var s = WorldAnchorA;
            var d = p - s;
            return d.Length();
        }
    }

    /// <summary>The current length between the anchor point on BodyB and WorldAnchorB</summary>
    public float CurrentLengthB
    {
        get
        {
            var p = _bodyB.GetWorldPoint(LocalAnchorB);
            var s = WorldAnchorB;
            var d = p - s;
            return d.Length();
        }
    }

    /// <summary>Get the pulley ratio.</summary>
    public float Ratio { get; set; }

    public override void ShiftOrigin(ref Vector2 newOrigin)
    {
        WorldAnchorA -= newOrigin;
        WorldAnchorB -= newOrigin;
    }

    public override Vector2 GetReactionForce(float invDt)
    {
        var P = _impulse * _uB;
        return invDt * P;
    }

    public override float GetReactionTorque(float invDt)
    {
        return 0.0f;
    }

    internal override void InitVelocityConstraints(ref SolverData data)
    {
        _indexA = BodyA.IslandIndex;
        _indexB = BodyB.IslandIndex;
        _localCenterA = BodyA._sweep.LocalCenter;
        _localCenterB = BodyB._sweep.LocalCenter;
        _invMassA = BodyA._invMass;
        _invMassB = BodyB._invMass;
        _invIA = BodyA._invI;
        _invIB = BodyB._invI;

        var cA = data.Positions[_indexA].C;
        var aA = data.Positions[_indexA].A;
        var vA = data.Velocities[_indexA].V;
        var wA = data.Velocities[_indexA].W;

        var cB = data.Positions[_indexB].C;
        var aB = data.Positions[_indexB].A;
        var vB = data.Velocities[_indexB].V;
        var wB = data.Velocities[_indexB].W;

        Rot qA = new(aA), qB = new(aB);

        _rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
        _rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);

        // Get the pulley axes.
        _uA = cA + _rA - WorldAnchorA;
        _uB = cB + _rB - WorldAnchorB;

        var lengthA = _uA.Length();
        var lengthB = _uB.Length();

        if (lengthA > 10.0f * Settings.LinearSlop)
            _uA *= 1.0f / lengthA;
        else
            _uA = Vector2.Zero;

        if (lengthB > 10.0f * Settings.LinearSlop)
            _uB *= 1.0f / lengthB;
        else
            _uB = Vector2.Zero;

        // Compute effective mass.
        var ruA = MathUtils.Cross(_rA, _uA);
        var ruB = MathUtils.Cross(_rB, _uB);

        var mA = _invMassA + _invIA * ruA * ruA;
        var mB = _invMassB + _invIB * ruB * ruB;

        _mass = mA + Ratio * Ratio * mB;

        if (_mass > 0.0f)
            _mass = 1.0f / _mass;

        if (data.Step.WarmStarting)
        {
            // Scale impulses to support variable time steps.
            _impulse *= data.Step.DeltaTimeRatio;

            // Warm starting.
            var PA = -_impulse * _uA;
            var PB = -Ratio * _impulse * _uB;

            vA += _invMassA * PA;
            wA += _invIA * MathUtils.Cross(_rA, PA);
            vB += _invMassB * PB;
            wB += _invIB * MathUtils.Cross(_rB, PB);
        }
        else
        {
            _impulse = 0.0f;
        }

        data.Velocities[_indexA].V = vA;
        data.Velocities[_indexA].W = wA;
        data.Velocities[_indexB].V = vB;
        data.Velocities[_indexB].W = wB;
    }

    internal override void SolveVelocityConstraints(ref SolverData data)
    {
        var vA = data.Velocities[_indexA].V;
        var wA = data.Velocities[_indexA].W;
        var vB = data.Velocities[_indexB].V;
        var wB = data.Velocities[_indexB].W;

        var vpA = vA + MathUtils.Cross(wA, _rA);
        var vpB = vB + MathUtils.Cross(wB, _rB);

        var Cdot = -Vector2.Dot(_uA, vpA) - Ratio * Vector2.Dot(_uB, vpB);
        var impulse = -_mass * Cdot;
        _impulse += impulse;

        var PA = -impulse * _uA;
        var PB = -Ratio * impulse * _uB;
        vA += _invMassA * PA;
        wA += _invIA * MathUtils.Cross(_rA, PA);
        vB += _invMassB * PB;
        wB += _invIB * MathUtils.Cross(_rB, PB);

        data.Velocities[_indexA].V = vA;
        data.Velocities[_indexA].W = wA;
        data.Velocities[_indexB].V = vB;
        data.Velocities[_indexB].W = wB;
    }

    internal override bool SolvePositionConstraints(ref SolverData data)
    {
        var cA = data.Positions[_indexA].C;
        var aA = data.Positions[_indexA].A;
        var cB = data.Positions[_indexB].C;
        var aB = data.Positions[_indexB].A;

        Rot qA = new(aA), qB = new(aB);

        var rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
        var rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);

        // Get the pulley axes.
        var uA = cA + rA - WorldAnchorA;
        var uB = cB + rB - WorldAnchorB;

        var lengthA = uA.Length();
        var lengthB = uB.Length();

        if (lengthA > 10.0f * Settings.LinearSlop)
            uA *= 1.0f / lengthA;
        else
            uA = Vector2.Zero;

        if (lengthB > 10.0f * Settings.LinearSlop)
            uB *= 1.0f / lengthB;
        else
            uB = Vector2.Zero;

        // Compute effective mass.
        var ruA = MathUtils.Cross(rA, uA);
        var ruB = MathUtils.Cross(rB, uB);

        var mA = _invMassA + _invIA * ruA * ruA;
        var mB = _invMassB + _invIB * ruB * ruB;

        var mass = mA + Ratio * Ratio * mB;

        if (mass > 0.0f)
            mass = 1.0f / mass;

        var C = _constant - lengthA - Ratio * lengthB;
        var linearError = Math.Abs(C);

        var impulse = -mass * C;

        var PA = -impulse * uA;
        var PB = -Ratio * impulse * uB;

        cA += _invMassA * PA;
        aA += _invIA * MathUtils.Cross(rA, PA);
        cB += _invMassB * PB;
        aB += _invIB * MathUtils.Cross(rB, PB);

        data.Positions[_indexA].C = cA;
        data.Positions[_indexA].A = aA;
        data.Positions[_indexB].C = cB;
        data.Positions[_indexB].A = aB;

        return linearError < Settings.LinearSlop;
    }
}