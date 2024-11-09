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
using System.Collections.Generic;
using System.Diagnostics;
using Genbox.VelcroPhysics.Collision.Broadphase;
using Genbox.VelcroPhysics.Collision.ContactSystem;
using Genbox.VelcroPhysics.Collision.Distance;
using Genbox.VelcroPhysics.Collision.RayCast;
using Genbox.VelcroPhysics.Collision.TOI;
using Genbox.VelcroPhysics.Dynamics.Handlers;
using Genbox.VelcroPhysics.Dynamics.Joints;
using Genbox.VelcroPhysics.Dynamics.Solver;
using Genbox.VelcroPhysics.Extensions.Controllers.ControllerBase;
using Genbox.VelcroPhysics.Shared;
using Genbox.VelcroPhysics.Utilities;
using Microsoft.Xna.Framework;

namespace Genbox.VelcroPhysics.Dynamics;

/// <summary>The world class manages all physics entities, dynamic simulation, and asynchronous queries.</summary>
public class World
{
    private readonly HashSet<Body> _bodyAddList = new();
    private readonly HashSet<Body> _bodyRemoveList = new();

    internal ContactManager _contactManager;
    internal Queue<Contact> _contactPool = new(256);
    private Vector2 _gravity;
    private float _invDt0;
    internal Island _island;
    internal bool _isLocked;
    private readonly HashSet<Joint> _jointAddList = new();
    private readonly HashSet<Joint> _jointRemoveList = new();
    private Fixture _myFixture;
    internal bool _newContacts;
    private Vector2 _point1;
    private Vector2 _point2;
    private Profile _profile;
    private Func<Fixture, bool> _queryAABBCallback;
    private readonly Func<int, bool> _queryAABBCallbackWrapper;
    private Func<Fixture, Vector2, Vector2, float, float> _rayCastCallback;
    private readonly Func<RayCastInput, int, float> _rayCastCallbackWrapper;
    private Body[] _stack = new Body[64];
    private bool _stepComplete = true;
    private List<Fixture> _testPointAllFixtures;
    private readonly Pool<Stopwatch> _timerPool = new(Stopwatch.StartNew, sw => sw.Restart(), 5, false);

    /// <summary>Initializes a new instance of the <see cref="World" /> class.</summary>
    protected World(Vector2 gravity)
    {
        _gravity = gravity;
        Enabled = true;
        SleepingAllowed = true;
        WarmStartingEnabled = true;
        ContinuousPhysicsEnabled = true;

        _island = new Island();
        ControllerList = new List<Controller>();
        BreakableBodyList = new List<BreakableBody>();
        BodyList = new List<Body>(32);
        JointList = new List<Joint>(32);

        _queryAABBCallbackWrapper = QueryAABBCallbackWrapper;
        _rayCastCallbackWrapper = RayCastCallbackWrapper;

        _contactManager = new ContactManager(new DynamicTreeBroadPhase());
    }

    public bool ContinuousPhysicsEnabled { get; set; }

    public bool SleepingAllowed { get; set; }

    public bool WarmStartingEnabled { get; set; }

    /// <summary>
    ///     Enabling diagnostics causes the engine to gather timing information. You can see how much time it took to
    ///     solve the contacts, solve CCD and update the controllers.
    /// </summary>
    public bool EnableDiagnostics { get; set; }

    /// <summary>Change the global gravity vector.</summary>
    public Vector2 Gravity
    {
        get => _gravity;
        set => _gravity = value;
    }

    public ref Profile Profile => ref _profile;

    public List<Controller> ControllerList { get; }

    public List<BreakableBody> BreakableBodyList { get; }

    /// <summary>Get the number of broad-phase proxies.</summary>
    /// <value>The proxy count.</value>
    public int ProxyCount => ContactManager.BroadPhase.ProxyCount;

    /// <summary>Get the contact manager for testing.</summary>
    /// <value>The contact manager.</value>
    public ContactManager ContactManager => _contactManager;

    /// <summary>Get the world body list.</summary>
    /// <value>The head of the world body list.</value>
    public List<Body> BodyList { get; }

    /// <summary>Get the world joint list.</summary>
    /// <value>The joint list.</value>
    public List<Joint> JointList { get; }

    /// <summary>If false, the whole simulation stops. It still processes added and removed geometries.</summary>
    public bool Enabled { get; set; }

    public bool IsLocked
    {
        get => _isLocked;
        set => _isLocked = value;
    }

    /// <summary>Fires whenever a body has been added</summary>
    public event BodyHandler BodyAdded;

    /// <summary>Fires whenever a body has been removed</summary>
    public event BodyHandler BodyRemoved;

    /// <summary>Fires every time a controller is added to the World.</summary>
    public event ControllerHandler ControllerAdded;

    /// <summary>Fires every time a controller is removed form the World.</summary>
    public event ControllerHandler ControllerRemoved;

    /// <summary>Fires whenever a fixture has been added</summary>
    public event FixtureHandler FixtureAdded;

    /// <summary>Fires whenever a fixture has been removed</summary>
    public event FixtureHandler FixtureRemoved;

    /// <summary>Fires whenever a joint has been added</summary>
    public event JointHandler JointAdded;

    /// <summary>Fires whenever a joint has been removed</summary>
    public event JointHandler JointRemoved;

    /// <summary>Add a rigid body.</summary>
    /// <param name="body">The body.</param>
    /// <param name="delayUntilNextStep">If true, the body is added at next time step</param>
    public void AddBody(Body body, bool delayUntilNextStep = false)
    {
        if (delayUntilNextStep)
        {
            Debug.Assert(!_bodyAddList.Contains(body), "You are adding the same body more than once.");

            _bodyAddList.Add(body);
        }
        else
        {
            Debug.Assert(!_isLocked);

            if (_isLocked)
                return;

            AddBodyInternal(body);
        }
    }

    /// <summary>Destroy a rigid body. Warning: This automatically deletes all associated shapes and joints.</summary>
    /// <param name="body">The body.</param>
    /// <param name="delayUntilNextStep">If true, the body is removed at next time step</param>
    public void RemoveBody(Body body, bool delayUntilNextStep = false)
    {
        if (delayUntilNextStep)
        {
            Debug.Assert(!_bodyRemoveList.Contains(body),
                "The body is already marked for removal. You are removing the body more than once.");

            _bodyRemoveList.Add(body);
        }
        else
        {
            Debug.Assert(!_isLocked);

            if (_isLocked)
                return;

            RemoveBodyInternal(body);
        }
    }

    /// <summary>Create a joint to constrain bodies together. This may cause the connected bodies to cease colliding.</summary>
    /// <param name="joint">The joint.</param>
    /// <param name="delayUntilNextStep">If true, the joint is added at next time step</param>
    public void AddJoint(Joint joint, bool delayUntilNextStep = false)
    {
        if (delayUntilNextStep)
        {
            Debug.Assert(!_jointAddList.Contains(joint), "You are adding the same joint more than once.");

            _jointAddList.Add(joint);
        }
        else
        {
            Debug.Assert(!_isLocked);

            if (_isLocked)
                return;

            AddJointInternal(joint);
        }
    }

    /// <summary>Destroy a joint. This may cause the connected bodies to begin colliding.</summary>
    /// <param name="joint">The joint.</param>
    /// <param name="delayUntilNextStep">If true, the joint is removed at next time step</param>
    public void RemoveJoint(Joint joint, bool delayUntilNextStep = false)
    {
        if (delayUntilNextStep)
        {
            Debug.Assert(!_jointRemoveList.Contains(joint),
                "The joint is already marked for removal. You are removing the joint more than once.");

            _jointRemoveList.Add(joint);
        }
        else
        {
            Debug.Assert(!_isLocked);

            if (_isLocked)
                return;

            RemoveJointInternal(joint);
        }
    }

    public void AddController(Controller controller)
    {
        Debug.Assert(!ControllerList.Contains(controller), "You are adding the same controller more than once.");

        controller.World = this;
        ControllerList.Add(controller);

        ControllerAdded?.Invoke(controller);
    }

    public void RemoveController(Controller controller)
    {
        Debug.Assert(ControllerList.Contains(controller),
            "You are removing a controller that is not in the simulation.");

        if (ControllerList.Contains(controller))
        {
            ControllerList.Remove(controller);

            ControllerRemoved?.Invoke(controller);
        }
    }

    public void AddBreakableBody(BreakableBody breakableBody)
    {
        BreakableBodyList.Add(breakableBody);
    }

    public void RemoveBreakableBody(BreakableBody breakableBody)
    {
        //The breakable body list does not contain the body you tried to remove.
        Debug.Assert(BreakableBodyList.Contains(breakableBody));

        BreakableBodyList.Remove(breakableBody);
    }

    /// <summary>Take a time step. This performs collision detection, integration, and constraint solution.</summary>
    /// <param name="dt">The amount of time to simulate, this should not vary.</param>
    /// <param name="velocityIterations">
    ///     The number of velocity iterations to do in this step. Lesser means more performance
    ///     but more inaccurate velocity calculations.
    /// </param>
    /// <param name="positionIterations">
    ///     The number of position iterations to do in this step. Lesser means more performance,
    ///     but also more inaccurate position calculations.
    /// </param>
    public void Step(float dt, int velocityIterations = 8, int positionIterations = 3)
    {
        //Velcro: We support disabling the world
        if (!Enabled)
            return;

        //Velcro: We reuse the timers to avoid generating garbage
        var stepTimer = _timerPool.GetFromPool(true);

        {
            //Velcro: We support add/removal of objects live in the engine.
            var timer = _timerPool.GetFromPool(true);
            ProcessChanges();
            _profile.AddRemoveTime = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }

        // If new fixtures were added, we need to find the new contacts.
        if (_newContacts)
        {
            //Velcro: We measure how much time is spent on finding new contacts
            var timer = _timerPool.GetFromPool(true);
            _contactManager.FindNewContacts();
            _newContacts = false;
            _profile.NewContactsTime = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }

        _isLocked = true;

        TimeStep step;
        step.DeltaTime = dt;
        step.VelocityIterations = velocityIterations;
        step.PositionIterations = positionIterations;
        step.WarmStarting = WarmStartingEnabled;
        if (dt > 0.0f)
            step.InvertedDeltaTime = 1.0f / dt;
        else
            step.InvertedDeltaTime = 0.0f;

        step.DeltaTimeRatio = _invDt0 * dt;

        {
            //Velcro: We have the concept of controllers. We update them here
            var timer = _timerPool.GetFromPool(true);
            for (var i = 0; i < ControllerList.Count; i++) ControllerList[i].Update(dt);
            _profile.ControllersUpdateTime = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }

        // Update contacts. This is where some contacts are destroyed.
        {
            var timer = _timerPool.GetFromPool(true);
            _contactManager.Collide();
            _profile.Collide = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (_stepComplete && step.DeltaTime > 0.0f)
        {
            var timer = _timerPool.GetFromPool(true);
            Solve(ref step);
            _profile.Solve = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }

        // Handle TOI events.
        if (ContinuousPhysicsEnabled && step.DeltaTime > 0.0f)
        {
            var timer = _timerPool.GetFromPool(true);
            SolveTOI(ref step);
            _profile.SolveTOI = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }

        if (step.DeltaTime > 0.0f)
            _invDt0 = step.InvertedDeltaTime;

        if (Settings.AutoClearForces)
            ClearForces();

        {
            //Velcro: We support breakable bodies. We update them here.
            var timer = _timerPool.GetFromPool(true);

            for (var i = 0; i < BreakableBodyList.Count; i++) BreakableBodyList[i].Update();
            _profile.BreakableBodies = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }

        _isLocked = false;

        _profile.Step = stepTimer.ElapsedTicks;
        _timerPool.ReturnToPool(stepTimer);
    }

    /// <summary>
    ///     Call this after you are done with time steps to clear the forces. You normally call this after each call to
    ///     Step, unless you are performing sub-steps. By default, forces will be automatically cleared, so you don't need to
    ///     call
    ///     this function.
    /// </summary>
    public void ClearForces()
    {
        for (var i = 0; i < BodyList.Count; i++)
        {
            var body = BodyList[i];
            body._force = Vector2.Zero;
            body._torque = 0.0f;
        }
    }

    /// <summary>
    ///     Query the world for all fixtures that potentially overlap the provided AABB. Inside the callback: Return true:
    ///     Continues the query Return false: Terminate the query
    /// </summary>
    /// <param name="callback">A user implemented callback class.</param>
    /// <param name="aabb">The AABB query box.</param>
    public void QueryAABB(Func<Fixture, bool> callback, ref AABB aabb)
    {
        _queryAABBCallback = callback;
        _contactManager.BroadPhase.Query(_queryAABBCallbackWrapper, ref aabb);
        _queryAABBCallback = null;
    }

    /// <summary>
    ///     Query the world for all fixtures that potentially overlap the provided AABB. Use the overload with a callback
    ///     for filtering and better performance.
    /// </summary>
    /// <param name="aabb">The AABB query box.</param>
    /// <returns>A list of fixtures that were in the affected area.</returns>
    public List<Fixture> QueryAABB(ref AABB aabb)
    {
        var affected = new List<Fixture>();

        QueryAABB(fixture =>
        {
            affected.Add(fixture);
            return true;
        }, ref aabb);

        return affected;
    }

    /// <summary>
    ///     Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you get the closest
    ///     point, any point, or n-points. The ray-cast ignores shapes that contain the starting point. Inside the callback:
    ///     return
    ///     -1: ignore this fixture and continue return 0: terminate the ray cast return fraction: clip the ray to this point
    ///     return 1: don't clip the ray and continue
    /// </summary>
    /// <param name="callback">A user implemented callback class.</param>
    /// <param name="point1">The ray starting point.</param>
    /// <param name="point2">The ray ending point.</param>
    public void RayCast(Func<Fixture, Vector2, Vector2, float, float> callback, Vector2 point1, Vector2 point2)
    {
        var input = new RayCastInput();
        input.MaxFraction = 1.0f;
        input.Point1 = point1;
        input.Point2 = point2;

        _rayCastCallback = callback;
        _contactManager.BroadPhase.RayCast(_rayCastCallbackWrapper, ref input);
        _rayCastCallback = null;
    }

    public List<Fixture> RayCast(Vector2 point1, Vector2 point2)
    {
        var affected = new List<Fixture>();

        float RayCastCallback(Fixture f, Vector2 vector2, Vector2 vector3, float f1)
        {
            affected.Add(f);
            return 1;
        }

        RayCast(RayCastCallback, point1, point2);

        return affected;
    }

    public Fixture TestPoint(Vector2 point)
    {
        AABB aabb;
        var d = new Vector2(MathConstants.Epsilon, MathConstants.Epsilon);
        aabb.LowerBound = point - d;
        aabb.UpperBound = point + d;

        _myFixture = null;
        _point1 = point;

        // Query the world for overlapping shapes.
        QueryAABB(TestPointCallback, ref aabb);

        return _myFixture;
    }

    /// <summary>Returns a list of fixtures that are at the specified point.</summary>
    /// <param name="point">The point.</param>
    public List<Fixture> TestPointAll(Vector2 point)
    {
        AABB aabb;
        var d = new Vector2(MathConstants.Epsilon, MathConstants.Epsilon);
        aabb.LowerBound = point - d;
        aabb.UpperBound = point + d;

        _point2 = point;
        _testPointAllFixtures = new List<Fixture>();

        // Query the world for overlapping shapes.
        QueryAABB(TestPointAllCallback, ref aabb);

        return _testPointAllFixtures;
    }

    /// <summary>
    ///     Shift the world origin. Useful for large worlds. The body shift formula is: position -= newOrigin @param
    ///     newOrigin the new origin with respect to the old origin Warning: Calling this method mid-update might cause a
    ///     crash.
    /// </summary>
    public void ShiftOrigin(Vector2 newOrigin)
    {
        Debug.Assert(!_isLocked);
        if (_isLocked)
            return;

        foreach (var b in BodyList)
        {
            b._xf.p -= newOrigin;
            b._sweep.C0 -= newOrigin;
            b._sweep.C -= newOrigin;
        }

        foreach (var joint in JointList) joint.ShiftOrigin(ref newOrigin);

        _contactManager.BroadPhase.ShiftOrigin(ref newOrigin);
    }

    public void Clear()
    {
        ProcessChanges();

        for (var i = BodyList.Count - 1; i >= 0; i--) RemoveBody(BodyList[i]);

        for (var i = ControllerList.Count - 1; i >= 0; i--) RemoveController(ControllerList[i]);

        for (var i = BreakableBodyList.Count - 1; i >= 0; i--) RemoveBreakableBody(BreakableBodyList[i]);

        //We call ProcessChanges again since the user could have added items to the body/joint queues in the Removed/Added events
        ProcessChanges();
    }

    internal void RaiseNewFixtureEvent(Fixture fixture)
    {
        FixtureAdded?.Invoke(fixture);
    }

    private void ProcessRemovedJoints()
    {
        if (_jointRemoveList.Count == 0)
            return;

        foreach (var joint in _jointRemoveList) RemoveJointInternal(joint);

        _jointRemoveList.Clear();
    }

    private void ProcessAddedJoints()
    {
        if (_jointAddList.Count == 0)
            return;

        foreach (var joint in _jointAddList) AddJointInternal(joint);

        _jointAddList.Clear();
    }

    private void ProcessAddedBodies()
    {
        if (_bodyAddList.Count == 0)
            return;

        foreach (var body in _bodyAddList) AddBodyInternal(body);

        _bodyAddList.Clear();
    }

    private void ProcessRemovedBodies()
    {
        if (_bodyRemoveList.Count == 0)
            return;

        foreach (var body in _bodyRemoveList) RemoveBodyInternal(body);

        _bodyRemoveList.Clear();
    }

    private bool QueryAABBCallbackWrapper(int proxyId)
    {
        var proxy = _contactManager.BroadPhase.GetProxy(proxyId);
        return _queryAABBCallback(proxy.Fixture);
    }

    private float RayCastCallbackWrapper(RayCastInput rayCastInput, int proxyId)
    {
        var proxy = _contactManager.BroadPhase.GetProxy(proxyId);
        var fixture = proxy.Fixture;
        var index = proxy.ChildIndex;
        var hit = fixture.RayCast(out var output, ref rayCastInput, index);

        if (hit)
        {
            var fraction = output.Fraction;
            var point = (1.0f - fraction) * rayCastInput.Point1 + fraction * rayCastInput.Point2;
            return _rayCastCallback(fixture, point, output.Normal, fraction);
        }

        return rayCastInput.MaxFraction;
    }

    private void Solve(ref TimeStep step)
    {
        _profile.SolveInit = 0;
        _profile.SolveVelocity = 0;
        _profile.SolvePosition = 0;

        // Size the island for the worst case.
        _island.Reset(BodyList.Count,
            _contactManager._contactCount,
            JointList.Count,
            _contactManager);

        // Clear all the island flags.
        foreach (var b in BodyList) b._flags &= ~BodyFlags.IslandFlag;

        for (var c = _contactManager._contactList; c != null; c = c._next) c._flags &= ~ContactFlags.IslandFlag;

        foreach (var j in JointList) j._islandFlag = false;

        // Build and simulate all awake islands.
        var stackSize = BodyList.Count;
        if (stackSize > _stack.Length)
            _stack = new Body[Math.Max(_stack.Length * 2, stackSize)];

        for (var index = BodyList.Count - 1; index >= 0; index--)
        {
            var seed = BodyList[index];
            if ((seed._flags & BodyFlags.IslandFlag) == BodyFlags.IslandFlag)
                continue;

            if (!seed.Awake || !seed.Enabled)
                continue;

            // The seed can be dynamic or kinematic.
            if (seed.BodyType == BodyType.Static)
                continue;

            // Reset island and stack.
            _island.Clear();
            var stackCount = 0;
            _stack[stackCount++] = seed;

            seed._flags |= BodyFlags.IslandFlag;

            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0)
            {
                // Grab the next body off the stack and add it to the island.
                var b = _stack[--stackCount];
                Debug.Assert(b.Enabled);
                _island.Add(b);

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.BodyType == BodyType.Static)
                    continue;

                // Make sure the body is awake (without resetting sleep timer).
                b._flags |= BodyFlags.AwakeFlag;

                // Search all contacts connected to this body.
                for (var ce = b._contactList; ce != null; ce = ce.Next)
                {
                    var contact = ce.Contact;

                    // Has this contact already been added to an island?
                    if (contact.IslandFlag)
                        continue;

                    // Is this contact solid and touching?
                    if (!contact.Enabled || !contact.IsTouching)
                        continue;

                    // Skip sensors.
                    var sensorA = contact._fixtureA.IsSensor;
                    var sensorB = contact._fixtureB.IsSensor;
                    if (sensorA || sensorB)
                        continue;

                    _island.Add(contact);
                    contact._flags |= ContactFlags.IslandFlag;

                    var other = ce.Other;

                    // Was the other body already added to this island?
                    if (other.IsIsland)
                        continue;

                    Debug.Assert(stackCount < stackSize);
                    _stack[stackCount++] = other;
                    other._flags |= BodyFlags.IslandFlag;
                }

                // Search all joints connect to this body.
                for (var je = b._jointList; je != null; je = je.Next)
                {
                    if (je.Joint._islandFlag)
                        continue;

                    var other = je.Other;

                    // WIP David
                    //Enter here when it's a non-fixed joint. Non-fixed joints have a other body.
                    if (other != null)
                    {
                        // Don't simulate joints connected to inactive bodies.
                        if (!other.Enabled)
                            continue;

                        _island.Add(je.Joint);
                        je.Joint._islandFlag = true;

                        if (other.IsIsland)
                            continue;

                        Debug.Assert(stackCount < stackSize);
                        _stack[stackCount++] = other;

                        other._flags |= BodyFlags.IslandFlag;
                    }
                    else
                    {
                        _island.Add(je.Joint);
                        je.Joint._islandFlag = true;
                    }
                }
            }

            var profile = new Profile();
            _island.Solve(ref profile, ref step, ref _gravity, SleepingAllowed);
            _profile.SolveInit += profile.SolveInit;
            _profile.SolveVelocity += profile.SolveVelocity;
            _profile.SolvePosition += profile.SolvePosition;

            // Post solve cleanup.
            for (var i = 0; i < _island._bodyCount; ++i)
            {
                // Allow static bodies to participate in other islands.
                var b = _island._bodies[i];
                if (b.BodyType == BodyType.Static)
                    b._flags &= ~BodyFlags.IslandFlag;
            }
        }

        {
            var timer = _timerPool.GetFromPool(true);

            // Synchronize fixtures, check for out of range bodies.
            foreach (var b in BodyList)
            {
                // If a body was not in an island then it did not move.
                if ((b._flags & BodyFlags.IslandFlag) == 0)
                    continue;

                if (b.BodyType == BodyType.Static)
                    continue;

                // Update fixtures (for broad-phase).
                b.SynchronizeFixtures();
            }

            // Look for new contacts.
            _contactManager.FindNewContacts();
            _profile.Broadphase = timer.ElapsedTicks;
            _timerPool.ReturnToPool(timer);
        }
    }

    private void SolveTOI(ref TimeStep step)
    {
        _island.Reset(2 * Settings.MaxTOIContacts, Settings.MaxTOIContacts, 0, _contactManager);

        if (_stepComplete)
        {
            for (var i = 0; i < BodyList.Count; i++)
            {
                BodyList[i]._flags &= ~BodyFlags.IslandFlag;
                BodyList[i]._sweep.Alpha0 = 0.0f;
            }

            for (var c = _contactManager._contactList; c != null; c = c._next)
            {
                // Invalidate TOI
                c._flags &= ~(ContactFlags.TOIFlag | ContactFlags.IslandFlag);
                c._toiCount = 0;
                c._toi = 1.0f;
            }
        }

        // Find TOI events and solve them.
        for (;;)
        {
            // Find the first TOI.
            Contact minContact = null;
            var minAlpha = 1.0f;

            for (var c = _contactManager._contactList; c != null; c = c._next)
            {
                // Is this contact disabled?
                if (!c.Enabled)
                    continue;

                // Prevent excessive sub-stepping.
                if (c._toiCount > Settings.MaxSubSteps)
                    continue;

                float alpha;
                if (c.TOIFlag)
                {
                    // This contact has a valid cached TOI.
                    alpha = c._toi;
                }
                else
                {
                    var fA = c._fixtureA;
                    var fB = c._fixtureB;

                    // Is there a sensor?
                    if (fA._isSensor || fB._isSensor)
                        continue;

                    var bA = fA.Body;
                    var bB = fB.Body;

                    var typeA = bA.BodyType;
                    var typeB = bB.BodyType;
                    Debug.Assert(typeA == BodyType.Dynamic || typeB == BodyType.Dynamic);

                    var activeA = bA.Awake && typeA != BodyType.Static;
                    var activeB = bB.Awake && typeB != BodyType.Static;

                    // Is at least one body active (awake and dynamic or kinematic)?
                    if (!activeA && !activeB)
                        continue;

                    var collideA = (bA.IsBullet || typeA != BodyType.Dynamic) &&
                                   (fA.IgnoreCcdWith & fB.CollisionCategories) == 0 && !bA.IgnoreCCD;
                    var collideB = (bB.IsBullet || typeB != BodyType.Dynamic) &&
                                   (fB.IgnoreCcdWith & fA.CollisionCategories) == 0 && !bB.IgnoreCCD;

                    // Are these two non-bullet dynamic bodies?
                    if (!collideA && !collideB)
                        continue;

                    // Compute the TOI for this contact.
                    // Put the sweeps onto the same time interval.
                    var alpha0 = bA._sweep.Alpha0;

                    if (bA._sweep.Alpha0 < bB._sweep.Alpha0)
                    {
                        alpha0 = bB._sweep.Alpha0;
                        bA._sweep.Advance(alpha0);
                    }
                    else if (bB._sweep.Alpha0 < bA._sweep.Alpha0)
                    {
                        alpha0 = bA._sweep.Alpha0;
                        bB._sweep.Advance(alpha0);
                    }

                    Debug.Assert(alpha0 < 1.0f);

                    // Compute the time of impact in interval [0, minTOI]
                    var input = new TOIInput();
                    input.ProxyA = new DistanceProxy(fA.Shape, c.ChildIndexA);
                    input.ProxyB = new DistanceProxy(fB.Shape, c.ChildIndexB);
                    input.SweepA = bA._sweep;
                    input.SweepB = bB._sweep;
                    input.TMax = 1.0f;

                    TimeOfImpact.CalculateTimeOfImpact(ref input, out var output);

                    // Beta is the fraction of the remaining portion of the .
                    var beta = output.T;
                    if (output.State == TOIOutputState.Touching)
                        alpha = Math.Min(alpha0 + (1.0f - alpha0) * beta, 1.0f);
                    else
                        alpha = 1.0f;

                    c._toi = alpha;
                    c._flags &= ~ContactFlags.TOIFlag;
                }

                if (alpha < minAlpha)
                {
                    // This is the minimum TOI found so far.
                    minContact = c;
                    minAlpha = alpha;
                }
            }

            if (minContact == null || 1.0f - 10.0f * MathConstants.Epsilon < minAlpha)
            {
                // No more TOI events. Done!
                _stepComplete = true;
                break;
            }

            // Advance the bodies to the TOI.
            var fA1 = minContact._fixtureA;
            var fB1 = minContact._fixtureB;
            var bA0 = fA1.Body;
            var bB0 = fB1.Body;

            var backup1 = bA0._sweep;
            var backup2 = bB0._sweep;

            bA0.Advance(minAlpha);
            bB0.Advance(minAlpha);

            // The TOI contact likely has some new contact points.
            minContact.Update(_contactManager);
            minContact._flags &= ~ContactFlags.TOIFlag;
            ++minContact._toiCount;

            // Is the contact solid?
            if (!minContact.Enabled || !minContact.IsTouching)
            {
                // Restore the sweeps.
                minContact._flags &= ~ContactFlags.EnabledFlag;
                bA0._sweep = backup1;
                bB0._sweep = backup2;
                bA0.SynchronizeTransform();
                bB0.SynchronizeTransform();
                continue;
            }

            bA0.Awake = true;
            bB0.Awake = true;

            // Build the island
            _island.Clear();
            _island.Add(bA0);
            _island.Add(bB0);
            _island.Add(minContact);

            bA0._flags |= BodyFlags.IslandFlag;
            bB0._flags |= BodyFlags.IslandFlag;
            minContact._flags &= ~ContactFlags.IslandFlag;

            // Get contacts on bodyA and bodyB.
            Body[] bodies = { bA0, bB0 };
            for (var i = 0; i < 2; ++i)
            {
                var body = bodies[i];
                if (body.BodyType == BodyType.Dynamic)
                    for (var ce = body._contactList; ce != null; ce = ce.Next)
                    {
                        var contact = ce.Contact;

                        if (_island._bodyCount == _island._bodyCapacity)
                            break;

                        if (_island._contactCount == _island._contactCapacity)
                            break;

                        // Has this contact already been added to the island?
                        if (contact.IslandFlag)
                            continue;

                        // Only add static, kinematic, or bullet bodies.
                        var other = ce.Other;
                        if (other.BodyType == BodyType.Dynamic &&
                            !body.IsBullet && !other.IsBullet)
                            continue;

                        // Skip sensors.
                        var sensorA = contact._fixtureA._isSensor;
                        var sensorB = contact._fixtureB._isSensor;
                        if (sensorA || sensorB)
                            continue;

                        // Tentatively advance the body to the TOI.
                        var backup = other._sweep;
                        if (!other.IsIsland)
                            other.Advance(minAlpha);

                        // Update the contact points
                        contact.Update(_contactManager);

                        // Was the contact disabled by the user?
                        if (!contact.Enabled)
                        {
                            other._sweep = backup;
                            other.SynchronizeTransform();
                            continue;
                        }

                        // Are there contact points?
                        if (!contact.IsTouching)
                        {
                            other._sweep = backup;
                            other.SynchronizeTransform();
                            continue;
                        }

                        // Add the contact to the island
                        minContact._flags |= ContactFlags.IslandFlag;
                        _island.Add(contact);

                        // Has the other body already been added to the island?
                        if (other.IsIsland)
                            continue;

                        // Add the other body to the island.
                        other._flags |= BodyFlags.IslandFlag;

                        if (other.BodyType != BodyType.Static)
                            other.Awake = true;

                        _island.Add(other);
                    }
            }

            TimeStep subStep;
            subStep.DeltaTime = (1.0f - minAlpha) * step.DeltaTime;
            subStep.InvertedDeltaTime = 1.0f / subStep.DeltaTime;
            subStep.DeltaTimeRatio = 1.0f;
            subStep.PositionIterations = 20;
            subStep.VelocityIterations = step.VelocityIterations;
            subStep.WarmStarting = false;
            _island.SolveTOI(ref subStep, bA0.IslandIndex, bB0.IslandIndex);

            // Reset island flags and synchronize broad-phase proxies.
            for (var i = 0; i < _island._bodyCount; ++i)
            {
                var body = _island._bodies[i];
                body._flags &= ~BodyFlags.IslandFlag;

                if (body.BodyType != BodyType.Dynamic)
                    continue;

                body.SynchronizeFixtures();

                // Invalidate all contact TOIs on this displaced body.
                for (var ce = body._contactList; ce != null; ce = ce.Next)
                    ce.Contact._flags &= ~(ContactFlags.TOIFlag | ContactFlags.IslandFlag);
            }

            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            _contactManager.FindNewContacts();

            if (Settings.EnableSubStepping)
            {
                _stepComplete = false;
                break;
            }
        }
    }

    private bool TestPointCallback(Fixture fixture)
    {
        var inside = fixture.TestPoint(ref _point1);
        if (inside)
        {
            _myFixture = fixture;
            return false;
        }

        // Continue the query.
        return true;
    }

    private bool TestPointAllCallback(Fixture fixture)
    {
        var inside = fixture.TestPoint(ref _point2);
        if (inside)
            _testPointAllFixtures.Add(fixture);

        // Continue the query.
        return true;
    }

    private void AddJointInternal(Joint joint)
    {
        // Connect to the world list.
        JointList.Add(joint);

        // Connect to the bodies' doubly linked lists.
        joint._edgeA.Joint = joint;
        joint._edgeA.Other = joint.BodyB;
        joint._edgeA.Prev = null;
        joint._edgeA.Next = joint.BodyA._jointList;

        if (joint.BodyA._jointList != null)
            joint.BodyA._jointList.Prev = joint._edgeA;

        joint.BodyA._jointList = joint._edgeA;

        // WIP David
        if (!joint.IsFixedType())
        {
            joint._edgeB.Joint = joint;
            joint._edgeB.Other = joint.BodyA;
            joint._edgeB.Prev = null;
            joint._edgeB.Next = joint.BodyB._jointList;

            if (joint.BodyB._jointList != null)
                joint.BodyB._jointList.Prev = joint._edgeB;

            joint.BodyB._jointList = joint._edgeB;

            var bodyA = joint.BodyA;
            var bodyB = joint.BodyB;

            // If the joint prevents collisions, then flag any contacts for filtering.
            if (!joint.CollideConnected)
            {
                var edge = bodyB._contactList;
                while (edge != null)
                {
                    if (edge.Other == bodyA)
                        // Flag the contact for filtering at the next time step (where either
                        // body is awake).
                        edge.Contact._flags |= ContactFlags.FilterFlag;

                    edge = edge.Next;
                }
            }
        }

        JointAdded?.Invoke(joint);

        // Note: creating a joint doesn't wake the bodies.
    }

    private void RemoveJointInternal(Joint joint)
    {
        var collideConnected = joint.CollideConnected;

        // Remove from the world list.
        JointList.Remove(joint);

        // Disconnect from island graph.
        var bodyA = joint.BodyA;
        var bodyB = joint.BodyB;

        // Wake up connected bodies.
        bodyA.Awake = true;

        // WIP David
        if (!joint.IsFixedType())
            bodyB.Awake = true;

        // Remove from body 1.
        if (joint._edgeA.Prev != null)
            joint._edgeA.Prev.Next = joint._edgeA.Next;

        if (joint._edgeA.Next != null)
            joint._edgeA.Next.Prev = joint._edgeA.Prev;

        if (joint._edgeA == bodyA._jointList)
            bodyA._jointList = joint._edgeA.Next;

        joint._edgeA.Prev = null;
        joint._edgeA.Next = null;

        // WIP David
        if (!joint.IsFixedType())
        {
            // Remove from body 2
            if (joint._edgeB.Prev != null)
                joint._edgeB.Prev.Next = joint._edgeB.Next;

            if (joint._edgeB.Next != null)
                joint._edgeB.Next.Prev = joint._edgeB.Prev;

            if (joint._edgeB == bodyB._jointList)
                bodyB._jointList = joint._edgeB.Next;

            joint._edgeB.Prev = null;
            joint._edgeB.Next = null;

            // If the joint prevents collisions, then flag any contacts for filtering.
            if (!collideConnected)
            {
                var edge = bodyB._contactList;
                while (edge != null)
                {
                    if (edge.Other == bodyA)
                        // Flag the contact for filtering at the next time step (where either
                        // body is awake).
                        edge.Contact._flags |= ContactFlags.FilterFlag;

                    edge = edge.Next;
                }
            }
        }

        JointRemoved?.Invoke(joint);
    }

    private void AddBodyInternal(Body body)
    {
        body._world = this;

        // Add to world list.
        BodyList.Add(body);

        //Velcro: We have events to notify the user that a body was added
        BodyAdded?.Invoke(body);

        //Velcro: We have events to notify fixtures was added
        if (FixtureAdded != null)
            for (var i = 0; i < body._fixtureList.Count; i++)
                FixtureAdded(body._fixtureList[i]);
    }

    private void RemoveBodyInternal(Body body)
    {
        Debug.Assert(BodyList.Count > 0);

        //Velcro: We check if the user is trying to remove a body more than one (to check for bugs)
        Debug.Assert(BodyList.Contains(body));

        // Delete the attached joints.
        var je = body._jointList;
        while (je != null)
        {
            var je0 = je;
            je = je.Next;

            RemoveJointInternal(je0.Joint);
        }

        body._jointList = null;

        // Delete the attached contacts.
        var ce = body._contactList;
        while (ce != null)
        {
            var ce0 = ce;
            ce = ce.Next;
            _contactManager.Remove(ce0.Contact);
        }

        body._contactList = null;

        // Delete the attached fixtures. This destroys broad-phase proxies.
        for (var i = 0; i < body._fixtureList.Count; i++)
        {
            var fixture = body._fixtureList[i];

            //Velcro: Added event
            FixtureRemoved?.Invoke(fixture);

            fixture.DestroyProxies(_contactManager.BroadPhase);
            fixture.Destroy();
        }

        body._fixtureList = null;

        //Velcro: We make sure to cleanup the references and delegates
        body._world = null;
        body.OnCollision = null;
        body.OnSeparation = null;

        // Remove world body list.
        BodyList.Remove(body);

        BodyRemoved?.Invoke(body);
    }

    private void ProcessChanges()
    {
        ProcessAddedBodies();
        ProcessAddedJoints();

        ProcessRemovedBodies();
        ProcessRemovedJoints();
    }
}