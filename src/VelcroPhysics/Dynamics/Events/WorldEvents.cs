using Genbox.VelcroPhysics.Dynamics.Handlers;
using Genbox.VelcroPhysics.Dynamics.Joints;
using Genbox.VelcroPhysics.Extensions.Controllers.ControllerBase;

namespace Genbox.VelcroPhysics.Dynamics.Events;

public class WorldVelcroEvents : IWorldVelcroEvents
{
    /// <summary>Fires whenever a body has been added</summary>
    public event BodyHandler? OnBodyAdded;

    /// <summary>Fires whenever a body has been removed</summary>
    public event BodyHandler? OnBodyRemoved;

    /// <summary>Fires every time a controller is added to the World.</summary>
    public event ControllerHandler? OnControllerAdded;

    /// <summary>Fires every time a controller is removed form the World.</summary>
    public event ControllerHandler? OnControllerRemoved;

    /// <summary>Fires whenever a fixture has been added</summary>
    public event FixtureHandler? OnFixtureAdded;

    /// <summary>Fires whenever a fixture has been removed</summary>
    public event FixtureHandler? OnFixtureRemoved;

    /// <summary>Fires whenever a joint has been added</summary>
    public event JointHandler? OnJointAdded;

    /// <summary>Fires whenever a joint has been removed</summary>
    public event JointHandler? OnJointRemoved;

    void IWorldVelcroEvents.BodyAdded(Body body)
    {
        OnBodyAdded?.Invoke(body);
    }

    void IWorldVelcroEvents.BodyRemoved(Body body)
    {
        OnBodyRemoved?.Invoke(body);
    }

    void IWorldVelcroEvents.ControllerAdded(Controller controller)
    {
        OnControllerAdded?.Invoke(controller);
    }

    void IWorldVelcroEvents.ControllerRemoved(Controller controller)
    {
        OnControllerRemoved?.Invoke(controller);
    }

    void IWorldVelcroEvents.FixtureAdded(Fixture fixture)
    {
        OnFixtureAdded?.Invoke(fixture);
    }

    void IWorldVelcroEvents.FixtureRemoved(Fixture fixture)
    {
        OnFixtureRemoved?.Invoke(fixture);
    }

    void IWorldVelcroEvents.JointAdded(Joint joint)
    {
        OnJointAdded?.Invoke(joint);
    }

    void IWorldVelcroEvents.JointRemoved(Joint joint)
    {
        OnJointRemoved?.Invoke(joint);
    }
}