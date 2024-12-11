using Genbox.VelcroPhysics.Dynamics.Handlers;
using Genbox.VelcroPhysics.Dynamics.Joints;
using Genbox.VelcroPhysics.Extensions.Controllers.ControllerBase;

namespace Genbox.VelcroPhysics.Dynamics.Events;

public interface IWorldVelcroEvents
{
    event BodyHandler? OnBodyAdded;
    
    event BodyHandler? OnBodyRemoved;
    
    event ControllerHandler? OnControllerAdded;
    
    event ControllerHandler? OnControllerRemoved;
    
    event FixtureHandler? OnFixtureAdded;
    
    event FixtureHandler? OnFixtureRemoved;
    
    event JointHandler? OnJointAdded;
    
    event JointHandler? OnJointRemoved;
    
    void BodyAdded(Body body);
    
    void BodyRemoved(Body body);
    
    void ControllerAdded(Controller controller);
    
    void ControllerRemoved(Controller controller);
    
    void FixtureAdded(Fixture fixture);
    
    void FixtureRemoved(Fixture fixture);
    
    void JointAdded(Joint joint);
    
    void JointRemoved(Joint joint);
}