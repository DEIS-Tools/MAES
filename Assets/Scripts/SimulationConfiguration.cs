namespace Dora
{
    // This class contains all settings related to an instance of an simulation
    public class SimulationConfiguration
    {
        
        // Times per second that robot logic is updated
        public readonly int LogicTickDeltaMillis = 100;
        
        // Amount of physics steps to calculate between each robot logic tick
        // Physics tick rate = LogicTickDelta / PhysicsTicksPerLogicUpdate
        public readonly int PhysicsTicksPerLogicUpdate = 10;
        
        public readonly float PhysicsTickDeltaSeconds;
        public readonly int PhysicsTickDeltaMillis;

        public SimulationConfiguration()
        {
            PhysicsTickDeltaMillis = LogicTickDeltaMillis / PhysicsTicksPerLogicUpdate;
            PhysicsTickDeltaSeconds = PhysicsTickDeltaMillis / 1000.0f;
        }
    }
}