namespace Dora
{
    // This class contains all settings related to an instance of an simulation
    public class SimulationConfiguration
    {
        
        // Times per second that robot logic is updated
        public int LogicTickDeltaMillis { set; get; } = 100;
        
        // Amount of physics steps to calculate between each robot logic tick
        // Physics tick rate = LogicTickDelta / PhysicsTicksPerLogicUpdate
        public int PhysicsTicksPerLogicUpdate { set; get; } = 10;
    }
}