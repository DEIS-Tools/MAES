namespace Dora
{
    // This class contains all settings related to an instance of an simulation
    public class SimulationConfiguration
    {
        public int LogicTickDeltaMillis { set; get; } = 100;
        public int PhysicsTickDeltaMillis { set; get; } = 5;


    }
}