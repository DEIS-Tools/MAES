using System;

namespace Dora
{
    public interface ISimulationUnit : ISavable<Object>
    {
        public void LogicUpdate(SimulationConfiguration config);
        
        public void PhysicsUpdate(SimulationConfiguration config);

    }
}