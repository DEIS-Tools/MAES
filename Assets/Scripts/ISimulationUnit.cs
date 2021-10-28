using System;

namespace Dora {
    public interface ISimulationUnit : ISavable<Object> {
        public void LogicUpdate();

        public void PhysicsUpdate();
    }
}