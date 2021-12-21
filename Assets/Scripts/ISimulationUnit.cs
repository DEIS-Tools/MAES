using System;

namespace Maes {
    public interface ISimulationUnit : ISavable<Object> {
        public void LogicUpdate();

        public void PhysicsUpdate();
    }
}