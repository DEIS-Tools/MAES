namespace Dora
{
    public interface SimulationUnit
    {
        public void LogicUpdate(SimulationConfiguration config);


        public void PhysicsUpdate(SimulationConfiguration config);

    }
}