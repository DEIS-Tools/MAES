namespace Dora.Robot
{
    public interface IRobotController
    {
        void SenseSurroundings();
        void MoveForward();
        void MoveBackwards();
        void RotateCounterClockwise();
        void RotateClockwise();
    }
    
}
