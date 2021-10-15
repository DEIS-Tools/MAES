using Dora.ExplorationAlgorithm;
using UnityEngine;

namespace Dora.MapGeneration
{
    public class RobotSpawner: MonoBehaviour
    {

        public GameObject robotPrefab;
        public Transform simulationContainer;
        
        // Temporary method for testing! Return list of robots?
        public void SpawnRobots()
        {
            for (int i = 0; i < 1; i++)
            {
                var robotID = i;
                var robotGameObject = Instantiate(robotPrefab, parent: simulationContainer);
                var robot = robotGameObject.GetComponent<Robot.Robot>();
                robot.id = robotID;
                robot.ExplorationAlgorithm = new RandomExplorationAlgorithm(robot, randomSeed: robotID);                
            }
        }
        
    }
}