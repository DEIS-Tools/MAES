using Dora.Robot.ExplorationAlgorithm;
using UnityEngine;

namespace Dora.MapGeneration
{
    public class RobotSpawner: MonoBehaviour
    {

        public GameObject robotPrefab;
        public Transform simulationContainer;
        
        // Temporary method for testing!
        public void SpawnRobots()
        {
            var gameObject = Instantiate(robotPrefab, parent: simulationContainer);
            gameObject.GetComponent<Robot.Robot>().ExplorationAlgorithm = new RandomExplorationAlgorithm();
        }
        
    }
}