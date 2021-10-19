using System.Collections.Generic;
using Dora.ExplorationAlgorithm;
using Dora.Robot;
using UnityEngine;

namespace Dora.MapGeneration
{
    public class RobotSpawner: MonoBehaviour
    {

        public GameObject robotPrefab;
        public Transform simulationContainer;
        
        // Temporary method for testing! Return list of robots?
        public List<MonaRobot> SpawnRobots(SimulationMap<bool> collsionMap)
        {
            List<MonaRobot> robots = new List<MonaRobot>();
            
            for (int i = 0; i < 1; i++)
            {
                var robotID = i;
                var robotGameObject = Instantiate(robotPrefab, parent: simulationContainer);
                var robot = robotGameObject.GetComponent<MonaRobot>();

                robot.transform.position = new Vector3(0.1f, 0.1f);
                robot.id = robotID;
                robot.ExplorationAlgorithm = new RandomExplorationAlgorithm(robot, randomSeed: robotID);
                robots.Add(robot);
            }

            return robots;
        }
        
    }
}