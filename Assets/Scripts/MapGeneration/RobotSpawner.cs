using System.Collections.Generic;
using Dora.ExplorationAlgorithm;
using Dora.Robot;
using UnityEngine;

namespace Dora.MapGeneration
{
    public class RobotSpawner: MonoBehaviour
    {

        public GameObject robotPrefab;

        // Temporary method for testing! Return list of robots?
        public List<MonaRobot> SpawnRobots(SimulationMap<bool> collsionMap, int seed)
        {
            List<MonaRobot> robots = new List<MonaRobot>();
            
            for (int i = 0; i < 5; i++)
            {
                var robotID = i;
                var robotGameObject = Instantiate(robotPrefab, parent: transform);
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