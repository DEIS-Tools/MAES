using System;
using System.Collections.Generic;
using System.Linq;
using Dora.ExplorationAlgorithm;
using Dora.Robot;
using UnityEngine;
using UnityEngine.Tilemaps;

namespace Dora.MapGeneration
{
    public class RobotSpawner: MonoBehaviour
    {
        private delegate IExplorationAlgorithm AlgorithmDelegate(MonaRobot robot, int randomSeed);

        public GameObject robotPrefab;

        public CommunicationManager CommunicationManager;
        
        // Temporary method for testing! Return list of robots?
        public List<MonaRobot> SpawnRobots(SimulationMap<bool> collisionMap, int seed, int numberOfRobots)
        {
            List<MonaRobot> robots = new List<MonaRobot>();
            
            for (int i = 0; i < numberOfRobots; i++)
            {
                robots.Add(CreateRobot(
                    x: 0.1f * i,
                    y: 0.1f * i,
                    scale: collisionMap.Scale,
                    robotId: i,
                    algorithm: new RandomExplorationAlgorithm(seed)
                ));
            }

            return robots;
        }

        public List<MonaRobot> SpawnRobotsTogetherBiggestRoom(SimulationMap<bool> collisionMap, int seed, int numberOfRobots) {
            List<MonaRobot> robots = new List<MonaRobot>();

            // Sort by room size
            collisionMap.rooms.Sort((r1, r2) =>
                r2.RoomSizeExcludingEdgeTiles() - r1.RoomSizeExcludingEdgeTiles());

            var biggestRoom = collisionMap.rooms[0];
            var possibleSpawnTiles = biggestRoom.tiles.Except(biggestRoom.edgeTiles).ToList();

            if (possibleSpawnTiles.Count < numberOfRobots)
                throw new ArgumentException("Room not big enough to fit the robots");

            // Make them spawn in a ordered fashion
            possibleSpawnTiles.Sort((c1, c2) => {
                if (c1.x == c2.x)
                    return c1.y - c2.y;
                return c1.x - c2.x;
            });
            
            
            int robotId = 0;
            foreach (var tile in possibleSpawnTiles) {
                if (robotId == numberOfRobots)
                    break;
                
                robots.Add(CreateRobot(
                    x: (tile.x * collisionMap.Scale) - collisionMap.HeightInTiles,
                    y: (tile.y * collisionMap.Scale) - collisionMap.HeightInTiles,
                    scale: collisionMap.Scale,
                    robotId: robotId++,
                    algorithm: new RandomExplorationAlgorithm(seed)
                    ));
            }

            return robots;
        }
        private MonaRobot CreateRobot(float x, float y, float scale, int robotId, IExplorationAlgorithm algorithm) {
            var robotID = robotId++;
            var robotGameObject = Instantiate(robotPrefab, parent: transform);
            var robot = robotGameObject.GetComponent<MonaRobot>();
            robot.transform.localScale = new Vector3(0.495f * scale, 0.495f * scale, 0.495f * scale);
                
            float offset = 0.01f; // Offset is used, since being exactly at integer value positions can cause issues with ray tracing
            robot.transform.position = new Vector3(x + offset, y + offset);
                
            robot.id = robotID;
            robot.ExplorationAlgorithm = algorithm;
            robot.movementController.CommunicationManager = CommunicationManager;
            algorithm.SetController(robot.movementController);

            return robot;
        }
        
    }
}