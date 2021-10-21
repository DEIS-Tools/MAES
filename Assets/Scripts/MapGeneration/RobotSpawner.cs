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

        public List<MonaRobot> SpawnRobotsInBiggestRoom(SimulationMap<bool> collisionMap, int seed, int numberOfRobots) {
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
            numberOfRobots = possibleSpawnTiles.Count;
            foreach (var tile in possibleSpawnTiles) {
                if (robotId == numberOfRobots)
                    break;
                
                robots.Add(CreateRobot(
                    x: (tile.x * collisionMap.Scale) - collisionMap.WidthInTiles,
                    y: (tile.y * collisionMap.Scale) - collisionMap.HeightInTiles,
                    scale: collisionMap.Scale,
                    robotId: robotId++,
                    algorithm: new RandomExplorationAlgorithm(seed + robotId)
                    ));
            }

            return robots;
        }

        public List<MonaRobot> SpawnRobotsTogether(SimulationMap<bool> collisionMap, int seed, int numberOfRobots, Coord? suggestedStartingPoint = null) {
            List<MonaRobot> robots = new List<MonaRobot>();
            
            // Get all spawnable tiles. We cannot spawn adjacent to a wall
            List<Coord> possibleSpawnTiles = new List<Coord>();
            foreach (var room in collisionMap.rooms) {
                possibleSpawnTiles.AddRange(room.tiles.Except(room.edgeTiles));
            }

            // If no starting point suggested, simply start as close as 0,0 as possible.
            if (suggestedStartingPoint == null) {
                // Make them spawn in a ordered fashion
                possibleSpawnTiles.Sort((c1, c2) => {
                    if (c1.x == c2.x)
                        return c1.y - c2.y;
                    return c1.x - c2.x;
                });
            }
            else {
                possibleSpawnTiles.Sort((c1, c2) => {
                    return c1.ManhattanDistanceTo(suggestedStartingPoint.Value) -
                           c2.ManhattanDistanceTo(suggestedStartingPoint.Value);
                });
            }

            // Flooding algorithm to find next tiles from neighbors
            var spawnTilesSelected = new List<Coord>();
            var startCoord = possibleSpawnTiles[0];
            // int[,] mapFlags = new int[collisionMap.WidthInTiles,  collisionMap.HeightInTiles];
            Queue<Coord> queue = new Queue<Coord> ();
            queue.Enqueue(startCoord);
            
            while (queue.Count > 0 && spawnTilesSelected.Count < numberOfRobots) {
                Coord tile = queue.Dequeue();
                spawnTilesSelected.Add(tile);

                // Check immediate neighbours
                for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                        if (IsInMapRange(x, y, collisionMap.WidthInTiles, collisionMap.HeightInTiles)
                            && (y == tile.y || x == tile.x)) {
                            var neighbourCoord = new Coord(x, y);
                            if (!spawnTilesSelected.Contains(neighbourCoord) 
                                && possibleSpawnTiles.Contains(neighbourCoord)
                                && !queue.Contains(neighbourCoord)) {
                                queue.Enqueue(neighbourCoord);
                            }
                        }
                    }
                }

                // If the current room is filled up, select a new starting point
                if (queue.Count < 1 && spawnTilesSelected.Count < numberOfRobots) {
                    try {
                        var newStartingPoint = possibleSpawnTiles.FirstOrDefault(c => !spawnTilesSelected.Contains(c));
                        queue.Enqueue(newStartingPoint);
                    }
                    catch (InvalidOperationException e) {
                        throw new ArgumentException(
                            $"Could not find enough adjacent spawn tiles. Queue empty, but still needs {numberOfRobots - spawnTilesSelected.Count}");
                    }
                }
            }

            int robotId = 0;
            foreach (var spawnTile in spawnTilesSelected) {
                var robot = CreateRobot(
                    x: (spawnTile.x * collisionMap.Scale) - collisionMap.WidthInTiles,
                    y: (spawnTile.y * collisionMap.Scale) - collisionMap.HeightInTiles,
                    scale: collisionMap.Scale,
                    robotId: robotId++,
                    algorithm: new RandomExplorationAlgorithm(seed + robotId)
                );
                robots.Add(robot);
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
            algorithm.SetController(robot.movementController);

            return robot;
        }
        
        
        private bool IsInMapRange(int x, int y, int mapWidth, int mapHeight) {
            return x >= 0 && x < mapWidth && y >= 0 && y < mapHeight;
        }
    }
}