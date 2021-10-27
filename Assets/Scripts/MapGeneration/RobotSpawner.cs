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

        public List<MonaRobot> SpawnRobotsInBiggestRoom(SimulationMap<bool> collisionMap, int seed, int numberOfRobots, float robotRelativeSize) {
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
                    x: tile.x,
                    y: tile.y,
                    relativeSize: robotRelativeSize,
                    robotId: robotId++,
                    algorithm: new RandomExplorationAlgorithm(seed + robotId),
                    collisionMap: collisionMap
                    ));
            }

            return robots;
        }

        public List<MonaRobot> SpawnRobotsTogether(SimulationMap<bool> collisionMap, int seed, int numberOfRobots, float robotRelativeSize, Coord? suggestedStartingPoint = null) {
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
                    catch (InvalidOperationException) {
                        throw new ArgumentException(
                            $"Could not find enough adjacent spawn tiles. Queue empty, but still needs {numberOfRobots - spawnTilesSelected.Count}");
                    }
                }
            }

            int robotId = 0;
            foreach (var spawnTile in spawnTilesSelected) {
                var robot = CreateRobot(
                    x: spawnTile.x,
                    y: spawnTile.y,
                    relativeSize: robotRelativeSize,
                    robotId: robotId++,
                    algorithm: new RandomExplorationAlgorithm(seed + robotId),
                    collisionMap: collisionMap
                );
                robots.Add(robot);
            }

            return robots;
        }

        public List<MonaRobot> SpawnAtHallWayEnds(SimulationMap<bool> collisionMap, int seed, int numberOfRobots, int robotRelativeSize) {
            var robots = new List<MonaRobot>();

            var hallWays = collisionMap.rooms.FindAll(r => r.isHallWay).ToList();
            List<Coord> possibleSpawnTiles = new List<Coord>();
            foreach (var hallWay in hallWays) {
                possibleSpawnTiles.AddRange(hallWay.tiles.Except(hallWay.edgeTiles));
            }
            
            possibleSpawnTiles.Sort((c1, c2) => {
                var c1DistanceFromTop = collisionMap.HeightInTiles - c1.y;
                var c1DistanceFromBottom = c1.y;
                var c1DistanceFromLeft = c1.x;
                var c1DistanceFromRight = collisionMap.WidthInTiles - c1.x;
                var c1Best = Math.Min(Math.Min(c1DistanceFromLeft, c1DistanceFromRight),Math.Min(c1DistanceFromTop, c1DistanceFromBottom));
                
                var c2DistanceFromTop = collisionMap.HeightInTiles - c2.y;
                var c2DistanceFromBottom = c2.y;
                var c2DistanceFromLeft = c2.x;
                var c2DistanceFromRight = collisionMap.WidthInTiles - c2.x;
                var c2Best = Math.Min(Math.Min(c2DistanceFromLeft, c2DistanceFromRight),Math.Min(c2DistanceFromTop, c2DistanceFromBottom));
                
                return c1Best - c2Best;
            });
            
            
            int robotId = 0;
            foreach (var tile in possibleSpawnTiles) {
                if (robotId == numberOfRobots)
                    break;
                
                robots.Add(CreateRobot(
                    x: tile.x,
                    y: tile.y,
                    relativeSize: robotRelativeSize,
                    robotId: robotId++,
                    algorithm: new RandomExplorationAlgorithm(seed + robotId),
                    collisionMap: collisionMap
                ));
            }
            

            return robots;
        }
        
        private MonaRobot CreateRobot(float x, float y, float relativeSize, int robotId, IExplorationAlgorithm algorithm, SimulationMap<bool> collisionMap) {
            var robotID = robotId;
            var robotGameObject = Instantiate(robotPrefab, parent: transform);
            var robot = robotGameObject.GetComponent<MonaRobot>();
            // robotRelativeSize is a floating point value in ]0,1.0]. 1.0 = robot is the same size as a tile.
            if (0.001f > relativeSize && relativeSize > 1.0000001f)
                throw new ArgumentException(
                    "Robot relative size cannot exceed 1.0f or be below 0.001f. Otherwise some areas of the map may be impossible to explore");
            robot.transform.localScale = new Vector3(0.495f * relativeSize * collisionMap.Scale,
                0.495f * relativeSize * collisionMap.Scale,
                0.495f * relativeSize * collisionMap.Scale);

            float RTOffset = 0.01f; // Offset is used, since being exactly at integer value positions can cause issues with ray tracing
            robot.transform.position = new Vector3((x * collisionMap.Scale) + RTOffset + collisionMap.ScaledOffset.x, (y * collisionMap.Scale) + RTOffset + collisionMap.ScaledOffset.y);
                
            robot.id = robotID;
            robot.ExplorationAlgorithm = algorithm;
            robot.movementController.CommunicationManager = CommunicationManager;
            algorithm.SetController(robot.movementController);

            return robot;
        }
        
        
        private bool IsInMapRange(int x, int y, int mapWidth, int mapHeight) {
            return x >= 0 && x < mapWidth && y >= 0 && y < mapHeight;
        }
    }
}