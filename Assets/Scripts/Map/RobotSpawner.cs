using System;
using System.Collections.Generic;
using System.Linq;
using Maes.ExplorationAlgorithm;
using Maes.Map.MapGen;
using Maes.Robot;
using UnityEngine;
using static Maes.Utilities.Geometry;

namespace Maes.Map
{
    public class RobotSpawner: MonoBehaviour {
        public delegate IExplorationAlgorithm CreateAlgorithmDelegate(int randomSeed);

        public GameObject robotPrefab;

        public CommunicationManager CommunicationManager;

        public RobotConstraints RobotConstraints;
        

        /// <summary>
        /// Spawns the robots in the biggest room. For building type map this is usually the hall way.
        /// </summary>
        /// <param name="collisionMap">Information regarding the possible spawn positions</param>
        /// <param name="seed">Injected into the spawned robots</param>
        /// <param name="numberOfRobots">How many robots should be created. The map may not fit all robots, which would throw an exception</param>
        /// <param name="createAlgorithmDelegate">Used to inject the exploration algorithm into the robot controller</param>
        /// <returns>List of all robot game objects.</returns>
        /// <exception cref="ArgumentException">If not enough open tiles for the requested number of robots.</exception>
        public List<MonaRobot> SpawnRobotsInBiggestRoom(SimulationMap<bool> collisionMap, int seed, int numberOfRobots, CreateAlgorithmDelegate createAlgorithmDelegate) {
            List<MonaRobot> robots = new List<MonaRobot>();

            // Sort by room size
            collisionMap.rooms.Sort((r1, r2) =>
                r2.RoomSizeExcludingEdgeTiles() - r1.RoomSizeExcludingEdgeTiles());

            var biggestRoom = collisionMap.rooms[0];
            
            // We need to peel off two layers of edges to make sure, that no robot is on a partly covered tile
            var roomWithoutEdgeTiles = biggestRoom.tiles.Except(biggestRoom.edgeTiles).ToList();
            var secondLayerOfEdgesTiles = FindEdgeTiles(roomWithoutEdgeTiles, true);
            var possibleSpawnTiles = roomWithoutEdgeTiles.Except(secondLayerOfEdgesTiles).ToList();

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
                    relativeSize: RobotConstraints.AgentRelativeSize,
                    robotId: robotId++,
                    algorithm: createAlgorithmDelegate(seed + robotId),
                    collisionMap: collisionMap,
                    seed: seed + robotId
                ));
            }

            return robots;
        }

        /// <summary>
        /// Spawns the robots either from the lower left corner or at the suggested starting point.
        /// </summary>
        /// <param name="collisionMap">Information regarding the possible spawn positions</param>
        /// <param name="seed">Injected into the spawned robots</param>
        /// <param name="numberOfRobots">How many robots should be created. The map may not fit all robots, which would throw an exception</param>
        /// <param name="suggestedStartingPoint">A flooding algorithm is performed to spawn robots as close as possible to this point.</param>
        /// <param name="createAlgorithmDelegate">Used to inject the exploration algorithm into the robot controller</param>
        /// <returns>List of all robot game objects.</returns>
        /// <exception cref="ArgumentException">If not enough open tiles for the requested number of robots.</exception>
        public List<MonaRobot> SpawnRobotsTogether(SimulationMap<bool> collisionMap, int seed, int numberOfRobots, Vector2Int? suggestedStartingPoint, CreateAlgorithmDelegate createAlgorithmDelegate) {
            List<MonaRobot> robots = new List<MonaRobot>();
            // Get all spawnable tiles. We cannot spawn adjacent to a wall
            List<Vector2Int> possibleSpawnTiles = new List<Vector2Int>();

            for (int x = 0; x < collisionMap.WidthInTiles; x++) {
                for (int y = 0; y < collisionMap.HeightInTiles; y++) {
                    if (collisionMap.GetTileByLocalCoordinate(x, y).IsTrueForAll(solid => !solid)) {
                        possibleSpawnTiles.Add(new Vector2Int(x, y));
                    }
                    
                }
            }

            // Remove the edges to make sure the robots are not in a solid coarse tile
            var edgeTiles = FindEdgeTiles(possibleSpawnTiles, true);
            possibleSpawnTiles = possibleSpawnTiles.Except(edgeTiles).ToList();
            
            // Offset suggested starting point to map
            suggestedStartingPoint = new Vector2Int(suggestedStartingPoint.Value.x - (int)collisionMap.ScaledOffset.x,
                suggestedStartingPoint.Value.y - (int)collisionMap.ScaledOffset.y);
            
            possibleSpawnTiles.Sort((c1, c2) => {
                return ManhattanDistance(c1, suggestedStartingPoint.Value) -
                       ManhattanDistance(c2, suggestedStartingPoint.Value);
            });
            
            
            // Flooding algorithm to find next tiles from neighbors
            var spawnTilesSelected = new List<Vector2Int>();
            var startCoord = possibleSpawnTiles[0];
            Queue<Vector2Int> queue = new Queue<Vector2Int>();
            queue.Enqueue(startCoord);
            while (queue.Count > 0 && spawnTilesSelected.Count < numberOfRobots) {
                Vector2Int tile = queue.Dequeue();
                spawnTilesSelected.Add(tile);

                // Check immediate neighbours
                for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                        if (IsInMapRange(x, y, collisionMap.WidthInTiles, collisionMap.HeightInTiles)
                            && (y == tile.y || x == tile.x)) {
                            var neighbourCoord = new Vector2Int(x, y);
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
                    relativeSize: RobotConstraints.AgentRelativeSize,
                    robotId: robotId++,
                    algorithm: createAlgorithmDelegate(seed + robotId),
                    collisionMap: collisionMap,
                    seed: seed + robotId
                );
                robots.Add(robot);
            }

            return robots;
        }

        /// <summary>
        /// SHOULD ONLY BE USED FOR BUILDING TYPE MAPS. Not cave maps.
        /// Spawns the robot in a spiraling pattern around the boundary of the map, but only in the hallways.
        /// </summary>
        /// <param name="collisionMap">Information regarding the possible spawn positions</param>
        /// <param name="seed">Injected into the spawned robots</param>
        /// <param name="numberOfRobots">How many robots should be created. The map may not fit all robots, which would throw an exception</param>
        /// <param name="createAlgorithmDelegate">Used to inject the exploration algorithm into the robot controller</param>
        /// <returns>List of all robot game objects.</returns>
        public List<MonaRobot> SpawnAtHallWayEnds(SimulationMap<bool> collisionMap, int seed, int numberOfRobots, CreateAlgorithmDelegate createAlgorithmDelegate) {
            var robots = new List<MonaRobot>();

            var hallWays = collisionMap.rooms.FindAll(r => r.isHallWay).ToList();
            List<Vector2Int> possibleSpawnTiles = new List<Vector2Int>();
            foreach (var hallWay in hallWays) {
                possibleSpawnTiles.AddRange(hallWay.tiles.Except(hallWay.edgeTiles));
            }

            possibleSpawnTiles.Sort((c1, c2) => {
                var c1DistanceFromTop = collisionMap.HeightInTiles - c1.y;
                var c1DistanceFromBottom = c1.y;
                var c1DistanceFromLeft = c1.x;
                var c1DistanceFromRight = collisionMap.WidthInTiles - c1.x;
                var c1Best = Math.Min(Math.Min(c1DistanceFromLeft, c1DistanceFromRight),
                    Math.Min(c1DistanceFromTop, c1DistanceFromBottom));

                var c2DistanceFromTop = collisionMap.HeightInTiles - c2.y;
                var c2DistanceFromBottom = c2.y;
                var c2DistanceFromLeft = c2.x;
                var c2DistanceFromRight = collisionMap.WidthInTiles - c2.x;
                var c2Best = Math.Min(Math.Min(c2DistanceFromLeft, c2DistanceFromRight),
                    Math.Min(c2DistanceFromTop, c2DistanceFromBottom));

                return c1Best - c2Best;
            });


            int robotId = 0;
            foreach (var tile in possibleSpawnTiles) {
                if (robotId == numberOfRobots)
                    break;

                robots.Add(CreateRobot(
                    x: tile.x,
                    y: tile.y,
                    relativeSize: RobotConstraints.AgentRelativeSize,
                    robotId: robotId++,
                    algorithm: createAlgorithmDelegate(seed + robotId),
                    collisionMap: collisionMap,
                    seed: seed + robotId
                ));
            }


            return robots;
        }

        private MonaRobot CreateRobot(float x, float y, float relativeSize, int robotId,
            IExplorationAlgorithm algorithm, SimulationMap<bool> collisionMap, int seed) {
            var robotID = robotId;
            var robotGameObject = Instantiate(robotPrefab, parent: transform);
            var robot = robotGameObject.GetComponent<MonaRobot>();
            // robotRelativeSize is a floating point value in ]0,1.0]. 1.0 = robot is the same size as a tile.
            if (0.001f > relativeSize && relativeSize > 1.0000001f)
                throw new ArgumentException(
                    "Robot relative size cannot exceed 1.0f or be below 0.001f. Otherwise some areas of the map may be impossible to explore");
            robot.transform.localScale = new Vector3(
                0.495f * relativeSize,
                0.495f * relativeSize,
                0.495f * relativeSize
            );

            robot.outLine.enabled = false;

            float
                RTOffset = 0.01f; // Offset is used, since being exactly at integer value positions can cause issues with ray tracing
            robot.transform.position = new Vector3(x + RTOffset + collisionMap.ScaledOffset.x,
                y + RTOffset + collisionMap.ScaledOffset.y);

            robot.id = robotID;
            robot.ExplorationAlgorithm = algorithm;
            robot.Controller.CommunicationManager = CommunicationManager;
            robot.Controller.SlamMap = new SlamMap(collisionMap, RobotConstraints, seed);
            robot.Controller.Constraints = RobotConstraints; 
            algorithm.SetController(robot.Controller);
            
            return robot;
        }
        
        private List<Vector2Int> FindEdgeTiles(List<Vector2Int> tiles, bool checkDiagonal) {
            var tilesHashSet = new HashSet<Vector2Int>();
            foreach (var tile in tiles) tilesHashSet.Add(tile);
            
            // An edge is any tile, where a neighbor is missing in the set of tiles.
            var edgeTiles = new List<Vector2Int>();

            foreach (var tile in tilesHashSet) {
                var isEdge = false;
                for (int x = tile.x - 1; x <= tile.x + 1; x++) {
                    for (int y = tile.y - 1; y <= tile.y + 1; y++) {
                        if (checkDiagonal) {
                            if (x == tile.x || y == tile.y) {
                                var neighbour = new Vector2Int(x, y);
                                if (!tilesHashSet.Contains(neighbour)) isEdge = true;
                            } 
                        }
                        else {
                            var neighbour = new Vector2Int(x, y);
                            if (!tilesHashSet.Contains(neighbour)) isEdge = true;
                        }
                        
                    }
                }
                
                if(isEdge) edgeTiles.Add(tile);
            }

            return edgeTiles;
        }


        private bool IsInMapRange(int x, int y, int mapWidth, int mapHeight) {
            return x >= 0 && x < mapWidth && y >= 0 && y < mapHeight;
        }
    }
}