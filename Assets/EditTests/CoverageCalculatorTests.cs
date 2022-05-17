using System.Collections.Generic;
using Maes.Map;
using Maes.Statistics;
using NUnit.Framework;
using UnityEngine;

namespace EditTests {
    public class CoverageCalculatorTest {

        private CoverageCalculator _coverageCalculator;
        private SimulationMap<bool> _collisionMap;
        private SimulationMap<ExplorationCell> _explorationMap;

        private const int Width=50, Height=50;

        [SetUp]
        public void InitializeCalculatorAndMaps() {
            _collisionMap = GenerateCollisionMap();
            _explorationMap = _collisionMap.FMap(isSolid => new ExplorationCell(!isSolid));
            _coverageCalculator = new CoverageCalculator(_explorationMap, _collisionMap);
        }

        // Generates a collision map where only the edge tiles are solid
        private static SimulationMap<bool> GenerateCollisionMap() {
            SimulationMapTile<bool>[,] tiles = new SimulationMapTile<bool>[Width, Height];
            for (int x = 0; x < Width; x++) {
                for (int y = 0; y < Height; y++) {
                    var isSolid = IsGeneratedTileSolid(new Vector2Int(x, y));
                    tiles[x, y] = new SimulationMapTile<bool>(() => isSolid);
                }
            }

            return new SimulationMap<bool>(tiles, Vector2.zero);
        }

        // All edges are solid. All other tiles are non-solid
        private static bool IsGeneratedTileSolid(Vector2Int tileCoordinate) {
            return (tileCoordinate.x == 0 ||
                    tileCoordinate.y == 0 ||
                    tileCoordinate.x == Width - 1 || 
                    tileCoordinate.y == Height - 1);
        }
        
        [Test]
        public void RobotOnTopOfTileCoverageTest() {
            // The test robot is positioned in the middle of the coarse tile at coordinates (20, 20)
            // (Equivalent to the slam tile at (40, 40)
            var robotWorldPos = new Vector2(20.25f, 20.25f);
            
            var ((_, cell1), (_, cell2)) = _explorationMap.GetMiniTilesByCoarseTileCoordinate(robotWorldPos);
            
            // Assert that none of cells are covered in advance
            Assert.IsFalse(cell1.IsCovered);
            Assert.IsFalse(cell2.IsCovered);
            
            // Register coverage for the testing robot 
            _coverageCalculator.UpdateRobotCoverage(robotWorldPos, 1, (_, __, ___, ____) => {
            });
            
            // Assert that the status of the tiles has now changed
            Assert.IsTrue(cell1.IsCovered);
            Assert.IsTrue(cell2.IsCovered);
        }
        
        
        [Test]
        [TestCase(20.00f, 20.00f)]
        [TestCase(20.5f, 20.5f)]
        [TestCase(20.25f, 20.25f)]
        [TestCase(20.75f, 20.75f)]
        [TestCase(20.49f, 20.49f)]
        [TestCase(20.99f, 20.99f)]
        public void AdjacentTilesAreCoveredTest(float robotX, float robotY) {
            // The test robot is positioned in the middle of the coarse tile at coordinates (20, 20)
            // (Equivalent to the slam tile at (40, 40)
            var robotWorldPos = new Vector2(robotX, robotY);

            // Find all cells that are immediate neighbours of tile currently occupied by the robot
            var cells = new List<ExplorationCell>();
            for (int x = -1; x < 1; x++) {
                for (int y = -1; y < 1; y++) {
                    var xOffset = x * 0.5f;
                    var yOffset = y * 0.5f;
                    var ((_, cell1), (__, cell2)) = _explorationMap
                        .GetMiniTilesByCoarseTileCoordinate(robotWorldPos + new Vector2(xOffset, yOffset));
                    cells.Add(cell1);
                    cells.Add(cell2);
                }
            }
            
            // Assert that none of cells are covered in advance
            foreach (var cell in cells) 
                Assert.IsFalse(cell.IsCovered);
            
            // Register coverage for the testing robot 
            _coverageCalculator.UpdateRobotCoverage(robotWorldPos, 1, (_, __, ___, ____) => {
            });
            
            // Assert that the status of the tiles has now changed
            foreach (var cell in cells) 
                Assert.IsTrue(cell.IsCovered);
        }
        
        
        [Test]
        public void CoverageTimeUpdateTest() {
            // The test robot is positioned in the middle of the coarse tile at coordinates (20, 20)
            // (Equivalent to the slam tile at (40, 40)
            var robotWorldPos = new Vector2(20.25f, 20.25f);
            var ((_, cell1), (_, cell2)) = _explorationMap.GetMiniTilesByCoarseTileCoordinate(robotWorldPos);
            
            // Assert that none of cells are covered in advance
            Assert.IsFalse(cell1.IsCovered);
            Assert.IsFalse(cell2.IsCovered);
            
            const int coverageTick = 123456;
            // Register coverage for the testing robot 
            _coverageCalculator.UpdateRobotCoverage(robotWorldPos, coverageTick, (_, __, ___, ____) => {
            });
            
            // Assert that the status of the tiles has now changed
            Assert.IsTrue(cell1.IsCovered);
            Assert.IsTrue(cell2.IsCovered);
            // Assert the the coverage time is updated
            Assert.AreEqual(cell1.LastCoverageTimeInTicks, coverageTick);
            Assert.AreEqual(cell2.LastCoverageTimeInTicks, coverageTick);
            
            // Cover again at one tick later
            _coverageCalculator.UpdateRobotCoverage(robotWorldPos, coverageTick + 1, (_, __, ___, ____) => {
            });
            Assert.AreEqual(cell1.LastCoverageTimeInTicks, coverageTick + 1);
            Assert.AreEqual(cell2.LastCoverageTimeInTicks, coverageTick + 1);
        }
    }
}

