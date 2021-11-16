using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading;
using Dora.MapGeneration;
using Dora.Robot;
using JetBrains.Annotations;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace Dora.Statistics {
    public class ExplorationTracker {
        // The low-resolution collision map used to create the smoothed map that robots are navigating 
        private SimulationMap<bool> _collisionMap;
        private ExplorationVisualizer _explorationVisualizer;

        private SimulationMap<ExplorationCell> _explorationMap;
        private RayTracingMap<ExplorationCell> _rayTracingMap;
        private readonly int _explorationMapWidth;
        private readonly int _explorationMapHeight;

        private readonly int _totalExplorableTriangles;
        public int ExploredTriangles { get; private set; }

        [CanBeNull] private MonaRobot _selectedRobot;

        private int _currentTick = 0;

        public float ExploredProportion => ExploredTriangles / (float) _totalExplorableTriangles;
        public float CoverageProportion => _tilesCovered / (float)_coverableTiles;
        
        private readonly bool[,] _coverageMap;
        private readonly bool[,] _canBeCovered;
        private readonly int _coverableTiles;
        private int _tilesCovered = 0;
        private bool _isFirstTick = true;

        public List<SnapShot<float>> _coverSnapshots = new List<SnapShot<float>>();
        public List<SnapShot<float>> _exploreSnapshots = new List<SnapShot<float>>();

        public struct SnapShot<TValue> {
            public readonly int Tick;
            public readonly TValue Value;

            public SnapShot(int tick, TValue value) {
                Tick = tick;
                this.Value = value;
            }
        }

        public ExplorationTracker(SimulationMap<bool> collisionMap, ExplorationVisualizer explorationVisualizer) {
            var explorableTriangles = 0;
            _collisionMap = collisionMap;
            _explorationVisualizer = explorationVisualizer;
            _explorationMap = collisionMap.FMap(isCellSolid => {
                if (!isCellSolid)
                    explorableTriangles++;

                return new ExplorationCell(!isCellSolid);
            });
            _totalExplorableTriangles = explorableTriangles;

            

            _explorationVisualizer.SetMap(_explorationMap, collisionMap.Scale, collisionMap.ScaledOffset);
            _rayTracingMap = new RayTracingMap<ExplorationCell>(_explorationMap);

            // Coverage
            _coverageMap = new bool[collisionMap.WidthInTiles, collisionMap.HeightInTiles];
            _canBeCovered = new bool[collisionMap.WidthInTiles, collisionMap.HeightInTiles];
            var openTiles = 0;
            for (int x = 0; x < collisionMap.WidthInTiles; x++) {
                for (int y = 0; y < collisionMap.WidthInTiles; y++) {
                    var tile = collisionMap.GetTileByLocalCoordinate(x, y);
                    if (tile.IsTrueForAll(isSolid => !isSolid)) {
                        openTiles++;
                        _canBeCovered[x, y] = true;
                    }
                }
            }

            _coverableTiles = openTiles;
        }

        public void CreateSnapShot() {
            _coverSnapshots.Add(new SnapShot<float>(_currentTick, CoverageProportion * 100));
            _exploreSnapshots.Add(new SnapShot<float>(_currentTick, ExploredProportion * 100));
        }

        private void UpdateCoverageStatus(MonaRobot robot) {
            var robotPos = GetCoverageMapPosition(robot.transform.position);
            // If already covered
            if (_coverageMap[robotPos.x, robotPos.y]) return;
            
            if(_canBeCovered[robotPos.x, robotPos.y])
                _tilesCovered++;
                _coverageMap[robotPos.x, robotPos.y] = true;
        }

        private Vector2Int GetCoverageMapPosition(Vector2 robotPosition) {
            robotPosition = robotPosition - _collisionMap.ScaledOffset;
            return new Vector2Int((int)robotPosition.x, (int)robotPosition.y);
        }

        public void LogicUpdate(List<MonaRobot> robots) {
            List<int> newlyExploredTriangles = new List<int>();
            float visibilityRange = GlobalSettings.LidarRange;

            foreach (var robot in robots) {
                SlamMap slamMap = null;

                // In the first tick, the robot does not have a position in the slam map.
                if (!_isFirstTick) UpdateCoverageStatus(robot);
                else _isFirstTick = false;

                if (robot.Controller.Constraints.AutomaticallyUpdateSlam) {
                    slamMap = robot.Controller.SlamMap;
                    slamMap.ResetRobotVisibility();
                    slamMap.UpdateApproxPosition(robot.transform.position);
                    slamMap.SetApproxRobotAngle(robot.Controller.GetForwardAngleRelativeToXAxis());
                }

                int traces = 90;
                float ratio = 360 / traces;
                for (int i = 0; i < traces; i++) {
                    var angle = i * ratio;
                    // Avoid ray casts that can be parallel to the lines of a triangle
                    if (angle % 45 == 0) angle += 0.5f;

                    _rayTracingMap.Raytrace(robot.transform.position, angle, visibilityRange, (index, cell) => {
                        if (cell.isExplorable && !cell.IsExplored) {
                            cell.IsExplored = true;
                            newlyExploredTriangles.Add(index);
                            ExploredTriangles++;
                        }

                        if (robot.Controller.Constraints.AutomaticallyUpdateSlam) {
                            slamMap.SetExploredByTriangle(triangleIndex: index, isOpen: cell.isExplorable);
                            slamMap.SetCurrentlyVisibleByTriangle(triangleIndex: index, isOpen: cell.isExplorable);
                        } 
                            
                        return cell.isExplorable;
                    });
                }
            }

            if (_selectedRobot == null)
                _explorationVisualizer.SetExplored(newlyExploredTriangles);
            else
                _explorationVisualizer.SetExplored(_selectedRobot.Controller.SlamMap, false);

            _currentTick++;
        }

        public void SetVisualizedRobot([CanBeNull] MonaRobot robot) {
            _selectedRobot = robot;

            if (robot != null) { 
                // Update map to show slam map for given robot
                _explorationVisualizer.SetExplored(robot.Controller.SlamMap, false);
            }
            else {
                // Update map to show exploration progress for all robots
                _explorationVisualizer.SetExplored(_explorationMap);
            }
        }
    }
}