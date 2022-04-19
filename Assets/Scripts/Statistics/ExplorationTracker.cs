using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using Maes.Map;
using Maes.Map.Visualization;
using Maes.Robot;
using Maes.Utilities;
using UnityEngine;

namespace Maes.Statistics {
    public class ExplorationTracker {
        // The low-resolution collision map used to create the smoothed map that robots are navigating 
        private SimulationMap<bool> _collisionMap;
        private ExplorationVisualizer _explorationVisualizer;

        private SimulationMap<ExplorationCell> _explorationMap;
        private RayTracingMap<ExplorationCell> _rayTracingMap;
        private readonly int _explorationMapWidth;
        private readonly int _explorationMapHeight;

        public delegate void VisualizationModeConsumer(VisualizationMode mode);
        public event VisualizationModeConsumer OnVisualizationModeChanged = delegate(VisualizationMode mode) {  };

        private readonly int _totalExplorableTriangles;
        public int ExploredTriangles { get; private set; }
        public int CoveredMiniTiles { get; private set; }

        [CanBeNull] private MonaRobot _selectedRobot;

        private int _currentTick = 0;

        public float ExploredProportion => ExploredTriangles / (float) _totalExplorableTriangles;
        // Coverage is measured in 'mini-tiles'. Each large map tile consists of 4 mini-tiles, // TODO: Is is actually measured in mini tiles?
        // where each mini-tile is composed of two triangles
        public float CoverageProportion => CoveredMiniTiles / (float) _coverableTiles;
        
        // private readonly bool[,] _isCovered;
        // private readonly bool[,] _canBeCovered;
        private readonly int _coverableTiles;
        private bool _isFirstTick = true;
        private RobotConstraints _constraints;

        public List<SnapShot<float>> _coverSnapshots = new List<SnapShot<float>>();
        public List<SnapShot<float>> _exploreSnapshots = new List<SnapShot<float>>();

        private VisualizationMode _currentVisualizationMode;

        public struct SnapShot<TValue> {
            public readonly int Tick;
            public readonly TValue Value;

            public SnapShot(int tick, TValue value) {
                Tick = tick;
                this.Value = value;
            }
        }

        public ExplorationTracker(SimulationMap<bool> collisionMap, ExplorationVisualizer explorationVisualizer, RobotConstraints constraints) {
            var explorableTriangles = 0;
            _collisionMap = collisionMap;
            _explorationVisualizer = explorationVisualizer;
            _constraints = constraints;
            _explorationMap = collisionMap.FMap(isCellSolid => {
                if (!isCellSolid)
                    explorableTriangles++;
                return new ExplorationCell(isExplorable: !isCellSolid);
            });
            _currentVisualizationMode = new AllRobotsExplorationVisualization(_explorationMap);
            _totalExplorableTriangles = explorableTriangles;
            
            _explorationVisualizer.SetMap(_explorationMap, collisionMap.ScaledOffset);
            _rayTracingMap = new RayTracingMap<ExplorationCell>(_explorationMap);
            
            // Register all coverable tiles
            for (int x = 0; x < collisionMap.WidthInTiles; x++) {
                for (int y = 0; y < collisionMap.HeightInTiles; y++) {
                    var tileCells = collisionMap.GetTileByLocalCoordinate(x, y).GetTriangles();
                    var explorationCells = _explorationMap.GetTileByLocalCoordinate(x, y).GetTriangles();

                    for (int i = 0; i < 8; i+=2) {
                        // If any of the two triangles forming the 'mini-tile' are solid,
                        // then mark both triangles as non-coverable
                        // (because the entire mini-tile will be considered as solid in the SLAM map used by the robots)
                        var isSolid = tileCells[i] || tileCells[i+1];
                        explorationCells[i].CanBeCovered = !isSolid;
                        explorationCells[i+1].CanBeCovered = !isSolid;
                        if (!isSolid) _coverableTiles++;
                    }
                }
            }
        }

        public void CreateSnapShot() {
            _coverSnapshots.Add(new SnapShot<float>(_currentTick, CoverageProportion * 100));
            _exploreSnapshots.Add(new SnapShot<float>(_currentTick, ExploredProportion * 100));
        }

        private void UpdateCoverageStatus(MonaRobot robot) {
            var newlyCoveredCells = new List<(int, ExplorationCell)> {};
            var robotPos = robot.transform.position;
            var coverageRadius = robot.Controller.Constraints.RobotRelativeSize / 3f;
            // The maximum distance between the robot and the center of a tile
            // that would cause that tile to be considered covered 
            var centerCoverageRadius = coverageRadius + 0.25f;
            var robotMiniTileCenterX = (float) Math.Truncate(robotPos.x) + Mathf.Round(robotPos.x - (float) Math.Truncate(robotPos.x)) * 0.5f + ((robotPos.x < 0) ? -0.25f : 0.25f);
            var robotMiniTileCenterY = (float) Math.Truncate(robotPos.y) + Mathf.Round(robotPos.y - (float) Math.Truncate(robotPos.y)) * 0.5f + ((robotPos.y < 0) ? -0.25f : 0.25f);
            var robotMiniTilePos = new Vector2(robotMiniTileCenterX, robotMiniTileCenterY);
            // Loop through alle tiles currently near the robot
            for (int x = -1; x <= 1; x++) {
                for (int y = -1; y <= 1; y++) {
                    var tilePosition = robotMiniTilePos + new Vector2(x * 0.5f, y * 0.5f);
                    // Only consider this tile if they are within coverage range in obth x- and y-axis
                    if (Mathf.Max(Mathf.Abs(tilePosition.x - robotPos.x), Mathf.Abs(tilePosition.y - robotPos.y)) > centerCoverageRadius)
                        continue;

                    var (triangle1, triangle2) = 
                    _explorationMap.GetMiniTileTrianglesByWorldCoordinates(tilePosition);
                    if (!triangle1.Item2.IsCovered) {
                        CoveredMiniTiles++; // This tile was not covered before, register as first coverage
                        newlyCoveredCells.Add(triangle1);
                        newlyCoveredCells.Add(triangle2);
                    }
                    
                    triangle1.Item2.RegisterCoverage(_currentTick);
                    triangle2.Item2.RegisterCoverage(_currentTick);
                }   
            }
            _currentVisualizationMode.RegisterNewlyCoveredCells(robot, newlyCoveredCells);
        }

        private Vector2Int GetCoverageMapPosition(Vector2 robotPosition) {
            robotPosition -= _collisionMap.ScaledOffset;
            return new Vector2Int((int)robotPosition.x, (int)robotPosition.y);
        }
        
        public void LogicUpdate(List<MonaRobot> robots) {
            // The user can specify the tick interval at which the slam map is updated. 
            var shouldUpdateSlamMap = _constraints.AutomaticallyUpdateSlam && 
                                      _currentTick % _constraints.SlamUpdateIntervalInTicks == 0; 
            PerformRayTracing(robots, shouldUpdateSlamMap);

            // In the first tick, the robot does not have a position in the slam map.
            if (!_isFirstTick) foreach (var robot in robots) UpdateCoverageStatus(robot);
            else _isFirstTick = false;

            if (_constraints.AutomaticallyUpdateSlam) {
                // Always update estimated robot position and rotation
                // regardless of whether the slam map was updated this tick
                foreach (var robot in robots) {
                    var slamMap = robot.Controller.SlamMap;
                    slamMap.UpdateApproxPosition(robot.transform.position);
                    slamMap.SetApproxRobotAngle(robot.Controller.GetForwardAngleRelativeToXAxis());
                }
            }
            
            _currentVisualizationMode.UpdateVisualization(_explorationVisualizer, _currentTick);
            _currentTick++;
        }

        // Updates both exploration tracker and robot slam maps
        private void PerformRayTracing(List<MonaRobot> robots, bool shouldUpdateSlamMap) {
            List<(int, ExplorationCell)> newlyExploredTriangles = new List<(int, ExplorationCell)>();
            float visibilityRange = _constraints.SlamRayTraceRange;

            foreach (var robot in robots) {
                SlamMap slamMap = null;

                if (shouldUpdateSlamMap) {
                    slamMap = robot.Controller.SlamMap;
                    slamMap.ResetRobotVisibility();
                }

                // Use amount of traces specified by user, or calculate circumference and use trace at interval of 4
                float tracesPerMeter = 2f;
                int traces = _constraints.SlamRayTraceCount ?? (int) (Math.PI * 2f * _constraints.SlamRayTraceRange * tracesPerMeter);
                float traceIntervalDegrees = 360f / traces;
                for (int i = 0; i < traces; i++) {
                    var angle = i * traceIntervalDegrees;
                    // Avoid ray casts that can be parallel to the lines of a triangle
                    if (angle % 45 == 0) angle += 0.5f;

                    _rayTracingMap.Raytrace(robot.transform.position, angle, visibilityRange, (index, cell) => {
                        if (cell.IsExplorable ){
                            if (!cell.IsExplored) {
                                cell.IsExplored = true;
                                cell.LastExplorationTimeInTicks = _currentTick;
                                cell.ExplorationTimeInTicks += 1;
                                newlyExploredTriangles.Add((index, cell));
                                ExploredTriangles++;
                            }
                            cell.RegisterExploration(_currentTick);
                        }
                        
                        // Update robot slam map if present (slam map only non-null if 'shouldUpdateSlamMap' is true)
                        slamMap?.SetExploredByTriangle(triangleIndex: index, isOpen: cell.IsExplorable);
                        slamMap?.SetCurrentlyVisibleByTriangle(triangleIndex: index, isOpen: cell.IsExplorable);

                        return cell.IsExplorable;
                    });
                }
                
                // Register newly explored cells of this robot for visualization
                _currentVisualizationMode.RegisterNewlyExploredCells(robot, newlyExploredTriangles);
                newlyExploredTriangles.Clear();
            }
        }

        public void SetVisualizedRobot([CanBeNull] MonaRobot robot) {
            _selectedRobot = robot;
            if (_selectedRobot != null)
                SetVisualizationMode(new CurrentlyVisibleAreaVisualization(_explorationMap, _selectedRobot.Controller));
            else if (_currentVisualizationMode is CurrentlyVisibleAreaVisualization) 
                // Revert to all robots exploration visualization when current robot is deselected
                // while visualization mode is based on the selected robot
                SetVisualizationMode(new AllRobotsExplorationVisualization(_explorationMap));
        }

        public void ShowAllRobotExploration() {
            SetVisualizationMode(new AllRobotsExplorationVisualization(_explorationMap));
        }

        public void ShowAllRobotCoverage() {
            SetVisualizationMode(new AllRobotsCoverageVisualization(_explorationMap));
        }

        public void ShowAllRobotCoverageHeatMap() {
            SetVisualizationMode(new CoverageHeatMapVisualization(_explorationMap));
        }

        public void ShowAllRobotExplorationHeatMap() {
            SetVisualizationMode(new ExplorationHeatMapVisualization(_explorationMap));
        }
        
        public void ShowSelectedRobotVisibleArea() {
            if (_selectedRobot == null)
                throw new Exception("Cannot change to 'ShowSelectedRobotVisibleArea' visualization mode when no robot is selected");
            SetVisualizationMode(new CurrentlyVisibleAreaVisualization(_explorationMap, _selectedRobot.Controller));
        }

        private void SetVisualizationMode(VisualizationMode newMode) {
            _currentVisualizationMode = newMode;
            _currentVisualizationMode.UpdateVisualization(_explorationVisualizer, _currentTick);
            OnVisualizationModeChanged(_currentVisualizationMode);
        }


    }
}