// Copyright 2024 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard, Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen,
// 
// Original repository: https://github.com/Molitany/MAES

using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using Maes.Map;
using Maes.Map.MapGen;
using Maes.Map.Visualization;
using Maes.Robot;
using Maes.Utilities;
using UnityEngine;

namespace Maes.Statistics {
    internal class ExplorationTracker {

        private CoverageCalculator _coverageCalculator;
        
        // The low-resolution collision map used to create the smoothed map that robots are navigating 
        private SimulationMap<Tile> _collisionMap;
        private ExplorationVisualizer _explorationVisualizer;

        private SimulationMap<ExplorationCell> _explorationMap;
        private RayTracingMap<ExplorationCell> _rayTracingMap;
        private readonly int _explorationMapWidth;
        private readonly int _explorationMapHeight;

        public delegate void VisualizationModeConsumer(VisualizationMode mode);
        public event VisualizationModeConsumer OnVisualizationModeChanged = delegate(VisualizationMode mode) {  };

        private readonly int _totalExplorableTriangles;
        public int ExploredTriangles { get; private set; }
        public int CoveredMiniTiles => _coverageCalculator.CoveredMiniTiles;

        [CanBeNull] private MonaRobot _selectedRobot;

        private int _currentTick = 0;

        public float ExploredProportion => ExploredTriangles / (float) _totalExplorableTriangles;
        // Coverage is measured in 'mini-tiles'. Each large map tile consists of 4 mini-tiles, 
        // where each mini-tile is composed of two triangles
        public float CoverageProportion => _coverageCalculator.CoverageProportion;

        private bool _isFirstTick = true;
        private RobotConstraints _constraints;

        public List<SnapShot<float>> _coverSnapshots = new List<SnapShot<float>>();
        public List<SnapShot<float>> _exploreSnapshots = new List<SnapShot<float>>();
        public List<SnapShot<float>> _distanceSnapshots = new List<SnapShot<float>>();
        private float mostRecentDistance;
        
        private VisualizationMode _currentVisualizationMode;

        public struct SnapShot<TValue> {
            public readonly int Tick;
            public readonly TValue Value;

            public SnapShot(int tick, TValue value) {
                Tick = tick;
                this.Value = value;
            }
        }

        public ExplorationTracker(SimulationMap<Tile> collisionMap, ExplorationVisualizer explorationVisualizer, RobotConstraints constraints) {
            var explorableTriangles = 0;
            _collisionMap = collisionMap;
            _explorationVisualizer = explorationVisualizer;
            _constraints = constraints;
            _explorationMap = collisionMap.FMap(tile => {
                if (!Tile.IsWall(tile.Type))
                    explorableTriangles++;
                return new ExplorationCell(isExplorable: !Tile.IsWall(tile.Type));
            });
            _currentVisualizationMode = new AllRobotsExplorationVisualization(_explorationMap);
            _totalExplorableTriangles = explorableTriangles;
            
            _explorationVisualizer.SetMap(_explorationMap, collisionMap.ScaledOffset);
            _rayTracingMap = new RayTracingMap<ExplorationCell>(_explorationMap);

            _coverageCalculator = new CoverageCalculator(_explorationMap, collisionMap);
        }

        public void CreateSnapShot() {
            _coverSnapshots.Add(new SnapShot<float>(_currentTick, CoverageProportion * 100));
            _exploreSnapshots.Add(new SnapShot<float>(_currentTick, ExploredProportion * 100));
            _distanceSnapshots.Add(new SnapShot<float>(_currentTick, mostRecentDistance));
        }

        private float calculateAverageDistance(List<MonaRobot> robots){
            List<float> averages = new List<float>();
            float average = 0;
            float sum = 0;
            foreach (var robot in robots) {
                var robotPosition = robot.transform.position;
                foreach (var otherRobot in robots){
                    var otherRobotPosition = otherRobot.transform.position;
                    averages.Add((float)Math.Sqrt(Math.Pow(robotPosition.x - otherRobotPosition.x, 2) + Math.Pow(robotPosition.y - otherRobotPosition.y, 2) + Math.Pow(robotPosition.z - otherRobotPosition.z, 2)));
                }
            }
            foreach (var number in averages) {
                sum += number;
            }
            average = sum / averages.Count;
            return average;
        }
        private void UpdateCoverageStatus(MonaRobot robot) {
            var newlyCoveredCells = new List<(int, ExplorationCell)> {};
            var robotPos = robot.transform.position;
            
            // Find each mini tile (two triangle cells) covered by the robot and execute the following function on it
            _coverageCalculator.UpdateRobotCoverage(robotPos, _currentTick,(index1, triangle1, index2, triangle2) => {
                if (!triangle1.IsCovered) {
                    // This tile was not covered before, register as newly covered
                    newlyCoveredCells.Add((index1, triangle1));
                    newlyCoveredCells.Add((index2, triangle2));
                }
            });
            
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
            if (!_isFirstTick) {
                foreach (var robot in robots) UpdateCoverageStatus(robot);
                mostRecentDistance = calculateAverageDistance(robots);
            } 
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
            else 
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

        public void ShowSelectedRobotSlamMap() {
            if (_selectedRobot == null)
                throw new Exception("Cannot change to 'ShowSelectedRobotSlamMap' visualization mode when no robot is selected");
            SetVisualizationMode(new SelectedRobotSlamMapVisualization(_explorationMap, _selectedRobot.Controller));
        }

        private void SetVisualizationMode(VisualizationMode newMode) {
            _currentVisualizationMode = newMode;
            _currentVisualizationMode.UpdateVisualization(_explorationVisualizer, _currentTick);
            OnVisualizationModeChanged(_currentVisualizationMode);
        }


    }
}