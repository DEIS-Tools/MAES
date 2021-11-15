using System;
using System.Collections.Generic;
using System.Diagnostics;
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

        public float ExploredProportion => ExploredTriangles / (float) _totalExplorableTriangles;

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
        }

        public void LogicUpdate(List<MonaRobot> robots) {
            List<int> newlyExploredTriangles = new List<int>();
            float visibilityRange = GlobalSettings.LidarRange;

            foreach (var robot in robots) {
                SlamMap slamMap = null;
                
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
                _explorationVisualizer.SetExplored(_selectedRobot.Controller.SlamMap, true);
        }

        public void SetVisualizedRobot([CanBeNull] MonaRobot robot) {
            _selectedRobot = robot;

            if (robot != null) {
                _explorationVisualizer.SetExplored(robot.Controller.SlamMap, true);
                // Update map to show slam map for given robot
                // _explorationVisualizer.SetExplored(robot.Controller.SlamMap);
            }
            else {
                // Update map to show exploration progress for all robots
                _explorationVisualizer.SetExplored(_explorationMap);
            }
        }
    }
}