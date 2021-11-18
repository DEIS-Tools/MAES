using System.Collections.Generic;
using System.Linq;
using Dora.MapGeneration.PathFinding;
using Dora.Robot;
using Dora.Utilities;
using JetBrains.Annotations;
using UnityEngine;

namespace Dora.ExplorationAlgorithm.TheNextFrontier {
    public class TnfExplorationAlgorithm : IExplorationAlgorithm {
        private static class Gaussian {
            private const float A = 5f, B = .5f, C = .1f; // Magnitude, Mean, and Spread.
            private const float Alpha = -1 / (C * C * 2);
            private const float Beta = B / (C * C);
            private static readonly float Gamma;

            static Gaussian() { // My best suggested constexpr substitute
                Gamma = Mathf.Log(A) - B * B / (2 * C * C);
            }

            /// <returns>
            /// value based on <a href="https://en.wikipedia.org/wiki/Gaussian_function#Properties">this</a> formula.
            /// </returns>
            public static float Gauss(float x) {
                return Mathf.Exp(Alpha * x * x + Beta * x + Gamma);
            }
        }

        private enum TnfStatus {
            Communicating,
            AwaitMoving,
            AwaitRotating,
            AwaitNextFrontier,
            AwaitCollisionMitigation
        }

        private IRobotController _robotController;
        private TnfStatus _robotTnfStatus;
        private SlamAlgorithmInterface _map;
        private int Alpha { get; }
        private int Beta { get; }

        private List<Frontier> _frontiers;

        private Vector2Int _robotPos = new Vector2Int(0, 0);
        private LinkedList<PathStep> _path;
        private PathStep _nextTileInPath;
        private const float AngleDelta = 1f;
        private const float MinimumMoveDistance = 0.8f;


        private class Frontier {
            public readonly List<(Vector2Int, float)> cells;

            public Frontier(List<(Vector2Int, float)> cells) {
                this.cells = cells;
                UtilityValue = -1;
            }

            public float UtilityValue { get; set; }
        }

        private float InformationPotential((Vector2Int, float) cell, List<(Vector2Int, float)> neighbours) {
            return Gaussian.Gauss(cell.Item2) + neighbours.Sum(neighbour => Gaussian.Gauss(neighbour.Item2));
        }

        private float InformationFactor(Frontier frontier) {
            return frontier.cells.Sum(c => InformationPotential(c, GetNeighbouringCells(c)));
        }

        private List<(Vector2Int, float)> GetNeighbouringCells((Vector2Int, float) cell) {
            var res = new List<(Vector2Int, float)>();

            for (var x = cell.Item1.x - 1; x < cell.Item1.x + 1; x++) {
                for (var y = cell.Item1.y - 1; y < cell.Item1.y + 1; y++) {
                    if (x == cell.Item1.x && y == cell.Item1.y) {
                        continue;
                    }
                    var position = new Vector2Int(x, y);
                    var v = _map.GetStatusOfTile(position).ToTnfCellValue();
                    res.Add((position, v));
                }
            }

            return res;
        }

        private float WavefrontNormalized(Frontier frontier, Vector2Int fromPosition, float normalizerConstant) {
            var wave = Wavefront(frontier, fromPosition).ToList();
            //var norm = Mathf.Sqrt(wave.Select(d => d * d).Sum());
            var distFactor = Mathf.Sqrt(wave.Select(d => d / normalizerConstant).Sum());
            return distFactor;
        }

        private float DistanceFactor(Frontier f, Vector2Int from, float normalizerConstant) {
            var val = WavefrontNormalized(f, from, normalizerConstant);
            return Mathf.Pow(val, Alpha) * Mathf.Pow(1 - val, Beta);
        }

        private IEnumerable<float> Wavefront(Frontier frontier, Vector2Int fromPosition) {
            return frontier.cells.Select(cell => Vector2Int.Distance(fromPosition, cell.Item1));
        }

        private float CoordinationFactor(Frontier frontier) {
            var neighbours = _robotController.SenseNearbyRobots();
            if (_robotTnfStatus != TnfStatus.Communicating) {
                if (neighbours.Count <= 0) {
                    return 0;
                }
                _robotTnfStatus = TnfStatus.Communicating;
                _robotController.Broadcast((_map, _robotController.GetRobotID()));
            }

            var sum = 0f;
            foreach (var neighbour in neighbours) {
                var pos = _robotPos + Vector2Int.RoundToInt(Geometry.VectorFromDegreesAndMagnitude(neighbour.Angle, neighbour.Distance));
                var normalizerConstant = _frontiers.Max(f => f.cells.Select(c => Vector2Int.Distance(pos, c.Item1)).Max());
                sum += WavefrontNormalized(frontier, pos, normalizerConstant);
            }
            return sum;
        }

        private float UtilityFunction(Frontier frontier, float normalizerConstant) {
            return InformationFactor(frontier) + DistanceFactor(frontier, _robotPos, normalizerConstant) - CoordinationFactor(frontier);
        }

        public void UpdateLogic() {

            _robotPos = Vector2Int.RoundToInt(_robotController.GetSlamMap().GetApproxPosition());
            if (_robotController.HasCollidedSinceLastLogicTick()) {
                _robotController.StopCurrentTask();
                CollisionMitigation();
                return;
            }

            switch (_robotTnfStatus) {
                case TnfStatus.AwaitNextFrontier:
                    var nextFrontier = CalculateNextFrontier(_robotPos);
                    if (nextFrontier == null || _robotTnfStatus == TnfStatus.Communicating) {
                        break;
                    }
                    MoveToFrontier(nextFrontier, _robotPos);
                    break;
                case TnfStatus.AwaitMoving:
                case TnfStatus.AwaitRotating:
                    DoMovement(_path);
                    break;
                case TnfStatus.Communicating:
                    AwaitCommunication();
                    break;
                case TnfStatus.AwaitCollisionMitigation:
                    if (_robotController.GetStatus() == RobotStatus.Idle)
                        _robotTnfStatus = TnfStatus.AwaitNextFrontier;
                    break;
            }

        }

        private void CollisionMitigation() {
            _robotTnfStatus = TnfStatus.AwaitCollisionMitigation;
            _robotController.Move(.3f, true);
        }

        private void AwaitCommunication() {
            var received = _robotController.ReceiveBroadcast();
            if (received.Any()) {
                var newMaps = new List<SlamMap>();
                foreach (var package in received) {
                    var pack = ((SlamAlgorithmInterface, int)) package;
                    newMaps.Add(pack.Item1 as SlamMap);
                }
                SlamMap.Combine(_map as SlamMap, newMaps);
                _robotTnfStatus = TnfStatus.AwaitNextFrontier;
            }

        }

        [CanBeNull]
        private Frontier CalculateNextFrontier(Vector2Int currentPosition) {
            // Get frontiers
            _frontiers = GetFrontiers();

            // For each frontier:
            if (_frontiers.Count <= 0) {
                return null;
            }
            var normalizerConstant = _frontiers.Max(f => f.cells.Select(c => Vector2Int.Distance(currentPosition, c.Item1)).Max());
            foreach (var frontier in _frontiers) {
                frontier.UtilityValue = UtilityFunction(frontier, normalizerConstant);
            }
            return _frontiers.OrderByDescending(f => f.UtilityValue).First();

        }

        private void MoveToFrontier(Frontier bestFrontier, Vector2Int currentPosition) {
            // var closestCell = bestFrontier.cells.OrderBy(c => Vector2Int.Distance(c.Item1, currentPosition)).First();
            // var closestCellAsCoarseTile = _map.GetCoarseMap().FromSlamMapCoordinate(closestCell.Item1);
            // var path = _map.GetCoarseMap().GetPath(closestCellAsCoarseTile);
            // if (path == null) {
            //     return;
            // }
            // _path = new LinkedList<Vector2Int>(path);
            var targetCell = bestFrontier.cells.OrderBy(c => Vector2Int.Distance(c.Item1, currentPosition)).First().Item1;
            var targetCellAsCoarseTile = _map.GetCoarseMap().FromSlamMapCoordinate(targetCell);
            var path = _map.GetCoarseMap().GetPathSteps(targetCellAsCoarseTile);
            if (path == null) {
                targetCell = bestFrontier.cells.OrderBy(c => Vector2Int.Distance(c.Item1, currentPosition)).Last().Item1;
                targetCellAsCoarseTile = _map.GetCoarseMap().FromSlamMapCoordinate(targetCell);
                path = _map.GetCoarseMap().GetPathSteps(targetCellAsCoarseTile);
                if (path == null) {
                    targetCell = bestFrontier.cells.OrderBy(c => Vector2Int.Distance(c.Item1, currentPosition)).ToList()[bestFrontier.cells.Count / 2].Item1;
                    targetCellAsCoarseTile = _map.GetCoarseMap().FromSlamMapCoordinate(targetCell);
                    path = _map.GetCoarseMap().GetPathSteps(targetCellAsCoarseTile);
                    if (path == null) {
                        return;
                    }
                }
            }
            _path = new LinkedList<PathStep>(path);
            if (!_path.Any()) {
                _robotTnfStatus = TnfStatus.AwaitNextFrontier;
                return;
            }
            _robotTnfStatus = TnfStatus.AwaitMoving;
            var nextStep = _path.First;
            _path.Remove(nextStep);
            _nextTileInPath = nextStep.Value;
            StartMoving();
        }

        private void DoMovement(LinkedList<PathStep> path) {
            if (_robotController.IsCurrentlyColliding()) {
                _robotTnfStatus = TnfStatus.AwaitNextFrontier;
                _robotController.StopCurrentTask();
                return;
            }

            if (_robotController.GetStatus() == RobotStatus.Idle) {
                if (path != null && path.Any()) {
                    if (_robotTnfStatus != TnfStatus.AwaitRotating) {
                        var nextStep = path.First;
                        _nextTileInPath = nextStep.Value;
                        path.Remove(nextStep);
                    }
                    _robotTnfStatus = TnfStatus.AwaitMoving;
                }
                else {
                    _robotTnfStatus = TnfStatus.AwaitNextFrontier;
                    return;
                }

                StartMoving();
            }
        }

        private void StartMoving() {
            if (_robotController.GetStatus() != RobotStatus.Idle) {
                return;
            }
            var relativePosition = _map.GetCoarseMap().GetTileCenterRelativePosition(_nextTileInPath.End);
            if (relativePosition.Distance < MinimumMoveDistance)
                return;
            if (Mathf.Abs(relativePosition.RelativeAngle) <= AngleDelta) {
                _robotController.Move(relativePosition.Distance);
            }
            else {
                _robotController.Rotate(relativePosition.RelativeAngle);
                _robotTnfStatus = TnfStatus.AwaitRotating;
            }
        }

        private List<Frontier> GetFrontiers() {
            var res = new List<Frontier>();
            var edges = new LinkedList<(Vector2Int, float)>();
            var map = this._map.GetExploredTiles();

            // Find edges
            var mapAsList = map.ToList();
            foreach (var cell in mapAsList) {
                var isEdge = false;
                for (var x = cell.Key.x - 1; x <= cell.Key.x + 1; x++) {
                    for (var y = cell.Key.y - 1; y <= cell.Key.y + 1; y++) {
                        var neighbour = new Vector2Int(x, y);
                        //if (!mapAsList.Exists(t => t.Key == neighbour)) isEdge = true;
                        if (!map.ContainsKey(neighbour)) {
                            isEdge = true;
                        }
                    }
                }
                if (isEdge && cell.Value != SlamMap.SlamTileStatus.Solid) {
                    edges.AddFirst(cell.ToTnfCell());
                }
            }

            // build frontiers by connected cells
            var fCellCount = 0u;
            var eCellCount = edges.Count;
            while (eCellCount > fCellCount) {
                var q = new Queue<(Vector2Int, float)>();
                q.Enqueue(edges.First());

                var frontier = new List<(Vector2Int, float)>();
                while (q.Count > 0) {
                    var next = q.Dequeue();
                    edges.Remove(next);
                    frontier.Add(next);
                    fCellCount++;
                    for (int x = next.Item1.x - 1; x < next.Item1.x + 1; x++) {
                        for (int y = next.Item1.y - 1; y < next.Item1.y + 1; y++) {
                            var position = new Vector2Int(x, y);
                            if (edges.Any(e => e.Item1 == position)) {
                                var newCell = edges.First(c => c.Item1 == position);
                                q.Enqueue(newCell);
                                edges.Remove(newCell);
                            }
                        }
                    }
                }


                res.Add(new Frontier(frontier));
            }

            return res;
        }

        public TnfExplorationAlgorithm(int alpha, int beta) {
            Alpha = alpha;
            Beta = beta;
            _robotTnfStatus = TnfStatus.AwaitNextFrontier;
        }

        public void SetController(Robot2DController controller) {
            _robotController = controller;
            _map = _robotController.GetSlamMap();
        }

        public string GetDebugInfo() {
            return _robotTnfStatus.ToString();
        }


        // FUTURE WORK
        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }
    }
}