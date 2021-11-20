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
            AwaitCollisionMitigation,
            OutOfFrontiers
        }

        private IRobotController _robotController;
        private TnfStatus _robotTnfStatus;
        private SlamAlgorithmInterface _map;
        private int Alpha { get; }
        private int Beta { get; }

        private List<Frontier> _frontiers;
        private static int _tagId = 0;

        private Vector2Int _robotPos = new Vector2Int(0, 0);
        private LinkedList<PathStep> _path;
        private PathStep _nextTileInPath;
        private const float AngleDelta = 1f;
        private const float MinimumMoveDistance = 0.1f;

        private int _logicTicksSinceLastCommunication = 0;


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
            //return frontier.cells.Sum(c => InformationPotential(c, GetNeighbouringCells(c))) / _map.GetExploredTiles().Count;
            return frontier.cells.Sum(c => InformationPotential(c, GetNeighbouringCells(c))) / _frontiers.Count;
        }

        private List<(Vector2Int, float)> GetNeighbouringCells((Vector2Int, float) cell) {
            var res = new List<(Vector2Int, float)>();

            for (var x = cell.Item1.x - 1; x <= cell.Item1.x + 1; x++) {
                for (var y = cell.Item1.y - 1; y <= cell.Item1.y + 1; y++) {
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

            _robotPos = _map.GetCurrentPositionSlamTile();
            _logicTicksSinceLastCommunication++;
            _logicTicksSinceLastCommunication %= 15;
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
                    CollisionMitigation();
                    break;
                case TnfStatus.OutOfFrontiers:
                    break;
            }
        }

        private void CollisionMitigation() {
            _robotTnfStatus = TnfStatus.AwaitNextFrontier;
            if (_robotController.GetStatus() != RobotStatus.Idle) return;
            _robotController.Move(.5f, true);
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
            if (_frontiers != null) {
                foreach (var frontier in _frontiers) {
                    UnVisualizeFrontier(frontier);
                }
            }
            _frontiers = GetFrontiers();

            // For each frontier:
            if (_frontiers.Count <= 0) {
                return null;
            }
            var normalizerConstant = _frontiers.Max(f => f.cells.Select(c => Vector2Int.Distance(currentPosition, c.Item1)).Max());
            foreach (var frontier in _frontiers) {
                frontier.UtilityValue = UtilityFunction(frontier, normalizerConstant);
                VisualizeFrontier(frontier);
            }
            return _frontiers.OrderByDescending(f => f.UtilityValue).First();

        }

        private void MoveToFrontier(Frontier bestFrontier, Vector2Int currentPosition) {
            VisualizeNextFrontier(bestFrontier);
            var targetCell = GetFrontierMoveTarget(bestFrontier);
            var targetCellAsCoarseTile = _map.GetCoarseMap().FromSlamMapCoordinate(targetCell);
            var path = _map.GetCoarseMap().GetPathSteps(targetCellAsCoarseTile);
            if (path == null) {
                _frontiers.Remove(bestFrontier);
                if (!_frontiers.Any()) {
                    _robotTnfStatus = TnfStatus.OutOfFrontiers;
                    return;
                }
                var newFrontier = _frontiers.OrderByDescending(f => f.UtilityValue).First();
                MoveToFrontier(newFrontier, currentPosition);
                return;
            }

            var comManager = CommunicationManager.singletonInstance;
            comManager.DepositTag(new TnfTag(_tagId++, true), Vector2.Scale(targetCell, new Vector2(.5f, .5f)));
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

        private void VisualizeNextFrontier(Frontier frontier) {
            var comManager = CommunicationManager.singletonInstance;
            foreach (var cell in frontier.cells) {
                var pos = Vector2.Scale(cell.Item1, new Vector2(.5f, .5f));
                comManager.DepositTag(new TnfTag(_tagId++, Color.magenta), pos);
            }
        }

        private Vector2Int GetFrontierMoveTarget(Frontier frontier) {
            var count = frontier.cells.Count;
            var first = 0;
            var middle = count / 2;
            var last = count - 1;
            var dest = new Vector2Int(
                (frontier.cells[first].Item1.x + frontier.cells[middle].Item1.x + frontier.cells[last].Item1.x) / 3,
                (frontier.cells[first].Item1.y + frontier.cells[middle].Item1.y + frontier.cells[last].Item1.y) / 3
            );
            return Vector2Int.Distance(_robotPos, dest) < 2f ? frontier.cells[middle].Item1 : dest;
        }

        private void VisualizeFrontier(Frontier frontier) {
            var comManager = CommunicationManager.singletonInstance;
            foreach (var cell in frontier.cells) {
                var pos = Vector2.Scale(cell.Item1, new Vector2(.5f, .5f));
                comManager.DepositTag(new TnfTag(_tagId++, false), pos);
            }
        }

        private void UnVisualizeFrontier(Frontier frontier) {
            var comManager = CommunicationManager.singletonInstance;
            foreach (var cell in frontier.cells) {
                var pos = Vector2.Scale(cell.Item1, new Vector2(.5f, .5f));
                comManager.RemoveTagAt(pos);
            }
        }

        private void DoMovement(LinkedList<PathStep> path) {
            if (_robotController.IsCurrentlyColliding()) {
                _robotTnfStatus = TnfStatus.AwaitCollisionMitigation;
                _robotController.StopCurrentTask();
                return;
            }

            if (_robotController.GetStatus() == RobotStatus.Idle) {
                if (path != null && path.Count > 0) {
                    if (_robotTnfStatus != TnfStatus.AwaitRotating) {
                        var nextStep = path.First;
                        _nextTileInPath = nextStep.Value;
                        path.Remove(nextStep);
                    }
                    _robotTnfStatus = TnfStatus.AwaitMoving;
                }
                else {
                    StartMoving();
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
                    for (int x = next.Item1.x - 1; x <= next.Item1.x + 1; x++) {
                        for (int y = next.Item1.y - 1; y <= next.Item1.y + 1; y++) {
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