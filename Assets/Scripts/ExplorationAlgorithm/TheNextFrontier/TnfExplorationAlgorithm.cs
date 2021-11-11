using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using Dora.Robot;
using Dora.Utilities;
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

        private IRobotController _robotController;
        private int Alpha { get; }
        private int Beta { get; }

        private List<Frontier> _frontiers;


        private readonly struct Frontier {
            public readonly List<(Vector2Int, float)> cells;

            public Frontier(List<(Vector2Int, float)> cells) {
                this.cells = cells;
            }
        }
        

        private void UpdateFrontiers() {
            _frontiers = new List<Frontier>();

            var f = new List<(Vector2Int, float)> {(new Vector2Int(1, 1), 0.5f)};
            _frontiers.Add(new Frontier(f));
            var s = _frontiers[0].cells[0].Item1;
        }

        private float InformationPotential((Vector2Int, float) cell, List<(Vector2Int, float)> neighbours) {
            return Gaussian.Gauss(cell.Item2) + neighbours.Sum(neighbour => Gaussian.Gauss(neighbour.Item2));
        }

        private float InformationFactor(Frontier frontier) {
            return frontier.cells.Sum(c => InformationPotential(c, GetNeighbours(c, frontier)));
        }

        private List<(Vector2Int, float)> GetNeighbours((Vector2Int, float) cell, Frontier frontier) {
            var res = new List<(Vector2Int, float)>();

            for (var x = cell.Item1.x - 1; x < cell.Item1.x + 1; x++) {
                for (var y = cell.Item1.y - 1; y < cell.Item1.y +1; y++) {
                    if (x == cell.Item1.x && y == cell.Item1.y) {
                        continue;
                    }
                    //TODO: Make sure cell is findable...
                    var c = frontier.cells.Find(c => c.Item1.x == x && c.Item1.y == y);
                    res.Add(c);
                }
            }
            
            return res;
        }

        private float DistanceFactor(Frontier frontier) {
            //TODO: The actual implementation
            // Figure out what the normalization means...
            var pos = new Vector2Int(0, 0);
            var dist = Vector2Int.Distance(pos, frontier.cells[0].Item1);
            return dist;
        }

        public void UpdateLogic() {
            if (_robotController.GetStatus() == RobotStatus.Idle) {
                _robotController.Move(5);
            }

            var fellows = _robotController.SenseNearbyRobots();
            if (fellows.Count > 0) {
                //TODO: Calculate coordination factor
            }

            // Get frontiers

            // For each frontier:

            // Utility(frontier) = Information(frontier) + Distance(frontier) - Coordination(frontier)

            // Information(frontier)
            // - For each cell in frontier.cells:
            // - - Sum += InformationPotential(cell)
            // 
            // InformationPotential(cell)
            // - sum = GaussianFValue(cell)
            // - For each c in cell.neighbours
            // - - sum += GaussianFValue(c)

            // Distance(frontier)
            // - Pow(Wavefront(frontier), _alpha - 1) * (1 - Pow(Wavefront(frontier), _beta -1))
            //
            // Wavefront(frontier)
            // - For each cell in frontier.cells
            // - - EuclidDistance(this.position, cell)
            // - - Normalize (See notes)

            // Coordination(frontier)
            // - For each robot in visibleRobots
            // - - For cell in frontier.cells
            // - - - EuclidDistance(robot.position, cell)
            // - - - Normalize?
        }

        public TnfExplorationAlgorithm(int alpha, int beta) {
            Alpha = alpha;
            Beta = beta;
        }

        public void SetController(Robot2DController controller) {
            _robotController = controller;
        }

        public string GetDebugInfo() {
            throw new System.NotImplementedException();
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