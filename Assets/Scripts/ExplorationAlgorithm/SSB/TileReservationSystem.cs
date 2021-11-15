using UnityEngine;

namespace Dora.ExplorationAlgorithm.SSB {
    public partial class SsbAlgorithm {
        
        public class TileReservationSystem {

            private SsbAlgorithm _algorithm;

            public TileReservationSystem(SsbAlgorithm algorithm) {
                _algorithm = algorithm;
            }

            public void AttemptToReserve(Vector2Int tile) {
                
            }

            public bool IsTileReservedByThisRobot(Vector2Int tile) {
                return false;
            }

            public void ClearReservation() {
                
            }
        }
        
    }
    
}