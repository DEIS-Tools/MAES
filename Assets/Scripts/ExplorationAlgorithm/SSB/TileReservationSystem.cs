#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using Dora.Robot;
using UnityEngine;

namespace Dora.ExplorationAlgorithm.SSB {
    public partial class SsbAlgorithm {

        public readonly struct Reservation {
            public readonly int ReservingRobot;
            public readonly Vector2Int ReservedTile;
            public readonly int StartingTick;

            public Reservation(int reservingRobot, Vector2Int reservedTile, int startingTick) {
                ReservingRobot = reservingRobot;
                ReservedTile = reservedTile;
                StartingTick = startingTick;
            }
        }
        
        public class TileReservationSystem {

            private SsbAlgorithm _algorithm;
            private Dictionary<Vector2Int, Reservation> _reservations;

            public TileReservationSystem(SsbAlgorithm algorithm) {
                _algorithm = algorithm;
            }

            public void Reserve(Vector2Int tile) {
                // Only perform reservation if this tile is not already reserved
                if (_reservations.ContainsKey(tile)) {
                    if (_reservations[tile].ReservingRobot != _algorithm._controller.GetRobotID())
                        throw new Exception("Attempted to reserve a tile that is already reserved by another robot");
                } else {
                    _reservations[tile] = new Reservation(_algorithm.RobotID(), tile, _algorithm._currentTick);
                }
            }
            
            public void Reserve(List<Vector2Int> tiles) {
                foreach (var tile in tiles) 
                    Reserve(tile);
            }

            public bool IsTileReservedByThisRobot(Vector2Int tile) {
                return _reservations.ContainsKey(tile) 
                       && _reservations[tile].ReservingRobot == _algorithm.RobotID()
                       && _algorithm._currentTick != _reservations[tile].StartingTick;
                // The last step avoids counting reservations that were made this tick,
                // to avoid conflicts in cases where to robots reserve the same tile at the same time
            }

            // Returns the id of the robot that is reserving the given tile
            // Returns null if no robot has reserved the tile
            public int? GetReservingRobot(Vector2Int tile) {
                if (_reservations.ContainsKey(tile)) {
                    var reservation = _reservations[tile];
                    // Ignore reservations made this tick by this robot (as there may be a conflict next tick)
                    if (_algorithm.RobotID() == reservation.ReservingRobot && _algorithm._currentTick == reservation.StartingTick)
                        return null;
                    return _reservations[tile].ReservingRobot;
                }
                    
                return null;
            }
            
            

            public void RegisterReservationFromOtherRobot(Reservation newRes) {
                // Only register reservation if no previous reservation exists
                // or if the new reservation has a higher robot id
                var tile = newRes.ReservedTile;
                if (!_reservations.ContainsKey(tile) || _reservations[tile].ReservingRobot > newRes.ReservingRobot)
                    _reservations[tile] = newRes;
            }
            
            public void ClearThisRobotsReservationsExcept(Vector2Int exception) {
                var removableReservations = new HashSet<Reservation>(_reservations
                    .Where(r => r.Value.ReservingRobot == _algorithm.RobotID() && !r.Key.Equals(exception))
                    .Select(e => e.Value));

                // Remove from local reservations list
                foreach (var reservation in removableReservations) 
                    _reservations.Remove(reservation.ReservedTile);
                
                // Broadcast removal to other robots
                _algorithm._controller.Broadcast(new ReservationClearingMessage(removableReservations));
            }

            public void ClearReservation(Reservation reservation) {
                _reservations.Remove(reservation.ReservedTile);
            }

            public void ClearReservations(HashSet<Reservation> reservationsToClear) {
                foreach (var reservation in reservationsToClear) 
                    _reservations.Remove(reservation.ReservedTile);
            }
        }

        public class ReservationMessage : ISsbBroadcastMessage {

            private HashSet<Reservation> _reservations;

            public ReservationMessage(HashSet<Reservation> reservations) {
                _reservations = reservations;
            }

            public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm) {
                foreach (var reservation in _reservations) 
                    algorithm._reservationSystem.RegisterReservationFromOtherRobot(reservation);
                
                return null;
            }

            public ISsbBroadcastMessage? Combine(ISsbBroadcastMessage other, SsbAlgorithm algorithm) {
                if (other is ReservationMessage reservationMsg) {
                    this._reservations.UnionWith(reservationMsg._reservations);
                    return this;
                }
                
                return null;
            }
        }
        
        public class ReservationClearingMessage : ISsbBroadcastMessage {

            private HashSet<Reservation> _reservationsToClear;

            public ReservationClearingMessage(HashSet<Reservation> reservationsToClear) {
                _reservationsToClear = reservationsToClear;
            }

            public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm) {
                algorithm._reservationSystem.ClearReservations(_reservationsToClear);
                
                return null;
            }

            public ISsbBroadcastMessage? Combine(ISsbBroadcastMessage other, SsbAlgorithm algorithm) {
                if (other is ReservationClearingMessage clearingMsg) {
                    this._reservationsToClear.UnionWith(clearingMsg._reservationsToClear);
                    return this;
                }
                
                return null;
            }
        }
        
    }
    
}