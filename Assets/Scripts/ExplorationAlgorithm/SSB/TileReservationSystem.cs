#nullable enable
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Maes.ExplorationAlgorithm.SSB {
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

            private readonly SsbAlgorithm _algorithm;
            private readonly Dictionary<Vector2Int, Reservation> _reservations = new Dictionary<Vector2Int, Reservation>();

            public TileReservationSystem(SsbAlgorithm algorithm) {
                _algorithm = algorithm;
            }

            // Creates a reservation locally and returns it. Returns null if the reservation already exists
            private Reservation? ReserveLocally(Vector2Int tile) {
                // Only perform reservation if this tile is not already reserved
                if (_reservations.ContainsKey(tile)) {
                    if (_reservations[tile].ReservingRobot != _algorithm._controller.GetRobotID())
                        throw new Exception("Attempted to reserve a tile that is already reserved by another robot");
                    return null;
                } else {
                    _reservations[tile] = new Reservation(_algorithm.RobotID(), tile, _algorithm._currentTick);
                    return _reservations[tile];
                }
            }
            
            // Saves and broadcasts reservations for the given set of tiles
            public void Reserve(HashSet<Vector2Int> tiles) {
                HashSet<Reservation> newReservations = new HashSet<Reservation>();
                foreach (var tile in tiles) {
                    var newRes = ReserveLocally(tile);
                    if (newRes != null) newReservations.Add(newRes.Value);
                }

                if (newReservations.Count == 0)
                    return; // No new reservations added, so no need to 
                
                // Broadcast reservations to all nearby robots
                _algorithm._controller.Broadcast(new ReservationMessage(new HashSet<Reservation>(
                    tiles.Select(t => new Reservation(_algorithm.RobotID(), t, _algorithm._currentTick)))
                ));
            }
            
            // Returns true if the robot has a confirmed reservation for the given tile
            public bool IsTileReservedByThisRobot(Vector2Int tile) {
                return _reservations.ContainsKey(tile) 
                       && _reservations[tile].ReservingRobot == _algorithm.RobotID()
                       && _algorithm._currentTick != _reservations[tile].StartingTick;
                // The last step avoids counting reservations that were made this tick,
                // to avoid conflicts in cases where to robots reserve the same tile at the same time
            }
            
            // Returns true if another robot has a confirmed reservation for the given tile
            public bool IsTileReservedByOtherRobot(Vector2Int tile) {
                return _reservations.ContainsKey(tile)
                       && _reservations[tile].ReservingRobot != _algorithm.RobotID();
            }

            // Returns the id of the robot that is reserving the given tile
            // Returns null if no robot has reserved the tile
            public int? GetReservingRobot(Vector2Int tile) {
                if (!_reservations.ContainsKey(tile)) return null;
                
                var reservation = _reservations[tile];
                // Ignore reservations made this tick by this robot (as there could potentially be a conflict next tick)
                if (_algorithm.RobotID() == reservation.ReservingRobot && _algorithm._currentTick == reservation.StartingTick)
                    return null;
                
                return _reservations[tile].ReservingRobot;
            }
            
            
            public void RegisterReservationFromOtherRobot(Reservation newRes) {
                // Only register reservation if no previous reservation exists
                // or if the new reservation has a higher robot id
                var tile = newRes.ReservedTile;
                if (!_reservations.ContainsKey(tile) || _reservations[tile].ReservingRobot < newRes.ReservingRobot)
                    _reservations[tile] = newRes;
            }
            
            public void ClearThisRobotsReservationsExcept(Vector2Int exception) {
                var removableReservations = new HashSet<Reservation>(_reservations
                    .Where(r => r.Value.ReservingRobot == _algorithm.RobotID() && !r.Key.Equals(exception))
                    .Select(e => e.Value));

                // Only broadcast if there are any removable tiles
                if (removableReservations.Count == 0)
                    return;
                
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

            public List<Vector2Int> GetTilesReservedByThisRobot() {
                var thisRobot = _algorithm.RobotID();
                return _reservations
                    .Where(res => res.Value.ReservingRobot == thisRobot)
                    .Select(entry => entry.Key)
                    .ToList();
            }

            public HashSet<Vector2Int> GetTilesReservedByOtherRobots() {
                var thisRobot = _algorithm.RobotID();
                return new HashSet<Vector2Int>(_reservations
                    .Where(entry => entry.Value.ReservingRobot != thisRobot)
                    .Select(entry => entry.Key));
            }

            public bool AnyTilesReservedByOtherRobot(HashSet<Vector2Int> tiles) {
                var thisRobot = _algorithm.RobotID();
                foreach (var tile in tiles) {
                    if (_reservations.ContainsKey(tile) && _reservations[tile].ReservingRobot != thisRobot)
                        return true;
                }

                return false;
            }

            public bool AllTilesReservedByThisRobot(HashSet<Vector2Int> tiles) {
                var thisRobot = _algorithm.RobotID();
                foreach (var tile in tiles) {
                    if (!_reservations.ContainsKey(tile) || _reservations[tile].ReservingRobot != thisRobot)
                        return false;
                }
                return true;
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
                
                // Debug.Log($"Robot {algorithm.RobotID()} " +
                //           $"registered {_reservations.Count} reservations from other robots");
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
                // Debug.Log($"Robot {algorithm.RobotID()} " +
                //           $"received and processed request to clear {_reservationsToClear} reservations");
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