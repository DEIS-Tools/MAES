using System;
using Dora.ExplorationAlgorithm;
using Dora.Robot;
using JetBrains.Annotations;
using UnityEngine;
using static Dora.Robot.SlamMap;

namespace Dora.MapGeneration.PathFinding {
    
    // This represents a low-resolution map where the robot can comfortably fit inside a single cell
    public class CoarseGrainedMap {
        
        private SlamMap _slamMap;
        private object[,] _objects;
        private int _width, _height;

        public CoarseGrainedMap(SlamMap slamMap, int width, int height) {
            _slamMap = slamMap;
            _width = width;
            _height = height;
            _objects = new object[width, height];
        }

        public object GetData(Vector2Int localCoordinate) {
            AssertWithinBounds(localCoordinate);
            return _objects[localCoordinate.x, localCoordinate.y];
        }
        
        public void SetData(Vector2Int localCoordinate, object data) {
            AssertWithinBounds(localCoordinate);
            _objects[localCoordinate.x, localCoordinate.y] = data;
        }

        private void AssertWithinBounds(Vector2Int coordinate) {
            var withinBounds= coordinate.x >= 0 && coordinate.x < _width && coordinate.y >= 0 && coordinate.y < _height;
            if (!withinBounds)
                throw new ArgumentException($"Given coordinate is out of bounds {coordinate} ({_width}, {_height})");
        }

        // Returns the status of the given tile (Solid, Open or Unseen)
        public SlamTileStatus GetTileStatus(Vector2Int localCoordinate) {
            var slamCoord = ToSlamMapCoordinate(localCoordinate);

            var status = _slamMap.GetStatusOfTile(slamCoord);
            status = AggregateStatus(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right));
            status = AggregateStatus(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.up));
            status = AggregateStatus(status, _slamMap.GetStatusOfTile(slamCoord + Vector2Int.right + Vector2Int.up));
            return status;
        }

        // Combines two SlamTileStatus in a 'pessimistic' fashion.
        // If any status is solid both are consider solid. If any status is unseen both are considered unseen 
        private SlamTileStatus AggregateStatus(SlamTileStatus status1, SlamTileStatus status2) {
            if (status1 == SlamTileStatus.Solid || status2 == SlamTileStatus.Solid)
                return SlamTileStatus.Solid;
            if (status1 == SlamTileStatus.Unseen || status2 == SlamTileStatus.Unseen)
                return SlamTileStatus.Unseen;
            return SlamTileStatus.Open;
        }

        
        // Converts the given Slam map coordinate to a local coordinate
        // The Slam map has twice as many tiles in each direction
        public Vector2Int FromSlamMapCoordinate(Vector2Int slamCoord) {
            return slamCoord / 2;
        }

        // Converts the given 
        public Vector2Int ToSlamMapCoordinate(Vector2Int localCoordinate) {
            return localCoordinate * 2;
        }
        
        
    }
    
}