using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Maes.Map {
    
    /// This type holds all environment tags deposited by robots in the simulated environment 
    /// This map is represented as a 2D array of tiles to make lookup faster
    public class EnvironmentTaggingMap {
        
        // Each tile in the map is a list of tags that are positioned within the bounds of that tile
        private List<EnvironmentTag>[,] _tagLists;

        private int _widthInTiles, _heightInTiles;
        private Vector2 offset;

        public EnvironmentTaggingMap(SimulationMap<bool> collisionMap) {
            this._widthInTiles = collisionMap.WidthInTiles;
            this._heightInTiles = collisionMap.HeightInTiles;
            this.offset = collisionMap.ScaledOffset;
            _tagLists = new List<EnvironmentTag>[_widthInTiles, _heightInTiles];
            for (int x = 0; x < _widthInTiles; x++) {
                for (int y = 0; y < _heightInTiles; y++) {
                    _tagLists[x, y] = new List<EnvironmentTag>();
                }
            }
        }
        
        public EnvironmentTag AddTag(Vector2 worldPosition, EnvironmentTag tag) {
            var gridCoordinate = ToLocalMapCoordinate(worldPosition);
            _tagLists[gridCoordinate.x, gridCoordinate.y].Add(tag);
            return tag;
        }
        
        public List<EnvironmentTag> GetTagsNear(Vector2 centerWorldPosition, float radius) {
            List<EnvironmentTag> nearbyTags = new List<EnvironmentTag>();
            
            var gridPosition = ToLocalMapCoordinate(centerWorldPosition);
            var maxTileRadius = (int) Math.Ceiling(radius);
            // Find bounding box of cells to check
            int minX = Math.Max(gridPosition.x - maxTileRadius, 0);
            int maxX = Math.Min(gridPosition.x + maxTileRadius, _widthInTiles);
            int minY = Math.Max(gridPosition.y - maxTileRadius, 0);
            int maxY = Math.Min(gridPosition.y + maxTileRadius, _heightInTiles);

            for (int x = minX; x < maxX; x++) {
                for (int y = minY; y < maxY; y++) {
                    // Add all tags from this tile that are within the given radius of the center 
                    nearbyTags.AddRange(_tagLists[x, y]
                        .Where(tag => Vector2.Distance(centerWorldPosition, tag.WorldPosition) <= radius));
                }
            }

            return nearbyTags;
        }
        
        // Takes a world coordinates and removes the offset and scale to translate it to a local map coordinate
        private Vector2Int ToLocalMapCoordinate(Vector2 worldCoordinate) {
            Vector2 localFloatCoordinate = worldCoordinate - offset;
            Vector2Int localCoordinate = new Vector2Int((int) localFloatCoordinate.x, (int) localFloatCoordinate.y);
            if (!IsWithinLocalMapBounds(localCoordinate)) {
                throw new ArgumentException("The given coordinate " + localCoordinate
                                                                    + "(World coordinate:" + worldCoordinate + " )"
                                                                    + " is not within map bounds: {" + _widthInTiles +
                                                                    ", " + _heightInTiles + "}");
            }

            return localCoordinate;
        }

        // Checks that the given coordinate is within the local map bounds
        private bool IsWithinLocalMapBounds(Vector2Int localCoordinates) {
            return localCoordinates.x >= 0 
                   && localCoordinates.x < _widthInTiles 
                   && localCoordinates.y >= 0 
                   && localCoordinates.y < _heightInTiles;
        }

        public interface ITag {
            public void DrawTag(Vector3 position);
        }

    }
}