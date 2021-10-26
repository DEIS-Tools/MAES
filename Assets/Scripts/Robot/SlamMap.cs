using System;
using System.Collections.Generic;
using UnityEngine;

namespace Dora.Robot
{
    public class SlamMap
    {

        // Size of a tile in world space
        private readonly float _tileSize;
        
        private readonly int _widthInTiles, _heightInTiles;

        // Represents the current approximate position of the given robot
        public Vector2 ApproximatePosition { get; private set; }

        // Added the template
        private class SlamTile
        {
            
        }
        

        // Synchronizes the given slam maps to create a new one
        public static SlamMap Combine(out List<SlamMap> maps)
        {
            throw new NotImplementedException();
        }
        

    }
}