// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using System;
using System.Linq;
using UnityEngine;

namespace Maes.Utilities {
    
    // Represents the 8 directions found on a compass 
    public class CardinalDirection {
        
        public const int CardinalDirectionsCount = 8;
        // Index representing 8 neighbouring tags/tiles
        public static readonly CardinalDirection
            East = new CardinalDirection(0),
            SouthEast = new CardinalDirection(1),
            South = new CardinalDirection(2),
            SouthWest = new CardinalDirection(3),
            West = new CardinalDirection(4),
            NorthWest = new CardinalDirection(5),
            North = new CardinalDirection(6),
            NorthEast = new CardinalDirection(7);

        public enum RelativeDirection {
            // Each relative direction is assign to the corresponding compass offset
            Front = 0, 
            FrontRight = 1, FrontLeft = -1, 
            Left = -2, 
            Right = 2,
            RearRight = 3, RearLeft = -3,
            Rear = 4
        }
        
        private static readonly CardinalDirection[] Directions = 
            {East, SouthEast, South, SouthWest, West, NorthWest, North, NorthEast};
        
        public readonly int Index;
        public readonly Vector2Int Vector;
        
        // Can only be constructed locally. Must be accessed through public static instances
        private CardinalDirection(int index) {
            Index = index;
            Vector = CalculateDirectionVector();
        }

        public CardinalDirection OppositeDirection() => GetDirection((Index + 4) % 8);
        public float DirectionToAngle() => ((8 - Index) % 8) * 45; 
        public bool IsDiagonal() => Index % 2 != 0;

        // Converts the given absolute angle (relative to the x-axis) to the closest corresponding cardinal direction
        public static CardinalDirection DirectionFromDegrees(float degrees) {
            if (degrees < 0f)
                throw new ArgumentException($"Degrees must be above zero, was: {degrees}");

            var offset = (int) (((degrees + 22.5f) % 360) / 45f);
            return Directions[(8 - offset) % 8];
        }

        public static CardinalDirection GetDirection(int index) {
            while (index < 0) index += 8;
            return Directions[index % 8];
        }

        public CardinalDirection Next() {
            return GetDirection(Index + 1);
        }

        public CardinalDirection Previous() {
            return GetDirection(Index - 1);
        }

        private Vector2Int CalculateDirectionVector() {
            var xDir = 0;
            var yDir = 0;
            
            if (Index > 6 || Index < 2) xDir = 1;
            else if (Index < 6 && Index > 2) xDir = -1;
            
            if (Index > 4) yDir = 1;
            else if (Index < 4 && Index > 0) yDir = -1;
            
            return new Vector2Int(xDir, yDir);
        }

        public CardinalDirection GetRelativeDirection(RelativeDirection dir) {
            return GetDirection(this.Index + (int) dir);
        }

        public static CardinalDirection[] AllDirections() => Directions;


        public static CardinalDirection FromVector(Vector2Int vector) {
            return AllDirections().First(dir => dir.Vector == vector);
        }

        public static CardinalDirection VectorToDirection(Vector2 vector)
        {
            var angle = Vector2.SignedAngle(Vector2.right, vector);
            return AngleToDirection(angle);
        }

        public static CardinalDirection AngleToDirection(float angle)
        {
            return FromVector(Vector2Int.RoundToInt(new Vector2(Mathf.Cos(angle * Mathf.Deg2Rad), Mathf.Sin(angle * Mathf.Deg2Rad))));
        }

        public CardinalDirection Counterclockwise()
        {
            return Index switch
            {
                //East
                0 => North,
                //Southeast
                1 => NorthEast,
                //South
                2 => East,
                //Southwest
                3 => SouthEast,
                //West
                4 => South,
                //Northwest
                5 => SouthWest,
                //North
                6 => West,
                //Northeast
                7 => NorthWest,
                _ => new CardinalDirection(-1),
            };
        }

        public static CardinalDirection PerpendicularDirection(Vector2 vector)
        {
            return VectorToDirection(Vector2.Perpendicular(vector));
        }

        public override string ToString()
        {
            return Index switch
            {
                0 => "East",
                1 => "Southeast",
                2 => "South",
                3 => "Southwest",
                4 => "West",
                5 => "Northwest",
                6 => "North",
                7 => "Northeast",
                _ => "",
            };
        }
    }
}