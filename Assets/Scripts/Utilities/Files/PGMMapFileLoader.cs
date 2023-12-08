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

using System.IO;
using JetBrains.Annotations;
using Maes.Map.MapGen;

namespace Maes.Utilities.Files
{
    internal class PgmMapFileLoader {

        [CanBeNull]
        public static Tile[,] LoadMapFromFileIfPresent(string fileName) {
            var stream = InputFileLoader.ReadInputFile(fileName);
            if (stream == null) // File does not exist
                return null;

            try {
                // Read meta data and initialize map array based on found size
                var (width, height) = ReadWidthAndHeight(stream);

                // Read pixel values into array
                var data = ReadMapDataIntoArray(stream, width, height);

                return data;
            }
            finally {
                stream.Close();
            }
        }

        // Reads the image height/width from the meta data and advances the stream pointer to the start of the data
        private static (int, int) ReadWidthAndHeight(TextReader stream) {
            // Skip first two meta data lines
            stream.ReadLine(); 
            stream.ReadLine();
            
            // Read resolution line
            var resolution = stream.ReadLine()!.Split(" ");
            // Skip the line integer indicating the maximum value - We assume it to be 255
            stream.ReadLine();
            // The stream pointer has now been advanced to the start of the data
            return (int.Parse(resolution[0]), int.Parse(resolution[1]));
        }

        private static Tile[,] ReadMapDataIntoArray(TextReader stream, int width, int height) {
            var data = new Tile[width, height];
            
            for (var y = 0; y < height; y++) {
                //var numberStrings = stream.ReadLine()!.Split(" ");
                //var debugString = "";
                for (var x = 0; x < width; x++) {
                    var line = stream.ReadLine()!;
                    var pixelValue = int.Parse(line.Trim());
                    var bitmapValue = pixelValue < 125 ? TileType.Wall : TileType.Room;
                    data[x, height - y - 1] = new Tile(bitmapValue);
                    //debugString += " " + bitmapValue;
                }
                
                // Debug.Log(debugString);
            }

            return data;
        }
        
        
    }
}