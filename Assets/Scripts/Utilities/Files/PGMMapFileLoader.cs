using System;
using System.IO;
using JetBrains.Annotations;
using Maes.Map.MapGen;
using UnityEngine;

namespace Maes.Utilities.Files {
    public class PgmMapFileLoader {

        [CanBeNull]
        public static int[,] LoadMapFromFileIfPresent(string fileName) {
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

            return null;
        }

        // Reads the image height/width from the meta data and advances the stream pointer to the start of the data
        private static (int, int) ReadWidthAndHeight(StreamReader stream) {
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

        private static int[,] ReadMapDataIntoArray(StreamReader stream, int width, int height) {
            var data = new int[width, height];
            
            for (int y = 0; y < height; y++) {
                //var numberStrings = stream.ReadLine()!.Split(" ");
                string debugString = "";
                for (int x = 0; x < width; x++) {
                    var line = stream.ReadLine()!;
                    var pixelValue = int.Parse(line.Trim());
                    var bitmapValue = pixelValue > 0 ? BitMapTypes.ROOM_TYPE : BitMapTypes.WALL_TYPE;
                    data[x, height - y - 1] = bitmapValue;
                    debugString += " " + bitmapValue;
                }
                
                Debug.Log(debugString);
            }

            return data;
        }
        
        
    }
}