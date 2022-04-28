using System.IO;
using JetBrains.Annotations;
using UnityEngine;

namespace Maes.Utilities.Files {
    public class PgmMapFileLoader {

        [CanBeNull]
        public static bool[,] LoadMapFromFileIfPresent(string fileName) {
            var stream = InputFileLoader.ReadInputFile(fileName);
            if (stream == null) // File does not exist
                return null;
            
            // Read meta data
            var (width, height) = ReadWidthAndHeight(stream);
            var data = new int[width, height];
            
            // Read pixel values 
            
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
        
        
    }
}