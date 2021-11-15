using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using static Dora.Statistics.ExplorationTracker;

namespace Dora.Statistics {
    public class StatisticsCSVWriter {
        private readonly List<SnapShot<float>> _coverSnapShots;
        private readonly List<SnapShot<float>> _exploreSnapshots;
        private string path;
        

        public StatisticsCSVWriter(string fileName, List<SnapShot<float>> coverSnapShots, List<SnapShot<float>> exploreSnapshots) {
            _coverSnapShots = coverSnapShots;
            _exploreSnapshots = exploreSnapshots;
            path = Environment.GetFolderPath(Environment.SpecialFolder.Desktop) + Path.DirectorySeparatorChar + fileName + ".csv";
        }

        public void CreateCSVFile(string separator) {
            var csv = new StringBuilder();

            
            
            csv.AppendLine("tick,covered,explored");
            for (int i = 0; i < _coverSnapShots.Count; i++) {
                var tick = "" + _coverSnapShots[i].Tick;
                var coverage = "" + _coverSnapShots[i].Value;
                var explore = "" +_exploreSnapshots[i].Value;
                csv.AppendLine($"{tick}{separator}{coverage}{separator}{explore}");
            }
            
            File.WriteAllText(path, csv.ToString());
        }
    }
}