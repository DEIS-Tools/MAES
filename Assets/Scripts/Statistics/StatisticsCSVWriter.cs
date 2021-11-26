using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using static Dora.Statistics.ExplorationTracker;

namespace Dora.Statistics {
    public class StatisticsCSVWriter {
        private readonly Simulation _simulation;
        private readonly List<SnapShot<float>> _coverSnapShots;
        private readonly List<SnapShot<float>> _exploreSnapshots;
        private string path;
        

        public StatisticsCSVWriter(Simulation simulation, string fileNameWithoutExtension) {
            _coverSnapShots = simulation.ExplorationTracker._coverSnapshots;
            _exploreSnapshots = simulation.ExplorationTracker._exploreSnapshots;
            _simulation = simulation;
            var resultForFileName =
                $"e{(int)_exploreSnapshots[_exploreSnapshots.Count - 1].Value}-c{(int)_coverSnapShots[_coverSnapShots.Count - 1].Value}";
            Directory.CreateDirectory(GlobalSettings.StatisticsOutPutPath);
            path = GlobalSettings.StatisticsOutPutPath + fileNameWithoutExtension + "-" + resultForFileName + ".csv";
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