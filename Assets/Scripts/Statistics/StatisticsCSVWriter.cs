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

using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;
using static Maes.Statistics.ExplorationTracker;

namespace Maes.Statistics
{
    internal class StatisticsCSVWriter
    {
        private readonly Simulation _simulation;
        private readonly List<SnapShot<float>> _coverSnapShots;
        private readonly List<SnapShot<float>> _exploreSnapShots;
        private readonly List<SnapShot<float>> _distanceSnapShots;
        private readonly Dictionary<int, SnapShot<bool>> _allAgentsConnectedSnapShots;
        public readonly Dictionary<int, SnapShot<float>> _biggestClusterPercentageSnapShots;
        private string path;


        public StatisticsCSVWriter(Simulation simulation, string fileNameWithoutExtension)
        {
            _coverSnapShots = simulation.ExplorationTracker._coverSnapshots;
            _exploreSnapShots = simulation.ExplorationTracker._exploreSnapshots;
            _distanceSnapShots = simulation.ExplorationTracker._distanceSnapshots;
            _allAgentsConnectedSnapShots = simulation._communicationManager.CommunicationTracker.InterconnectionSnapShot;
            _biggestClusterPercentageSnapShots = simulation._communicationManager.CommunicationTracker.BiggestClusterPercentageSnapshots;

            _simulation = simulation;
            var resultForFileName =
                $"e{(int)_exploreSnapShots[_exploreSnapShots.Count - 1].Value}-c{(int)_coverSnapShots[_coverSnapShots.Count - 1].Value}";
            path = GlobalSettings.StatisticsOutPutPath + fileNameWithoutExtension + "_" + resultForFileName + ".csv";
        }

        public void CreateCSVFile(string separator)
        {
            using var csv = new StreamWriter(path);
            csv.WriteLine("Tick,Covered,Explored,Agents Interconnected, Biggest Cluster %");
            for (int i = 0; i < _coverSnapShots.Count; i++)
            {
                var tick = _coverSnapShots[i].Tick;
                var coverage = "" + _coverSnapShots[i].Value;
                var explore = "" + _exploreSnapShots[i].Value;
                var distance = "" + _distanceSnapShots[i].Value;
                StringBuilder line = new StringBuilder();
                line.Append(
                    $"{"" + tick}{separator}{coverage}{separator}{explore}{separator}{distance}{separator}");
                if (_allAgentsConnectedSnapShots.ContainsKey(tick))
                {
                    var allAgentsInterconnectedString = _allAgentsConnectedSnapShots[tick].Value ? "" + 1 : "" + 0;
                    line.Append($"{allAgentsInterconnectedString}");
                }

                line.Append($"{separator}");
                if (_biggestClusterPercentageSnapShots.ContainsKey(tick))
                {
                    line.Append($"{_biggestClusterPercentageSnapShots[tick].Value}");
                }

                csv.WriteLine(line.ToString());
            }
            Debug.Log($"Writing statistics to path: {path}");
        }
    }
}