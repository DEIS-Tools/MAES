using Maes.Map.MapGen;
using Maes.Robot;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Maes.ExplorationAlgorithm.Minotaur;
using Maes.ExperimentSimulations;

namespace Maes
{
    public class MinotaurDemo : ExperimentBase
    {
        void Start()
        {
            RunSimulation("building", "minotaur", "Material", "100", 100, 775411, 9);
        }
    }
}