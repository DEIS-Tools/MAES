// Copyright 2024 MAES
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
// Contributors: Rasmus Borrisholt Schmidt, Andreas Sebastian Sørensen, Thor Beregaard
// 
// Original repository: https://github.com/Molitany/MAES

using Maes;
using Maes.ExplorationAlgorithm.Minotaur;
using Maes.Robot;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Maes.ExperimentSimulations
{
    public class SceneToExperiments : ExperimentBase
    {
        // Start is called before the first frame update
        void Start()
        {
            var sceneName = SceneManager.GetActiveScene().name;
            var configuration = sceneName.Split("_");

            RunSimulation(configuration[0], configuration[1], configuration[2], configuration[3], 100);
        }
    }
}
