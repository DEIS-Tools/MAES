using Maes;
using Maes.ExplorationAlgorithm.Minotaur;
using Maes.Robot;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Maes.ExperimentSimulations
{
    public class MinotaurExperiments : ExperimentBase
    {
        // Start is called before the first frame update
        void Start()
        {
            var sceneName = SceneManager.GetActiveScene().name;
            var configuration = sceneName.Split("_");

            RunSimulation(configuration[0], configuration[1], configuration[2]);
        }
    }
}
