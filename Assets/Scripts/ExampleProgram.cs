using System;
using UnityEngine;

namespace Maes {
    internal class ExampleProgram : MonoBehaviour {
        private void Start() {
            var simulator = Simulator.GetInstance();
            simulator.DefaultStart();
            //simulator.StartSimulation();
        }
    }
}