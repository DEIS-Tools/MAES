using Maes.UI;
using UnityEngine;

namespace Maes {
    public class Simulator {

        private static Simulator _instance = null;
        private GameObject _maesGameObject;
        private SimulationManager _simulationManager;

        private Simulator() {
            // Initialize the simulator by loading the prefab from the resources and then instantiating the prefab
            var prefab = Resources.Load("MAES", typeof(GameObject)) as GameObject;
            _maesGameObject = Object.Instantiate(prefab);
            _simulationManager = _maesGameObject.GetComponentInChildren<SimulationManager>();
        }
        
        public static Simulator GetInstance() {
            return _instance ??= new Simulator();
        }

        public void StartSimulation() {
            _simulationManager.AttemptSetPlayState(SimulationPlayState.Play);
        }
        
        

    }
}