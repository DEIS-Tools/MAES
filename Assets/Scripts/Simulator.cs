using UnityEngine;

namespace Maes {
    public class Simulator {

        private static Simulator _instance = null;
        private GameObject _maesGameObject;

        private Simulator() {
            // Initialize the simulator by loading the prefab from the resources and then instantiating the prefb
            var prefab = Resources.Load("MAES", typeof(GameObject)) as GameObject;
            _maesGameObject = Object.Instantiate(prefab);
        }
        
        public static Simulator GetInstance() {
            return _instance ??= new Simulator();
        }
        
        

    }
}