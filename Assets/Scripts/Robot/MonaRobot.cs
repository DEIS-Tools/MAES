using System.Collections.Generic;
using Maes.ExplorationAlgorithm;
using Maes.UI;
using UnityEngine;

namespace Maes.Robot {
    public class MonaRobot : MonoBehaviour, ISimulationUnit {
        public Transform leftWheelTransform;
        public Transform rightWheelTransform;
        public Outline outLine;

        public int id = -1;

        // The controller that provides an interface for moving the robot
        public Robot2DController Controller { get; private set; }

        public delegate void OnRobotSelectedDelegate(MonaRobot robot);

        public OnRobotSelectedDelegate OnRobotSelected = (r) => { };

        // The algorithm that controls the logic of the robot
        public IExplorationAlgorithm ExplorationAlgorithm { get; set; }

        public List<GameObject> _collidingGameObjects = new List<GameObject>();

        private void Awake() {
            var rigidBody = GetComponent<Rigidbody2D>();
            Controller = new Robot2DController(rigidBody, transform, leftWheelTransform, rightWheelTransform, this);
        }

        public void LogicUpdate() {
            ExplorationAlgorithm.UpdateLogic();
            Controller.UpdateLogic();
        }

        public void PhysicsUpdate() {
            Controller.UpdateMotorPhysics();
        }

        private void OnCollisionEnter2D(Collision2D other) {
            if (!_collidingGameObjects.Contains(other.gameObject))
                _collidingGameObjects.Add(other.gameObject);

            Controller.NotifyCollided();
        }

        private void OnCollisionExit2D(Collision2D other) {
            _collidingGameObjects.Remove(other.gameObject);

            if (_collidingGameObjects.Count == 0)
                Controller.NotifyCollisionExit();
        }

        public object SaveState() {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo) {
            throw new System.NotImplementedException();
        }

        public void OnMouseDown() {
            CameraController.singletonInstance.movementTransform = transform;
            OnRobotSelected(this);
        }
    }
}