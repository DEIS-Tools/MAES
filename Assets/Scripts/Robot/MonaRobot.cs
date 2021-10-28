using System;
using Dora.ExplorationAlgorithm;
using UnityEngine;

namespace Dora.Robot
{
    public class MonaRobot: MonoBehaviour, ISimulationUnit
    {
        
        public Transform leftWheelTransform;
        public Transform rightWheelTransform;
        
        public int id = -1;

        // The controller that provides an interface for moving the robot
        public Robot2DController Controller { get; private set; }

        public delegate void OnRobotSelectedDelegate(MonaRobot robot);
        public OnRobotSelectedDelegate OnRobotSelected = (r) => { };

        // The algorithm that controls the logic of the robot
        public IExplorationAlgorithm ExplorationAlgorithm { get; set; }

        private void Awake()
        {
            var rigidBody = GetComponent<Rigidbody2D>();
            Controller = new Robot2DController(rigidBody, transform, leftWheelTransform, rightWheelTransform, this);
        }

        public void LogicUpdate()
        {
            ExplorationAlgorithm.UpdateLogic();
            Controller.UpdateLogic();
        }

        public void PhysicsUpdate()
        {
            Controller.UpdateMotorPhysics();
        }

        private void OnCollisionEnter2D(Collision2D other)
        {
            Controller.NotifyCollided();
        }

        private void OnCollisionExit2D(Collision2D other)
        {
            // TODO? Check that all collisions have exited before calling collision exit on controller
            Controller.NotifyCollisionExit();
        }

        public object SaveState()
        {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }
        
        public void OnMouseDown()
        {
            CameraController.SingletonInstance.movementTransform = transform;
            OnRobotSelected(this);
        }

    }
}