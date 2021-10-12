using System;
using Dora.Robot.ExplorationAlgorithm;
using UnityEngine;

namespace Dora.Robot
{
    public class Robot: MonoBehaviour, ISimulationUnit
    {
        
        public Transform leftWheelTransform;
        public Transform rightWheelTransform;

        // The controller that provides an interface for moving the robot
        public Robot2DController movementController { get; private set; }

        // The algorithm that controls the logic of the robot
        public IExplorationAlgorithm ExplorationAlgorithm { get; set; }

        private void Start()
        {
            var rigidBody = GetComponent<Rigidbody2D>();
            movementController = new Robot2DController(rigidBody, transform, leftWheelTransform, rightWheelTransform);
        }

        public void LogicUpdate(SimulationConfiguration config)
        {
            ExplorationAlgorithm.UpdateLogic(config);
            movementController.UpdateLogic(config);
        }

        public void PhysicsUpdate(SimulationConfiguration config)
        {
            movementController.UpdateMotorPhysics(config);
        }

        private void OnCollisionEnter2D(Collision2D other)
        {
            movementController.NotifyCollided();
        }

        private void OnCollisionExit2D(Collision2D other)
        {
            // TODO? Check that all collisions have exited before calling collision exit on controller
            movementController.NotifyCollisionExit();
        }

        public object SaveState()
        {
            throw new System.NotImplementedException();
        }

        public void RestoreState(object stateInfo)
        {
            throw new System.NotImplementedException();
        }

    }
}