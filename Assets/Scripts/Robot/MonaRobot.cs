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

using System;
using System.Collections.Generic;
using Maes.ExplorationAlgorithm;
using Maes.UI;
using UnityEditor;
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

        public void OnMouseDown() {
            CameraController.singletonInstance.movementTransform = transform;
            OnRobotSelected(this);
        }

        public void OnMouseEnter() {
            Tooltip.ShowTooltip_Static($"robot{id}");
        }

        public void OnMouseExit() {
            Tooltip.HideTooltip_Static();
        }

        public GameObject ClaimTag() {
            var envTagHolder = GameObject.Find("EnvTagHolder");
            var gameObj = Instantiate(Resources.Load<GameObject>("TagPost"), envTagHolder.transform);
            gameObj.transform.position = this.transform.position + new Vector3(0,0,-0.1f);
            gameObj.SetActive(false);
            gameObj.name = $"robot{0}-" + gameObj.name;
            return gameObj;
        }
        private void OnDrawGizmos()
        {
            if (_collidingGameObjects == null) return;
            foreach (var circle in Controller.DebugCircle)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(circle.Item1, circle.Item2);
            }
        }
    }
}