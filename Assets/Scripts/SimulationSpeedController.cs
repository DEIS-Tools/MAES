using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace Dora
{
    public class SimulationSpeedController : MonoBehaviour
    {
        
        public Simulator simulator;
        public Button pauseButton;
        public Button playButton;
        public Button fastForwardButton;
        public Button fastAsPossibleButton;

        private void Start()
        {
            UpdateButtonsUI(simulator.PlayState);
        }

        private void UpdateButtonsUI(SimulationPlayState currentState)
        {
            pauseButton.image.color = (currentState == SimulationPlayState.Paused)? Color.green : Color.white;
            playButton.image.color = (currentState == SimulationPlayState.Play)? Color.green : Color.white;
            fastForwardButton.image.color = (currentState == SimulationPlayState.FastForward)? Color.green : Color.white;
            fastAsPossibleButton.image.color = (currentState == SimulationPlayState.FastAsPossible)? Color.green : Color.white;
        }

        public void Pause()
        {
            AttemptSwitchState(SimulationPlayState.Paused);
        }
        
        public void Play()
        {
            AttemptSwitchState(SimulationPlayState.Play);
        }

        public void FastForward()
        {
            AttemptSwitchState(SimulationPlayState.FastForward);
        }

        public void FastAsPossible()
        {
            AttemptSwitchState(SimulationPlayState.FastAsPossible);
        }

        private void AttemptSwitchState(SimulationPlayState newPlayState)
        {
            var actualState = simulator.AttemptSetPlayState(newPlayState);
            UpdateButtonsUI(actualState);
        }
    }
}
