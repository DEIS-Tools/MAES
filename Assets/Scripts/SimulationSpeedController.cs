using UnityEngine;
using UnityEngine.UI;

namespace Maes {
    public class SimulationSpeedController : MonoBehaviour {
        public Simulator simulator;
        public Button pauseButton;
        public Button playButton;
        public Button fastForwardButton;
        public Button fastAsPossibleButton;
        public Button stepperButton;

        private void Start() {
            UpdateButtonsUI(simulator.PlayState);
            pauseButton.onClick.AddListener(Pause);
            playButton.onClick.AddListener(Play);
            fastForwardButton.onClick.AddListener(FastForward);
            fastAsPossibleButton.onClick.AddListener(FastAsPossible);
            stepperButton.onClick.AddListener(Step);
        }

        public void UpdateButtonsUI(SimulationPlayState currentState) {
            // Do not change ui for the duration of the step
            if (currentState == SimulationPlayState.Step) return;

            pauseButton.image.color = (currentState == SimulationPlayState.Paused) ? Color.green : Color.white;
            playButton.image.color = (currentState == SimulationPlayState.Play) ? Color.green : Color.white;
            fastForwardButton.image.color =
                (currentState == SimulationPlayState.FastForward) ? Color.green : Color.white;
            fastAsPossibleButton.image.color =
                (currentState == SimulationPlayState.FastAsPossible) ? Color.green : Color.white;
        }

        public void Pause() {
            AttemptSwitchState(SimulationPlayState.Paused);
        }

        public void Play() {
            AttemptSwitchState(SimulationPlayState.Play);
        }

        public void FastForward() {
            AttemptSwitchState(SimulationPlayState.FastForward);
        }

        public void FastAsPossible() {
            AttemptSwitchState(SimulationPlayState.FastAsPossible);
        }

        // Perform a single logic step then stop again
        public void Step() {
            AttemptSwitchState(SimulationPlayState.Step);
        }

        private void AttemptSwitchState(SimulationPlayState newPlayState) {
            var actualState = simulator.AttemptSetPlayState(newPlayState);
            UpdateButtonsUI(actualState);
        }
    }
}