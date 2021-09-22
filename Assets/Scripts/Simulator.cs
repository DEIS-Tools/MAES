using System;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

namespace Dora
{
    public class Simulator : MonoBehaviour
    {
        public SimulationConfiguration SimConfig = new SimulationConfiguration(); // Should this be a struct?!
        private SimulationPlayState _playState = SimulationPlayState.Paused;

        public Text Text;
        private int _physicsTickCount = 0;
        private int _robotLogicTickCount = 0;
        private int _physicsTicksSinceUpdate = 0;
        private int _simulatedTimeMillis = 0;

        public SimulationPlayState PlayState { get; }

        public SimulationPlayState AttemptSetPlayState(SimulationPlayState targetState)
        {
            if (targetState == _playState) return _playState;
            // TODO: Check if possible to change (For example, not possible if no map is generated)
            _playState = targetState;
            // Reset next update time when changing play mode to avoid skipping ahead
            _nextUpdateTimeMillis = Utils.CurrentTimeMillis();
            return _playState;
        }

        private void Start()
        {
            Text.text = "Start";
        }

        // Timing variables for controlling the simulation in a manner that is decoupled from Unity's update system
        private long _nextUpdateTimeMillis = 0;

        private void FixedUpdate()
        {
            if (_playState == SimulationPlayState.Paused) return;

            long startTimeMillis = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            int millisPerFixedUpdate = (int) (1000f * Time.fixedDeltaTime);
            // Subtract 5 milliseconds to allow for other procedures such as rendering to occur between updates 
            millisPerFixedUpdate -= 5;
            long fixedUpdateEndTime = startTimeMillis + millisPerFixedUpdate;

            // ReSharper disable once PossibleLossOfFraction
            int physicsTickDeltaMillis = SimConfig.LogicTickDeltaMillis / SimConfig.PhysicsTicksPerLogicUpdate;
            float physicsTickDeltaSeconds = physicsTickDeltaMillis / 1000.0f;

            // Only calculate updates if there is still time left in the current update
            while (Utils.CurrentTimeMillis() - startTimeMillis < millisPerFixedUpdate)
            {
                // Yield if no more updates are needed this FixedUpdate cycle
                if (_nextUpdateTimeMillis > fixedUpdateEndTime) break;

                Physics.Simulate(physicsTickDeltaSeconds);
                _physicsTickCount += 1;
                _simulatedTimeMillis += physicsTickDeltaMillis;
                _physicsTicksSinceUpdate++;
                if (_physicsTicksSinceUpdate >= SimConfig.PhysicsTicksPerLogicUpdate)
                {
                    // TODO: Update robot logic
                    _physicsTicksSinceUpdate = 0;
                    _robotLogicTickCount += 1;
                }

                // The delay between each update is dependant on the current play state
                int updateDelayMillis = physicsTickDeltaMillis / (int) _playState;
                // Calculate next time to do an update
                _nextUpdateTimeMillis = _nextUpdateTimeMillis + updateDelayMillis;
                // Do not try to catch up if more than 0.5 seconds behind (higher if tick delta is high)
                long maxDelayMillis = Math.Max(500, physicsTickDeltaMillis * 10);
                _nextUpdateTimeMillis = Math.Max(_nextUpdateTimeMillis, Utils.CurrentTimeMillis() - maxDelayMillis);
            }
            var simulatedTimeSpan = TimeSpan.FromMilliseconds(_simulatedTimeMillis);
            var output = simulatedTimeSpan.ToString(@"hh\:mm\:ss");
            Text.text = "Phys. ticks: " + _physicsTickCount + 
                        "\nLogic ticks: " + _robotLogicTickCount + 
                        "\nSimulated: " + output;
        }
    }
}