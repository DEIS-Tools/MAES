using System;
using System.Threading;
using UnityEngine;

namespace Dora
{
    public class Simulator : MonoBehaviour
    {
        public SimulationConfiguration SimConfig; // Should this be a struct?!
        private SimulationPlayState _playState;

        public SimulationPlayState PlayState
        {
            get => _playState;
            set
            {
                if (value == PlayState) return;
                if (_playState == SimulationPlayState.Paused)
                { // Changing from paused to other state
                    _nextUpdateTimeMillis = Utils.CurrentTimeMillis();
                }
                
                // TODO: Check if possible to change (For example, not possible if no map is generated)
                _playState = value;
            }
        }
        
        // Timing variables for controlling the simulation in a manner that is decoupled from Unity's update system
        private long _nextUpdateTimeMillis = 0;
        
        private void FixedUpdate()
        {
            long startTimeMillis = DateTime.Now.Ticks / TimeSpan.TicksPerMillisecond;
            int millisPerFixedUpdate = (int) (1000f * Time.fixedDeltaTime);
            long fixedUpdateEndTime = startTimeMillis + millisPerFixedUpdate;
            float physicsTickDeltaSeconds = SimConfig.PhysicsTickDeltaMillis * 1000;

            while (Utils.CurrentTimeMillis() - startTimeMillis > millisPerFixedUpdate)
            {
                // Yield if no more updates are needed this FixedUpdate cycle
                if (_nextUpdateTimeMillis > fixedUpdateEndTime) break;

                int millisUntilNextPhysicsUpdate;
                while ((millisUntilNextPhysicsUpdate = (int) (_nextUpdateTimeMillis - Utils.CurrentTimeMillis())) > 0)
                {
                    Thread.Sleep(millisUntilNextPhysicsUpdate);
                }
                Physics.Simulate(physicsTickDeltaSeconds);
                
                // Calculate next time to do an update
                _nextUpdateTimeMillis = _nextUpdateTimeMillis + SimConfig.PhysicsTickDeltaMillis;
                // Do not try to catch up if more than 0.5 seconds behind (or more if tick delta is high)
                long maxDelayMillis = Math.Max(500, SimConfig.PhysicsTickDeltaMillis * 10);
                _nextUpdateTimeMillis = Math.Max(_nextUpdateTimeMillis, Utils.CurrentTimeMillis() - maxDelayMillis);
            }
            throw new NotImplementedException();
        }
    }
    
}
