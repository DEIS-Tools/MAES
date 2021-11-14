#nullable enable
using JetBrains.Annotations;

namespace Dora.ExplorationAlgorithm.SSB {
    public interface ISsbBroadcastMessage {
        
        public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm);

        // Attempt to combine messages into a new message, if possible
        public ISsbBroadcastMessage? Combine(ISsbBroadcastMessage other, SsbAlgorithm algorithm);

    }
}