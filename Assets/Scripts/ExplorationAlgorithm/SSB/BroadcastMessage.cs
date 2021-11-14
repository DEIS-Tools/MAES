#nullable enable
using JetBrains.Annotations;

namespace Dora.ExplorationAlgorithm.SSB {
    public interface ISsbBroadcastMessage {
        
        public ISsbBroadcastMessage? Process(SsbAlgorithm algorithm);

    }
}