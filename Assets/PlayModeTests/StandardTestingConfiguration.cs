using Maes;
using Maes.Map.MapGen;

namespace PlayModeTests {
    public class StandardTestingConfiguration {

        public static MapFactory EmptyCaveMapSpawner(int randomSeed) {
            var mapConfiguration = new CaveMapConfig(randomSeed: randomSeed, 
                widthInTiles: 50, 
                heightInTiles: 50, 
                smoothingRuns: 4,
                connectionPassagesWidth: 4, 
                randomFillPercent: 0, 
                wallThresholdSize: 10, 
                roomThresholdSize: 10,
                borderSize: 1);
            return (generator => generator.GenerateCaveMap(mapConfiguration, 2f));
        }

    }
}