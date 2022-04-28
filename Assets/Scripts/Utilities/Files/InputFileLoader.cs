using System;
using System.IO;
using JetBrains.Annotations;

namespace Maes.Utilities.Files {
    public class InputFileLoader {

        public static string GetDefaultInputPath() {
            return Path.Join(Path.Join(Directory.GetCurrentDirectory(), "maes-ros-slam-ws"), "src",
                "maes_ros2_interface");
        }

        [CanBeNull]
        public static StreamReader ReadInputFile(String fileName) {
            return new StreamReader(Path.Join(GetDefaultInputPath(), fileName));
        }


    }
}