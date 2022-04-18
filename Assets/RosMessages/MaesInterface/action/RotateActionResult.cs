using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MaesInterface
{
    public class RotateActionResult : ActionResult<RotateResult>
    {
        public const string k_RosMessageName = "maes_interface/RotateActionResult";
        public override string RosMessageName => k_RosMessageName;


        public RotateActionResult() : base()
        {
            this.result = new RotateResult();
        }

        public RotateActionResult(HeaderMsg header, GoalStatusMsg status, RotateResult result) : base(header, status)
        {
            this.result = result;
        }
        public static RotateActionResult Deserialize(MessageDeserializer deserializer) => new RotateActionResult(deserializer);

        RotateActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = RotateResult.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.result);
        }


#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
