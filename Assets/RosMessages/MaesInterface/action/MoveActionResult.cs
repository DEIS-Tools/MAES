using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MaesInterface
{
    public class MoveActionResult : ActionResult<MoveResult>
    {
        public const string k_RosMessageName = "maes_interface/MoveActionResult";
        public override string RosMessageName => k_RosMessageName;


        public MoveActionResult() : base()
        {
            this.result = new MoveResult();
        }

        public MoveActionResult(HeaderMsg header, GoalStatusMsg status, MoveResult result) : base(header, status)
        {
            this.result = result;
        }
        public static MoveActionResult Deserialize(MessageDeserializer deserializer) => new MoveActionResult(deserializer);

        MoveActionResult(MessageDeserializer deserializer) : base(deserializer)
        {
            this.result = MoveResult.Deserialize(deserializer);
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
