using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MaesInterface
{
    public class RotateActionFeedback : ActionFeedback<RotateFeedback>
    {
        public const string k_RosMessageName = "maes_interface/RotateActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public RotateActionFeedback() : base()
        {
            this.feedback = new RotateFeedback();
        }

        public RotateActionFeedback(HeaderMsg header, GoalStatusMsg status, RotateFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static RotateActionFeedback Deserialize(MessageDeserializer deserializer) => new RotateActionFeedback(deserializer);

        RotateActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = RotateFeedback.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.status);
            serializer.Write(this.feedback);
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
