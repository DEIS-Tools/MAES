using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MaesInterface
{
    public class MoveActionFeedback : ActionFeedback<MoveFeedback>
    {
        public const string k_RosMessageName = "maes_interface/MoveActionFeedback";
        public override string RosMessageName => k_RosMessageName;


        public MoveActionFeedback() : base()
        {
            this.feedback = new MoveFeedback();
        }

        public MoveActionFeedback(HeaderMsg header, GoalStatusMsg status, MoveFeedback feedback) : base(header, status)
        {
            this.feedback = feedback;
        }
        public static MoveActionFeedback Deserialize(MessageDeserializer deserializer) => new MoveActionFeedback(deserializer);

        MoveActionFeedback(MessageDeserializer deserializer) : base(deserializer)
        {
            this.feedback = MoveFeedback.Deserialize(deserializer);
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
