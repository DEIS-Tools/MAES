using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MaesInterface
{
    public class MoveActionGoal : ActionGoal<MoveGoal>
    {
        public const string k_RosMessageName = "maes_interface/MoveActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public MoveActionGoal() : base()
        {
            this.goal = new MoveGoal();
        }

        public MoveActionGoal(HeaderMsg header, GoalIDMsg goal_id, MoveGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static MoveActionGoal Deserialize(MessageDeserializer deserializer) => new MoveActionGoal(deserializer);

        MoveActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = MoveGoal.Deserialize(deserializer);
        }
        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.goal_id);
            serializer.Write(this.goal);
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
