using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Actionlib;

namespace RosMessageTypes.MaesInterface
{
    public class RotateActionGoal : ActionGoal<RotateGoal>
    {
        public const string k_RosMessageName = "maes_interface/RotateActionGoal";
        public override string RosMessageName => k_RosMessageName;


        public RotateActionGoal() : base()
        {
            this.goal = new RotateGoal();
        }

        public RotateActionGoal(HeaderMsg header, GoalIDMsg goal_id, RotateGoal goal) : base(header, goal_id)
        {
            this.goal = goal;
        }
        public static RotateActionGoal Deserialize(MessageDeserializer deserializer) => new RotateActionGoal(deserializer);

        RotateActionGoal(MessageDeserializer deserializer) : base(deserializer)
        {
            this.goal = RotateGoal.Deserialize(deserializer);
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
