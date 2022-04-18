using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.MaesInterface
{
    public class MoveAction : Action<MoveActionGoal, MoveActionResult, MoveActionFeedback, MoveGoal, MoveResult, MoveFeedback>
    {
        public const string k_RosMessageName = "maes_interface/MoveAction";
        public override string RosMessageName => k_RosMessageName;


        public MoveAction() : base()
        {
            this.action_goal = new MoveActionGoal();
            this.action_result = new MoveActionResult();
            this.action_feedback = new MoveActionFeedback();
        }

        public static MoveAction Deserialize(MessageDeserializer deserializer) => new MoveAction(deserializer);

        MoveAction(MessageDeserializer deserializer)
        {
            this.action_goal = MoveActionGoal.Deserialize(deserializer);
            this.action_result = MoveActionResult.Deserialize(deserializer);
            this.action_feedback = MoveActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
