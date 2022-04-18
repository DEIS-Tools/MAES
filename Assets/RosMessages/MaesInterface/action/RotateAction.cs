using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;


namespace RosMessageTypes.MaesInterface
{
    public class RotateAction : Action<RotateActionGoal, RotateActionResult, RotateActionFeedback, RotateGoal, RotateResult, RotateFeedback>
    {
        public const string k_RosMessageName = "maes_interface/RotateAction";
        public override string RosMessageName => k_RosMessageName;


        public RotateAction() : base()
        {
            this.action_goal = new RotateActionGoal();
            this.action_result = new RotateActionResult();
            this.action_feedback = new RotateActionFeedback();
        }

        public static RotateAction Deserialize(MessageDeserializer deserializer) => new RotateAction(deserializer);

        RotateAction(MessageDeserializer deserializer)
        {
            this.action_goal = RotateActionGoal.Deserialize(deserializer);
            this.action_result = RotateActionResult.Deserialize(deserializer);
            this.action_feedback = RotateActionFeedback.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.action_goal);
            serializer.Write(this.action_result);
            serializer.Write(this.action_feedback);
        }

    }
}
