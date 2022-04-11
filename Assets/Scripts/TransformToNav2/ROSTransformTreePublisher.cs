using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.SlamExample;
using UnityEngine;

public class ROSTransformTreePublisher : MonoBehaviour
{
    const string k_TfTopic = "/tf";
    
    [SerializeField]
    double m_PublishRateHz = 20f;
    [SerializeField]
    List<string> m_GlobalFrameIds = new List<string> { "map", "odom"};
    [SerializeField]
    GameObject m_RootGameObject;
    
    double m_LastPublishTimeSeconds;

    TransformTreeNode m_TransformRoot;
    ROSConnection m_ROS;

    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    // Start is called before the first frame update
    void Start()
    {
        if (m_RootGameObject == null)
        {
            Debug.LogWarning($"No GameObject explicitly defined as {nameof(m_RootGameObject)}, so using {name} as root.");
            m_RootGameObject = gameObject;
        }

        m_ROS = ROSConnection.GetOrCreateInstance();
        m_TransformRoot = new TransformTreeNode(m_RootGameObject);
        m_ROS.RegisterPublisher<TFMessageMsg>(k_TfTopic);
        m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;
    }

    static void PopulateTFList(List<TransformStampedMsg> tfList, TransformTreeNode tfNode)
    {
        // TODO: Some of this could be done once and cached rather than doing from scratch every time
        // Only generate transform messages from the children, because This node will be parented to the global frame
        foreach (var childTf in tfNode.Children)
        {
            tfList.Add(TransformTreeNode.ToTransformStamped(childTf));
            
            if (!childTf.IsALeafNode)
            {
                PopulateTFList(tfList, childTf);
            }
        }
    }

    void PublishMessageCustom() {
        // Publish fake transform messages (copied from slam example project)
        var tfMessage = new TFMessageMsg(GenerateTransformMessages().ToArray());
        m_ROS.Publish(k_TfTopic, tfMessage);
        m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
    }

    void PublishMessage()
    {
        var tfMessageList = new List<TransformStampedMsg>();

        if (m_GlobalFrameIds.Count > 0) {
            // This fixes the error of "transform out of bounds of cost map"
            var pos = new Vector3(m_TransformRoot.Transform.position.x, 
                m_TransformRoot.Transform.position.y, 
                m_TransformRoot.Transform.position.z);
            var qat = m_TransformRoot.Transform.rotation;
            
            Vector3 eulers = this.transform.rotation.eulerAngles;
            // qat = Quaternion.Euler(new Vector3(-90, eulers.y, eulers.z));
            var tra = new TransformMsg(new Vector3Msg(pos.x, pos.y, pos.y),
                    new QuaternionMsg(qat.x, qat.y, qat.z, qat.w));
            
            
            
            var tfRootToGlobal = new TransformStampedMsg(
                header: new HeaderMsg(stamp: new TimeStamp(Clock.time), 
                                      frame_id: m_GlobalFrameIds.Last()), 
                child_frame_id: m_TransformRoot.name,
                transform: tra);
            
            tfMessageList.Add(tfRootToGlobal);
        }
        else
        {
            Debug.LogWarning($"No {m_GlobalFrameIds} specified, transform tree will be entirely local coordinates.");
        }
        
        // In case there are multiple "global" transforms that are effectively the same coordinate frame, 
        // treat this as an ordered list, first entry is the "true" global
        for (var i = 1; i < m_GlobalFrameIds.Count; ++i)
        {
            var tfGlobalToGlobal = new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(Clock.time), m_GlobalFrameIds[i - 1]),
                m_GlobalFrameIds[i],
                // Initializes to identity transform
                new TransformMsg());
            tfMessageList.Add(tfGlobalToGlobal);
        }

        PopulateTFList(tfMessageList, m_TransformRoot);
        
        var tfMessage = new TFMessageMsg(tfMessageList.ToArray());
        m_ROS.Publish(k_TfTopic, tfMessage);
        m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
    }

    void Update()
    {
        if (ShouldPublishMessage)
        {
            PublishMessageCustom();
        }

    }
    
    private List<TransformStampedMsg> GenerateTransformMessages() {
        var robotTransform = transform;
        var robot_position = robotTransform.position;
        var robot_rotation = robotTransform.rotation.eulerAngles.z - 90f;
        
        // 90
        // 360 - 90 = 270
        
        // 45 
        // 360 - 45 = 315
        
        var quat = Quaternion.Euler(0, 0, robot_rotation);
        // Debug.Log($"Euler angles: {quat.eulerAngles} vs. robot {robotTransform.rotation.eulerAngles}");
        var list = new List<TransformStampedMsg>() {
                ToStampedTransformMsg("odom", "base_footprint",
                    new Vector3(robot_position.x, robot_position.y,0), 
                    quat),
                ToStampedTransformMsg("map", "odom", 
                    new Vector3(0f,0, 0), new Quaternion(0f, 0f, 0f, 1f)
                    ),
                
                ToStampedTransformMsg("base_footprint", "base_link", 
                    new Vector3(-1.1932570487260818e-09f,5.281606263451977e-10f, 0.009999998845160007f), 
                    new Quaternion(4.5811162863174104e-08f, -4.1443854570388794e-08f, 8.56289261719212e-09f, -1.0f)),
                
                ToStampedTransformMsg("base_link", "base_scan", 
                    new Vector3(-0.06399999558925629f,-5.218086407410283e-09f, 0.12200000882148743f), 
                    new Quaternion(-4.461279701217791e-08f, 2.9802322387695312e-08f, 1.517582859378308e-08f, -1.0f)),
                
                ToStampedTransformMsg("base_link", "imu_link", 
                    new Vector3(-3.4731337805737894e-09f,-4.79555950505528e-10f, 0.06800001114606857f), 
                    new Quaternion(-4.461279701217791e-08f, 2.9802322387695312e-08f, 1.517582859378308e-08f, -1.0f)),
                
                ToStampedTransformMsg("base_link", "wheel_right_link", 
                    new Vector3(1.3163943268779121e-09f,-0.14400000870227814f, 0.02300000749528408f), 
                    new Quaternion(0.7068246006965637f, 0.0009544402710162103f, 0.0009550635586492717f, -0.7073875665664673f)),
                
                ToStampedTransformMsg("base_link", "wheel_left_link", 
                    new Vector3(-2.0845297044047584e-08f,0.14399999380111694f, 0.02300000749528408f), 
                    new Quaternion(0.7068251371383667f, -0.00028174000908620656f, -0.0002820161171257496f, -0.7073882222175598f)),
            };

            return list;
        }

        private static TransformStampedMsg ToStampedTransformMsg(string parentFrame, string childFrame, Vector3 position, Quaternion quaternion) {
            return new TransformStampedMsg(
                new HeaderMsg(new TimeStamp(Clock.time), parentFrame),
                childFrame,
                new TransformMsg(new Vector3Msg(position.x, position.y, position.z), 
                    new QuaternionMsg(quaternion.x, quaternion.y, quaternion.z, quaternion.w)));
        }
}
