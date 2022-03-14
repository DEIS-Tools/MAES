using System;
using System.Collections.Generic;
using System.Linq;
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
        Debug.Log($"Children in node {m_TransformRoot.name} =  {m_TransformRoot.Children.Count}");
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
            PublishMessage();
        }

    }
}
