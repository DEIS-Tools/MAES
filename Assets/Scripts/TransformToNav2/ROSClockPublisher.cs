// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;
using Unity.Robotics.Core;

public class ROSClockPublisher : MonoBehaviour
{
    [SerializeField]
    Clock.ClockMode m_ClockMode;

    [SerializeField, HideInInspector]
    Clock.ClockMode m_LastSetClockMode;
    
    [SerializeField] 
    double m_PublishRateHz = 100f;

    double m_LastPublishTimeSeconds;

    ROSConnection m_ROS;

    private string m_ClockTopic = "/clock";

    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;

    bool ShouldPublishMessage => Clock.FrameStartTimeInSeconds - PublishPeriodSeconds > m_LastPublishTimeSeconds;

    void OnValidate()
    {
        var clocks = FindObjectsOfType<ROSClockPublisher>();
        if (clocks.Length > 1)
        {
            Debug.LogWarning("Found too many clock publishers in the scene, there should only be one!");
        }

        if (Application.isPlaying && m_LastSetClockMode != m_ClockMode)
        {
            Debug.LogWarning("Can't change ClockMode during simulation! Setting it back...");
            m_ClockMode = m_LastSetClockMode;
        }
        
        SetClockMode(m_ClockMode);
    }

    void SetClockMode(Clock.ClockMode mode)
    {
        Clock.Mode = mode;
        m_LastSetClockMode = mode;
    }

    // Start is called before the first frame update
    void Start()
    {
        SetClockMode(m_ClockMode);
        m_ROS = ROSConnection.GetOrCreateInstance();
        m_ROS.RegisterPublisher<ClockMsg>(m_ClockTopic);
    }

    void PublishMessage()
    {
        var publishTime = Clock.time;
        var clockMsg = new TimeMsg
        {
            sec = (int)publishTime,
            nanosec = (uint)((publishTime - Math.Floor(publishTime)) * Clock.k_NanoSecondsInSeconds)
        };
        m_LastPublishTimeSeconds = publishTime;
        m_ROS.Publish(m_ClockTopic, clockMsg);
    }

    void Update()
    {
        if (ShouldPublishMessage)
        {
            PublishMessage();
        }
    }
}
