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

using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public static class TransformExtensions
{

    public static TransformMsg ToROSTransform(this Transform tfUnity) {
        return new TransformMsg(
            // Using vector/quaternion To<>() because Transform.To<>() doesn't use localPosition/localRotation
            tfUnity.localPosition.To<FLU>(),
            tfUnity.localRotation.To<FLU>());
    }

    public static TransformStampedMsg ToROSTransformStamped(this Transform tfUnity, double timeStamp) {
        return new TransformStampedMsg(
            new HeaderMsg(new TimeStamp(timeStamp), tfUnity.parent.gameObject.name),
            tfUnity.gameObject.name,
            tfUnity.ToROSTransform());
    }
}
