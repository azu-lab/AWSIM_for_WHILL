// Copyright 2026 shoumexi-03
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System.Collections.Generic;
using UnityEngine;
using ROS2;
using Awsim.Common;
using Awsim.Entity;

namespace Awsim.InfraLidar
{
    /// <summary>
    /// Publishes the pose of the infrastructure LiDARs as static transforms
    /// (parent frame -> RglLidarPublisher.frameId) on /tf_static.
    ///
    /// The point clouds of RglLidarPublisher are expressed in the sensor frame, so a
    /// consumer needs this transform to put them in the map frame. "map" here is the
    /// same frame as /ground_truth/odom: the Unity world converted to ROS axes
    /// (x = Unity z, y = -Unity x, z = Unity y).
    ///
    /// Attach to the parent object of the LiDARs. Every active RglLidarPublisher under
    /// it is published. The message is sent once with TRANSIENT_LOCAL durability and
    /// again whenever a sensor moves.
    /// </summary>
    public class InfraLidarTfPublisher : MonoBehaviour
    {
        [SerializeField] string _topic = "/tf_static";
        [SerializeField] string _parentFrameId = "map";
        [SerializeField, Tooltip("Seconds between checks whether a sensor has moved.")]
        float _checkInterval = 1.0f;

        IPublisher<tf2_msgs.msg.TFMessage> _publisher;
        readonly List<(RglLidarPublisher publisher, Vector3 position, Quaternion rotation)> _published =
            new List<(RglLidarPublisher, Vector3, Quaternion)>();
        float _nextCheck;

        void Update()
        {
            if (_publisher == null && !TryCreatePublisher())
                return;

            if (Time.unscaledTime < _nextCheck)
                return;
            _nextCheck = Time.unscaledTime + _checkInterval;

            var sensors = GetComponentsInChildren<RglLidarPublisher>(false);
            if (_published.Count == sensors.Length && !HasMoved())
                return;

            Publish(sensors);
        }

        bool TryCreatePublisher()
        {
            try
            {
                var qos = new QualityOfServiceProfile();
                qos.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE);
                qos.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
                qos.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 1);
                _publisher = AwsimRos2Node.CreatePublisher<tf2_msgs.msg.TFMessage>(_topic, qos);
                return true;
            }
            catch (System.Exception)
            {
                // AwsimRos2Node is initialized by the scene controller; retry next frame.
                return false;
            }
        }

        bool HasMoved()
        {
            foreach (var p in _published)
            {
                if (p.publisher == null)
                    return true;
                var t = p.publisher.transform;
                if ((t.position - p.position).sqrMagnitude > 1e-8f || Quaternion.Angle(t.rotation, p.rotation) > 1e-3f)
                    return true;
            }
            return false;
        }

        void Publish(RglLidarPublisher[] sensors)
        {
            AwsimRos2Node.GetTime(out var sec, out var nanosec);
            var transforms = new tf2_msgs.msg.TFMessage();
            var list = new List<geometry_msgs.msg.TransformStamped>();
            _published.Clear();

            foreach (var sensor in sensors)
            {
                var t = sensor.transform;
                var p = Ros2Utility.UnityToRos2Position(t.position);
                var q = Ros2Utility.UnityToRosRotation(t.rotation);

                var tf = new geometry_msgs.msg.TransformStamped();
                tf.Header.Stamp.Sec = sec;
                tf.Header.Stamp.Nanosec = nanosec;
                tf.Header.Frame_id = _parentFrameId;
                tf.Child_frame_id = sensor.frameId;
                tf.Transform.Translation.X = p.x;
                tf.Transform.Translation.Y = p.y;
                tf.Transform.Translation.Z = p.z;
                tf.Transform.Rotation.X = q.x;
                tf.Transform.Rotation.Y = q.y;
                tf.Transform.Rotation.Z = q.z;
                tf.Transform.Rotation.W = q.w;
                list.Add(tf);
                _published.Add((sensor, t.position, t.rotation));
            }

            transforms.Transforms = list.ToArray();
            _publisher.Publish(transforms);
        }

        void OnDestroy()
        {
            if (_publisher != null)
                AwsimRos2Node.RemovePublisher<tf2_msgs.msg.TFMessage>(_publisher);
        }
    }
}
