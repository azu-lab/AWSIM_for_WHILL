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
using System.Globalization;
using System.Reflection;
using UnityEngine;
using Awsim.DreamerV3;
using Awsim.Entity;
using Awsim.Optimizer;
using Awsim.Ros2;
using EgoVehicleComponent = Awsim.Scene.AutowareSimulationDemo.EgoVehicle;

namespace Awsim.Benchmark
{
    /// <summary>
    /// Duplicates the infrastructure LiDAR and the WHILL at start-up, to measure how the resource
    /// usage of AWSIM grows with their number (scripts/bench_resources.sh, docs/RESOURCE_USAGE.md),
    /// with one player build and no scene change.
    ///
    /// **Does nothing unless one of these environment variables is set:**
    ///
    ///   AWSIM_BENCH_INFRA_LIDARS     total number of infrastructure LiDARs (including the one of the scene)
    ///   AWSIM_BENCH_WHILLS           total number of WHILLs (including the one of the scene)
    ///   AWSIM_BENCH_LIDAR_SPACING    distance between LiDAR copies along the WHILL heading [m] (default 4)
    ///   AWSIM_BENCH_WHILL_POSITIONS  places of the WHILL copies relative to the scene WHILL,
    ///                                "forward:right;forward:right;..." [m] (in the order of use)
    ///   AWSIM_BENCH_VEHICLE_LIDAR    0: switch off the vehicle LiDAR (VLP16) of every WHILL, including
    ///                                the one of the scene, before it starts (WHILL = camera only)
    ///
    /// Copies:
    ///   - LiDAR i (i >= 2): copy of InfraLidars/InfraLidar0, same height and orientation, publishes
    ///     /bench/infra_lidar_i/points.
    ///   - WHILL i (i >= 2): copy of the WHILL prefab instance (camera, vehicle LiDAR, physics), put on
    ///     the ground. Its camera, vehicle LiDAR, odometry, collision and wheel-speed topics get the
    ///     prefix /bench/whill_i; it subscribes the same cmd_vel / gear topics as the scene WHILL, so
    ///     one command drives all. Its OptimizerBridge is removed (one TCP port per process).
    ///     The scene controller only updates the scene WHILL, so this component initializes and
    ///     updates the copies (EgoVehicle.Initialize / OnUpdate / OnFixedUpdate).
    ///   - Every 5 s the log gets "[ResourceBenchmark] fps value=..." and, per WHILL, its position,
    ///     yaw rate and number of camera frames (to check that every copy really drives and renders).
    /// </summary>
    public class ResourceBenchmarkSpawner : MonoBehaviour
    {
        const string InfraLidarRoot = "InfraLidars";
        const BindingFlags Private = BindingFlags.NonPublic | BindingFlags.Instance;

        int _lidarCount;
        int _whillCount;
        float _lidarSpacing;
        List<Vector2> _whillPositions = new List<Vector2>();   // (forward, right) [m]
        bool _vehicleLidar = true;
        bool _spawned;
        float _nextLogTime;
        float _lastLogTime;
        int _framesSinceLog;

        readonly List<EgoVehicleComponent> _clones = new List<EgoVehicleComponent>();
        readonly List<EgoVehicleComponent> _all = new List<EgoVehicleComponent>();
        readonly List<int> _cameraFrames = new List<int>();

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void Bootstrap()
        {
            int lidars = ReadInt("AWSIM_BENCH_INFRA_LIDARS");
            int whills = ReadInt("AWSIM_BENCH_WHILLS");
            if (lidars <= 0 && whills <= 0)
                return;
            var go = new GameObject(nameof(ResourceBenchmarkSpawner));
            var s = go.AddComponent<ResourceBenchmarkSpawner>();
            s._lidarCount = Mathf.Max(lidars, 1);
            s._whillCount = Mathf.Max(whills, 1);
            s._lidarSpacing = float.TryParse(System.Environment.GetEnvironmentVariable("AWSIM_BENCH_LIDAR_SPACING"),
                NumberStyles.Float, CultureInfo.InvariantCulture, out float sp) ? sp : 4f;
            s._vehicleLidar = System.Environment.GetEnvironmentVariable("AWSIM_BENCH_VEHICLE_LIDAR") != "0";
            if (!s._vehicleLidar)
            {
                // AfterSceneLoad runs after Awake and before Start: the sensor never starts
                var ego = FindFirstObjectByType<EgoVehicleComponent>();
                if (ego != null)
                    DisableVehicleLidar(ego.gameObject);
            }
            var positions = System.Environment.GetEnvironmentVariable("AWSIM_BENCH_WHILL_POSITIONS");
            if (!string.IsNullOrEmpty(positions))
            {
                foreach (var item in positions.Split(';'))
                {
                    var fr = item.Split(':');
                    s._whillPositions.Add(new Vector2(
                        float.Parse(fr[0], CultureInfo.InvariantCulture),
                        fr.Length > 1 ? float.Parse(fr[1], CultureInfo.InvariantCulture) : 0f));
                }
            }
        }

        static int ReadInt(string key) =>
            int.TryParse(System.Environment.GetEnvironmentVariable(key), out int v) ? v : 0;

        void Update()
        {
            // First Update: after the Start of the scene controller (ROS 2 node initialized).
            if (!_spawned)
            {
                _spawned = true;
                Spawn();
            }
            foreach (var clone in _clones)
                clone.OnUpdate();
            _framesSinceLog++;
            if (Time.realtimeSinceStartup >= _nextLogTime)
            {
                _nextLogTime = Time.realtimeSinceStartup + 5f;
                LogStatus();
            }
        }

        void FixedUpdate()
        {
            foreach (var clone in _clones)
                clone.OnFixedUpdate();
        }

        void Spawn()
        {
            var ego = FindFirstObjectByType<EgoVehicleComponent>();   // active objects only: the WHILL, not the inactive Lexus
            if (ego == null)
            {
                Debug.LogError("[ResourceBenchmark] no EgoVehicleComponent in the scene");
                return;
            }
            var forward = Vector3.ProjectOnPlane(ego.transform.forward, Vector3.up).normalized;
            var right = Vector3.Cross(Vector3.up, forward);
            SpawnLidars(forward);
            SpawnWhills(ego, forward, right);
            Debug.Log($"[ResourceBenchmark] spawned infra_lidars={_lidarCount} whills={_all.Count} vehicle_lidar={_vehicleLidar}");
        }

        void SpawnLidars(Vector3 forward)
        {
            if (_lidarCount < 2)
                return;
            RglLidarPublisher original = null;
            foreach (var root in gameObject.scene.GetRootGameObjects())
                if (root.name == InfraLidarRoot) original = root.GetComponentInChildren<RglLidarPublisher>();
            if (original == null)
                foreach (var root in UnityEngine.SceneManagement.SceneManager.GetActiveScene().GetRootGameObjects())
                    if (root.name == InfraLidarRoot) original = root.GetComponentInChildren<RglLidarPublisher>();
            if (original == null)
            {
                Debug.LogError($"[ResourceBenchmark] no LiDAR under '{InfraLidarRoot}'");
                return;
            }
            var src = original.gameObject;
            for (int i = 2; i <= _lidarCount; i++)
            {
                var position = src.transform.position + forward * _lidarSpacing * (i - 1);
                var clone = Instantiate(src, position, src.transform.rotation, src.transform.parent);
                clone.name = $"InfraLidar_bench{i}";
                var publisher = clone.GetComponent<RglLidarPublisher>();
                publisher.frameId = $"bench_infra_lidar_{i}";
                publisher.pointCloud2Publishers[0].topic = $"/bench/infra_lidar_{i}/points";   // before its Start
                for (int k = 1; k < publisher.pointCloud2Publishers.Count; k++)
                    publisher.pointCloud2Publishers[k].topic = $"/bench/infra_lidar_{i}/points_{k}";
                Debug.Log($"[ResourceBenchmark] infra lidar {i} at {position}");
            }
        }

        void SpawnWhills(EgoVehicleComponent ego, Vector3 forward, Vector3 right)
        {
            _all.Add(ego);
            var ignore = new List<Transform> { ego.transform };
            float clearance = GroundProbe.TryFindGround(ego.transform.position.x, ego.transform.position.z,
                ego.transform.position.y + 1.5f, ignore, out float g0) ? ego.transform.position.y - g0 : 0f;

            // Copies are made under an inactive holder, so that nothing of them runs (Awake, Start,
            // the TCP server, ROS 2 publishers) before their topics are changed.
            var holder = new GameObject("BenchWhillHolder");
            holder.SetActive(false);
            for (int i = 2; i <= _whillCount; i++)
            {
                var place = i - 2 < _whillPositions.Count ? _whillPositions[i - 2] : new Vector2(4f * (i - 1), 0f);
                var position = ego.transform.position + forward * place.x + right * place.y;
                if (GroundProbe.TryFindGround(position.x, position.z, position.y + 3f, ignore, out float g))
                    position.y = g + clearance;
                var go = Instantiate(ego.gameObject, position, ego.transform.rotation, holder.transform);
                go.name = $"WhillModelCR_bench{i}";
                foreach (var bridge in go.GetComponentsInChildren<OptimizerBridge>(true))
                    DestroyImmediate(bridge);
                Rename(go, $"/bench/whill_{i}");
                if (!_vehicleLidar)
                    DisableVehicleLidar(go);
                go.transform.SetParent(null, true);   // activates the copy
                var clone = go.GetComponent<EgoVehicleComponent>();
                clone.Initialize();
                _clones.Add(clone);
                _all.Add(clone);
                ignore.Add(go.transform);
                Debug.Log($"[ResourceBenchmark] whill {i} at {position} (forward {place.x} m, right {place.y} m)");
            }
            Destroy(holder);

            foreach (var vehicle in _all)
            {
                int index = _cameraFrames.Count;
                _cameraFrames.Add(0);
                var camera = vehicle.GetComponentInChildren<CameraSensor>(true);
                if (camera != null)
                    camera.OnOutput += _ => _cameraFrames[index]++;
            }
        }

        static void DisableVehicleLidar(GameObject vehicle)
        {
            int n = 0;
            foreach (var lidar in vehicle.GetComponentsInChildren<RGLUnityPlugin.LidarSensor>(true))
            {
                lidar.enabled = false;
                n++;
            }
            foreach (var publisher in vehicle.GetComponentsInChildren<RglLidarPublisher>(true))
                publisher.enabled = false;
            Debug.Log($"[ResourceBenchmark] vehicle lidar off on {vehicle.name} ({n} sensor)");
        }

        /// <summary>Topics of the sensors of a copy, so that they do not mix with the scene WHILL.</summary>
        static void Rename(GameObject go, string prefix)
        {
            foreach (var cam in go.GetComponentsInChildren<CameraRos2Publisher>(true))
            {
                typeof(CameraRos2Publisher).GetField("_imageTopic", Private)?.SetValue(cam, $"{prefix}/camera/image_raw");
                typeof(CameraRos2Publisher).GetField("_cameraInfoTopic", Private)?.SetValue(cam, $"{prefix}/camera/camera_info");
            }
            foreach (var lidar in go.GetComponentsInChildren<RglLidarPublisher>(true))
            {
                lidar.frameId = $"{prefix.TrimStart('/')}/velodyne_top";
                for (int k = 0; k < lidar.pointCloud2Publishers.Count; k++)
                    lidar.pointCloud2Publishers[k].topic = $"{prefix}/lidar/pointcloud_{k}";
            }
            foreach (var odom in go.GetComponentsInChildren<GroundTruthPublisher>(true))
                odom.topicName = $"{prefix}/ground_truth/odom";
            foreach (var wheels in go.GetComponentsInChildren<WheelSpeedPublisher>(true))
            {
                wheels.topicName = $"{prefix}/wheel_velocities_sim";
                wheels.nodeName = $"wheel_speed_publisher_{prefix.Trim('/').Replace('/', '_')}";
            }
            foreach (var collision in go.GetComponentsInChildren<CollisionPublisher>(true))
                typeof(CollisionPublisher).GetField("_topicName", Private)?.SetValue(collision, $"{prefix}/collision_sensor");
        }

        void LogStatus()
        {
            float now = Time.realtimeSinceStartup;
            if (_lastLogTime > 0f)
                Debug.Log($"[ResourceBenchmark] fps value={_framesSinceLog / (now - _lastLogTime):F2}");
            _framesSinceLog = 0;
            _lastLogTime = now;
            for (int i = 0; i < _all.Count; i++)
            {
                var v = _all[i];
                if (v == null) continue;
                var rb = v.GetComponentInChildren<Rigidbody>();
                float speed = rb != null ? rb.linearVelocity.magnitude : -1f;
                float yawRate = rb != null ? Mathf.Abs(rb.angularVelocity.y) : -1f;
                var p = v.transform.position;
                Debug.Log($"[ResourceBenchmark] status whill={i + 1} pos=({p.x:F2},{p.y:F2},{p.z:F2}) speed={speed:F3} "
                        + $"yaw_rate={yawRate:F3} yaw={v.transform.eulerAngles.y:F1} camera_frames={_cameraFrames[i]}");
            }
        }
    }
}
