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
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Awsim.InfraLidar;

namespace Awsim.InfraLidar.Editor
{
    /// <summary>
    /// Builds the infrastructure LiDARs of a scene from config/infra_lidar.yaml.
    ///
    /// The YAML is the source of truth: pose, sensor model, topic, frame and QoS of a
    /// sensor placed by hand in the Inspector end up buried in the scene YAML and cannot
    /// be reviewed or reproduced. Running this tool again always gives the same scene.
    ///
    /// Batchmode:
    ///   Unity -batchmode -quit -projectPath . \
    ///     -executeMethod Awsim.InfraLidar.Editor.InfraLidarSetup.SetupBatch
    ///   (optional) -infraLidarConfig config/infra_lidar.yaml -infraLidarScene Assets/...unity
    /// </summary>
    public static class InfraLidarSetup
    {
        public const string DefaultConfigPath = "config/infra_lidar.yaml";
        public const string DefaultScenePath = "Assets/Awsim/Scenes/AutowareSimulationDemo.unity";
        public const string RootName = "InfraLidars";
        const string PrefabDirectory = "Assets/Awsim/Prefabs/Entity/EgoVehicle/Sensor/Lidars/Hdrp";

        [MenuItem("AWSIM for WHILL/Setup infrastructure LiDARs (config/infra_lidar.yaml)")]
        static void SetupMenu()
        {
            Setup(DefaultConfigPath);
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }

        public static void SetupBatch()
        {
            var config = GetArg("-infraLidarConfig") ?? DefaultConfigPath;
            var scenePath = GetArg("-infraLidarScene") ?? DefaultScenePath;
            var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            Setup(config);
            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
                throw new System.Exception($"Failed to save {scenePath}");
        }

        /// <summary>
        /// Replace the "InfraLidars" object of the active scene with the sensors described in the YAML.
        /// </summary>
        public static void Setup(string configPath)
        {
            if (!File.Exists(configPath))
                throw new FileNotFoundException($"Infrastructure LiDAR config not found: {configPath}");

            var root = new YamlDotNet.Serialization.DeserializerBuilder().Build()
                .Deserialize<Dictionary<object, object>>(File.ReadAllText(configPath));

            var scene = EditorSceneManager.GetActiveScene();
            foreach (var go in scene.GetRootGameObjects())
            {
                if (go.name == RootName)
                    Object.DestroyImmediate(go);
            }

            var parent = new GameObject(RootName);
            SceneManager_MoveToScene(parent, scene);
            var tfPublisher = parent.AddComponent<InfraLidarTfPublisher>();
            var tfSo = new SerializedObject(tfPublisher);
            tfSo.FindProperty("_parentFrameId").stringValue = ReadString(root, "parent_frame_id", "map");
            tfSo.ApplyModifiedPropertiesWithoutUndo();

            if (!root.TryGetValue("infra_lidars", out var listObj) || !(listObj is List<object> list))
                throw new System.Exception($"{configPath}: 'infra_lidars' must be a list");

            var usedFrames = new HashSet<string>();
            var usedTopics = new HashSet<string>();
            foreach (var item in list)
            {
                var config = (Dictionary<object, object>)item;
                var name = ReadString(config, "name", "InfraLidar");
                var model = ReadString(config, "model", "VelodyneVLP16");
                var frameId = ReadString(config, "frame_id", name);
                var topic = ReadString(config, "topic", $"/infrastructure/{name}/points");
                if (!usedFrames.Add(frameId))
                    throw new System.Exception($"{configPath}: duplicated frame_id '{frameId}'");
                if (!usedTopics.Add(topic))
                    throw new System.Exception($"{configPath}: duplicated topic '{topic}'");

                var prefabPath = $"{PrefabDirectory}/{model}.prefab";
                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
                if (prefab == null)
                    throw new System.Exception($"{configPath}: LiDAR prefab not found: {prefabPath} (model = {model})");

                var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab, parent.transform);
                instance.name = name;

                var position = Dig(config, "position");
                var rotation = Dig(config, "rotation_deg");
                instance.transform.position = new Vector3(
                    ReadFloat(position, "x", 0f), ReadFloat(position, "y", 0f), ReadFloat(position, "z", 0f));
                // Unity euler angles. pitch > 0 tilts the sensor down.
                instance.transform.rotation = Quaternion.Euler(
                    ReadFloat(rotation, "pitch", 0f), ReadFloat(rotation, "yaw", 0f), ReadFloat(rotation, "roll", 0f));

                var publisher = instance.GetComponent<Awsim.Entity.RglLidarPublisher>();
                var lidar = instance.GetComponent<RGLUnityPlugin.LidarSensor>();
                if (publisher == null || lidar == null || publisher.pointCloud2Publishers.Count == 0)
                    throw new System.Exception($"{prefabPath} has no LidarSensor / RglLidarPublisher / PointCloud2 publisher");

                publisher.frameId = frameId;
                ApplyQos(publisher, Dig(config, "qos"), configPath);
                publisher.pointCloud2Publishers[0].topic = topic;
                publisher.pointCloud2Publishers[0].publish = true;
                // Every element is initialized (= advertised) even with publish = false,
                // so drop the extra ones to advertise exactly one topic per sensor.
                if (publisher.pointCloud2Publishers.Count > 1)
                    publisher.pointCloud2Publishers.RemoveRange(1, publisher.pointCloud2Publishers.Count - 1);
                publisher.radarScanPublishers.Clear();
                lidar.AutomaticCaptureHz = (int)ReadFloat(config, "publish_hz", 10f);

                EditorUtility.SetDirty(instance);
                EditorUtility.SetDirty(publisher);
                EditorUtility.SetDirty(lidar);
                Debug.Log($"[InfraLidarSetup] {name}: model={model} pos={instance.transform.position} "
                          + $"rot={instance.transform.eulerAngles} topic={topic} frame={frameId} hz={lidar.AutomaticCaptureHz}");
            }
        }

        static void SceneManager_MoveToScene(GameObject go, UnityEngine.SceneManagement.Scene scene)
        {
            UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(go, scene);
        }

        static void ApplyQos(Awsim.Entity.RglLidarPublisher publisher, Dictionary<object, object> qos, string configPath)
        {
            if (publisher.qos == null)
                publisher.qos = new Awsim.Entity.RglQos();
            if (qos == null)
                return;

            switch (ReadString(qos, "reliability", "best_effort"))
            {
                case "best_effort": publisher.qos.reliabilityPolicy = RGLUnityPlugin.RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_BEST_EFFORT; break;
                case "reliable": publisher.qos.reliabilityPolicy = RGLUnityPlugin.RGLQosPolicyReliability.QOS_POLICY_RELIABILITY_RELIABLE; break;
                default: throw new System.Exception($"{configPath}: qos.reliability must be best_effort or reliable");
            }
            switch (ReadString(qos, "durability", "volatile"))
            {
                case "volatile": publisher.qos.durabilityPolicy = RGLUnityPlugin.RGLQosPolicyDurability.QOS_POLICY_DURABILITY_VOLATILE; break;
                case "transient_local": publisher.qos.durabilityPolicy = RGLUnityPlugin.RGLQosPolicyDurability.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL; break;
                default: throw new System.Exception($"{configPath}: qos.durability must be volatile or transient_local");
            }
            switch (ReadString(qos, "history", "keep_last"))
            {
                case "keep_last": publisher.qos.historyPolicy = RGLUnityPlugin.RGLQosPolicyHistory.QOS_POLICY_HISTORY_KEEP_LAST; break;
                case "keep_all": publisher.qos.historyPolicy = RGLUnityPlugin.RGLQosPolicyHistory.QOS_POLICY_HISTORY_KEEP_ALL; break;
                default: throw new System.Exception($"{configPath}: qos.history must be keep_last or keep_all");
            }
            publisher.qos.historyDepth = (int)ReadFloat(qos, "depth", 5f);
        }

        static Dictionary<object, object> Dig(Dictionary<object, object> d, string key)
        {
            return d != null && d.TryGetValue(key, out var v) ? v as Dictionary<object, object> : null;
        }

        static string ReadString(Dictionary<object, object> d, string key, string fallback)
        {
            return d != null && d.TryGetValue(key, out var v) && v != null ? v.ToString() : fallback;
        }

        static float ReadFloat(Dictionary<object, object> d, string key, float fallback)
        {
            return d != null && d.TryGetValue(key, out var v) && v != null
                ? float.Parse(v.ToString(), CultureInfo.InvariantCulture)
                : fallback;
        }

        static string GetArg(string name)
        {
            var args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i] == name)
                    return args[i + 1];
            return null;
        }
    }
}
