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

using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Awsim.WhillModelCR.Editor
{
    /// <summary>
    /// Generates the scenes of this project from the default scene, so that every scene can be
    /// rebuilt from code instead of hand-edited in the Inspector.
    ///
    /// | Scene | Map | Use |
    /// |---|---|---|
    /// | AutowareSimulationDemo.unity (default) | West Shinjuku (AWSIM Externals, scripts/fetch_shinjuku_map.sh) | driving, optimization, infrastructure LiDAR |
    /// | WhillCorridor.unity | corridor built from primitives, obstacles | DreamerV3 obstacle avoidance |
    /// | WhillOpenField.unity | flat 200 m x 200 m ground | optimization without the Shinjuku map |
    ///
    ///   -executeMethod Awsim.WhillModelCR.Editor.WhillSceneTools.SetupScenesBatch
    ///   -executeMethod Awsim.WhillModelCR.Editor.WhillSceneTools.CreateOpenFieldSceneBatch
    /// </summary>
    public static class WhillSceneTools
    {
        public const string DefaultScene = "Assets/Awsim/Scenes/AutowareSimulationDemo.unity";
        public const string CorridorScene = "Assets/Awsim/Scenes/WhillCorridor.unity";
        public const string OpenFieldScene = "Assets/Awsim/Scenes/WhillOpenField.unity";
        public const string ShinjukuInfraLidarConfig = "config/infra_lidar.yaml";
        public const string CorridorInfraLidarConfig = "config/infra_lidar_corridor.yaml";

        const string ShinjukuRoot = "Environments";                         // upstream map (+ V2I)
        static readonly string[] CorridorRoots = { "roukamap", "shougaikaihi", "goal_2" };

        class Layout
        {
            public string Name;
            public bool Shinjuku;
            public Vector3 WhillPosition;
            public Quaternion WhillRotation;
            public string Goal;            // EnvironmentReset._goalObject
            public string[] Obstacles;     // EnvironmentReset._obstacles
            public string InfraLidarConfig;
            public bool CollideWithMap;    // CollisionPublisher._detectEnvironment
            public TaskObstacle[] TaskObstacles = new TaskObstacle[0];   // created under "TaskObstacles"
            // EnvironmentReset: offset (Unity axes) / yaw range applied to the obstacles at every reset
            public Vector3 ObstacleRandomMin, ObstacleRandomMax;
            public float ObstacleYawMin, ObstacleYawMax;
        }

        /// <summary>Box obstacle of the RL task, placed on the ground (a copy of the cardboard box "danbo-ru").</summary>
        class TaskObstacle
        {
            public string Name;
            public float X, Z;             // Unity x, z (map frame: x = Unity z, y = -Unity x)
            public Vector3 Size;           // Unity x (width), y (height), z (depth) [m]
        }

        const string TaskObstaclesRoot = "TaskObstacles";
        const string CardboardTemplate = "danbo-ru";

        // Start pose of soturon's optimization runs (scene of soturon e619625, January 2026).
        static readonly Layout ShinjukuLayout = new Layout
        {
            Name = "Shinjuku",
            Shinjuku = true,
            WhillPosition = new Vector3(-30f, -7.368f, -140f),
            WhillRotation = Quaternion.identity,
            Goal = "goal",                 // 5 m ahead of the start pose
            InfraLidarConfig = ShinjukuInfraLidarConfig,
            CollideWithMap = true,         // buildings, curbs and shrubs of the map are not tagged
            // Example obstacle of the RL task rl/configs/tasks/shinjuku_box.yaml: on the same sidewalk,
            // 14 m ahead of the optimization start, 2.5 m ahead of the task start (-32.8, z -126).
            // Move it in the Unity Editor (Scene view) or add more objects tagged "Obstacle".
            TaskObstacles = new[] { new TaskObstacle { Name = "cardboard_box", X = -32.8f, Z = -123.5f, Size = new Vector3(0.28f, 0.54f, 0.5f) } },
            Obstacles = new[] { "cardboard_box" },
            ObstacleRandomMin = new Vector3(-0.4f, 0f, -0.5f),
            ObstacleRandomMax = new Vector3(0.4f, 0f, 0.5f),
            ObstacleYawMin = 0f,
            ObstacleYawMax = 360f,
        };

        // Corridor of soturon e91b33f (start pose and goal of the obstacle-avoidance task).
        static readonly Layout CorridorLayout = new Layout
        {
            Name = "Corridor",
            Shinjuku = false,
            WhillPosition = new Vector3(-0.603f, -0.704f, -0.363f),
            WhillRotation = new Quaternion(0.0019500707f, -0.7448704f, 0.00174674f, 0.667204f),
            Goal = "goal_2",
            Obstacles = new[] { "danbo-ru" },
            InfraLidarConfig = CorridorInfraLidarConfig,
            CollideWithMap = false,        // as soturon: walls and obstacles are tagged "Obstacle"
            // soturon e91b33f
            ObstacleRandomMin = new Vector3(-1f, 0f, -0.5f),
            ObstacleRandomMax = new Vector3(-0.5f, 0f, 0.5f),
            ObstacleYawMin = 0f,
            ObstacleYawMax = 360f,
        };

        [MenuItem("AWSIM for WHILL/Regenerate scenes (Shinjuku default + corridor)")]
        public static void SetupScenesBatch()
        {
            // The corridor scene is a copy of the default scene with another layout.
            var scene = EditorSceneManager.OpenScene(DefaultScene, OpenSceneMode.Single);
            if (!EditorSceneManager.SaveScene(scene, CorridorScene, true))
                throw new System.Exception($"Failed to copy {DefaultScene} to {CorridorScene}");

            Configure(CorridorScene, CorridorLayout);
            Configure(DefaultScene, ShinjukuLayout);
        }

        static void Configure(string path, Layout layout)
        {
            var scene = EditorSceneManager.OpenScene(path, OpenSceneMode.Single);
            var all = scene.GetRootGameObjects()
                .SelectMany(r => r.GetComponentsInChildren<Transform>(true))
                .ToList();
            Transform Find(string name) => all.FirstOrDefault(t => t.name == name)
                ?? throw new System.Exception($"{path}: object '{name}' not found");

            Find(ShinjukuRoot).gameObject.SetActive(layout.Shinjuku);
            foreach (var name in CorridorRoots)
                Find(name).gameObject.SetActive(!layout.Shinjuku);

            var vehicle = Object.FindObjectsByType<Awsim.Entity.AccelVehicle>(FindObjectsInactive.Exclude, FindObjectsSortMode.None).Single();
            vehicle.transform.SetPositionAndRotation(layout.WhillPosition, layout.WhillRotation);

            // Obstacles of the RL task (re-created from the layout)
            foreach (var old in scene.GetRootGameObjects().Where(g => g.name == TaskObstaclesRoot).ToList())
                Object.DestroyImmediate(old);
            if (layout.TaskObstacles.Length > 0)
            {
                Physics.SyncTransforms();
                var root = new GameObject(TaskObstaclesRoot).transform;
                var template = Find(CardboardTemplate).gameObject;
                foreach (var o in layout.TaskObstacles)
                {
                    var box = Object.Instantiate(template, root);
                    box.name = o.Name;
                    box.SetActive(true);
                    box.transform.localScale = o.Size;
                    float ground = Awsim.DreamerV3.GroundProbe.TryFindGround(o.X, o.Z, layout.WhillPosition.y + 5f,
                        new[] { vehicle.transform, root }, out float g) ? g : layout.WhillPosition.y;
                    box.transform.SetPositionAndRotation(new Vector3(o.X, ground + o.Size.y * 0.5f, o.Z), Quaternion.identity);
                    all.Add(box.transform);
                    Debug.Log($"[WhillSceneTools] {path}: obstacle {o.Name} at {box.transform.position} (ground {ground:F3})");
                }
            }

            var reset = Object.FindFirstObjectByType<Awsim.DreamerV3.EnvironmentReset>(FindObjectsInactive.Include);
            if (reset != null)
            {
                var so = new SerializedObject(reset);
                so.FindProperty("_goalObject").objectReferenceValue = Find(layout.Goal);
                var obstacles = so.FindProperty("_obstacles");
                obstacles.arraySize = layout.Obstacles.Length;
                for (int i = 0; i < layout.Obstacles.Length; i++)
                    obstacles.GetArrayElementAtIndex(i).objectReferenceValue = Find(layout.Obstacles[i]);
                so.FindProperty("_obstacleInitialPositions").arraySize = 0;
                so.FindProperty("_obstacleInitialRotations").arraySize = 0;
                so.FindProperty("_randomizeObstacles").boolValue = true;
                so.FindProperty("_obstacleRandomRangeMin").vector3Value = layout.ObstacleRandomMin;
                so.FindProperty("_obstacleRandomRangeMax").vector3Value = layout.ObstacleRandomMax;
                so.FindProperty("_randomizeObstacleRotation").boolValue = layout.ObstacleYawMax > layout.ObstacleYawMin;
                so.FindProperty("_obstacleRotationMin").floatValue = layout.ObstacleYawMin;
                so.FindProperty("_obstacleRotationMax").floatValue = layout.ObstacleYawMax;
                so.ApplyModifiedPropertiesWithoutUndo();
            }

            var collision = vehicle.GetComponentInChildren<Awsim.DreamerV3.CollisionPublisher>(true)
                ?? throw new System.Exception($"{path}: CollisionPublisher not found on the vehicle");
            var cso = new SerializedObject(collision);
            cso.FindProperty("_detectEnvironment").boolValue = layout.CollideWithMap;
            cso.ApplyModifiedPropertiesWithoutUndo();

            Awsim.InfraLidar.Editor.InfraLidarSetup.Setup(layout.InfraLidarConfig);

            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
                throw new System.Exception($"Failed to save {path}");
            Debug.Log($"[WhillSceneTools] {path}: layout {layout.Name}, WHILL at {layout.WhillPosition}");
        }

        const string FloorMaterial = "Assets/Awsim/Models/Entity/New Material 1.mat";
        const float FloorTop = -0.85f;   // top of the corridor floor (Unity y)
        static readonly string[] OpenFieldRemovedRoots = { "roukamap", "shougaikaihi", "goal", "goal_2", "DreamerV3Manager", "InfraLidars", TaskObstaclesRoot };

        /// <summary>
        /// Obstacle-free scene for the parameter optimization when the Shinjuku map is not
        /// available: the corridor scene without corridor, obstacles, goals, RL manager and
        /// infrastructure LiDAR, plus a flat 200 m x 200 m ground at the corridor floor height.
        /// </summary>
        [MenuItem("AWSIM for WHILL/Regenerate WhillOpenField scene (optimization)")]
        public static void CreateOpenFieldSceneBatch()
        {
            var source = EditorSceneManager.OpenScene(CorridorScene, OpenSceneMode.Single);
            if (!EditorSceneManager.SaveScene(source, OpenFieldScene, true))
                throw new System.Exception($"Failed to copy {CorridorScene} to {OpenFieldScene}");

            var scene = EditorSceneManager.OpenScene(OpenFieldScene, OpenSceneMode.Single);
            foreach (var root in scene.GetRootGameObjects().Where(g => OpenFieldRemovedRoots.Contains(g.name)))
            {
                Debug.Log($"[WhillSceneTools] removing {root.name}");
                Object.DestroyImmediate(root);
            }

            var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "OpenFieldGround";
            ground.transform.position = new Vector3(0f, FloorTop - 0.05f, 0f);
            ground.transform.localScale = new Vector3(200f, 0.1f, 200f);
            var material = AssetDatabase.LoadAssetAtPath<Material>(FloorMaterial);
            if (material != null)
                ground.GetComponent<MeshRenderer>().sharedMaterial = material;

            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
                throw new System.Exception($"Failed to save {OpenFieldScene}");
            Debug.Log($"[WhillSceneTools] wrote {OpenFieldScene}");
        }

        /// <summary>
        /// Render the WHILL from a camera placed next to it (or from above with -captureTopDown)
        /// and save a PNG (documentation / test evidence). Needs graphics: run batchmode
        /// without -nographics.
        ///
        ///   -executeMethod Awsim.WhillModelCR.Editor.WhillSceneTools.CapturePreviewBatch
        ///   [-captureScene Assets/...unity] [-captureOut path.png] [-captureTopDown 30 [-captureCenter x,z] [-captureHeight 150]]
        /// -captureTopDown: half width of the view [m]; -captureCenter: Unity (x, z) of the center (default: the WHILL).
        /// </summary>
        public static void CapturePreviewBatch()
        {
            var output = GetArg("-captureOut") ?? "docs/images/whill_model_cr_shinjuku.png";
            var scenePath = GetArg("-captureScene") ?? DefaultScene;
            var topDown = GetArg("-captureTopDown");

            EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            var vehicle = Object.FindObjectsByType<Awsim.Entity.AccelVehicle>(FindObjectsInactive.Exclude, FindObjectsSortMode.None).Single();
            var t = vehicle.transform;

            var go = new GameObject("PreviewCamera");
            var cam = go.AddComponent<Camera>();
            cam.nearClipPlane = 0.05f;
            cam.farClipPlane = 2000f;
            if (topDown != null)
            {
                // Orthographic view from above, centered on the WHILL, Unity +z up in the image.
                float halfSize = float.Parse(topDown, System.Globalization.CultureInfo.InvariantCulture);
                cam.orthographic = true;
                cam.orthographicSize = halfSize;
                var center = t.position;
                var c = GetArg("-captureCenter");
                if (c != null)
                {
                    var xz = c.Split(',').Select(v => float.Parse(v, System.Globalization.CultureInfo.InvariantCulture)).ToArray();
                    center = new Vector3(xz[0], center.y, xz[1]);
                }
                var h = GetArg("-captureHeight");   // above the WHILL [m]; raise it near skyscrapers
                go.transform.position = center + Vector3.up * (h != null ? float.Parse(h, System.Globalization.CultureInfo.InvariantCulture) : 150f);
                go.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
            }
            else
            {
                cam.fieldOfView = 50f;
                // In front of the WHILL, to its right, looking back at it.
                go.transform.position = t.position + t.forward * 1.6f + t.right * 0.9f + Vector3.up * 1.1f;
                go.transform.LookAt(t.position + Vector3.up * 0.45f);
            }

            int width = topDown != null ? 1024 : 1280, height = topDown != null ? 1024 : 720;
            var rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
            cam.targetTexture = rt;
            for (int i = 0; i < 4; i++)   // let HDRP settle its history buffers / exposure
                cam.Render();
            RenderTexture.active = rt;
            var tex = new Texture2D(width, height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            tex.Apply();
            RenderTexture.active = null;
            System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(output));
            System.IO.File.WriteAllBytes(output, tex.EncodeToPNG());
            Debug.Log($"[WhillSceneTools] captured {output} (WHILL at {t.position})");
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
