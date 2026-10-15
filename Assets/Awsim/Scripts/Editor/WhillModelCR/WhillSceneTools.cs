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
    /// Generates the obstacle-free scene used by the vehicle parameter optimization.
    ///
    /// The real-WHILL scenarios (straight 4.6 m, turn, circle, S-curve, lemniscate) need open
    /// space. In the corridor scene the WHILL hits the cardboard box and the walls, so the
    /// optimization would fit collisions instead of the vehicle dynamics. soturon ran its
    /// optimization before the corridor existed, on the (non-redistributable) Shinjuku map.
    ///
    /// The open-field scene is the corridor scene with the corridor, the obstacles, the goal
    /// objects, the RL manager and the infrastructure LiDAR removed and a flat 200 m x 200 m
    /// ground added at the height of the corridor floor. Everything else (WHILL prefab,
    /// clock, UI, OptimizerBridge on the WHILL) is identical.
    ///
    ///   -executeMethod Awsim.WhillModelCR.Editor.WhillSceneTools.CreateOpenFieldSceneBatch
    /// </summary>
    public static class WhillSceneTools
    {
        public const string CorridorScene = "Assets/Awsim/Scenes/AutowareSimulationDemo.unity";
        public const string OpenFieldScene = "Assets/Awsim/Scenes/WhillOpenField.unity";
        static readonly string[] RemovedRoots = { "roukamap", "shougaikaihi", "goal", "goal_2", "DreamerV3Manager", "InfraLidars" };
        const string FloorMaterial = "Assets/Awsim/Models/Entity/New Material 1.mat";
        const float FloorTop = -0.85f;   // top of the corridor floor (Unity y)

        [MenuItem("AWSIM for WHILL/Regenerate WhillOpenField scene (optimization)")]
        public static void CreateOpenFieldSceneBatch()
        {
            var source = EditorSceneManager.OpenScene(CorridorScene, OpenSceneMode.Single);
            if (!EditorSceneManager.SaveScene(source, OpenFieldScene, true))
                throw new System.Exception($"Failed to copy {CorridorScene} to {OpenFieldScene}");

            var scene = EditorSceneManager.OpenScene(OpenFieldScene, OpenSceneMode.Single);
            foreach (var root in scene.GetRootGameObjects().Where(g => RemovedRoots.Contains(g.name)))
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
        /// Render the WHILL of the corridor scene from a camera placed next to it and save a PNG
        /// (documentation / test evidence). Needs graphics: run batchmode without -nographics.
        ///
        ///   -executeMethod Awsim.WhillModelCR.Editor.WhillSceneTools.CapturePreviewBatch -captureOut path.png
        /// </summary>
        public static void CapturePreviewBatch()
        {
            var output = "docs/images/whill_model_cr_placeholder.png";
            var args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i] == "-captureOut")
                    output = args[i + 1];

            EditorSceneManager.OpenScene(CorridorScene, OpenSceneMode.Single);
            var vehicle = Object.FindObjectsByType<Awsim.Entity.AccelVehicle>(FindObjectsInactive.Exclude, FindObjectsSortMode.None).Single();
            var t = vehicle.transform;

            var go = new GameObject("PreviewCamera");
            var cam = go.AddComponent<Camera>();
            cam.fieldOfView = 50f;
            cam.nearClipPlane = 0.05f;
            // In front of the WHILL, to its right, looking back at it.
            go.transform.position = t.position + t.forward * 1.6f + t.right * 0.9f + Vector3.up * 1.1f;
            go.transform.LookAt(t.position + Vector3.up * 0.45f);

            const int width = 1280, height = 720;
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
            Debug.Log($"[WhillSceneTools] captured {output}");
        }
    }
}
