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
using System.IO;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Awsim.WhillModelCR.Editor
{
    /// <summary>
    /// Checks a scene for missing scripts, missing prefabs, broken object references
    /// and renderers without mesh/material, and writes a plain-text report.
    ///
    /// Problems on inactive objects are reported separately: the upstream scene keeps
    /// the (disabled) Shinjuku map, whose meshes come from the AWSIM "Externals"
    /// download and are not part of this repository.
    ///
    ///   Unity -batchmode -quit -projectPath . \
    ///     -executeMethod Awsim.WhillModelCR.Editor.ProjectValidator.ValidateBatch \
    ///     -validateScene Assets/Awsim/Scenes/AutowareSimulationDemo.unity -validateOut report.txt
    ///
    /// The process exits with code 1 when an active object has a problem.
    /// </summary>
    public static class ProjectValidator
    {
        class Report
        {
            public readonly List<string> Active = new List<string>();
            public readonly List<string> Inactive = new List<string>();
            public void Add(GameObject go, string message)
            {
                (go.activeInHierarchy ? Active : Inactive).Add($"{PathOf(go)}: {message}");
            }
        }

        [MenuItem("AWSIM for WHILL/Validate open scene")]
        static void ValidateMenu()
        {
            var text = Validate(EditorSceneManager.GetActiveScene(), out var activeProblems);
            Debug.Log(text);
            if (activeProblems > 0)
                Debug.LogError($"[ProjectValidator] {activeProblems} problem(s) on active objects");
        }

        public static void ValidateBatch()
        {
            var scenePath = GetArg("-validateScene") ?? "Assets/Awsim/Scenes/AutowareSimulationDemo.unity";
            var output = GetArg("-validateOut") ?? "validation_report.txt";
            var scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
            var text = Validate(scene, out var activeProblems);
            text += "\n" + ValidatePrefabs("Assets/Awsim/Prefabs/Entity/EgoVehicle/WhillModelCR", out var prefabProblems);
            File.WriteAllText(output, text);
            Debug.Log(text);
            if (activeProblems + prefabProblems > 0)
                EditorApplication.Exit(1);
        }

        public static string Validate(UnityEngine.SceneManagement.Scene scene, out int activeProblems)
        {
            var report = new Report();
            var all = scene.GetRootGameObjects()
                .SelectMany(r => r.GetComponentsInChildren<Transform>(true))
                .Select(t => t.gameObject)
                .ToList();

            foreach (var go in all)
            {
                int missing = GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(go);
                if (missing > 0)
                    report.Add(go, $"{missing} missing script(s)");

                if (PrefabUtility.IsPrefabAssetMissing(go))
                    report.Add(go, "missing prefab asset");

                var mf = go.GetComponent<MeshFilter>();
                var mr = go.GetComponent<MeshRenderer>();
                if (mf != null && mr != null && mr.enabled && mf.sharedMesh == null)
                    report.Add(go, "MeshFilter without mesh");
                if (mr != null && mr.enabled && mr.sharedMaterials.Any(m => m == null))
                    report.Add(go, "MeshRenderer with missing material");

                foreach (var component in go.GetComponents<Component>())
                {
                    if (component == null)
                        continue;
                    var so = new SerializedObject(component);
                    var it = so.GetIterator();
                    while (it.Next(true))
                    {
                        if (it.propertyType != SerializedPropertyType.ObjectReference)
                            continue;
                        // A reference that points to something (instance id != 0) which cannot be loaded.
                        if (it.objectReferenceValue == null && it.objectReferenceInstanceIDValue != 0)
                            report.Add(go, $"{component.GetType().Name}.{it.propertyPath}: broken reference");
                    }
                }
            }

            activeProblems = report.Active.Count;
            var sb = new StringBuilder();
            sb.AppendLine($"Scene: {scene.path}");
            sb.AppendLine($"GameObjects: {all.Count} (active {all.Count(g => g.activeInHierarchy)})");
            sb.AppendLine($"Problems on ACTIVE objects: {report.Active.Count}");
            foreach (var line in report.Active)
                sb.AppendLine("  " + line);
            sb.AppendLine($"Problems on INACTIVE objects: {report.Inactive.Count}");
            foreach (var group in report.Inactive
                         .GroupBy(l => string.Join("/", l.Split(':')[0].Split('/').Take(3)) + " | " + l.Substring(l.LastIndexOf(':') + 2))
                         .OrderBy(g => g.Key))
                sb.AppendLine($"  {group.Key}: {group.Count()}");
            return sb.ToString();
        }

        static string ValidatePrefabs(string directory, out int problems)
        {
            var sb = new StringBuilder();
            problems = 0;
            foreach (var guid in AssetDatabase.FindAssets("t:Prefab", new[] { directory }))
            {
                var path = AssetDatabase.GUIDToAssetPath(guid);
                var root = PrefabUtility.LoadPrefabContents(path);
                try
                {
                    int missing = root.GetComponentsInChildren<Transform>(true)
                        .Sum(t => GameObjectUtility.GetMonoBehavioursWithMissingScriptCount(t.gameObject));
                    int noMesh = root.GetComponentsInChildren<MeshFilter>(true).Count(m => m.sharedMesh == null);
                    int noMat = root.GetComponentsInChildren<MeshRenderer>(true).Count(r => r.sharedMaterials.Any(m => m == null));
                    sb.AppendLine($"Prefab: {path}: missing scripts={missing}, meshes={noMesh}, materials={noMat}");
                    problems += missing + noMesh + noMat;
                }
                finally
                {
                    PrefabUtility.UnloadPrefabContents(root);
                }
            }
            return sb.ToString();
        }

        static string PathOf(GameObject go)
        {
            var names = new List<string>();
            for (var t = go.transform; t != null; t = t.parent)
                names.Add(t.name);
            names.Reverse();
            return string.Join("/", names);
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
