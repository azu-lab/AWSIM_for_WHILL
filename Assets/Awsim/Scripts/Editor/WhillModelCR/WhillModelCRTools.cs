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
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using Awsim.Entity;

namespace Awsim.WhillModelCR.Editor
{
    /// <summary>
    /// Editor tools for the WHILL Model CR vehicle prefab.
    ///
    /// The public repository cannot ship the original 3D model of the WHILL Model CR,
    /// so the visual part of the prefab is rebuilt from Unity primitives.
    /// The placeholder is generated from the physics components (WheelColliders and the
    /// body BoxCollider), so it always matches the simulated geometry and never feeds
    /// back into the physics: every primitive has its Collider removed.
    ///
    /// Prefab layout this tool expects:
    ///
    ///   WhillModelCR                (Rigidbody, AccelVehicle, ...)   = physics root
    ///   ├── Colliders               (WheelCollider x4, BodyCollider)
    ///   ├── VisualRoot              (no physics)
    ///   │   ├── WheelVisualFL/FR/RL/RR  (driven by AccelVehicle every frame)
    ///   │   │   └── Tire
    ///   │   └── PublicPlaceholderModel
    ///   ├── Inputs, CameraSensorScheduler, VelodyneVLP16, ...
    ///   └── RealAxle_center         (GroundTruthPublisher = base_link)
    /// </summary>
    public static class WhillModelCRTools
    {
        public const string PrefabPath = "Assets/Awsim/Prefabs/Entity/EgoVehicle/WhillModelCR/WhillModelCR.prefab";
        public const string MaterialDirectory = "Assets/Awsim/Prefabs/Entity/EgoVehicle/WhillModelCR/Materials";
        public const string VisualRootName = "VisualRoot";
        public const string PlaceholderName = "PublicPlaceholderModel";
        public const string BodyColliderName = "BodyCollider";
        const string TireName = "Tire";

        // Width of the drive wheels and the front casters [m]. Visual only.
        const float DriveWheelWidth = 0.07f;
        const float CasterWidth = 0.05f;

        static readonly string[] AxleWheelPaths =
        {
            "_frontAxle._leftWheel", "_frontAxle._rightWheel",
            "_rearAxle._leftWheel", "_rearAxle._rightWheel",
        };

        [MenuItem("AWSIM for WHILL/Rebuild WHILL Model CR placeholder model")]
        static void RebuildPlaceholderMenu()
        {
            RebuildPlaceholder(PrefabPath);
            Debug.Log($"[WhillModelCRTools] Rebuilt placeholder model in {PrefabPath}");
        }

        [MenuItem("AWSIM for WHILL/Export WHILL Model CR parameters of the open scene")]
        static void ExportParametersMenu()
        {
            var vehicle = Object.FindFirstObjectByType<AccelVehicle>();
            if (vehicle == null)
            {
                Debug.LogError("[WhillModelCRTools] No AccelVehicle in the open scene.");
                return;
            }
            var path = EditorUtility.SaveFilePanel("Export vehicle parameters", "", "whill_model_cr_parameters.json", "json");
            if (string.IsNullOrEmpty(path))
                return;
            File.WriteAllText(path, ExportVehicleParameters(vehicle.gameObject));
            Debug.Log($"[WhillModelCRTools] Exported {path}");
        }

        /// <summary>
        /// Batchmode entry point: rebuild the placeholder of the prefab at <see cref="PrefabPath"/>.
        /// </summary>
        public static void RebuildPlaceholderBatch()
        {
            RebuildPlaceholder(PrefabPath);
        }

        /// <summary>
        /// Batchmode entry point: export the parameters of every AccelVehicle in a scene.
        ///
        ///   -executeMethod Awsim.WhillModelCR.Editor.WhillModelCRTools.ExportParametersBatch
        ///   -whillScene Assets/...unity -whillOut /path/to/out.json
        /// </summary>
        public static void ExportParametersBatch()
        {
            var scene = GetArg("-whillScene");
            var output = GetArg("-whillOut");
            EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            var vehicles = Object.FindObjectsByType<AccelVehicle>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .Where(v => v.gameObject.activeInHierarchy)
                .OrderBy(v => v.name)
                .ToArray();
            var sb = new StringBuilder();
            sb.Append("{\n");
            for (int i = 0; i < vehicles.Length; i++)
            {
                sb.Append($"\"{Escape(vehicles[i].name)}\": ");
                sb.Append(ExportVehicleParameters(vehicles[i].gameObject));
                sb.Append(i + 1 < vehicles.Length ? ",\n" : "\n");
            }
            sb.Append("}\n");
            File.WriteAllText(output, sb.ToString());
            Debug.Log($"[WhillModelCRTools] Exported {vehicles.Length} vehicle(s) to {output}");
        }

        public static void RebuildPlaceholder(string prefabPath)
        {
            var root = PrefabUtility.LoadPrefabContents(prefabPath);
            try
            {
                BuildPlaceholder(root);
                PrefabUtility.SaveAsPrefabAsset(root, prefabPath);
            }
            finally
            {
                PrefabUtility.UnloadPrefabContents(root);
            }
        }

        /// <summary>
        /// (Re)create the primitive-based visual model under VisualRoot.
        /// Only the visual hierarchy is touched; physics components are read, never written.
        /// </summary>
        public static void BuildPlaceholder(GameObject root)
        {
            var vehicle = root.GetComponent<AccelVehicle>();
            if (vehicle == null)
                throw new System.InvalidOperationException($"{root.name} has no AccelVehicle.");

            var visualRoot = root.transform.Find(VisualRootName);
            if (visualRoot == null)
                throw new System.InvalidOperationException($"{root.name} has no child named {VisualRootName}.");

            var bodyCollider = root.GetComponentsInChildren<BoxCollider>(true)
                .FirstOrDefault(b => b.name == BodyColliderName);
            if (bodyCollider == null)
                throw new System.InvalidOperationException($"{root.name} has no BoxCollider named {BodyColliderName}.");

            var bodyMaterial = GetOrCreateMaterial("WhillPlaceholderBody", new Color(0.18f, 0.20f, 0.23f));
            var seatMaterial = GetOrCreateMaterial("WhillPlaceholderSeat", new Color(0.05f, 0.05f, 0.06f));
            var tireMaterial = GetOrCreateMaterial("WhillPlaceholderTire", new Color(0.08f, 0.08f, 0.08f));
            var accentMaterial = GetOrCreateMaterial("WhillPlaceholderAccent", new Color(0.10f, 0.65f, 0.85f));

            visualRoot.localPosition = Vector3.zero;
            visualRoot.localRotation = Quaternion.identity;
            visualRoot.localScale = Vector3.one;

            var old = visualRoot.Find(PlaceholderName);
            if (old != null)
                Object.DestroyImmediate(old.gameObject);

            var placeholder = new GameObject(PlaceholderName).transform;
            placeholder.SetParent(visualRoot, false);

            // --- Body: proportions taken from the body collider (root space). ---
            var rootTf = root.transform;
            var bodyCenter = rootTf.InverseTransformPoint(bodyCollider.transform.TransformPoint(bodyCollider.center));
            var bodySize = Vector3.Scale(bodyCollider.size, Abs(bodyCollider.transform.lossyScale));
            bodySize = Vector3.Scale(bodySize, Inverse(Abs(rootTf.lossyScale)));
            float bottom = bodyCenter.y - bodySize.y * 0.5f;
            float top = bodyCenter.y + bodySize.y * 0.5f;
            float width = bodySize.x;
            float length = bodySize.z;
            float h = top - bottom;

            Box(placeholder, "Chassis", bodyMaterial,
                new Vector3(bodyCenter.x, bottom + h * 0.25f, bodyCenter.z),
                new Vector3(width * 0.80f, h * 0.20f, length * 0.80f));
            Box(placeholder, "SeatPost", bodyMaterial,
                new Vector3(bodyCenter.x, bottom + h * 0.43f, bodyCenter.z - length * 0.05f),
                new Vector3(width * 0.22f, h * 0.18f, length * 0.12f));
            Box(placeholder, "SeatCushion", seatMaterial,
                new Vector3(bodyCenter.x, bottom + h * 0.55f, bodyCenter.z - length * 0.04f),
                new Vector3(width * 0.84f, h * 0.10f, length * 0.46f));
            Box(placeholder, "Backrest", seatMaterial,
                new Vector3(bodyCenter.x, bottom + h * 0.80f, bodyCenter.z - length * 0.27f),
                new Vector3(width * 0.84f, h * 0.45f, length * 0.07f));
            Box(placeholder, "ArmrestLeft", bodyMaterial,
                new Vector3(bodyCenter.x - width * 0.45f, bottom + h * 0.72f, bodyCenter.z - length * 0.02f),
                new Vector3(width * 0.10f, h * 0.05f, length * 0.42f));
            Box(placeholder, "ArmrestRight", bodyMaterial,
                new Vector3(bodyCenter.x + width * 0.45f, bottom + h * 0.72f, bodyCenter.z - length * 0.02f),
                new Vector3(width * 0.10f, h * 0.05f, length * 0.42f));
            Box(placeholder, "Controller", accentMaterial,
                new Vector3(bodyCenter.x + width * 0.45f, bottom + h * 0.78f, bodyCenter.z + length * 0.17f),
                new Vector3(width * 0.10f, h * 0.07f, length * 0.08f));
            Box(placeholder, "Footrest", bodyMaterial,
                new Vector3(bodyCenter.x, bottom + h * 0.08f, bodyCenter.z + length * 0.42f),
                new Vector3(width * 0.65f, h * 0.05f, length * 0.16f));
            // Small marker on the front so that the heading is visible in the viewer.
            Box(placeholder, "FrontMarker", accentMaterial,
                new Vector3(bodyCenter.x, bottom + h * 0.36f, bodyCenter.z + length * 0.40f),
                new Vector3(width * 0.30f, h * 0.04f, length * 0.03f));

            // --- Wheels: one cylinder per WheelCollider, under the visual transform AccelVehicle drives. ---
            var so = new SerializedObject(vehicle);
            foreach (var path in AxleWheelPaths)
            {
                var wheelCollider = so.FindProperty(path + "._wheelCollider").objectReferenceValue as WheelCollider;
                var visual = so.FindProperty(path + "._wheelVisualTransform").objectReferenceValue as Transform;
                if (wheelCollider == null || visual == null)
                    throw new System.InvalidOperationException($"{path} has no WheelCollider or visual transform.");

                foreach (var mf in visual.GetComponents<MeshFilter>())
                    Object.DestroyImmediate(mf);
                foreach (var mr in visual.GetComponents<MeshRenderer>())
                    Object.DestroyImmediate(mr);
                for (int i = visual.childCount - 1; i >= 0; i--)
                    Object.DestroyImmediate(visual.GetChild(i).gameObject);

                // Rest pose of the wheel. At runtime AccelVehicle moves it to WheelCollider.GetWorldPose().
                visual.position = wheelCollider.transform.TransformPoint(wheelCollider.center);
                visual.localRotation = Quaternion.identity;
                visual.localScale = Vector3.one;

                bool isRear = path.StartsWith("_rearAxle");
                float radius = wheelCollider.radius * Mathf.Abs(wheelCollider.transform.lossyScale.y);
                float tireWidth = isRear ? DriveWheelWidth : CasterWidth;
                // Unity's cylinder is 1 m in diameter and 2 m tall along +Y; lay it along X (axle).
                var tire = Primitive(PrimitiveType.Cylinder, TireName, tireMaterial, visual);
                tire.localPosition = Vector3.zero;
                tire.localRotation = Quaternion.Euler(0f, 0f, 90f);
                tire.localScale = new Vector3(radius * 2f, tireWidth * 0.5f, radius * 2f);
            }
        }

        static Transform Box(Transform parent, string name, Material material, Vector3 localPosition, Vector3 localScale)
        {
            var t = Primitive(PrimitiveType.Cube, name, material, parent);
            t.localPosition = localPosition;
            t.localRotation = Quaternion.identity;
            t.localScale = localScale;
            return t;
        }

        static Transform Primitive(PrimitiveType type, string name, Material material, Transform parent)
        {
            var go = GameObject.CreatePrimitive(type);
            go.name = name;
            // The placeholder is visual only. A collider here would change the vehicle physics.
            foreach (var c in go.GetComponents<Collider>())
                Object.DestroyImmediate(c);
            go.GetComponent<MeshRenderer>().sharedMaterial = material;
            go.transform.SetParent(parent, false);
            return go.transform;
        }

        static Material GetOrCreateMaterial(string name, Color color)
        {
            var path = $"{MaterialDirectory}/{name}.mat";
            var material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material != null)
                return material;

            if (!AssetDatabase.IsValidFolder(MaterialDirectory))
            {
                var parent = Path.GetDirectoryName(MaterialDirectory).Replace('\\', '/');
                AssetDatabase.CreateFolder(parent, Path.GetFileName(MaterialDirectory));
            }

            var shader = Shader.Find("HDRP/Lit") ?? Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            material = new Material(shader) { name = name };
            material.SetColor("_BaseColor", color);
            material.SetColor("_Color", color);
            material.SetFloat("_Smoothness", 0.3f);
            AssetDatabase.CreateAsset(material, path);
            return material;
        }

        // ----------------------------------------------------------------------------------
        // Parameter export (used to prove that the visual swap did not change the physics)
        // ----------------------------------------------------------------------------------

        /// <summary>
        /// Serialize every physics-relevant value of a vehicle into JSON:
        /// AccelVehicle, Rigidbody, WheelColliders, BoxColliders and the pose of the
        /// sensor / base_link transforms relative to the vehicle root.
        /// </summary>
        public static string ExportVehicleParameters(GameObject root)
        {
            var sb = new StringBuilder();
            sb.Append("{\n");
            var sections = new List<string>();

            var vehicle = root.GetComponent<AccelVehicle>();
            if (vehicle != null)
                sections.Add($"  \"AccelVehicle\": {DumpSerialized(vehicle)}");
            var rb = root.GetComponent<Rigidbody>();
            if (rb != null)
                sections.Add($"  \"Rigidbody\": {DumpSerialized(rb)}");

            var rootTf = root.transform;
            foreach (var wc in root.GetComponentsInChildren<WheelCollider>(true).OrderBy(w => w.name))
            {
                sections.Add($"  \"WheelCollider/{Escape(wc.name)}\": {DumpSerialized(wc)}");
                sections.Add($"  \"Pose/{Escape(wc.name)}\": {DumpPose(rootTf, wc.transform)}");
                sections.Add($"  \"WheelRadiusWorld/{Escape(wc.name)}\": {F(wc.radius * Mathf.Abs(wc.transform.lossyScale.y))}");
            }
            foreach (var box in root.GetComponentsInChildren<BoxCollider>(true).OrderBy(b => b.name))
            {
                if (IsUnder(box.transform, VisualRootName, rootTf))
                    continue;
                sections.Add($"  \"BoxCollider/{Escape(box.name)}\": {DumpSerialized(box)}");
                sections.Add($"  \"Pose/{Escape(box.name)}\": {DumpPose(rootTf, box.transform)}");
            }
            foreach (var mb in root.GetComponentsInChildren<MonoBehaviour>(true).OrderBy(m => m.GetType().Name))
            {
                if (mb == null || mb is AccelVehicle)
                    continue;
                var typeName = mb.GetType().Name;
                bool isSensorLike = typeName.Contains("Publisher") || typeName.Contains("Sensor")
                    || typeName.Contains("Bridge") || typeName.Contains("Input");
                if (!isSensorLike)
                    continue;
                sections.Add($"  \"Component/{Escape(typeName)}@{Escape(mb.name)}\": {DumpSerialized(mb)}");
                sections.Add($"  \"Pose/{Escape(typeName)}@{Escape(mb.name)}\": {DumpPose(rootTf, mb.transform)}");
            }
            var colliders = root.GetComponentsInChildren<Collider>(true)
                .Where(c => !(c is WheelCollider))
                .Select(c => $"\"{Escape(PathOf(rootTf, c.transform))}:{c.GetType().Name}\"")
                .OrderBy(s => s);
            sections.Add($"  \"AllNonWheelColliders\": [{string.Join(", ", colliders)}]");

            sb.Append(string.Join(",\n", sections));
            sb.Append("\n}");
            return sb.ToString();
        }

        static string DumpSerialized(Object obj)
        {
            var so = new SerializedObject(obj);
            var it = so.GetIterator();
            var items = new List<string>();
            bool enterChildren = true;
            while (it.Next(enterChildren))
            {
                enterChildren = true;
                var p = it.propertyPath;
                if (p.StartsWith("m_Script") || p.StartsWith("m_ObjectHideFlags") || p.StartsWith("m_GameObject")
                    || p.StartsWith("m_CorrespondingSourceObject") || p.StartsWith("m_PrefabInstance")
                    || p.StartsWith("m_PrefabAsset") || p == "m_EditorHideFlags" || p == "m_EditorClassIdentifier"
                    || p == "m_Name")
                {
                    enterChildren = false;
                    continue;
                }
                string v = null;
                switch (it.propertyType)
                {
                    case SerializedPropertyType.Float: v = F(it.floatValue); break;
                    case SerializedPropertyType.Integer: v = it.longValue.ToString(CultureInfo.InvariantCulture); break;
                    case SerializedPropertyType.Boolean: v = it.boolValue ? "true" : "false"; break;
                    case SerializedPropertyType.Enum: v = it.intValue.ToString(CultureInfo.InvariantCulture); break;
                    case SerializedPropertyType.String:
                        v = $"\"{Escape(it.stringValue)}\"";
                        enterChildren = false;   // do not expand the characters of the string
                        break;
                    case SerializedPropertyType.LayerMask: v = it.intValue.ToString(CultureInfo.InvariantCulture); break;
                    case SerializedPropertyType.ObjectReference:
                        v = it.objectReferenceValue == null ? "null" : $"\"{Escape(it.objectReferenceValue.name)}\"";
                        enterChildren = false;
                        break;
                    case SerializedPropertyType.AnimationCurve:
                        v = $"\"keys={it.animationCurveValue.length}\"";
                        enterChildren = false;
                        break;
                }
                if (v != null)
                    items.Add($"\"{Escape(p)}\": {v}");
            }
            return "{" + string.Join(", ", items) + "}";
        }

        static string DumpPose(Transform root, Transform t)
        {
            var p = root.InverseTransformPoint(t.position);
            var r = (Quaternion.Inverse(root.rotation) * t.rotation).eulerAngles;
            return $"{{\"position\": [{F(p.x)}, {F(p.y)}, {F(p.z)}], \"euler\": [{F(r.x)}, {F(r.y)}, {F(r.z)}]}}";
        }

        static bool IsUnder(Transform t, string ancestorName, Transform stopAt)
        {
            for (var p = t; p != null && p != stopAt; p = p.parent)
                if (p.name == ancestorName)
                    return true;
            return false;
        }

        static string PathOf(Transform root, Transform t)
        {
            var names = new List<string>();
            for (var p = t; p != null && p != root; p = p.parent)
                names.Add(p.name);
            names.Reverse();
            return string.Join("/", names);
        }

        static string F(float v) => v.ToString("R", CultureInfo.InvariantCulture);
        static string Escape(string s) => s.Replace("\\", "\\\\").Replace("\"", "\\\"");
        static Vector3 Abs(Vector3 v) => new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
        static Vector3 Inverse(Vector3 v) => new Vector3(1f / v.x, 1f / v.y, 1f / v.z);

        static string GetArg(string name)
        {
            var args = System.Environment.GetCommandLineArgs();
            for (int i = 0; i < args.Length - 1; i++)
                if (args[i] == name)
                    return args[i + 1];
            throw new System.ArgumentException($"Missing command line argument {name}");
        }
    }
}
