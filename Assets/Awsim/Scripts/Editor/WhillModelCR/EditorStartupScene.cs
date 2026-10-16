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

using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Awsim.WhillModelCR.Editor
{
    /// <summary>
    /// Opens the scene given as <c>-awsimScene Assets/....unity</c> when the Editor starts
    /// (used by scripts/open_editor.sh; Unity's own -openfile does not open a scene of a project).
    /// Only once per Editor session, so that a script reload does not switch the scene back.
    /// </summary>
    [InitializeOnLoad]
    static class EditorStartupScene
    {
        const string Arg = "-awsimScene";
        const string DoneKey = "AwsimForWhill.StartupSceneOpened";

        static EditorStartupScene()
        {
            if (Application.isBatchMode || SessionState.GetBool(DoneKey, false))
                return;
            var scene = GetArg(Arg);
            if (string.IsNullOrEmpty(scene))
                return;
            SessionState.SetBool(DoneKey, true);
            EditorApplication.delayCall += () =>
            {
                if (AssetDatabase.LoadAssetAtPath<SceneAsset>(scene) == null)
                {
                    Debug.LogError($"[EditorStartupScene] Scene not found: {scene}");
                    return;
                }
                if (EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                {
                    EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
                    Debug.Log($"[EditorStartupScene] Opened {scene}");
                }
            };
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
