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

using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace Awsim.WhillModelCR.Editor
{
    /// <summary>
    /// Builds the Linux standalone player (used by scripts/build_player.sh).
    ///
    ///   AWSIM_BUILD_SCENE   scene to build (default: the corridor scene)
    ///   AWSIM_BUILD_OUTPUT  executable path (default: Builds/AWSIM_for_WHILL/AWSIM_for_WHILL.x86_64)
    ///   AWSIM_BUILD_DEVELOPMENT=1  development build
    /// </summary>
    public static class WhillPlayerBuilder
    {
        const string DefaultScene = "Assets/Awsim/Scenes/AutowareSimulationDemo.unity";
        const string DefaultOutput = "Builds/AWSIM_for_WHILL/AWSIM_for_WHILL.x86_64";

        public static void BuildLinux64()
        {
            var scene = EnvOr("AWSIM_BUILD_SCENE", DefaultScene);
            var output = EnvOr("AWSIM_BUILD_OUTPUT", DefaultOutput);
            var development = Environment.GetEnvironmentVariable("AWSIM_BUILD_DEVELOPMENT") == "1";

            if (!File.Exists(scene))
                Fail($"Scene not found: {scene}");

            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(output)));
            var options = new BuildPlayerOptions
            {
                scenes = new[] { scene },
                locationPathName = output,
                target = BuildTarget.StandaloneLinux64,
                targetGroup = BuildTargetGroup.Standalone,
                options = development ? BuildOptions.Development : BuildOptions.None,
            };

            Debug.Log($"[WhillPlayerBuilder] scene={scene} output={output} development={development}");
            var report = BuildPipeline.BuildPlayer(options);
            var summary = report.summary;
            Debug.Log($"[WhillPlayerBuilder] result={summary.result} size={summary.totalSize / (1024 * 1024)}MB time={summary.totalTime}");

            if (summary.result != BuildResult.Succeeded)
            {
                foreach (var step in report.steps)
                    foreach (var msg in step.messages)
                        if (msg.type == LogType.Error || msg.type == LogType.Exception)
                            Debug.LogError($"[WhillPlayerBuilder] {step.name}: {msg.content}");
                Fail($"Build failed: {summary.result}");
            }
            EditorApplication.Exit(0);
        }

        static string EnvOr(string name, string fallback)
        {
            var v = Environment.GetEnvironmentVariable(name);
            return string.IsNullOrEmpty(v) ? fallback : v;
        }

        static void Fail(string message)
        {
            Debug.LogError("[WhillPlayerBuilder] " + message);
            EditorApplication.Exit(1);
        }
    }
}
