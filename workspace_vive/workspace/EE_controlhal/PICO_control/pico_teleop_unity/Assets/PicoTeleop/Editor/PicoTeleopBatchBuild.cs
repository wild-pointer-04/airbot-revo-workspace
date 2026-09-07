using System;
using System.Linq;
using UnityEditor;
using UnityEditor.Build.Reporting;
using UnityEngine;

namespace EgoARX.PicoTeleop.Editor
{
    public static class PicoTeleopBatchBuild
    {
        public static void BuildAndroid()
        {
            const string output = "Builds/PicoTeleopUE.apk";
            var scenes = EditorBuildSettings.scenes
                .Where(scene => scene.enabled)
                .Select(scene => scene.path)
                .ToArray();

            if (scenes.Length == 0)
            {
                throw new InvalidOperationException("No enabled scenes in EditorBuildSettings.");
            }

            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
            PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.productName = "PicoTeleopUE";
            PlayerSettings.SetApplicationIdentifier(
                BuildTargetGroup.Android,
                "com.egoarx.picoteleopue");
            PlayerSettings.SetScriptingBackend(
                BuildTargetGroup.Android,
                ScriptingImplementation.IL2CPP);

            var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
            {
                scenes = scenes,
                locationPathName = output,
                target = BuildTarget.Android,
                options = BuildOptions.CleanBuildCache,
            });

            if (report.summary.result != BuildResult.Succeeded)
            {
                throw new Exception(
                    $"Android build failed: {report.summary.result}, " +
                    $"errors={report.summary.totalErrors}");
            }

            Debug.Log($"[PicoTeleop] APK built: {output} ({report.summary.totalSize} bytes)");
        }
    }
}
