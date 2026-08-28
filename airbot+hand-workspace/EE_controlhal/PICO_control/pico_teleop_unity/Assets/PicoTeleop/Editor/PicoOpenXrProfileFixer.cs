using UnityEditor;
using UnityEditor.XR.OpenXR.Features;
using UnityEngine;
using UnityEngine.XR.OpenXR;
using UnityEngine.XR.OpenXR.Features;

namespace EgoARX.PicoTeleop.EditorTools
{
    /// <summary>
    /// PICO controller profiles only register after PICO_OPENXR_SDK compiles.
    /// This refreshes OpenXR feature sets so they appear in Project Settings.
    /// </summary>
    public static class PicoOpenXrProfileFixer
    {
        const string PicoFeatureSetId = "com.picoxr.openxr.features";

        static readonly string[] PicoProfileIds =
        {
            "com.unity.openxr.feature.input.PICO4touch",
            "com.unity.openxr.feature.input.PICO4Ultratouch",
        };

        static readonly string[] OptionalFeatureIdsToDisable =
        {
            "com.pico.openxr.feature.scenecapture",
            "com.pico.openxr.feature.spatialanchor",
            "com.pico.openxr.feature.spatialmesh",
            "com.pico.openxr.feature.passthrough",
            "com.pico.openxr.feature.PICO_BodyTracking",
            "com.pico.openxr.feature.foveation",
            "com.pico.openxr.feature.LayerSecureContent",
        };

        [InitializeOnLoadMethod]
        static void Init()
        {
            EditorApplication.delayCall += TryEnable;
        }

        [MenuItem("PicoTeleop/Enable PICO OpenXR Profiles")]
        public static void EnableFromMenu()
        {
            TryEnable();
        }

        static void TryEnable()
        {
            FeatureHelpers.RefreshFeatures(BuildTargetGroup.Android);

            var sets = OpenXRFeatureSetManager.FeatureSetsForBuildTarget(BuildTargetGroup.Android);
            if (sets == null || sets.Count == 0)
            {
                Debug.LogWarning("[PicoTeleop] No OpenXR feature sets for Android. Restart the Unity Editor.");
                return;
            }

            var pico = OpenXRFeatureSetManager.GetFeatureSetWithId(BuildTargetGroup.Android, PicoFeatureSetId);
            if (pico == null)
            {
                Debug.LogWarning(
                    "[PicoTeleop] PICO XR feature set is not registered. Close Unity completely and reopen this project.");
                return;
            }

            pico.isEnabled = true;
            OpenXRFeatureSetManager.SetFeaturesFromEnabledFeatureSets(BuildTargetGroup.Android);

            var found = FeatureHelpers.GetFeaturesWithIdsForBuildTarget(BuildTargetGroup.Android, PicoProfileIds);
            if (found != null)
            {
                foreach (OpenXRFeature feature in found)
                {
                    if (feature == null)
                    {
                        continue;
                    }

                    feature.enabled = true;
                }
            }

            var optional = FeatureHelpers.GetFeaturesWithIdsForBuildTarget(
                BuildTargetGroup.Android, OptionalFeatureIdsToDisable);
            if (optional != null)
            {
                foreach (OpenXRFeature feature in optional)
                {
                    if (feature == null)
                    {
                        continue;
                    }

                    if (feature.enabled)
                    {
                        feature.enabled = false;
                        Debug.Log($"[PicoTeleop] Disabled optional OpenXR feature {feature.GetType().Name}");
                    }
                }
            }

            var settings = OpenXRSettings.GetSettingsForBuildTargetGroup(BuildTargetGroup.Android);
            if (settings != null)
            {
                EditorUtility.SetDirty(settings);
            }

            AssetDatabase.SaveAssets();
            Debug.Log("[PicoTeleop] PICO OpenXR profiles enabled (MR/passthrough/body/foveation off).");
        }
    }
}
