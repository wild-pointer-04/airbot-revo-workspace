using EgoARX.PicoTeleop.Core;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace EgoARX.PicoTeleop.EditorTools
{
    public static class PicoTeleopEditorMenu
    {
        const string ScenePath = "Assets/PicoTeleop/Scenes/PicoTeleop.unity";

        [MenuItem("PicoTeleop/Fix Android Player Settings")]
        public static void FixAndroidPlayerSettings()
        {
            PlayerSettings.colorSpace = ColorSpace.Linear;
            PlayerSettings.defaultInterfaceOrientation = UIOrientation.LandscapeLeft;
            PlayerSettings.Android.minSdkVersion = AndroidSdkVersions.AndroidApiLevel29;
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.Android, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.Android, new[] { GraphicsDeviceType.Vulkan });
            PlayerSettings.Android.forceSDCardPermission = false;
            PlayerSettings.Android.targetArchitectures = AndroidArchitecture.ARM64;
            PlayerSettings.SetScriptingBackend(BuildTargetGroup.Android, ScriptingImplementation.IL2CPP);
            Debug.Log("[PicoTeleop] LandscapeLeft, Linear, Vulkan, ARM64/IL2CPP. Build again.");
        }

        [MenuItem("PicoTeleop/Add Manager To Scene")]
        public static void AddManagerToScene()
        {
            var existing =
#if UNITY_2023_1_OR_NEWER
                Object.FindFirstObjectByType<PicoTeleopManager>();
#else
                Object.FindObjectOfType<PicoTeleopManager>();
#endif
            if (existing != null)
            {
                Selection.activeGameObject = existing.gameObject;
                Debug.Log("[PicoTeleop] Manager already exists in the scene.");
                return;
            }

            var go = new GameObject("PicoTeleopManager");
            go.AddComponent<PicoTeleopManager>();
            Undo.RegisterCreatedObjectUndo(go, "Add PicoTeleopManager");
            Selection.activeGameObject = go;
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
        }

        [MenuItem("PicoTeleop/Print XR Setup Checklist")]
        public static void PrintChecklist()
        {
            Debug.Log(
                "[PicoTeleop] XR setup checklist:\n" +
                "1. File > Build Settings > Android, Switch Platform\n" +
                "2. Player: IL2CPP, ARM64, Minimum API 29\n" +
                "3. XR Plug-in Management > Android > enable OpenXR (not PICO XR loader)\n" +
                "4. OpenXR > Android: enable PICO XR Support, PICO4 and PICO4 Ultra controller profiles\n" +
                "5. Integration SDK auto-sets scripting define PICO_OPENXR_SDK when OpenXR loader is active\n" +
                "6. Select PicoTeleopManager and set Receiver IP to the Ubuntu LAN address\n" +
                "7. Do not import PICO-Unity-OpenXR-SDK-main into this project\n" +
                $"8. Open scene: {ScenePath}");
        }
    }
}
