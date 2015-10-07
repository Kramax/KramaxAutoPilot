using System;
using System.Collections;
using UnityEngine;

namespace Kramax.Toolbar
{
    using Utility;

    public static class AppLauncherAutoPilot
    {
        private static ApplicationLauncherButton btnLauncher;
        
        public static void Awake()
        {
        }

        public static void Start()
        {
            if (btnLauncher == null)
                btnLauncher =
                    ApplicationLauncher.Instance.AddModApplication(OnToggleTrue, OnToggleFalse, null, null, null, null,
                                        ApplicationLauncher.AppScenes.FLIGHT, 
                                        GameDatabase.Instance.GetTexture("KramaxAutoPilot/Icon/AppLauncherIcon", false));
        }

        private static void OnToggleTrue()
        {
            KramaxAutoPilot.bDisplayAutoPilot = true;
        }

        private static void OnToggleFalse()
        {
            KramaxAutoPilot.bDisplayAutoPilot = false;
        }

        public static void setBtnState(bool state, bool click = false)
        {
            if (state)
                btnLauncher.SetTrue(click);
            else
                btnLauncher.SetFalse(click);
        }

        public static void OnDestroy()
        {
            if (btnLauncher != null)
            {
                ApplicationLauncher.Instance.RemoveModApplication(btnLauncher);
                btnLauncher = null;
            }
        }
    }
}
