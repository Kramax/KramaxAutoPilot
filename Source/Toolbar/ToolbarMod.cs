using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Kramax.Toolbar
{
    public static class ToolbarMod
    {
        /*
        private static IButton apButton;
        private static IButton menuButton;
        */

        public static void Awake()
        {
        }

        public static void Start()
        {
            /*
            menuButton = ToolbarManager.Instance.add("PilotAssistant", "PilotAssistantMenuBtn");
            menuButton.TexturePath = PilotAssistantFlightCore.Instance.blizMenuTexPath;
            menuButton.ToolTip = "Open Pilot Assistant Menu";
            menuButton.OnClick += (e) => PilotAssistantFlightCore.bDisplayOptions = !PilotAssistantFlightCore.bDisplayOptions;

            apButton = ToolbarManager.Instance.add("PilotAssistant", "PilotAssistantAsstBtn");
            apButton.TexturePath = PilotAssistantFlightCore.Instance.blizAsstTexPath;
            apButton.ToolTip = "Open Pilot Assistant Window";
            apButton.OnClick += (e) => PilotAssistantFlightCore.bDisplayAssistant = !PilotAssistantFlightCore.bDisplayAssistant;
             */
        }

        public static void OnDestroy()
        {
            /*
            if (menuButton != null) menuButton.Destroy();
            if (apButton != null) asstButton.Destroy();
             */
        }
    }
}
