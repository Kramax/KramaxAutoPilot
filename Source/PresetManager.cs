using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using KramaxReloadExtensions;

namespace Kramax
{
    using PID;
    using Utility;
    using Presets;

    [KSPAddon(KSPAddon.Startup.Flight, true)]
	public class PresetManager : ReloadableMonoBehaviour
//	public class PresetManager : MonoBehaviour
    {
        private static PresetManager instance;
        public static PresetManager Instance
        {
            get
            {
                return instance;
            }
        }

        public List<APPreset> APPresetList = new List<APPreset>();

        public APPreset activeAPPreset = null;

        public Dictionary<string, CraftPreset> craftPresetDict = new Dictionary<string, CraftPreset>();

        const string presetsPath = "GameData/KramaxAutoPilot/Presets.cfg";
        const string defaultsPath = "GameData/KramaxAutoPilot/Defaults.cfg";

        const string craftDefaultName = "default";
        const string apDefaultName = "default";

        const string craftPresetNodeName = "CraftPreset";
        const string apPresetNodeName = "PIDPreset";

        const string craftAPKey = "pilot";

        const string hdgCtrlr = "HdgBankController";
        const string yawCtrlr = "HdgYawController";
        const string aileronCtrlr = "AileronController";
        const string rudderCtrlr = "RudderController";
        const string altCtrlr = "AltitudeController";
        const string vertCtrlr = "AoAController";
        const string elevCtrlr = "ElevatorController";
        const string speedCtrlr = "SpeedController";
        const string accelCtrlr = "AccelController";
        const string cdiCtrlr = "CDIController";

        const string pGain = "PGain";
        const string iGain = "IGain";
        const string dGain = "DGain";
        const string min = "MinOut";
        const string max = "MaxOut";
        const string iLower = "ClampLower";
        const string iUpper = "ClampUpper";
        const string scalar = "Scalar";
        const string ease = "Ease";
        const string delay = "Delay";

        double[] defaultPresetPitchGains = { 0.15, 0.0, 0.06, 3, 20 }; // Kp/i/d, scalar, delay
        double[] defaultPresetRollGains = { 0.1, 0.0, 0.06, 3, 20 };
        double[] defaultPresetHdgGains = { 2.0, 0.0, 0.0, 0.1, 20 };

        public void Awake()
        {
            Deb.Log("PresetManager.Awake: creating singleton instance");
            instance = this;
        }

        public void Start()
        {
            Deb.Log("PresetManager.Start: initialize");
            loadPresetsFromFile();
            DontDestroyOnLoad(this);
        }

        public void OnDestroy()
        {
            saveToFile();
        }

        public void OnGUI()
        {
            // create the GUISkin
            if (GeneralUI.UISkin == null)
                GeneralUI.customSkin();
        }

        public static void loadPresetsFromFile()
        {
            APPreset asstDefault = null;
            
            foreach (ConfigNode node in GameDatabase.Instance.GetConfigNodes(apPresetNodeName))
            {
                if (node == null)
                    continue;

                List<double[]> gains = new List<double[]>();
                gains.Add(controllerGains(node.GetNode(hdgCtrlr), AsstList.HdgBank));
                gains.Add(controllerGains(node.GetNode(yawCtrlr), AsstList.BankToYaw));
                gains.Add(controllerGains(node.GetNode(aileronCtrlr), AsstList.Aileron));
                gains.Add(controllerGains(node.GetNode(rudderCtrlr), AsstList.Rudder));
                gains.Add(controllerGains(node.GetNode(altCtrlr), AsstList.Altitude));
                gains.Add(controllerGains(node.GetNode(vertCtrlr), AsstList.VertSpeed));
                gains.Add(controllerGains(node.GetNode(elevCtrlr), AsstList.Elevator));
                gains.Add(controllerGains(node.GetNode(speedCtrlr), AsstList.Speed));
                gains.Add(controllerGains(node.GetNode(accelCtrlr), AsstList.Acceleration));
                gains.Add(controllerGains(node.GetNode(cdiCtrlr), AsstList.CdiVelocity));

                string name = node.GetValue("name");
                if (name == apDefaultName)
                    asstDefault = new APPreset(gains, name);
                else if (!instance.APPresetList.Any(p => p.name == name))
                    instance.APPresetList.Add(new APPreset(gains, name));
            }

            /*
            foreach (ConfigNode node in GameDatabase.Instance.GetConfigNodes(sasPresetNodeName))
            {
                if (node == null || node.GetValue("stock") == "false")
                    continue;

                List<double[]> gains = new List<double[]>();
                gains.Add(controllerSASGains(node.GetNode(elevCtrlr), SASList.Pitch));
                gains.Add(controllerSASGains(node.GetNode(aileronCtrlr), SASList.Bank));
                gains.Add(controllerSASGains(node.GetNode(rudderCtrlr), SASList.Hdg));

                string name = node.GetValue("name");
                if (name == SASDefaultName)
                    SASDefault = new SASPreset(gains, name);
                else if (!instance.SASPresetList.Any(p => p.name == name))
                    instance.SASPresetList.Add(new SASPreset(gains, name));
            }
            */

            foreach (ConfigNode node in GameDatabase.Instance.GetConfigNodes(craftPresetNodeName))
            {
                if (node == null || instance.craftPresetDict.ContainsKey(node.GetValue("name")))
                    continue;

                string name = node.GetValue("name");
                if (name == craftDefaultName)
                    instance.craftPresetDict.Add(craftDefaultName, 
                        new CraftPreset(craftDefaultName, asstDefault));
                else
                {
                    CraftPreset cP = new CraftPreset(name,
                                            instance.APPresetList.FirstOrDefault(p => p.name == node.GetValue(craftAPKey)));

                    instance.craftPresetDict.Add(cP.Name, cP);
                }
            }
        }

        public static void saveToFile()
        {
            if (!instance)
            {
                Debug.Log("PresetManager.saveToFile: no instance, return.");
                return;
            }

            ConfigNode node = new ConfigNode();

            node.AddValue("dummy", "do not delete me");

            foreach (APPreset p in instance.APPresetList)
            {
                node.AddNode(APPresetNode(p));
            }

            /*
            foreach (SASPreset p in instance.SASPresetList)
            {
                node.AddNode(SASPresetNode(p));
            }
             */

            foreach (KeyValuePair<string, CraftPreset> cP in instance.craftPresetDict)
            {
                if (cP.Value == null || cP.Key == craftDefaultName || cP.Value.Dead)
                    continue;

                node.AddNode(CraftNode(cP.Value));
            }

            node.Save(KSPUtil.ApplicationRootPath.Replace("\\", "/") + presetsPath);
        }

        public static void saveDefaults()
        {
            ConfigNode node = new ConfigNode();
            CraftPreset cP = instance.craftPresetDict[craftDefaultName];

            if (cP.apPreset != null)
                node.AddNode(APPresetNode(cP.apPreset));

            node.AddNode(CraftNode(cP));
            node.Save(KSPUtil.ApplicationRootPath.Replace("\\", "/") + defaultsPath);
        }

        public static void updateDefaults()
        {
            instance.craftPresetDict[craftDefaultName].apPreset.PIDGains = instance.activeAPPreset.PIDGains;
            saveDefaults();
        }

        public static double[] controllerGains(ConfigNode node, AsstList type)
        {
            double[] gains = new double[9];

            if (node == null)
                return defaultControllerGains(type);

            double.TryParse(node.GetValue(pGain), out gains[0]);
            double.TryParse(node.GetValue(iGain), out gains[1]);
            double.TryParse(node.GetValue(dGain), out gains[2]);
            double.TryParse(node.GetValue(min), out gains[3]);
            double.TryParse(node.GetValue(max), out gains[4]);
            double.TryParse(node.GetValue(iLower), out gains[5]);
            double.TryParse(node.GetValue(iUpper), out gains[6]);
            double.TryParse(node.GetValue(scalar), out gains[7]);
            double.TryParse(node.GetValue(ease), out gains[8]);

            return gains;
        }

        public static double[] defaultControllerGains(AsstList type)
        {
            switch(type)
            {
                case AsstList.HdgBank:
                    return George.defaultHdgBankGains;
                case AsstList.BankToYaw:
                    return George.defaultBankToYawGains;
                case AsstList.Aileron:
                    return George.defaultAileronGains;
                case AsstList.Rudder:
                    return George.defaultRudderGains;
                case AsstList.Altitude:
                    return George.defaultAltitudeGains;
                case AsstList.VertSpeed:
                    return George.defaultVSpeedGains;
                case AsstList.Elevator:
                    return George.defaultElevatorGains;
                case AsstList.Speed:
                    return George.defaultSpeedGains;
                case AsstList.Acceleration:
                    return George.defaultAccelGains;
                case AsstList.CdiVelocity:
                    return George.defaultXtrkGains;
                default:
                    return George.defaultAileronGains;
            }
        }

        public static ConfigNode APPresetNode(APPreset preset)
        {
            ConfigNode node = new ConfigNode(apPresetNodeName);
            node.AddValue("name", preset.name);
            node.AddNode(PIDnode(hdgCtrlr, (int)AsstList.HdgBank, preset));
            node.AddNode(PIDnode(yawCtrlr, (int)AsstList.BankToYaw, preset));
            node.AddNode(PIDnode(aileronCtrlr, (int)AsstList.Aileron, preset));
            node.AddNode(PIDnode(rudderCtrlr, (int)AsstList.Rudder, preset));
            node.AddNode(PIDnode(altCtrlr, (int)AsstList.Altitude, preset));
            node.AddNode(PIDnode(vertCtrlr, (int)AsstList.VertSpeed, preset));
            node.AddNode(PIDnode(elevCtrlr, (int)AsstList.Elevator, preset));
            node.AddNode(PIDnode(speedCtrlr, (int)AsstList.Speed, preset));
            node.AddNode(PIDnode(accelCtrlr, (int)AsstList.Acceleration, preset));
            node.AddNode(PIDnode(cdiCtrlr, (int)AsstList.CdiVelocity, preset));

            return node;
        }

        public static ConfigNode PIDnode(string name, int index, APPreset preset)
        {
            ConfigNode node = new ConfigNode(name);
            node.AddValue(pGain, preset.PIDGains[index][0]);
            node.AddValue(iGain, preset.PIDGains[index][1]);
            node.AddValue(dGain, preset.PIDGains[index][2]);
            node.AddValue(min, preset.PIDGains[index][3]);
            node.AddValue(max, preset.PIDGains[index][4]);
            node.AddValue(iLower, preset.PIDGains[index][5]);
            node.AddValue(iUpper, preset.PIDGains[index][6]);
            node.AddValue(scalar, preset.PIDGains[index][7]);
            node.AddValue(ease, preset.PIDGains[index][8]);
            return node;
        }

        public static ConfigNode CraftNode(CraftPreset preset)
        {
            ConfigNode node = new ConfigNode(craftPresetNodeName);
            if (!string.IsNullOrEmpty(preset.Name))
            {
                node.AddValue("name", preset.Name);
                if (preset.apPreset != null && !string.IsNullOrEmpty(preset.apPreset.name))
                    node.AddValue(craftAPKey, preset.apPreset.name);
            }

            return node;
        }

        #region APPreset
        public static void newAPPreset(ref string name, APController[] controllers, Vessel v)
        {
            if (name == "")
                return;

            string tempName = name;
            if (Instance.APPresetList.Any(p => p.name == tempName))
            {
                GeneralUI.postMessage("Failed to add preset with duplicate name");
                return;
            }
            APPreset newPreset = new APPreset(controllers, name);
            updateCraftPreset(newPreset, v);
            Instance.APPresetList.Add(newPreset);
            Instance.activeAPPreset = PresetManager.Instance.APPresetList.Last();
            saveToFile();
            name = "";
        }

        public static void loadAPPreset(APPreset p, George instance)
        {
            APController[] c = instance.controllers;
            for (int i = 0; i < c.Length && i < p.PIDGains.Count; i++)
            {
                c[i].PGain = p.PIDGains[i][0];
                c[i].IGain = p.PIDGains[i][1];
                c[i].DGain = p.PIDGains[i][2];
                c[i].OutMin = p.PIDGains[i][3];
                c[i].OutMax = p.PIDGains[i][4];
                c[i].ClampLower = p.PIDGains[i][5];
                c[i].ClampUpper = p.PIDGains[i][6];
                c[i].Scalar = p.PIDGains[i][7];
                c[i].Easing = p.PIDGains[i][8];
            }
            
            Instance.activeAPPreset = p;
            GeneralUI.postMessage("Loaded preset " + p.name);
            
            if (Instance.activeAPPreset != Instance.craftPresetDict[craftDefaultName].apPreset)
                updateCraftPreset(Instance.activeAPPreset, instance.vessel);

            saveToFile();
        }

        public static void updateAPPreset(George instance)
        {
            Instance.activeAPPreset.Update(instance.controllers);
            saveToFile();
        }

        public static void deleteAPPreset(APPreset p)
        {
            GeneralUI.postMessage("Deleted preset " + p.name);
            if (Instance.activeAPPreset == p)
                Instance.activeAPPreset = null;
            Instance.APPresetList.Remove(p);

            p = null;

            saveToFile();
        }
        #endregion

        #region Craft Presets

        // called on vessel load
        public static void loadCraftAPPreset(George instance)
        {
            if (Instance.craftPresetDict.ContainsKey(FlightGlobals.ActiveVessel.vesselName) &&
                Instance.craftPresetDict[FlightGlobals.ActiveVessel.vesselName].apPreset != null)
            {
                loadAPPreset(Instance.craftPresetDict[FlightGlobals.ActiveVessel.vesselName].apPreset, instance);
            }
            else
            {
                loadAPPreset(Instance.craftPresetDict[craftDefaultName].apPreset, instance);
            }
        }

        public static void initDefaultPresets(APPreset p)
        {
            initDefaultPresets();
            if (Instance.craftPresetDict[craftDefaultName].apPreset == null)
                Instance.craftPresetDict[craftDefaultName].apPreset = p;
            PresetManager.saveDefaults();
        }

        public static void initDefaultPresets()
        {
            if (!Instance.craftPresetDict.ContainsKey("default"))
                Instance.craftPresetDict.Add("default", new CraftPreset("default", null));
        }

        public static void updateCraftPreset(APPreset p, Vessel v)
        {
            initCraftPreset(v);
            Instance.craftPresetDict[v.vesselName].apPreset = p;
        }

        public static void initCraftPreset(Vessel v)
        {
            if (!Instance.craftPresetDict.ContainsKey(v.vesselName))
            {
                Instance.craftPresetDict.Add(v.vesselName,
                   new CraftPreset(v.vesselName,
                                   Instance.activeAPPreset == Instance.craftPresetDict[craftDefaultName].apPreset ? null : Instance.activeAPPreset));
            }
        }

        #endregion
    }
}
