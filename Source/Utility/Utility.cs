/*
 * SimplePartlessPlugin.cs
 * 
 * Part of the KSP modding examples from Thunder Aerospace Corporation.
 * 
 * (C) Copyright 2013, Taranis Elsu
 * 
 * Kerbal Space Program is Copyright (C) 2013 Squad. See http://kerbalspaceprogram.com/. This
 * project is in no way associated with nor endorsed by Squad.
 * 
 * This code is licensed under the Apache License Version 2.0. See the LICENSE.txt and NOTICE.txt
 * files for more information.
 * 
 * Note that Thunder Aerospace Corporation is a ficticious entity created for entertainment
 * purposes. It is in no way meant to represent a real entity. Any similarity to a real entity
 * is purely coincidental.
 */
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Kramax.Utility
{
    public static class Deb
    {
        public static void Log(String format, params System.Object[] args)
        {
            Debug.Log("[KRAMAX] " + String.Format(format, args));
        }

        public static void Log(String message)
        {
            Debug.Log("[KRAMAX] " + message);
        }

        public static void Verb(String format, params System.Object[] args)
        {
            Debug.Log("[KRAMAX] " + String.Format(format, args));
        }

        public static void Verb(String message)
        {
            Debug.Log("[KRAMAX] " + message);
        }

        public static void Err(String format, params System.Object[] args)
        {
            Debug.LogError("[KRAMAX] " + String.Format(format, args));
        }
    }

    public static class Utils
    {
        public static float Clamp(this float val, float min, float max)
        {
            if (val < min)
                return min;
            else if (val > max)
                return max;
            else
                return val;
        }

        public static double Clamp(this double val, double min, double max)
        {
            if (val < min)
                return min;
            else if (val > max)
                return max;
            else
                return val;
        }

        /// <summary>
        /// Circular rounding to keep compass measurements within a 360 degree range
        /// maxHeading is the top limit, bottom limit is maxHeading - 360
        /// </summary>
        public static double headingClamp(this double valToClamp, double maxHeading)
        {
            while (valToClamp > maxHeading)
                valToClamp -= 360;
            while (valToClamp < (maxHeading - 360))
                valToClamp += 360;
            return valToClamp;
        }

        /// <summary>
        /// Plane normal vector from a given heading (surface right vector)
        /// </summary>
        public static Vector3 vecHeading(double target, VesselData vdata)
        {
            double angleDiff = target - vdata.heading;
            return Quaternion.AngleAxis((float)(angleDiff + 90), (Vector3)vdata.planetUp) * vdata.surfVesForward;
        }

        /// <summary>
        /// calculate current heading from plane normal vector
        /// </summary>
        public static double calculateTargetHeading(Vector3 direction, VesselData vdata)
        {
            Vector3 fwd = Vector3.Cross(direction, vdata.planetUp);
            double heading = Vector3.Angle(fwd, vdata.planetNorth) * Math.Sign(Vector3.Dot(fwd, vdata.planetEast));
            return heading.headingClamp(360);
        }

        /// <summary>
        /// calculate current heading from plane rotation
        /// </summary>
        public static double calculateTargetHeading(Quaternion rotation, Vessel vessel, VesselData vdata)
        {
            Vector3 fwd = Vector3.Cross(getPlaneNormal(rotation, vessel), vdata.planetUp);
            double heading = Vector3.Angle(fwd, vdata.planetNorth) * Math.Sign(Vector3.Dot(fwd, vdata.planetEast));
            return heading.headingClamp(360);
        }

        /// <summary>
        /// calculates the angle to feed corrected for 0/360 crossings
        /// eg. if the target is 350 and the current is 10, it will return 370 giving a diff of -20 degrees
        /// else you get +ve 340 and the turn is in the wrong direction
        /// </summary>
        public static double CurrentAngleTargetRel(double current, double target, double maxAngle)
        {
            double diff = target - current;
            if (diff < maxAngle - 360)
                return current - 360;
            else if (diff > maxAngle)
                return current + 360;
            else
                return current;
        }

        /// <summary>
        /// calculate the planet relative rotation from the plane normal vector
        /// </summary>
        public static Quaternion getPlaneRotation(Vector3 planeNormal, Vessel vessel)
        {
            return Quaternion.FromToRotation(vessel.mainBody.transform.right, planeNormal);
        }

        public static Quaternion getPlaneRotation(double heading, Vessel vessel, VesselData vdata)
        {
            Vector3 planeNormal = vecHeading(heading, vdata);
            return getPlaneRotation(planeNormal, vessel);
        }

        public static Vector3 getPlaneNormal(Quaternion rotation, Vessel vessel)
        {
            return rotation * vessel.mainBody.transform.right;
        }

        public static bool IsNeutral(AxisBinding axis)
        {
            return axis.IsNeutral() && Math.Abs(axis.GetAxis()) < 0.00001;
        }

        public static bool hasYawInput()
        {
            return GameSettings.YAW_LEFT.GetKey() || GameSettings.YAW_RIGHT.GetKey() || !Utils.IsNeutral(GameSettings.AXIS_YAW);
        }

        public static bool hasPitchInput()
        {
            return GameSettings.PITCH_DOWN.GetKey() || GameSettings.PITCH_UP.GetKey() || !Utils.IsNeutral(GameSettings.AXIS_PITCH);
        }

        public static bool hasRollInput()
        {
            return GameSettings.ROLL_LEFT.GetKey() || GameSettings.ROLL_RIGHT.GetKey() || !Utils.IsNeutral(GameSettings.AXIS_ROLL);
        }

        public static bool hasThrottleInput()
        {
            return GameSettings.THROTTLE_UP.GetKey() || GameSettings.THROTTLE_DOWN.GetKey() || (GameSettings.THROTTLE_CUTOFF.GetKeyDown() && !GameSettings.MODIFIER_KEY.GetKey()) || GameSettings.THROTTLE_FULL.GetKeyDown();
        }


        public static Vector3d projectOnPlane(this Vector3d vector, Vector3d planeNormal)
        {
            return vector - Vector3d.Project(vector, planeNormal);
        }

        public static double speedUnitTransform(SpeedUnits units, double soundSpeed)
        {
            switch (units)
            {
                case SpeedUnits.mSec:
                    return 1;
                case SpeedUnits.knots:
                    return 1.943844492440604768413343347219;
                case SpeedUnits.kmph:
                    return 3.6;
                case SpeedUnits.mph:
                    return 2.236936;
                case SpeedUnits.mach:
                    return 1 / soundSpeed;
            }
            return 1;
        }

        public static double calcStagnationPres(Vessel vessel)
        {
            double stagnationPres =
                Math.Pow(((vessel.mainBody.atmosphereAdiabaticIndex - 1) *
                   vessel.mach * vessel.mach * 0.5) + 1,
                   vessel.mainBody.atmosphereAdiabaticIndex /
                   (vessel.mainBody.atmosphereAdiabaticIndex - 1));

            return stagnationPres;
        }

        public static double mSecToSpeedUnit(this double mSec, SpeedMode mode, SpeedUnits units, Vessel vessel)
        {
            if (mode == SpeedMode.Mach)
                return mSec / vessel.speedOfSound;
            else
            {
                double speed = mSec * speedUnitTransform(units, vessel.speedOfSound);
                switch (mode)
                {
                    case SpeedMode.True:
                        return speed;
                    case SpeedMode.Indicated:
                        return speed * Math.Sqrt(vessel.atmDensity / 1.225) * calcStagnationPres(vessel);
                    case SpeedMode.Equivalent:
                        return speed * Math.Sqrt(vessel.atmDensity / 1.225);
                }
                return 0;
            }
        }

        public static double SpeedUnitToMSec(this double speedUnit, SpeedMode mode, SpeedUnits units, Vessel vessel)
        {
            if (mode == SpeedMode.Mach)
                return speedUnit * vessel.speedOfSound;
            else
            {
                double speed = speedUnit / speedUnitTransform(units, vessel.speedOfSound);
                switch (mode)
                {
                    case SpeedMode.True:
                        return speed;
                    case SpeedMode.Indicated:
                        return speed * Math.Sqrt(vessel.atmDensity / 1.225) * calcStagnationPres(vessel);
                    case SpeedMode.Equivalent:
                        return speed / Math.Sqrt(vessel.atmDensity / 1.225);
                }
                return 0;
            }
        }

        public static string unitString(SpeedUnits unit)
        {
            switch (unit)
            {
                case SpeedUnits.mSec:
                    return "m/s";
                case SpeedUnits.mach:
                    return "mach";
                case SpeedUnits.knots:
                    return "knots";
                case SpeedUnits.kmph:
                    return "km/h";
                case SpeedUnits.mph:
                    return "mph";
            }
            return "";
        }


        public static string TryGetValue(this ConfigNode node, string key, string defaultValue)
        {
            if (node.HasValue(key))
                return node.GetValue(key);
            return defaultValue;
        }

        public static bool TryGetValue(this ConfigNode node, string key, bool defaultValue)
        {
            bool val;
            if (node.HasValue(key) && bool.TryParse(node.GetValue(key), out val))
                return val;
            return defaultValue;
        }

        public static int TryGetValue(this ConfigNode node, string key, int defaultValue)
        {
            int val;
            if (node.HasValue(key) && int.TryParse(node.GetValue(key), out val))
                return val;
            return defaultValue;
        }

        public static float TryGetValue(this ConfigNode node, string key, float defaultValue)
        {
            float val;
            if (node.HasValue(key) && float.TryParse(node.GetValue(key), out val))
                return val;
            return defaultValue;
        }

        public static double TryGetValue(this ConfigNode node, string key, double defaultValue)
        {
            double val;
            if (node.HasValue(key) && double.TryParse(node.GetValue(key), out val))
                return val;
            return defaultValue;
        }

        public static KeyCode TryGetValue(this ConfigNode node, string key, KeyCode defaultValue)
        {
            if (node.HasValue(key))
            {
                try
                {
                    KeyCode val = (KeyCode)System.Enum.Parse(typeof(KeyCode), node.GetValue(key));
                    return val;
                }
                catch { }
            }
            return defaultValue;
        }

        public static Rect TryGetValue(this ConfigNode node, string key, Rect defaultValue)
        {
            if (node.HasValue(key))
            {
                string[] stringVals = node.GetValue(key).Split(',').Select(s => s.Trim(new char[] { ' ', '(', ')' })).ToArray();
                if (stringVals.Length != 4)
                    return defaultValue;
                float x = 0, y = 0, w = 0, h = 0;
                if (!float.TryParse(stringVals[0].Substring(2), out x) || !float.TryParse(stringVals[1].Substring(2), out y) || !float.TryParse(stringVals[2].Substring(6), out w) || !float.TryParse(stringVals[3].Substring(7), out h))
                {
                    Debug.LogError(x + "," + y + "," + w + "," + h);
                    return defaultValue;
                }
                return new Rect(x, y, w, h);
            }
            return defaultValue;
        }
    }
}
