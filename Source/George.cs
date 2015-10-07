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
using KramaxReloadExtensions;

namespace Kramax
{
    using PID;
    // DRN fixme
    // using Presets;
    using Utility;
    using System.Collections;
    using Presets;

    public enum AsstList
    {
        HdgBank,
        BankToYaw,
        Aileron,
        Rudder,
        Altitude,
        VertSpeed,
        Elevator,
        Speed,
        Acceleration,
        CdiVelocity,
    }

    public enum VertMode
    {
        ToggleOn = -1,
        Pitch = 0,
        VSpeed = 1,
        Altitude = 2,
        RadarAltitude = 3,
        Glideslope = 4,
        Last
    }

    public enum HrztMode
    {
        ToggleOn = -1,
        Bank = 0,
        Heading = 1,
        Course = 2,
        Last
    }

    public enum ThrottleMode
    {
        ToggleOn = -1,
        Speed = 0,
        Acceleration = 1,
        Autoland = 2,
        Last
    }

    public enum LandingMode
    {
        None = 0,
        ToIAF = 1,
        ToFAF = 2,
        Final = 3,
        Touchdown = 4,
        Stop = 5,
    }

    public enum SpeedUnits
    {
        mSec,
        kmph,
        mph,
        knots,
        mach
    }

    public enum SpeedMode
    {
        True,
        Indicated,
        Equivalent,
        Mach
    }

    // handle raising and lowering the landing gear
    // you can find out if the gear is currently down or not with this:
    //  bool gear_is_up =  vessel.ActionGroups[KSPActionGroup.Gear]
    // We can quickly get the vessel radar alt with this call:
    //  vessel.heightFromSurface
    // but annoyingly it returns a height below see level until off th coast some at which
    // point it returns -1. 
    // Can get "terrain altitude" which is height of actual terrain (negative over ocean) and just calculate it:
    //  vessel.altitude - Max(vessel.terrainAltitude, 0);
    // So logic to raise/lower gear:
    // if below 75m set raise trigger
    // if above 150m raise gear if trigger is on, unset raise trigger
    // if above 600m set lower trigger
    // if below 500m lower gear if trigger is on, unset lower trigger
    public class GearHandler
    {
        public bool autoGearEnabled = true; // when false will not ever act
        public bool raiseTrigger = false; // when true we are enabled to raise the gear
        public bool downTrigger = false; // when true we can put the gear down
        public double ralt = Double.NaN;
        public double lowTrigger = 75; // must get below this on descent to cause a climb to raise gear
        public double highTrigger = 600; // must get above this to cause a descent to lower gear
        public double raiseHeight = 150; // above this and we will raise the gear
        public double lowerHeight = 500; // below this and we will lower the gear

        // this should get called once every physics frame no matter what
        public void AlwaysUpdate(Vessel vessel)
        {
            if (vessel == null)
            {
                // invalidate stuff?
                raiseTrigger = false;
                downTrigger = false;
                ralt = Double.NaN;
                return;
            }

            ralt = vessel.altitude - Math.Max(0, vessel.terrainAltitude);

            // if we are in the band 500-600 we should enable lowering the gear
            if (ralt > highTrigger)
            {
                // if we get above a certain height, we retrigger so we can lower gear if get too low
                downTrigger = true;
            }
            else if (ralt < lowTrigger)
            {
                // if we are below a certain height, we retrigger so we can raise gear when we climb
                raiseTrigger = true;
            }
        }

        // only call when you want it to handle the gear
        public void Update(Vessel vessel)
        {
            if (vessel == null || !autoGearEnabled)
            {
                return;
            }

            // bool gear_is_up = vessel.ActionGroups[KSPActionGroup.Gear];

            if (raiseTrigger && ralt > raiseHeight)
            {
                Deb.Log("GearHandler: raising gear at height {0}", ralt);

                vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, false);
                raiseTrigger = false;
            }
            if (downTrigger && ralt < lowerHeight)
            {
                Deb.Log("GearHandler: lowering gear at height {0}", ralt);

                vessel.ActionGroups.SetGroup(KSPActionGroup.Gear, true);
                downTrigger = false;
            }        
        }
    }

    public enum WPFlag
    {
        Flown = 1, // already flown passed it
        Active = 2, // flying TO it
        Vertical = 4, // altitude valid
        IAF = 8, // it is an initial approach fix
        FAF = 16, // it is the final approach fix
        RW = 32, // it is the start of the runway
        Stop = 64, // it is the end of the runway, so land already
        Current = 128, // it is just the current location of the ship
    }

    public class WayPoint
    {
        public String name = "";
        public int flags = 0;
        public double lat = 0;
        public double lon = 0;
        public double alt = -1.0; // ref alt, not above ground    
        public double distance = 0.0; // only if next is not null
        public double bearing = 0.0; // only if next is not null
        public WayPoint next = null;

        // does not clone next!
        public WayPoint Clone()
        {
            var result = new WayPoint();

            result.name = String.Copy(name);
            result.flags = flags;
            result.lat = lat;
            result.lon = lon;
            result.alt = alt;

            return result;
        }

        public void ClearFlag(WPFlag f)
        {
            flags = flags & (~(int)f);
        }

        public void SetFlag(WPFlag f)
        {
            flags = flags | (int)f;
        }

        public bool HasFlag(WPFlag f)
        {
            return (flags & (int)f) != 0;
        }

        public override string ToString()
        {
            String sflags = "";

            foreach (var enval in Enum.GetValues(typeof(WPFlag)).Cast<WPFlag>())
            {
                if (HasFlag(enval))
                {
                    sflags += enval.ToString();
                    sflags += " ";
                }
            }

            return String.Format("WayPoint {4} {0:F3} {1:F3} {2:F0}m {3}",
                lat, lon, alt, sflags, name);
        }

        public ConfigNode ToConfigNode()
        {
            ConfigNode result = new ConfigNode("WayPoint");

            foreach (var enval in Enum.GetValues(typeof(WPFlag)).Cast<WPFlag>())
            {
                // we do not want to store some info
                if (enval == WPFlag.Flown) continue;
                if (enval == WPFlag.Active) continue;
                if (enval == WPFlag.Current) continue;

                if (HasFlag(enval))
                    result.AddValue(enval.ToString(), "true");
            }
            result.AddValue("lat", lat);
            result.AddValue("lon", lon);
            result.AddValue("alt", alt);
            result.AddValue("name", name);

            return result;
        }

        public void SetFromConfigNode(ConfigNode node)
        {
            flags = 0;

            foreach (var enval in Enum.GetValues(typeof(WPFlag)).Cast<WPFlag>())
            {
                if (node.GetValue(enval.ToString()) == "true")
                    SetFlag(enval);
            }

            name = node.GetValue("name");

            if (name == null)
                name = "";

            double.TryParse(node.GetValue("lat"), out lat);
            double.TryParse(node.GetValue("lon"), out lon);
            double.TryParse(node.GetValue("alt"), out alt);
        }
    }

    public class CourseStatus
    {
        public double courseBearing = Double.NaN; // initial bearing at start point
        public double courseDistance = Double.NaN; // circle distance from prev to next
        public double currentBearing = Double.NaN; // bearing if on course to get to next
        public double currentDistance = Double.NaN; // distance along route remaining to next
        public double directBearing = Double.NaN; // bearing direct to next from current pos
        public double directDistance = Double.NaN; // distance direct to next from current pos
        public double distanceTraveled = Double.NaN; // distance we have traveled along the course already
        public double dXt = Double.NaN; // distance between course and current pos
        public double dVt = Double.NaN; // distance between glideslope and us vertically, negative is too low
        public double currentAltitude = Double.NaN; // altitude at current location if we were on glideslope
        public double courseSlope = Double.NaN; // slope in V/H to follow glideslope
        public double fraction = Double.NaN; // parameter 0 to 1 of how far along course we are
        public double vXt = Double.NaN; // cross-track velocity
        public double vC = Double.NaN; // along-track velocity (positive is towards next)
        public double timestamp = Double.NaN; // when was this updated
        public bool beenValid = false; // has this course every had a fraction between 0 and 1?
    }


    public class CourseUtils
    {
        static public double ToRadians(double angle)
        {
            return (Math.PI / 180.0) * angle;
        }

        static public double ToDegrees(double angle)
        {
            var result = (180.0 / Math.PI) * angle;

            if (result < 0)
                result += 360.0;

            return result;
        }

        // All values are in radians
        static public void BearingAndDistanceRad(CelestialBody planet,
                                              double lat1, double lon1,
                                              double lat2, double lon2,
                                              out double bearing,
                                              out double distance)
        {
            // what if point 1 and 2 are coincident? fixme
            var R = planet.Radius; // metres
            var phi1 = lat1;
            var phi2 = lat2;
            var dPhi = lat2 - lat1;
            var dLambda = lon2 - lon1;

            var a = Math.Sin(dPhi / 2) * Math.Sin(dPhi / 2) +
                    Math.Cos(phi1) * Math.Cos(phi2) *
                    Math.Sin(dLambda / 2) * Math.Sin(dLambda / 2);
            var c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));

            distance = R * c;

            var y = Math.Sin(dLambda) * Math.Cos(phi2);
            var x = Math.Cos(phi1) * Math.Sin(phi2) -
                    Math.Sin(phi1) * Math.Cos(phi2) * Math.Cos(dLambda);
            bearing = Math.Atan2(y, x);
        }

        static public void BearingAndDistance(CelestialBody planet,
                                              double lat1, double lon1,
                                              double lat2, double lon2,
                                              out double bearing,
                                              out double distance)
        {
            double rbearing;

            BearingAndDistanceRad(planet, ToRadians(lat1), ToRadians(lon1),
                ToRadians(lat2), ToRadians(lon2), out rbearing, out distance);

            bearing = ToDegrees(rbearing);
        }

        // course is from lat1,lon1 to lat2,lon2
        // check where lat3,lon3 is in respect to that course
        static public void CrossTrackError(CelestialBody planet,
                                           double lat1, double lon1, // course start point
                                           double lat2, double lon2, // course end point
                                           double lat3, double lon3, // point we are at
                                           CourseStatus cs)
        {
            // all from:
            // http://www.movable-type.co.uk/scripts/latlong.html
            double R = planet.Radius;
            double rlat1 = ToRadians(lat1);
            double rlon1 = ToRadians(lon1);
            double rlat2 = ToRadians(lat2);
            double rlon2 = ToRadians(lon2);
            double rlat3 = ToRadians(lat3);
            double rlon3 = ToRadians(lon3);

            // first get the distance from start point to current point
            double b13, d13;

            BearingAndDistanceRad(planet, rlat1, rlon1, rlat3, rlon3, out b13, out d13);

            var dA13 = d13 / R;

            double b12;
            double d12;

            // distance of course on ideal course line
            BearingAndDistanceRad(planet, rlat1, rlon1, rlat2, rlon2, out b12, out d12);

            // finally distance from point 3 to the end point (point 2)
            double b32, d32;

            BearingAndDistanceRad(planet, rlat3, rlon3, rlat2, rlon2, out b32, out d32);


            cs.courseBearing = ToDegrees(b12);
            cs.courseDistance = d12;

            // cross track error, sign tells which side of course you are on
            // this is valid even when outside start/end points
            cs.dXt = Math.Asin(Math.Sin(d13 / R) * Math.Sin(b13 - b12)) * R;

            if (d13 > d12)
            {
              // if the distance d13 is GREATER than d12 it means we are beyond the end point
              cs.distanceTraveled = d12;
              cs.currentDistance = 0;
            }
            else
            {
              // distance along the course that nearest point coresponds to
              cs.distanceTraveled = Math.Acos(Math.Cos(dA13) / Math.Cos(cs.dXt / R)) * R;
            
              if (d32 > d12)
              {
                // if the distance d32 is GREATER than d12 it means we are actually further
                // away than the start point (beyond end of start point)
                cs.distanceTraveled = -cs.distanceTraveled;
              }
              else
              {
                cs.currentDistance = d12 - cs.distanceTraveled;
              }

              // distance remaining if you were traveling along the course line,
              // may be more than direct distance if went beyond start point in
              // wrong direction.
              cs.currentDistance = d12 - cs.distanceTraveled;
            }

            cs.directBearing = ToDegrees(b32);
            cs.directDistance = d32;

            // fraction along course traveled 0 to 1; do not clamp cause we want to be able to detect
            // valid range.
            cs.fraction = cs.distanceTraveled / cs.courseDistance;

            var a = Math.Cos(rlat1) * Math.Cos(rlat2);
            var dA12 = cs.courseDistance / R;
            var b = Math.Sin(cs.distanceTraveled / R) / Math.Sin(dA12);
            var x = a * Math.Cos(rlat1) * Math.Cos(rlon1) + b * Math.Cos(rlat2) * Math.Cos(rlon2);
            var y = a * Math.Cos(rlat1) * Math.Sin(rlon1) + b * Math.Cos(rlat2) * Math.Sin(rlon2);
            var z = a * Math.Sin(rlat1) + b * Math.Sin(rlat2);
            var rlat_i = Math.Atan2(z, Math.Sqrt(x * x + y * y));
            var rlon_i = Math.Atan2(y, x);

            double ri2, di2; // di2 should match curentDistance
            BearingAndDistanceRad(planet, rlat_i, rlon_i, rlat2, rlon2, out ri2, out di2);

            //  Deb.Log("d12:{0:F2} d13:{1:F2} d32:{2:F2}, cd:{3:F2} di2:{4:F2} fr:{5:F3}", 
            //        d12, d13, d32, cs.currentDistance, di2, cs.fraction);
            
            cs.currentBearing = ToDegrees(ri2);
        }

        // assumes that cs.fraction is already set
        static public void UpdateCourseVertical(CelestialBody planet,
            double alt1, double alt2, // course start and end altitudes
            double alt3, // current altitude
            CourseStatus cs)
        {
            if (Double.IsNaN(alt1) || Double.IsNaN(alt2) || Double.IsNaN(cs.fraction))
            {
                return;
            }

            var frac = Utils.Clamp(cs.fraction, 0, 1);

            cs.currentAltitude = alt2 * frac + alt1 * (1.0 - frac);
            cs.dVt = alt3 - cs.currentAltitude;

            if (cs.courseDistance > 0)
                cs.courseSlope = (alt2 - alt1) / cs.courseDistance;
            else
                cs.courseSlope = Double.NaN;
        }


    }

    public class FlightPlan : CourseUtils
    {
        public String name = "flightplan";
        public String description = "Generic Flight Plan";
        public List<WayPoint> course = new List<WayPoint>();
        public WayPoint prev = null;
        public WayPoint next = null;
        public WayPoint position = new WayPoint();
        public CourseStatus courseStatus = new CourseStatus();
        public CelestialBody planet = null;

        public FlightPlan Clone()
        {
            var result = new FlightPlan();

            result.name = String.Copy(name);
            result.description = String.Copy(description);
            result.planet = planet;

            WayPoint prev_wp = null;

            foreach (var wp in course)
            {
                var new_wp = wp.Clone();

                if (prev_wp != null)
                    prev_wp.next = new_wp;

                result.course.Add(new_wp);

                prev_wp = new_wp;
            }

            Deb.Log("FlightPlan.Clone: update waypoint values for resulting course.");
            result.UpdateWayPointValues(planet);

            return result;
        }

        private CelestialBody GetCelestialBodyForName(String name)
        {
            if (name == null)
                return null;

            foreach (var body in FlightGlobals.Bodies)
            {
                if (name == body.name)
                    return body;
            }
            return null;
        }

        public ConfigNode ToConfigNode()
        {
            ConfigNode result = new ConfigNode("FlightPlan");

            ConfigNode courseNode = result.AddNode("WayPoints");

            foreach (var wp in course)
            {
                if (wp.HasFlag(WPFlag.Current))
                    continue; // do not store current waypoints

                courseNode.AddNode(wp.ToConfigNode());
            }

            result.AddValue("planet", planet.name);
            result.AddValue("name", name);
            result.AddValue("description", description);

            return result;
        }

        public void SetFromConfigNode(ConfigNode node)
        {
            planet = GetCelestialBodyForName(node.GetValue("planet"));
            name = node.GetValue("name");
            description = node.GetValue("description");

            course.Clear();

            var courseNode = node.GetNode("WayPoints");

            if (courseNode != null)
            {
                foreach (var wp_node in courseNode.GetNodes("WayPoint"))
                {
                    var wp = new WayPoint();
                    wp.SetFromConfigNode(wp_node);
                    course.Add(wp);
                }
            }
        }

        // does not try to sequence
        private void UpdateData(George george, Vessel vessel, VesselData vdata)
        {
            planet = vessel.mainBody;

            // var mypos = vessel.findWorldCenterOfMass();
            double mylat = vessel.latitude;
            double mylon = vessel.longitude;
            double myalt = vessel.altitude;

            position.lat = mylat;
            position.lon = mylon;
            position.alt = myalt;
            position.SetFlag(WPFlag.Current);

            if (prev == null || next == null)
                return;

            double prev_dXt = courseStatus.dXt;
            double prev_distance = courseStatus.currentDistance;

            CrossTrackError(planet,
                            prev.lat, prev.lon,
                            next.lat, next.lon,
                            mylat, mylon,
                            courseStatus);

            // track if this course has ever been valid to avoid sequencing over things entirely
            // without user being aware. Not fraction is clamped 0 to 1 so do not use that.
            if (!courseStatus.beenValid && (0 < courseStatus.fraction && courseStatus.fraction < 1))
            {
                Deb.Log("UpdateData: marking course to {0} with dist {1} as valid", next, courseStatus.distanceTraveled);
                courseStatus.beenValid = true;
            }

            double timestamp = Planetarium.GetUniversalTime();
            double delta_time = timestamp - courseStatus.timestamp;
            courseStatus.timestamp = timestamp;

            // who knows if the sign will be right here; fixme
            // heading is not stable if craft is wallowing
            // courseStatus.vXt = vessel.srfSpeed * Math.Sin(ToRadians(vdata.heading-courseStatus.currentBearing));
            double delta_dXt = courseStatus.dXt - prev_dXt;
            double delta_distance = prev_distance - courseStatus.currentDistance;

            if (delta_time > 0 && !Double.IsNaN(delta_dXt) && !Double.IsNaN(delta_distance))
            {
                courseStatus.vXt = delta_dXt / delta_time;
                courseStatus.vC = delta_distance / delta_time;
            }
            else
            {
                courseStatus.vXt = Double.NaN;
                courseStatus.vC = Double.NaN;
            }

            // Deb.Log("FP: delta_t={2}, dXt={0}, vXt={1}", courseStatus.dXt, courseStatus.vXt, delta_time);
            // Deb.Log("Along track velocity: {0}", courseStatus.vC);

            if (next.HasFlag(WPFlag.Vertical))
            {
                if (prev.HasFlag(WPFlag.Vertical))
                {
                    UpdateCourseVertical(planet, prev.alt, next.alt, myalt, courseStatus);
                }
                else
                {
                    courseStatus.dVt = myalt - next.alt;
                    courseStatus.currentAltitude = next.alt;
                }
            }
            else
            {
                courseStatus.dVt = Double.NaN;
                courseStatus.currentAltitude = Double.NaN;
            }
        }

        private void CheckForSequence(George george, Vessel vessel, VesselData vdata)
        {
            if (next == null)
                return;

            planet = vessel.mainBody;

            var nextNext = next.next;

            // do not skip over courses that we have never been on
            // otherwise it can auto-sequence through a whole series of things that
            // the player meant to fly.
            if (!courseStatus.beenValid)
            {
                return;
            }
            
            // have to figure out if we have passed next yet
            // looking at distance not good enough, should look for
            // either reversal of heading neeeded or closer to next course

            // -10 to deal with any round-off error
            if (courseStatus.distanceTraveled >= (courseStatus.courseDistance-10))
            {
                if (nextNext != null)
                {
                    Deb.Log("FlightPlan.Update: traveled past current leg, sequence");
                    SequenceWaypoint(george, vessel, vdata);
                    return;
                }
                else
                {
                    next = null;
                }
            }
            else if (nextNext != null &&
                     !next.HasFlag(WPFlag.FAF) &&
                     !next.HasFlag(WPFlag.RW) &&
                     !next.HasFlag(WPFlag.Stop))
            {
                CourseStatus nextStatus = new CourseStatus();

                CrossTrackError(planet,
                                next.lat, next.lon,
                                nextNext.lat, nextNext.lon,
                                position.lat, position.lon,
                                nextStatus);

                var remaining = courseStatus.currentDistance;
                var speed = vessel.srfSpeed;

                if (speed > 0)
                {
                    var timeToNext = remaining / speed;
                    var turnAmount = Math.Abs(nextStatus.courseBearing - courseStatus.courseBearing);

                    // deal with 350-10
                    if (turnAmount > 180)
                    {
                        turnAmount = 360 - turnAmount;
                    }

                    // estimate turn time
                    var turnRate = 3; // 3 degrees per second gives standard rate turn, should get from vessel
                    var timeToTurn = turnAmount / turnRate;

                    if (timeToNext < timeToTurn)
                    {
                        Deb.Log("FlightPlan.Update: time to turn (speed {5}, remain {0}, timeToNext {1}, amount {2}, rate {3}, timeToTurn {4}",
                            remaining, timeToNext, turnAmount, turnRate, timeToTurn, speed);
                        SequenceWaypoint(george, vessel, vdata);
                        return;
                    }
                }
            }

            if (next != null)
            {
                next.bearing = courseStatus.currentBearing;
                next.distance = courseStatus.currentDistance;
            }

            // ok, we know how far off we are, with dXt, what heading do we need?
            // can we use the PID controller for this?
        }

        public void Update(George george, Vessel vessel, VesselData vdata)
        {
            UpdateData(george, vessel, vdata);
            CheckForSequence(george, vessel, vdata);
        }

        // If on glideslope what decent rate do we want given our speed
        public double GlideSlopeDescentRate(Vessel vessel, VesselData vesselData)
        {
            if (Double.IsNaN(courseStatus.courseSlope))
                return 0; // maybe return NaN; fixme

            var speed = vessel.srfSpeed;

            if (speed > 0)
            {
                // need meters per second to follow glideslope
                // so slope is V/H dimensionless
                // multiply by H speed in M/S gives V speed in M/S
                return courseStatus.courseSlope * speed;
            }
            else
            {
                return 0; // maybe return NaN; fixme
            }

        }

        // what descent rate do we need to get to glideslope based on current state
        public double RequiredDescentRate(Vessel vessel, VesselData vesselData)
        {
            var speed = vessel.srfSpeed;
            var error = courseStatus.dVt;

            // we know the optimal descent slope from courseStatus
            // that would give us a nominal descent rate
            // if we are high we need a higher descent rate
            // if low we need lower descent rate
            // D = distance from glideslope
            // T = time in future
            // GS = glide slope 
            // V = descent rate
            // GV = descent rate for glide slope
            // S = speed (assume constant)
            // In T time we will travel about S * T
            // Traveling S * T the slope will drop (S*T) * GS
            // In time T will will drop T * V meters
            // We want (S*T)*GS to match (T*V)
            // or (S*T*GS) == (T*V) or
            // S*GS = GV
            // But we are D above glideslope so need to drop extra D in time T
            // 
            // If glideslope at our position is A, then our altitude is A+D
            // We want to get to an altitude in time T of A + T * (S*GS)
            // so (A+D)+T*V == A + T * (S*GS)
            // A+D+T*V == A + T * S * GS
            // D+T*V = T*S*GS
            // T*V = T*S*GS-D
            // V = S*GS - D/T
            // So to get there in 10 seconds we need to subtract D/10 from optimal glide slope rate

            if (speed <= 0)
                return 0;

            if (!Double.IsNaN(error) && !Double.IsNaN(courseStatus.courseSlope))
            {
                var nominalRate = courseStatus.courseSlope * speed;
                var timeToIntercept = 5;
                var adjust = (-error / timeToIntercept);
                // if adjust is negative we will be going faster and so in fact we need more delta
                // if positive we will be going slower so need less
                var up_fudge = 0.8;
                var down_fudge = 1.2;

                if (adjust < 0)
                    adjust = adjust * down_fudge;
                else
                    adjust = adjust * up_fudge;

                var result = nominalRate + adjust;

                // Deb.Log("RequiredDescentRate: slp {5}, nom {0}, T {1}, error {2}, adj {3}, result {4}",
                //    nominalRate, timeToIntercept, error, adjust, result, courseStatus.courseSlope, speed);

                return result;
            }
            else
            {
                return Double.NaN;
            }
        }


        public double CourseHeading()
        {
            return courseStatus.courseBearing;
        }

        private void UpdateWayPoint(CelestialBody planet, WayPoint wp)
        {
            if (planet == null)
                return;

            var after = wp.next;

            Deb.Log("UpdateWayPoint: {0}, next={1}", wp, after);

            if (after == null)
                return;

            CourseUtils.BearingAndDistance(planet,
                wp.lat, wp.lon, after.lat, after.lon,
                out after.bearing, out after.distance);
        }

        public void UpdateWayPointValues(CelestialBody planet)
        {
            foreach (var wp in course)
            {
                UpdateWayPoint(planet, wp);
            }
        }

        public void AppendWayPoint(WayPoint wp)
        {
            var last = course.LastOrDefault();
            var clone = wp.Clone();

            clone.ClearFlag(WPFlag.Active);
            clone.ClearFlag(WPFlag.Flown);

            course.Add(clone);

            if (last != null)
            {
                last.next = clone;
            }

            if (planet != null)
                UpdateWayPointValues(planet);
        }

        public void Activate(George george, Vessel vessel, VesselData vdata)
        {
            planet = vessel.mainBody;

            position.lat = vessel.latitude;
            position.lon = vessel.longitude;
            position.alt = vessel.altitude;
            position.SetFlag(WPFlag.Current);

            var start = position.Clone();

            start.ClearFlag(WPFlag.Active);
            start.SetFlag(WPFlag.Flown);
            start.SetFlag(WPFlag.Vertical);
            start.SetFlag(WPFlag.Current);

            start.next = course.FirstOrDefault();
            course.Insert(0, start);

            prev = null;
            next = start;

            UpdateWayPointValues(planet);
            SequenceWaypoint(george, vessel, vdata);
            UpdateWayPointValues(planet);
        }

        // logic: find waypoint and mark all previous waypoints flown
        // insert current pos as first waypoint
        // EXCEPTION; if it would by-pass final approach or IAF to FAF we want to go to the leg
        public void DirectToWaypoint(George george, WayPoint awp, Vessel vessel, VesselData vdata)
        {            
            Deb.Log("DirectToWaypoint: direct to {0}", next);

            if (next != null)
                next.ClearFlag(WPFlag.Active);

            bool activateLeg = false;

            if (awp.HasFlag(WPFlag.RW) || awp.HasFlag(WPFlag.FAF))
            {
                activateLeg = true;
            }

            // should not happen, but just in case
            if (course.Count == 0)
            {
                Deb.Err("DirectToWaypoint: flight plan has no waypoint");
                return;
            }

            // need to copy all waypoints and prune out any "current" ones that are in the middle
            var newCourse = new List<WayPoint>();
            int ndx = -1;
            int j = 0; // in case we skip any

            WayPoint prev_wp = null;
            WayPoint prevToDirect = null;

            for (int i = 0; i < course.Count; i++)
            {
                var wp = course[i];

                if (System.Object.ReferenceEquals(wp, awp))
                {
                    // may have skipped so use index of destination
                    prevToDirect = prev_wp;
                    ndx = j;
                }

                if (i > 0 && wp.HasFlag(WPFlag.Current))
                    continue;

                var newWp = wp.Clone();

                if (prev_wp != null)
                {
                    prev_wp.next= newWp;
                }

                prev_wp = newWp;

                newCourse.Add(newWp);
                j += 1;
            }

            if (ndx < 0)
            {             
                Deb.Err("DirectToWaypoint: wp {0} not found.", awp);
                return;
            }

            // everything before active wp should be marked flown and inactive
            for (int i = 0; i < ndx; i++)
            {
                var wp = newCourse[i];

                wp.SetFlag(WPFlag.Flown);
                wp.ClearFlag(WPFlag.Active);
            }

            // everything after active wp should be marked inactive and NOT flown
            for (int i = ndx; i < newCourse.Count; i++)
            {
                var wp = newCourse[i];

                wp.ClearFlag(WPFlag.Flown);
                wp.ClearFlag(WPFlag.Active);
            }

            if (activateLeg)
            {
                // ignore our position
                if (prevToDirect == null)
                {
                    // should not happen, but just in case
                    Deb.Err("DirectToWaypoint: no previous waypoint for leg!");
                    return;
                }
    
                course = newCourse;

                next = course[ndx];
                prev = prevToDirect.Clone();
                prev.next = next;
                next.SetFlag(WPFlag.Active);
            }
            else
            {
                // insert a fake waypoint for our current position
                course = newCourse;

                var current = position.Clone();
                current.SetFlag(WPFlag.Vertical);
                current.SetFlag(WPFlag.Flown);
                current.SetFlag(WPFlag.Current);

                next = course[ndx];
                course.Insert(ndx, current);
                // don't set old_prev to link to current: old_prev.next = current;
                prev = current;
                prev.next = next;
                next.SetFlag(WPFlag.Active);
            }

            // need fresh status
            courseStatus = new CourseStatus();

            UpdateWayPointValues(planet);
            UpdateData(george, vessel, vdata);
            george.WayPointSequenced(next);
        }

        public void SequenceWaypoint(George george, Vessel vessel, VesselData vdata)
        {
            Deb.Log("SequenceWaypoint: go");

            if (next == null)
            {
                Deb.Log("SequenceWaypoint: no next waypoint.");
                return;
            }

            next.SetFlag(WPFlag.Flown);

            var nextNext = next.next;

            if (nextNext == null)
                return;

            // do not carry over values from last course which are invalid at this point
            courseStatus = new CourseStatus();

            next.ClearFlag(WPFlag.Active);

            // do not just reference the waypoint because we want to modify
            // it in case we have made an early sequence and altitude is messed up
            prev = next.Clone();
            next = nextNext;
            prev.next = next;

            // check prev altitude to make sure we do not dive down or up to
            // altitude
            if (prev.HasFlag(WPFlag.Vertical))
            {
                if (next.HasFlag(WPFlag.RW)) // are we on final approach?
                {
                    Deb.Log("SequenceWaypoint: final approach, do not touch altitude");
                }
                else
                {
                    if (prev.alt < vessel.altitude)
                    {
                        Deb.Log("SequenceWaypoint: adjust alt of prev from {0} to {1}",
                                prev.alt, vessel.altitude);
                        prev.alt = vessel.altitude;
                    }
                    else
                    {
                        Deb.Log("SequenceWaypoint: current alt below prev alt, keep it");
                    }
                }
            }
            else
            {
                Deb.Log("SequenceWaypoint: prev has no vertical info, so fill in our alt");
                prev.SetFlag(WPFlag.Vertical);
                prev.alt = vessel.altitude;
            }

            next.SetFlag(WPFlag.Active);

            UpdateData(george, vessel, vdata);
            george.WayPointSequenced(next);
            CheckForSequence(george, vessel, vdata);
        }
    }


    public class George : ReloadableMonoBehaviour
    {
        #region Globals
        public Vessel vessel = null;
        public VesselData vesselData = null;

        public APController[] controllers = new APController[10];

        // aliases for controllers array above
        public APController hdgBankCtrl;
        public APController bankToYawCtrl;
        public APController aileronCtrl;
        public APController rudderCtrl;
        public APController accelCtrl;
        public APController speedCtrl;
        public APController vertSpeedCtrl;
        public APController elevCtrl;
        public APController altCtrl;
        public APController xtrkSpeedCtrl;

        public GearHandler gearHandler = new GearHandler();

        public FlightPlan flightPlan;

        public bool bLockInput = false;

        public bool HrztActive = false;
        public HrztMode CurrentHrztMode = HrztMode.Heading;
        static GUIContent[] hrztLabels = new GUIContent[3] { 
            new GUIContent("Bank", "Mode: Bank Angle Control"), 
            new GUIContent("Hdg", "Mode: Heading Control - Direction"), 
            new GUIContent("NAV", "Mode: Follow Flight Plan Course"),
        };

        public bool VertActive = false;
        public VertMode CurrentVertMode = VertMode.VSpeed;
        static GUIContent[] vertLabels = new GUIContent[5] { 
            new GUIContent("Pitch", "Mode: Pitch Control"),
            new GUIContent("VSpd", "Mode: Vertical Speed Control"),
            new GUIContent("Alt", "Mode: Altitude Control"),
            new GUIContent("RAlt", "Mode: Radar Altitude Control"),
            new GUIContent("GS", "Mode: Follow Glideslope"), 
        };

        public bool ThrtActive = false;
        public ThrottleMode CurrentThrottleMode = ThrottleMode.Speed;
        static GUIContent[] throttleLabels = new GUIContent[3] { 
            new GUIContent("Vel", "Mode: Velocity Control"),
            new GUIContent("Acc", "Mode: Acceleration Control"),
            new GUIContent("Land", "Mode: Autoland"),
        };

        public LandingMode landingMode = LandingMode.None;

        public CDI indicator = null;

        public static Rect window = new Rect(10, 130, 10, 10);

        public static bool showPresets = false;
        public static bool showPIDLimits = false;
        public static bool showControlSurfaces = false;
        public static bool doublesided = false;

        public bool showCDI = true;

        double targetVert = 0;
        double targetHeading = 0;
        double targetSpeed = 0;

        const string yawLockID = "Pilot Assistant Yaw Lock";
        public static bool yawLockEngaged = false;
        const string pitchLockID = "Pilot Assistant Pitch Lock";
        public static bool pitchLockEngaged = false;

        // rate values for keyboard input
        const double hrztScale = 0.4;
        const double vertScale = 0.4; // altitude rate is x10
        const double throttleScale = 0.4; // acceleration rate is x0.1

        double speedToIAF = 200.0;
        double speedToFAF = 175.0;
        double speedFinal = 100.0;
        double adjSpeedToIAF = 200.0;
        double adjSpeedToFAF = 175.0;
        double adjSpeedFinal = 100.0;

        bool bShowHdg = true;
        bool bShowVert = true;
        bool bShowThrottle = true;
        bool bShowFlightPlan = true;
        bool bShowLandingSpeeds = false;
        bool showPIDValues = false;

        bool bMinimiseHdg = false;
        bool bMinimiseVert = false;
        bool bMinimiseThrt = false;

        Vector2 HdgScrollbar = Vector2.zero;
        public float hdgScrollHeight = 55;
        public static float maxHdgScrollbarHeight = 55;
        Vector2 VertScrollbar = Vector2.zero;
        public float vertScrollHeight = 55;
        public static float maxVertScrollbarHeight = 55;
        Vector2 ThrtScrollbar = Vector2.zero;
        public float thrtScrollHeight = 55;
        public static float maxThrtScrollbarHeight = 55;
        Vector2 FlightPlanScrollbar = Vector2.zero;
        public float flightPlanScrollHeight = 300;
        public static float maxFlightPlanScrollbarHeight = 300;

        float dragStart = 0;
        float dragID = 0; // 0 = inactive, 1 = hdg, 2 = vert, 3 = thrt

        string newPresetName = "";
        static Rect presetWindow = new Rect(0, 0, 200, 10);

        static Rect flightPlanManagerWindow = new Rect(0, 0, 300, 10);

        float pitchSet = 0;

        // Kp, Ki, Kd, Min Out, Max Out, I Min, I Max, Scalar, Easing
        public static readonly double[] defaultHdgBankGains = { 2, 0, 0, -30, 30, -1, 1, 0.1, 1 };
        public static readonly double[] defaultBankToYawGains = { 0, 0, 0, -2, 2, -0.5, 0.5, 1, 1 };
        public static readonly double[] defaultAileronGains = { 0.02, 0.005, 0.01, -1, 1, -1, 1, 1, 1 };
        public static readonly double[] defaultRudderGains = { 0.1, 0.025, 0.05, -1, 1, -1, 1, 1, 1 };
        public static readonly double[] defaultAltitudeGains = { 0.15, 0, 0, -50, 50, 0, 0, 1, 100 };
        public static readonly double[] defaultVSpeedGains = { 2, 0.8, 2, -15, 15, -10, 10, 1, 10 };
        // public static readonly double[] defaultElevatorGains = { 0.05, 0.01, 0.1, -1, 1, -1, 1, 2, 1 }; // old ones
        //public static readonly double[] defaultElevatorGains = { 0.05, 0.2, 0.1, -1, 1, -1, 1, 2, 1 };
        public static readonly double[] defaultElevatorGains = { 0.05, 0.1, 0.1, -1, 1, -1, 1, 2, 1 };
        public static readonly double[] defaultSpeedGains = { 0.2, 0.0, 0.0, -10, 10, -10, 10, 1, 10 };
        public static readonly double[] defaultAccelGains = { 0.2, 0.08, 0.0, -1, 0, -1, 1, 1, 1 };
        public static readonly double[] defaultCdiGains = { 50, 0.0, 10.0, -30, 30, -1, 1, 1.0, 1 };
        public static readonly double[] defaultXtrkGains = { 4.0, 0.8, 2.0, -30, 30, -1, 1, 1.0, 1 };
        public static readonly double[] defaultGSGains = { 0.15, 0, 0, -50, 50, 0, 0, 1, 100 };

        public APController GetController(AsstList id)
        {
            return controllers[(int)id];
        }

        public bool isFlightControlLocked()
        {
            return (InputLockManager.IsLocked(ControlTypes.PITCH) && !pitchLockEngaged) ||
                    InputLockManager.IsLocked(ControlTypes.ROLL) ||
                   (InputLockManager.IsLocked(ControlTypes.YAW) && !yawLockEngaged) ||
                    InputLockManager.IsLocked(ControlTypes.THROTTLE);
        }


        #endregion

        public George()
        {
            Deb.Log("George: Mark IV ctor {0}", this.GetInstanceID());
        }

        public void Awake()
        {
            Debug.Log("George: awaken");
            Deb.Log("George: Awake {0}", this.GetInstanceID());
        }

        private WayPoint AddNamedPoint(double lat, double lon, double alt, String name,
                                       WPFlag flaga = 0,
                                       WPFlag flagb = 0,
                                       WPFlag flagc = 0,
                                       WPFlag flagd = 0)
        {
            var wp = new WayPoint();

            wp.lat = lat;
            wp.lon = lon;
            wp.alt = alt;
            wp.name = name;
            if (flaga != 0) wp.SetFlag(flaga);
            if (flagb != 0) wp.SetFlag(flagb);
            if (flagc != 0) wp.SetFlag(flagc);
            if (flagd != 0) wp.SetFlag(flagd);

            flightPlan.AppendWayPoint(wp);

            return wp;
        }

        private int wpnum = 1;

        private WayPoint AddPoint(double lat, double lon, double alt,
                                  WPFlag flaga = 0,
                                  WPFlag flagb = 0,
                                  WPFlag flagc = 0,
                                  WPFlag flagd = 0)
        {
            return AddNamedPoint(lat, lon, alt, String.Format("WP_{0}", wpnum++),
                                 flaga, flagb, flagc, flagd);
        }

        public void Start()
        {
            Deb.Log("George: Start {0}", this.GetInstanceID());

            Initialise();

            StartFlightPlanManager();

            var comp = AddComponent(typeof(CDI));

            Deb.Log("George: Start: CDI add component result is {0}", comp);

            indicator = comp as CDI;

            if (indicator == null)
            {
                Deb.Err("George: Start-indicator component create failed.");
            }

            InputLockManager.RemoveControlLock(pitchLockID);
            InputLockManager.RemoveControlLock(yawLockID);
            pitchLockEngaged = false;
            yawLockEngaged = false;

            GameEvents.onVesselCreate.Add(VesselSwitched);
            GameEvents.onVesselChange.Add(VesselChanged);
            GameEvents.onVesselDestroy.Add(VesselDestroyed);
            GameEvents.onTimeWarpRateChanged.Add(WarpHandler);


            // vessel may already be active if we started late
            Vessel currentVessel = FlightGlobals.ActiveVessel;

            if (currentVessel != null)
            {
                SwitchVessels(currentVessel);
            }
        }

        void Initialise()
        {
            hdgBankCtrl = controllers[(int)AsstList.HdgBank] = new APController(AsstList.HdgBank, defaultHdgBankGains);
            bankToYawCtrl = controllers[(int)AsstList.BankToYaw] = new APController(AsstList.BankToYaw, defaultBankToYawGains);
            aileronCtrl = controllers[(int)AsstList.Aileron] = new APController(AsstList.Aileron, defaultAileronGains);
            rudderCtrl = controllers[(int)AsstList.Rudder] = new APController(AsstList.Rudder, defaultRudderGains);
            altCtrl = controllers[(int)AsstList.Altitude] = new APController(AsstList.Altitude, defaultAltitudeGains);
            vertSpeedCtrl = controllers[(int)AsstList.VertSpeed] = new APController(AsstList.VertSpeed, defaultVSpeedGains);
            elevCtrl = controllers[(int)AsstList.Elevator] = new APController(AsstList.Elevator, defaultElevatorGains);
            speedCtrl = controllers[(int)AsstList.Speed] = new APController(AsstList.Speed, defaultSpeedGains);
            accelCtrl = controllers[(int)AsstList.Acceleration] = new APController(AsstList.Acceleration, defaultAccelGains);
            xtrkSpeedCtrl = controllers[(int)AsstList.CdiVelocity] =  new APController(AsstList.CdiVelocity, defaultXtrkGains);

            // Set up a default preset that can be easily returned to
            PresetManager.initDefaultPresets(new APPreset(controllers, "default"));

            hdgBankCtrl.invertOutput = true;
            bankToYawCtrl.invertOutput = true;
            aileronCtrl.invertInput = true;
            altCtrl.invertOutput = true;
            vertSpeedCtrl.invertOutput = true;
            elevCtrl.invertOutput = true;
            speedCtrl.invertOutput = true;
            accelCtrl.invertOutput = true;

            aileronCtrl.InMax = 180;
            aileronCtrl.InMin = -180;
            altCtrl.InMin = 0;
            speedCtrl.InMin = 0;
            hdgBankCtrl.isHeadingControl = true; // fix for derivative freaking out when heading target flickers across 0/360
        }

        private void UnhookVessel(Vessel v)
        {
            if (v == null)
            {
                Deb.Log("UnhookVessel: vessel is null");
                return;
            }

            Deb.Log("UnhookVessel: unhooking {0}", v);

            v.OnPreAutopilotUpdate -= new FlightInputCallback(PreAutoPilotUpdate);
            v.OnPostAutopilotUpdate -= new FlightInputCallback(PostAutoPilotUpdate);
        }

        private void SwitchVessels(Vessel v)
        {
            if (v == null)
            {
                Deb.Log("George.SwitchVessels: null vessel");

                if (vessel != null)
                    UnhookVessel(vessel);

                vesselData = null;
                vessel = null;
            }

            Deb.Log("George.SwitchVessels: new vessel {0}", v);

            if (vessel == v)
            {
                Deb.Log("George.switchVessels: same vessel");
                return;
            }

            // so the on vessel change gets called with things that fall off during staging!
            // we do not want to control those and leave the main vessel.
            // hopefully by watching on active vessel we will be OK.
            if (!v.isActiveVessel || v.isEVA)
            {
                Deb.Log("George.switchVessels: new vessel is not active one or EVA, ignore.");
                return;
            }
            
            var lastVessel = vessel;

            vessel = v;

            if (lastVessel != null)
                UnhookVessel(lastVessel);

            if (vessel.mainBody == null)
            {
                Deb.Err("George.SwitchVessels: vessel has no main body");
            }
            // update to use this new vessel
            vesselData = new VesselData();
            vesselData.updateAttitude(vessel);

            vessel.OnPreAutopilotUpdate += new FlightInputCallback(PreAutoPilotUpdate);
            vessel.OnPostAutopilotUpdate += new FlightInputCallback(PostAutoPilotUpdate);

            if (HrztActive)
                InputLockManager.SetControlLock(ControlTypes.YAW, yawLockID);
            if (VertActive)
                InputLockManager.SetControlLock(ControlTypes.PITCH, pitchLockID);

            PresetManager.loadCraftAPPreset(this);

            // test flight plane; fixme
            if (flightPlan != null)
                flightPlan.Activate(this, vessel, vesselData);
        }

        private void VesselSwitched(Vessel v)
        {
            Deb.Log("VesselSwitched: {0}", v);
            SwitchVessels(v);
        }


        private void VesselChanged(Vessel v)
        {
            Deb.Log("VesselChanged: {0}", v);
            SwitchVessels(v);
        }

        private void VesselDestroyed(Vessel v)
        {
            Deb.Log("George.vesselDestroyed: vessel going away {0}", v);

            if (vessel != v)
            {
                Deb.Log("George.vesselDestroyed: not our vessel");
                return;
            }

            SwitchVessels(null);
        }

        public void WarpHandler()
        {
            // reset any setpoints on leaving warp
            if (TimeWarp.CurrentRateIndex == 0 && TimeWarp.CurrentRate != 1 && TimeWarp.WarpMode == TimeWarp.Modes.HIGH)
            {
                HrztActive = false;
                VertActive = false;
                ThrtActive = false;
            }
        }

        public void PreAutoPilotUpdate(FlightCtrlState state)
        {
            if (vessel != null)
            {
                if (!HrztActive && !VertActive && !ThrtActive && !KramaxAutoPilot.bDisplayAutoPilot)
                {
                    return;
                }

                vesselData.updateAttitude(vessel);

                if (flightPlan != null)
                    flightPlan.Update(this, vessel, vesselData);
            }
        }

        public void PostAutoPilotUpdate(FlightCtrlState state)
        {
            if (vessel == null || (!HrztActive && !VertActive && !ThrtActive && !KramaxAutoPilot.bDisplayAutoPilot))
                return;

            vesselController(state);
        }

        // called even when autopilot is not active
        public void AlwaysUpdate()
        {
            gearHandler.AlwaysUpdate(vessel);
        }

        // called only when autopilot is enabled
        public void UpdateWhenEnabled()
        {
            gearHandler.Update(vessel);
            UpdateFlightPlans(this, vessel);
        }

        public void Update()
        {
            // fixme; should get input response working
            // InputResponse();

            AlwaysUpdate();

            if (vessel == null || 
                (!HrztActive && !VertActive && !ThrtActive && !KramaxAutoPilot.bDisplayAutoPilot))
                return;

            UpdateWhenEnabled();
        }

        public void InputResponse()
        {
            if ((vessel == null) || !vessel.isActiveVessel || bLockInput || isFlightControlLocked() || !FlightGlobals.ready)
                return;

            double scale = GameSettings.MODIFIER_KEY.GetKey() ? 10 : 1; // normally *1, with LAlt is *10
            if (FlightInputHandler.fetch.precisionMode)
                scale = 0.1 / scale; // normally *0.1, with alt is *0.01

            // ============================================================ Hrzt Controls ============================================================
            if (HrztActive && !vessel.checkLanded() && Utils.hasYawInput())
            {
                double hdg = GameSettings.YAW_LEFT.GetKey() ? -hrztScale * scale : 0;
                hdg += GameSettings.YAW_RIGHT.GetKey() ? hrztScale * scale : 0;
                hdg += hrztScale * scale * GameSettings.AXIS_YAW.GetAxis();
                if (CurrentHrztMode == HrztMode.Bank)
                {
                    var ctrl = aileronCtrl;
                    ctrl.SetPoint = Utils.headingClamp(ctrl.SetPoint + hdg / 4, 180);
                    targetHeading = ctrl.SetPoint;
                }
                else
                {
                    var ctrl = hdgBankCtrl;
                    ctrl.SetPoint = (ctrl.SetPoint + hdg).headingClamp(360);
                    targetHeading = ctrl.SetPoint;
                }
            }
            // ============================================================ Vertical Controls ============================================================
            if (VertActive && Utils.hasPitchInput())
            {
                double vert = GameSettings.PITCH_DOWN.GetKey() ? -vertScale * scale : 0;
                vert += GameSettings.PITCH_UP.GetKey() ? vertScale * scale : 0;
                vert += !Utils.IsNeutral(GameSettings.AXIS_PITCH) ? vertScale * scale * GameSettings.AXIS_PITCH.GetAxis() : 0;

                if (CurrentVertMode == VertMode.Altitude || CurrentVertMode == VertMode.RadarAltitude)
                {
                    var ctrl = altCtrl;
                    ctrl.SetPoint = Math.Max(ctrl.SetPoint + vert * 10, 0);
                    targetVert = ctrl.SetPoint;
                }
                else if (CurrentVertMode == VertMode.VSpeed)
                {
                    var ctrl = vertSpeedCtrl;
                    ctrl.SetPoint += vert;
                    targetVert = ctrl.SetPoint;
                }
                else if (CurrentVertMode == VertMode.Pitch)
                {
                    var ctrl = elevCtrl;
                    ctrl.SetPoint = Utils.Clamp(ctrl.SetPoint + vert, -90, 90);
                    targetVert = ctrl.SetPoint;
                }
                else if (CurrentVertMode == VertMode.Glideslope)
                {
                    var ctrl = vertSpeedCtrl;
                    ctrl.SetPoint += vert;
                    targetVert = ctrl.SetPoint;
                }
            }
            // ============================================================ Throttle Controls ============================================================
            if (ThrtActive && Utils.hasThrottleInput())
            {
                double speed = GameSettings.THROTTLE_UP.GetKey() ? throttleScale * scale : 0;
                speed -= GameSettings.THROTTLE_DOWN.GetKey() ? throttleScale * scale : 0;
                speed += GameSettings.THROTTLE_FULL.GetKeyDown() ? 100 * scale : 0;
                speed -= (GameSettings.THROTTLE_CUTOFF.GetKeyDown() && !GameSettings.MODIFIER_KEY.GetKey()) ? 100 * scale : 0;

                if (CurrentThrottleMode == ThrottleMode.Speed)
                {
                    var ctrl = speedCtrl;
                    ctrl.SetPoint = Math.Max(ctrl.SetPoint + speed, 0);
                    targetSpeed = ctrl.SetPoint;
                }
                else
                {
                    var ctrl = accelCtrl;
                    ctrl.SetPoint += speed / 10;
                    targetSpeed = ctrl.SetPoint;
                }
            }
        }

        public void UpdateLandingMode(WayPoint wp)
        {
            if (wp == null)
            {
                Deb.Log("UpdateLandingMode: no waypoint");
                return;
            }

            Deb.Log("UpdateLandingMode: {0}", wp);

            if (wp.HasFlag(WPFlag.Stop))
            {
                Deb.Log("UpdateLandingMode: autoland--touchdown sequence started.");
                landingModeChanged(LandingMode.Touchdown);
            }
            else if (wp.HasFlag(WPFlag.IAF))
            {
                Deb.Log("UpdateLandingMode: autoland--flying to IAF.");
                landingModeChanged(LandingMode.ToIAF);
            }
            else if (wp.HasFlag(WPFlag.FAF))
            {
                Deb.Log("UpdateLandingMode: autoland--flying to FAF.");
                landingModeChanged(LandingMode.ToFAF);
            }
            else if (wp.HasFlag(WPFlag.RW))
            {
                Deb.Log("UpdateLandingMode: autoland--on final.");
                landingModeChanged(LandingMode.Final);
            }
            else
            {
                Deb.Log("WayPointSequenced: autoland--not on approach yet");
                landingModeChanged(LandingMode.None);
            }
        }

        public void WayPointSequenced(WayPoint wp)
        {
            Deb.Log("WayPointSequenced: {0} {1} {2} {3}", wp.lat, wp.lon, wp.alt, wp.flags);
            UpdateLandingMode(wp);
        }

        private void landingModeChanged(LandingMode newMode)
        {
            Deb.Log("landingModeChanged: new mode {0}", newMode);
            landingMode = newMode;
        }

        private void hdgModeChanged(HrztMode newMode, bool active, bool setTarget = true)
        {
            hdgBankCtrl.skipDerivative = true;
            bankToYawCtrl.skipDerivative = true;
            aileronCtrl.skipDerivative = true;
            rudderCtrl.skipDerivative = true;
            xtrkSpeedCtrl.skipDerivative = true;

            if (!active)
            {
                InputLockManager.RemoveControlLock(yawLockID);
                yawLockEngaged = false;
                hdgBankCtrl.Clear();
                bankToYawCtrl.Clear();
                aileronCtrl.Clear();
                rudderCtrl.Clear();
                xtrkSpeedCtrl.Clear();
            }
            else
            {
                if (!yawLockEngaged)
                {
                    InputLockManager.SetControlLock(ControlTypes.YAW, yawLockID);
                    yawLockEngaged = true;
                }

                APController ctrl;

                switch (newMode)
                {
                    case HrztMode.Course:
                        // we do not use hdgBankCtrl in course mode, but nice to make
                        // its set point match the current course I guess.
                        if (validFlightPlan())
                        {
                            // go direct to waypoint to avoid huge banks to get on course
                            flightPlan.DirectToWaypoint(this, flightPlan.next, vessel, vesselData);

                            hdgBankCtrl.SetPoint = flightPlan.CourseHeading();

                            var vC = flightPlan.courseStatus.vC;

                            // try to avoid super sharp course corrections
                            if (Double.IsNaN(vC))
                            {
                                // no hope, we have no current cross track velocity
                                xtrkSpeedCtrl.SetPoint = 0;
                            }
                            else
                             {
                                xtrkSpeedCtrl.SetPoint = vC;
                                xtrkSpeedCtrl.BumplessSetPoint = 0;
                             }
                        }
                        else
                        {
                            hdgBankCtrl.SetPoint = 0;
                            xtrkSpeedCtrl.SetPoint = 0;
                        }
                        break;

                    case HrztMode.Heading:
                        if (setTarget)
                            hdgBankCtrl.SetPoint = vesselData.heading;
                        targetHeading = hdgBankCtrl.SetPoint;
                        break;

                    case HrztMode.Bank:
                        ctrl = aileronCtrl;
                        if (setTarget)
                            ctrl.SetPoint = vesselData.bank;
                        targetHeading = ctrl.SetPoint;
                        break;
                }
            }
            HrztActive = active;
            CurrentHrztMode = newMode;
        }

        // newMode is the mode the user is currently requesting, may be same as current mode
        // active is true if we should be controlling pitch at all
        // implicitSet means that we should set the target based on the current flight state
        // it is true when the user just changes the mode or enables the vert AP, it is false when they actually
        // press the "change current target" button.
        private void vertModeChanged(VertMode newMode, bool active, bool implicitSet = true)
        {
            // So what is the logic here?
            // So if they are disabling, pretty clear (though must we invalidate the PID ctrls or not?)
            // Can be turned on via the generic "enable vert AP" button, presumably there is a mode selected
            // but either it has been used before or not. But the "target" is visible to the user.
            // Can be turned on by selecting a "target" value with a given mode. In this case it is clear
            // what to do: change the proper set point to that target. 
            // Also the mode could get changed with us active. In that case the "target" for that mode
            // is actually not controlled by user initially and we must come up with a reasonable target.
            //
            // So specifically: if new mode is pitch and active is true and implicit set is true
            //  -> elev set point is current pitch
            // If implicit set is false
            //  -> elev set point should already be valid

            vertSpeedCtrl.skipDerivative = true;
            elevCtrl.skipDerivative = true;
            altCtrl.skipDerivative = true;

            // If it is going inactive, the "newMode" is not really applicable--it is just the last mode that was used.
            // But if we are inactive and the user simply changes the mode (leaving us inactive) the new mode will be
            // different.
            if (!active)
            {
                // allow the player to control the vertical axis again
                InputLockManager.RemoveControlLock(pitchLockID);
                pitchLockEngaged = false;

                // should we clear these?
                altCtrl.Clear();
                vertSpeedCtrl.Clear();
                elevCtrl.Clear();

                // could use state.pitchTrim here if we wanted to leave craft with most recent pitch as trim
                CurrentVertMode = newMode;
                VertActive = false;

                return;
            }

            // we are active

            // lock the player out of pitch axis controls
            if (!pitchLockEngaged)
            {
                InputLockManager.SetControlLock(ControlTypes.PITCH, pitchLockID);
                pitchLockEngaged = true;
            }

            // used to call PDI.Preset() on all the controllers
            // this sets the integral error sum to something.
            // I can see that you would be concerned that the integral error
            // is no longer valid for new situation, but it caused problems
            // with pitch suddenly moving the wrong direction.

            switch (newMode)
            {
                case VertMode.Pitch:
                    if (implicitSet)
                    {
                        // so the elevCtrl output is a scalar -1 to 1 for how much pitch input is given
                        // the input to elevCtrl is a desired "pitch angle" normally checked against
                        // the vesselData.pitch which is calculated from the planet "up vector" and
                        // the vessel.transform.up.
                        elevCtrl.SetPoint = vesselData.pitch;
                    }
                    targetVert = elevCtrl.SetPoint;
                    break;
                    
                case VertMode.VSpeed:
                    if (implicitSet)
                    {
                        vertSpeedCtrl.SetPoint = vesselData.vertSpeed;
                    }
                    targetVert = vertSpeedCtrl.SetPoint;
                    break;

                case VertMode.Altitude:
                    if (implicitSet)
                    {
                        altCtrl.SetPoint = vessel.altitude;
                    }
                    targetVert = altCtrl.SetPoint;
                    break;

                case VertMode.RadarAltitude:
                    if (implicitSet)
                    {
                        altCtrl.SetPoint = vesselData.radarAlt;
                    }
                    targetVert = altCtrl.SetPoint;
                    break;

                case VertMode.Glideslope:
                    if (validFlightPlan())
                    {
                        vertSpeedCtrl.SetPoint = vesselData.vertSpeed; // flightPlan.GlideSlopeDescentRate(vessel, vesselData);
                        targetVert = vertSpeedCtrl.SetPoint;
                    }
                    else
                    {
                        vertSpeedCtrl.SetPoint = vesselData.vertSpeed;
                        targetVert = vertSpeedCtrl.SetPoint;
                    }
                    break;
            }

            VertActive = active;
            CurrentVertMode = newMode;
        }

        private double getAutoLandSpeed()
        {
            switch (landingMode)
            {
                case LandingMode.None:
                    return vessel.srfSpeed;

                case LandingMode.ToIAF:
                    return speedToIAF;

                case LandingMode.ToFAF:
                    return speedToFAF;

                case LandingMode.Final:
                    return speedFinal;

                case LandingMode.Touchdown:
                case LandingMode.Stop:
                    return 0;
            }

            return vessel.srfSpeed;
        }

        private void throttleModeChanged(ThrottleMode newMode, bool active, bool setTarget = true)
        {
            accelCtrl.skipDerivative = true;
            speedCtrl.skipDerivative = true;

            if (!active)
            {
                accelCtrl.Clear();
                speedCtrl.Clear();
            }
            else
            {
                switch (newMode)
                {
                    case ThrottleMode.Speed:
                        if (setTarget)
                            speedCtrl.SetPoint = vessel.srfSpeed;
                        targetSpeed = speedCtrl.SetPoint;
                        break;

                    case ThrottleMode.Autoland:
                        if (setTarget)
                            speedCtrl.SetPoint = getAutoLandSpeed();
                        targetSpeed = speedCtrl.SetPoint;
                        break;
                    case ThrottleMode.Acceleration:
                        if (setTarget)
                            accelCtrl.SetPoint = vesselData.acceleration;
                        targetSpeed = accelCtrl.SetPoint;
                        break;
                }
            }
            ThrtActive = active;
            CurrentThrottleMode = newMode;
        }

        private bool validFlightPlan()
        {
            if (flightPlan == null)
                return false;

            if (flightPlan.course.Count == 0)
                return false;

            /* causes problems with state changes in guilayout stuff
            if (flightPlan.next == null)
                return false;
            */
            return true;
        }

        public void vesselController(FlightCtrlState state)
        {
            if (state == null)
            {
                Deb.Err("vesselController: state is null");
                return;
            }

            if (vessel == null)
            {
                Deb.Err("vesselController: no vessel");
                return;

            }

            pitchSet = state.pitch; // last pitch ouput, used for presetting the elevator

            if (!vessel.IsControllable)
            {
                Deb.Log("vesselController: not controllable or paused.");
                return;
            }

            bool useIntegral = !vessel.checkLanded();
            bool useIntegralYaw = useIntegral; // on ground we want to use it for yaw

            // Heading Control
            if (HrztActive)
            {
                if (CurrentHrztMode == HrztMode.Course)
                {
                    if (validFlightPlan())
                    {                                     
                        // timeFactor is how long in future we WANT to intercept course
                        double timeFactor = 10;
                        var dXt = flightPlan.courseStatus.dXt;
                        var vXt = flightPlan.courseStatus.vXt;
                        var vC = flightPlan.courseStatus.vC;

                        if (Double.IsNaN(dXt) || Double.IsNaN(vXt) || Double.IsNaN(vC))
                        {
                            Deb.Log("HrztActive: NAV mode, invalid course guidance (NaN), skip setting.");
                        }
                        else
                        {
                            useIntegralYaw = true;

                            double desiredCrossTrackVelocity = -dXt / timeFactor;

                            // we want to make sure
                            // a) we are going the right direction along course (not backwards)
                            // b) that we are not closing at a sharp angle like 90 degrees
                            // We can ensure this by looking at ratio of along-course speed
                            // to cross-track speed. We want positive 2 or greater.

                            // double closingFactor = vC / Math.Max(Math.Abs(vXt), 0.000000000000001);

                            double hdgBankResponse;
                            var velocity = vessel.srfSpeed;

                            // handle going wrong way, going at sharp angle to course or craft is not moving
                            if (velocity < 0.05)
                            {
                                hdgBankResponse = 0;
                            }
                            else
                            {
                                if (vC <= 0)
                                {
                                    // wrong heading, we are going the wrong direction
                                    // if we are left of course (-dXt) and going backwards we need
                                    // a left turn (negative bank).
                                    // or if we are at sharp angle to course bank away from the angle
                                    // hdgBankResponse = dXt < 0 ? 45 : -45; // could have wrong sign here; fixme
                                    desiredCrossTrackVelocity = -desiredCrossTrackVelocity;

                                    // Deb.Log("xtrk wrong way: Vc={0:F1}, vXt={1:F1}, dXt={2:F1}",
                                    //       vC, vXt, dXt);
                                }

                                // do not want to close faster than 2:1 ratio
                                // velocity to x-track
                                double closingFactor = desiredCrossTrackVelocity / velocity;

                                if (-0.5 <= closingFactor && closingFactor <= 0.5)
                                {
                                    // good rate, use it                                                    
                                    xtrkSpeedCtrl.SetPoint = desiredCrossTrackVelocity;

                                    hdgBankResponse =
                                       -xtrkSpeedCtrl.ResponseD(vXt, useIntegralYaw);

                                    //Deb.Log("xtrk valid: Vc={0:F1}, vXt={1:F1}, dXt={2:F1}, des={4:F1}, bank={3:F0}",
                                    //        vC, vXt, dXt, hdgBankResponse, desiredCrossTrackVelocity);
                                }
                                else
                                {
                                    // too fast a closing rate, clamp it
                                    double mod_dctv = (closingFactor < 0) ?
                                    -0.5 * velocity : 0.5 * velocity; // we know velocity is positive

                                    xtrkSpeedCtrl.SetPoint = mod_dctv;

                                    hdgBankResponse =
                                        -xtrkSpeedCtrl.ResponseD(vXt, useIntegralYaw);

                                    //Deb.Log("xtrk too fast: Vc={0:F1}, vXt={1:F1}, dXt={2:F1}, des={4:F1}, mod={5:F1}, bank={3:F1}",
                                    //        vC, vXt, dXt, hdgBankResponse, desiredCrossTrackVelocity, mod_dctv);
                                }
                            }


                            if (vessel.checkLanded())
                            {
                                hdgBankResponse = Utils.Clamp(hdgBankResponse, -20.0, 20.0);

                                var rudderAngle = -hdgBankResponse*0.2;

                                aileronCtrl.SetPoint = 0;
                                rudderCtrl.SetPoint = rudderAngle;

                                var yaw_rate = rudderCtrl.ResponseF(vesselData.yaw, useIntegralYaw).Clamp(-1, 1);
                                Deb.Log("Hrzt: landed, Vc={0:F1} vXt={1:F1} dXt={2:F1} hbr={3:F2} rudder={4:F2} rate={5:F2} yaw={6:F2}", 
                                        vC, vXt, dXt, hdgBankResponse, rudderAngle, yaw_rate, vesselData.yaw);
                            }
                            else
                            {

                                // aileron setpoint updated, bank angle also used for yaw calculations
                                // (don't go direct to rudder because we want yaw stabilisation *or* turn
                                // assistance)
                                bankToYawCtrl.SetPoint = aileronCtrl.SetPoint = hdgBankResponse;
                                rudderCtrl.SetPoint = bankToYawCtrl.ResponseD(vesselData.yaw, useIntegralYaw);
                            }
                        }
                    }
                }
                else if (CurrentHrztMode == HrztMode.Heading)
                {
                    // calculate the bank angle response based on the current heading
                    double hdgBankResponse;

                    hdgBankResponse =
                        hdgBankCtrl.ResponseD(Utils.CurrentAngleTargetRel(vesselData.progradeHeading,
                                                                          hdgBankCtrl.SetPoint, 180),
                                              useIntegral);

                    // aileron setpoint updated, bank angle also used for yaw calculations (don't go direct to rudder because we want yaw stabilisation *or* turn assistance)
                    bankToYawCtrl.SetPoint = aileronCtrl.SetPoint = hdgBankResponse;
                    rudderCtrl.SetPoint = bankToYawCtrl.ResponseD(vesselData.yaw, useIntegral);
                }
                else
                {
                    rudderCtrl.SetPoint = 0;
                }

                // don't want SAS inputs contributing here, so calculate manually
                float rollInput = GameSettings.ROLL_LEFT.GetKey() ? -1 : 0;
                rollInput += GameSettings.ROLL_RIGHT.GetKey() ? 1 : 0;
                rollInput += !Utils.IsNeutral(GameSettings.AXIS_ROLL) ? GameSettings.AXIS_ROLL.GetAxis() : 0;
                rollInput *= FlightInputHandler.fetch.precisionMode ? 0.33f : 1;

#warning Reacts badly to -180 degrees
                state.roll =
                    (aileronCtrl.ResponseF(Utils.CurrentAngleTargetRel(vesselData.bank, aileronCtrl.SetPoint, 180), useIntegralYaw)
                         + rollInput).Clamp(-1, 1);
                state.yaw = rudderCtrl.ResponseF(vesselData.yaw, useIntegralYaw).Clamp(-1, 1);
            }

            if (VertActive)
            {
                if (CurrentVertMode == VertMode.Altitude ||
                    CurrentVertMode == VertMode.RadarAltitude ||
                    CurrentVertMode == VertMode.VSpeed)
                {
                    if (CurrentVertMode == VertMode.Altitude)
                    {
                        vertSpeedCtrl.SetPoint =
                            Utils.Clamp(altCtrl.ResponseD(vessel.altitude, useIntegral),
                                        vessel.srfSpeed * -0.9, vessel.srfSpeed * 0.9);
                    }
                    else if (CurrentVertMode == VertMode.RadarAltitude)
                    {
                        vertSpeedCtrl.SetPoint =
                            Utils.Clamp(getClimbRateForConstAltitude() +
                                        altCtrl.ResponseD(vesselData.radarAlt *
                                                          Vector3.Dot(vesselData.surfVelForward,
                                                                      vessel.srf_velocity.normalized),
                                                          useIntegral),
                                        -vessel.srfSpeed * 0.9, vessel.srfSpeed * 0.9);
                    }

                    var esp = vertSpeedCtrl.ResponseD(vesselData.vertSpeed, useIntegral);

                    if (Math.Abs(vesselData.bank) > 90)
                        esp = -esp; // flying upside down
                    
                    elevCtrl.SetPoint = esp;

                    state.pitch = elevCtrl.ResponseF(vesselData.AoA, useIntegral).Clamp(-1, 1);
                }
                else if (CurrentVertMode == VertMode.Pitch)
                {
                    var pitch_rate = elevCtrl.ResponseF(vesselData.pitch, useIntegral);

                    if (Math.Abs(vesselData.bank) > 90)
                    {
                        Deb.Log("VertMode.Pitch: upside down");
                        pitch_rate *= -1; // flying upside down
                    }

                    state.pitch = pitch_rate.Clamp(-1,1);

                    //Deb.Log("VertMode.Pitch: vessel pitch {0:F1}, setpoint {1:F1}, rate {2:F3}",
                    //        vesselData.pitch, elevCtrl.SetPoint, pitch_rate);
                }
                else if (CurrentVertMode == VertMode.Glideslope)
                {
                    // follow glideslope to maintain proper altitude GSM1
                    if (validFlightPlan())
                    {
                        switch (landingMode)
                        {
                            case LandingMode.None:
                            case LandingMode.ToIAF:
                            case LandingMode.ToFAF:
                            case LandingMode.Final:
                                var drate = flightPlan.RequiredDescentRate(vessel, vesselData);

                                if (Double.IsNaN(drate))
                                {
                                    // if we just sequenced it will not be valid
                                    Deb.Log("VertMode GS, NaN drate, skip setting vspeed.");
                                }
                                else
                                {
                                    vertSpeedCtrl.SetPoint =
                                        Utils.Clamp(drate, vessel.srfSpeed * -0.2, vessel.srfSpeed * 0.2);
                                }
                                break;

                            case LandingMode.Touchdown:
                            case LandingMode.Stop:
                                if (vessel.checkLanded())
                                {
                                    vertSpeedCtrl.SetPoint = -0.1; // keep it on the ground?
                                }
                                else
                                {
                                    var vsp = -(vesselData.radarAlt * 0.5);
                                    vertSpeedCtrl.SetPoint = Utils.Clamp(vsp, -6.0, 0.0);

                                    Deb.Log("VertMode: touching down from {2:F1}, pitch {4:F2}, hsp {3:F0} with vsp {0} (clamped {1})",
                                            vsp, vertSpeedCtrl.SetPoint, vesselData.radarAlt, vessel.srfSpeed,
                                            state.pitch);
                                }
                                break;
                        }

                        elevCtrl.SetPoint = vertSpeedCtrl.ResponseD(vesselData.vertSpeed, useIntegral);

                        if (Math.Abs(vesselData.bank) > 90)
                            elevCtrl.SetPoint *= -1; // flying upside down

                        state.pitch = elevCtrl.ResponseF(vesselData.AoA, useIntegral).Clamp(-1, 1);
                    }
                }
            }

            if (ThrtActive)
            {
                if (CurrentThrottleMode == ThrottleMode.Autoland)
                {
                    bool cutThrottle = false;

                    switch (landingMode)
                    {
                        case LandingMode.None:
                            break;

                        case LandingMode.ToIAF:
                            speedCtrl.SetPoint = speedToIAF;
                            break;

                        case LandingMode.ToFAF:
                            speedCtrl.SetPoint = speedToFAF;
                            break;

                        case LandingMode.Final:
                            speedCtrl.SetPoint = speedFinal;
                            break;

                        case LandingMode.Touchdown:
                        case LandingMode.Stop:
                            speedCtrl.SetPoint = 0;
                            cutThrottle = true;
                            break;
                    }

                    if (cutThrottle)
                    {
                        state.mainThrottle = 0;
                    }
                    else
                    {
                        accelCtrl.SetPoint = speedCtrl.ResponseD(vessel.srfSpeed, useIntegral);
                        state.mainThrottle = accelCtrl.ResponseF(vesselData.acceleration, useIntegral).Clamp(0, 1);
                    }

                    if (landingMode == LandingMode.Stop || vessel.checkLanded())
                    {
                        speedCtrl.SetPoint = 0;
                        vessel.ActionGroups.SetGroup(KSPActionGroup.Brakes, true);
                        cutThrottle = true;
                        state.mainThrottle = 0;
                    }

                }
                else if (vessel.ActionGroups[KSPActionGroup.Brakes] ||
                         (speedCtrl.SetPoint == 0 && vessel.srfSpeed < -accelCtrl.OutMin))
                {
                    state.mainThrottle = 0;
                }
                else
                {
                    if (CurrentThrottleMode == ThrottleMode.Speed)
                    {
                        accelCtrl.SetPoint = speedCtrl.ResponseD(vessel.srfSpeed, useIntegral);
                    }
                    state.mainThrottle = accelCtrl.ResponseF(vesselData.acceleration, useIntegral).Clamp(0, 1);
                }

                FlightInputHandler.state.mainThrottle = state.mainThrottle; // set throttle state permanently
            }
        }

        double getClimbRateForConstAltitude()
        {
            // work out angle for ~1s to approach the point
            // 1.55 is ~89 degrees
            double angle = Math.Min(Math.Atan(4 * vessel.horizontalSrfSpeed / vesselData.radarAlt), 1.55);

            // 0.25 is 14.3 degrees
            if (double.IsNaN(angle) || angle < 0.25)
                return 0; // fly without predictive if high/slow
            else
            {
                double slope = 0;
                terrainSlope(angle, out slope);
                return slope * vessel.horizontalSrfSpeed;
            }
        }

        /// <param name="angle">angle in radians to ping. 0 is straight down</param>
        /// <param name="slope">the calculated terrain slope</param>
        /// <returns>true if an object was encountered</returns>
        bool terrainSlope(double angle, out double slope)
        {
            slope = 0;
            angle += vesselData.pitch * Math.PI / 180;
            double RayDist = findTerrainDistAtAngle((float)(angle * 180 / Math.PI), 10000);
            double AltAhead = 0;
            if (RayDist == -1)
                return false;
            else
            {
                AltAhead = RayDist * Math.Cos(angle);
                if (vessel.mainBody.ocean)
                    AltAhead = Math.Min(AltAhead, vessel.altitude);
            }
            slope = (vesselData.radarAlt - AltAhead) / (AltAhead * Math.Tan(angle));
            return true;
        }

        /// <summary>
        /// raycast from vessel CoM along the given angle, returns the distance at which terrain is detected (-1 if never detected). Angle is degrees to rotate forwards from vertical
        /// </summary>
        float findTerrainDistAtAngle(float angle, float maxDist)
        {
            Vector3 direction = Quaternion.AngleAxis(angle, -vesselData.surfVelRight) * -vesselData.planetUp;
            Vector3 origin = vessel.rootPart.transform.position;
            RaycastHit hitInfo;

            // ~1 masks off layer 0 which is apparently the parts on the current vessel. Seems to work
            if (FlightGlobals.ready && Physics.Raycast(origin, direction, out hitInfo, maxDist, ~1))
                return hitInfo.distance;

            return -1;
        }

        public void FixedUpdate()
        {
            // Deb.Verb("George: FixedUpdate");
        }

        public void LateUpdate()
        {
            // Deb.Verb("George: LateUpdate");
        }

        public void drawGUI()
        {
            if (!KramaxAutoPilot.bDisplayAutoPilot)
                return;

            if (Event.current.type == EventType.Layout)
            {
                bMinimiseHdg = maxHdgScrollbarHeight == 10;
                bMinimiseVert = maxVertScrollbarHeight == 10;
                bMinimiseThrt = maxThrtScrollbarHeight == 10;
            }

            // main window
            #region Main Window resizing (scroll views dont work nicely with GUILayout)

            // Have to put the width changes before the draw so the close button is correctly placed
            float width;
            if (showPIDLimits && controllers.Any(c => controllerVisible(c))) // use two column view if show limits option and a controller is open
                width = 340;
            else
                width = 210;

            width = 340;

            if (bShowHdg && dragID != 1)
            {
                hdgScrollHeight = 0; // no controllers visible when in wing lvl mode unless ctrl surf's are there
                if (CurrentHrztMode != HrztMode.Bank)
                {
                    hdgScrollHeight += hdgBankCtrl.bShow ? 168 : 29;
                    hdgScrollHeight += bankToYawCtrl.bShow ? 140 : 27;
                    hdgScrollHeight += xtrkSpeedCtrl.bShow ? 140 : 27;

                }
                if (showControlSurfaces)
                {
                    hdgScrollHeight += aileronCtrl.bShow ? 168 : 29;
                    hdgScrollHeight += rudderCtrl.bShow ? 168 : 27;
                }
            }
            if (bShowVert && dragID != 2)
            {
                vertScrollHeight = 0;
                if (CurrentVertMode == VertMode.Altitude || CurrentVertMode == VertMode.RadarAltitude)
                    vertScrollHeight += altCtrl.bShow ? 168 : 27;
                if (CurrentVertMode != VertMode.Pitch)
                    vertScrollHeight += vertSpeedCtrl.bShow ? 168 : 29;
                if (showControlSurfaces)
                    vertScrollHeight += elevCtrl.bShow ? 168 : 27;
            }
            if (bShowThrottle && dragID != 3)
            {
                thrtScrollHeight = 0;
                if (CurrentThrottleMode == ThrottleMode.Speed)
                    thrtScrollHeight += speedCtrl.bShow ? 168 : 27;
                thrtScrollHeight += accelCtrl.bShow ? 168 : 29;
            }
            #endregion

            window = GUILayout.Window(382498, window, displayWindow, "", GeneralUI.UISkin.box, GUILayout.Height(0), GUILayout.Width(width));

            // tooltip window. Label skin is transparent so it's only drawing what's inside it
            if (tooltip != "" && KramaxAutoPilot.bDisplayTooltips)
                GUILayout.Window(382499, new Rect(Input.mousePosition.x + 20, // window.x + window.width,
                                                 Screen.height - Input.mousePosition.y + 20,
                                                 300, 0),
                                 tooltipWindow, "", GeneralUI.UISkin.label);

            if (indicator != null)
            {
                indicator.bShowCDI = showCDI;
                indicator.bShowH = (CurrentHrztMode == HrztMode.Course);
                indicator.bShowV = (CurrentVertMode == VertMode.Glideslope);

                if (validFlightPlan())
                {
                    indicator.deviationH = flightPlan.courseStatus.dXt;
                    indicator.deviationV = flightPlan.courseStatus.dVt;
                }
                else
                {
                    indicator.deviationH = 0;
                    indicator.deviationV = 0;
                }
            }

            if (bDisplayFlightPlanManager)
            {
                flightPlanManagerWindow.x = window.x + window.width;
                flightPlanManagerWindow.y = window.y + 200;

                flightPlanManagerWindow =
                    GUILayout.Window(382500, flightPlanManagerWindow, DisplayFlightPlanManagerWindow,
                                     "", GeneralUI.UISkin.box, GUILayout.Width(300));
            }

            if (showPresets)
            {
                // move the preset window to sit to the right of the main window, with the tops level
                presetWindow.x = window.x + window.width;
                presetWindow.y = window.y;

                presetWindow = GUILayout.Window(382501, presetWindow, displayPresetWindow, "", GeneralUI.UISkin.box, GUILayout.Width(200));
            }
        }

        private bool controllerVisible(APController controller)
        {
            if (!controller.bShow)
                return false;
            switch (controller.ctrlID)
            {
                case AsstList.HdgBank:
                case AsstList.BankToYaw:
                    return bShowHdg && CurrentHrztMode != HrztMode.Bank;
                case AsstList.Aileron:
                case AsstList.Rudder:
                    return bShowHdg && showControlSurfaces;
                case AsstList.Altitude:
                    return bShowVert && (CurrentVertMode == VertMode.Altitude || CurrentVertMode == VertMode.RadarAltitude);
                case AsstList.VertSpeed:
                    return bShowVert;
                case AsstList.Elevator:
                    return bShowVert && showControlSurfaces;
                case AsstList.Speed:
                    return bShowThrottle && CurrentThrottleMode == ThrottleMode.Speed;
                case AsstList.Acceleration:
                    return bShowThrottle;
                default:
                    return true;
            }
        }

        private static String FormatLonNum(double lon)
        {
            var x = lon;

            if (x < 0)
            {
                x += 360.0;
            }
            return String.Format("{0:F3}°", x);
        }

        private static String FormatLon(double lon, String format = "{0:F3}")
        {
            var x = lon;

            if (x < -180)
                x += 360;
            else if (x > 180)
                x -= 360;

            String fmt = format + "°";

            if (x < 0)
            {
                fmt += "W";
                x = -x;
            }
            else
            {
                fmt += "E";
            }

            return String.Format(fmt, x);
        }


        private static String FormatLatNum(double lat)
        {
            var x = lat;
            return String.Format("{0:F3}°", x);
        }

        private static String FormatLat(double lat, String format = "{0:F3}")
        {
            var x = lat;

            String fmt = format + "°";

            if (x < 0)
            {
                fmt += "S";
                x = -x;
            }
            else
            {
                fmt += "N";
            }

            return String.Format(fmt, x);
        }

        static private int[] wpFieldWidths = { 37, 100, 45, 30, 50, 40 };

        static private GUIStyle wpLabelStyle_;

        static private GUIStyle wpLabelStyle()
        {
            if (wpLabelStyle_ == null)
            {
                wpLabelStyle_ = new GUIStyle(GeneralUI.UISkin.textArea);
                wpLabelStyle_.active.textColor =
                    wpLabelStyle_.hover.textColor =
                    wpLabelStyle_.focused.textColor =
                    wpLabelStyle_.normal.textColor =
                    wpLabelStyle_.onActive.textColor =
                    wpLabelStyle_.onHover.textColor =
                    wpLabelStyle_.onNormal.textColor =
                    wpLabelStyle_.onFocused.textColor =
                     XKCDColors.Green;
                wpLabelStyle_.alignment = TextAnchor.MiddleRight;
                wpLabelStyle_.fontSize = 11;
            }
            return wpLabelStyle_;
        }

        static private GUIStyle wpLeftLabelStyle_;

        static private GUIStyle wpLeftLabelStyle()
        {
            if (wpLeftLabelStyle_ == null)
            {
                wpLeftLabelStyle_ = new GUIStyle(wpLabelStyle());
                wpLeftLabelStyle_.alignment = TextAnchor.MiddleLeft;
            }
            return wpLeftLabelStyle_;
        }

        static private GUIStyle wpBtnStyle_;

        static private GUIStyle wpBtnStyle()
        {
            if (wpBtnStyle_ == null)
            {
                wpBtnStyle_ = new GUIStyle(GeneralUI.UISkin.button);
                wpBtnStyle_.active.textColor =
                    wpBtnStyle_.hover.textColor =
                    wpBtnStyle_.focused.textColor =
                    wpBtnStyle_.normal.textColor =
                    wpBtnStyle_.onActive.textColor =
                    wpBtnStyle_.onHover.textColor =
                    wpBtnStyle_.onNormal.textColor =
                    wpBtnStyle_.onFocused.textColor =
                     XKCDColors.Green;
                wpBtnStyle_.alignment = TextAnchor.MiddleRight;
                wpBtnStyle_.fontSize = 11;
            }
            return wpBtnStyle_;
        }

        private void drawFieldHeader(String value, int fieldNum)
        {
            GUILayout.Label(value, GUILayout.Width(wpFieldWidths[fieldNum]));
        }

        private void drawField(String value, int fieldNum, bool leftJust = false)
        {
            if (String.IsNullOrEmpty(value))
                GUILayout.Label(value, GUILayout.Width(wpFieldWidths[fieldNum]));
            else
            {
                GUILayout.Label(value,
                                leftJust ? wpLeftLabelStyle() : wpLabelStyle(),
                                GUILayout.Width(wpFieldWidths[fieldNum]));
            }
        }

        private bool drawButtonField(String value, int fieldNum)
        {
            return GUILayout.Button(value, wpBtnStyle(), GUILayout.Width(wpFieldWidths[fieldNum]));
        }


        private void drawWayPointHeader()
        {
            GUILayout.BeginHorizontal();

            var i = 0;

            drawFieldHeader("Leg", i++);
            drawFieldHeader(" Loc", i++);
            drawFieldHeader("Alt(m)", i++);
            drawFieldHeader("Hdg", i++);
            drawFieldHeader("km", i++);

            GUILayout.EndHorizontal();
        }

        private void directToWaypoint(WayPoint wp)
        {
            if (flightPlan != null)
            {
                flightPlan.DirectToWaypoint(this, wp, vessel, vesselData);
            }
        }

        private void drawWayPoint(WayPoint wp)
        {
            if (wp.HasFlag(WPFlag.Current))
                return;

            GUILayout.BeginHorizontal();

            var i = 0;

            if (wp.HasFlag(WPFlag.Active))
            {
                drawField("Active", i++);
            }
            else
            {
                if (drawButtonField("D➔", i++)) // ->
                {
                    directToWaypoint(wp);
                }
            }

            if (!String.IsNullOrEmpty(wp.name))
            {
                drawField(wp.name, i++, true);
            }
            else
            {
                var lat = FormatLat(wp.lat, "{0:F2}");
                var lon = FormatLon(wp.lon, "{0:F2}");
                drawField(String.Format("{0} {1}", lat, lon), i++, true);
            }

            if (wp.HasFlag(WPFlag.Vertical))
                drawField(String.Format("{0:F0}", wp.alt), i++);
            else
                drawField("", i++);

            if (wp.HasFlag(WPFlag.Flown))
            {
                drawField("", i++);
                drawField("", i++);
            }
            else
            {
                drawField(String.Format("{0:F0}°", wp.bearing), i++);
                drawField(String.Format("{0:F1}", wp.distance / 1000.0), i++);
            }

            String notes = "";

            if (wp.HasFlag(WPFlag.IAF))
                notes = "IAF";
            else if (wp.HasFlag(WPFlag.FAF))
                notes = "FAF";
            else if (wp.HasFlag(WPFlag.RW))
                notes = "Final";
            else if (wp.HasFlag(WPFlag.Stop))
                notes = "RW";

            drawField(notes, i++);

            GUILayout.EndHorizontal();
        }

        private void displayWindow(int id)
        {
            var panelWidth = 330;

            GUILayout.BeginHorizontal();

            showPIDValues =
                GUILayout.Toggle(showPIDValues, new GUIContent("PID", "Show/Hide PID Values"),
                                 GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle]);

            showPIDLimits =
                GUILayout.Toggle(showPIDLimits, new GUIContent("Lim", "Show/Hide PID Limits"),
                                 GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle]);

            showControlSurfaces =
                GUILayout.Toggle(showControlSurfaces, new GUIContent("Srf", "Show/Hide Control Surfaces"),
                                 GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle]);

            doublesided =
                GUILayout.Toggle(doublesided, new GUIContent("SmM", "Separate Min and Max limits"),
                                 GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle]);

            KramaxAutoPilot.bDisplayTooltips =
                GUILayout.Toggle(KramaxAutoPilot.bDisplayTooltips, new GUIContent("Tt", "Show/Hide Tooltips"),
                                 GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle]);

            GUILayout.FlexibleSpace();

            showPresets =
                GUILayout.Toggle(showPresets, new GUIContent("P", "Show/Hide Presets"),
                                 GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle]);

            if (GUILayout.Button("X", GeneralUI.UISkin.customStyles[(int)myStyles.redButtonText]))
                KramaxAutoPilot.bDisplayAutoPilot = false;

            GUILayout.EndHorizontal();

            #region Hdg GUI

            GUILayout.BeginHorizontal();

            GUI.backgroundColor = GeneralUI.HeaderButtonBackground;
            bShowHdg = GUILayout.Toggle(bShowHdg, bShowHdg ? "-" : "+",
                GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(20));

            if (HrztActive)
                GUI.backgroundColor = GeneralUI.ActiveBackground;
            else
                GUI.backgroundColor = GeneralUI.InActiveBackground;
            if (GUILayout.Button(HrztActive ? "Roll (Active)" : "Roll", GUILayout.Width(panelWidth - 14)))
                hdgModeChanged(CurrentHrztMode, !HrztActive);

            // reset colour
            GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;
            GUILayout.EndHorizontal();

            if (bShowHdg)
            {
                if (!bMinimiseHdg)
                {
                    HrztMode tempMode =
                        (HrztMode)GUILayout.SelectionGrid((int)CurrentHrztMode, hrztLabels, 3,
                            GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(panelWidth));

                    if (CurrentHrztMode != tempMode)
                        hdgModeChanged(tempMode, HrztActive);
                }
                if (CurrentHrztMode == HrztMode.Course)
                {
                    // should draw CDI and desired heading; fixme
                }
                else if (CurrentHrztMode == HrztMode.Heading)
                {
                    GUILayout.BeginHorizontal();

                    if (GUILayout.Button("Target Hdg: ", GUILayout.Width(90)))
                    {
                        ScreenMessages.PostScreenMessage(String.Format("Heading updated to {0:F0}°", targetHeading));

                        hdgBankCtrl.SetPoint = targetHeading;

                        hdgModeChanged(CurrentHrztMode, true, false);

                        GUI.FocusControl("Target Hdg: ");
                        GUI.UnfocusWindow();
                    }

                    double displayTargetDelta = 0; // active setpoint or absolute value to change (yaw L/R input)

                    displayTargetDelta = hdgBankCtrl.SetPoint - vesselData.heading;
                    displayTargetDelta = displayTargetDelta.headingClamp(180);

                    bool updated = false;

                    targetHeading = GeneralUI.numberEntryBox(targetHeading, out updated, 5.0, "0.00", 51);

                    if (HrztActive && updated)
                    {
                        ScreenMessages.PostScreenMessage(String.Format("Heading updated to {0:F0}°", targetHeading));

                        hdgBankCtrl.SetPoint = targetHeading;
                        hdgModeChanged(CurrentHrztMode, true, false);
                    }

                    GUILayout.Label(displayTargetDelta.ToString("0.00"),
                        GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox],
                        GUILayout.Width(51));

                    GUILayout.EndHorizontal();
                }
                else
                {
                    GUILayout.BeginHorizontal();
                    if (GUILayout.Button("Target Bank: ", GUILayout.Width(90)))
                    {
                        double newBank = targetHeading;

                        aileronCtrl.SetPoint = newBank;
                        hdgModeChanged(CurrentHrztMode, true, false);
                        GUI.FocusControl("Target Bank: ");
                        GUI.UnfocusWindow();
                    }

                    bool updated = false;

                    targetHeading = GeneralUI.numberEntryBox(targetHeading, out updated, 1.0, "0.00", 51);

                    if (HrztActive && updated)
                    {
                        aileronCtrl.SetPoint = targetHeading;
                        hdgModeChanged(CurrentHrztMode, true, false);
                    }

                    if (GUILayout.Button("Level", GUILayout.Width(51)))
                    {
                        aileronCtrl.SetPoint = 0;
                        hdgModeChanged(CurrentHrztMode, true, false);
                    }
                    GUILayout.EndHorizontal();
                }

                if (!bMinimiseHdg && showPIDValues)
                {
                    HdgScrollbar = GUILayout.BeginScrollView(HdgScrollbar, GUIStyle.none, GeneralUI.UISkin.verticalScrollbar, GUILayout.Height(Math.Min(hdgScrollHeight, maxHdgScrollbarHeight)));

                    if (CurrentHrztMode == HrztMode.Course)
                    {
                        if (validFlightPlan())
                        {
                            drawPIDvalues(AsstList.CdiVelocity, "Xtrk", "m/s",
                                          flightPlan.courseStatus.vXt, 2,
                                          "XTV", "\u00B0");
                        }
                    }
                    else if (CurrentHrztMode == HrztMode.Heading ||
                             CurrentHrztMode == HrztMode.Course)
                    {
                        drawPIDvalues(AsstList.HdgBank, "Heading", "\u00B0", vesselData.heading, 2, "Bank", "\u00B0");
                        drawPIDvalues(AsstList.BankToYaw, "Yaw", "\u00B0", vesselData.yaw, 2, "Yaw", "\u00B0", true, false);
                    }
                    if (showControlSurfaces)
                    {
                        drawPIDvalues(AsstList.Aileron, "Bank", "\u00B0", vesselData.bank, 3, "Deflection", "\u00B0");
                        drawPIDvalues(AsstList.Rudder, "Yaw", "\u00B0", vesselData.yaw, 3, "Deflection", "\u00B0");
                    }
                    GUILayout.EndScrollView();
                }

                if (GUILayout.RepeatButton("", GUILayout.Height(8)))
                {// drag resizing code from Dmagics Contracts window + used as a template
                    if (dragID == 0 && Event.current.button == 0)
                    {
                        dragID = 1;
                        dragStart = Input.mousePosition.y;
                        maxHdgScrollbarHeight = hdgScrollHeight = Math.Min(maxHdgScrollbarHeight, hdgScrollHeight);
                    }
                }

                if (dragID == 1)
                {
                    if (Input.GetMouseButtonUp(0))
                        dragID = 0;
                    else
                    {
                        float height = Math.Max(Input.mousePosition.y, 0);
                        maxHdgScrollbarHeight += dragStart - height;
                        hdgScrollHeight = maxHdgScrollbarHeight = Mathf.Clamp(maxHdgScrollbarHeight, 10, 500);
                        if (maxHdgScrollbarHeight > 10)
                            dragStart = height;
                    }
                }

                aileronCtrl.OutMin = Math.Min(Math.Max(aileronCtrl.OutMin, -1), 1);
                aileronCtrl.OutMax = Math.Min(Math.Max(aileronCtrl.OutMax, -1), 1);

                rudderCtrl.OutMin = Math.Min(Math.Max(rudderCtrl.OutMin, -1), 1);
                rudderCtrl.OutMax = Math.Min(Math.Max(rudderCtrl.OutMax, -1), 1);
            }
            #endregion
            #region Pitch GUI

            GUILayout.BeginHorizontal();

            GUI.backgroundColor = GeneralUI.HeaderButtonBackground;
            bShowVert = GUILayout.Toggle(bShowVert, bShowVert ? "-" : "+",
                                         GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle],
                                         GUILayout.Width(20));

            if (VertActive)
                GUI.backgroundColor = GeneralUI.ActiveBackground;
            else
                GUI.backgroundColor = GeneralUI.InActiveBackground;
            if (GUILayout.Button(VertActive ? "Vertical (Active)" : "Vertical", GUILayout.Width(panelWidth - 14)))
                vertModeChanged(CurrentVertMode, !VertActive);

            // reset colour
            GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;
            GUILayout.EndHorizontal();
            if (bShowVert)
            {
                if (!bMinimiseVert)
                {
                    VertMode tempMode =
                        (VertMode)GUILayout.SelectionGrid((int)CurrentVertMode, vertLabels, 5,
                        GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(panelWidth));

                    if (tempMode != CurrentVertMode)
                        vertModeChanged(tempMode, VertActive);
                }

                GUILayout.BeginHorizontal();

                string buttonString = "Target ";
                if (CurrentVertMode == VertMode.VSpeed)
                    buttonString += "Speed";
                else if (CurrentVertMode == VertMode.Altitude)
                    buttonString += "Altitude";
                else if (CurrentVertMode == VertMode.RadarAltitude)
                    buttonString += "Radar Alt";
                else if (CurrentVertMode == VertMode.Pitch)
                    buttonString += "Pitch";

                if (CurrentVertMode != VertMode.Glideslope)
                {
                    if (GUILayout.Button(buttonString, GUILayout.Width(118)))
                    {
                        ScreenMessages.PostScreenMessage(buttonString + " updated");

                        double newVal = targetVert;

                        if (CurrentVertMode == VertMode.Altitude || CurrentVertMode == VertMode.RadarAltitude)
                        {
                            altCtrl.SetPoint = newVal;
                        }
                        else if (CurrentVertMode == VertMode.VSpeed)
                        {
                            vertSpeedCtrl.SetPoint = newVal;
                        }
                        else if (CurrentVertMode == VertMode.Pitch)
                        {
                            elevCtrl.SetPoint = newVal;
                        }

                        vertModeChanged(CurrentVertMode, true, false);

                        GUI.FocusControl("Target Hdg: ");
                        GUI.UnfocusWindow();
                    }
                }

                bool updated = false;

                if (CurrentVertMode == VertMode.VSpeed)
                {
                    targetVert = GeneralUI.numberEntryBox(targetVert, out updated, 10.0, "0.00", 78);

                    if (updated && VertActive)
                        vertSpeedCtrl.SetPoint = targetVert;
                }
                else if (CurrentVertMode == VertMode.Altitude)
                {
                    targetVert = GeneralUI.numberEntryBox(targetVert, out updated, 500.0, "0.00", 78);

                    if (updated && VertActive)
                        altCtrl.SetPoint = targetVert;
                }
                else if (CurrentVertMode == VertMode.RadarAltitude)
                {
                    targetVert = GeneralUI.numberEntryBox(targetVert, out updated, 100.0, "0.00", 78);

                    if (updated && VertActive)
                        altCtrl.SetPoint = targetVert;
                }
                else if (CurrentVertMode == VertMode.Pitch)
                {
                    targetVert = GeneralUI.numberEntryBox(targetVert, out updated, 1.0, "0.00", 78);

                    if (updated && VertActive)
                        elevCtrl.SetPoint = targetVert;
                }

                GUILayout.EndHorizontal();

                if (!bMinimiseVert && showPIDValues)
                {
                    VertScrollbar = GUILayout.BeginScrollView(VertScrollbar, GUIStyle.none, GeneralUI.UISkin.verticalScrollbar, GUILayout.Height(Math.Min(vertScrollHeight, maxVertScrollbarHeight)));
                    if (CurrentVertMode == VertMode.RadarAltitude)
                        drawPIDvalues(AsstList.Altitude, "RAltitude", "m", vesselData.radarAlt, 2, "Speed ", "m/s", true);
                    if (CurrentVertMode == VertMode.Altitude)
                        drawPIDvalues(AsstList.Altitude, "Altitude", "m", vessel.altitude, 2, "Speed ", "m/s", true);
                    if (CurrentVertMode == VertMode.VSpeed)
                        drawPIDvalues(AsstList.VertSpeed, "Vertical Speed", "m/s", vesselData.vertSpeed, 2, "AoA", "\u00B0", true);
                    if (CurrentVertMode == VertMode.Glideslope)
                        drawPIDvalues(AsstList.VertSpeed, "Vertical Speed", "m/s", vesselData.vertSpeed, 2, "AoA", "\u00B0", true);

                    if (showControlSurfaces)
                    {
                        if (CurrentVertMode == VertMode.Pitch)
                        {
                        drawPIDvalues(AsstList.Elevator,
                            "Pitch", "\u00B0", vesselData.pitch, 
                            3, "Deflection", "°", true);
                        }
                        else
                        {
                           drawPIDvalues(AsstList.Elevator,
                                         "Angle of Attack", "°", vesselData.AoA,
                                         3, "Deflection", "°", true);
                        }
                    }

                    elevCtrl.OutMin = Utils.Clamp(elevCtrl.OutMin, -1, 1);
                    elevCtrl.OutMax = Utils.Clamp(elevCtrl.OutMax, -1, 1);

                    GUILayout.EndScrollView();
                }

                if (GUILayout.RepeatButton("", GUILayout.Height(8)))
                {// drag resizing code from Dmagics Contracts window + used as a template
                    if (dragID == 0 && Event.current.button == 0)
                    {
                        dragID = 2;
                        dragStart = Input.mousePosition.y;
                        maxVertScrollbarHeight = vertScrollHeight = Math.Min(maxVertScrollbarHeight, vertScrollHeight);
                    }
                }
                if (dragID == 2)
                {
                    if (Input.GetMouseButtonUp(0))
                        dragID = 0;
                    else
                    {
                        float height = Math.Max(Input.mousePosition.y, 0);
                        maxVertScrollbarHeight += dragStart - height;
                        vertScrollHeight = maxVertScrollbarHeight = Mathf.Clamp(maxVertScrollbarHeight, 10, 500);
                        if (maxVertScrollbarHeight > 10)
                            dragStart = height;
                    }
                }
            }
            #endregion

            #region Throttle GUI

            GUILayout.BeginHorizontal();
            // button background
            GUI.backgroundColor = GeneralUI.HeaderButtonBackground;
            bShowThrottle = GUILayout.Toggle(bShowThrottle, bShowThrottle ? "-" : "+", GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(20));
            if (ThrtActive)
                GUI.backgroundColor = GeneralUI.ActiveBackground;
            else
                GUI.backgroundColor = GeneralUI.InActiveBackground;

            if (GUILayout.Button(ThrtActive ? "Auto-Throttle (Active)" : "Auto-Throttle",
                                 GUILayout.Width(panelWidth - 14)))
            {
                throttleModeChanged(CurrentThrottleMode, !ThrtActive);
            }
            // reset colour
            GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;
            GUILayout.EndHorizontal();

            if (bShowThrottle)
            {
                if (!bMinimiseThrt)
                {
                    ThrottleMode tempMode =
                        (ThrottleMode)GUILayout.SelectionGrid((int)CurrentThrottleMode, throttleLabels, 3,
                                GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(panelWidth));

                    if (tempMode != CurrentThrottleMode)
                        throttleModeChanged(tempMode, ThrtActive);
                }

                GUILayout.BeginHorizontal();

                if (CurrentThrottleMode == ThrottleMode.Speed)
                {
                    bool update_button_pressed = false;

                    if (GUILayout.Button("Target Speed:", GUILayout.Width(118)))
                    {
                        update_button_pressed = true;
                    }

                    bool updated = false;

                    targetSpeed = GeneralUI.numberEntryBox(targetSpeed, out updated, 10.0, "0.00", 78);

                    if (update_button_pressed || (ThrtActive && updated))
                    {
                        GeneralUI.postMessage("Target Speed updated");

                        speedCtrl.SetPoint = targetSpeed;

                        throttleModeChanged(CurrentThrottleMode, true, false);

                        GUI.FocusControl("Target Hdg: ");
                        GUI.UnfocusWindow();
                    }
                }
                else if (CurrentThrottleMode == ThrottleMode.Acceleration)
                {
                    bool update_button_pressed = false;

                    if (GUILayout.Button("Target Accel:", GUILayout.Width(118)))
                    {
                        update_button_pressed = true;
                    }                        

                    bool updated = false;

                    targetSpeed = GeneralUI.numberEntryBox(targetSpeed, out updated, 10.0, "0.00", 78);

                    if (update_button_pressed || (ThrtActive && updated))
                    {
                        GeneralUI.postMessage("Target Accel updated");

                        accelCtrl.SetPoint = targetSpeed;

                        throttleModeChanged(CurrentThrottleMode, true, false);

                        GUI.FocusControl("Target Hdg: ");
                        GUI.UnfocusWindow();
                    }
                }
                else if (CurrentThrottleMode == ThrottleMode.Autoland)
                {
                    GUILayout.Label("Auto-Land Speed:", GUILayout.Width(120));
                    GUILayout.Label(String.Format("{0:F0} m/s", speedCtrl.SetPoint),
                                    GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox],
                                    GUILayout.Width(100));
                }

                GUILayout.EndHorizontal();

                if (!bMinimiseThrt && showPIDValues)
                {
                    ThrtScrollbar =
                        GUILayout.BeginScrollView(ThrtScrollbar, GUIStyle.none,
                                                  GeneralUI.UISkin.verticalScrollbar,
                                                  GUILayout.Height(Math.Min(thrtScrollHeight,
                                                                            maxThrtScrollbarHeight)));

                    if (CurrentThrottleMode == ThrottleMode.Speed)
                        drawPIDvalues(AsstList.Speed, "Speed", " m/s", vessel.srfSpeed, 2, "Accel ", " m/s", true);

                    drawPIDvalues(AsstList.Acceleration, "Acceleration", " m/s", vesselData.acceleration, 1, "Throttle ", "%", true);
                    // can't have people bugging things out now can we...
                    // DRN fixme: is this right? Mixing accelCtrl and speedCtrl?
                    accelCtrl.OutMax = speedCtrl.OutMax.Clamp(-1, 0);
                    accelCtrl.OutMin = speedCtrl.OutMin.Clamp(-1, 0);

                    GUILayout.EndScrollView();
                }

                if (GUILayout.RepeatButton("", GUILayout.Height(8)))
                {// drag resizing code from Dmagics Contracts window + used as a template
                    if (dragID == 0 && Event.current.button == 0)
                    {
                        dragID = 3;
                        dragStart = Input.mousePosition.y;
                        maxThrtScrollbarHeight = thrtScrollHeight = Math.Min(maxThrtScrollbarHeight, thrtScrollHeight);
                    }
                }
                if (dragID == 3)
                {
                    if (Input.GetMouseButtonUp(0))
                        dragID = 0;
                    else
                    {
                        float height = Math.Max(Input.mousePosition.y, 0);
                        maxThrtScrollbarHeight += dragStart - height;
                        thrtScrollHeight = maxThrtScrollbarHeight = Mathf.Clamp(maxThrtScrollbarHeight, 10, 500);
                        if (maxThrtScrollbarHeight > 10)
                            dragStart = height;
                    }
                }
            }
            #endregion

            #region FlightPlan GUI

            GUILayout.BeginHorizontal();
            // button background
            GUI.backgroundColor = GeneralUI.HeaderButtonBackground;
            bShowFlightPlan = GUILayout.Toggle(bShowFlightPlan, bShowFlightPlan ? "-" : "+",
                GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(20));

            if (bShowFlightPlan)
                GUI.backgroundColor = GeneralUI.ActiveBackground;
            else
                GUI.backgroundColor = GeneralUI.InActiveBackground;

            if (GUILayout.Button("Flight Plan",
                                 GUILayout.Width(0.45f * (panelWidth - 14))))
            {
                bShowFlightPlan = !bShowFlightPlan;
            }

            if (!bDisplayFlightPlanManager)
                GUI.backgroundColor = GeneralUI.ActiveBackground;
            else
                GUI.backgroundColor = GeneralUI.InActiveBackground;

            if (GUILayout.Button(new GUIContent("Load/Store", "Load or store a flight plan.")))
            {
                bDisplayFlightPlanManager = !bDisplayFlightPlanManager;
            }

            // reset colour
            GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;
            GUILayout.EndHorizontal();

            if (bShowFlightPlan)
            {
                FlightPlanScrollbar =
                    GUILayout.BeginScrollView(FlightPlanScrollbar, GUIStyle.none,
                         GeneralUI.UISkin.verticalScrollbar,
                         GUILayout.Height(Math.Min(flightPlanScrollHeight, maxFlightPlanScrollbarHeight)));

                GUILayout.BeginVertical();

                GUILayout.BeginHorizontal();

                GUILayout.Label("Plan:", GUILayout.Width(80));

                GUIContent lcontent;

                if (validFlightPlan())
                {
                    lcontent = new GUIContent(flightPlan.name, flightPlan.description);
                }
                else
                {
                    lcontent = new GUIContent("<none>", "Use Load/Store dialog to load flightplan");
                }

                GUILayout.Label(lcontent,
                                GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox]);

                GUILayout.EndHorizontal();

                var f23w = 65;
                var f4w = 120;

                GUILayout.BeginHorizontal();

                GUILayout.Label("Appr Mode:", GUILayout.Width(80));

                var lmode = "none";

                if (vessel.checkLanded())
                {
                    lmode = "Ground";
                }
                else
                {
                    switch (landingMode)
                    {
                        case LandingMode.None:
                            lmode = "Cruise";
                            break;

                        case LandingMode.ToIAF:
                            lmode = "Initial";
                            break;

                        case LandingMode.ToFAF:
                            lmode = "FAF";
                            break;

                        case LandingMode.Final:
                            lmode = "Final";
                            break;

                        case LandingMode.Touchdown:
                            lmode = "Landing";
                            break;

                        case LandingMode.Stop:
                            lmode = "Braking";
                            break;
                    }
                }


                GUILayout.Label(lmode,
                        GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox],
                        GUILayout.Width(f23w));

                GUILayout.EndHorizontal();


                GUILayout.BeginHorizontal();

                GUILayout.Label("Position:", GUILayout.Width(80));

                GUILayout.Label(FormatLat(vessel.latitude),
                        GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox],
                        GUILayout.Width(f23w));

                GUILayout.Label(FormatLon(vessel.longitude),
                        GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox],
                        GUILayout.Width(f23w));

                GUILayout.Label(String.Format("{0:F0}m ({1:F0}m)", vessel.altitude, vesselData.radarAlt),
                                     GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox],
                                     GUILayout.Width(f4w));

                GUILayout.EndHorizontal();


                if (!validFlightPlan())
                {
                    GUILayout.Label("No Flight Plan", GUILayout.Width(200));
                }
                else
                {
                    drawWayPointHeader();

                    foreach (var wp in flightPlan.course)
                    {
                        drawWayPoint(wp);
                    }
                }

                GUILayout.EndVertical();
                GUILayout.EndScrollView();

                if (GUILayout.RepeatButton("", GUILayout.Height(8)))
                {// drag resizing code from Dmagics Contracts window + used as a template
                    if (dragID == 0 && Event.current.button == 0)
                    {
                        dragID = 3;
                        dragStart = Input.mousePosition.y;
                        maxFlightPlanScrollbarHeight = flightPlanScrollHeight = Math.Min(maxFlightPlanScrollbarHeight, flightPlanScrollHeight);
                    }
                }
                if (dragID == 3)
                {
                    if (Input.GetMouseButtonUp(0))
                        dragID = 0;
                    else
                    {
                        float height = Math.Max(Input.mousePosition.y, 0);
                        maxFlightPlanScrollbarHeight += dragStart - height;
                        flightPlanScrollHeight = maxFlightPlanScrollbarHeight = Mathf.Clamp(maxFlightPlanScrollbarHeight, 10, 500);
                        if (maxFlightPlanScrollbarHeight > 10)
                            dragStart = height;
                    }
                }
            }
            #endregion

            GUILayout.BeginHorizontal();
            // button background
            GUI.backgroundColor = GeneralUI.HeaderButtonBackground;
            bShowLandingSpeeds = GUILayout.Toggle(bShowLandingSpeeds, bShowLandingSpeeds ? "-" : "+",
                GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(20));

            if (bShowLandingSpeeds)
                GUI.backgroundColor = GeneralUI.ActiveBackground;
            else
                GUI.backgroundColor = GeneralUI.InActiveBackground;

            if (GUILayout.Button("Approach Speeds", GUILayout.Width(panelWidth - 14)))
            {
                bShowLandingSpeeds = !bShowLandingSpeeds;
            }

            GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;
            GUILayout.EndHorizontal();

            if (bShowLandingSpeeds)
            {
                // fields to enter approach speeds
                var setBtnWidth = 118;
                var textFieldWidth = 88;

                bool update_button_pressed = false;
                bool updated = false;

                GUILayout.BeginHorizontal();
                if (GUILayout.Button(new GUIContent("Set IAF Speed:", "Set speed used when flying to IAF"),
                                     GUILayout.Width(setBtnWidth)))
                {
                    update_button_pressed = true;
                }



                adjSpeedToIAF = GeneralUI.numberEntryBox(adjSpeedToIAF, out updated, 5.0, "0.0", textFieldWidth);
                GUILayout.EndHorizontal();

                if (update_button_pressed || updated)
                {
                    speedToIAF = adjSpeedToIAF;
                    Deb.Log("Changed speedToIAF to {0:F2}", speedToIAF);
                    GeneralUI.postMessage("To IAF speed updated");

                    GUI.FocusControl("Target Hdg: ");
                    GUI.UnfocusWindow();
                }

                update_button_pressed = false;
                updated = false;

                GUILayout.BeginHorizontal();

                if (GUILayout.Button(new GUIContent("Set FAF Speed:", "Set speed used when flying to FAF"),
                                     GUILayout.Width(setBtnWidth)))
                {
                    update_button_pressed = true;
                  
                }

                adjSpeedToFAF = GeneralUI.numberEntryBox(adjSpeedToFAF, out updated, 5.0, "0.0", textFieldWidth);
                GUILayout.EndHorizontal();

                if (update_button_pressed || updated)
                {
                    speedToFAF = adjSpeedToFAF;
                    Deb.Log("Changed speedToFAF to {0:F2}", speedToFAF);
                    GeneralUI.postMessage("To FAF speed updated");

                    GUI.FocusControl("Target Hdg: ");
                    GUI.UnfocusWindow();
                }

                GUILayout.BeginHorizontal();
                if (GUILayout.Button(new GUIContent("Set Final Speed:", "Set speed used when flying final approach"),
                                     GUILayout.Width(setBtnWidth)))
                {
                    update_button_pressed = true;
                    
                }

                adjSpeedFinal = GeneralUI.numberEntryBox(adjSpeedFinal, out updated, 5.0, "0.0", textFieldWidth);
                GUILayout.EndHorizontal();

                if (update_button_pressed || updated)
                {
                    speedFinal = adjSpeedFinal;
                    Deb.Log("Changed speedFinal to {0:F2}", speedFinal);
                    GeneralUI.postMessage("Final approach speed updated");

                    GUI.FocusControl("Target Hdg: ");
                    GUI.UnfocusWindow();
                }
            }

            GUI.DragWindow();
            if (Event.current.type == EventType.Repaint)
                tooltip = GUI.tooltip;
        }

        const string OutMaxTooltip = "The absolute maximum value the controller can output";
        const string OutMinTooltip = "The absolute minimum value the controller can output";

        string tooltip = "";

        private void tooltipWindow(int id)
        {
            GUILayout.Label(tooltip, GeneralUI.UISkin.textArea);
        }

        private void drawPIDvalues(AsstList controllerid, string inputName, string inputUnits, double inputValue, int displayPrecision, string outputName, string outputUnits, bool invertOutput = false, bool showTarget = true)
        {
            var controller = GetController(controllerid);

            controller.bShow = GUILayout.Toggle(controller.bShow, string.Format("{0}: {1}{2}", inputName, inputValue.ToString("N" + displayPrecision.ToString()), inputUnits), GeneralUI.UISkin.customStyles[(int)myStyles.btnToggle], GUILayout.Width(200));

            if (controller.bShow)
            {
                if (showTarget)
                    GUILayout.Label("Target: " + controller.SetPoint.ToString("N" + displayPrecision.ToString()) + inputUnits, GUILayout.Width(200));

                GUILayout.BeginHorizontal();
                GUILayout.BeginVertical();

                controller.PGain = GeneralUI.labPlusNumBox(GeneralUI.KpLabel, controller.PGain.ToString("G3"), 45);
                controller.IGain = GeneralUI.labPlusNumBox(GeneralUI.KiLabel, controller.IGain.ToString("G3"), 45);
                controller.DGain = GeneralUI.labPlusNumBox(GeneralUI.KdLabel, controller.DGain.ToString("G3"), 45);
                controller.Scalar = GeneralUI.labPlusNumBox(GeneralUI.ScalarLabel, controller.Scalar.ToString("G3"), 45);

                if (showPIDLimits)
                {
                    GUILayout.EndVertical();
                    GUILayout.BeginVertical();

                    if (!invertOutput)
                    {
                        controller.OutMax = GeneralUI.labPlusNumBox(new GUIContent(string.Format("Max {0}{1}:", outputName, outputUnits), OutMaxTooltip), controller.OutMax.ToString("G3"));
                        if (doublesided)
                            controller.OutMin = GeneralUI.labPlusNumBox(new GUIContent(string.Format("Min {0}{1}:", outputName, outputUnits), OutMinTooltip), controller.OutMin.ToString("G3"));
                        else
                            controller.OutMin = -controller.OutMax;
                        if (doublesided)
                            controller.ClampLower = GeneralUI.labPlusNumBox(GeneralUI.IMinLabel, controller.ClampLower.ToString("G3"));
                        else
                            controller.ClampLower = -controller.ClampUpper;
                        controller.ClampUpper = GeneralUI.labPlusNumBox(GeneralUI.IMaxLabel, controller.ClampUpper.ToString("G3"));

                        controller.Easing = GeneralUI.labPlusNumBox(GeneralUI.EasingLabel, controller.Easing.ToString("G3"));
                    }
                    else
                    { // used when response * -1 is used to get the correct output
                        controller.OutMin = -1 * GeneralUI.labPlusNumBox(new GUIContent(string.Format("Max {0}{1}:", outputName, outputUnits), OutMaxTooltip), (-controller.OutMin).ToString("G3"));
                        if (doublesided)
                            controller.OutMax = -1 * GeneralUI.labPlusNumBox(new GUIContent(string.Format("Min {0}{1}:", outputName, outputUnits), OutMinTooltip), (-controller.OutMax).ToString("G3"));
                        else
                            controller.OutMax = -controller.OutMin;

                        if (doublesided)
                            controller.ClampUpper = -1 * GeneralUI.labPlusNumBox(GeneralUI.IMinLabel, (-controller.ClampUpper).ToString("G3"));
                        else
                            controller.ClampUpper = -controller.ClampLower;
                        controller.ClampLower = -1 * GeneralUI.labPlusNumBox(GeneralUI.IMaxLabel, (-controller.ClampLower).ToString("G3"));

                        controller.Easing = GeneralUI.labPlusNumBox(GeneralUI.EasingLabel, controller.Easing.ToString("G3"));
                    }
                }
                GUILayout.EndVertical();
                GUILayout.EndHorizontal();
            }
        }

        private void displayPresetWindow(int id)
        {
            if (GUI.Button(new Rect(presetWindow.width - 16, 2, 14, 14), ""))
                showPresets = false;

            if (PresetManager.Instance.activeAPPreset != null) // preset will be null after deleting an active preset
            {
                GUILayout.Label(string.Format("Active Preset: {0}", PresetManager.Instance.activeAPPreset.name));
                if (PresetManager.Instance.activeAPPreset.name != "default")
                {
                    if (GUILayout.Button("Update Preset"))
                        PresetManager.updateAPPreset(this);
                }
                GUILayout.Box("", GUILayout.Height(10), GUILayout.Width(180));
            }

            GUILayout.BeginHorizontal();
            newPresetName = GUILayout.TextField(newPresetName);
            if (GUILayout.Button("+", GUILayout.Width(25)))
                PresetManager.newAPPreset(ref newPresetName, controllers, vessel);
            GUILayout.EndHorizontal();

            GUILayout.Box("", GUILayout.Height(10));

            if (GUILayout.Button("Reset to Defaults"))
                PresetManager.loadAPPreset(PresetManager.Instance.craftPresetDict["default"].apPreset, this);

            GUILayout.Box("", GUILayout.Height(10));

            APPreset presetToDelete = null;
            foreach (APPreset p in PresetManager.Instance.APPresetList)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button(p.name))
                    PresetManager.loadAPPreset(p, this);
                else if (GUILayout.Button("x", GUILayout.Width(25)))
                    presetToDelete = p;
                GUILayout.EndHorizontal();
            }
            if (presetToDelete != null)
            {
                PresetManager.deleteAPPreset(presetToDelete);
                presetWindow.height = 0;
            }
        }

        public void OnGUI()
        {
            // Deb.Verb("George: OnGUI");
            if (KramaxAutoPilot.bHideUI || !KramaxAutoPilot.bDisplayAutoPilot || vessel == null)
                return;

            drawGUI();
        }

        /*
         * Called when the game is leaving the scene (or exiting). Perform any clean up work here.
         */
        public void OnDestroy()
        {
            Deb.Log("George: OnDestroy {0} with vessel {1}", this.GetInstanceID(), vessel);

            UnhookVessel(vessel);

            GameEvents.onVesselCreate.Remove(VesselSwitched);
            GameEvents.onVesselChange.Remove(VesselChanged);
            GameEvents.onVesselDestroy.Remove(VesselDestroyed);
            GameEvents.onTimeWarpRateChanged.Remove(WarpHandler);

            vessel = null;
        }

        #region Flight Plan Manager
        public FlightPlan current;
        private String currentName;
        private String currentDesc;
        public Dictionary<String, List<FlightPlan>> flightPlansDict =
            new Dictionary<String, List<FlightPlan>>();
        public List<FlightPlan> plans = null;
        CelestialBody planet;
        public bool bDisplayFlightPlanManager;

        const string flightPlanPath = "GameData/KramaxAutoPilot/FlightPlans.cfg";
        const string flightPlansNodeName = "KramaxAutoPilotPlans";
        const string defFlightPlanPath = "GameData/KramaxAutoPilot/DefaultFlightPlans.cfg";
        const string defFlightPlansNodeName = "KramaxAutoPilotPlansDefault";

        public void StartFlightPlanManager()
        {
            Deb.Log("FlightPlanManager.Start: load flight plans from config");
            LoadPlansFromConfig();
        }

        public void UpdateFlightPlans(George george, Vessel vessel)
        {
            if (vessel == null)
            {
                current = null;
                plans.Clear();
                planet = null;
            }
            else
            {
                if (!System.Object.ReferenceEquals(current, george.flightPlan))
                {
                    current = george.flightPlan;
                    currentName = String.Copy(current.name);
                    currentDesc = String.Copy(current.description);
                }

                if (planet != vessel.mainBody || plans == null)
                {
                    planet = vessel.mainBody;

                    if (planet != null)
                    {
                        if (flightPlansDict.TryGetValue(planet.name, out plans))
                        {
                            Deb.Log("UpdateFlightPlans: got plans for planet {0}", planet.name);
                        }
                    }
                    else
                    {
                        plans = null;
                    }
                }
            }
        }

        public String GetFlightPlanURI()
        {
            return KSPUtil.ApplicationRootPath.Replace("\\", "/") + flightPlanPath;
        }

        public String GetDefFlightPlanURI()
        {
            return KSPUtil.ApplicationRootPath.Replace("\\", "/") + defFlightPlanPath;
        }

        private void LoadPlansFromNode(ConfigNode node)
        {
            foreach (ConfigNode listNode in node.nodes)
            {
                Deb.Log("LoadPlansFromNode: list node {0}", listNode.name);

                var planet_name = listNode.name;

                List<FlightPlan> fplist = null;

                if (flightPlansDict.TryGetValue(planet_name, out fplist))
                {
                    Deb.Log("LoadPlansFromNode: fplist for planet {0} already exists.", planet_name);
                }
                else
                {
                    Deb.Log("LoadPlansFromNode: create fplist for planet {0}.", planet_name);
                    fplist = new List<FlightPlan>();
                    flightPlansDict[planet_name] = fplist;
                }

                foreach (ConfigNode fpNode in listNode.nodes)
                {
                    FlightPlan fp = new FlightPlan();

                    fp.SetFromConfigNode(fpNode);

                    int existing_index = fplist.FindIndex(afp => (fp.name == afp.name));

                    if (existing_index < 0)
                    {
                        Deb.Log("LoadPlansFromNode: new flight plan named {0}.", fp.name);
                        fplist.Add(fp);
                    }
                    else
                    {
                        Deb.Log("LoadPlansFromNode: existing flight plan named {0} will be overwritten.", fp.name);
                        fplist[existing_index] = fp;
                    }
                }
            }
        }

        private void LoadPlansFromSingleFile(String path)
        {
            ConfigNode root = ConfigNode.Load(path);

            if (root == null)
            {
                Deb.Err("LoadPlansFromFile: load of node at {0} failed.", path);
                return;
            }

            foreach (ConfigNode node in root.nodes)
            {
                Deb.Log("LoadPlansFromFile: tl node {0}", node.name);
                LoadPlansFromNode(node);

                // try to update the in game nodes to ours
                // this will only be done for the non-default nodes
                if (node.name == flightPlansNodeName)
                {
                    Deb.Log("LoadPlansFromFile: try to update ingame nodes...");

                    foreach (ConfigNode enode in GameDatabase.Instance.GetConfigNodes(flightPlansNodeName))
                    {
                        enode.ClearNodes();
                        enode.ClearData();

                        node.CopyTo(enode);
                        // Deb.Log("LoadPlansFromFile: in game node now {0}", enode.ToString());
                    }
                    Deb.Log("LoadPlansFromFile: done.");
                }
            }
        }

        private void LoadPlansFromFiles()
        {
            flightPlansDict.Clear();
            LoadPlansFromSingleFile(GetDefFlightPlanURI());
            LoadPlansFromSingleFile(GetFlightPlanURI());

            planet = vessel.mainBody;

            if (planet != null)
            {
                if (flightPlansDict.TryGetValue(planet.name, out plans))
                {
                    Deb.Log("LoadPlansFromFile: got plans for planet {0}", planet.name);
                }
            }
        }

        private void LoadPlansFromConfigNamed(String nodeName)
        {
            foreach (ConfigNode node in GameDatabase.Instance.GetConfigNodes(nodeName))
            {
                Deb.Log("LoadPlansFromConfig: tl node {0}", node.name);
                LoadPlansFromNode(node);
            }
        }

        private void LoadPlansFromConfig()
        {
            flightPlansDict.Clear();
            LoadPlansFromConfigNamed(defFlightPlansNodeName);
            LoadPlansFromConfigNamed(flightPlansNodeName);
        }

        public void ApplyPlan(FlightPlan plan)
        {
            flightPlan = plan.Clone();
            current = plan;
            currentName = String.Copy(current.name);
            currentDesc = String.Copy(current.description);

            flightPlan.Activate(this, vessel, vesselData);
        }

        public void SavePlan(FlightPlan plan)
        {
            if (plan.planet == null)
            {
                Deb.Err("SavePlan: plan has no planet");
                return;
            }

            if (plan.name == currentName)
            {
                Deb.Log("SavePlan: same name, overwrite current");
                plan.description = currentDesc;
            }
            else
            {
                current = plan.Clone();
                current.name = currentName;
                current.description = currentDesc;
                plan = current;
            }

            flightPlan = current;

            List<FlightPlan> plans_for_planet = null;

            if (!flightPlansDict.TryGetValue(planet.name, out plans_for_planet))
            {
                Deb.Log("SavePlan: creating new plan list for planet {0}", planet.name);
                plans_for_planet = new List<FlightPlan>();
                flightPlansDict[planet.name] = plans_for_planet;
            }

            bool plan_present = false;

            foreach (var pplan in plans_for_planet)
            {
                if (System.Object.ReferenceEquals(plan, pplan))
                {
                    plan_present = true;
                    break;
                }
            }

            if (!plan_present)
            {
                Deb.Log("SavePlan: plan not present, create one in list");
                plans_for_planet.Add(plan);
            }

            ConfigNode rootNode = new ConfigNode();

            ConfigNode node = rootNode.AddNode(flightPlansNodeName);

            // plan_list will be key/value pair
            foreach (var pair in flightPlansDict)
            {
                ConfigNode listNode = node.AddNode(pair.Key);

                foreach (var aplan in pair.Value)
                {
                    listNode.AddNode(aplan.ToConfigNode());
                }
            }

            rootNode.Save(GetFlightPlanURI());
        }

        private void DisplayFlightPlanManagerWindow(int id)
        {
            GUILayout.BeginHorizontal();

            if (bDisplayFlightPlanManager)
                GUI.backgroundColor = GeneralUI.ActiveBackground;
            else
                GUI.backgroundColor = GeneralUI.InActiveBackground;

            if (GUILayout.Button(new GUIContent("Flight Plan Management", "Load or store a flight plan."),
                                 GUILayout.Width(170),
                                 GUILayout.Height(22)))
            {
                bDisplayFlightPlanManager = !bDisplayFlightPlanManager;
            }

            GUI.backgroundColor = GeneralUI.ActiveBackground;

            if (GUILayout.Button(new GUIContent("Refresh", "Reload from " + flightPlanPath),
                                 GUILayout.Width(100),
                                 GUILayout.Height(22)))
            {
                LoadPlansFromFiles();
                GeneralUI.postMessage("Flight plans reloaded from FlightPlans.cfg");
            }

            // reset colour
            GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;

            GUILayout.FlexibleSpace();

            if (GUILayout.Button("X", GeneralUI.UISkin.customStyles[(int)myStyles.redButtonText]))
                bDisplayFlightPlanManager = false;

            GUILayout.EndHorizontal();

            var f1w = 120;
            var f2w = 175;

            GUILayout.BeginHorizontal();
            GUILayout.Label(String.Format("Planet:", GUILayout.Width(f1w)));
            GUILayout.Label(planet == null ? "<none>" : planet.name,
                            GeneralUI.UISkin.customStyles[(int)myStyles.greenTextBox],
                            GUILayout.Width(f2w));
            GUILayout.EndHorizontal();

            if (planet == null)
                return;

            if (current == null)
            {
                GUILayout.Label("<no currently loaded plan>");
            }
            else
            {
                GUILayout.BeginHorizontal();
                GUILayout.Label("Current plan name: ", GUILayout.Width(f1w));

                currentName =
                    GUILayout.TextField(currentName,
                                        GeneralUI.UISkin.customStyles[(int)myStyles.numBoxText],
                                        GUILayout.Width(f2w));

                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Current plan info: ", GUILayout.Width(f1w));

                currentDesc =
                    GUILayout.TextField(currentDesc,
                                        GeneralUI.UISkin.customStyles[(int)myStyles.numBoxText],
                                        GUILayout.Width(f2w));

                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.Label("Modify Plan: ", GUILayout.Width(f1w));
                if (GUILayout.Button(new GUIContent("Update/Rename",
                                                    "Write out with name and description from above."),
                                     GeneralUI.UISkin.customStyles[(int)myStyles.leftButtonText],
                                     GUILayout.Width(f2w)))
                {
                    SavePlan(current);
                }
                GUILayout.EndHorizontal();
            }

            GUILayout.Space(5);

            GUI.backgroundColor = GeneralUI.ActiveBackground;

            if (GUILayout.Button(new GUIContent("Available Plans", "Click on the plan name to load it.")))
            {
            }

            // reset colour
            GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;


            if (plans == null || plans.Count == 0)
            {
                GUILayout.Label(String.Format("<no flight plans for {0}>", planet.name));
            }
            else
            {
                foreach (var fp in plans)
                {
                    if (GUILayout.Button(new GUIContent(fp.name, fp.description),
                                         GeneralUI.UISkin.customStyles[(int)myStyles.leftButtonText]))
                    {
                        ApplyPlan(fp);
                    }
                }
            }

            if (Event.current.type == EventType.Repaint)
                tooltip = GUI.tooltip;
        }
        #endregion
    }
}
