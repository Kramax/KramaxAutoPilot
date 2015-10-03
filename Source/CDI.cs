using Kramax.Utility;
using KramaxReloadExtensions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace Kramax
{
    // Used to use code from http://wiki.unity3d.com/index.php?title=DrawLine but no longer needed.
    public class Drawing
    {
        static Texture2D lineTex = null;

        public static void DrawRect(Rect rect, Color color)
        {
            // Generate a single pixel texture if it doesn't exist
            if (!lineTex) { lineTex = new Texture2D(1, 1); }

            // Store current GUI color, so we can switch it back later,
            // and set the GUI color to the color parameter
            Color savedColor = GUI.color;
            GUI.color = color;

            // Finally, draw the actual line.
            // We're really only drawing a 1x1 texture from pointA.
            // The matrix operations done with ScaleAroundPivot and RotateAroundPivot will make this
            //  render with the proper width, length, and angle.
            GUI.DrawTexture(rect, lineTex);

            GUI.color = savedColor;
        }

        static Texture2D diamondTex = null;
                                          
        private static void CreateDiamondTex()
        {
            if (diamondTex == null)
            {
                int sx = 19;
                int sy = 19;
                int hy = 9;

                var off = new Color(0, 0, 0, 0);
                var on = new Color(1, 1, 1, 1);
                
                diamondTex = new Texture2D(sx, sy, TextureFormat.ARGB32, false);

                // clear it to start out
                for(int x = 0; x<sx; x++)
                {
                    for(int y = 0; y<sy; y++)
                        diamondTex.SetPixel(x, y, off);
                }
 
                sx -= 1;
                sy -= 1;

                for(int y=0; y<hy; y++)
                {
                    for(int x=hy-y; x<hy+y; x++)
                    {
                        diamondTex.SetPixel(x, y, on);
                        diamondTex.SetPixel(x, sy-y, on);
                    }
                }

                for(int x=0; x<sx; x++)
                    diamondTex.SetPixel(x, hy, on);

                diamondTex.Apply();
            }
         }

        public static void DrawDiamond(Rect rect, Color color)
        {
            if (diamondTex == null)
                CreateDiamondTex();

            // Save the current GUI matrix, since we're going to make changes to it.
            Matrix4x4 matrix = GUI.matrix;

            // Store current GUI color, so we can switch it back later,
            // and set the GUI color to the color parameter
            Color savedColor = GUI.color;
            GUI.color = color;

            // Finally, draw the actual line.
            // We're really only drawing a 1x1 texture from pointA.
            // The matrix operations done with ScaleAroundPivot and RotateAroundPivot will make this
            //  render with the proper width, length, and angle.
            GUI.DrawTexture(rect, diamondTex);

            // We're done.  Restore the GUI matrix and GUI color to whatever they were before.
            GUI.matrix = matrix;
            GUI.color = savedColor;
        }
    }

    public class CDI : ReloadableMonoBehaviour
    {             
        public bool bShowCDI = true;
        public bool bShowH = true;
        public bool bShowV = true;
        public double deviationH = 0;
        public double deviationV = 0;
        public Rect window = new Rect(655, 965, 300, 255);
        public Rect navBallRect = new Rect(100, 100, 200, 200);

        public void Awake()
        {
            Debug.Log("CDI: awaken");
            Deb.Log("CDI: Awake {0}", this.GetInstanceID());
        }

        public void Start()
        {
            Deb.Log("CDI: Start {0}", this.GetInstanceID());
            
            GetNavBallRect();
            AdjustRectForNavBallRect();
        }

        public void OnGUI()
        {
            if (!KramaxAutoPilot.bDisplayAutoPilot)
                return;

            drawGUI();
        }

        public void AdjustRectForNavBallRect()
        {
            // ratios come from measuring at 1600x1200 and using nominal position
            // and size of Rect(655, 965, 300, 255)
            window.width = (int)(navBallRect.width * (300.0f/223.0f));
            window.height = (int)(navBallRect.height * (255.0f / 200.0f));
            window.x = (int) (0.5+navBallRect.x - ((688.0f - 655.0f) * (window.width / 300.0f)));
            window.y = (int) (0.5+navBallRect.y - ((1000 - 965) * (window.height / 255.0f)));

            Deb.Log("CDI overlay rect: {0}", window);
        }

        public void GetNavBallRect()
        {
            var navball = GameObject.FindObjectOfType<NavBall>();

            if (navball == null)
            {
                Deb.Err("GetNavBallArea: no navball object found");
            }

            Deb.Log("Navball game object: {0}", navball);
            Deb.Log("Navball transform: {0}", navball.transform.localToWorldMatrix);
            Deb.Log("Navball transform.position: {0}", navball.transform.position);

            // position is (1.0,0.1,0.0) when visible
            // position is (1.0,-0.1,0.0) when not visible
            var camera = ScreenSafeUI.referenceCam;

            // this looks to be the center of the ball with Y counting up from bottom of screen
            // so 800,85 for 1600x1200 screen.
            Vector3 center = camera.WorldToScreenPoint(navball.transform.position);
            Deb.Log("Navball center in screen point: {0}", center);

            // cannot figure out how to get actual size. Just assume that since we have the center
            // and the Y is offset from 0 the size is about twice that.
            // the 200 and 223 come from measuring this in a 1600x1200 frame
            float height = (float)center.y * (200.0f / 85.6f);
            float width = (float)center.y * (223.0f / 85.6f);

            // this does not work; maybe only valid inside GUI draw calls
            // Vector2 guiCenter = GUIUtility.ScreenToGUIPoint(new Vector2(center.x, center.y));

            // Just assume as is the case currently with unity that the Y reverses direction
            navBallRect =
                new Rect(center.x - width / 2, Screen.height - height, width, height);

            Deb.Log("Navball Rect calculated to be: {0}", navBallRect);
        }        


        public void drawGUI()
        {
            /*
            if (Event.current.type == EventType.Layout)
            {
            }
            */

            if (bShowCDI)
            {
                //cdiWindow.x = window.x + window.width;
                //cdiWindow.y = window.y + 100;

                GUI.backgroundColor = Color.clear;
                window =
                    GUI.Window(34248, window, DisplayCDI, "",
                    // GeneralUI.UISkin.customStyles[(int)myStyles.txWindow],
                                     GeneralUI.UISkin.box);
                GUI.backgroundColor = GeneralUI.stockBackgroundGUIColor;
            }
        }

        private void DrawIndicatorH(Rect rect, float value, float minV, float maxV)
        {
            GUI.BeginGroup(rect);
         
            bool clamped = false;

            if (value < minV)
            {
                value = minV;
                clamped = true;
            }
            if (value > maxV)
            {
                value = maxV;
                clamped = true;
            }

            var sx = rect.width;
            var sy = rect.height;

            GUI.Box(new Rect(0, 0, sx, sy), "");

            Color color = XKCDColors.NeonGreen;

            float spacing = (float)Math.Floor(sx * 0.11f + 0.5f);
            float middle = (float)Math.Floor(sx * 0.5f + 0.5f);

            var lineRect = new Rect(0, 3, 1, sy - 6);

            for (float x = middle + spacing; x < sx; x += spacing)
            {
                lineRect.x = x;
                Drawing.DrawRect(lineRect, color);
            }

            for (float x = middle - spacing; x > 0; x -= spacing)
            {
                lineRect.x = x;
                Drawing.DrawRect(lineRect, color);
            }

            lineRect.x = middle-1;
            lineRect.y = 1;
            lineRect.height = sy - 3;
            lineRect.width = 3;
            color = XKCDColors.NeonBlue;

            Drawing.DrawRect(lineRect, color);

            // our box runs for 0 to sx-1
            // we do not want diamond to get clipped
            // so lowest y should be 0 and highest should be (sx-1)-9
            // center should draw at middle - 4
            var frac = -0.5f + (value - minV) / (maxV - minV); // range -0.5 to 0.5
            var range = (sx - 1) - 9;
            float xx = middle + range * frac - 4;

            var dcolor = clamped ? XKCDColors.NeonRed : XKCDColors.NeonBlue;
            Drawing.DrawDiamond(new Rect(xx, 3, 9, 15), dcolor);

            GUI.EndGroup();
        }

        private void DrawIndicatorV(Rect rect, float value, float minV, float maxV)
        {
            GUI.BeginGroup(rect);

            bool clamped = false;

            if (value < minV)
            {
                value = minV;
                clamped = true;
            }
            if (value > maxV)
            {
                value = maxV;
                clamped = true;
            }

            var sx = rect.width;
            var sy = rect.height;

            GUI.Box(new Rect(0, 0, sx, sy), "");

            Color color = XKCDColors.NeonGreen;

            float spacing = (float)Math.Floor(sy * 0.11f + 0.5f);
            float middle = (float)Math.Floor(sy * 0.5f + 0.5f);
            var lineRect = new Rect(3, 0, sx - 6, 1);

            for (float y = middle + spacing; y < sy; y += spacing)
            {
                lineRect.y = y;
                Drawing.DrawRect(lineRect, color);
            }

            for (float y = middle - spacing; y > 0; y -= spacing)
            {
                lineRect.y = y;
                Drawing.DrawRect(lineRect, color);
            }

            lineRect.y = middle - 1;
            lineRect.x = 1;
            lineRect.width = sx - 3;
            lineRect.height = 3;
            color = XKCDColors.NeonBlue;

            Drawing.DrawRect(lineRect, color);

            // our box runs for 0 to sy-1
            // we do not want diamond to get clipped
            // so lowest y should be 0 and highest should be (sy-1)-9
            // center should draw at middle - 4
            var frac = -0.5f + (value - minV) / (maxV - minV); // range -0.5 to 0.5
            var range = (sy-1)-9;
            float yy = middle + range*frac - 4;

            var dcolor = clamped ? XKCDColors.NeonRed : XKCDColors.NeonBlue;
            Drawing.DrawDiamond(new Rect(3, yy, 15, 9), dcolor);

            GUI.EndGroup();
        }

        private void DisplayCDI(int id)
        {
            var width = window.width;
            var height = window.height;
            float ctrl_width = width - 60;
            float ctrl_height = height - 80;

            float minV = -1;
            float maxV = 1;

            GUI.backgroundColor = Color.clear;

            if (bShowH)
            {
                float delta = (float)deviationH;
                float value;

                String deltaString;

                if (delta < 0)
                {
                    value = (float)(Math.Sqrt(-delta) * -0.1);

                    if (delta > -0.5f)
                        deltaString = "0 m";
                    else
                        deltaString = String.Format("{0:F0} m", delta);
                }
                else
                {
                    value = (float)(Math.Sqrt(delta) * 0.1);

                    if (delta < 0.5f)
                        deltaString = "0 m";
                    else
                        deltaString = String.Format("+{0:F0} m", delta);
                }

                GUI.Label(new Rect(20, 0, ctrl_width, 20), deltaString,
                    GeneralUI.UISkin.customStyles[(int)myStyles.cdiLabel]);

                GUI.backgroundColor = XKCDColors.NeonRed;
                DrawIndicatorH(new Rect(20, 20, ctrl_width, 20), -value, minV, maxV);
                GUI.backgroundColor = Color.clear;
            }

            if (bShowV)
            {
                double delta = deviationV;
                float value;

                String deltaString;

                if (delta < 0)
                {
                    value = (float)(Math.Sqrt(-delta) * -0.1);

                    if (delta > -0.5f)
                        deltaString = "0 m";
                    else
                        deltaString = String.Format("{0:F0} m", delta);
                }
                else
                {
                    value = (float)(Math.Sqrt(delta) * 0.1);

                    if (delta < 0.5f)
                        deltaString = "0 m";
                    else
                        deltaString = String.Format("+{0:F0} m", delta);
                }

                GUI.backgroundColor = XKCDColors.NeonRed;
                DrawIndicatorV(new Rect(width - 40, 40, 20, ctrl_height), value, minV, maxV);
                GUI.backgroundColor = Color.clear;

                float barX = (width - 30) - 2;

                GUI.Label(new Rect(barX - 45, 40 + ctrl_height, 90, 20), deltaString,
                  GeneralUI.UISkin.customStyles[(int)myStyles.cdiLabel]);
            }    

            /* GUI.DragWindow(); */
        }
    }
}
