
namespace Kramax
{
    public class FuzzyMembership
    {
        public FuzzyMembership()
        {
        }

        public virtual float Eval(float x)
        {
            return 0;
        }
    }

    public class FuzzyTrap : FuzzyMembership
    {
        public float left0, left100, right100, right0;
        public float r_left_range, r_right_range;

        public FuzzyTrap()
        {
        }

        public FuzzyTrap(float left0_, float left100_, float right100_, float right0_)
        {
            left0 = left0_;
            left100 = left100_;
            right100 = right100_;
            right0 = right0_;
            r_left_range = 1 / (left100 - left0);
            r_right_range = 1 / (right0 - right100);
        }

        // zero to one-ness
        public float RealEval(float x)
        {
            if (x < left0)
                return 0;
            if (x > right0)
                return 0;

            if (left100 <= x && x <= right100)
                return 1;

            if (x < left100)
                return (x - left0) * r_left_range;

            // if (x > right100)
            return (right0 - x) * r_right_range;
        }

        public override float Eval(float x)
        {
            var result = RealEval(x);
            // Deb.Log("Eval({0}) => {1}", x, result);
            return result;
        }

    }

    // 1 at negative infinity slope down from left100 to left0
    public class FuzzyTriLeft : FuzzyMembership
    {
        public float left0, left100;
        public float r_left_range;

        public FuzzyTriLeft(float left100_, float left0_)
        {
            left0 = left0_;
            left100 = left100_;
            r_left_range = 1 / (left100 - left0);
        }

        // zero to one-ness
        public override float Eval(float x)
        {
            if (x <= left100)
                return 1;

            if (x >= left0)
                return 0;

            return (x - left0) * r_left_range;
        }
    }

    // slope up from right0 to right 100 and one at infinity
    public class FuzzyTriRight : FuzzyMembership
    {
        public float right0, right100;
        public float r_right_range;

        public FuzzyTriRight(float right0_, float right100_)
        {
            right0 = right0_;
            right100 = right100_;
            r_right_range = 1 / (right100 - right0);
        }

        // zero to one-ness
        public override float Eval(float x)
        {
            if (x <= right0)
                return 0;

            if (x >= right100)
                return 1;

            return (x - right0) * r_right_range;
        }
    }

    public class SlowFuzzyResultTri : FuzzyTrap
    {
        // triangle 0 at left grows to 1 at center and diminish back to 0 at right
        float left, center, right;
        float delta_left; // center_minus_left0;
        float delta_right; // center_minus_right0, a negative number
        float max_val = 1;

        public SlowFuzzyResultTri(float l, float c, float r)
        {
            left = l;
            center = c;
            right = r;

            left0 = left;
            right0 = right;
            delta_left = center - left0;
            delta_right = center - right0;

            // following will get modified by Clamp, but get valid values anyways
            right100 = center;
            left100 = center;

            r_left_range = 1 / (left100 - left0);
            r_right_range = 1 / (right0 - right100);
        }

        public override float Eval(float x)
        {
            var result = RealEval(x);
            result *= max_val;

            // Deb.Log("Eval({0}) => {1}", x, result);
            return result;
        }


        // cut off the top of the triangle at height to form a trapezoid
        public void Clamp(float y)
        {
            max_val = y;

            // need to find left100 and right100
            // line is from (left0,0) to (center,1)
            // y = mx+b
            // m = 1/(center-left0)
            // y = x/(center-left0)+b
            // 0 = left0/(center-left0)+b
            // b = -left0/(center-left0)
            // y = x/(center-left0)-left0/(center-left0)
            // y+left0/(center-left0) = x/(center-left0)
            // x = (center-left0)*(y+left0/(center-left0))
            // x = y*(center-left0) + left0;
            // x = y*delta_left + left;
            // x is left100
            left100 = y * delta_left + left;

            // line is from (right0,0) to (center,1)
            // y = mx+b
            // m = 1/(center-right0)
            // y = x/(center-right0)+b
            // 0 = right0/(center-right0)+b
            // b = -right0/(center-right0)
            // y = x/(center-right0)-right0/(center-right0)
            // y+right0/(center-right0) = x/(center-right0)
            // x = (center-right0)*(y+right0/(center-right0))
            // x = y*(center-right0) + right0;
            // x = y*delta_right + right;
            // x is right100
            right100 = y * delta_right + right;

            r_left_range = 1 / (left100 - left0);
            r_right_range = 1 / (right0 - right100);
        }
    }

    public class FuzzyResultTri
    {
        // triangle 0 at left grows to 1 at center and diminish back to 0 at right
        float left, center, right;
        float r_delta_left; // center_minus_left0;
        float r_delta_right; // center_minus_right0, a negative number
        float max_val = 1;

        public FuzzyResultTri(float l, float c, float r)
        {
            left = l;
            center = c;
            right = r;

            r_delta_left = 1/(center - left);
            r_delta_right = 1/(center - right);
        }

        public float Eval(float x)
        {
            if (x <= left) return 0;

            if (x >= right) return 0;

            float result =
                result = (x <= center) ? (x-left)*r_delta_left : (x-right)*r_delta_right;
    
            if (result < max_val) return result;
            
            return max_val;
        }

        // cut off the top of the triangle at height to form a trapezoid
        public void Clamp(float y)
        {
            max_val = y;
        }
    }

    public class FuzzyNavCtrl
    {
        private FuzzyMembership leftOfCourse;
        private FuzzyMembership rightOfCourse;
        private FuzzyMembership zeroCourse;
        private FuzzyMembership rightToLeft;
        private FuzzyMembership leftToRight;
        private FuzzyMembership parallel;
        private FuzzyResultTri turnHardLeft;
        private FuzzyResultTri turnLeft;
        private FuzzyResultTri turnSlightLeft;
        private FuzzyResultTri turnNone;
        private FuzzyResultTri turnSlightRight;
        private FuzzyResultTri turnRight;
        private FuzzyResultTri turnHardRight;

        public FuzzyNavCtrl()
        {
            // membership funcs for Cross-Track Error Et (meters)
            // leftOfCourse, ZeroCourse, rightOfCourse
            leftOfCourse = new FuzzyTriLeft(-2000, 0);
            rightOfCourse = new FuzzyTriRight(0, 2000);
            zeroCourse = new FuzzyTrap(-2000, -1, 1, 2000);

            // membership funcs for Cross-Track Velocity Vt (meters/sec)
            // leftToRight, rightToLeft, and parallel
            rightToLeft = new FuzzyTriLeft(-50, 0);
            leftToRight = new FuzzyTriRight(0, 50);
            parallel = new FuzzyTrap(-50, 0, 0, 50);

            // membership funcs for output turn rate in degrees/second
            // standard rate turn is 3 degrees per second
            turnHardLeft = new FuzzyResultTri(-9, -6, -3);
            turnLeft = new FuzzyResultTri(-5, -3, -1);
            turnSlightLeft = new FuzzyResultTri(-3, -0.25f, 0);
            turnNone = new FuzzyResultTri(-0.5f, 0, 0.5f);
            turnSlightRight = new FuzzyResultTri(0, 0.25f, 3);
            turnRight = new FuzzyResultTri(1, 3, 5);
            turnHardRight = new FuzzyResultTri(3, 6, 9);
        }

        private float FzAnd(float x, float y)
        {
            return x * y; // could do min as well
        }

        private float MaxF(float x, float y)
        {
            return (x < y) ? y : x;
        }

        // how hard we should turn left or right
        // Given:
        // Delta time in seconds (deltaT),
        // Cross-Track Error in meters (Et),
        // Cross-Track Velocity in meters/sec (Vt)
        public float TurnResponse(float deltaT, float Et, float Vt)
        {
            var left_ness = leftOfCourse.Eval(Et);
            var right_ness = rightOfCourse.Eval(Et);
            var oncourse_ness = zeroCourse.Eval(Et);

            var rightToLeft_ness = rightToLeft.Eval(Vt);
            var leftToRight_ness = leftToRight.Eval(Vt);
            var parallel_ness = parallel.Eval(Vt);

            /*
            Deb.Log("left_ness: {0}", left_ness);
            Deb.Log("right_ness: {0}", right_ness);
            Deb.Log("oncourse_ness: {0}", oncourse_ness);
            Deb.Log("rightToLeft_ness: {0}", rightToLeft_ness);
            Deb.Log("leftToRight_ness: {0}", leftToRight_ness);
            Deb.Log("parallel_ness: {0}", parallel_ness);
            */

            float hardRight, right, slightRight;
            float hardLeft, left, slightLeft;
            float noTurn;

            hardRight = FzAnd(left_ness, rightToLeft_ness);
            right = FzAnd(left_ness, parallel_ness);
            slightLeft = FzAnd(left_ness, leftToRight_ness);

            slightRight = FzAnd(right_ness, rightToLeft_ness);
            left = FzAnd(right_ness, parallel_ness);
            hardLeft = FzAnd(right_ness, leftToRight_ness);

            right = MaxF(right, FzAnd(oncourse_ness, rightToLeft_ness));
            noTurn = FzAnd(oncourse_ness, parallel_ness);
            left = MaxF(left,FzAnd(oncourse_ness, leftToRight_ness));

            /*
            // we need to "defuzzyify" but I am wondering if we can just
            // do a simple weighted average
            // hmmm... but this ignores the noTurn result entirely
            float turnResult =
              hardRight * hardRightValue +
              right * rightValue +
              slightRight * slightRightValue +
              slightLeft * slightLeftValue +
              left * leftValue +
              hardLeft * hardLeftValue;
            */

            // best defuzz advice seems to be centroid based
            // we can estimate it using sampling
            float defuzRez = 0.2f;
            float minResult = -9;
            float maxResult = 9;
            float sum = 0;
            float centroid = 0;
            float value;

            /*
            Deb.Log("hardLeft: {0}", hardLeft);
            Deb.Log("left: {0}", left);
            Deb.Log("slightLeft: {0}", slightLeft);
            Deb.Log("noTurn: {0}", noTurn);
            Deb.Log("slightRight: {0}", slightRight);
            Deb.Log("right: {0}", right);
            Deb.Log("hardRight: {0}", hardRight);
            */

            turnHardLeft.Clamp(hardLeft);
            turnLeft.Clamp(left);
            turnSlightLeft.Clamp(slightLeft);
            turnNone.Clamp(noTurn);
            turnSlightRight.Clamp(slightRight);
            turnRight.Clamp(right);
            turnHardRight.Clamp(hardRight);

            for (float x = minResult; x <= maxResult; x += defuzRez)
            {
                value = turnHardLeft.Eval(x);
                value = MaxF(value, turnLeft.Eval(x));
                value = MaxF(value, turnSlightLeft.Eval(x));
                value = MaxF(value, turnNone.Eval(x));
                value = MaxF(value, turnHardRight.Eval(x));
                value = MaxF(value, turnRight.Eval(x));
                value = MaxF(value, turnSlightRight.Eval(x));

                sum += value;
                centroid += (value * x);
            }

            return centroid / sum;
        }
    }
}
