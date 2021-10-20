public static class Tweens
{
    public static float CubicEaseInOut(float t)
    {
        if (t < 0.5)
        {
            return 4 * t * t * t;
        }
        else
        {
            float x = -2 * t + 2;
            return 1.0f - 0.5f * (x * x * x);
        }
    }
}
