using OpenTK;

namespace SharpNav.Examples
{
    public static class OpenTKExts
    {
        public static System.Numerics.Vector3 toNumerics(this Vector3 v)
        {
            return new System.Numerics.Vector3(v.X, v.Y, v.Z);
        }
    }
}