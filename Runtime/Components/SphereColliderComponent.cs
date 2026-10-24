namespace Turian.Engine.Physics;

/// <summary>A sphere collider.</summary>
[ComponentContextMenu("Physics/Sphere Collider")]
[TypeId("e4ab1a39-10af-4304-87e5-66db1f088ff8")]
public class SphereColliderComponent : ColliderComponent
{
    /// <summary>The radius in the node's local space; the largest scale axis scales it.</summary>
    public float Radius
    {
        get;
        set => Change(ref field, value);
    } = 0.5f;

    /// <inheritdoc />
    public override ShapeDesc Describe(Vector3 scale, out Quaternion rotation)
    {
        rotation = Quaternion.Identity;
        return ShapeDesc.Sphere(MathF.Abs(Radius) * MaxAbs(scale));
    }
}
