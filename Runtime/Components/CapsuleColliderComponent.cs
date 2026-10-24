namespace Turian.Engine.Physics;

/// <summary>A capsule collider: a cylinder with hemispherical ends.</summary>
[ComponentContextMenu("Physics/Capsule Collider")]
[TypeId("ffc7fee2-e9cb-47fa-8a8e-f15458dea12e")]
public class CapsuleColliderComponent : ColliderComponent
{
    /// <summary>The radius of the ends and body.</summary>
    public float Radius
    {
        get;
        set => Change(ref field, value);
    } = 0.5f;

    /// <summary>The total height, including both ends.</summary>
    public float Height
    {
        get;
        set => Change(ref field, value);
    } = 2f;

    /// <summary>The local axis along the height: 0 for X, 1 for Y, 2 for Z.</summary>
    public int Direction
    {
        get;
        set => Change(ref field, Math.Clamp(value, 0, 2));
    } = 1;

    /// <inheritdoc />
    public override ShapeDesc Describe(Vector3 scale, out Quaternion rotation)
    {
        rotation = AxisRotation(Direction);
        var (along, across) = AxisScale(scale, Direction);
        var radius = MathF.Abs(Radius) * across;
        return ShapeDesc.Capsule(radius, MathF.Max(0f, MathF.Abs(Height) * along - 2f * radius));
    }
}
