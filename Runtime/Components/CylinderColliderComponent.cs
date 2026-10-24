namespace Turian.Engine.Physics;

/// <summary>A cylinder collider.</summary>
[ComponentContextMenu("Physics/Cylinder Collider")]
[TypeId("52e9c756-387c-4f38-9a0f-46e746b938dd")]
public class CylinderColliderComponent : ColliderComponent
{
    /// <summary>The radius.</summary>
    public float Radius
    {
        get;
        set => Change(ref field, value);
    } = 0.5f;

    /// <summary>The full height.</summary>
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
        return ShapeDesc.Cylinder(MathF.Abs(Radius) * across, MathF.Abs(Height) * along);
    }
}
