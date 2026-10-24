namespace Turian.Engine.Physics;

/// <summary>A box collider.</summary>
[ComponentContextMenu("Physics/Box Collider")]
[TypeId("6084b6f2-581c-40f2-842b-13768dad5a13")]
public class BoxColliderComponent : ColliderComponent
{
    /// <summary>The box's full extents in the node's local space.</summary>
    public Vector3 Size
    {
        get;
        set => Change(ref field, value);
    } = Vector3.One;

    /// <inheritdoc />
    public override ShapeDesc Describe(Vector3 scale, out Quaternion rotation)
    {
        rotation = Quaternion.Identity;
        return ShapeDesc.Box(Vector3.Abs(Size * scale));
    }
}
