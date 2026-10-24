namespace Turian.Engine.Physics;

/// <summary>Keeps the anchors together while this body turns freely, optionally within a cone around an axis.</summary>
[ComponentContextMenu("Physics/Ball Socket Joint")]
[TypeId("de0d78ed-f2ba-4f63-a258-600e253084f6")]
public class BallSocketJointComponent : JointComponent
{
    /// <summary>The axis the swing limit's cone opens around, in this node's local space.</summary>
    public Vector3 SwingAxis
    {
        get;
        set => Change(ref field, value);
    } = -Vector3.UnitY;

    /// <summary>Whether <see cref="SwingLimit"/> is enforced.</summary>
    public bool UseSwingLimit
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The cone's half angle in degrees.</summary>
    public float SwingLimit
    {
        get;
        set => Change(ref field, value);
    } = 45f;

    /// <inheritdoc />
    public override JointDesc Describe(out Vector3 worldAxis)
    {
        worldAxis = WorldAxis(SwingAxis);
        return new JointDesc
        {
            Type = JointType.BallSocket,
            Limit = new JointLimit(UseSwingLimit, 0f, Radians(MathF.Max(SwingLimit, 0f))),
        };
    }
}
