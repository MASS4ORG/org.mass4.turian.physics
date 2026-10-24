namespace Turian.Engine.Physics;

/// <summary>Locks this body to the connected one, or to the world, keeping their relative pose.</summary>
[ComponentContextMenu("Physics/Fixed Joint")]
[TypeId("dcb45716-7540-4279-aed7-072fca73fd66")]
public class FixedJointComponent : JointComponent
{
    /// <inheritdoc />
    public override JointDesc Describe(out Vector3 worldAxis)
    {
        worldAxis = Vector3.UnitX;
        return new JointDesc { Type = JointType.Fixed };
    }
}
