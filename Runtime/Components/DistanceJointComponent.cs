namespace Turian.Engine.Physics;

/// <summary>Keeps the anchors between a minimum and a maximum distance, as a rope or a rod does.</summary>
[ComponentContextMenu("Physics/Distance Joint")]
[TypeId("b5eeb870-cfcd-40d5-ac8a-bd23ed6df271")]
public class DistanceJointComponent : JointComponent
{
    /// <summary>The shortest distance between the anchors in meters.</summary>
    public float MinDistance
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The longest distance between the anchors in meters; a negative value uses their distance at creation.</summary>
    public float MaxDistance
    {
        get;
        set => Change(ref field, value);
    } = -1f;

    /// <inheritdoc />
    public override JointDesc Describe(out Vector3 worldAxis)
    {
        worldAxis = Vector3.UnitX;
        return new JointDesc
        {
            Type = JointType.Distance,
            Limit = new JointLimit(true, MathF.Max(MinDistance, 0f), MaxDistance),
        };
    }
}
