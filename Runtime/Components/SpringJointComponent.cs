namespace Turian.Engine.Physics;

/// <summary>Pulls the anchors towards a rest distance with a spring, optionally within a distance range.</summary>
[ComponentContextMenu("Physics/Spring Joint")]
[TypeId("d3572447-0875-49ae-87b1-c164fc0a50d3")]
public class SpringJointComponent : JointComponent
{
    /// <summary>The spring's oscillation frequency in hertz; higher is stiffer.</summary>
    public float Frequency
    {
        get;
        set => Change(ref field, value);
    } = 2f;

    /// <summary>The spring's damping ratio: 1 settles without overshoot, 0 keeps bouncing.</summary>
    public float DampingRatio
    {
        get;
        set => Change(ref field, value);
    } = 0.3f;

    /// <summary>The distance the spring pulls towards in meters; a negative value uses the distance at creation.</summary>
    public float RestDistance
    {
        get;
        set => Change(ref field, value);
    } = -1f;

    /// <summary>The shortest distance between the anchors in meters.</summary>
    public float MinDistance
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The longest distance between the anchors in meters; zero or less leaves it unlimited.</summary>
    public float MaxDistance
    {
        get;
        set => Change(ref field, value);
    }

    /// <inheritdoc />
    public override JointDesc Describe(out Vector3 worldAxis)
    {
        worldAxis = Vector3.UnitX;
        return new JointDesc
        {
            Type = JointType.Spring,
            Spring = new JointSpring(true, Frequency, DampingRatio, RestDistance),
            Limit = new JointLimit(MinDistance > 0f || MaxDistance > 0f, MathF.Max(MinDistance, 0f),
                MaxDistance > 0f ? MaxDistance : float.MaxValue),
        };
    }
}
