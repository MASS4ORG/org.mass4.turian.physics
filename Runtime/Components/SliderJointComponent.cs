namespace Turian.Engine.Physics;

/// <summary>Lets this body slide along one axis without turning, as a drawer or a piston does.</summary>
[ComponentContextMenu("Physics/Slider Joint")]
[TypeId("32cedc73-b2d2-4749-be3f-114034708f43")]
public class SliderJointComponent : JointComponent
{
    /// <summary>The slide axis in this node's local space.</summary>
    public Vector3 Axis
    {
        get;
        set => Change(ref field, value);
    } = Vector3.UnitX;

    /// <summary>Whether <see cref="MinDistance"/> and <see cref="MaxDistance"/> are enforced.</summary>
    public bool UseLimits
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The lowest offset along the axis in meters.</summary>
    public float MinDistance
    {
        get;
        set => Change(ref field, value);
    } = -1f;

    /// <summary>The highest offset along the axis in meters.</summary>
    public float MaxDistance
    {
        get;
        set => Change(ref field, value);
    } = 1f;

    /// <summary>Whether the motor drives the slider.</summary>
    public bool UseMotor
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The motor's target speed in meters per second.</summary>
    public float MotorSpeed
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The largest force the motor applies, in newtons.</summary>
    public float MotorForce
    {
        get;
        set => Change(ref field, value);
    } = 10f;

    /// <inheritdoc />
    public override JointDesc Describe(out Vector3 worldAxis)
    {
        worldAxis = WorldAxis(Axis);
        return new JointDesc
        {
            Type = JointType.Slider,
            Limit = new JointLimit(UseLimits, MathF.Min(MinDistance, MaxDistance), MathF.Max(MinDistance, MaxDistance)),
            Motor = new JointMotor(UseMotor, MotorSpeed, MotorForce),
        };
    }
}
