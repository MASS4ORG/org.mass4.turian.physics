namespace Turian.Engine.Physics;

/// <summary>Lets this body turn around one axis, as a door or a wheel does, with an optional limit, motor or spring.</summary>
[ComponentContextMenu("Physics/Hinge Joint")]
[TypeId("d7ffc4c9-a5b1-4c23-bcd6-f40f80c46ae8")]
public class HingeJointComponent : JointComponent
{
    /// <summary>The hinge axis in this node's local space.</summary>
    public Vector3 Axis
    {
        get;
        set => Change(ref field, value);
    } = Vector3.UnitX;

    /// <summary>Whether <see cref="MinAngle"/> and <see cref="MaxAngle"/> are enforced.</summary>
    public bool UseLimits
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The lowest angle in degrees.</summary>
    public float MinAngle
    {
        get;
        set => Change(ref field, value);
    } = -45f;

    /// <summary>The highest angle in degrees.</summary>
    public float MaxAngle
    {
        get;
        set => Change(ref field, value);
    } = 45f;

    /// <summary>Whether the motor drives the hinge.</summary>
    public bool UseMotor
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The motor's target speed in degrees per second.</summary>
    public float MotorSpeed
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The largest torque the motor applies, in newton-meters.</summary>
    public float MotorForce
    {
        get;
        set => Change(ref field, value);
    } = 10f;

    /// <summary>Whether a spring pulls the hinge towards <see cref="SpringTargetAngle"/>.</summary>
    public bool UseSpring
    {
        get;
        set => Change(ref field, value);
    }

    /// <summary>The spring's oscillation frequency in hertz; higher is stiffer.</summary>
    public float SpringFrequency
    {
        get;
        set => Change(ref field, value);
    } = 2f;

    /// <summary>The spring's damping ratio: 1 settles without overshoot, 0 keeps swinging.</summary>
    public float SpringDamping
    {
        get;
        set => Change(ref field, value);
    } = 1f;

    /// <summary>The angle in degrees the spring pulls towards.</summary>
    public float SpringTargetAngle
    {
        get;
        set => Change(ref field, value);
    }

    /// <inheritdoc />
    public override JointDesc Describe(out Vector3 worldAxis)
    {
        worldAxis = WorldAxis(Axis);
        return new JointDesc
        {
            Type = JointType.Hinge,
            Limit = new JointLimit(UseLimits, Radians(MathF.Min(MinAngle, MaxAngle)), Radians(MathF.Max(MinAngle, MaxAngle))),
            Motor = new JointMotor(UseMotor, Radians(MotorSpeed), MotorForce),
            Spring = new JointSpring(UseSpring, SpringFrequency, SpringDamping, Radians(SpringTargetAngle)),
        };
    }
}
