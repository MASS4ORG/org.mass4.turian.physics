namespace Turian.Engine.Physics;

/// <summary>Default mass, collider dimensions and swing limits for a bone-node ragdoll.</summary>
public sealed record RagdollSettings
{
    /// <summary>Total mass distributed among the selected bones according to their weights.</summary>
    public float TotalMass { get; init; } = 70f;

    /// <summary>Default capsule radius, reduced for segments shorter than twice this value.</summary>
    public float Radius { get; init; } = 0.1f;

    /// <summary>Length along the bone's local Y axis when it has no selected child or explicit end bone.</summary>
    public float TerminalLength { get; init; } = 0.25f;

    /// <summary>Default ball-socket swing cone half angle in degrees.</summary>
    public float SwingLimit { get; init; } = 45f;

    /// <summary>Whether the rig initially follows its bones with physics disabled.</summary>
    public bool StartKinematic { get; init; } = true;
}
