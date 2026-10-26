namespace Turian.Engine.Physics;

/// <summary>One selected skeleton bone, with optional collider and mass overrides.</summary>
public sealed record RagdollBone
{
    /// <summary>The posed skeleton node driven by this part of the ragdoll.</summary>
    public required Node Bone { get; init; }

    /// <summary>An optional descendant at the capsule's end; otherwise the first selected child is used.</summary>
    public Node? EndBone { get; init; }

    /// <summary>Positive relative mass weight; the builder normalizes all weights to the rig's total mass.</summary>
    public float MassWeight { get; init; } = 1f;

    /// <summary>Positive capsule radius override, or zero for the rig default.</summary>
    public float Radius { get; init; }
}

/// <summary>A generated body and collider bound to one skeleton bone, with its optional parent constraint.</summary>
/// <param name="Bone">The original bone node.</param>
/// <param name="Body">The generated rigidbody at the bone's world pose.</param>
/// <param name="Collider">The generated capsule oriented along the bone segment.</param>
/// <param name="Joint">The connection to the nearest selected ancestor, or null at a root bone.</param>
public sealed record RagdollPart(Node Bone, RigidbodyComponent Body, CapsuleColliderComponent Collider,
    BallSocketJointComponent? Joint);
