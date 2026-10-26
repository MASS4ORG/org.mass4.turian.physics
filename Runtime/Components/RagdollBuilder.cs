namespace Turian.Engine.Physics;

/// <summary>Creates independent physics proxies for a posed skeleton without changing its bone hierarchy.</summary>
public static class RagdollBuilder
{
    /// <summary>Infers capsule segments and parent joints from selected bone nodes, including skipped helper nodes.</summary>
    /// <param name="skeletonRoot">The model or skeleton hierarchy containing every selected bone.</param>
    /// <param name="bones">The bones to simulate, in any order.</param>
    /// <param name="settings">Mass and shape defaults; null uses the standard defaults.</param>
    public static RagdollRig FromSkeleton(Node skeletonRoot, IReadOnlyList<Node> bones,
        RagdollSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(bones);
        return Create(skeletonRoot, bones.Select(static bone => new RagdollBone { Bone = bone }).ToArray(), settings);
    }

    /// <summary>Creates a rig using explicit end bones, radii and relative masses where supplied.</summary>
    public static RagdollRig Create(Node skeletonRoot, IReadOnlyList<RagdollBone> bones,
        RagdollSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(skeletonRoot);
        ArgumentNullException.ThrowIfNull(bones);
        settings ??= new();
        ValidateSettings(settings);
        var selected = ValidateBones(skeletonRoot, bones);
        var plans = Plan(bones, selected, settings);
        var parent = skeletonRoot.Parent ?? skeletonRoot;
        if (!Matrix4x4.Invert(parent.GlobalTransform.Matrix4X4(), out _))
        {
            throw new ArgumentException("The physics parent must have an invertible transform.", nameof(skeletonRoot));
        }
        var root = Add(parent, "Ragdoll physics");
        root.GlobalTransform = Transform.Identity;
        try
        {
            var parts = BuildParts(root, plans, settings);
            return new RagdollRig(root, parts, settings.StartKinematic);
        }
        catch
        {
            root.DestroyImmediate();
            throw;
        }
    }

    static void ValidateSettings(RagdollSettings settings)
    {
        Positive(settings.TotalMass, nameof(settings.TotalMass));
        Positive(settings.Radius, nameof(settings.Radius));
        Positive(settings.TerminalLength, nameof(settings.TerminalLength));
        if (!float.IsFinite(settings.SwingLimit) || settings.SwingLimit < 0 || settings.SwingLimit > 180)
        {
            throw new ArgumentOutOfRangeException(nameof(settings), "Swing limits must be between zero and 180 degrees.");
        }
    }

    static HashSet<Node> ValidateBones(Node root, IReadOnlyList<RagdollBone> bones)
    {
        if (bones.Count == 0)
        {
            throw new ArgumentException("Select at least one bone.", nameof(bones));
        }
        var selected = new HashSet<Node>(ReferenceEqualityComparer.Instance);
        foreach (var definition in bones)
        {
            ArgumentNullException.ThrowIfNull(definition);
            var bone = definition.Bone;
            ValidateBone(root, definition);
            if (!selected.Add(bone))
            {
                throw new ArgumentException("Each bone may be selected only once.", nameof(bones));
            }
        }
        if (root.Parent is null && selected.Contains(root))
        {
            throw new ArgumentException("A selected root bone needs a parent for independent physics proxies.", nameof(root));
        }
        return selected;
    }

    static void ValidateBone(Node root, RagdollBone definition)
    {
        var bone = definition.Bone;
        ArgumentNullException.ThrowIfNull(bone);
        if (root.IsDestroyed || bone.IsDestroyed || !Under(bone, root))
        {
            throw new ArgumentException("Selected bones must be live descendants of the skeleton root.", nameof(definition));
        }
        ValidateEndBone(bone, definition.EndBone);
        Positive(definition.MassWeight, nameof(definition.MassWeight));
        if (!float.IsFinite(definition.Radius) || definition.Radius < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(definition), "A radius override must be nonnegative and finite.");
        }
    }

    static void ValidateEndBone(Node bone, Node? end)
    {
        if (end is null)
        {
            return;
        }
        if (end.IsDestroyed || !Under(end, bone) || ReferenceEquals(end, bone))
        {
            throw new ArgumentException("An end bone must be a live descendant of its bone.", nameof(end));
        }
    }

    static List<BonePlan> Plan(IReadOnlyList<RagdollBone> bones, HashSet<Node> selected, RagdollSettings settings)
    {
        var mass = bones.Sum(static bone => (double)bone.MassWeight);
        var plans = new List<BonePlan>(bones.Count);
        foreach (var definition in bones.OrderBy(static definition => Depth(definition.Bone)))
        {
            plans.Add(PlanBone(definition, bones, selected, settings, mass));
        }
        return plans;
    }

    static BonePlan PlanBone(RagdollBone definition, IReadOnlyList<RagdollBone> bones, HashSet<Node> selected,
        RagdollSettings settings, double mass)
    {
        var bone = definition.Bone;
        var pose = bone.GlobalTransform;
        var end = definition.EndBone ?? bones.FirstOrDefault(candidate =>
            ReferenceEquals(SelectedParent(candidate.Bone, selected), bone))?.Bone;
        var delta = end is null ? pose.Up * settings.TerminalLength : end.GlobalTransform.Position - pose.Position;
        var length = delta.Length();
        if (!float.IsFinite(length) || !Finite(pose.Position))
        {
            throw new ArgumentException("Bone poses must have finite positions and lengths.", nameof(bones));
        }
        if (length < 1e-5f)
        {
            delta = pose.Up * settings.TerminalLength;
            length = settings.TerminalLength;
        }
        var radius = definition.Radius > 0 ? definition.Radius : MathF.Min(settings.Radius, length * 0.45f);
        return new(bone, SelectedParent(bone, selected), pose, delta, length, radius,
            (float)(definition.MassWeight / mass * settings.TotalMass));
    }

    static List<RagdollPart> BuildParts(Node root, List<BonePlan> plans, RagdollSettings settings)
    {
        var bodies = new Dictionary<Node, RigidbodyComponent>(ReferenceEqualityComparer.Instance);
        var parts = new List<RagdollPart>(plans.Count);
        foreach (var plan in plans)
        {
            var node = Add(root, plan.Bone.Name + " body");
            node.GlobalTransform = plan.Pose with { Scale = Vector3.One };
            var body = (RigidbodyComponent)node.AddComponent(new RigidbodyComponent
            {
                Mass = plan.Mass, IsKinematic = true, IsActive = false,
                CollisionDetectionMode = CollisionDetectionMode.Continuous,
            });
            bodies.Add(plan.Bone, body);
            var shape = Add(node, plan.Bone.Name + " capsule");
            shape.PhysicsLayer = plan.Bone.PhysicsLayer;
            shape.GlobalTransform = new Transform
            {
                Position = plan.Pose.Position + plan.Delta * 0.5f,
                Orientation = Along(plan.Delta),
            };
            var collider = (CapsuleColliderComponent)shape.AddComponent(new CapsuleColliderComponent
            {
                Radius = plan.Radius, Height = MathF.Max(plan.Length, plan.Radius * 2), IsActive = false,
            });
            var joint = Connect(node, plan, bodies, settings.SwingLimit);
            parts.Add(new(plan.Bone, body, collider, joint));
        }
        return parts;
    }

    static BallSocketJointComponent? Connect(Node node, BonePlan plan,
        Dictionary<Node, RigidbodyComponent> bodies, float swing)
    {
        if (plan.Parent is null)
        {
            return null;
        }
        return (BallSocketJointComponent)node.AddComponent(new BallSocketJointComponent
        {
            ConnectedBody = bodies[plan.Parent], UseSwingLimit = true, SwingLimit = swing, IsActive = false,
            SwingAxis = Vector3.Transform(Vector3.Normalize(plan.Delta), Quaternion.Conjugate(plan.Pose.Orientation)),
        });
    }

    static Node Add(Node parent, string name)
    {
        var node = new Node { Name = name, Parent = parent };
        parent.Children.Add(node);
        node.Awake(parent);
        return node;
    }

    static Node? SelectedParent(Node bone, HashSet<Node> selected)
    {
        for (var parent = bone.Parent; parent is not null; parent = parent.Parent)
        {
            if (selected.Contains(parent))
            {
                return parent;
            }
        }
        return null;
    }

    static bool Under(Node bone, Node root)
    {
        for (var current = bone; current is not null; current = current.Parent)
        {
            if (ReferenceEquals(current, root))
            {
                return true;
            }
        }
        return false;
    }

    static int Depth(Node bone)
    {
        var depth = 0;
        for (var parent = bone.Parent; parent is not null; parent = parent.Parent)
        {
            depth++;
        }
        return depth;
    }

    static Quaternion Along(Vector3 delta)
    {
        var direction = Vector3.Normalize(delta);
        var dot = Vector3.Dot(Vector3.UnitY, direction);
        return dot < -0.99999f ? Quaternion.CreateFromAxisAngle(Vector3.UnitX, MathF.PI)
            : Quaternion.Normalize(new Quaternion(Vector3.Cross(Vector3.UnitY, direction), 1 + dot));
    }

    static bool Finite(Vector3 value) => float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

    static void Positive(float value, string name)
    {
        if (!float.IsFinite(value) || value <= 0)
        {
            throw new ArgumentOutOfRangeException(name, "The value must be positive and finite.");
        }
    }

    sealed record BonePlan(Node Bone, Node? Parent, Transform Pose, Vector3 Delta, float Length, float Radius, float Mass);
}
