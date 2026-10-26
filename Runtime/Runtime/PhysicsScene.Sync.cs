namespace Turian.Engine.Physics;

public sealed partial class PhysicsScene
{
    // Components compare by id; the scene tracks instances, and reference lookups avoid hashing the id.
    readonly List<PhysicsBody> records = [];
    List<PhysicsBody>? assignmentBuffer = [];
    readonly Dictionary<RigidbodyComponent, PhysicsBody> bodies = new(ReferenceEqualityComparer.Instance);
    readonly Dictionary<ColliderComponent, PhysicsBody> owners = new(ReferenceEqualityComparer.Instance);
    readonly Dictionary<int, PhysicsBody> byHandle = [];
    readonly List<BodyPose> awakePoses = [];
    int assignment;

    /// <summary>The number of backend bodies built for the scene's rigidbodies and static colliders.</summary>
    public int BodyCount => byHandle.Count;

    internal PhysicsPose? GetPose(RigidbodyComponent body) =>
        Built(body) is { } record ? World!.GetPose(record.Handle) : null;

    internal PhysicsVelocity? GetVelocity(RigidbodyComponent body) =>
        Built(body) is { } record ? World!.GetVelocity(record.Handle) : null;

    internal void SetVelocity(RigidbodyComponent body, in PhysicsVelocity velocity)
    {
        if (Built(body) is { } record)
        {
            World!.SetVelocity(record.Handle, velocity);
        }
    }

    internal Vector3? GetCenterOfMass(RigidbodyComponent body) =>
        Built(body) is { } record ? World!.GetCenterOfMass(record.Handle) : null;

    internal void Sleep(RigidbodyComponent body)
    {
        if (Built(body) is { } record)
        {
            World!.Sleep(record.Handle);
        }
    }

    internal void WakeUp(RigidbodyComponent body)
    {
        if (Built(body) is { } record)
        {
            World!.Wake(record.Handle);
        }
    }

    internal bool IsSleeping(RigidbodyComponent body) =>
        Built(body) is not { } record || !World!.IsAwake(record.Handle);

    internal BodyHandle HandleOf(RigidbodyComponent body) => Built(body)?.Handle ?? default;

    internal PhysicsBody? RecordOf(BodyHandle handle) => byHandle.GetValueOrDefault(handle.Id);

    PhysicsBody? Built(RigidbodyComponent body) =>
        World is not null && bodies.TryGetValue(body, out var record) && record.Handle.IsValid ? record : null;

    // Brings the backend up to date with the scene before a step.
    void Synchronize(float deltaTime)
    {
        if (assignmentDirty)
        {
            Assign();
        }

        foreach (var record in records)
        {
            if (record.Dirty)
            {
                Build(record);
            }
            else
            {
                PushTransform(record);
            }

            UpdateReporting(record);
            ApplyCommands(record, deltaTime);
        }

        SynchronizeJoints();
    }

    // Groups colliders under their nearest rigidbody; the others become one static body each. Records keep their
    // collider lists when the grouping is unchanged, so spawning one body does not rebuild the others.
    void Assign()
    {
        assignmentDirty = false;
        var stamp = ++assignment;
        var kept = assignmentBuffer ?? [];
        assignmentBuffer = null;
        try
        {
            GatherAssignments(stamp, kept);

            foreach (var record in kept)
            {
                Adopt(record);
            }

            RemoveStale(stamp);
            records.Clear();
            records.AddRange(kept);
            IndexOwners();
        }
        finally
        {
            kept.Clear();
            assignmentBuffer = kept;
        }
    }

    void GatherAssignments(int stamp, List<PhysicsBody> kept)
    {
        foreach (var body in registeredBodies)
        {
            if (!bodies.TryGetValue(body, out var record))
            {
                bodies[body] = record = new PhysicsBody(body.Node!, body);
            }

            Gather(record, stamp, kept);
        }

        foreach (var collider in registeredColliders)
        {
            if (!collider.HasShape)
            {
                continue;
            }

            if (NearestBody(collider) is { } body && bodies.TryGetValue(body, out var owner) && owner.Assignment == stamp)
            {
                owner.Gathered.Add(collider);
            }
            else
            {
                var record = StaticRecord(collider);
                Gather(record, stamp, kept);
                record.Gathered.Add(collider);
            }
        }
    }

    static void Gather(PhysicsBody record, int stamp, List<PhysicsBody> kept)
    {
        record.Assignment = stamp;
        record.Gathered.Clear();
        kept.Add(record);
    }

    // The rigidbody a collider was last grouped with is still the nearest while it stays enabled on the same node.
    RigidbodyComponent? NearestBody(ColliderComponent collider) =>
        owners.TryGetValue(collider, out var record) && record.Rigidbody is { IsActive: true } body &&
        ReferenceEquals(body.Node, collider.Node)
            ? body
            : FindRigidbody(collider.Node);

    PhysicsBody StaticRecord(ColliderComponent collider) =>
        owners.TryGetValue(collider, out var existing) && existing.Rigidbody is null
            ? existing
            : new PhysicsBody(collider.Node!, null);

    void RemoveStale(int stamp)
    {
        foreach (var stale in records)
        {
            if (stale.Assignment == stamp)
            {
                continue;
            }

            // Every registered rigidbody's record was gathered, so a stale one belongs to a removed rigidbody.
            Destroy(stale);
            if (stale.Rigidbody is { } body)
            {
                bodies.Remove(body);
            }
        }
    }

    void IndexOwners()
    {
        owners.Clear();
        foreach (var record in records)
        {
            foreach (var collider in record.Colliders)
            {
                owners[collider] = record;
            }
        }
    }

    static void Adopt(PhysicsBody record)
    {
        if (!SameColliders(record.Colliders, record.Gathered))
        {
            record.Colliders = [.. record.Gathered];
            record.Dirty = true;
        }

        record.Gathered.Clear();
    }

    static bool SameColliders(List<ColliderComponent> a, List<ColliderComponent> b)
    {
        if (a.Count != b.Count)
        {
            return false;
        }

        for (var i = 0; i < a.Count; i++)
        {
            if (!ReferenceEquals(a[i], b[i]))
            {
                return false;
            }
        }

        return true;
    }

    void Build(PhysicsBody record)
    {
        var world = World!;
        var velocity = record.Handle.IsValid
            ? world.GetVelocity(record.Handle)
            : record.Rigidbody?.DetachedVelocity ?? default;
        Destroy(record);
        record.Dirty = false;
        var origin = record.NodePose();
        var colliders = DescribeColliders(record, world, origin);
        if (record.Rigidbody is not { } rigidbody)
        {
            record.Motion = MotionType.Static;
            record.Handle = colliders.Length > 0 ? world.CreateStatic(new StaticDesc(origin, colliders)) : default;
        }
        else
        {
            record.Motion = rigidbody.IsKinematic ? MotionType.Kinematic : MotionType.Dynamic;
            record.Handle = world.CreateBody(DescribeBody(rigidbody, origin, velocity, colliders));
        }

        record.Version = record.Node.TransformVersion;
        record.Previous = record.Current = origin;
        record.Reporting = null;
        if (record.Handle.IsValid)
        {
            byHandle[record.Handle.Id] = record;
        }
    }

    // Creates the colliders' shapes and records the geometry they were built from.
    ColliderDesc[] DescribeColliders(PhysicsBody record, IPhysicsWorld world, in PhysicsPose origin)
    {
        var colliders = new ColliderDesc[record.Colliders.Count];
        for (var i = 0; i < colliders.Length; i++)
        {
            var collider = record.Colliders[i];
            var (local, scale) = record.Geometry(collider, origin);
            collider.WorldPose(out var shape);
            var handle = world.CreateShape(shape);
            record.Shapes.Add(handle);
            record.ColliderStates.Add((collider.Node?.TransformVersion ?? 0, local, scale,
                collider.Node?.PhysicsLayer ?? 0));
            colliders[i] = new ColliderDesc(handle, local, FilterOf(collider), MaterialOf(collider),
                collider.IsTrigger, collider.PhysicsId);
        }

        return colliders;
    }

    BodyDesc DescribeBody(RigidbodyComponent rigidbody, in PhysicsPose origin, in PhysicsVelocity velocity,
        ColliderDesc[] colliders) => new()
        {
            Motion = rigidbody.IsKinematic ? MotionType.Kinematic : MotionType.Dynamic,
            Pose = origin,
            Velocity = velocity,
            Colliders = colliders,
            Mass = new PhysicsMass(rigidbody.Mass, rigidbody.CenterOfMass, rigidbody.AutomaticCenterOfMass),
            LinearDamping = rigidbody.LinearDamping,
            AngularDamping = rigidbody.AngularDamping,
            GravityScale = rigidbody.UseGravity ? rigidbody.GravityScale : 0f,
            Continuous = rigidbody.CollisionDetectionMode == CollisionDetectionMode.Continuous,
            Locks = (AxisLocks)rigidbody.Constraints,
            SleepThreshold = Settings.SleepThreshold,
        };

    void Destroy(PhysicsBody record)
    {
        if (World is { } world)
        {
            if (record.Handle.IsValid)
            {
                byHandle.Remove(record.Handle.Id);
                world.Remove(record.Handle);
            }

            foreach (var shape in record.Shapes)
            {
                world.RemoveShape(shape);
            }
        }

        record.Handle = default;
        record.Shapes.Clear();
        record.ColliderStates.Clear();
    }

    // A script moved the node: teleport the body, or rebuild it when a collider moved within it or was rescaled.
    void PushTransform(PhysicsBody record)
    {
        if (!record.Handle.IsValid || record.InSync())
        {
            return;
        }

        var origin = record.NodePose();
        for (var i = 0; i < record.Colliders.Count; i++)
        {
            if (record.GeometryChanged(i, origin))
            {
                Build(record);
                return;
            }
        }

        var version = record.Node.TransformVersion;
        if (version != record.Version)
        {
            World!.SetPose(record.Handle, origin);
            record.Previous = record.Current = origin;
        }

        record.Version = version;
    }

    void ApplyCommands(PhysicsBody record, float deltaTime)
    {
        if (record.Rigidbody is not { } rigidbody || !record.Handle.IsValid)
        {
            return;
        }

        var commands = rigidbody.Commands;
        if (commands.TargetPosition is not null || commands.TargetRotation is not null)
        {
            ApplyMove(record, commands, deltaTime);
        }

        if (commands.HasForces && record.Motion == MotionType.Dynamic)
        {
            ApplyForces(record.Handle, commands, deltaTime);
        }

        commands.ClearForces();
    }

    // Kinematic bodies reach the target over the tick so they push; the others teleport.
    void ApplyMove(PhysicsBody record, RigidbodyCommands commands, float deltaTime)
    {
        var world = World!;
        var current = world.GetPose(record.Handle);
        var target = new PhysicsPose(commands.TargetPosition ?? current.Position,
            commands.TargetRotation ?? current.Orientation);
        if (record.Motion == MotionType.Kinematic)
        {
            world.SetKinematicTarget(record.Handle, target, deltaTime);
        }
        else
        {
            world.SetPose(record.Handle, target);
        }

        commands.TargetPosition = null;
        commands.TargetRotation = null;
    }

    void ApplyForces(BodyHandle handle, RigidbodyCommands commands, float deltaTime)
    {
        var world = World!;
        var center = world.GetCenterOfMass(handle);
        var linear = commands.Force * deltaTime + commands.Impulse;
        var angular = commands.Torque * deltaTime + commands.AngularImpulse;
        foreach (var (force, point, continuous) in commands.PointForces)
        {
            var impulse = continuous ? force * deltaTime : force;
            linear += impulse;
            angular += Vector3.Cross(point - center, impulse);
        }

        world.ApplyImpulse(handle, linear, center);
        world.ApplyAngularImpulse(handle, angular);
        var spin = commands.AngularAcceleration * deltaTime + commands.AngularVelocityChange;
        if (spin != Vector3.Zero)
        {
            var velocity = world.GetVelocity(handle);
            world.SetVelocity(handle, velocity with { Angular = velocity.Angular + spin });
        }
    }

    // Moves the nodes of awake bodies to their simulated poses. The backend lists only the awake bodies; the nodes are
    // still written in record order, so a body's node moves before those of bodies nested under it.
    void PullPoses()
    {
        World!.GetAwakePoses(awakePoses);
        foreach (var (handle, pose) in awakePoses)
        {
            if (byHandle.TryGetValue(handle.Id, out var record) && record.Motion != MotionType.Static)
            {
                record.Previous = record.Current;
                record.Current = pose;
                record.Moved = steps;
            }
        }

        foreach (var record in records)
        {
            if (record.Moved == steps)
            {
                record.WriteNodePose(record.Current);
            }
            else
            {
                record.Previous = record.Current;
            }
        }
    }

    void Interpolate(float alpha)
    {
        foreach (var record in records)
        {
            if (record.Rigidbody is not { Interpolation: RigidbodyInterpolation.Interpolate } ||
                !record.Handle.IsValid || record.Motion == MotionType.Static)
            {
                continue;
            }

            var pose = new PhysicsPose(Vector3.Lerp(record.Previous.Position, record.Current.Position, alpha),
                Quaternion.Slerp(record.Previous.Orientation, record.Current.Orientation, alpha));
            record.WriteNodePose(pose);
        }
    }

    /// <summary>The collider's layer, and the layers it collides with after its include and exclude masks.</summary>
    internal CollisionFilter FilterOf(ColliderComponent collider)
    {
        var layer = LayerOf(collider.Node);
        var accepts = (layerMatrix[layer] | collider.IncludeLayers.Value) & ~collider.ExcludeLayers.Value;
        return new CollisionFilter(1u << layer, accepts);
    }

    MaterialDesc MaterialOf(ColliderComponent collider) =>
        collider.Material?.ToDesc() ?? Settings.DefaultMaterial?.ToDesc() ?? MaterialDesc.Default;
}
