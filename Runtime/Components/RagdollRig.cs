namespace Turian.Engine.Physics;

/// <summary>Owns generated ragdoll proxies and transfers poses between them and the original skeleton nodes.</summary>
public sealed class RagdollRig : IDisposable
{
    readonly Transform[] bindPose;
    readonly Vector3[] scales;
    bool disposed;

    internal RagdollRig(Node root, List<RagdollPart> parts, bool startKinematic)
    {
        Root = root;
        Parts = parts.AsReadOnly();
        bindPose = [.. parts.Select(static part => part.Bone.Transform)];
        scales = [.. parts.Select(static part => part.Bone.GlobalTransform.Scale)];
        if (!startKinematic)
        {
            SetKinematic(false);
        }
    }

    /// <summary>The generated proxy group; the original bone hierarchy is outside this group.</summary>
    public Node Root { get; }

    /// <summary>Generated parts in parent-before-child order, regardless of the input bone order.</summary>
    public IReadOnlyList<RagdollPart> Parts { get; }

    /// <summary>Whether the rig follows animation and its generated physics components are disabled.</summary>
    public bool IsKinematic { get; private set; } = true;

    /// <summary>Switches between animation and simulation, transferring the current pose and clearing stale velocities.</summary>
    public void SetKinematic(bool value)
    {
        ObjectDisposedException.ThrowIf(disposed, this);
        if (value == IsKinematic)
        {
            return;
        }
        if (value)
        {
            SyncToBones();
        }
        else
        {
            SyncFromBones();
        }
        foreach (var part in Parts)
        {
            part.Body.IsKinematic = value;
            part.Body.LinearVelocity = Vector3.Zero;
            part.Body.AngularVelocity = Vector3.Zero;
            part.Body.IsActive = !value;
            part.Collider.IsActive = !value;
            if (part.Joint is { } joint)
            {
                joint.IsActive = !value;
            }
        }
        IsKinematic = value;
    }

    /// <summary>Copies animation poses to the proxies while kinematic; call after posing the skeleton.</summary>
    public void SyncFromBones()
    {
        ObjectDisposedException.ThrowIf(disposed, this);
        if (!IsKinematic)
        {
            return;
        }
        foreach (var part in Parts)
        {
            part.Body.Node!.GlobalTransform = part.Bone.GlobalTransform with { Scale = Vector3.One };
        }
    }

    /// <summary>Copies simulated proxy poses to skeleton bones while dynamic; call after physics and before rendering.</summary>
    public void SyncToBones()
    {
        ObjectDisposedException.ThrowIf(disposed, this);
        if (IsKinematic)
        {
            return;
        }
        for (var i = 0; i < Parts.Count; i++)
        {
            var part = Parts[i];
            part.Bone.GlobalTransform = part.Body.Node!.GlobalTransform with { Scale = scales[i] };
        }
    }

    /// <summary>Disables physics and restores the local bone poses captured at construction.</summary>
    public void RestoreBindPose()
    {
        ObjectDisposedException.ThrowIf(disposed, this);
        SetKinematic(true);
        for (var i = 0; i < Parts.Count; i++)
        {
            Parts[i].Bone.Transform = bindPose[i];
        }
        SyncFromBones();
    }

    /// <summary>Destroys only the generated physics group; original bones and their components are preserved.</summary>
    public void Dispose()
    {
        if (disposed)
        {
            return;
        }
        disposed = true;
        Root.DestroyImmediate();
    }
}
