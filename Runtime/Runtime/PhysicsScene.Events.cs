namespace Turian.Engine.Physics;

public sealed partial class PhysicsScene
{
    // Enough for debug drawing a busy scene; more would only slow the editor.
    const int MaxRecordedContacts = 4096;

    readonly List<ContactPoint> lastContacts = [];
    readonly List<ICollisionHandler> collisionHandlers = [];
    readonly List<ITriggerHandler> triggerHandlers = [];
    ContactEvent[] contactBuffer = [];
    TriggerEvent[] triggerBuffer = [];
    int contactCount;
    bool lastContactsStale;

    /// <summary>The touching contact points of the last step, seen from the first collider of each pair.</summary>
    /// <remarks>Built when first read after a step, so steps nobody draws pay nothing for it.</remarks>
    public IReadOnlyList<ContactPoint> LastContacts
    {
        get
        {
            if (lastContactsStale)
            {
                lastContactsStale = false;
                RecordContacts();
            }

            return lastContacts;
        }
    }

    // Callbacks run after the step on the game loop thread; they may change the scene, so events are copied first,
    // into buffers kept across steps because a busy scene reports thousands of pairs every step.
    void DispatchEvents()
    {
        var world = World!;
        var contacts = Copy(world.Contacts, ref contactBuffer);
        var triggers = Copy(world.Triggers, ref triggerBuffer);
        contactCount = contacts.Length;
        lastContactsStale = true;
        foreach (ref readonly var contact in contacts)
        {
            if ((Listens(contact.BodyA, contact.ColliderA) || Listens(contact.BodyB, contact.ColliderB)) &&
                Resolve(contact.ColliderA, contact.ColliderB) is var (a, b))
            {
                SendCollision(a, b, contact, 1f);
                SendCollision(b, a, contact, -1f);
            }
        }

        foreach (ref readonly var trigger in triggers)
        {
            if (Resolve(trigger.ColliderA, trigger.ColliderB) is var (a, b))
            {
                SendTrigger(a, b, trigger.State);
                SendTrigger(b, a, trigger.State);
            }
        }
    }

    static ReadOnlySpan<T> Copy<T>(ReadOnlySpan<T> events, ref T[] buffer)
    {
        if (buffer.Length < events.Length)
        {
            buffer = new T[Math.Max(events.Length, buffer.Length * 2)];
        }

        events.CopyTo(buffer);
        return buffer.AsSpan(0, events.Length);
    }

    // Pairs whose collider was disabled or destroyed are dropped, as Unity drops their exits.
    (ColliderComponent, ColliderComponent)? Resolve(ulong a, ulong b) =>
        collidersById.TryGetValue(a, out var first) && collidersById.TryGetValue(b, out var second)
            ? (first, second)
            : null;

    void RecordContacts()
    {
        lastContacts.Clear();
        foreach (ref readonly var contact in contactBuffer.AsSpan(0, contactCount))
        {
            if (lastContacts.Count >= MaxRecordedContacts)
            {
                return;
            }

            if (Resolve(contact.ColliderA, contact.ColliderB) is var (a, b))
            {
                for (var i = 0; i < contact.PointCount && lastContacts.Count < MaxRecordedContacts; i++)
                {
                    var point = contact.Points[i];
                    lastContacts.Add(new ContactPoint(point.Point, point.Normal, point.Separation, a, b));
                }
            }
        }
    }

    /// <summary>
    /// Reports the contacts of every body in <see cref="LastContacts"/>, not only those a collision handler receives;
    /// for debugging, as it costs event work for every touching pair.
    /// </summary>
    public bool ReportAllContacts { get; set; }

    // The backend only reports the contacts of bodies a handler may receive, so the many bodies without one cost no
    // event work; the check runs before each step, so a handler added by a callback receives the next step's.
    void UpdateReporting(PhysicsBody record)
    {
        var report = ReportAllContacts || Listens(record);
        if (record.Handle.IsValid && record.Reporting != report)
        {
            World!.SetContactReporting(record.Handle, report);
            record.Reporting = report;
        }
    }

    // Whether a collision handler may receive the body's contacts, checked once per body and step.
    bool Listens(PhysicsBody record)
    {
        if (record.ListenersChecked != steps)
        {
            record.ListenersChecked = steps;
            var bodyNode = record.Rigidbody?.Node;
            var listens = HasHandler(bodyNode);
            foreach (var collider in record.Colliders)
            {
                listens = listens || (collider.Node != bodyNode && HasHandler(collider.Node));
            }

            record.Listens = listens;
        }

        return record.Listens;
    }

    bool Listens(BodyHandle handle, ulong colliderId)
    {
        if (byHandle.TryGetValue(handle.Id, out var record))
        {
            return Listens(record);
        }

        // The body was removed before the step, as when its rigidbody was disabled; its collider may still exit.
        return collidersById.TryGetValue(colliderId, out var collider) &&
               (HasHandler(collider.Node) || HasHandler(collider.AttachedRigidbody?.Node));
    }

    static bool HasHandler(Node? node)
    {
        if (node is not { IsActiveInHierarchy: true })
        {
            return false;
        }

        var components = node.Components;
        for (var i = 0; i < components.Count; i++)
        {
            if (components[i] is ICollisionHandler && components[i].IsActive)
            {
                return true;
            }
        }

        return false;
    }

    // The collision is only described when a handler receives it; most colliders have none.
    void SendCollision(ColliderComponent self, ColliderComponent other, in ContactEvent contact, float sign)
    {
        Handlers(self, collisionHandlers);
        if (collisionHandlers.Count == 0)
        {
            return;
        }

        var points = new ContactPoint[contact.PointCount];
        for (var i = 0; i < points.Length; i++)
        {
            var point = contact.Points[i];
            points[i] = new ContactPoint(point.Point, point.Normal * sign, point.Separation, self, other);
        }

        var collision = new Collision(self, other, points, contact.Impulse * sign, contact.RelativeVelocity * sign);
        var state = contact.State;
        foreach (var handler in collisionHandlers)
        {
            Invoke(handler, state switch
            {
                PhysicsEventState.Enter => static (h, c) => h.OnCollisionEnter(c),
                PhysicsEventState.Stay => static (h, c) => h.OnCollisionStay(c),
                _ => static (h, c) => h.OnCollisionExit(c),
            }, collision);
        }
    }

    void SendTrigger(ColliderComponent self, ColliderComponent other, PhysicsEventState state)
    {
        Handlers(self, triggerHandlers);
        foreach (var handler in triggerHandlers)
        {
            Invoke(handler, state switch
            {
                PhysicsEventState.Enter => static (h, o) => h.OnTriggerEnter(o),
                PhysicsEventState.Stay => static (h, o) => h.OnTriggerStay(o),
                _ => static (h, o) => h.OnTriggerExit(o),
            }, other);
        }
    }

    // The collider's node and, when different, the node of the rigidbody it was simulated with receive the callbacks,
    // as in Unity. The list is reused; the handlers it lists run before it is filled again.
    void Handlers<T>(ColliderComponent collider, List<T> handlers) where T : class
    {
        handlers.Clear();
        Collect(collider.Node, handlers);
        var body = owners.TryGetValue(collider, out var record) ? record.Rigidbody : collider.AttachedRigidbody;
        if (body?.Node is { } bodyNode && bodyNode != collider.Node)
        {
            Collect(bodyNode, handlers);
        }
    }

    static void Collect<T>(Node? node, List<T> handlers) where T : class
    {
        if (node is not { IsActiveInHierarchy: true })
        {
            return;
        }

        // Indexing avoids the boxed enumerator Collection<T> allocates per foreach.
        var components = node.Components;
        for (var i = 0; i < components.Count; i++)
        {
            if (components[i] is T handler && components[i].IsActive)
            {
                handlers.Add(handler);
            }
        }
    }

    static void Invoke<THandler, TArgument>(THandler handler, Action<THandler, TArgument> callback, TArgument argument)
    {
        try
        {
            callback(handler, argument);
        }
        catch (Exception ex)
        {
            Log.Logger.LogError(ex, "{Handler} threw in a physics callback", handler?.GetType().Name);
        }
    }
}
