using System.Buffers;

namespace Turian.Engine.Physics;

public sealed partial class PhysicsScene
{
    /// <summary>Writes the nearest distinct ray hits in distance order using pooled scratch storage.</summary>
    public int RaycastNonAlloc(Vector3 origin, Vector3 direction, Span<RaycastHit> results,
        float maxDistance = float.PositiveInfinity, LayerMask? layerMask = null,
        QueryTriggerInteraction queryTriggerInteraction = default)
    {
        if (World is null || results.IsEmpty || !TryDirection(direction, maxDistance, out var unit, out var length))
        {
            return 0;
        }

        var buffer = ArrayPool<RaycastHit>.Shared.Rent(results.Length);
        try
        {
            var collector = new BoundedRayCollector(buffer, results.Length, length, collidersById);
            World.CastRay(new RaycastInput(origin, origin + unit * length, FilterOf(layerMask),
                HitsTriggers(queryTriggerInteraction)), ref collector);
            buffer.AsSpan(0, collector.Count).CopyTo(results);
            return collector.Count;
        }
        finally
        {
            ArrayPool<RaycastHit>.Shared.Return(buffer, clearArray: true);
        }
    }

    /// <summary>Keeps nearest distinct hits in bounded scratch because backends cannot retain a caller's span.</summary>
    internal struct BoundedRayCollector(RaycastHit[] buffer, int capacity, float length,
        Dictionary<ulong, ColliderComponent> colliders) : IHitCollector
    {
        public int Count { get; private set; }

        public readonly bool EarlyOutOnFirstHit => false;

        public readonly float MaxFraction => Count == capacity ? buffer[Count - 1].Distance / length : 1f;

        public bool AddHit(in PhysicsQueryHit hit)
        {
            if (hit.Fraction <= 0f || hit.Fraction > MaxFraction ||
                !colliders.TryGetValue(hit.ColliderData, out var collider))
            {
                return false;
            }

            var candidate = new RaycastHit(hit.Point, hit.Normal, hit.Fraction * length, collider);
            for (var i = 0; i < Count; i++)
            {
                if (ReferenceEquals(buffer[i].Collider, collider))
                {
                    if (buffer[i].Distance <= candidate.Distance)
                    {
                        return false;
                    }

                    buffer.AsSpan(i + 1, Count - i - 1).CopyTo(buffer.AsSpan(i));
                    Count--;
                    break;
                }
            }

            Insert(candidate);
            return true;
        }

        void Insert(in RaycastHit hit)
        {
            var index = Math.Min(Count, capacity - 1);
            while (index > 0 && buffer[index - 1].Distance > hit.Distance)
            {
                buffer[index] = buffer[index - 1];
                index--;
            }

            buffer[index] = hit;
            Count = Math.Min(Count + 1, capacity);
        }
    }
}
