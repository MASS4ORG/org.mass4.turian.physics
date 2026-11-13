namespace Turian.Engine.Physics;

/// <summary>
/// Registers <see cref="PhysicsScene"/> as a session subsystem, which the lifetimes step as a fixed-tick system, and
/// backs core's <see cref="PhysicsQueries"/> with the running session's world.
/// </summary>
public sealed class PhysicsModule : IEngineServiceModule
{
    /// <inheritdoc />
    public void ConfigureServices(IServiceCollection services)
    {
        ArgumentNullException.ThrowIfNull(services);
        services.AddSubsystem(LifetimeScope.Session, static scope => new PhysicsScene(
            scope.Services?.GetService<IAppSettings>(), scope.Services?.GetService<LayerFilter>(),
            scope.Services?.GetService<SimulationClock>(), scope.Services?.GetService<IPhysicsWorldFactory>(),
            scope.Services?.GetService<IAssetLoader>()));
        services.AddSingleton<IPhysicsQueryBackend>(static sp =>
            new SessionPhysicsQueries(sp.GetRequiredService<ISessionLifetime>()));
    }

    /// <summary>Answers queries from the running session's world, and misses between sessions.</summary>
    sealed class SessionPhysicsQueries(ISessionLifetime lifetime) : IPhysicsQueryBackend
    {
        IPhysicsQueryBackend? Current => lifetime.Session?.Find(typeof(PhysicsScene)) as PhysicsScene;

        public IEnumerable<PhysicsHit> RaycastAll(Ray ray, float maxDistance) =>
            Current?.RaycastAll(ray, maxDistance) ?? [];

        public IEnumerable<Node> OverlapBox(Bounds bounds) => Current?.OverlapBox(bounds) ?? [];

        public IEnumerable<PhysicsHit> BoxCastAll(Bounds bounds, Vector3 direction, float maxDistance) =>
            Current?.BoxCastAll(bounds, direction, maxDistance) ?? [];
    }
}
