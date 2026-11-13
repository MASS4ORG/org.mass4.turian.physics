# 3D Physics

Rigid bodies, colliders, joints and raycasts for Turian, close to Unity's physics.

This brick holds the components. Add a backend, such as [`org.mass4.turian.physics.bepu`](https://github.com/MASS4ORG/org.mass4.turian.physics.bepu), to make things move.

## Install

```sh
turian-cli brick add org.mass4.turian.physics
turian-cli brick add org.mass4.turian.physics.bepu
```

Or use the Bricks panel.

## Quick start

1. Add a `RigidbodyComponent` and a `BoxColliderComponent` to a node. It falls.
2. Add a collider without a rigidbody to the floor. It stays put.
3. Push it from code:

```csharp
[InjectService] PhysicsScene physics = null!;

body.AddForce(Vector3.UnitY * 10, ForceMode.Impulse);

if (physics.Raycast(origin, Vector3.UnitZ, out var hit, 100))
    target = hit.Node; // what the ray touched
```

## What's inside

- **Bodies:** `RigidbodyComponent` (mass, gravity, kinematic, forces).
- **Colliders:** box, sphere, capsule, cylinder, mesh and heightfield.
- **Joints:** fixed, hinge, slider, ball socket, distance and spring.
- **Characters and cars:** `CharacterControllerComponent` and `WheelColliderComponent`.
- **Materials:** `PhysicsMaterial` sets friction and bounciness.
- **Ragdolls:** `RagdollBuilder.FromSkeleton` turns a skeleton into a ragdoll.
- **Events:** implement `ICollisionHandler` or `ITriggerHandler` to hear about contacts.
- **Queries:** raycasts, shape casts and overlaps on `PhysicsScene`.

## Settings

Create **Settings / Physics** under `Assets/Settings` to change gravity, layers and solver quality.

There is one physics world per game session. Restarting the session starts a fresh world.

## Limitations

- There is no layer-matrix editor yet.
- Heightfields are plain static meshes.
- Wheels only touch the ground through a ray.

## Development

Keep a Turian checkout at `../../Turian`, then run from it:

```sh
dotnet test --project ../Bricks/org.mass4.turian.physics/Brick~/Tests/Turian.Physics.Tests/Turian.Physics.Tests.csproj -p:NoLocalPackages=true
```
