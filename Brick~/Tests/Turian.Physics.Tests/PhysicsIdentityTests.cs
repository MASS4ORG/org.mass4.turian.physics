using System.Reflection;
using MASS4.Attributes;
using Microsoft.Extensions.Logging.Abstractions;
using Turian.Editor.Core;
using Turian.Engine.Physics;

namespace Turian.Tests;

/// <summary>Editable scripts preserve the physics types used by existing serialized scenes.</summary>
public sealed class PhysicsIdentityTests
{
    /// <summary>Every explicitly identified physics type appears under the same id in its script manifest.</summary>
    [Fact]
    public void ScriptsPreserveExistingTypeIds()
    {
        var directory = new DirectoryInfo(AppContext.BaseDirectory);
        while (directory is not null && !File.Exists(Path.Combine(directory.FullName, "package.json")))
        {
            directory = directory.Parent;
        }

        Assert.NotNull(directory);
        var entries = UserCodeTypeManifestGenerator.ScriptTypes(directory.FullName, NullLogger.Instance);
        var types = typeof(RigidbodyComponent).Assembly.GetExportedTypes()
            .Where(static type => type.GetCustomAttribute<TypeIdAttribute>(inherit: false) is not null).ToArray();
        Assert.NotEmpty(types);
        foreach (var type in types)
        {
            var entry = Assert.Single(entries, item => item.FullyQualifiedName == type.FullName);
            Assert.Equal(TypeRegistry.GetIdOrThrow(type), entry.TypeId);
        }
    }
}
