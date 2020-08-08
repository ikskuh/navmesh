const Builder = @import("std").build.Builder;

pub fn build(b: *Builder) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard release options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall.
    const mode = b.standardReleaseOptions();

    const exe = b.addExecutable("navmesh", "src/main.zig");
    exe.addPackage(.{
        .name = "wavefront-obj",
        .path = "./lib/zig-gamedev-lib/src/wavefront-obj.zig",
    });
    exe.addPackage(.{
        .name = "sdl2",
        .path = "./lib/SDL.zig/src/lib.zig",
    });
    exe.addPackage(.{
        .name = "zlm",
        .path = "./lib/zlm/zlm.zig",
    });
    exe.linkSystemLibrary("sdl2");
    exe.linkLibC();
    exe.setTarget(target);
    exe.setBuildMode(mode);
    exe.install();

    const test_navmesh = b.addTest("src/navmesh.zig");
    test_navmesh.addPackage(.{
        .name = "wavefront-obj",
        .path = "./lib/zig-gamedev-lib/src/wavefront-obj.zig",
    });

    const run_cmd = exe.run();
    run_cmd.step.dependOn(b.getInstallStep());

    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const test_step = b.step("test", "Tests the navmesh implementation");
    test_step.dependOn(&test_navmesh.step);
}
