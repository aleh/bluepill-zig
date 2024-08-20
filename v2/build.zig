const std = @import("std");

pub fn build(b: *std.Build) !void {
    const exe = b.addExecutable(.{
        .name = "main",
        .root_source_file = b.path("main.zig"),
        // We support only STM32F103xx.
        .target = b.resolveTargetQuery(try std.Build.parseTargetQuery(.{
            .arch_os_abi = "arm-freestanding-none",
            .cpu_features = "cortex_m23",
        })),
        .optimize = .ReleaseSmall,
    });
    exe.linker_script = b.path("bluepill.ld");
    exe.root_module.addImport("z41", b.createModule(.{ .root_source_file = b.path("../lib/z41.zig") }));

    const objcopy = b.addObjCopy(exe.getEmittedBin(), .{ .basename = "main", .format = .hex });
    const install_hex = b.addInstallBinFile(objcopy.getOutput(), "main.hex");
    b.getInstallStep().dependOn(&install_hex.step);

    const flash_cmd = b.addSystemCommand(&.{ "st-flash", "--reset", "--format", "ihex", "write" });
    // Could be better to depend on the installed hex?
    flash_cmd.addFileArg(objcopy.getOutput());

    const flash_step = b.step("flash", "Flash the binary");
    flash_step.dependOn(&flash_cmd.step);
}
