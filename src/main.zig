const std = @import("std");
const sdl = @import("sdl2");
const navmesh = @import("navmesh.zig");
const wavefront_obj = @import("wavefront-obj");

fn importMesh(allocator: *std.mem.Allocator, file_name: []const u8) !navmesh.NavMesh {
    var builder = navmesh.Builder.init(allocator);
    defer builder.deinit();

    {
        const model = try wavefront_obj.load(allocator, "./data/basic.obj");
        defer model.deinit();

        for (model.faces.items) |face| {
            var verts: [4]navmesh.Vertex = undefined;

            for (face.vertices[0..face.count]) |index, offset| {
                verts[offset] = navmesh.Vertex{
                    .x = model.positions.items[index.position].x,
                    .y = model.positions.items[index.position].y,
                    .z = model.positions.items[index.position].z,
                };
            }

            try builder.insert(verts[0..face.count]);
        }
    }

    return try builder.createMesh(allocator);
}

const camera = struct {
    var pos_x: f32 = 0.0;
    var pos_y: f32 = 0.0;
    var zoom: f32 = 50.0;

    fn project(x: f32, y: f32) sdl.Point {
        return sdl.Point{
            .x = @floatToInt(i32, std.math.round(640 + zoom * (x - pos_x))),
            .y = @floatToInt(i32, std.math.round(360 + zoom * (y - pos_y))),
        };
    }
};

pub fn main() anyerror!void {
    var mesh = try importMesh(std.heap.c_allocator, "./data/basic.obj");
    defer mesh.deinit();

    try sdl.init(sdl.InitFlags.everything);
    defer sdl.quit();

    var window = try sdl.createWindow(
        "NavMesh Demo",
        .centered,
        .centered,
        1280,
        720,
        .{},
    );
    defer window.destroy();

    var renderer = try sdl.createRenderer(
        window,
        null,
        .{
            .accelerated = true,
        },
    );
    defer renderer.destroy();

    mainLoop: while (true) {
        const camera_vel = 0.5;
        while (sdl.pollEvent()) |ev| {
            switch (ev) {
                .key_down => |key_event| switch (key_event.keysym.sym) {
                    sdl.c.SDLK_UP => camera.pos_y -= camera_vel,
                    sdl.c.SDLK_DOWN => camera.pos_y += camera_vel,
                    sdl.c.SDLK_LEFT => camera.pos_x -= camera_vel,
                    sdl.c.SDLK_RIGHT => camera.pos_x += camera_vel,
                    sdl.c.SDLK_PLUS => camera.zoom += 1,
                    sdl.c.SDLK_MINUS => camera.zoom -= 1,
                    sdl.c.SDLK_ESCAPE => break :mainLoop,
                    else => {},
                },
                .quit => break :mainLoop,
                else => {},
            }
        }

        try renderer.setColorRGB(0x00, 0x00, 0x40);
        try renderer.clear();

        try renderer.setColorRGB(0xFF, 0xFF, 0xFF);
        for (mesh.polygons) |poly| {
            for (poly.vertices) |_, i| {
                const edge = poly.getEdge(i);

                const v0 = mesh.vertices[edge.v0];
                const v1 = mesh.vertices[edge.v1];

                const p0 = camera.project(v0.x, v0.z);
                const p1 = camera.project(v1.x, v1.z);

                try renderer.drawLine(p0.x, p0.y, p1.x, p1.y);

                if (edge.adjacent) |_| {
                    var mid_x = @divTrunc(p0.x + p1.x, 2);
                    var mid_y = @divTrunc(p0.y + p1.y, 2);

                    var dx = p1.x - p0.x;
                    var dy = p1.y - p0.y;

                    var scale = 10 / std.math.sqrt(@intToFloat(f32, dx * dx + dy * dy));

                    dx = @floatToInt(i32, std.math.round(@intToFloat(f32, dx) * scale));
                    dy = @floatToInt(i32, std.math.round(@intToFloat(f32, dy) * scale));

                    try renderer.setColorRGB(0x00, 0x00, 0xFF);
                    try renderer.drawLine(mid_x, mid_y, mid_x - dy, mid_y + dx);
                    try renderer.setColorRGB(0xFF, 0xFF, 0xFF);
                }
            }
        }

        try renderer.setColorRGB(0xFF, 0x00, 0x00);
        for (mesh.vertices) |vert| {
            try renderer.fillRect(sdl.Rectangle{
                .x = camera.project(vert.x, vert.z).x - 2,
                .y = camera.project(vert.y, vert.z).y - 2,
                .width = 5,
                .height = 5,
            });
        }

        renderer.present();
    }
}
