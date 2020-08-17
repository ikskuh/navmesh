const std = @import("std");
const sdl = @import("sdl2");
const navmesh = @import("navmesh.zig");
const wavefront_obj = @import("wavefront-obj");
const zlm = @import("zlm");

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

pub const Position = struct {
    const Self = @This();

    const movement_epsilon = 1e-9;

    mesh: *navmesh.NavMesh,
    polygon: usize,
    offset: zlm.Vec2,

    /// Implementing https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    const LineTestResult = struct {
        t: f32, // (a[1] - a[0])
        u: f32, // (b[1] - b[0])
    };
    fn intersect(a: [2]zlm.Vec2, b: [2]zlm.Vec2) ?LineTestResult {
        const epsilon = 1e-9;

        const x1 = a[0].x;
        const x2 = a[1].x;
        const x3 = b[0].x;
        const x4 = b[1].x;

        const y1 = a[0].y;
        const y2 = a[1].y;
        const y3 = b[0].y;
        const y4 = b[1].y;

        const det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        //Check if lines are near-parallel
        if (std.math.fabs(det) < epsilon)
            return null;

        const res = LineTestResult{
            .t = (((x1 - x3) * (y3 - y4)) - ((y1 - y3) * (x3 - x4))) / det,
            .u = (((x1 - x2) * (y1 - y3)) - ((y1 - y2) * (x1 - x3))) / det,
        };
        if (res.t < 0 or res.t > 1)
            return null;

        // if (res.u < 0 or res.u > 1)
        //     return null;

        return res;
    }

    fn move(self: *Self, delta: zlm.Vec2) void {
        if (delta.length2() < movement_epsilon) // ignore diminutive movements
            return;
        const poly = self.mesh.polygons[self.polygon];

        const ray = [2]zlm.Vec2{
            self.offset,
            self.offset.add(delta),
        };

        const Hit = struct {
            edge: navmesh.Edge,
            hit: LineTestResult,
        };

        var closest_hit: ?Hit = null;
        for (poly.vertices) |_, edge_index| {
            const edge = poly.getEdge(edge_index);
            const line = [2]zlm.Vec2{
                zlm.vec2(self.mesh.vertices[edge.v0].x, self.mesh.vertices[edge.v0].z),
                zlm.vec2(self.mesh.vertices[edge.v1].x, self.mesh.vertices[edge.v1].z),
            };

            if (intersect(ray, line)) |res| {
                const hit = Hit{
                    .hit = res,
                    .edge = edge,
                };
                if (closest_hit) |last| {
                    if (last.hit.t > res.t) {
                        closest_hit = hit;
                    }
                } else {
                    closest_hit = hit;
                }
            }
        }

        if (closest_hit) |res| {
            if (res.edge.adjacent) |adjacent_index| {

                // We collided with a wall with a adjacent polygon.
                // Move to that polygon and move the rest
                self.offset = self.offset.add(delta.scale(res.hit.t + 0.001));
                self.polygon = adjacent_index;

                return self.move(delta.sub(delta.scale(res.hit.t + 0.001)));
            } else {
                // We had a collision with the wall, we move up close to the wall, but do not stop on the line itself
                self.offset = self.offset.add(delta.scale(res.hit.t - 0.001));
                return;
            }
        } else {
            // Trivial case: We didn't cross any borders, just move inside the polygon
            self.offset = self.offset.add(delta);
        }
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

    var current_position = Position{
        .mesh = &mesh,
        .polygon = 0,
        .offset = blk: {
            var poly = mesh.polygons[0];

            var center = zlm.vec2(0, 0);
            for (poly.vertices) |v| {
                const p = mesh.vertices[v];
                center.x += p.x;
                center.y += p.z;
            }
            break :blk center.scale(1.0 / @intToFloat(f32, poly.vertices.len));
        },
    };

    mainLoop: while (true) {
        const camera_vel = 0.5;
        const position_vel = 0.25;
        while (sdl.pollEvent()) |ev| {
            switch (ev) {
                .key_down => |key_event| switch (key_event.keysym.sym) {
                    sdl.c.SDLK_UP => camera.pos_y -= camera_vel,
                    sdl.c.SDLK_DOWN => camera.pos_y += camera_vel,
                    sdl.c.SDLK_LEFT => camera.pos_x -= camera_vel,
                    sdl.c.SDLK_RIGHT => camera.pos_x += camera_vel,
                    sdl.c.SDLK_w => current_position.move(zlm.vec2(0.0, -position_vel)),
                    sdl.c.SDLK_s => current_position.move(zlm.vec2(0.0, position_vel)),
                    sdl.c.SDLK_a => current_position.move(zlm.vec2(-position_vel, 0.0)),
                    sdl.c.SDLK_d => current_position.move(zlm.vec2(position_vel, 0.0)),
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
        for (mesh.polygons) |poly, poly_index| {
            for (poly.vertices) |_, i| {
                const edge = poly.getEdge(i);

                const v0 = mesh.vertices[edge.v0];
                const v1 = mesh.vertices[edge.v1];

                const p0 = camera.project(v0.x, v0.z);
                const p1 = camera.project(v1.x, v1.z);

                try renderer.drawLine(p0.x, p0.y, p1.x, p1.y);
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

        try renderer.setColorRGB(0xFF, 0xFF, 0xFF);
        {
            const poly = mesh.polygons[current_position.polygon];
            try renderer.setColorRGB(0x00, 0xFF, 0xFF);
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
                    try renderer.setColorRGB(0x00, 0xFF, 0xFF);
                }
            }
        }

        try renderer.setColorRGB(0xFF, 0xFF, 0x00);
        {
            var p = camera.project(current_position.offset.x, current_position.offset.y);

            try renderer.drawLine(
                p.x - 5,
                p.y,
                p.x + 5,
                p.y,
            );
            try renderer.drawLine(
                p.x,
                p.y - 5,
                p.x,
                p.y + 5,
            );
        }

        renderer.present();
    }
}
