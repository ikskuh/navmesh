const std = @import("std");

const log = std.log.scoped(.navmesh);

/// Polygonal mesh structure that allows searching paths and navigating on the mesh surface.
pub const NavMesh = struct {
    const Self = @This();

    /// A position on the mesh. Using this instead of vector coordinates
    /// allows quicker navigation of the mesh structure.
    pub const Position = struct {
        mesh: *Self,
        polygon: usize,
        x: f32,
        y: f32,
        z: f32,
    };

    /// Backing buffer for all items in the NavMesh
    arena: std.heap.ArenaAllocator,

    /// List of all polygons in the mesh.
    polygons: []Polygon,

    /// Shared vertices for reduced memory footprint.
    /// This is used for deduplicating the corners of the mesh.
    vertices: []Vertex,

    /// Deserializes a previously serialized NavMesh
    pub fn deserialize(allocator: *std.mem.Allocator, reader: anytype) !Self {
        const vertices_len = try reader.readIntLittle(u64);
        const polygons_len = try reader.readIntLittle(u64);

        var arena = std.heap.ArenaAllocator.init(allocator);
        errdefer arena.deinit();

        const vertices = try arena.allocator.alloc(Vertex, vertices_len);

        for (vertices) |*vert| {
            // TODO: This assumes little endian f32
            try reader.readNoEof(std.mem.asBytes(&vert.x));
            try reader.readNoEof(std.mem.asBytes(&vert.y));
            try reader.readNoEof(std.mem.asBytes(&vert.z));
        }

        const polygons = try arena.allocator.alloc(Polygon, polygons_len);
        for (polygons) |*poly| {
            poly.* = Polygon{
                .vertices = undefined,
                .adjacent_polygons = undefined,
            };

            for (poly.vertices) |*vert_index| {
                vert_index.* = try reader.readIntLittle(u64);
                if (vert_index.* >= vertices.len)
                    return error.InvalidData;
            }

            for (poly.adjacent_polygons) |*maybe_poly| {
                const index = try reader.readIntLittle(u64);
                maybe_poly.* = if (index != std.math.maxInt(u64))
                    if (index < polygons.len)
                        index
                    else
                        return error.InvalidData
                else
                    null;
            }
        }

        return NavMesh{
            .arena = arena,
            .polygons = polygons,
            .vertices = vertices,
        };
    }

    /// Releases all resources in the NavMesh.
    pub fn deinit(self: *Self) void {
        self.arena.deinit();
        self.* = undefined;
    }

    /// Serializes the NavMesh into a data stream which can later be deserialized again.
    pub fn serialize(self: Self, writer: anytype) !void {
        try writer.writeIntLittle(u64, self.vertices.len);
        try writer.writeIntLittle(u64, self.polygons.len);

        for (self.vertices) |vert| {
            // TODO: This assumes little endian f32
            try writer.writeAll(std.mem.asBytes(&vert.x));
            try writer.writeAll(std.mem.asBytes(&vert.y));
            try writer.writeAll(std.mem.asBytes(&vert.z));
        }

        for (self.polygons) |poly| {
            std.debug.assert(poly.vertices.len == poly.adjacent_polygons.len);
            for (poly.vertices) |vert_index| {
                try writer.writeIntLittle(u64, vert_index);
            }
            for (poly.adjacent_polygons) |maybe_poly| {
                try writer.writeIntLittle(u64, maybe_poly orelse std.math.maxInt(u64));
            }
        }
    }
};

/// An edge of a polygon. Each edge has two vertices and an optional adjacent
/// polygon.
pub const Edge = struct {
    v0: usize,
    v1: usize,
    adjacent: ?usize,
};

/// A node in the `NavMesh` tree structure
pub const Polygon = struct {
    const Self = @This();

    /// The vertex coordinates of the polygon. Each entry is an index into `NavMesh.vertices`.
    vertices: [3]usize,

    /// Adjacent polygons that are connected at vertex `[i]` and `[(i+1)%N]`. Each entry is an index into `NavMesh.polygons`.
    adjacent_polygons: [3]?usize,

    pub fn getEdge(self: Self, index: usize) Edge {
        return Edge{
            .v0 = self.vertices[index],
            .v1 = self.vertices[(index + 1) % self.vertices.len],
            .adjacent = self.adjacent_polygons[index],
        };
    }
};

/// A corner of a NavMesh
pub const Vertex = struct {
    const Self = @This();

    x: f32,
    y: f32,
    z: f32,

    /// Compares two vertices for equality with a positional epsilon.
    pub fn eql(a: Self, b: Self, epsilon: f32) bool {
        return std.math.approxEq(f32, a.x, b.x, epsilon) and std.math.approxEq(f32, a.y, b.y, epsilon) and std.math.approxEq(f32, a.z, b.z, epsilon);
    }
};

// Use vertices as "just positions"
fn cross(a: Vertex, b: Vertex) Vertex {
    return Vertex{
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x,
    };
}

fn dot(a: Vertex, b: Vertex) f32 {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

fn length(v: Vertex) f32 {
    return std.math.sqrt(dot(v, v));
}

fn scale(v: Vertex, f: f32) Vertex {
    return Vertex{
        .x = v.x * f,
        .y = v.x * f,
        .z = v.x * f,
    };
}

fn sub(a: Vertex, b: Vertex) Vertex {
    return Vertex{
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z,
    };
}

/// Helper structure that allows building a optimized NavMesh.
pub const Builder = struct {
    const Self = @This();

    arena: std.heap.ArenaAllocator,
    vertices: std.ArrayList(Vertex),
    polygons: std.ArrayList(Polygon),

    // stores one list per vertex with the indices of all used polygons.
    // This allows faster detection of adjacent polygons
    // TODO: Use ArrayListUnmanaged
    deduplication_list: std.ArrayList(std.ArrayList(usize)),

    /// Epsilon that is used to deduplicate vertices.
    vertex_epsilon: f32 = 1e-4, // using fairly "large" epsilon here considering this is meant for games which usually use meters as a unit

    /// Creates a new Builder.
    pub fn init(allocator: *std.mem.Allocator) Self {
        return Self{
            .arena = std.heap.ArenaAllocator.init(allocator),
            .vertices = std.ArrayList(Vertex).init(allocator),
            .polygons = std.ArrayList(Polygon).init(allocator),
            .deduplication_list = std.ArrayList(std.ArrayList(usize)).init(allocator),
        };
    }

    /// Creates a new NavMesh from the current state of the Builder.
    /// All data will be duplicated into a arena backed by the given allocator.
    /// This allows creating a really small memory footprint and improved cache coherency
    /// than using the slices allocated while building the mesh.
    pub fn createMesh(self: *Self, allocator: *std.mem.Allocator) !NavMesh {
        const vertices = self.vertices.items;
        const polygons = self.polygons.items;

        var mesh = NavMesh{
            .arena = std.heap.ArenaAllocator.init(allocator),
            .polygons = undefined,
            .vertices = undefined,
        };
        errdefer mesh.arena.deinit();

        mesh.polygons = try mesh.arena.allocator.dupe(Polygon, polygons);
        mesh.vertices = try mesh.arena.allocator.dupe(Vertex, vertices);

        return mesh;
    }

    /// Inserts the given polygon into the mesh.
    /// Note that this is a fairly slow process and might not be efficient, if you don't need a dynamic NavMesh,
    /// consider pregenerating it and store a serialized version.
    /// TODO: Create a data structure / library that allows "viewing" a slice with certain properties (x,y,z) in that case
    pub fn insert(self: *Self, polygon: []const Vertex) !void {
        std.debug.assert(polygon.len >= 3); // calling it with <3 elements is an API violation

        // we only accept flat/planar polygons, check that and return an error if not
        {
            var p0 = polygon[0];
            var p1 = polygon[1];
            var p2 = polygon[2];

            const t0 = sub(p1, p0);
            const t1 = sub(p2, p0);

            const n_unscaled = cross(t0, t1);
            const n = scale(n_unscaled, 1.0 / length(n_unscaled));

            var index: usize = 3;
            while (index < polygon.len) : (index += 1) {
                const p = polygon[index];

                const v = sub(p, p0);
                const dist = dot(v, n);

                if (std.math.fabs(dist) > self.vertex_epsilon) {
                    log.warn("Polygon is not planar. Vertex {} has distance of {} to plane!\n", .{
                        index,
                        dist,
                    });
                    return error.PolygonNotPlanar;
                }
            }
        }

        // Triangulating a convex polygon is easy: just fan out the polygon
        // see also: https://en.wikipedia.org/wiki/Fan_triangulation
        {
            var index: usize = 2;
            while (index < polygon.len) : (index += 1) {
                const triangle = [3]Vertex{
                    polygon[0],
                    polygon[index - 1],
                    polygon[index - 0],
                };
                try self.insertTriangle(triangle);
            }
        }
    }

    /// Inserts the given triangle into the mesh.
    /// Note that this is a fairly slow process and might not be efficient, if you don't need a dynamic NavMesh,
    /// consider pregenerating it and store a serialized version.
    pub fn insertTriangle(self: *Self, triangle: [3]Vertex) !void {
        const polygon_index = self.polygons.items.len;
        const vertex_count = self.vertices.items.len;

        // Just reset the lists back to the original length
        errdefer self.polygons.shrink(polygon_index);
        errdefer self.vertices.shrink(vertex_count);
        errdefer {
            for (self.deduplication_list.items[vertex_count..]) |*list| {
                list.deinit();
            }
            self.deduplication_list.shrink(vertex_count);
        }

        var unique_vertices: usize = 0;
        var vertices: [3]usize = undefined;
        for (vertices) |*vert, index| {
            const insert_vertex = triangle[index];
            vert.* = for (self.vertices.items) |v, i| {
                if (insert_vertex.eql(v, self.vertex_epsilon))
                    break i;
            } else blk: {
                var last_index = self.vertices.items.len;
                try self.vertices.append(insert_vertex);
                try self.deduplication_list.append(std.ArrayList(usize).init(self.deduplication_list.allocator));

                unique_vertices += 1;

                break :blk last_index;
            };

            // Check all previously inserted vertices if they match our currently inserted vertex
            // for duplications. This will find degenerated polygons
            for (vertices[0..index]) |v| {
                if (v == vert.*) {
                    return error.DuplicateVertex;
                }
            }
        }

        if (unique_vertices == 0) {
            // This checks if all our vertices share a common polygon.
            // If this is the case, we have found a duplicate and can drop it
            // without any problems
            const v0 = vertices[0];
            outer: for (self.deduplication_list.items[v0].items) |p0| {
                for (vertices[1..]) |v1| {
                    const contains = for (self.deduplication_list.items[v1].items) |p1| {
                        if (p1 == p0)
                            break true;
                    } else false;
                    if (!contains)
                        continue :outer;
                }
                // we found a polygon with which is included in all vertices:
                // this means this polygon or a "superset" already exists,
                // we can ignore this one
                return;
            }
        }

        var result = Polygon{
            .vertices = vertices,
            .adjacent_polygons = [_]?usize{ null, null, null },
        };

        for (result.adjacent_polygons) |*adj_poly_index, index| {
            const v0 = result.vertices[index];
            const v1 = result.vertices[(index + 1) % result.vertices.len];

            // Search in both adjacency lists for a match.
            // We are guaranteed that the first match will be the only match possible
            // due to all deduplications.
            adj_poly_index.* = for (self.deduplication_list.items[v0].items) |adjancent_to_v0| {
                const temp = for (self.deduplication_list.items[v1].items) |adjancent_to_v1| {
                    if (adjancent_to_v0 == adjancent_to_v1)
                        break adjancent_to_v0;
                } else null;
                if (temp) |t| break t;
            } else null;

            // If we found an adjacent polygon, connect the other side to us as well
            // as it isn't connected yet.
            if (adj_poly_index.*) |other_polygon_index| {
                const other_polygon = &self.polygons.items[other_polygon_index];

                const edge_index = for (other_polygon.vertices) |other_v0, i| {
                    const other_v1 = other_polygon.vertices[(i + 1) % other_polygon.vertices.len];

                    if ((other_v0 == v0 and other_v1 == v1) or (other_v0 == v1 and other_v1 == v0))
                        break i;
                } else unreachable; // the other polygon must have two adjacent vertices, otherwise we're having corrupted our data structure

                // TODO: Reset these on error!
                std.debug.assert(other_polygon.adjacent_polygons[edge_index] == null); // this must be given, otherwise we have an implementation error in the deduplication
                other_polygon.adjacent_polygons[edge_index] = polygon_index;
            }
        }

        // Finally add our polygon to the adjacency lists so we can find our polygon again
        // TODO: Clean up the lists on error!
        for (result.vertices) |vert_index| {
            try self.deduplication_list.items[vert_index].append(polygon_index);
        }

        try self.polygons.append(result);
    }

    /// Destroys the builder.
    pub fn deinit(self: *Self) void {
        self.arena.deinit();
        self.vertices.deinit();
        self.polygons.deinit();
        for (self.deduplication_list.items) |*list| {
            list.deinit();
        }
        self.deduplication_list.deinit();
        self.* = undefined;
    }
};

test "NavMesh Builder" {
    var builder = Builder.init(std.testing.allocator);
    defer builder.deinit();

    try builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
    });

    var mesh = try builder.createMesh(std.testing.allocator);
    defer mesh.deinit();

    std.testing.expectEqualSlices(Vertex, &[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
    }, mesh.vertices);

    std.testing.expectEqual(@as(usize, 1), mesh.polygons.len);

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 2 }, &mesh.polygons[0].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ null, null, null }, &mesh.polygons[0].adjacent_polygons);
}

test "NavMesh Builder (vertex deduplication)" {
    var builder = Builder.init(std.testing.allocator);
    defer builder.deinit();

    try builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
    });

    try builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 0, .z = 1 },
    });

    var mesh = try builder.createMesh(std.testing.allocator);
    defer mesh.deinit();

    std.testing.expectEqualSlices(Vertex, &[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
        .{ .x = 0, .y = 0, .z = 1 },
    }, mesh.vertices);

    std.testing.expectEqual(@as(usize, 2), mesh.polygons.len);

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 2 }, &mesh.polygons[0].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ 1, null, null }, &mesh.polygons[0].adjacent_polygons);

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 3 }, &mesh.polygons[1].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ 0, null, null }, &mesh.polygons[1].adjacent_polygons);
}

test "NavMesh Builder (polygon deduplication)" {
    var builder = Builder.init(std.testing.allocator);
    defer builder.deinit();

    try builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
    });

    try builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
    });

    var mesh = try builder.createMesh(std.testing.allocator);
    defer mesh.deinit();

    std.testing.expectEqualSlices(Vertex, &[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 1, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
    }, mesh.vertices);

    std.testing.expectEqual(@as(usize, 1), mesh.polygons.len);

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 2 }, &mesh.polygons[0].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ null, null, null }, &mesh.polygons[0].adjacent_polygons);
}

test "NavMesh Builder (error.DuplicatedVertex)" {
    // This test tests if the Builder recognized degenerated polygons that
    // have multiple vertices in the same location
    var builder = Builder.init(std.testing.allocator);
    defer builder.deinit();

    std.testing.expectError(error.DuplicateVertex, builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 0, .y = 0, .z = 0 },
    }));

    std.testing.expectError(error.DuplicateVertex, builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 0, .y = 1, .z = 0 },
        .{ .x = 0, .y = 0, .z = 0 },
    }));

    std.testing.expectError(error.DuplicateVertex, builder.insert(&[_]Vertex{
        .{ .x = 0, .y = 1, .z = 0 },
        .{ .x = 0, .y = 0, .z = 0 },
        .{ .x = 0, .y = 0, .z = 0 },
    }));
}

fn loadTestMesh() !NavMesh {
    const wavefront_obj = @import("wavefront-obj");

    var builder = Builder.init(std.testing.allocator);
    defer builder.deinit();

    {
        const model = try wavefront_obj.load(std.testing.allocator, "./data/basic.obj");
        defer model.deinit();

        for (model.faces.items) |face| {
            var verts: [4]Vertex = undefined;

            for (face.vertices[0..face.count]) |index, offset| {
                verts[offset] = Vertex{
                    .x = model.positions.items[index.position].x,
                    .y = model.positions.items[index.position].y,
                    .z = model.positions.items[index.position].z,
                };
            }

            try builder.insert(verts[0..face.count]);
        }
    }

    return try builder.createMesh(std.testing.allocator);
}

test "NavMesh (OBJ)" {
    // This testcase tests if the navmesh implementation survives a bigger mesh without any memory leaks
    var mesh = try loadTestMesh();
    defer mesh.deinit();
}

test "NavMesh (Serialize/Deserialize)" {
    // This testcase tests if serialization/deserialization yields the same navmesh
    var src_mesh = try loadTestMesh();
    defer src_mesh.deinit();

    var backing_buffer: [65536]u8 = undefined;

    const serialized_len = blk: {
        var stream = std.io.fixedBufferStream(&backing_buffer);
        try src_mesh.serialize(stream.writer());
        break :blk stream.pos;
    };

    var dst_mesh = try NavMesh.deserialize(std.testing.allocator, std.io.fixedBufferStream(backing_buffer[0..serialized_len]).reader());
    defer dst_mesh.deinit();

    std.testing.expectEqualSlices(Vertex, src_mesh.vertices, dst_mesh.vertices);
    std.testing.expectEqual(src_mesh.polygons.len, dst_mesh.polygons.len);
    for (src_mesh.polygons) |truth, i| {
        const tested = dst_mesh.polygons[i];
        std.testing.expectEqualSlices(usize, &truth.vertices, &tested.vertices);
        std.testing.expectEqualSlices(?usize, &truth.adjacent_polygons, &tested.adjacent_polygons);
    }
}
