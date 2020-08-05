const std = @import("std");

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
        // TODO
        unreachable;
    }

    /// Releases all resources in the NavMesh.
    pub fn deinit(self: *Self) void {
        self.arena.deinit();
        self.* = undefined;
    }

    /// Serializes the NavMesh into a data stream which can later be deserialized again.
    pub fn serialize(self: Self, writer: anytype) !void {
        // TODO
        unreachable;
    }
};

/// A node in the `NavMesh` tree structure
pub const Polygon = struct {
    /// The vertex coordinates of the polygon. Each entry is an index into `NavMesh.vertices`.
    vertices: []usize,

    /// Adjacent polygons that are connected at vertex `[i]` and `[(i+1)%N]`. Each entry is an index into `NavMesh.polygons`.
    adjacent_polygons: []?usize,
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

        for (mesh.polygons) |*poly, i| {
            poly.vertices = try mesh.arena.allocator.dupe(usize, poly.vertices);
            poly.adjacent_polygons = try mesh.arena.allocator.dupe(?usize, poly.adjacent_polygons);
        }

        return mesh;
    }

    /// Inserts the given polygon into the mesh.
    /// Note that this is a fairly slow process and might not be efficient, if you don't need a dynamic NavMesh,
    /// consider pregenerating it and store a serialized version.
    /// TODO: Create a data structure / library that allows "viewing" a slice with certain properties (x,y,z) in that case
    pub fn insert(self: *Self, polygon: []const Vertex) !void {
        std.debug.assert(polygon.len >= 3); // calling it with <3 elements is an API violation

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
        const vertices = try self.arena.allocator.alloc(usize, polygon.len);
        for (vertices) |*vert, index| {
            const insert_vertex = polygon[index];
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
                self.arena.allocator.free(vertices);
                return;
            }
        }

        var result = Polygon{
            .vertices = vertices,
            .adjacent_polygons = try self.arena.allocator.alloc(?usize, polygon.len),
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

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 2 }, mesh.polygons[0].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ null, null, null }, mesh.polygons[0].adjacent_polygons);
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

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 2 }, mesh.polygons[0].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ 1, null, null }, mesh.polygons[0].adjacent_polygons);

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 3 }, mesh.polygons[1].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ 0, null, null }, mesh.polygons[1].adjacent_polygons);
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

    std.testing.expectEqualSlices(usize, &[_]usize{ 0, 1, 2 }, mesh.polygons[0].vertices);
    std.testing.expectEqualSlices(?usize, &[_]?usize{ null, null, null }, mesh.polygons[0].adjacent_polygons);
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

test "NavMesh (OBJ)" {
    // This testcase tests if the navmesh implementation survives a bigger mesh without any memory leaks
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

    var mesh = try builder.createMesh(std.testing.allocator);
    defer mesh.deinit();
}
