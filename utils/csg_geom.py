import math


#Represents a 3D vector.
#
#Example usage:
#
#  CSG.Vector(1, 2, 3);
#  CSG.Vector([1, 2, 3]);
#  CSG.Vector({ x: 1, y: 2, z: 3 });
class Vector(object):
    def __init__(self, o):
        self.x = o[0]
        self.y = o[1]
        self.z = o[2]
        
    def clone(self):
        return Vector([self.x, self.y, self.z])

    def times(self, a):
        return Vector([self.x * a, self.y * a, self.z * a])

    def dividedBy(self, a):
        return Vector([self.x / a, self.y / a, self.z / a])

    def plus(self, a):
        return Vector([self.x + a.x, self.y + a.y, self.z + a.z])

    def minus(self, a):
        return Vector([self.x - a.x, self.y - a.y, self.z - a.z])

    def unit(self):
        return self.dividedBy(self.length())

    def length(self):
        return math.sqrt(self.dot(self))

    def dot(self, a):
        return self.x * a.x + self.y * a.y + self.z * a.z

    def cross(self, a):
        return Vector([self.y * a.z - self.z * a.y, self.z * a.x - self.x * a.z, self.x * a.y - self.y * a.x])

    def negated(self):
        return Vector([-self.x, -self.y, -self.z])

    def lerp(self, a, t):
        return self.plus(a.minus(self).times(t))

#Represents a vertex of a polygon. Use your own vertex class instead of this
#one to provide additional features like texture coordinates and vertex
#colors. Custom vertex classes need to provide a `pos` property and `clone()`,
#`flip()`, and `interpolate()` methods that behave analogous to the ones
#defined by `CSG.Vertex`. This class provides `normal` so convenience
#functions like `CSG.sphere()` can return a smooth vertex normal, but `normal`
#is not used anywhere else.
class Vertex(object):

    def __init__(self, pos, normal):
        self.pos = pos.clone()
        self.normal = normal.clone()

    def clone(self):
        return Vertex(self.pos.clone(), self.normal.clone())

    # Invert all orientation-specific data (e.g. vertex normal). Called when the
    # orientation of a polygon is flipped.
    def flip(self):
        self.normal = self.normal.negated()

    # Create a vertex between this vertex and `other` by linearly
    # interpolating all properties using a parameter of `t`. Subclasses should
    # override this to interpolate additional properties.
    def interpolate(self, other, t):
        return Vertex(self.pos.lerp(other.pos, t), self.normal.lerp(other.normal, t))
    
#Holds a node in a BSP tree. A BSP tree is built from a collection of polygons
#by picking a polygon to split along. That polygon (and all other coplanar
#polygons) are added directly to that node and the other polygons are added to
#the front and/or back subtrees. This is not a leafy BSP tree since there is
#no distinction between internal and leaf nodes.
class Node(object):
    
    def __init__(self, polygons = None):
        self.plane = None
        self.front = None
        self.back = None
        self.polygons = []

        if (polygons != None):
            self.build(polygons)

    def clone(self):
        node = Node()
        node.plane = None if self.plane == None else self.plane.clone()
        node.front = None if self.front == None else self.front.clone()
        node.back = None if self.back == None else self.back.clone()

        if self.polygons != None:
            node.polygons = []

            for polygon in self.polygons:
                node.polygons.append(polygon.clone())

        return node

    # Convert solid space to empty space and empty space to solid space.
    def invert(self):
        for i in range (0, len(self.polygons)):
            self.polygons[i].flip()

        self.plane.flip()

        if self.front != None:
            self.front.invert()
        if self.back != None:
            self.back.invert()

        temp = self.front
        self.front = self.back
        self.back = temp

    # Recursively remove all polygons in `polygons` that are inside this BSP
    # tree.
    def clipPolygons(self, polygons):
        if self.plane == None:
            return polygons[:]
        front = []
        back = []
        for i in range(0, len(polygons)):
            self.plane.splitPolygon(polygons[i], front, back, front, back)
            
        if self.front != None:
            front = self.front.clipPolygons(front)
        if self.back != None:
            back = self.back.clipPolygons(back)
        else:
            back = []
        
        return front + back

    # Remove all polygons in this BSP tree that are inside the other BSP tree
    # `bsp`.
    def clipTo(self, bsp):
        self.polygons = bsp.clipPolygons(self.polygons)
        
        if self.front != None:
            self.front.clipTo(bsp)

        if self.back != None:
            self.back.clipTo(bsp)

    # Return a list of all polygons in this BSP tree.
    def allPolygons(self):
        polygons = self.polygons[:]

        if self.front != None:
            polygons += self.front.allPolygons()

        if self.back != None:
            polygons += self.back.allPolygons()

        return polygons

    # Build a BSP tree out of `polygons`. When called on an existing tree, the
    # polygons are filtered down to the bottom of the tree and become new
    # nodes there. Each set of polygons is partitioned using the first polygon
    # (no heuristic is used to pick a good split).
    def build(self, polygons):
        if len(polygons) == 0:
            return
        
        if self.plane == None:
            self.plane = polygons[0].plane.clone()

        front = []
        back = []
        for i in range(0, len(polygons)):
            self.plane.splitPolygon(polygons[i], self.polygons, self.polygons, front, back)
        
        if len(front) > 0:
            if self.front == None:
                self.front = Node()
            
            self.front.build(front)
        
        if len(back) > 0:
            if self.back == None:
                self.back = Node()
            
            self.back.build(back)


#Represents a plane in 3D space.
class Plane(object):

    # `CSG.Plane.EPSILON` is the tolerance used by `splitPolygon()` to decide if a
    # point is on the plane.
    EPSILON = 1.0e-5
    
    COPLANAR = 0
    FRONT = 1
    BACK = 2
    SPANNING = 3

    def __init__(self, normal, w):
        self.normal = normal
        self.w = w

    @staticmethod
    def fromPoints(a, b, c):
        n = b.minus(a).cross(c.minus(a)).unit()
        return Plane(n, n.dot(a))

    def clone(self):
        return Plane(self.normal.clone(), self.w)

    def flip(self):
        self.normal = self.normal.negated()
        self.w = -self.w

    # Split `polygon` by this plane if needed, then put the polygon or polygon
    # fragments in the appropriate lists. Coplanar polygons go into either
    # `coplanarFront` or `coplanarBack` depending on their orientation with
    # respect to this plane. Polygons in front or in back of this plane go into
    # either `front` or `back`.
    def splitPolygon(self, polygon, coplanarFront, coplanarBack, front, back):
        # Classify each point as well as the entire polygon into one of the above
        # four classes.
        polygonType = 0
        types = []
        for i in range(0, len(polygon.vertices)):
            t = self.normal.dot(polygon.vertices[i].pos) - self.w
            currentType = Plane.BACK if (t < -Plane.EPSILON) else Plane.FRONT if (t > Plane.EPSILON) else Plane.COPLANAR
            polygonType |= currentType
            types.append(currentType)

        # Put the polygon in the correct list, splitting it when necessary.
        if (polygonType == Plane.COPLANAR):
            (coplanarFront if self.normal.dot(polygon.plane.normal) > 0 else coplanarBack).append(polygon)
        elif (polygonType == Plane.FRONT):
            front.append(polygon)
        if (polygonType == Plane.BACK):
            back.append(polygon)
        if (polygonType == Plane.SPANNING):
            f = []
            b = []
            for i in range(0, len(polygon.vertices)):
                j = (i + 1) % len(polygon.vertices)
                ti = types[i]
                tj = types[j]
                vi = polygon.vertices[i]
                vj = polygon.vertices[j]
                if (ti != Plane.BACK):
                    f.append(vi)
                if (ti != Plane.FRONT):
                    b.append(vi.clone() if ti != Plane.BACK else vi)
                if ((ti | tj) == Plane.SPANNING):
                    t = (self.w - self.normal.dot(vi.pos)) / self.normal.dot(vj.pos.minus(vi.pos))
                    v = vi.interpolate(vj, t)
                    f.append(v)
                    b.append(v.clone())
            
            if (len(f) >= 3):
                front.append(Polygon(f, polygon.shared))
            if (len(b) >= 3):
                back.append(Polygon(b, polygon.shared))
                
#Represents a convex polygon. The vertices used to initialize a polygon must
#be coplanar and form a convex loop. They do not have to be `CSG.Vertex`
#instances but they must behave similarly (duck typing can be used for
#customization).
#
#Each convex polygon has a `shared` property, which is shared between all
#polygons that are clones of each other or were split from the same polygon.
#This can be used to define per-polygon properties (such as surface color).
class Polygon:

    vertices = None
    shared = None
    plane = None

    def __init__(self, vertices, shared = None):
        self.vertices = vertices
        self.shared = shared
        self.plane = Plane.fromPoints(vertices[0].pos, vertices[1].pos, vertices[2].pos)

    def clone(self):
        vertexClone = []

        for vertex in self.vertices:
            vertexClone.append(vertex)

        return Polygon(vertexClone, self.shared)

    def flip(self):
        self.vertices.reverse()

        for vertex in self.vertices:
            vertex.flip()

        self.plane.flip()