// Copyright 2022 MAES
// 
// This file is part of MAES
// 
// MAES is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your option)
// any later version.
// 
// MAES is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along
// with MAES. If not, see http://www.gnu.org/licenses/.
// 
// Contributors: Malte Z. Andreasen, Philip I. Holler and Magnus K. Jensen
// 
// Original repository: https://github.com/MalteZA/MAES

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using System.Linq;
using UnityEditor;
using System.Collections;

namespace Maes.Map.MapGen
{
    internal class MeshGenerator : MonoBehaviour
    {
        // The inner walls, that the robots can collide with
        public MeshFilter InnerWalls3D;
        public MeshFilter InnerWalls2D;

        // Rendering for the walls
        public MeshRenderer InnerWallsRenderer3D;
        public MeshRenderer InnerWallsRenderer2D;

        // The outer/upper parts of the closed off cave
        public MeshFilter WallRoof;
        public MeshRenderer WallRoofRenderer;

        // The materials used to signify different types of walls
        public List<Material> Materials;

        [Tooltip("Include an invisible 3D collider on the inner walls to allow for ray trace collisions. " +
                 "Enabling this can impact performance of map generation by up to 2x")]
        public bool Include3DCollider = true;

        // Uses the marching squares algorithm to smooth out the grid and create a continuous wall around the rooms
        private SquareGrid _squareGrid2D;
        // Since the squares have an index and this index is depending on the 
        // order of the vertices, we have to include an additional grid for 3D.
        private SquareGrid _squareGrid3D;

        private List<Node> _vertices2D = new();
        private List<Node> _vertices3D = new();

        // list of all vertices in triangles
        // This list tells unity in which order to read the vertices.
        private Dictionary<TileType, List<int>> _triangles2D = new();
        private Dictionary<TileType, List<int>> _triangles3D = new();

        // Map from vertex index to all triangles containing the vertex.
        private readonly Dictionary<int, List<Triangle>> _triangleDictionary2D = new();
        private readonly Dictionary<int, List<Triangle>> _triangleDictionary3D = new();

        // A list of all outlines containing lists of vertex indexes contained in the given outline.
        // An outline is a wall either around an island of walls inside a room
        // or the walls around a room.
        private readonly List<List<Node>> _outlines2D = new();
        private readonly List<List<Node>> _outlines3D = new();

        // Used to avoid checking the same case twice
        private readonly HashSet<int> _checkedVertices2D = new();
        private readonly HashSet<int> _checkedVertices3D = new();

        private List<Node> _gizmoWallVertices = new();
        private Dictionary<TileType, List<int>> _gizmoWallTriangles = new();
        internal void ClearMesh()
        {
            _squareGrid2D = null;
            _squareGrid3D = null;
            Destroy(InnerWalls3D.gameObject.GetComponent<MeshCollider>());
            InnerWalls3D.sharedMesh?.Clear();
            Destroy(InnerWalls2D.gameObject.GetComponent<MeshCollider>());
            InnerWalls2D.sharedMesh?.Clear();
            WallRoof.sharedMesh?.Clear();
            _vertices2D.Clear();
            _vertices3D.Clear();
            _triangles2D.Clear();
            _triangles3D.Clear();
            _triangleDictionary2D.Clear();
            _triangleDictionary3D.Clear();
            _outlines2D.Clear();
            _outlines3D.Clear();
            _checkedVertices2D.Clear();
            _checkedVertices3D.Clear();
        }

        internal SimulationMap<Tile> GenerateMesh(Tile[,] map, float wallHeight,
            bool disableCornerRounding, List<Room> rooms)
        {
            InnerWallsRenderer2D.materials = Materials.ToArray();
            InnerWallsRenderer3D.materials = Materials.ToArray();

            // Generate grid of squares containing control nodes and between nodes 
            // for the marching square algorithm
            _squareGrid2D = new SquareGrid(map);
            _squareGrid3D = new SquareGrid(map);

            _vertices2D = new List<Node>();
            _triangles2D = new Dictionary<TileType, List<int>>();
            _vertices3D = new List<Node>();
            _triangles3D = new Dictionary<TileType, List<int>>();

            for (var x = 0; x < _squareGrid2D.Squares.GetLength(0); x++)
            {
                for (var y = 0; y < _squareGrid2D.Squares.GetLength(1); y++)
                {
                    // Create triangles from all the points in the squares
                    // assigned to variables "vertices" and "triangles"
                    TriangulateSquare(_squareGrid2D.Squares[x, y], false, disableCornerRounding);
                }
            }

            if (Include3DCollider)
            {
                for (var x = 0; x < _squareGrid3D.Squares.GetLength(0); x++)
                {
                    for (var y = 0; y < _squareGrid3D.Squares.GetLength(1); y++)
                    {
                        // Create triangles from all the points in the squares
                        // assigned to variables "vertices" and "triangles"
                        TriangulateSquare(_squareGrid3D.Squares[x, y], true, disableCornerRounding);
                    }
                }
            }

            CreateRoofMesh();
            CreateWallMesh(wallHeight, false);

            if (Include3DCollider)
            {
                // We must rotate and move the inner walls before creating the mesh, otherwise
                // the mesh, and thus the collider, will be created with the wrong orientation
                // Apparently, Unity does not update this, when the game object is rotated.
                InnerWalls3D.transform.rotation = Quaternion.AngleAxis(-90, Vector3.right);
                var oldPosition = InnerWalls3D.transform.position;
                InnerWalls3D.transform.position = new Vector3(oldPosition.x, oldPosition.y, -wallHeight);
                CreateWallMesh(wallHeight, true);
            }

            Generate2DColliders();

            return GenerateCollisionMap(_squareGrid2D,
                new Vector2(_squareGrid2D.XOffset, _squareGrid2D.YOffset), disableCornerRounding, rooms);
        }

        private void CreateRoofMesh()
        {
            // Create roof mesh
            var wallRoofMesh = new Mesh
            {
                vertices = _vertices2D.Select(vertex => vertex.Position).ToArray(),
            };
            var subMesh = 0;
            wallRoofMesh.subMeshCount = Materials.Count;

            foreach (var (_, indices) in _triangles2D)
            {
                wallRoofMesh.SetTriangles(indices, subMesh);
                subMesh++;
            }

            //_gizmoWallTriangles = _triangles2D;
            //_gizmoWallVertices = _vertices2D;

            WallRoofRenderer.materials = Materials.ToArray();
            // Apply mesh to wall roof
            WallRoof.mesh = wallRoofMesh;
        }

        public SimulationMap<Tile> GenerateCollisionMap(SquareGrid squareGrid, Vector3 offset,
            bool removeRoundedCorners, List<Room> rooms)
        {
            var width = squareGrid.Squares.GetLength(0);
            var height = squareGrid.Squares.GetLength(1);
            // Create a bool type SimulationMap with default value of false in all cells
            var collisionMap = new SimulationMap<Tile>(() => new Tile(TileType.Room), width, height, offset, rooms);

            for (var x = 0; x < width; x++)
            {
                for (var y = 0; y < height; y++)
                {
                    var square = squareGrid.Squares[x, y];
                    var collisionTile = collisionMap.GetTileByLocalCoordinate(x, y);
                    // Create triangles from all the points in the squares
                    // assigned to variables "vertices" and "triangles"
                    AdaptCollisionMapTile(collisionTile, square, removeRoundedCorners);
                }
            }

            return collisionMap;
        }

        private static void AdaptCollisionMapTile(SimulationMapTile<Tile> tile, Square square, bool removeRoundedCorners)
        {
            int[] triangles = { };
            switch (square.Configuration)
            {
                case 0b0000:
                    break;

                // 1 point:
                case 0b0001:
                    triangles = removeRoundedCorners ? new[] { 0, 1 } : new[] { 0 };
                    break;
                case 0b0010:
                    triangles = removeRoundedCorners ? new[] { 2, 3 } : new[] { 3 };
                    break;
                case 0b0100:
                    triangles = removeRoundedCorners ? new[] { 6, 7 } : new[] { 7 };
                    break;
                case 0b1000:
                    triangles = removeRoundedCorners ? new[] { 4, 5 } : new[] { 4 };
                    break;

                // 2 points:
                case 0b0011:
                    triangles = new[] { 0, 1, 2, 3 };
                    break;
                case 0b0110:
                    triangles = new[] { 2, 3, 6, 7 };
                    break;
                case 0b1001:
                    triangles = new[] { 0, 1, 4, 5 };
                    break;
                case 0b1100:
                    triangles = new[] { 4, 5, 6, 7 };
                    break;
                case 0b0101:
                    triangles = new[] { 0, 1, 2, 5, 6, 7 };
                    break;
                case 0b1010:
                    triangles = new[] { 1, 2, 3, 4, 5, 6 };
                    break;

                // 3 points:
                case 0b0111:
                    triangles = removeRoundedCorners ? new[] { 0, 1, 2, 3, 6, 7 } : new[] { 0, 1, 2, 3, 5, 6, 7 };
                    break;
                case 0b1011:
                    triangles = removeRoundedCorners ? new[] { 0, 1, 2, 3, 4, 5 } : new[] { 0, 1, 2, 3, 4, 5, 6 };
                    break;
                case 0b1101:
                    triangles = removeRoundedCorners ? new[] { 0, 1, 4, 5, 6, 7 } : new[] { 0, 1, 2, 4, 5, 6, 7 };
                    break;
                case 0b1110:
                    triangles = removeRoundedCorners ? new[] { 2, 3, 4, 5, 6, 7 } : new[] { 1, 2, 3, 4, 5, 6, 7 };
                    break;

                // 4 point:
                case 0b1111:
                    triangles = new[] { 0, 1, 2, 3, 4, 5, 6, 7 };
                    break;
            }

            foreach (var index in triangles)
            {
                tile.SetCellValue(index, new Tile(square.Center.Type));
            }
        }

        private void Generate2DColliders()
        {
            // remove colliders from last build
            var currentColliders = gameObject.GetComponents<EdgeCollider2D>();
            foreach (var currentCollider in currentColliders)
            {
                Destroy(currentCollider);
            }

            // Assigns outline vertices to list variable "outlines"
            // An outline is a wall either around an island of walls inside a room
            // or the walls around a room.
            // CalculateMeshOutlines();
            foreach (var outline in _outlines2D)
            {
                var edgeCollider = gameObject.AddComponent<EdgeCollider2D>();
                var edgePoints = new Vector2[outline.Count];
                for (var i = 0; i < outline.Count; i++)
                {
                    edgePoints[i] = new Vector2(outline[i].Position.x, outline[i].Position.y);
                }

                edgeCollider.points = edgePoints;
            }
        }

        private void CreateWallMesh(float wallHeight, bool isMesh3D)
        {
            // Assigns outline vertices to list variable "outlines"
            // An outline is a wall either around an island of walls inside a room
            // or the walls around a room.
            CalculateMeshOutlines(isMesh3D);

            var wallVertices = new List<Node>();
            var wallTriangles = new Dictionary<TileType, List<int>>();
            var innerWallsMesh = new Mesh();

            var outlines = isMesh3D ? _outlines3D : _outlines2D;

            foreach (var outline in outlines)
            {
                for (var i = 0; i < outline.Count - 1; i++)
                {
                    var topLeft = outline[i];
                    var topRight = outline[i + 1];
                    var wallTypes = new[] { topLeft, topRight }.Where(x => Tile.IsWall(x.Type)).ToList();
                    if (wallTypes.Count() == 1)
                    {
                        topLeft.Type = wallTypes[0].Type;
                        topRight.Type = wallTypes[0].Type;
                    }

                    // The wall stick out in different axes depending on 2D or 3D.
                    var direction = isMesh3D ? Vector3.up : Vector3.back;
                    var bottomLeft = new Node(topLeft, topLeft.Position - direction * wallHeight);
                    var bottomRight = new Node(topRight, topRight.Position - direction * wallHeight);



                    // Create section of the wall currently being made
                    // as viewed from inside the room looked at the wall
                    wallVertices.Add(topLeft); // top left (0)
                    wallVertices.Add(topRight); // top right (1)
                    wallVertices.Add(bottomLeft); // bottom left (2)
                    wallVertices.Add(bottomRight); // bottom right (3)
                }
            }

            var wallIndexType = new Dictionary<TileType, List<int>>();
            for (var i = 0; i < wallVertices.Count; i++)
            {
                var type = wallVertices[i].Type;
                if (wallIndexType.ContainsKey(type))
                    wallIndexType[type].Add(i);
                else
                    wallIndexType[type] = new List<int> { i };
            }

            var startVertexIndex = 0;
            foreach (var (type, vertexIndices) in wallIndexType)
            {
                wallTriangles[type] = new List<int>();
                for (var i = startVertexIndex; i < startVertexIndex + vertexIndices.Count; i += 4)
                {
                    // The "outside" of the mesh with the texture depends on the order
                    // Since the rotation is vertical for 2D, we have to invert the order
                    // Triangle one (left side lower side of the wall square)
                    wallTriangles[type].Add(i + 0); // Top left
                    wallTriangles[type].Add(i + 2); // Bottom left
                    wallTriangles[type].Add(i + 3); // Bottom right

                    // Triangle two (right side upper side of the wall square)
                    wallTriangles[type].Add(i + 3); // Bottom Right
                    wallTriangles[type].Add(i + 1); // Top right
                    wallTriangles[type].Add(i + 0); // Top left
                }
                startVertexIndex += vertexIndices.Count;
            }

            innerWallsMesh.subMeshCount = wallIndexType.Keys.Count;
            
            // Debug variables
            _gizmoWallVertices = wallVertices;
            _gizmoWallTriangles = wallTriangles;

            innerWallsMesh.vertices = wallVertices.Select(vertex => vertex.Position).ToArray();

            var subMeshIndex = 0;
            foreach (var type in wallIndexType.Keys)
            {
                innerWallsMesh.SetTriangles(wallTriangles[type], subMeshIndex);
                subMeshIndex++;
            }

            if (isMesh3D)
            {
                var wallCollider = InnerWalls3D.gameObject.AddComponent<MeshCollider>();
                wallCollider.sharedMesh = innerWallsMesh;
                InnerWalls3D.mesh = innerWallsMesh;
            }
            else
            {
                InnerWalls2D.mesh = innerWallsMesh;
            }
        }

        // According to the marching squares algorithm,
        // there are 16 cases, since there are 16 combinations of ON
        // and OFF for a box where each of the 4 corners can have either of these 
        // states.
        // Find the states in this image: http://jamie-wong.com/2014/08/19/metaballs-and-marching-squares/#MathJax-Element-15-Frame
        // removeRoundedCorners simply ignores case 1, 2, 4, 7, 8, 11, 13, 14 by using a center point to square off the edges
        private void TriangulateSquare(Square square, bool isMesh3D, bool removeRoundedCorners = false)
        {
            switch (square.Configuration)
            {
                case 0b0000:
                    break;

                // 1 points:
                case 0b001:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.Center, square.CentreBottom, square.BottomLeft, square.CentreLeft);
                    else
                        MeshFromPoints(isMesh3D, square.CentreLeft, square.CentreBottom, square.BottomLeft);
                    break;
                case 0b0010:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.CentreRight, square.BottomRight, square.CentreBottom, square.Center);
                    else
                        MeshFromPoints(isMesh3D, square.BottomRight, square.CentreBottom, square.CentreRight);
                    break;
                case 0b0100:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.TopRight, square.CentreRight, square.Center, square.CentreTop);
                    else
                        MeshFromPoints(isMesh3D, square.TopRight, square.CentreRight, square.CentreTop);
                    break;
                case 0b1000:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.TopLeft, square.CentreTop, square.Center, square.CentreLeft);
                    else
                        MeshFromPoints(isMesh3D, square.TopLeft, square.CentreTop, square.CentreLeft);
                    break;

                // 2 points:
                case 0b0011:
                    MeshFromPoints(isMesh3D, square.CentreRight, square.BottomRight, square.BottomLeft, square.CentreLeft);
                    break;
                case 0b0110:
                    MeshFromPoints(isMesh3D, square.CentreTop, square.TopRight, square.BottomRight, square.CentreBottom);
                    break;
                case 0b1001:
                    MeshFromPoints(isMesh3D, square.TopLeft, square.CentreTop, square.CentreBottom, square.BottomLeft);
                    break;
                case 0b1100:
                    MeshFromPoints(isMesh3D, square.TopLeft, square.TopRight, square.CentreRight, square.CentreLeft);
                    break;
                case 0b0101:
                    MeshFromPoints(isMesh3D, square.CentreTop, square.TopRight, square.CentreRight, square.CentreBottom,
                        square.BottomLeft, square.CentreLeft);
                    break;
                case 0b1010:
                    MeshFromPoints(isMesh3D, square.TopLeft, square.CentreTop, square.CentreRight, square.BottomRight,
                        square.CentreBottom, square.CentreLeft);
                    break;

                // 3 point:
                case 0b0111:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.Center, square.CentreTop, square.TopRight, square.BottomRight,
                            square.BottomLeft, square.CentreLeft);
                    else
                        MeshFromPoints(isMesh3D, square.CentreTop, square.TopRight, square.BottomRight, square.BottomLeft,
                            square.CentreLeft);
                    break;
                case 0b1011:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.Center, square.CentreRight, square.BottomRight, square.BottomLeft,
                            square.TopLeft, square.CentreTop);
                    else
                        MeshFromPoints(isMesh3D, square.TopLeft, square.CentreTop, square.CentreRight, square.BottomRight,
                            square.BottomLeft);
                    break;
                case 0b1101:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.Center, square.CentreBottom, square.BottomLeft, square.TopLeft,
                            square.TopRight, square.CentreRight);
                    else
                        MeshFromPoints(isMesh3D, square.TopLeft, square.TopRight, square.CentreRight, square.CentreBottom,
                            square.BottomLeft);
                    break;
                case 0b1110:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.Center, square.CentreLeft, square.TopLeft, square.TopRight,
                            square.BottomRight, square.CentreBottom);
                    else
                        MeshFromPoints(isMesh3D, square.TopLeft, square.TopRight, square.BottomRight, square.CentreBottom,
                            square.CentreLeft);
                    break;

                // 4 point:
                case 0b1111:
                    MeshFromPoints(isMesh3D, square.TopLeft, square.TopRight, square.BottomRight, square.BottomLeft);
                    // If all 4 are active walls, it cannot be an outline to a room, and should thus not be checked
                    if (isMesh3D)
                    {
                        _checkedVertices3D.Add(square.TopLeft.VertexIndex);
                        _checkedVertices3D.Add(square.TopRight.VertexIndex);
                        _checkedVertices3D.Add(square.BottomRight.VertexIndex);
                        _checkedVertices3D.Add(square.BottomLeft.VertexIndex);
                    }
                    else
                    {
                        _checkedVertices2D.Add(square.TopLeft.VertexIndex);
                        _checkedVertices2D.Add(square.TopRight.VertexIndex);
                        _checkedVertices2D.Add(square.BottomRight.VertexIndex);
                        _checkedVertices2D.Add(square.BottomLeft.VertexIndex);
                    }

                    break;
            }
        }

        private void MeshFromPoints(bool isMesh3D, params Node[] points)
        {
            AssignIndexesToVertices(points, isMesh3D);

            if (points.Length >= 3)
                CreateTriangle(points[0], points[1], points[2], isMesh3D);
            if (points.Length >= 4)
                CreateTriangle(points[0], points[2], points[3], isMesh3D);
            if (points.Length >= 5)
                CreateTriangle(points[0], points[3], points[4], isMesh3D);
            if (points.Length >= 6)
                CreateTriangle(points[0], points[4], points[5], isMesh3D);
        }

        private void AssignIndexesToVertices(IEnumerable<Node> points, bool isMesh3D)
        {
            foreach (var point in points)
            {
                if (point.VertexIndex != -1)
                {
                    if (!Tile.IsWall(_vertices2D[point.VertexIndex].Type) && Tile.IsWall(point.Type))
                        _vertices2D[point.VertexIndex].Type = point.Type;
                    continue;
                }
                point.VertexIndex = isMesh3D ? _vertices3D.Count : _vertices2D.Count;
                if (isMesh3D)
                    _vertices3D.Add(point);
                else
                {
                    // The map is rotated in 2d mode                                                             
                    var pos2D = new Vector3(point.Position.x, point.Position.z, point.Position.y);
                    var rotatedPoint = new Node(point, pos2D);
                    _vertices2D.Add(rotatedPoint);
                }
            }
        }

        private void CreateTriangle(Node a, Node b, Node c, bool isMesh3D)
        {
            var triangles = isMesh3D ? _triangles3D : _triangles2D;
            var triangle = new Triangle(a, b, c);

            if (!triangles.ContainsKey(triangle.Type))
                triangles[triangle.Type] = new List<int>();

            triangles[triangle.Type].Add(triangle.VertexA.VertexIndex);
            triangles[triangle.Type].Add(triangle.VertexB.VertexIndex);
            triangles[triangle.Type].Add(triangle.VertexC.VertexIndex);

            AddTriangleToDictionary(triangle.VertexA, triangle, isMesh3D);
            AddTriangleToDictionary(triangle.VertexB, triangle, isMesh3D);
            AddTriangleToDictionary(triangle.VertexC, triangle, isMesh3D);
        }

        private void AddTriangleToDictionary(Node vertexKey, Triangle triangle, bool isMesh3D)
        {
            var triangleDictionary = isMesh3D ? _triangleDictionary3D : _triangleDictionary2D;

            if (triangleDictionary.ContainsKey(vertexKey.VertexIndex))
            {
                triangleDictionary[vertexKey.VertexIndex].Add(triangle);
            }
            else
            {
                var triangleList = new List<Triangle> { triangle };
                triangleDictionary.Add(vertexKey.VertexIndex, triangleList);
            }
        }

        private void CalculateMeshOutlines(bool isMesh3D)
        {
            var vertices = isMesh3D ? _vertices3D : _vertices2D;
            var outlines = isMesh3D ? _outlines3D : _outlines2D;
            var checkedVertices = isMesh3D ? _checkedVertices3D : _checkedVertices2D;

            for (var vertexIndex = 0; vertexIndex < vertices.Count; vertexIndex++)
            {
                if (checkedVertices.Contains(vertexIndex))
                    continue;

                var vertex = vertices[vertexIndex];
                var newOutlineVertex = GetConnectedOutlineVertexIndex(vertex, isMesh3D);
                if (newOutlineVertex == -1)
                    continue;
                checkedVertices.Add(vertexIndex);

                var newOutline = new List<Node> { vertex };
                outlines.Add(newOutline);
                FollowOutline(newOutlineVertex, outlines.Count - 1, isMesh3D);
                outlines[^1].Add(vertices[vertexIndex]);
            }
        }

        private void FollowOutline(int vertexIndex, int outlineIndex, bool isMesh3D)
        {
            var vertices = isMesh3D ? _vertices3D : _vertices2D;
            var checkedVertices = isMesh3D ? _checkedVertices3D : _checkedVertices2D;
            var outlines = isMesh3D ? _outlines3D : _outlines2D;

            while (true)
            {
                var vertex = vertices[vertexIndex];
                outlines[outlineIndex].Add(vertex);
                checkedVertices.Add(vertex.VertexIndex);
                var nextVertexIndex = GetConnectedOutlineVertexIndex(vertex, isMesh3D);

                if (nextVertexIndex != -1)
                {
                    vertexIndex = nextVertexIndex;
                    continue;
                }
                break;
            }
        }

        private int GetConnectedOutlineVertexIndex(Node vertexA, bool isMesh3D)
        {
            var trianglesContainingVertex = isMesh3D ? _triangleDictionary3D[vertexA.VertexIndex] : _triangleDictionary2D[vertexA.VertexIndex];

            foreach (var triangle in trianglesContainingVertex)
            {
                for (var j = 0; j < 3; j++)
                {
                    var vertexB = triangle[j];
                    var isChecked = isMesh3D ? _checkedVertices3D.Contains(vertexB.VertexIndex) : _checkedVertices2D.Contains(vertexB.VertexIndex);
                    if (vertexB == vertexA || isChecked)
                        continue;
                    if (IsOutlineEdge(vertexA.VertexIndex, vertexB.VertexIndex, isMesh3D))
                        return vertexB.VertexIndex;
                }
            }
            return -1;
        }

        private bool IsOutlineEdge(int vertexIndexA, int vertexIndexB, bool isMesh3D)
        {
            // The inner walls made up of triangles are recognized based on the
            // number of triangles shared between the two vertices. The outer ones only have 1 in common.
            var trianglesContainingVertexA = isMesh3D ? _triangleDictionary3D[vertexIndexA] : _triangleDictionary2D[vertexIndexA];
            var sharedTriangleCount = 0;

            for (var i = 0; i < trianglesContainingVertexA.Count; i++)
            {
                if (!trianglesContainingVertexA[i].Contains(vertexIndexB))
                    continue;
                sharedTriangleCount++;
                if (sharedTriangleCount > 1)
                {
                    break;
                }
            }

            return sharedTriangleCount == 1;
        }

        private readonly struct Triangle
        {
            public readonly Node VertexA;
            public readonly Node VertexB;
            public readonly Node VertexC;
            public readonly TileType Type;
            private readonly Node[] _vertices;

            public Triangle(Node a, Node b, Node c)
            {
                VertexA = a;
                VertexB = b;
                VertexC = c;

                var types = new List<TileType> { a.Type, b.Type, c.Type };
                var type = types.Max();
                if (!Tile.IsWall(type) && (Tile.IsWall(a.Type) || Tile.IsWall(b.Type) || Tile.IsWall(c.Type)))
                {
                    if (Tile.IsWall(a.Type)) type = a.Type;
                    else if (Tile.IsWall(b.Type)) type = b.Type;
                    else type = c.Type;

                    a.Type = type;
                    b.Type = type;
                    c.Type = type;
                }
                Type = type;

                _vertices = new Node[3];
                _vertices[0] = a;
                _vertices[1] = b;
                _vertices[2] = c;
            }

            public Node this[int i] => _vertices[i];

            public bool Contains(int vertexIndex)
            {
                return vertexIndex == VertexA.VertexIndex || vertexIndex == VertexB.VertexIndex || vertexIndex == VertexC.VertexIndex;
            }
        }

        internal class SquareGrid
        {
            public Square[,] Squares;
            public readonly float XOffset, YOffset;

            public SquareGrid(Tile[,] map)
            {
                var nodeCountX = map.GetLength(0);
                var nodeCountY = map.GetLength(1);
                float mapWidth = nodeCountX;
                float mapHeight = nodeCountY;
                // float squareSize = 1f;

                // Create map of control nodes
                var controlNodes = new ControlNode[nodeCountX, nodeCountY];

                // In Marching squares, squares are offset by 0.5 
                XOffset = -mapWidth / 2 + 0.5f;
                YOffset = -mapHeight / 2 + 0.5f;

                for (var x = 0; x < nodeCountX; x++)
                {
                    for (var y = 0; y < nodeCountY; y++)
                    {
                        // Divided by 2, since we start in 0,0 and can go both above and below 0.
                        var position = new Vector3(x + XOffset, 0, y + YOffset);
                        controlNodes[x, y] = new ControlNode(position, map[x, y].Type);
                    }
                }

                Squares = new Square[nodeCountX - 1, nodeCountY - 1];
                for (var x = 0; x < nodeCountX - 1; x++)
                {
                    for (var y = 0; y < nodeCountY - 1; y++)
                    {
                        Squares[x, y] = new Square(controlNodes[x, y + 1],
                            controlNodes[x + 1, y + 1],
                            controlNodes[x + 1, y],
                            controlNodes[x, y]);
                    }
                }
            }
        }

        internal class Square
        {
            // This class is used in the marching squares algorithm.
            // Control nodes can be either on or off
            public ControlNode TopLeft, TopRight, BottomRight, BottomLeft;
            public Node CentreTop, CentreRight, CentreBottom, CentreLeft;
            public Node Center; // Used for square off for offices. Ignoring case 1, 2, 4, 7, 8, 11, 13, 14
            public int Configuration;

            public Square(ControlNode topLeft, ControlNode topRight, ControlNode bottomRight, ControlNode bottomLeft)
            {
                TopLeft = topLeft;
                TopRight = topRight;
                BottomRight = bottomRight;
                BottomLeft = bottomLeft;

                // Assign references
                CentreTop = TopLeft.Right;
                CentreRight = BottomRight.Above;
                CentreBottom = BottomLeft.Right;
                CentreLeft = BottomLeft.Above;

                // Find middle
                var xDiff = Math.Abs(topLeft.Position.x - topRight.Position.x);
                var zDiff = Math.Abs(topLeft.Position.z - bottomLeft.Position.z);
                var centerX = bottomLeft.Position.x + (xDiff / 2f);
                var centerZ = bottomLeft.Position.z + (zDiff / 2f);

                var types = new List<TileType>
                {
                    topLeft.Type,
                    topRight.Type,
                    bottomLeft.Type,
                    bottomRight.Type,
                };
                var maxType = types.Max();

                Center = new Node(new Vector3(centerX, topLeft.Position.y, centerZ), maxType);

                // There are only 16 possible configurations
                // Consider them in binary xxxx
                // First bit is bottomLeft, second is bottomRight etc.
                // this way we can deduct the configuration from the active nodes
                // like below.
                if (TopLeft.IsWall)
                    Configuration += 0b1000;
                if (TopRight.IsWall)
                    Configuration += 0b0100;
                if (BottomRight.IsWall)
                    Configuration += 0b0010;
                if (BottomLeft.IsWall)
                    Configuration += 0b0001;
            }
        }

        internal class Node
        {
            public Vector3 Position;
            public int VertexIndex = -1;
            public TileType Type;

            public Node(Vector3 position, TileType type)
            {
                Position = position;
                Type = type;
            }
            public Node(Node node, Vector3 position)
            {
                Position = position;
                Type = node.Type;
                VertexIndex = node.VertexIndex;
            }
        }

        internal class ControlNode : Node
        {
            public bool IsWall;
            public Node Above, Right;

            public ControlNode(Vector3 position, TileType type) : base(position, type)
            {
                IsWall = Tile.IsWall(type);
                Above = new Node(Position + Vector3.forward / 2f, type);
                Right = new Node(Position + Vector3.right / 2f, type);
            }
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            foreach (var (type, vertices) in _gizmoWallTriangles)
            {

                Gizmos.color = type switch
                {
                    TileType.Wall => Color.black,
                    TileType.Room => Color.white,
                    TileType.Hall => Color.gray,
                    TileType.Wood => Color.green,
                    TileType.Concrete => Color.yellow,
                    TileType.Brick => Color.red,
                    _ => Color.blue
                };

                //Handles.color = type switch
                //{
                //    TileType.Wall => Color.black,
                //    TileType.Room => Color.white,
                //    TileType.Hall => Color.gray,
                //    TileType.Concrete => Color.yellow,
                //    TileType.Wood => Color.green,
                //    TileType.Brick => Color.red,
                //    _ => Color.blue
                //};

                for (var index = 0; index < vertices.Count - 1; index++)
                {
                    var vec = _gizmoWallVertices[vertices[index]].Position;
                    vec.Set(vec.x, vec.y, vec.z - 2f);
                    var vec2 = _gizmoWallVertices[vertices[index + 1]].Position;
                    vec2.Set(vec2.x, vec2.y, vec2.z - 2f);
                    Gizmos.DrawLine(vec, vec2);
                    //Handles.Label(vec, index.ToString());
                }

            }
        }
#endif
    }
}
