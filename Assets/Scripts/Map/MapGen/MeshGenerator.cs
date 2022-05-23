using System;
using System.Collections.Generic;
using UnityEngine;

namespace Maes.Map.MapGen {
    public class MeshGenerator : MonoBehaviour {
        /**
	 * Uses the marching squares algorithm to smooth out
	 * the grid and create a continuous wall around the rooms 
	 */
        public SquareGrid squareGrid2D;
        // Since the squares have an index and this index is depending on the 
        // order of the vertices, we have to include an additional grid for 3D.
        public SquareGrid squareGrid3D;

        // The inner walls, that the robots can collide with
        public MeshFilter innerWalls3D;
        public MeshFilter innerWalls2D;

        // The outer/upper parts of the closed off cave
        public MeshFilter wallRoof;

        private List<Vector3> vertices2D = new List<Vector3>();
        private List<Vector3> vertices3D = new List<Vector3>();

        // list of all vertices in triangles
        // This list tells unity in which order to read the vertices.
        private List<int> triangles2D = new List<int>();
        private List<int> triangles3D = new List<int>();

        // Map from vertex index to all triangles containing the vertex.
        private Dictionary<int, List<Triangle>> triangleDictionary2D = new Dictionary<int, List<Triangle>>();
        private Dictionary<int, List<Triangle>> triangleDictionary3D = new Dictionary<int, List<Triangle>>();

        // A list of all outlines containing lists of vertex indexes contained in the given outline.
        // An outline is a wall either around an island of walls inside a room
        // or the walls around a room.
        private List<List<int>> outlines2D = new List<List<int>>();
        private List<List<int>> outlines3D  = new List<List<int>>();

        // Used to avoid checking the same case twice
        private HashSet<int> checkedVertices2D = new HashSet<int>();
        private HashSet<int> checkedVertices3D = new HashSet<int>();

        [Tooltip("Include an invisible 3D collider on the inner walls to allow for ray trace collisions. " +
                 "Enabling this can impact performance of map generation by up to 2x")]
        public bool include3DCollider = true;
        
        private const int WALL_TYPE = 1, ROOM_TYPE = 0;

        public void ClearMesh() {
            squareGrid2D = null;
            squareGrid3D = null;
            Destroy(innerWalls3D.gameObject.GetComponent<MeshCollider>());
            innerWalls3D.mesh.Clear();
            Destroy(innerWalls2D.gameObject.GetComponent<MeshCollider>());
            innerWalls2D.mesh.Clear();
            wallRoof.mesh.Clear();
            vertices2D.Clear();
            vertices3D.Clear();
            triangles2D.Clear();
            triangles3D.Clear();
            triangleDictionary2D.Clear();
            triangleDictionary3D.Clear();
            outlines2D.Clear();
            outlines3D.Clear();
            checkedVertices2D.Clear();
            checkedVertices3D.Clear();
        }

        public SimulationMap<bool> GenerateMesh(int[,] map, float wallHeight,
            bool disableCornerRounding, List<Room> rooms) {

            // Generate grid of squares containing control nodes and between nodes 
            // for the marching square algorithm
            squareGrid2D = new SquareGrid(map);
            squareGrid3D = new SquareGrid(map);

            vertices2D = new List<Vector3>();
            triangles2D = new List<int>();
            vertices3D = new List<Vector3>();
            triangles3D = new List<int>();

            for (int x = 0; x < squareGrid2D.squares.GetLength(0); x++) {
                for (int y = 0; y < squareGrid2D.squares.GetLength(1); y++) {
                    // Create triangles from all the points in the squares
                    // assigned to variables "vertices" and "triangles"
                    TriangulateSquare(squareGrid2D.squares[x, y], false, disableCornerRounding);
                }
            }

            if (include3DCollider) {
                for (int x = 0; x < squareGrid3D.squares.GetLength(0); x++) {
                    for (int y = 0; y < squareGrid3D.squares.GetLength(1); y++) {
                        // Create triangles from all the points in the squares
                        // assigned to variables "vertices" and "triangles"
                        TriangulateSquare(squareGrid3D.squares[x,y], true, disableCornerRounding);
                    }
                }
            }

            // Create roof mesh
            Mesh wallRoofMesh = new Mesh();

            wallRoofMesh.vertices = vertices2D.ToArray();
            wallRoofMesh.triangles = triangles2D.ToArray();
            wallRoofMesh.RecalculateNormals();

            // Apply mesh to wall roof
            wallRoof.mesh = wallRoofMesh;

            CreateWallMesh(wallHeight, false);

            if (include3DCollider) {
                // We must rotate and move the inner walls before creating the mesh, otherwise
                // the mesh, and thus the collider, will be created with the wrong orientation
                // Apparently, Unity does not update this, when the game object is rotated.
                innerWalls3D.transform.rotation = Quaternion.AngleAxis(-90, Vector3.right);
                var oldPosition = innerWalls3D.transform.position;
                innerWalls3D.transform.position = new Vector3(oldPosition.x, oldPosition.y, -wallHeight);
                CreateWallMesh(wallHeight, true);
            }

            Generate2DColliders();

            return GenerateCollisionMap(squareGrid2D,
                new Vector2(squareGrid2D.XOffset, squareGrid2D.YOffset), disableCornerRounding, rooms);
        }

        private SimulationMap<bool> GenerateCollisionMap(SquareGrid squareGrid, Vector3 offset,
            bool removeRoundedCorners, List<Room> rooms) {
            var width = squareGrid.squares.GetLength(0);
            var height = squareGrid.squares.GetLength(1);
            // Create a bool type SimulationMap with default value of false in all cells
            SimulationMap<bool> collisionMap = new SimulationMap<bool>(() => false, width, height, offset, rooms);

            for (int x = 0; x < width; x++) {
                for (int y = 0; y < height; y++) {
                    var square = squareGrid.squares[x, y];
                    var collisionTile = collisionMap.GetTileByLocalCoordinate(x, y);
                    // Create triangles from all the points in the squares
                    // assigned to variables "vertices" and "triangles"
                    AdaptCollisionMapTile(collisionTile, square, removeRoundedCorners);
                }
            }

            return collisionMap;
        }

        private void AdaptCollisionMapTile(SimulationMapTile<bool> tile, Square square, bool removeRoundedCorners) {
            int[] triangles = { };
            switch (square.configuration) {
                case 0:
                    break;

                // 1 point:
                case 1:
                    triangles = removeRoundedCorners ? new int[] {0, 1} : new[] {0};
                    break;
                case 2:
                    triangles = removeRoundedCorners ? new int[] {2, 3} : new[] {3};
                    break;
                case 4:
                    triangles = removeRoundedCorners ? new int[] {6, 7} : new[] {7};
                    break;
                case 8:
                    triangles = removeRoundedCorners ? new int[] {4, 5} : new[] {4};
                    break;

                // 2 points:
                case 3:
                    triangles = new[] {0, 1, 2, 3};
                    break;
                case 6:
                    triangles = new[] {2, 3, 6, 7};
                    break;
                case 9:
                    triangles = new[] {0, 1, 4, 5};
                    break;
                case 12:
                    triangles = new[] {4, 5, 6, 7};
                    break;
                case 5:
                    triangles = new[] {0, 1, 2, 5, 6, 7};
                    break;
                case 10:
                    triangles = new[] {1, 2, 3, 4, 5, 6};
                    break;

                // 3 points:
                case 7:
                    triangles = removeRoundedCorners ? new[] {0, 1, 2, 3, 6, 7} : new[] {0, 1, 2, 3, 5, 6, 7};
                    break;
                case 11:
                    triangles = removeRoundedCorners ? new[] {0, 1, 2, 3, 4, 5} : new[] {0, 1, 2, 3, 4, 5, 6};
                    break;
                case 13:
                    triangles = removeRoundedCorners ? new[] {0, 1, 4, 5, 6, 7} : new[] {0, 1, 2, 4, 5, 6, 7};
                    break;
                case 14:
                    triangles = removeRoundedCorners ? new[] {2, 3, 4, 5, 6, 7} : new[] {1, 2, 3, 4, 5, 6, 7};
                    break;

                // 4 point:
                case 15:
                    triangles = new[] {0, 1, 2, 3, 4, 5, 6, 7};
                    break;
            }

            foreach (var index in triangles) {
                tile.SetCellValue(index, true);
            }
        }

        void Generate2DColliders() {
            // remove colliders from last build
            EdgeCollider2D[] currentColliders = gameObject.GetComponents<EdgeCollider2D>();
            for (int i = 0; i < currentColliders.Length; i++) {
                Destroy(currentColliders[i]);
            }

            // Assigns outline vertices to list variable "outlines"
            // An outline is a wall either around an island of walls inside a room
            // or the walls around a room.
            // CalculateMeshOutlines();
            foreach(List<int> outline in outlines2D) {
                EdgeCollider2D edgeCollider = gameObject.AddComponent<EdgeCollider2D>();
                Vector2[] edgePoints = new Vector2[outline.Count];

                for (int i = 0; i < outline.Count; i++) {
                    edgePoints[i] = new Vector2(vertices2D[outline[i]].x, vertices2D[outline[i]].y);
                }

                edgeCollider.points = edgePoints;
            }
        }

        void CreateWallMesh(float wallHeight, bool isMesh3D) {
            // Assigns outline vertices to list variable "outlines"
            // An outline is a wall either around an island of walls inside a room
            // or the walls around a room.
            CalculateMeshOutlines(isMesh3D);

            List<Vector3> wallVertices = new List<Vector3>();
            List<int> wallTriangles = new List<int>();
            Mesh innerWallsMesh = new Mesh();

            var vertices = isMesh3D ? vertices3D : vertices2D;
            var outlines = isMesh3D ? outlines3D : outlines2D;

            foreach (List<int> outline in outlines) {
                for (int i = 0; i < outline.Count - 1; i++) {
                    int startIndex = wallVertices.Count;
                    // Create section of the wall currently being made
                    // as viewed from inside the room looked at the wall
                    wallVertices.Add(vertices[outline[i]]); // top left (0)
                    wallVertices.Add(vertices[outline[i + 1]]); // top right (1)

                    // The wall stick out in different axes depending on 2D or 3D.
                    if (isMesh3D) {
                        wallVertices.Add(vertices[outline[i]] - Vector3.up * wallHeight); // bottom left (2)
                        wallVertices.Add(vertices[outline[i + 1]] - Vector3.up * wallHeight); // bottom right (3)
                        
                    }
                    else {
                        wallVertices.Add(vertices[outline[i]] - Vector3.back * wallHeight); // bottom left (2)
                        wallVertices.Add(vertices[outline[i + 1]] - Vector3.back * wallHeight); // bottom right (3)
                    }

                    // The "outside" of the mesh with the texture depends on the order
                    // Since the rotation is vertical for 2D, we have to invert the order
                    // Triangle one (left side lower side of the wall square)
                    wallTriangles.Add(startIndex + 0); // Top left
                    wallTriangles.Add(startIndex + 2); // Bottom left
                    wallTriangles.Add(startIndex + 3); // Bottom right

                    // Triangle two (right side upper side of the wall square)
                    wallTriangles.Add(startIndex + 3); // Bottom Right
                    wallTriangles.Add(startIndex + 1); // Top right
                    wallTriangles.Add(startIndex + 0); // Top left
                }
            }

            // Unity cannot work with lists, so ToArray() is needed
            innerWallsMesh.vertices = wallVertices.ToArray();
            innerWallsMesh.triangles = wallTriangles.ToArray();

            if (isMesh3D) {
                MeshCollider wallCollider = innerWalls3D.gameObject.AddComponent<MeshCollider>();
                wallCollider.sharedMesh = innerWallsMesh; 
                innerWalls3D.mesh = innerWallsMesh;
            }
            else {
                innerWalls2D.mesh = innerWallsMesh;
            }
        }

        // According to the marching squares algorithm,
        // there are 16 cases, since there are 16 combinations of ON
        // and OFF for a box where each of the 4 corners can have either of these 
        // states.
        // Find the states in this image: http://jamie-wong.com/2014/08/19/metaballs-and-marching-squares/#MathJax-Element-15-Frame
        // removeRoundedCorners simply ignores case 1, 2, 4, 7, 8, 11, 13, 14 by using a center point to square off the edges
        void TriangulateSquare(Square square, bool isMesh3D, bool removeRoundedCorners = false) {
            switch (square.configuration) {
                case 0:
                    break;

                // 1 points:
                case 1:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.center, square.centreBottom, square.bottomLeft, square.centreLeft);
                    else
                        MeshFromPoints(isMesh3D, square.centreLeft, square.centreBottom, square.bottomLeft);
                    break;
                case 2:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.centreRight, square.bottomRight, square.centreBottom, square.center);
                    else
                        MeshFromPoints(isMesh3D, square.bottomRight, square.centreBottom, square.centreRight);
                    break;
                case 4:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.topRight, square.centreRight, square.center, square.centreTop);
                    else
                        MeshFromPoints(isMesh3D, square.topRight, square.centreRight, square.centreTop);
                    break;
                case 8:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.topLeft, square.centreTop, square.center, square.centreLeft);
                    else
                        MeshFromPoints(isMesh3D, square.topLeft, square.centreTop, square.centreLeft);
                    break;

                // 2 points:
                case 3:
                    MeshFromPoints(isMesh3D, square.centreRight, square.bottomRight, square.bottomLeft, square.centreLeft);
                    break;
                case 6:
                    MeshFromPoints(isMesh3D, square.centreTop, square.topRight, square.bottomRight, square.centreBottom);
                    break;
                case 9:
                    MeshFromPoints(isMesh3D, square.topLeft, square.centreTop, square.centreBottom, square.bottomLeft);
                    break;
                case 12:
                    MeshFromPoints(isMesh3D, square.topLeft, square.topRight, square.centreRight, square.centreLeft);
                    break;
                case 5:
                    MeshFromPoints(isMesh3D, square.centreTop, square.topRight, square.centreRight, square.centreBottom,
                        square.bottomLeft, square.centreLeft);
                    break;
                case 10:
                    MeshFromPoints(isMesh3D, square.topLeft, square.centreTop, square.centreRight, square.bottomRight,
                        square.centreBottom, square.centreLeft);
                    break;

                // 3 point:
                case 7:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.center, square.centreTop, square.topRight, square.bottomRight,
                            square.bottomLeft, square.centreLeft);
                    else
                        MeshFromPoints(isMesh3D, square.centreTop, square.topRight, square.bottomRight, square.bottomLeft,
                            square.centreLeft);
                    break;
                case 11:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.center, square.centreRight, square.bottomRight, square.bottomLeft,
                            square.topLeft, square.centreTop);
                    else
                        MeshFromPoints(isMesh3D, square.topLeft, square.centreTop, square.centreRight, square.bottomRight,
                            square.bottomLeft);
                    break;
                case 13:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.center, square.centreBottom, square.bottomLeft, square.topLeft,
                            square.topRight, square.centreRight);
                    else
                        MeshFromPoints(isMesh3D, square.topLeft, square.topRight, square.centreRight, square.centreBottom,
                            square.bottomLeft);
                    break;
                case 14:
                    if (removeRoundedCorners)
                        MeshFromPoints(isMesh3D, square.center, square.centreLeft, square.topLeft, square.topRight,
                            square.bottomRight, square.centreBottom);
                    else
                        MeshFromPoints(isMesh3D, square.topLeft, square.topRight, square.bottomRight, square.centreBottom,
                            square.centreLeft);
                    break;

                // 4 point:
                case 15:
                    MeshFromPoints(isMesh3D, square.topLeft, square.topRight, square.bottomRight, square.bottomLeft);
                    // If all 4 are active walls, it cannot be an outline to a room, and should thus not be checked
                    if (isMesh3D) {
                        checkedVertices3D.Add(square.topLeft.vertexIndex);
                        checkedVertices3D.Add(square.topRight.vertexIndex);
                        checkedVertices3D.Add(square.bottomRight.vertexIndex);
                        checkedVertices3D.Add(square.bottomLeft.vertexIndex);
                    }
                    else {
                        checkedVertices2D.Add(square.topLeft.vertexIndex);
                        checkedVertices2D.Add(square.topRight.vertexIndex);
                        checkedVertices2D.Add(square.bottomRight.vertexIndex);
                        checkedVertices2D.Add(square.bottomLeft.vertexIndex);
                    }
                    
                    break;
            }
        }

        void MeshFromPoints(bool isMesh3D, params Node[] points) {
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

        void AssignIndexesToVertices(Node[] points, bool isMesh3D) {
            for (int i = 0; i < points.Length; i++) {
                if (points[i].vertexIndex == -1) {
                    points[i].vertexIndex = isMesh3D ? vertices3D.Count : vertices2D.Count;
                    if (isMesh3D) {
                        vertices3D.Add(points[i].position); 
                    }
                    else {
                        // The map is rotated in 2d mode                                                             
                        var pos2D = new Vector3(points[i].position.x, points[i].position.z, points[i].position.y);   
                        vertices2D.Add(pos2D);  
                    }
                }
            }
        }

        void CreateTriangle(Node a, Node b, Node c, bool isMesh3D) {
            var triangles = isMesh3D ? triangles3D : triangles2D;

            triangles.Add(a.vertexIndex);
            triangles.Add(b.vertexIndex);
            triangles.Add(c.vertexIndex);


            Triangle triangle = new Triangle(a.vertexIndex, b.vertexIndex, c.vertexIndex);
            AddTriangleToDictionary(triangle.vertexIndexA, triangle, isMesh3D);
            AddTriangleToDictionary(triangle.vertexIndexB, triangle, isMesh3D);
            AddTriangleToDictionary(triangle.vertexIndexC, triangle, isMesh3D);
        }

        void AddTriangleToDictionary(int vertexIndexKey, Triangle triangle, bool isMesh3D) {
            var triangleDictionary = isMesh3D ? triangleDictionary3D : triangleDictionary2D;
            
            if (triangleDictionary.ContainsKey(vertexIndexKey)) {
                triangleDictionary[vertexIndexKey].Add(triangle);
            }
            else {
                List<Triangle> triangleList = new List<Triangle>();
                triangleList.Add(triangle);
                triangleDictionary.Add(vertexIndexKey, triangleList);
            }
        }

        void CalculateMeshOutlines(bool isMesh3D) {
            var vertices = isMesh3D ? vertices3D : vertices2D;
            var outlines = isMesh3D ? outlines3D : outlines2D;
            var checkedVertices = isMesh3D ? checkedVertices3D : checkedVertices2D;
            
            for (int vertexIndex = 0; vertexIndex < vertices.Count; vertexIndex++) {
                if (!checkedVertices.Contains(vertexIndex)) {
                    int newOutlineVertex = GetConnectedOutlineVertex(vertexIndex, isMesh3D);
                    if (newOutlineVertex != -1) {
                        checkedVertices.Add(vertexIndex);

                        List<int> newOutline = new List<int>();
                        newOutline.Add(vertexIndex);
                        outlines.Add(newOutline);
                        FollowOutline(newOutlineVertex, outlines.Count - 1, isMesh3D);
                        outlines[outlines.Count - 1].Add(vertexIndex);
                    }
                }
            }
        }

        void FollowOutline(int vertexIndex, int outlineIndex, bool isMesh3D) {
            if (isMesh3D) {
                outlines3D[outlineIndex].Add(vertexIndex);
                checkedVertices3D.Add(vertexIndex);
                int nextVertexIndex = GetConnectedOutlineVertex(vertexIndex, isMesh3D);

                if (nextVertexIndex != -1) {
                    FollowOutline(nextVertexIndex, outlineIndex, isMesh3D);
                }
            }
            else 
            {
                outlines2D[outlineIndex].Add(vertexIndex);
                checkedVertices2D.Add(vertexIndex);
                int nextVertexIndex = GetConnectedOutlineVertex(vertexIndex, isMesh3D);

                if (nextVertexIndex != -1) {
                    FollowOutline(nextVertexIndex, outlineIndex, isMesh3D);
                }  
            }
            
        }

        int GetConnectedOutlineVertex(int vertexIndex, bool isMesh3D) {
            List<Triangle> trianglesContainingVertex = isMesh3D ? triangleDictionary3D[vertexIndex] : triangleDictionary2D[vertexIndex];

            for (int i = 0; i < trianglesContainingVertex.Count; i++) {
                Triangle triangle = trianglesContainingVertex[i];

                for (int j = 0; j < 3; j++) {
                    int vertexB = triangle[j];
                    bool isChecked = isMesh3D ? checkedVertices3D.Contains(vertexB) : checkedVertices2D.Contains(vertexB);
                    if (vertexB != vertexIndex && !isChecked) {
                        if (IsOutlineEdge(vertexIndex, vertexB, isMesh3D)) {
                            return vertexB;
                        }
                    }
                }
            }

            return -1;
        }

        bool IsOutlineEdge(int vertexA, int vertexB, bool isMesh3D) {
            // The inner walls made up of triangles are recognized based on the
            // number of triangles shared between the two vertices. The outer ones only have 1 in common.
            List<Triangle> trianglesContainingVertexA = isMesh3D ? triangleDictionary3D[vertexA] :  triangleDictionary2D[vertexA];
            int sharedTriangleCount = 0;

            for (int i = 0; i < trianglesContainingVertexA.Count; i++) {
                if (trianglesContainingVertexA[i].Contains(vertexB)) {
                    sharedTriangleCount++;
                    if (sharedTriangleCount > 1) {
                        break;
                    }
                }
            }

            return sharedTriangleCount == 1;
        }

        struct Triangle {
            public int vertexIndexA;
            public int vertexIndexB;
            public int vertexIndexC;
            int[] vertices;

            public Triangle(int a, int b, int c) {
                vertexIndexA = a;
                vertexIndexB = b;
                vertexIndexC = c;

                vertices = new int[3];
                vertices[0] = a;
                vertices[1] = b;
                vertices[2] = c;
            }

            public int this[int i] {
                get { return vertices[i]; }
            }


            public bool Contains(int vertexIndex) {
                return vertexIndex == vertexIndexA || vertexIndex == vertexIndexB || vertexIndex == vertexIndexC;
            }
        }

        public class SquareGrid {
            public Square[,] squares;
            public readonly float XOffset, YOffset;

            public SquareGrid(int[,] map) {
                int nodeCountX = map.GetLength(0);
                int nodeCountY = map.GetLength(1);
                float mapWidth = nodeCountX;
                float mapHeight = nodeCountY;
                // float squareSize = 1f;

                // Create map of control nodes
                ControlNode[,] controlNodes = new ControlNode[nodeCountX, nodeCountY];

                // In Marching squares, squares are offset by 0.5 
                XOffset = -mapWidth / 2 + 0.5f;
                YOffset = -mapHeight / 2 + 0.5f;

                for (int x = 0; x < nodeCountX; x++) {
                    for (int y = 0; y < nodeCountY; y++) {
                        // Divided by 2, since we start in 0,0 and can go both above and below 0.
                        Vector3 position = new Vector3(x + XOffset, 0, y + YOffset);
                        controlNodes[x, y] = new ControlNode(position, map[x, y] == WALL_TYPE);
                    }
                }

                squares = new Square[nodeCountX - 1, nodeCountY - 1];
                for (int x = 0; x < nodeCountX - 1; x++) {
                    for (int y = 0; y < nodeCountY - 1; y++) {
                        squares[x, y] = new Square(controlNodes[x, y + 1],
                            controlNodes[x + 1, y + 1],
                            controlNodes[x + 1, y],
                            controlNodes[x, y]);
                    }
                }
            }
        }

        public class Square {
            // This class is used in the marching squares algorithm.
            // Control nodes can be either on or off
            public ControlNode topLeft, topRight, bottomRight, bottomLeft;
            public Node centreTop, centreRight, centreBottom, centreLeft;
            public Node center; // Used for square off for offices. Ignoring case 1, 2, 4, 7, 8, 11, 13, 14
            public int configuration;

            public Square(ControlNode topLeft, ControlNode topRight, ControlNode bottomRight, ControlNode bottomLeft) {
                this.topLeft = topLeft;
                this.topRight = topRight;
                this.bottomRight = bottomRight;
                this.bottomLeft = bottomLeft;

                // Assign references
                centreTop = this.topLeft.right;
                centreRight = this.bottomRight.above;
                centreBottom = this.bottomLeft.right;
                centreLeft = this.bottomLeft.above;

                // Find middle
                var xDiff = Math.Abs(topLeft.position.x - topRight.position.x);
                var zDiff = Math.Abs(topLeft.position.z - bottomLeft.position.z);
                var centerX = bottomLeft.position.x + (xDiff / 2f);
                var centerZ = bottomLeft.position.z + (zDiff / 2f);
                this.center = new Node(new Vector3(centerX, topLeft.position.y, centerZ));

                // There are only 16 possible configurations
                // Consider them in binary xxxx
                // First bit is bottomLeft, second is bottomRight etc.
                // this way we can deduct the configuration from the active nodes
                // like below.
                if (this.topLeft.isWall)
                    configuration += 8;
                if (this.topRight.isWall)
                    configuration += 4;
                if (this.bottomRight.isWall)
                    configuration += 2;
                if (this.bottomLeft.isWall)
                    configuration += 1;
            }
        }

        public class Node {
            public Vector3 position;
            public int vertexIndex = -1;

            public Node(Vector3 position) {
                this.position = position;
            }
        }

        public class ControlNode : Node {
            public bool isWall;
            public Node above, right;

            public ControlNode(Vector3 position, bool isWall) : base(position) {
                this.isWall = isWall;
                above = new Node(base.position + Vector3.forward / 2f);
                right = new Node(base.position + Vector3.right / 2f);
            }
        }
    }
}