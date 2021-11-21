using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using XPBD;
using System.Text;

[RequireComponent(typeof(MeshFilter))]
public class SoftBodyMesh : MonoBehaviour
{
    [SerializeField]
    private float _mass = 1;
    [SerializeField]
    private float _distanceFlexibility = 0;
    [SerializeField]
    private float _bendingFlexibility = 0.1f;

    [SerializeField]
    [Range(4, 20)]
    private int _iterateCount = 4;

    [SerializeField]
    [Range(0f, 1f)]
    private float _bounciness = 0.5f;

    [SerializeField]
    [Range(0f, 1)]
    private float _dynamicFrictionFactor = 0.5f;

    [SerializeField]
    private bool _enableBendingConstraint = false;

    private XPBDSolver _solver;

    private int[] _vertexToBone;

    private List<Vector2Int> _edges = new List<Vector2Int>();

    [SerializeField]
    private bool _enableAttach = false;

    [SerializeField]
    private uint[] _attachParticleIndexes = new uint[] { 0 };

    private List<Vector3> _attachParticleInitOffsets = new List<Vector3>();

    [SerializeField]
    private bool _enableGroudCollider = false;

    private bool _hasAttached = false;

    private NativeArray<Vector3> _vertices;
    private Mesh _mesh;
    [SerializeField]
    private bool _drawGizmos = true;
    [SerializeField]
    [Range(0.01f, 0.1f)]
    private float _gizmoPointSize = 0.1f;

    [SerializeField]
    private Material _material;



    void Start()
    {
        var meshFilter = GetComponent<MeshFilter>();
        _mesh = meshFilter.mesh;

        _vertices = new NativeArray<Vector3>(_mesh.vertexCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);

        _solver = this.CreateSolver(_mesh);
        if (_enableGroudCollider)
        {

            _solver.colliderManager.AddInfinitePlane(new InfinitePlane()
            {
                normal = Vector3.up,
                originToPlane = -GameObject.FindWithTag("Ground").transform.position.y,
            });
        }
    }

    private void BuildVertexMap(Vector3[] vertices, out int[] vertexToBone, out Vector3[] bonePositions)
    {
        vertexToBone = new int[vertices.Length];
        var bonePositionsList = new List<Vector3>();

        for (var i = 0; i < vertexToBone.Length; i++)
        {
            vertexToBone[i] = -1;
        }
        for (var i = 0; i < vertices.Length; i++)
        {
            var v0 = vertices[i];
            int boneIndex = vertexToBone[i];
            if (boneIndex < 0)
            {
                boneIndex = bonePositionsList.Count;
                vertexToBone[i] = boneIndex;
                bonePositionsList.Add(v0);
            }

            for (var j = i + 1; j < vertices.Length; j++)
            {
                var v1 = vertices[j];
                if (Vector3.Distance(v0, v1) < 0.01)
                {
                    vertexToBone[j] = boneIndex;
                }
            }
        }
        bonePositions = bonePositionsList.ToArray();
    }

    private XPBDSolver CreateSolver(Mesh mesh)
    {
        var vertices = mesh.vertices;

        Vector3[] bonePositions;

        BuildVertexMap(vertices, out _vertexToBone, out bonePositions);
        var triangles = mesh.triangles;
        var particleCount = bonePositions.Length;
        NativeArray<Vector3> positions = new NativeArray<Vector3>(particleCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        NativeArray<float> masses = new NativeArray<float>(particleCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        var localToWorld = transform.localToWorldMatrix;
        for (var i = 0; i < particleCount; i++)
        {
            positions[i] = localToWorld.MultiplyPoint3x4(bonePositions[i]);
            masses[i] = _mass / particleCount;
        }

        Dictionary<int, DistanceConstraint> distanceConstraintsMap = new Dictionary<int, DistanceConstraint>();
        _edges.Clear();
        BendingConstraintBuilder bendingConstraintBuilder = new BendingConstraintBuilder()
        {
            stiffnessInv = _bendingFlexibility,
        };
        for (var i = 0; i < triangles.Length / 3; i++)
        {
            var v1 = triangles[3 * i];
            var v2 = triangles[3 * i + 1];
            var v3 = triangles[3 * i + 2];

            var b1 = _vertexToBone[v1];
            var b2 = _vertexToBone[v2];
            var b3 = _vertexToBone[v3];

            var e1 = GetEdgeId(b1, b2);
            var e2 = GetEdgeId(b2, b3);
            var e3 = GetEdgeId(b3, b1);


            if (!distanceConstraintsMap.ContainsKey(e1))
            {
                distanceConstraintsMap.Add(e1, new DistanceConstraint((ulong)b1, (ulong)b2, _distanceFlexibility));
                _edges.Add(new Vector2Int(b1, b2));
            }
            if (!distanceConstraintsMap.ContainsKey(e2))
            {
                distanceConstraintsMap.Add(e2, new DistanceConstraint((ulong)b2, (ulong)b3, _distanceFlexibility));
                _edges.Add(new Vector2Int(b2, b3));
            }
            if (!distanceConstraintsMap.ContainsKey(e3))
            {
                distanceConstraintsMap.Add(e3, new DistanceConstraint((ulong)b3, (ulong)b1, _distanceFlexibility));
                _edges.Add(new Vector2Int(b3, b1));
            }
            if (_enableBendingConstraint)
            {
                bendingConstraintBuilder.AddTriangle(b1, b2, b3);
            }
        }

        NativeArray<DistanceConstraint> distanceConstraints = new NativeArray<DistanceConstraint>(distanceConstraintsMap.Count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        int index = 0;
        foreach (var pair in distanceConstraintsMap)
        {
            distanceConstraints[index++] = pair.Value;
        }

        Debug.Log(OutputPositions(positions));
        Debug.Log(OutputDistanceCosntraints(distanceConstraints));


        var solverBuilder = new XPBDSolverBuilder()
            .FillDistanceConstraints(distanceConstraints)
            .FillPositions(positions)
            .FillMasses(masses)
            .SetBounciness(_bounciness)
            .SetDynamicFrictionFactor(_dynamicFrictionFactor)
            .SetIterateCount(_iterateCount);

        NativeArray<TriangleBendingConstraint> bendingConstraints = default;
        if (_enableBendingConstraint)
        {
            var bendingConstraintsList = bendingConstraintBuilder.ToBendingConstraints();
            bendingConstraints = new NativeArray<TriangleBendingConstraint>(bendingConstraintsList.ToArray(), Allocator.Temp);
            solverBuilder.FillTriangleBendingConstraints(bendingConstraints);
            Debug.Log(OutputBendingConstraints(bendingConstraints));
        }
        var solver = solverBuilder.Build();

        //gravity
        solver.AddFieldAcceleration(Vector3.down * 9.8f);

        foreach (var attachParticleIndex in _attachParticleIndexes)
        {
            _attachParticleInitOffsets.Add(positions[(int)attachParticleIndex] - transform.position);
        }
        distanceConstraints.Dispose();
        positions.Dispose();
        masses.Dispose();
        if (bendingConstraints.IsCreated)
        {
            bendingConstraints.Dispose();
        }
        return solver;
    }

    internal static int GetEdgeId(int pIndex1, int pIndex2)
    {
        if (pIndex1 < pIndex2)
        {
            return (pIndex1 << 16) | pIndex2;
        }
        else
        {
            return (pIndex2 << 16) | pIndex1;
        }
    }

    private void UpdateVertices()
    {
        for (var i = 0; i < _vertexToBone.Length; i++)
        {
            var boneIndex = _vertexToBone[i];
            _vertices[i] = _solver.GetPosition(boneIndex);
        }
        _mesh.SetVertices(_vertices);
        _mesh.RecalculateNormals();
        _mesh.RecalculateBounds();
    }

    void Update()
    {
        if (_solver == null)
        {
            return;
        }

        if (_enableAttach)
        {
            var index = 0;
            foreach (var pIndex in _attachParticleIndexes)
            {
                _solver.AttachParticle(pIndex, this.transform.position + _attachParticleInitOffsets[index]);
                index++;
            }
            _hasAttached = true;
        }
        else if (_hasAttached)
        {
            foreach (var pIndex in _attachParticleIndexes)
            {
                _solver.DetachParticle(pIndex);
            }
        }
        _solver.Update();
        this.UpdateVertices();
        Graphics.DrawMesh(_mesh, Vector3.zero, Quaternion.identity, _material, gameObject.layer);
    }

    void OnDrawGizmos()
    {
        if (_solver == null)
        {
            return;
        }
        if (!_drawGizmos)
        {
            return;
        }
        for (var i = 0; i < _solver.particleCount; i++)
        {
            Gizmos.DrawSphere(_solver.GetPosition(i), _gizmoPointSize);
        }

        foreach (var edge in _edges)
        {
            var p1 = _solver.GetPosition(edge.x);
            var p2 = _solver.GetPosition(edge.y);
            Gizmos.DrawLine(p1, p2);
        }
    }

    void OnDestroy()
    {
        _vertices.Dispose();
    }

    private string OutputPositions(NativeArray<Vector3> positions)
    {
        StringBuilder builder = new StringBuilder();
        for (var i = 0; i < positions.Length; i++)
        {
            var pos = positions[i];
            builder.AppendFormat("point![{0},{1},{2}],", pos.x, pos.y, pos.z);
            builder.AppendLine();
        }
        return builder.ToString();
    }

    private string OutputDistanceCosntraints(NativeArray<DistanceConstraint> distanceConstraints)
    {
        StringBuilder builder = new StringBuilder();
        for (var i = 0; i < distanceConstraints.Length; i++)
        {
            var cons = distanceConstraints[i];
            builder.AppendFormat("DistanceConstraint::new({0}, {1}, {2:f}),", cons.index0, cons.index1, cons.stiffnessInv);
            builder.AppendLine();
        }
        return builder.ToString();
    }

    private string OutputBendingConstraints(NativeArray<TriangleBendingConstraint> bendingConstraints)
    {
        StringBuilder builder = new StringBuilder();
        for (var i = 0; i < bendingConstraints.Length; i++)
        {
            var cons = bendingConstraints[i];
            builder.AppendLine("TriangleBendConstraint {");
            builder.AppendFormat("p_indexes: [{0}, {1}, {2}, {3}],\n", cons.index0, cons.index1, cons.index2, cons.index3);
            builder.AppendFormat("stiffness_inv: {0:f},\n", cons.stiffnessInv);
            builder.AppendLine("},");
        }
        return builder.ToString();
    }



    private class BendingConstraintBuilder
    {
        private Dictionary<int, List<int>> _bendingVerticesMap = new Dictionary<int, List<int>>();
        public float stiffnessInv;

        private void FillBendingConstruct(int p1, int p2, int p3)
        {
            var e = GetEdgeId(p1, p2);
            List<int> vertices;
            if (!_bendingVerticesMap.TryGetValue(e, out vertices))
            {
                vertices = new List<int>(4);
                vertices.Add(p1);
                vertices.Add(p2);
                _bendingVerticesMap.Add(e, vertices);
            }
            vertices.Add(p3);
        }
        public void AddTriangle(int p1, int p2, int p3)
        {
            FillBendingConstruct(p1, p2, p3);
            FillBendingConstruct(p2, p3, p1);
            FillBendingConstruct(p3, p1, p2);
        }

        public List<TriangleBendingConstraint> ToBendingConstraints()
        {
            List<TriangleBendingConstraint> list = new List<TriangleBendingConstraint>(_bendingVerticesMap.Count);
            foreach (var pair in _bendingVerticesMap)
            {
                var vertices = pair.Value;
                if (vertices.Count == 4)
                {
                    list.Add(new TriangleBendingConstraint()
                    {
                        index0 = (ulong)vertices[0],
                        index1 = (ulong)vertices[1],
                        index2 = (ulong)vertices[2],
                        index3 = (ulong)vertices[3],
                        stiffnessInv = stiffnessInv,
                    });
                }
            }
            return list;
        }
    }
}
