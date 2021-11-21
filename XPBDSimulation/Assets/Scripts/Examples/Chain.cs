using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using XPBD;

public class Chain : MonoBehaviour
{

    [SerializeField]
    private int _nodeCount = 10;

    [SerializeField]
    [Range(0, 0.1f)]
    private float _flexibility = 0;

    [SerializeField]
    [Range(1, 12)]
    private int _iterateCount = 4;

    [SerializeField]
    [Range(0.1f, 5)]
    private float _massPerNode = 1;

    private XPBDSolver _solver;

    [SerializeField]
    private bool _verticle = false;

    private void CreateSolver()
    {
        int chainNodeCount = _nodeCount;
        var positions = new NativeArray<Vector3>(chainNodeCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        var distances = new NativeArray<DistanceConstraint>(chainNodeCount - 1, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
        var masses = new NativeArray<float>(chainNodeCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

        var rootPosition = this.transform.position;

        var deltaPosition = _verticle ? new Vector3(0, -1, 0) : new Vector3(1, 0, 0);

        for (var i = 0; i < chainNodeCount; i++)
        {
            positions[i] = rootPosition + deltaPosition * i;
            masses[i] = _massPerNode;
            if (i < chainNodeCount - 1)
            {
                distances[i] = new DistanceConstraint((ulong)i, (ulong)(i + 1), _flexibility);
            }
        }
        _solver = new XPBDSolverBuilder()
        .FillPositions(positions)
        .FillMasses(masses)
        .FillDistanceConstraints(distances)
        .SetIterateCount(_iterateCount)
        .Build();
        _solver.AddFieldAcceleration(Vector3.down * 9.8f);

        distances.Dispose();
        positions.Dispose();
        masses.Dispose();
    }

    void Start()
    {
        this.CreateSolver();
    }

    // Update is called once per frame
    void Update()
    {
        _solver.AttachParticle((uint)0, this.transform.position);
        _solver.Update();
    }

    void OnDrawGizmos()
    {
        if (_solver == null)
        {
            return;
        }
        for (var i = 0; i < _solver.particleCount; i++)
        {
            var pos = _solver.GetPosition(i);
            Gizmos.DrawSphere(pos, 0.1f);
            if (i < _solver.particleCount - 1)
            {
                Gizmos.DrawLine(pos, _solver.GetPosition(i + 1));
            }
        }
    }
}
