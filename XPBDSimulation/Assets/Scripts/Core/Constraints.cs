using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;


[StructLayout(LayoutKind.Sequential)]
public struct DistanceConstraint
{
    public ulong index0;
    public ulong index1;
    public float stiffnessInv;
    private float _rest;

    public DistanceConstraint(ulong index0, ulong index1, float stiffnessInv)
    {
        this.index0 = index0;
        this.index1 = index1;
        this.stiffnessInv = stiffnessInv;
        _rest = 0;
    }
}

[StructLayout(LayoutKind.Sequential)]
public struct TriangleBendingConstraint
{
    public ulong index0;
    public ulong index1;
    public ulong index2;
    public ulong index3;
    public float stiffnessInv;
    private float _rest;
}
