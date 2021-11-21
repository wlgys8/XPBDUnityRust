using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System;
using UnityEngine;

namespace XPBD
{

    [StructLayout(LayoutKind.Sequential)]
    public struct VecRawParts
    {
        public IntPtr data;
        public uint length;
        public uint capacity;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct XPBDCreateOptions
    {
        public float dt;
        public int iterateCount;
        public VecRawParts distanceConstraints;
        public VecRawParts bendingConstraints;
        public VecRawParts positions;
        public VecRawParts masses;
        public float bounciness;
        public float dynamicFrictionFactor;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct InfinitePlane
    {
        public Vector3 normal;
        public float originToPlane;
    }


    public class NativeAPI
    {
        [DllImport("xpbd_simulation")]
        public static extern void create_array(uint byteLength, IntPtr out_raw_parts);
        [DllImport("xpbd_simulation")]
        public static extern IntPtr create_xpbd_solver(XPBDCreateOptions buildOptions);
        [DllImport("xpbd_simulation")]
        public static extern void update_xpbd_solver(IntPtr xpbdSolver);
        [DllImport("xpbd_simulation")]
        public static extern void destroy_xpbd_solver(IntPtr xpbdSolver);
        [DllImport("xpbd_simulation")]
        public static extern void add_field_force(IntPtr xpbdSolver, Vector3 force);
        [DllImport("xpbd_simulation")]
        public static extern void add_acceleration_field(IntPtr xpbdSolver, Vector3 acc);
        [DllImport("xpbd_simulation")]
        public static extern int get_particles_count(IntPtr xpbdSolver);
        [DllImport("xpbd_simulation")]
        public static extern System.IntPtr get_particles(IntPtr xpbdSolver);
        [DllImport("xpbd_simulation")]
        public static extern void attach_particle(IntPtr xpbdSolver, uint particleIndex, Vector3 position);
        [DllImport("xpbd_simulation")]
        public static extern bool detach_particle(IntPtr xpbdSolver, uint particleIndex);
        [DllImport("xpbd_simulation")]
        public static extern void get_position(IntPtr xpbdSolver, int index, System.IntPtr position);
        [DllImport("xpbd_simulation")]
        public static extern System.IntPtr get_collider_manager(IntPtr xpbdSolver);
        [DllImport("xpbd_simulation")]
        public static extern uint add_infinite_plane_collider(IntPtr colliderManager, InfinitePlane plane);
        [DllImport("xpbd_simulation")]
        public static extern bool remove_infinite_plane_collider(IntPtr colliderManager, uint id);
        [DllImport("xpbd_simulation")]
        public static extern bool clear_colliders(IntPtr colliderManager);
        [DllImport("xpbd_simulation")]
        public static extern void copy_positions(IntPtr xpbdSolver, IntPtr positionArrayPtr);

    }

    public class NativeVec<T> where T : unmanaged
    {
        private VecRawParts _parts;
        private static readonly int _elementSize = Marshal.SizeOf<T>();
        public NativeVec(int capacity)
        {
            VecRawParts rawParts;
            unsafe
            {
                VecRawParts* ptr = &rawParts;
                var byteLength = (uint)(_elementSize * capacity);
                NativeAPI.create_array(byteLength, (IntPtr)ptr);
            }
            _parts = rawParts;
        }

        internal VecRawParts rawParts
        {
            get
            {
                return _parts;
            }
        }

        public int Length
        {
            get
            {
                return (int)_parts.length / _elementSize;
            }
        }

        public int ByteLength
        {
            get
            {
                return (int)_parts.length;
            }
        }

        public int Capacity
        {
            get
            {
                return (int)_parts.capacity / _elementSize;
            }
        }

        public void Set(int index, T value)
        {
            unsafe
            {
                T* ptr = (T*)_parts.data;
                *(ptr + index) = value;
            }
        }

        public T Get(int index)
        {
            unsafe
            {
                T* ptr = (T*)_parts.data;
                return *(ptr + index);
            }
        }
    }
}
