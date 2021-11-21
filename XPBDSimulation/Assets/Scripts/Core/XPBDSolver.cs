using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using Unity.Collections;

namespace XPBD
{

    public class XPBDSolverBuilder
    {
        private NativeVec<DistanceConstraint> _distanceConstraints;
        private NativeVec<TriangleBendingConstraint> _triangleBendingConstraints;
        private NativeVec<Vector3> _positions;
        private NativeVec<float> _masses;

        public int iterate_count = 4;

        public float bounciness = 0.5f;
        public float dynamicFrictionFactor = 0.5f;


        private static NativeVec<T> CreateNativeVecFromNativeArray<T>(NativeArray<T> constraints) where T : unmanaged
        {
            var vec = new NativeVec<T>(constraints.Length);
            unsafe
            {
                var sourcePtr = Unity.Collections.LowLevel.Unsafe.NativeArrayUnsafeUtility.GetUnsafePtr(constraints);
                var destPtr = vec.rawParts.data;
                System.Buffer.MemoryCopy(sourcePtr, (void*)destPtr, vec.ByteLength, vec.ByteLength);
            }
            return vec;
        }

        public XPBDSolverBuilder FillDistanceConstraints(NativeArray<DistanceConstraint> constraints)
        {
            _distanceConstraints = CreateNativeVecFromNativeArray(constraints);
            return this;
        }

        public XPBDSolverBuilder FillTriangleBendingConstraints(NativeArray<TriangleBendingConstraint> constraints)
        {
            _triangleBendingConstraints = CreateNativeVecFromNativeArray(constraints);
            return this;
        }


        public XPBDSolverBuilder FillPositions(NativeArray<Vector3> positions)
        {
            _positions = CreateNativeVecFromNativeArray(positions);
            return this;
        }

        public XPBDSolverBuilder FillMasses(NativeArray<float> masses)
        {
            _masses = CreateNativeVecFromNativeArray(masses);
            return this;
        }

        public XPBDSolverBuilder SetIterateCount(int count)
        {
            this.iterate_count = count;
            return this;
        }

        public XPBDSolverBuilder SetBounciness(float bounciness)
        {
            this.bounciness = bounciness;
            return this;
        }

        public XPBDSolverBuilder SetDynamicFrictionFactor(float dynamicFrictionFactor)
        {
            this.dynamicFrictionFactor = dynamicFrictionFactor;
            return this;
        }

        public XPBDSolver Build()
        {
            var options = new XPBDCreateOptions()
            {
                dt = 0.005f,
                distanceConstraints = _distanceConstraints.rawParts,
                positions = _positions.rawParts,
                masses = _masses.rawParts,
                iterateCount = this.iterate_count,
                bounciness = this.bounciness,
                dynamicFrictionFactor = this.dynamicFrictionFactor,

            };
            if (_triangleBendingConstraints != null)
            {
                options.bendingConstraints = _triangleBendingConstraints.rawParts;
            }
            return new XPBDSolver(options);
        }
    }
    public class XPBDSolver
    {

        private System.IntPtr _ptr;
        private float _dt;
        private Dictionary<uint, Vector3> _attachments = new Dictionary<uint, Vector3>();
        private ColliderManager _colliderManager;
        internal XPBDSolver(XPBDCreateOptions options)
        {
            _ptr = NativeAPI.create_xpbd_solver(options);
            _dt = options.dt;
        }

        public Vector3 GetPosition(int index)
        {
            Vector3 position;
            unsafe
            {
                Vector3* ptr = &position;
                NativeAPI.get_position(_ptr, index, (System.IntPtr)ptr);
            }

            return position;
        }

        public void AddFieldForce(Vector3 force)
        {
            NativeAPI.add_field_force(_ptr, force);
        }

        public void AddFieldAcceleration(Vector3 acc)
        {
            NativeAPI.add_acceleration_field(_ptr, acc);
        }

        public int particleCount
        {
            get
            {
                return NativeAPI.get_particles_count(_ptr);
            }
        }

        public void AttachParticle(uint index, Vector3 position)
        {
            if (_attachments.ContainsKey(index))
            {
                _attachments[index] = position;
            }
            else
            {
                _attachments.Add(index, position);
            }

        }

        public bool DetachParticle(uint index)
        {
            _attachments.Remove(index);
            return NativeAPI.detach_particle(_ptr, index);
        }

        public ColliderManager colliderManager
        {
            get
            {
                if (_colliderManager == null)
                {
                    _colliderManager = new ColliderManager(NativeAPI.get_collider_manager(_ptr));
                }
                return _colliderManager;
            }
        }

        private float _time = 0;
        private int _updateCount = 0;

        public void Update()
        {
            _time += Time.deltaTime;
            while (_time >= this.dt)
            {
                foreach (var pair in _attachments)
                {
                    NativeAPI.attach_particle(_ptr, pair.Key, pair.Value);
                }
                _time -= this.dt;
                NativeAPI.update_xpbd_solver(_ptr);
                _updateCount++;
                // Debug.Log("udpate count = " + _updateCount);
            }
        }

        public float dt
        {
            get
            {
                return _dt;
            }
        }

        ~XPBDSolver()
        {
            if (_ptr != System.IntPtr.Zero)
            {
                NativeAPI.destroy_xpbd_solver(_ptr);
                _ptr = System.IntPtr.Zero;
            }
        }
    }
}
