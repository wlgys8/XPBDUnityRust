using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XPBD
{
    public class ColliderManager
    {
        private System.IntPtr _rawPtr;
        internal ColliderManager(System.IntPtr ptr)
        {
            _rawPtr = ptr;
        }

        public uint AddInfinitePlane(InfinitePlane plane)
        {
            return NativeAPI.add_infinite_plane_collider(_rawPtr, plane);
        }

        public bool RemoveInfinitePlaneCollider(uint id)
        {
            return NativeAPI.remove_infinite_plane_collider(_rawPtr, id);
        }
    }
}
