// Copyright 2026 shoumexi-03
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System.Collections.Generic;
using UnityEngine;

namespace Awsim.DreamerV3
{
    /// <summary>
    /// Finds the ground below a point, so that a start pose or a goal given only as
    /// (x, y) of the map frame can be placed anywhere in the West Shinjuku map (whose height
    /// varies by several meters).
    /// </summary>
    public static class GroundProbe
    {
        /// <summary>Height above which the probe starts when no height hint is given [m].</summary>
        public const float ProbeTopWithoutHint = 500f;
        /// <summary>The probe starts this far above a height hint [m].</summary>
        public const float ProbeAboveHint = 1.5f;

        /// <summary>
        /// Height (Unity y) of the first non-trigger collider below (x, top, z) that does not belong to
        /// one of <paramref name="ignore"/>.
        /// </summary>
        public static bool TryFindGround(float x, float z, float top, IList<Transform> ignore, out float groundY)
        {
            var hits = Physics.RaycastAll(new Vector3(x, top, z), Vector3.down, top + 1000f,
                Physics.DefaultRaycastLayers, QueryTriggerInteraction.Ignore);
            System.Array.Sort(hits, (a, b) => a.distance.CompareTo(b.distance));
            foreach (var hit in hits)
            {
                if (IsUnder(hit.collider.transform, ignore)) continue;
                groundY = hit.point.y;
                return true;
            }
            groundY = 0f;
            return false;
        }

        /// <summary>Start height of the probe: above the hint (Unity y) if it is finite, else from high above.</summary>
        public static float ProbeTop(double hintY) =>
            double.IsNaN(hintY) || double.IsInfinity(hintY) ? ProbeTopWithoutHint : (float)hintY + ProbeAboveHint;

        static bool IsUnder(Transform t, IList<Transform> roots)
        {
            if (roots == null) return false;
            foreach (var root in roots)
                if (root != null && t.IsChildOf(root)) return true;
            return false;
        }
    }
}
