/*
 * Copyright (C) 2021 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "FFilamentAsset.h"
#include "FFilamentInstance.h"

#include <math/vec4.h>

#include <tsl/robin_map.h>

#include <vector>

struct cgltf_node;
struct cgltf_mesh;
struct cgltf_primitive;

namespace gltfio {

/**
 * Helper for doing the morphing animation on CPU and re-upload the result mesh.
 *
 * This is mostly for supporting more than 4 morph targets. In comprison, the stock
 * MorphHelper merely pick the 4 highest weights and rearrange the targets.
 * 
 * Obviously doing the morphing on the CPU is much slower as we do need to upload
 * the vertex buffer every frame. So beware of the performance penalty.
 */
class CpuMorpher {
public:
    using Entity = utils::Entity;
    CpuMorpher(FFilamentAsset* asset);
    ~CpuMorpher();

    void applyWeights(Entity targetEntity, float const* weights, size_t count) noexcept;

private:
    struct GltfTarget {
        int morphTargetIndex;
        cgltf_attribute_type attribute_type;
        cgltf_type type;
        std::vector<uint16_t> indices;
        std::vector<float> values;
    };

    struct GltfPrimitive {
        filament::VertexBuffer* vertexBuffer;
        int baseSlot;
        std::vector<float> vertexCache;
        filament::BufferObject* bufferObject;
        std::vector<GltfTarget> targets;
    };

    struct TableEntry {
        std::vector<GltfPrimitive> primitives;
    };

    void addPrimitive(cgltf_mesh const* mesh, int primitiveIndex, TableEntry* entry);
    int determineBaseSlot(const cgltf_primitive& prim) const;

    std::vector<float> mPartiallySortedWeights;
    tsl::robin_map<Entity, TableEntry> mMorphTable;
    const FFilamentAsset* mAsset;
};

} // namespace gltfio
