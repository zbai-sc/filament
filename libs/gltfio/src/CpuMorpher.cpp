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

#include "CpuMorpher.h"

#include <filament/BufferObject.h>
#include <filament/RenderableManager.h>
#include <filament/VertexBuffer.h>

#include "GltfEnums.h"
#include "TangentsJob.h"

using namespace filament;
using namespace filament::math;
using namespace utils;

static constexpr uint8_t kUnused = 0xff;

namespace gltfio {

uint32_t computeBindingSize(const cgltf_accessor* accessor);
uint32_t computeBindingOffset(const cgltf_accessor* accessor);

static const auto FREE_CALLBACK = [](void* mem, size_t, void*) { free(mem); };

static const uint8_t* cgltf_buffer_view_data(const cgltf_buffer_view* view) {
	if (view->data)
		return (const uint8_t*)view->data;

	if (!view->buffer->data)
		return NULL;

	const uint8_t* result = (const uint8_t*)view->buffer->data;
	result += view->offset;
	return result;
}

static cgltf_size cgltf_component_size(cgltf_component_type component_type) {
	switch (component_type)
	{
	case cgltf_component_type_r_8:
	case cgltf_component_type_r_8u:
		return 1;
	case cgltf_component_type_r_16:
	case cgltf_component_type_r_16u:
		return 2;
	case cgltf_component_type_r_32u:
	case cgltf_component_type_r_32f:
		return 4;
	case cgltf_component_type_invalid:
	default:
		return 0;
	}
}

static cgltf_size cgltf_component_read_index(const void* in, cgltf_component_type component_type) {
	switch (component_type)
	{
		case cgltf_component_type_r_16:
			return *((const int16_t*) in);
		case cgltf_component_type_r_16u:
			return *((const uint16_t*) in);
		case cgltf_component_type_r_32u:
			return *((const uint32_t*) in);
		case cgltf_component_type_r_32f:
			return (cgltf_size)*((const float*) in);
		case cgltf_component_type_r_8:
			return *((const int8_t*) in);
		case cgltf_component_type_r_8u:
			return *((const uint8_t*) in);
		default:
			return 0;
	}
}

static cgltf_float cgltf_component_read_float(const void* in, cgltf_component_type component_type, 
        cgltf_bool normalized) {
	if (component_type == cgltf_component_type_r_32f)
	{
		return *((const float*) in);
	}

	if (normalized)
	{
		switch (component_type)
		{
			// note: glTF spec doesn't currently define normalized conversions for 32-bit integers
			case cgltf_component_type_r_16:
				return *((const int16_t*) in) / (cgltf_float)32767;
			case cgltf_component_type_r_16u:
				return *((const uint16_t*) in) / (cgltf_float)65535;
			case cgltf_component_type_r_8:
				return *((const int8_t*) in) / (cgltf_float)127;
			case cgltf_component_type_r_8u:
				return *((const uint8_t*) in) / (cgltf_float)255;
			default:
				return 0;
		}
	}

	return (cgltf_float)cgltf_component_read_index(in, component_type);
}

static cgltf_bool cgltf_element_read_float(const uint8_t* element, cgltf_type type, 
        cgltf_component_type component_type, cgltf_bool normalized, cgltf_float* out, 
        cgltf_size element_size) {
	cgltf_size num_components = cgltf_num_components(type);

	if (element_size < num_components) {
		return 0;
	}

	// There are three special cases for component extraction, see #data-alignment in the 2.0 spec.

	cgltf_size component_size = cgltf_component_size(component_type);

	for (cgltf_size i = 0; i < num_components; ++i)
	{
		out[i] = cgltf_component_read_float(element + component_size * i, component_type, normalized);
	}
	return 1;
}

CpuMorpher::CpuMorpher(FFilamentAsset* asset) : mAsset(asset) {
    NodeMap& sourceNodes = asset->mNodeMap;
    for (auto pair : sourceNodes) {
        cgltf_node const* node = pair.first;
        cgltf_mesh const* mesh = node->mesh;
        if (mesh) {
            cgltf_primitive const* prims = mesh->primitives;
            for (cgltf_size pi = 0, count = mesh->primitives_count; pi < count; ++pi) {
                if (mesh->primitives[pi].targets_count > 0) {
                    addPrimitive(mesh, pi, &mMorphTable[pair.second]);
                }
            }
        }
    }
}

CpuMorpher::~CpuMorpher() {
    auto engine = mAsset->mEngine;
    for (auto& entry : mMorphTable) {
        for (auto& prim : entry.second.primitives) {
            engine->destroy(prim.bufferObject);
        }
    }
}

void CpuMorpher::applyWeights(Entity entity, float const* weights, size_t count) noexcept {
    auto& engine = *mAsset->mEngine;
    auto renderableManager = &engine.getRenderableManager();
    auto renderable = renderableManager->getInstance(entity);

    // If there are 4 or fewer targets, we can simply re-use the original VertexBuffer.
    if (count <= 4) {
        float4 vec{};
        for (size_t i = 0; i < count; i++) {
            vec[i] = weights[i];
        }
        renderableManager->setMorphWeights(renderable, vec);
        return;
    }

    for (auto& prim : mMorphTable[entity].primitives) {
        size_t size = prim.vertexCache.size() * 4;
        float* data = (float*) malloc(size);
        memcpy(data, prim.vertexCache.data(), size);

        for (size_t index = 0; index < count; index ++) {
            const float w = weights[index];
            if (w < 0.0001f) continue;

            const GltfTarget& target = prim.targets[index];
            cgltf_size floats_per_element = cgltf_num_components(target.type); 

            for (size_t i = 0; i < target.indices.size(); ++i) {
                uint16_t index = target.indices[i];
                for (size_t j = 0; j < floats_per_element; ++j) {
                    data[index * floats_per_element + j] += w * target.values[i * floats_per_element + j];
                }
            }
        }

        VertexBuffer* vb = prim.vertexBuffer;
        VertexBuffer::BufferDescriptor bd(data, size, FREE_CALLBACK);
        if (prim.bufferObject) {
            engine.destroy(prim.bufferObject);
        }
        prim.bufferObject = BufferObject::Builder().size(size).build(engine);
        prim.bufferObject->setBuffer(engine, std::move(bd));
        vb->setBufferObjectAt(engine, prim.baseSlot, prim.bufferObject);
    }
}

void CpuMorpher::addPrimitive(cgltf_mesh const* mesh, int primitiveIndex, TableEntry* entry) {
    auto& engine = *mAsset->mEngine;
    const cgltf_primitive& prim = mesh->primitives[primitiveIndex];
    VertexBuffer* vertexBuffer = mAsset->mMeshCache.at(mesh)[primitiveIndex].vertices;

    int slot = determineBaseSlot(prim);

    entry->primitives.push_back({ vertexBuffer, slot });

    cgltf_attribute& positionAttribute = prim.attributes[slot];
    std::vector<float>& vertexCache = entry->primitives.back().vertexCache;

    vertexCache.resize(positionAttribute.data->count * cgltf_num_components(positionAttribute.data->type));
    cgltf_accessor_unpack_floats(positionAttribute.data, vertexCache.data(), vertexCache.size());

    std::vector<GltfTarget>& targets = entry->primitives.back().targets;

    for (int targetIndex = 0; targetIndex < prim.targets_count; targetIndex++) {
        const cgltf_morph_target& morphTarget = prim.targets[targetIndex];
        for (cgltf_size aindex = 0; aindex < morphTarget.attributes_count; aindex++) {
            const cgltf_attribute& attribute = morphTarget.attributes[aindex];
            const cgltf_accessor* accessor = attribute.data;
            const cgltf_attribute_type atype = attribute.type;

            // only works for morphing of positions for now
            if (atype == cgltf_attribute_type_position) {
                targets.push_back({targetIndex, atype, accessor->type});

                // expect the morph target to use a sparse accessor
                if (accessor->is_sparse) {
                    const cgltf_accessor_sparse& sparse = accessor->sparse;
                    
                    const uint8_t* index_data = cgltf_buffer_view_data(sparse.indices_buffer_view);
		            const uint8_t* reader_head = cgltf_buffer_view_data(sparse.values_buffer_view);

                    index_data += sparse.indices_byte_offset;
                    reader_head += sparse.values_byte_offset;

                    cgltf_size index_stride = cgltf_component_size(sparse.indices_component_type);
                    // targets.push_back({targetIndex, atype});

                    cgltf_size floats_per_element = cgltf_num_components(accessor->type);
                    targets.back().indices.resize(sparse.count);
                    targets.back().values.resize(sparse.count * floats_per_element);

                    for (cgltf_size reader_index = 0; reader_index < sparse.count; reader_index++) {
                        targets.back().indices[reader_index] = cgltf_component_read_index(index_data, 
                            sparse.indices_component_type);
                        cgltf_element_read_float(reader_head, accessor->type, accessor->component_type, 
                            accessor->normalized, 
                            targets.back().values.data() + reader_index * floats_per_element, 
                            floats_per_element);
                        index_data += index_stride;
                        reader_head += accessor->stride;
                    } 
                }
            }
        }
    }
}

// trying to find the slot for the vertex position
int CpuMorpher::determineBaseSlot(const cgltf_primitive& prim) const {
    int slot = 0;
    bool hasNormals = false;
    for (cgltf_size aindex = 0; aindex < prim.attributes_count; aindex++) {
        const cgltf_attribute& attribute = prim.attributes[aindex];
        const cgltf_attribute_type atype = attribute.type;
        if (atype == cgltf_attribute_type_position) {
            break;
        }
        slot++;
    }

    return slot;
};

}  // namespace gltfio
