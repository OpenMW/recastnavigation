#include <cstddef>
#include <cstdint>

#include "catch2/catch_all.hpp"

#include "Recast.h"

namespace
{
	int triangleWave(const int value, const int period)
	{
		const int phase = value % period;
		return rcMin(phase, period - phase);
	}

	bool isTerrainHole(const int x, const int z, const int width, const int height)
	{
		if (x < 2 || z < 2 || x >= width - 2 || z >= height - 2)
			return true;
		const int circleX = x % 48 - 24;
		const int circleZ = z % 48 - 24;
		if (circleX * circleX + circleZ * circleZ < 36)
			return true;
		return x % 71 >= 33 && x % 71 <= 36 && z % 89 < 57;
	}

	bool buildTerrainHeightfield(
		rcContext& context, rcHeightfield& heightfield, const int width, const int height, const int layers)
	{
		const float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
		const float boundsMax[3] = { (float)width, 512.0f, (float)height };
		if (!rcCreateHeightfield(
				&context, heightfield, width, height, boundsMin, boundsMax, 1.0f, 0.5f))
		{
			return false;
		}

		for (int z = 0; z < height; ++z)
		{
			for (int x = 0; x < width; ++x)
			{
				if (isTerrainHole(x, z, width, height))
					continue;
				const int terrain = 20 + triangleWave(x, 32) / 4 + triangleWave(z, 40) / 5;
				for (int layer = 0; layer < layers; ++layer)
				{
					const unsigned short spanMin = (unsigned short)(terrain + layer * 24);
					if (!rcAddSpan(&context, heightfield, x, z, spanMin,
							(unsigned short)(spanMin + 1), RC_WALKABLE_AREA, 1))
					{
						return false;
					}
				}
			}
		}
		return true;
	}

	bool buildCompactToPoly(rcContext& context, const rcHeightfield& heightfield,
		rcCompactHeightfield& compact, rcContourSet& contours, rcPolyMesh& mesh)
	{
		return rcBuildCompactHeightfield(&context, 4, 2, heightfield, compact)
			&& rcErodeWalkableArea(&context, 1, compact)
			&& rcBuildDistanceField(&context, compact)
			&& rcBuildRegions(&context, compact, 0, 8, 20)
			&& rcBuildContours(&context, compact, 1.3f, 12, contours)
			&& rcBuildPolyMesh(&context, contours, 6, mesh);
	}

	std::uint64_t hashBytes(std::uint64_t result, const void* data, const std::size_t size)
	{
		const unsigned char* bytes = (const unsigned char*)data;
		for (std::size_t i = 0; i < size; ++i)
		{
			result ^= bytes[i];
			result *= 1099511628211ull;
		}
		return result;
	}

	template <class T>
	std::uint64_t hashValues(const std::uint64_t result, const T* values, const std::size_t count)
	{
		return hashBytes(result, values, count * sizeof(T));
	}

	std::uint64_t compactPolyMeshChecksum(const rcCompactHeightfield& compact, const rcPolyMesh& mesh)
	{
		std::uint64_t result = 1469598103934665603ull;
		for (int i = 0; i < compact.spanCount; ++i)
		{
			const rcCompactSpan& span = compact.spans[i];
			result = hashValues(result, &span.y, 1);
			result = hashValues(result, &span.reg, 1);
			const unsigned int connections = span.con;
			result = hashValues(result, &connections, 1);
			const unsigned char height = (unsigned char)span.h;
			result = hashValues(result, &height, 1);
		}
		result = hashValues(result, compact.areas, (std::size_t)compact.spanCount);
		result = hashValues(result, compact.dist, (std::size_t)compact.spanCount);
		result = hashValues(result, mesh.verts, (std::size_t)mesh.nverts * 3);
		result = hashValues(result, mesh.polys, (std::size_t)mesh.npolys * mesh.nvp * 2);
		result = hashValues(result, mesh.regs, (std::size_t)mesh.npolys);
		result = hashValues(result, mesh.flags, (std::size_t)mesh.npolys);
		result = hashValues(result, mesh.areas, (std::size_t)mesh.npolys);
		return result;
	}

	std::uint64_t detailMeshChecksum(const rcPolyMeshDetail& detail)
	{
		std::uint64_t result = 1469598103934665603ull;
		result = hashValues(result, detail.meshes, (std::size_t)detail.nmeshes * 4);
		result = hashValues(result, detail.verts, (std::size_t)detail.nverts * 3);
		result = hashValues(result, detail.tris, (std::size_t)detail.ntris * 4);
		return result;
	}
}

TEST_CASE("Compact heightfield hot path benchmarks", "[.benchmark][recast]")
{
	{
		rcContext context(false);
		rcHeightfield multilayerHeightfield;
		REQUIRE(buildTerrainHeightfield(context, multilayerHeightfield, 112, 112, 3));
		{
			rcCompactHeightfield expectedCompact;
			rcContourSet expectedContours;
			rcPolyMesh expectedMesh;
			REQUIRE(buildCompactToPoly(
				context, multilayerHeightfield, expectedCompact, expectedContours, expectedMesh));
			REQUIRE(compactPolyMeshChecksum(expectedCompact, expectedMesh) != 0);
		}

		BENCHMARK("build compact-to-poly pipeline for 112x112 terrain with three layers")
		{
			rcContext benchmarkContext(false);
			rcCompactHeightfield compact;
			rcContourSet contours;
			rcPolyMesh mesh;
			if (!buildCompactToPoly(benchmarkContext, multilayerHeightfield, compact, contours, mesh))
				return std::uint64_t{ 0 };
			return compactPolyMeshChecksum(compact, mesh);
		};
	}

	rcContext context(false);
	rcHeightfield detailHeightfield;
	REQUIRE(buildTerrainHeightfield(context, detailHeightfield, 64, 64, 1));
	rcCompactHeightfield detailCompact;
	rcContourSet detailContours;
	rcPolyMesh detailMesh;
	REQUIRE(buildCompactToPoly(context, detailHeightfield, detailCompact, detailContours, detailMesh));
	rcPolyMeshDetail* expectedDetail = rcAllocPolyMeshDetail();
	REQUIRE(expectedDetail != NULL);
	const bool detailBuilt
		= rcBuildPolyMeshDetail(&context, detailMesh, detailCompact, 1.0f, 0.1f, *expectedDetail);
	const std::uint64_t expectedDetailChecksum = detailBuilt ? detailMeshChecksum(*expectedDetail) : 0;
	rcFreePolyMeshDetail(expectedDetail);
	REQUIRE(detailBuilt);
	REQUIRE(expectedDetailChecksum != 0);

	BENCHMARK("build detail mesh for 64x64 terrain")
	{
		rcContext benchmarkContext(false);
		rcPolyMeshDetail* detail = rcAllocPolyMeshDetail();
		if (detail == NULL)
			return std::uint64_t{ 0 };
		const bool built = rcBuildPolyMeshDetail(
			&benchmarkContext, detailMesh, detailCompact, 1.0f, 0.1f, *detail);
		const std::uint64_t checksum = built ? detailMeshChecksum(*detail) : 0;
		rcFreePolyMeshDetail(detail);
		return checksum;
	};
}
