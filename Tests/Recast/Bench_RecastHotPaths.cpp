#include <cstdint>
#include <vector>

#include "catch2/catch_all.hpp"

#include "Recast.h"

namespace
{
	struct SpanSpec
	{
		int x;
		int z;
		unsigned short min;
		unsigned short max;
	};

	std::uint32_t nextRandom(std::uint32_t& state)
	{
		state = state * 1664525u + 1013904223u;
		return state;
	}

	float randomFloat(std::uint32_t& state, const float min, const float max)
	{
		const float unit = (float)(nextRandom(state) >> 8) * (1.0f / 16777216.0f);
		return min + unit * (max - min);
	}

	std::vector<float> makeOverlappingTriangles(const int triangleCount)
	{
		std::uint32_t random = 0x5eed1234u;
		std::vector<float> result((std::size_t)triangleCount * 9);
		for (int triangle = 0; triangle < triangleCount; ++triangle)
		{
			for (int vertex = 0; vertex < 3; ++vertex)
			{
				const std::size_t offset = (std::size_t)(triangle * 3 + vertex) * 3;
				result[offset] = randomFloat(random, -4.0f, 68.0f);
				result[offset + 1] = randomFloat(random, -2.0f, 66.0f);
				result[offset + 2] = randomFloat(random, -4.0f, 68.0f);
			}
		}
		return result;
	}

	std::vector<float> makeSingleCellTriangles(const int triangleCount)
	{
		std::uint32_t random = 0x12345eedu;
		std::vector<float> result((std::size_t)triangleCount * 9);
		for (int triangle = 0; triangle < triangleCount; ++triangle)
		{
			const int x = (int)(nextRandom(random) % 64);
			const int z = (int)(nextRandom(random) % 64);
			for (int vertex = 0; vertex < 3; ++vertex)
			{
				const std::size_t offset = (std::size_t)(triangle * 3 + vertex) * 3;
				result[offset] = (float)x + randomFloat(random, 0.05f, 0.95f);
				result[offset + 1] = randomFloat(random, 0.0f, 64.0f);
				result[offset + 2] = (float)z + randomFloat(random, 0.05f, 0.95f);
			}
		}
		return result;
	}

	std::vector<SpanSpec> makeTallColumns(const int width, const int height, const int spansPerColumn)
	{
		std::vector<SpanSpec> result;
		result.reserve((std::size_t)width * height * spansPerColumn);
		for (int z = 0; z < height; ++z)
		{
			for (int x = 0; x < width; ++x)
			{
				unsigned short min = (unsigned short)((x + z) % 3);
				for (int i = 0; i < spansPerColumn; ++i)
				{
					const unsigned short max = (unsigned short)(min + 1 + (x + z + i) % 2);
					result.push_back(SpanSpec{ x, z, min, max });
					min = (unsigned short)(max + 2 + (x * 3 + z + i) % 4);
				}
			}
		}
		return result;
	}

	unsigned int spanChecksum(const rcHeightfield& heightfield)
	{
		unsigned int result = 0;
		for (int column = 0; column < heightfield.width * heightfield.height; ++column)
		{
			for (const rcSpan* span = heightfield.spans[column]; span != NULL; span = span->next)
				result = result * 33u + span->smin + span->smax + span->area;
		}
		return result;
	}
}

TEST_CASE("Recast hot path benchmarks", "[.benchmark][recast]")
{
	const float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
	const float boundsMax[3] = { 64.0f, 64.0f, 64.0f };
	const std::vector<unsigned char> overlappingAreas(1024, RC_WALKABLE_AREA);
	const std::vector<float> overlapping = makeOverlappingTriangles((int)overlappingAreas.size());
	const std::vector<unsigned char> singleCellAreas(4096, RC_WALKABLE_AREA);
	const std::vector<float> singleCell = makeSingleCellTriangles((int)singleCellAreas.size());

	BENCHMARK("rasterize 1024 overlapping triangles")
	{
		rcContext context(false);
		rcHeightfield heightfield;
		if (!rcCreateHeightfield(
				&context, heightfield, 64, 64, boundsMin, boundsMax, 1.0f, 0.25f))
			return 0u;
		const bool result = rcRasterizeTriangles(&context, overlapping.data(), overlappingAreas.data(),
			(int)overlappingAreas.size(), heightfield, 2);
		const unsigned int checksum = result ? spanChecksum(heightfield) : 0u;
		return checksum;
	};

	BENCHMARK("rasterize 4096 single-cell triangles")
	{
		rcContext context(false);
		rcHeightfield heightfield;
		if (!rcCreateHeightfield(
				&context, heightfield, 64, 64, boundsMin, boundsMax, 1.0f, 0.25f))
			return 0u;
		const bool result = rcRasterizeTriangles(&context, singleCell.data(), singleCellAreas.data(),
			(int)singleCellAreas.size(), heightfield, 2);
		const unsigned int checksum = result ? spanChecksum(heightfield) : 0u;
		return checksum;
	};

	const int filterWidth = 32;
	const int filterHeight = 32;
	const float filterBoundsMax[3] = { (float)filterWidth, 512.0f, (float)filterHeight };
	const std::vector<SpanSpec> tallColumns = makeTallColumns(filterWidth, filterHeight, 24);
	BENCHMARK("filter 32x32 columns with 24 spans")
	{
		rcContext context(false);
		rcHeightfield heightfield;
		if (!rcCreateHeightfield(&context, heightfield, filterWidth, filterHeight,
				boundsMin, filterBoundsMax, 1.0f, 1.0f))
			return 0u;
		for (const SpanSpec& span : tallColumns)
		{
			if (!rcAddSpan(&context, heightfield, span.x, span.z, span.min, span.max, RC_WALKABLE_AREA, 0))
			{
				return 0u;
			}
		}
		rcFilterLedgeSpans(&context, 4, 2, heightfield);
		const unsigned int checksum = spanChecksum(heightfield);
		return checksum;
	};
}
