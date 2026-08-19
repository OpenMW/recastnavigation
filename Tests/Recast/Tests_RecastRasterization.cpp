#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "catch2/catch_all.hpp"

#include "Recast.h"

namespace
{
	enum class TriangleOverload
	{
		Unindexed,
		IndexedInt,
		IndexedShort,
	};

	using SpanValue = std::array<unsigned int, 4>;

	std::vector<SpanValue> getSpans(const rcHeightfield& heightfield)
	{
		std::vector<SpanValue> result;
		for (int column = 0; column < heightfield.width * heightfield.height; ++column)
		{
			for (const rcSpan* span = heightfield.spans[column]; span != NULL; span = span->next)
			{
				result.push_back(SpanValue{ (unsigned int)column, (unsigned int)span->smin,
					(unsigned int)span->smax, (unsigned int)span->area });
			}
		}
		return result;
	}

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

	bool overlapBoundsReference(
		const float* aMin, const float* aMax, const float* bMin, const float* bMax)
	{
		return aMin[0] <= bMax[0] && aMax[0] >= bMin[0]
			&& aMin[1] <= bMax[1] && aMax[1] >= bMin[1]
			&& aMin[2] <= bMax[2] && aMax[2] >= bMin[2];
	}

	enum class Axis
	{
		X = 0,
		Z = 2,
	};

	// The generic polygon split is an independent oracle for specialized clipping.
	void dividePolyReference(const float* inVerts, const int inVertsCount,
		float* outVerts1, int& outVerts1Count, float* outVerts2, int& outVerts2Count,
		const float axisOffset, const Axis axis)
	{
		if (inVertsCount > 12)
			throw std::logic_error("reference clipping input exceeds 12 vertices");
		const int axisIndex = (int)axis;
		float inVertAxisDelta[12];
		for (int inVert = 0; inVert < inVertsCount; ++inVert)
			inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axisIndex];

		int poly1Vert = 0;
		int poly2Vert = 0;
		for (int inVertA = 0, inVertB = inVertsCount - 1;
			inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
		{
			const bool sameSide
				= (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);
			if (!sameSide)
			{
				const float s = inVertAxisDelta[inVertB]
					/ (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
				for (int component = 0; component < 3; ++component)
				{
					outVerts1[poly1Vert * 3 + component] = inVerts[inVertB * 3 + component]
						+ (inVerts[inVertA * 3 + component] - inVerts[inVertB * 3 + component]) * s;
				}
				rcVcopy(&outVerts2[poly2Vert * 3], &outVerts1[poly1Vert * 3]);
				++poly1Vert;
				++poly2Vert;

				if (inVertAxisDelta[inVertA] > 0)
				{
					rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
					++poly1Vert;
				}
				else if (inVertAxisDelta[inVertA] < 0)
				{
					rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
					++poly2Vert;
				}
			}
			else
			{
				if (inVertAxisDelta[inVertA] >= 0)
				{
					rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
					++poly1Vert;
					if (inVertAxisDelta[inVertA] != 0)
						continue;
				}
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				++poly2Vert;
			}
		}

		outVerts1Count = poly1Vert;
		outVerts2Count = poly2Vert;
	}

	bool rasterizeTriangleReference(rcContext& context, const float* v0, const float* v1, const float* v2,
		const unsigned char area, rcHeightfield& heightfield, const int mergeThreshold)
	{
		float triangleMin[3];
		rcVcopy(triangleMin, v0);
		rcVmin(triangleMin, v1);
		rcVmin(triangleMin, v2);
		float triangleMax[3];
		rcVcopy(triangleMax, v0);
		rcVmax(triangleMax, v1);
		rcVmax(triangleMax, v2);
		if (!overlapBoundsReference(triangleMin, triangleMax, heightfield.bmin, heightfield.bmax))
			return true;

		const int width = heightfield.width;
		const int height = heightfield.height;
		const float inverseCellSize = 1.0f / heightfield.cs;
		const float inverseCellHeight = 1.0f / heightfield.ch;
		const float boundsHeight = heightfield.bmax[1] - heightfield.bmin[1];
		int z0 = (int)((triangleMin[2] - heightfield.bmin[2]) * inverseCellSize);
		int z1 = (int)((triangleMax[2] - heightfield.bmin[2]) * inverseCellSize);
		z0 = rcClamp(z0, -1, height - 1);
		z1 = rcClamp(z1, 0, height - 1);

		float buffer[7 * 3 * 4];
		float* in = buffer;
		float* inRow = buffer + 7 * 3;
		float* polygon1 = inRow + 7 * 3;
		float* polygon2 = polygon1 + 7 * 3;
		rcVcopy(&in[0], v0);
		rcVcopy(&in[3], v1);
		rcVcopy(&in[6], v2);
		int inCount = 3;

		for (int z = z0; z <= z1; ++z)
		{
			int rowCount = 0;
			const float cellZ = heightfield.bmin[2] + (float)z * heightfield.cs;
			dividePolyReference(in, inCount, inRow, rowCount, polygon1, inCount,
				cellZ + heightfield.cs, Axis::Z);
			rcSwap(in, polygon1);
			if (rowCount < 3 || z < 0)
				continue;

			float minX = inRow[0];
			float maxX = inRow[0];
			for (int vertex = 1; vertex < rowCount; ++vertex)
			{
				minX = rcMin(minX, inRow[vertex * 3]);
				maxX = rcMax(maxX, inRow[vertex * 3]);
			}
			int x0 = (int)((minX - heightfield.bmin[0]) * inverseCellSize);
			int x1 = (int)((maxX - heightfield.bmin[0]) * inverseCellSize);
			if (x1 < 0 || x0 >= width)
				continue;
			x0 = rcClamp(x0, -1, width - 1);
			x1 = rcClamp(x1, 0, width - 1);

			int remainderCount = rowCount;
			for (int x = x0; x <= x1; ++x)
			{
				int cellCount = 0;
				const float cellX = heightfield.bmin[0] + (float)x * heightfield.cs;
				dividePolyReference(inRow, remainderCount, polygon1, cellCount, polygon2, remainderCount,
					cellX + heightfield.cs, Axis::X);
				rcSwap(inRow, polygon2);
				if (cellCount < 3 || x < 0)
					continue;

				float spanMin = polygon1[1];
				float spanMax = polygon1[1];
				for (int vertex = 1; vertex < cellCount; ++vertex)
				{
					spanMin = rcMin(spanMin, polygon1[vertex * 3 + 1]);
					spanMax = rcMax(spanMax, polygon1[vertex * 3 + 1]);
				}
				spanMin -= heightfield.bmin[1];
				spanMax -= heightfield.bmin[1];
				if (spanMax < 0.0f || spanMin > boundsHeight)
					continue;
				spanMin = rcMax(spanMin, 0.0f);
				spanMax = rcMin(spanMax, boundsHeight);
				const unsigned short minCell = (unsigned short)rcClamp(
					(int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
				const unsigned short maxCell = (unsigned short)rcClamp(
					(int)ceilf(spanMax * inverseCellHeight), (int)minCell + 1, RC_SPAN_MAX_HEIGHT);
				if (!rcAddSpan(&context, heightfield, x, z, minCell, maxCell, area, mergeThreshold))
					return false;
			}
		}
		return true;
	}

	bool rasterizeTriangles(rcContext& context, const TriangleOverload overload,
		const std::vector<float>& vertices, const std::vector<unsigned char>& areas,
		rcHeightfield& heightfield, const int mergeThreshold)
	{
		const int triangleCount = (int)areas.size();
		if (overload == TriangleOverload::Unindexed)
		{
			return rcRasterizeTriangles(
				&context, vertices.data(), areas.data(), triangleCount, heightfield, mergeThreshold);
		}

		if (overload == TriangleOverload::IndexedInt)
		{
			std::vector<int> indices((std::size_t)triangleCount * 3);
			for (std::size_t i = 0; i < indices.size(); ++i)
				indices[i] = (int)i;
			return rcRasterizeTriangles(&context, vertices.data(), (int)indices.size(), indices.data(),
				areas.data(), triangleCount, heightfield, mergeThreshold);
		}

		std::vector<unsigned short> indices((std::size_t)triangleCount * 3);
		for (std::size_t i = 0; i < indices.size(); ++i)
			indices[i] = (unsigned short)i;
		return rcRasterizeTriangles(&context, vertices.data(), (int)indices.size(), indices.data(),
			areas.data(), triangleCount, heightfield, mergeThreshold);
	}
}

TEST_CASE("Triangle rasterization matches reference clipping and incremental insertion", "[recast, rasterization]")
{
	const float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
	const float boundsMax[3] = { 32.0f, 64.0f, 32.0f };
	const int triangleCounts[] = { 1, 63, 64, 65, 192 };

	for (int overloadIndex = 0; overloadIndex < 3; ++overloadIndex)
	{
		for (const int triangleCount : triangleCounts)
		{
			for (std::uint32_t seed = 1; seed <= 4; ++seed)
			{
				const TriangleOverload overload = (TriangleOverload)overloadIndex;
				const int mergeThreshold = (int)((seed + triangleCount + overloadIndex) % 5);
				std::uint32_t random
					= seed * 7919u + (std::uint32_t)triangleCount * 104729u + (std::uint32_t)overloadIndex;
				std::vector<float> vertices((std::size_t)triangleCount * 9);
				std::vector<unsigned char> areas((std::size_t)triangleCount);
				for (int triangle = 0; triangle < triangleCount; ++triangle)
				{
					for (int vertex = 0; vertex < 3; ++vertex)
					{
						const std::size_t offset = (std::size_t)(triangle * 3 + vertex) * 3;
						vertices[offset] = randomFloat(random, -4.0f, 36.0f);
						vertices[offset + 1] = randomFloat(random, -2.0f, 66.0f);
						vertices[offset + 2] = randomFloat(random, -4.0f, 36.0f);
					}
					areas[triangle] = (unsigned char)(nextRandom(random) % 64);
				}

				rcContext context;
				rcHeightfield optimized;
				rcHeightfield reference;
				CAPTURE(overloadIndex, seed, triangleCount, mergeThreshold);
				REQUIRE(rcCreateHeightfield(
					&context, optimized, 32, 32, boundsMin, boundsMax, 1.0f, 0.25f));
				REQUIRE(rcCreateHeightfield(
					&context, reference, 32, 32, boundsMin, boundsMax, 1.0f, 0.25f));

				REQUIRE(rasterizeTriangles(context, overload, vertices, areas, optimized, mergeThreshold));
				for (int triangle = 0; triangle < triangleCount; ++triangle)
				{
					REQUIRE(rasterizeTriangleReference(context, &vertices[(std::size_t)triangle * 9],
						&vertices[(std::size_t)triangle * 9 + 3], &vertices[(std::size_t)triangle * 9 + 6],
						areas[triangle], reference, mergeThreshold));
				}

				CHECK_THAT(getSpans(optimized), Catch::Matchers::Equals(getSpans(reference)));
			}
		}
	}
}

TEST_CASE("Bulk rasterization preserves a non-empty heightfield", "[recast, rasterization]")
{
	const float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
	const float boundsMax[3] = { 8.0f, 32.0f, 8.0f };
	const int triangleCount = 64;
	std::uint32_t random = 0x51a7e123u;
	std::vector<float> vertices((std::size_t)triangleCount * 9);
	std::vector<unsigned char> areas((std::size_t)triangleCount);
	for (int triangle = 0; triangle < triangleCount; ++triangle)
	{
		for (int vertex = 0; vertex < 3; ++vertex)
		{
			const std::size_t offset = (std::size_t)(triangle * 3 + vertex) * 3;
			vertices[offset] = randomFloat(random, -1.0f, 9.0f);
			vertices[offset + 1] = randomFloat(random, -1.0f, 33.0f);
			vertices[offset + 2] = randomFloat(random, -1.0f, 9.0f);
		}
		areas[triangle] = (unsigned char)(nextRandom(random) % 64);
	}

	rcContext context;
	rcHeightfield optimized;
	rcHeightfield reference;
	REQUIRE(rcCreateHeightfield(&context, optimized, 8, 8, boundsMin, boundsMax, 1.0f, 0.25f));
	REQUIRE(rcCreateHeightfield(&context, reference, 8, 8, boundsMin, boundsMax, 1.0f, 0.25f));
	REQUIRE(rcAddSpan(&context, optimized, 3, 3, 1, 3, 7, 2));
	REQUIRE(rcAddSpan(&context, reference, 3, 3, 1, 3, 7, 2));
	REQUIRE(rcRasterizeTriangles(
		&context, vertices.data(), areas.data(), triangleCount, optimized, 2));
	for (int triangle = 0; triangle < triangleCount; ++triangle)
	{
		REQUIRE(rasterizeTriangleReference(context, &vertices[(std::size_t)triangle * 9],
			&vertices[(std::size_t)triangle * 9 + 3], &vertices[(std::size_t)triangle * 9 + 6],
			areas[triangle], reference, 2));
	}
	CHECK_THAT(getSpans(optimized), Catch::Matchers::Equals(getSpans(reference)));
}

TEST_CASE("Specialized clipping matches generic clipping at grid and heightfield boundaries",
	"[recast, rasterization]")
{
	const float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
	const float boundsMax[3] = { 4.0f, 8.0f, 4.0f };
	const std::array<std::array<float, 9>, 5> triangles = { {
		{ 0.0f, 1.0f, 0.0f, 1.0f, 2.0f, 0.0f, 0.0f, 3.0f, 1.0f },
		{ -1.0f, -1.0f, 0.5f, 2.0f, 4.0f, 0.5f, 0.5f, 9.0f, 2.0f },
		{ 3.5f, 1.0f, 3.5f, 4.5f, 2.0f, 3.5f, 3.5f, 3.0f, 4.5f },
		{ 1.0f, 1.0f, 1.0f, 2.0f, 2.0f, 1.0f, 1.0f, 3.0f, 2.0f },
		{ 1.2f, 1.0f, 1.2f, 2.8f, 2.0f, 1.7f, 1.7f, 3.0f, 2.8f },
	} };

	for (std::size_t caseIndex = 0; caseIndex < triangles.size(); ++caseIndex)
	{
		rcContext context;
		rcHeightfield optimized;
		rcHeightfield reference;
		CAPTURE(caseIndex);
		REQUIRE(rcCreateHeightfield(&context, optimized, 4, 4, boundsMin, boundsMax, 1.0f, 0.5f));
		REQUIRE(rcCreateHeightfield(&context, reference, 4, 4, boundsMin, boundsMax, 1.0f, 0.5f));
		const std::array<float, 9>& triangle = triangles[caseIndex];
		REQUIRE(rcRasterizeTriangle(
			&context, &triangle[0], &triangle[3], &triangle[6], 42, optimized, 1));
		REQUIRE(rasterizeTriangleReference(
			context, &triangle[0], &triangle[3], &triangle[6], 42, reference, 1));
		CHECK_THAT(getSpans(optimized), Catch::Matchers::Equals(getSpans(reference)));
	}
}

TEST_CASE("Rasterizing a triangle contained by one voxel produces its height span", "[recast, rasterization]")
{
	rcContext context;
	const float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
	const float boundsMax[3] = { 4.0f, 8.0f, 4.0f };
	rcHeightfield heightfield;
	REQUIRE(rcCreateHeightfield(&context, heightfield, 4, 4, boundsMin, boundsMax, 1.0f, 0.5f));

	const float vertices[9] = {
		1.1f, 1.1f, 2.1f,
		1.8f, 2.4f, 2.2f,
		1.3f, 1.6f, 2.9f,
	};
	REQUIRE(rcRasterizeTriangle(&context, &vertices[0], &vertices[3], &vertices[6], 42, heightfield, 1));

	const std::vector<SpanValue> expected = { SpanValue{ 9, 2, 5, 42 } };
	CHECK_THAT(getSpans(heightfield), Catch::Matchers::Equals(expected));
}
