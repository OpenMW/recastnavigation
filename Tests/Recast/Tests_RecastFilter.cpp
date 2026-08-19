#include <stdio.h>
#include <string.h>
#include <array>
#include <cstdint>
#include <vector>

#include "catch2/catch_all.hpp"

#include "Recast.h"
#include "RecastAlloc.h"

namespace
{
	void freeHeightfieldSpans(const rcHeightfield& heightfield)
	{
		for (int i = 0; i < heightfield.height * heightfield.width; ++i)
		{
			rcSpan* span = heightfield.spans[i];
			while (span != NULL)
			{
				rcSpan* next = span->next;
				rcFree(span);
				span = next;
			}
		}
	}

	std::vector<unsigned> getAreas(const rcHeightfield& heightfield)
	{
		std::vector<unsigned> result(heightfield.height * heightfield.width);
		for (int i = 0; i < heightfield.height * heightfield.width; ++i)
			if (const rcSpan* const span = heightfield.spans[i])
				result[i] = span->area;
		return result;
	}

	using SpanValue = std::array<unsigned int, 4>;

	std::vector<SpanValue> getSpans(const rcHeightfield& heightfield)
	{
		std::vector<SpanValue> result;
		for (int column = 0; column < heightfield.height * heightfield.width; ++column)
		{
			for (const rcSpan* span = heightfield.spans[column]; span != NULL; span = span->next)
			{
				result.push_back(SpanValue{ (unsigned int)column, (unsigned int)span->smin,
					(unsigned int)span->smax, (unsigned int)span->area });
			}
		}
		return result;
	}

	// The linked-list implementation is an independent oracle for the indexed scan.
	void filterLedgeSpansReference(
		const int walkableHeight, const int walkableClimb, rcHeightfield& heightfield)
	{
		const int maxHeight = 0xffff;
		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				for (rcSpan* span = heightfield.spans[x + z * heightfield.width]; span; span = span->next)
				{
					if (span->area == RC_NULL_AREA)
						continue;

					const int floor = (int)span->smax;
					const int ceiling = span->next ? (int)span->next->smin : maxHeight;
					int lowestNeighborFloorDifference = maxHeight;
					int lowestTraversableNeighborFloor = span->smax;
					int highestTraversableNeighborFloor = span->smax;

					for (int direction = 0; direction < 4; ++direction)
					{
						const int neighborX = x + rcGetDirOffsetX(direction);
						const int neighborZ = z + rcGetDirOffsetY(direction);
						if (neighborX < 0 || neighborZ < 0
							|| neighborX >= heightfield.width || neighborZ >= heightfield.height)
						{
							lowestNeighborFloorDifference = -walkableClimb - 1;
							break;
						}

						const rcSpan* neighborSpan
							= heightfield.spans[neighborX + neighborZ * heightfield.width];
						int neighborCeiling = neighborSpan ? (int)neighborSpan->smin : maxHeight;
						if (rcMin(ceiling, neighborCeiling) - floor > walkableHeight)
						{
							lowestNeighborFloorDifference = -walkableClimb - 1;
							break;
						}

						for (; neighborSpan != NULL; neighborSpan = neighborSpan->next)
						{
							const int neighborFloor = (int)neighborSpan->smax;
							neighborCeiling
								= neighborSpan->next ? (int)neighborSpan->next->smin : maxHeight;
							if (rcMin(ceiling, neighborCeiling) - rcMax(floor, neighborFloor)
								<= walkableHeight)
							{
								continue;
							}

							const int difference = neighborFloor - floor;
							lowestNeighborFloorDifference
								= rcMin(lowestNeighborFloorDifference, difference);
							if (rcAbs(difference) <= walkableClimb)
							{
								lowestTraversableNeighborFloor
									= rcMin(lowestTraversableNeighborFloor, neighborFloor);
								highestTraversableNeighborFloor
									= rcMax(highestTraversableNeighborFloor, neighborFloor);
							}
							else if (difference < -walkableClimb)
							{
								break;
							}
						}
					}

					if (lowestNeighborFloorDifference < -walkableClimb
						|| highestTraversableNeighborFloor - lowestTraversableNeighborFloor > walkableClimb)
					{
						span->area = RC_NULL_AREA;
					}
				}
			}
		}
	}

	std::uint32_t nextRandom(std::uint32_t& state)
	{
		state = state * 1664525u + 1013904223u;
		return state;
	}
}

TEST_CASE("rcFilterLowHangingWalkableObstacles", "[recast, filtering]")
{
	rcContext context;
	int walkableHeight = 5;

	rcHeightfield heightfield;
	heightfield.width = 1;
	heightfield.height = 1;
	heightfield.bmin[0] = 0;
	heightfield.bmin[1] = 0;
	heightfield.bmin[2] = 0;
	heightfield.bmax[0] = 1;
	heightfield.bmax[1] = 1;
	heightfield.bmax[2] = 1;
	heightfield.cs = 1;
	heightfield.ch = 1;
	heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("Span with no spans above it is unchanged")
	{
		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);

		rcFree(span);
	}

	SECTION("Span with span above that is higher than walkableHeight is unchanged")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 1 + walkableHeight;
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that nothing has changed.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		// Check again but with a more clearance
		secondSpan->smin += 10;
		secondSpan->smax += 10;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that nothing has changed.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Marks low obstacles walkable if they're below the walkableClimb")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 1 + (walkableHeight - 1);
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that the second span was changed to walkable.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == 1);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Low obstacle that overlaps the walkableClimb distance is not changed")
	{
		// Put the second span just above the first one.
		rcSpan* secondSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		secondSpan->area = RC_NULL_AREA;
		secondSpan->next = NULL;
		secondSpan->smin = 2 + (walkableHeight - 1);
		secondSpan->smax = secondSpan->smin + 1;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = secondSpan;
		span->smin = 0;
		span->smax = 1;

		heightfield.spans[0] = span;

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		// Check that the second span was changed to walkable.
		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(span);
		rcFree(secondSpan);
	}

	SECTION("Only the first of multiple, low obstacles are marked walkable")
	{
		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcSpan* previousSpan = span;
		for (int i = 0; i < 9; ++i)
		{
			rcSpan* nextSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
			nextSpan->area = RC_NULL_AREA;
			nextSpan->next = NULL;
			nextSpan->smin = previousSpan->smax + (walkableHeight - 1);
			nextSpan->smax = nextSpan->smin + 1;
			previousSpan->next = nextSpan;
			previousSpan = nextSpan;
		}

		rcFilterLowHangingWalkableObstacles(&context, walkableHeight, heightfield);

		rcSpan* currentSpan = heightfield.spans[0];
		for (int i = 0; i < 10; ++i)
		{
			REQUIRE(currentSpan != NULL);
			// only the first and second spans should be marked as walkabl
			REQUIRE(currentSpan->area == (i <= 1 ? 1 : RC_NULL_AREA));
			currentSpan = currentSpan->next;
		}

		std::vector<rcSpan*> toFree;
		span = heightfield.spans[0];
		for (int i = 0; i < 10; ++i)
		{
			toFree.push_back(span);
			span = span->next;
		}

		for (int i = 0; i < 10; ++i)
		{
			rcFree(toFree[i]);
		}
	}
}

TEST_CASE("rcFilterLedgeSpans", "[recast, filtering]")
{
	rcContext context;
	int walkableClimb = 5;
	int walkableHeight = 10;

	rcHeightfield heightfield;
	heightfield.width = 10;
	heightfield.height = 10;
	heightfield.bmin[0] = 0;
	heightfield.bmin[1] = 0;
	heightfield.bmin[2] = 0;
	heightfield.bmax[0] = 10;
	heightfield.bmax[1] = 1;
	heightfield.bmax[2] = 10;
	heightfield.cs = 1;
	heightfield.ch = 1;
	heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("Edge spans are marked unwalkable")
	{
		// Create a flat plane.
		for (int x = 0; x < heightfield.width; ++x)
		{
			for (int z = 0; z < heightfield.height; ++z)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->next = NULL;
				span->smin = 0;
				span->smax = 1;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		for (int x = 0; x < heightfield.width; ++x)
		{
			for (int z = 0; z < heightfield.height; ++z)
			{
				rcSpan* span = heightfield.spans[x + z * heightfield.width];
				REQUIRE(span != NULL);

				if (x == 0 || z == 0 || x == 9 || z == 9)
				{
					REQUIRE(span->area == RC_NULL_AREA);
				}
				else
				{
					REQUIRE(span->area == 1);
				}

				REQUIRE(span->next == NULL);
				REQUIRE(span->smin == 0);
				REQUIRE(span->smax == 1);
			}
		}

		freeHeightfieldSpans(heightfield);
	}

	SECTION("Random sample")
	{
		heightfield.width = 5;
		heightfield.height = 5;

		const unsigned smin[] = {
			0,  0,  0,  0,  0,
			0,  0, 11,  0,  0,
			0,  6,  0, 10,  0,
			0,  0, 11,  0,  0,
			0,  0,  0,  0,  0,
		};

		const unsigned smax[] = {
			1,  1,  1,  1,  1,
			1,  1, 12,  1,  1,
			1,  7,  1, 11,  1,
			1,  1, 12,  1,  1,
			1,  1,  1,  1,  1,
		};

		for (int z = 0; z < heightfield.height; ++z)
		{
			for (int x = 0; x < heightfield.width; ++x)
			{
				rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
				span->area = 1;
				span->smin = smin[x + z * heightfield.width];
				span->smax = smax[x + z * heightfield.width];
				span->next = NULL;
				heightfield.spans[x + z * heightfield.width] = span;
			}
		}

		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, heightfield);

		const std::vector<unsigned> expectedAreas = {
			0, 0, 0, 0, 0,
			0, 1, 0, 1, 0,
			0, 0, 1, 0, 0,
			0, 1, 0, 1, 0,
			0, 0, 0, 0, 0,
		};

		CHECK_THAT(getAreas(heightfield), Catch::Matchers::Equals(expectedAreas));

		freeHeightfieldSpans(heightfield);
	}
}

TEST_CASE("rcFilterLedgeSpans matches the reference scan for multi-span columns", "[recast, filtering]")
{
	const int width = 12;
	const int height = 12;
	const float boundsMin[3] = { 0.0f, 0.0f, 0.0f };
	const float boundsMax[3] = { (float)width, 256.0f, (float)height };
	rcContext context;

	for (std::uint32_t seed = 1; seed <= 32; ++seed)
	{
		rcHeightfield reference;
		rcHeightfield optimized;
		CAPTURE(seed);
		REQUIRE(rcCreateHeightfield(
			&context, reference, width, height, boundsMin, boundsMax, 1.0f, 1.0f));
		REQUIRE(rcCreateHeightfield(
			&context, optimized, width, height, boundsMin, boundsMax, 1.0f, 1.0f));

		std::uint32_t random = seed * 7919u;
		for (int z = 0; z < height; ++z)
		{
			for (int x = 0; x < width; ++x)
			{
				unsigned short spanMin = (unsigned short)(nextRandom(random) % 4);
				const int spanCount = 1 + (int)(nextRandom(random) % 8);
				for (int spanIndex = 0; spanIndex < spanCount; ++spanIndex)
				{
					const unsigned short spanMax
						= (unsigned short)(spanMin + 1 + nextRandom(random) % 3);
					const unsigned char area
						= nextRandom(random) % 5 == 0 ? RC_NULL_AREA : (unsigned char)1;
					REQUIRE(rcAddSpan(&context, reference, x, z, spanMin, spanMax, area, 0));
					REQUIRE(rcAddSpan(&context, optimized, x, z, spanMin, spanMax, area, 0));
					spanMin = (unsigned short)(spanMax + 1 + nextRandom(random) % 6);
				}
			}
		}

		const int walkableHeight = 2 + (int)(seed % 5);
		const int walkableClimb = 1 + (int)(seed % 4);
		filterLedgeSpansReference(walkableHeight, walkableClimb, reference);
		rcFilterLedgeSpans(&context, walkableHeight, walkableClimb, optimized);
		CHECK_THAT(getSpans(optimized), Catch::Matchers::Equals(getSpans(reference)));
	}
}

TEST_CASE("rcFilterWalkableLowHeightSpans", "[recast, filtering]")
{
	rcContext context;
	int walkableHeight = 5;

	rcHeightfield heightfield;
	heightfield.width = 1;
	heightfield.height = 1;
	heightfield.bmin[0] = 0;
	heightfield.bmin[1] = 0;
	heightfield.bmin[2] = 0;
	heightfield.bmax[0] = 1;
	heightfield.bmax[1] = 1;
	heightfield.bmax[2] = 1;
	heightfield.cs = 1;
	heightfield.ch = 1;
	heightfield.spans = (rcSpan**)rcAlloc(heightfield.width * heightfield.height * sizeof(rcSpan*), RC_ALLOC_PERM);
	heightfield.pools = NULL;
	heightfield.freelist = NULL;

	SECTION("span nothing above is unchanged")
	{
		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = NULL;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);

		rcFree(span);
	}

	SECTION("span with lots of room above is unchanged")
	{
		rcSpan* overheadSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		overheadSpan->area = RC_NULL_AREA;
		overheadSpan->next = NULL;
		overheadSpan->smin = 10;
		overheadSpan->smax = 11;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = overheadSpan;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == 1);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(overheadSpan);
		rcFree(span);
	}

	SECTION("Span with low hanging obstacle is marked as unwalkable")
	{
		rcSpan* overheadSpan = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		overheadSpan->area = RC_NULL_AREA;
		overheadSpan->next = NULL;
		overheadSpan->smin = 3;
		overheadSpan->smax = 4;

		rcSpan* span = (rcSpan*)rcAlloc(sizeof(rcSpan), RC_ALLOC_PERM);
		span->area = 1;
		span->next = overheadSpan;
		span->smin = 0;
		span->smax = 1;
		heightfield.spans[0] = span;

		rcFilterWalkableLowHeightSpans(&context, walkableHeight, heightfield);

		REQUIRE(heightfield.spans[0]->area == RC_NULL_AREA);
		REQUIRE(heightfield.spans[0]->next->area == RC_NULL_AREA);

		rcFree(overheadSpan);
		rcFree(span);
	}
}
