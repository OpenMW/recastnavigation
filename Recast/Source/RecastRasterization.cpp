//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <math.h>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const float* aMin, const float* aMax, const float* bMin, const float* bMax)
{
	return
		aMin[0] <= bMax[0] && aMax[0] >= bMin[0] &&
		aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&
		aMin[2] <= bMax[2] && aMax[2] >= bMin[2];
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	heightfield		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static rcSpan* allocSpan(rcHeightfield& heightfield)
{
	// If necessary, allocate new page and update the freelist.
	if (heightfield.freelist == NULL || heightfield.freelist->next == NULL)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* spanPool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (spanPool == NULL)
		{
			return NULL;
		}

		// Add the pool into the list of pools.
		spanPool->next = heightfield.pools;
		heightfield.pools = spanPool;
		
		// Add new spans to the free list.
		rcSpan* freeList = heightfield.freelist;
		rcSpan* head = &spanPool->items[0];
		rcSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		}
		while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	rcSpan* newSpan = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	heightfield		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(rcHeightfield& heightfield, rcSpan* span)
{
	if (span == NULL)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = heightfield.freelist;
	heightfield.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	heightfield					Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaID				The new span's area type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge area type IDs
static bool addSpan(rcHeightfield& heightfield,
                    const int x, const int z,
                    const unsigned short min, const unsigned short max,
                    const unsigned char areaID, const int flagMergeThreshold)
{
	// Create the new span.
	rcSpan* newSpan = allocSpan(heightfield);
	if (newSpan == NULL)
	{
		return false;
	}
	newSpan->smin = min;
	newSpan->smax = max;
	newSpan->area = areaID;
	newSpan->next = NULL;
	
	const int columnIndex = x + z * heightfield.width;
	rcSpan* previousSpan = NULL;
	rcSpan* currentSpan = heightfield.spans[columnIndex];
	
	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != NULL)
	{
		if (currentSpan->smin > newSpan->smax)
		{
			// Current span is completely after the new span, break.
			break;
		}
		
		if (currentSpan->smax < newSpan->smin)
		{
			// Current span is completely before the new span.  Keep going.
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			if (currentSpan->smin < newSpan->smin)
			{
				newSpan->smin = currentSpan->smin;
			}
			if (currentSpan->smax > newSpan->smax)
			{
				newSpan->smax = currentSpan->smax;
			}
			
			// Merge flags.
			if (rcAbs((int)newSpan->smax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				newSpan->area = rcMax(newSpan->area, currentSpan->area);
			}
			
			// Remove the current span since it's now merged with newSpan.
			// Keep going because there might be other overlapping spans that also need to be merged.
			rcSpan* next = currentSpan->next;
			freeSpan(heightfield, currentSpan);
			if (previousSpan)
			{
				previousSpan->next = next;
			}
			else
			{
				heightfield.spans[columnIndex] = next;
			}
			currentSpan = next;
		}
	}
	
	// Insert new span after prev
	if (previousSpan != NULL)
	{
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// This span should go before the others in the list
		newSpan->next = heightfield.spans[columnIndex];
		heightfield.spans[columnIndex] = newSpan;
	}

	return true;
}

// A span produced while rasterizing a triangle. Batching spans while the
// heightfield is empty lets each column be merged in contiguous memory and
// emits the final linked lists column-by-column. The per-column input order is
// retained, so area merging has the same semantics as repeated addSpan calls.
struct RasterizedSpan
{
	int columnIndex;
	unsigned int span;
};

static const unsigned int SPAN_HEIGHT_BITS = 13;
static const unsigned int SPAN_HEIGHT_MASK = (1u << SPAN_HEIGHT_BITS) - 1;

static unsigned int packRasterizedSpan(
	const unsigned short smin, const unsigned short smax, const unsigned char area)
{
	return (unsigned int)smin | ((unsigned int)smax << SPAN_HEIGHT_BITS)
		| ((unsigned int)area << (SPAN_HEIGHT_BITS * 2));
}

static unsigned short getRasterizedSpanMin(const unsigned int span)
{
	return (unsigned short)(span & SPAN_HEIGHT_MASK);
}

static unsigned short getRasterizedSpanMax(const unsigned int span)
{
	return (unsigned short)((span >> SPAN_HEIGHT_BITS) & SPAN_HEIGHT_MASK);
}

static unsigned char getRasterizedSpanArea(const unsigned int span)
{
	return (unsigned char)(span >> (SPAN_HEIGHT_BITS * 2));
}

static const int SPAN_WORD_BITS = 64;
static const int SPAN_WORD_COUNT = (SPAN_HEIGHT_MASK + 1) / SPAN_WORD_BITS;
static const int SPAN_WORD_GROUP_COUNT = (SPAN_WORD_COUNT + SPAN_WORD_BITS - 1) / SPAN_WORD_BITS;

static int findFirstSetBit(const unsigned long long value)
{
#if defined(_MSC_VER)
	unsigned long index;
#if defined(_M_X64) || defined(_M_ARM64)
	_BitScanForward64(&index, value);
	return (int)index;
#else
	if (_BitScanForward(&index, (unsigned long)value))
	{
		return (int)index;
	}
	_BitScanForward(&index, (unsigned long)(value >> 32));
	return (int)index + 32;
#endif
#elif defined(__GNUC__) || defined(__clang__)
	return __builtin_ctzll(value);
#else
	int index = 0;
	unsigned long long remaining = value;
	while ((remaining & 1) == 0)
	{
		remaining >>= 1;
		++index;
	}
	return index;
#endif
}

static int findLastSetBit(const unsigned long long value)
{
#if defined(_MSC_VER)
	unsigned long index;
#if defined(_M_X64) || defined(_M_ARM64)
	_BitScanReverse64(&index, value);
	return (int)index;
#else
	if (_BitScanReverse(&index, (unsigned long)(value >> 32)))
	{
		return (int)index + 32;
	}
	_BitScanReverse(&index, (unsigned long)value);
	return (int)index;
#endif
#elif defined(__GNUC__) || defined(__clang__)
	return 63 - __builtin_clzll(value);
#else
	int index = 0;
	unsigned long long remaining = value;
	while (remaining >>= 1)
	{
		++index;
	}
	return index;
#endif
}

static int findActiveSpanAtOrBefore(const unsigned long long* occupied,
	const unsigned long long* occupiedWords, const int height)
{
	const int word = height / SPAN_WORD_BITS;
	const int bit = height % SPAN_WORD_BITS;
	const unsigned long long throughBit = bit == SPAN_WORD_BITS - 1
		? ~0ull : (1ull << (bit + 1)) - 1;
	const unsigned long long inWord = occupied[word] & throughBit;
	if (inWord != 0)
	{
		return word * SPAN_WORD_BITS + findLastSetBit(inWord);
	}

	const int firstGroup = word / SPAN_WORD_BITS;
	for (int group = firstGroup; group >= 0; --group)
	{
		unsigned long long candidates = occupiedWords[group];
		if (group == firstGroup)
		{
			const int wordInGroup = word % SPAN_WORD_BITS;
			candidates &= wordInGroup == 0 ? 0 : (1ull << wordInGroup) - 1;
		}
		if (candidates == 0)
		{
			continue;
		}
		const int candidateWord = group * SPAN_WORD_BITS + findLastSetBit(candidates);
		return candidateWord * SPAN_WORD_BITS + findLastSetBit(occupied[candidateWord]);
	}
	return -1;
}

static int findActiveSpanAtOrAfter(const unsigned long long* occupied,
	const unsigned long long* occupiedWords, const int height)
{
	const int word = height / SPAN_WORD_BITS;
	const int bit = height % SPAN_WORD_BITS;
	const unsigned long long inWord = occupied[word] & (~0ull << bit);
	if (inWord != 0)
	{
		return word * SPAN_WORD_BITS + findFirstSetBit(inWord);
	}

	const int firstGroup = word / SPAN_WORD_BITS;
	for (int group = firstGroup; group < SPAN_WORD_GROUP_COUNT; ++group)
	{
		unsigned long long candidates = occupiedWords[group];
		if (group == firstGroup)
		{
			const int wordInGroup = word % SPAN_WORD_BITS;
			candidates &= wordInGroup == SPAN_WORD_BITS - 1 ? 0 : ~0ull << (wordInGroup + 1);
		}
		if (candidates == 0)
		{
			continue;
		}
		const int candidateWord = group * SPAN_WORD_BITS + findFirstSetBit(candidates);
		return candidateWord * SPAN_WORD_BITS + findFirstSetBit(occupied[candidateWord]);
	}
	return -1;
}

static int findActiveSpanAfter(const unsigned long long* occupied,
	const unsigned long long* occupiedWords, const int height)
{
	return height == (int)SPAN_HEIGHT_MASK ? -1
		: findActiveSpanAtOrAfter(occupied, occupiedWords, height + 1);
}

static void setActiveSpan(unsigned long long* occupied, unsigned long long* occupiedWords, const int height)
{
	const int word = height / SPAN_WORD_BITS;
	occupied[word] |= 1ull << (height % SPAN_WORD_BITS);
	occupiedWords[word / SPAN_WORD_BITS] |= 1ull << (word % SPAN_WORD_BITS);
}

static void clearActiveSpan(unsigned long long* occupied, unsigned long long* occupiedWords, const int height)
{
	const int word = height / SPAN_WORD_BITS;
	occupied[word] &= ~(1ull << (height % SPAN_WORD_BITS));
	if (occupied[word] == 0)
	{
		occupiedWords[word / SPAN_WORD_BITS] &= ~(1ull << (word % SPAN_WORD_BITS));
	}
}

static void mergeRasterizedSpan(unsigned int* spans, unsigned long long* occupied,
	unsigned long long* occupiedWords, unsigned short spanMin, unsigned short spanMax,
	unsigned char spanArea, const int flagMergeThreshold)
{
	int current = findActiveSpanAtOrBefore(occupied, occupiedWords, spanMin);
	if (current == -1 || getRasterizedSpanMax(spans[current]) < spanMin)
	{
		current = findActiveSpanAtOrAfter(occupied, occupiedWords, spanMin);
	}

	while (current != -1 && current <= spanMax)
	{
		const unsigned int currentSpan = spans[current];
		const unsigned short currentMin = getRasterizedSpanMin(currentSpan);
		const unsigned short currentMax = getRasterizedSpanMax(currentSpan);
		const int next = findActiveSpanAfter(occupied, occupiedWords, current);
		spanMin = rcMin(spanMin, currentMin);
		spanMax = rcMax(spanMax, currentMax);
		if (rcAbs((int)spanMax - (int)currentMax) <= flagMergeThreshold)
		{
			spanArea = rcMax(spanArea, getRasterizedSpanArea(currentSpan));
		}
		clearActiveSpan(occupied, occupiedWords, current);
		current = next;
	}

	spans[spanMin] = packRasterizedSpan(spanMin, spanMax, spanArea);
	setActiveSpan(occupied, occupiedWords, spanMin);
}

static bool addRasterizedSpans(
	rcHeightfield& heightfield, rcTempVector<RasterizedSpan>& rasterized, const int flagMergeThreshold)
{
	const int columnCount = heightfield.width * heightfield.height;
	rcTempVector<int> offsets(columnCount + 1, 0);
	for (int i = 0; i < rasterized.size(); ++i)
	{
		++offsets[rasterized[i].columnIndex + 1];
	}
	for (int i = 1; i <= columnCount; ++i)
	{
		offsets[i] += offsets[i - 1];
	}

	rcTempVector<int> next(offsets);
	rcTempVector<unsigned int> ordered(rasterized.size());
	for (int i = 0; i < rasterized.size(); ++i)
	{
		const RasterizedSpan& span = rasterized[i];
		ordered[next[span.columnIndex]++] = span.span;
	}

	rcTempVector<unsigned int> merged(SPAN_HEIGHT_MASK + 1);
	unsigned long long occupied[SPAN_WORD_COUNT] = {};
	unsigned long long occupiedWords[SPAN_WORD_GROUP_COUNT] = {};
	for (int columnIndex = 0; columnIndex < columnCount; ++columnIndex)
	{
		for (int i = offsets[columnIndex]; i < offsets[columnIndex + 1]; ++i)
		{
			const unsigned int span = ordered[i];
			mergeRasterizedSpan(merged.data(), occupied, occupiedWords, getRasterizedSpanMin(span),
				getRasterizedSpanMax(span), getRasterizedSpanArea(span), flagMergeThreshold);
		}

		rcSpan* previous = NULL;
		for (int group = 0; group < SPAN_WORD_GROUP_COUNT; ++group)
		{
			unsigned long long activeWords = occupiedWords[group];
			while (activeWords != 0)
			{
				const int word = group * SPAN_WORD_BITS + findFirstSetBit(activeWords);
				unsigned long long activeSpans = occupied[word];
				while (activeSpans != 0)
				{
					const int active = word * SPAN_WORD_BITS + findFirstSetBit(activeSpans);
					const unsigned int mergedSpan = merged[active];
					rcSpan* span = allocSpan(heightfield);
					if (span == NULL)
					{
						return false;
					}
					span->smin = getRasterizedSpanMin(mergedSpan);
					span->smax = getRasterizedSpanMax(mergedSpan);
					span->area = getRasterizedSpanArea(mergedSpan);
					span->next = NULL;
					if (previous == NULL)
					{
						heightfield.spans[columnIndex] = span;
					}
					else
					{
						previous->next = span;
					}
					previous = span;
					activeSpans &= activeSpans - 1;
				}
				occupied[word] = 0;
				activeWords &= activeWords - 1;
			}
			occupiedWords[group] = 0;
		}
	}
	return true;
}

bool rcAddSpan(rcContext* context, rcHeightfield& heightfield,
               const int x, const int z,
               const unsigned short spanMin, const unsigned short spanMax,
               const unsigned char areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

static void includeRowX(const float x, const int count, float& minX, float& maxX)
{
	if (count == 0)
	{
		minX = x;
		maxX = x;
	}
	else
	{
		if (minX > x)
		{
			minX = x;
		}
		if (maxX < x)
		{
			maxX = x;
		}
	}
}

/// Divides a convex polygon of max 12 vertices into a grid row and the
/// remainder above it.
/// 
/// @param[in]	inVerts			The input polygon vertices
/// @param[in]	inVertsCount	The number of input polygon vertices
/// @param[out]	outRow			Resulting row polygon vertices
/// @param[out]	outRowCount	The number of resulting row polygon vertices
/// @param[out]	outRemainder	Resulting remainder polygon vertices
/// @param[out]	outRemainderCount The number of resulting remainder polygon vertices
/// @param[in]	axisOffset		The row's upper Z bound
/// @param[out]	minX			Minimum X extent of the row polygon
/// @param[out]	maxX			Maximum X extent of the row polygon
static void dividePolyRow(const float* inVerts, int inVertsCount,
                          float* outRow, int* outRowCount,
                          float* outRemainder, int* outRemainderCount,
                          float axisOffset, float& minX, float& maxX)
{
	rcAssert(inVertsCount <= 12);
	if (inVertsCount == 0)
	{
		*outRowCount = 0;
		*outRemainderCount = 0;
		return;
	}

	int rowVert = 0;
	int remainderVert = 0;
	float inVertBDelta = axisOffset - inVerts[(inVertsCount - 1) * 3 + 2];
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		const float inVertADelta = axisOffset - inVerts[inVertA * 3 + 2];
		// If the two vertices are on the same side of the separating axis
		bool sameSide = (inVertADelta >= 0) == (inVertBDelta >= 0);

		if (!sameSide)
		{
			float s = inVertBDelta / (inVertBDelta - inVertADelta);
			outRow[rowVert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outRow[rowVert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outRow[rowVert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			includeRowX(outRow[rowVert * 3], rowVert, minX, maxX);
			rcVcopy(&outRemainder[remainderVert * 3], &outRow[rowVert * 3]);
			rowVert++;
			remainderVert++;
			
			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (inVertADelta > 0)
			{
				rcVcopy(&outRow[rowVert * 3], &inVerts[inVertA * 3]);
				includeRowX(outRow[rowVert * 3], rowVert, minX, maxX);
				rowVert++;
			}
			else if (inVertADelta < 0)
			{
				rcVcopy(&outRemainder[remainderVert * 3], &inVerts[inVertA * 3]);
				remainderVert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (inVertADelta >= 0)
			{
				rcVcopy(&outRow[rowVert * 3], &inVerts[inVertA * 3]);
				includeRowX(outRow[rowVert * 3], rowVert, minX, maxX);
				rowVert++;
				if (inVertADelta != 0)
				{
					inVertBDelta = inVertADelta;
					continue;
				}
			}
			rcVcopy(&outRemainder[remainderVert * 3], &inVerts[inVertA * 3]);
			remainderVert++;
		}
		inVertBDelta = inVertADelta;
	}

	*outRowCount = rowVert;
	*outRemainderCount = remainderVert;
}

static void includeSpanHeight(const float height, int& count, float& spanMin, float& spanMax)
{
	if (count == 0)
	{
		spanMin = height;
		spanMax = height;
	}
	else
	{
		spanMin = rcMin(spanMin, height);
		spanMax = rcMax(spanMax, height);
	}
	++count;
}

// The positive output of the X-axis clip is consumed only through its vertex
// count and Y extents. Preserve the original clipping order and remainder
// polygon, but reduce those heights as they are produced instead of writing
// and rereading a full temporary polygon.
static void dividePolyCell(const float* inVerts, const int inVertsCount,
	int* outVertsCount, float& spanMin, float& spanMax,
	float* outRemainder, int* outRemainderCount, const float axisOffset)
{
	rcAssert(inVertsCount <= 12);
	if (inVertsCount == 0)
	{
		*outVertsCount = 0;
		*outRemainderCount = 0;
		return;
	}

	int cellVert = 0;
	int remainderVert = 0;
	float inVertBDelta = axisOffset - inVerts[(inVertsCount - 1) * 3];
	for (int inVertA = 0, inVertB = inVertsCount - 1;
		inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		const float inVertADelta = axisOffset - inVerts[inVertA * 3];
		const bool sameSide = (inVertADelta >= 0) == (inVertBDelta >= 0);
		if (!sameSide)
		{
			const float s = inVertBDelta / (inVertBDelta - inVertADelta);
			outRemainder[remainderVert * 3 + 0] = inVerts[inVertB * 3 + 0]
				+ (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outRemainder[remainderVert * 3 + 1] = inVerts[inVertB * 3 + 1]
				+ (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outRemainder[remainderVert * 3 + 2] = inVerts[inVertB * 3 + 2]
				+ (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			includeSpanHeight(outRemainder[remainderVert * 3 + 1], cellVert, spanMin, spanMax);
			++remainderVert;

			if (inVertADelta > 0)
			{
				includeSpanHeight(inVerts[inVertA * 3 + 1], cellVert, spanMin, spanMax);
			}
			else if (inVertADelta < 0)
			{
				rcVcopy(&outRemainder[remainderVert * 3], &inVerts[inVertA * 3]);
				++remainderVert;
			}
		}
		else
		{
			if (inVertADelta >= 0)
			{
				includeSpanHeight(inVerts[inVertA * 3 + 1], cellVert, spanMin, spanMax);
				if (inVertADelta != 0)
				{
					inVertBDelta = inVertADelta;
					continue;
				}
			}
			rcVcopy(&outRemainder[remainderVert * 3], &inVerts[inVertA * 3]);
			++remainderVert;
		}
		inVertBDelta = inVertADelta;
	}

	*outVertsCount = cellVert;
	*outRemainderCount = remainderVert;
}

static bool addRasterizedCellSpan(rcHeightfield& heightfield, const int x, const int z,
	float spanMin, float spanMax, const float heightfieldMin, const float heightfieldHeight,
	const float inverseCellHeight, const unsigned char areaID, const int flagMergeThreshold,
	rcTempVector<RasterizedSpan>* rasterizedSpans)
{
	spanMin -= heightfieldMin;
	spanMax -= heightfieldMin;

	if (spanMax < 0.0f || spanMin > heightfieldHeight)
	{
		return true;
	}

	spanMin = rcMax(spanMin, 0.0f);
	spanMax = rcMin(spanMax, heightfieldHeight);
	const unsigned short spanMinCellIndex = (unsigned short)rcClamp(
		(int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
	const unsigned short spanMaxCellIndex = (unsigned short)rcClamp(
		(int)ceilf(spanMax * inverseCellHeight), (int)spanMinCellIndex + 1, RC_SPAN_MAX_HEIGHT);

	if (rasterizedSpans != NULL)
	{
		RasterizedSpan rasterizedSpan;
		rasterizedSpan.columnIndex = x + z * heightfield.width;
		rasterizedSpan.span = packRasterizedSpan(
			spanMinCellIndex, spanMaxCellIndex, (unsigned char)(areaID & 0x3f));
		rasterizedSpans->push_back(rasterizedSpan);
		return true;
	}
	return addSpan(heightfield, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold);
}

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// @param[in] 	v0					Triangle vertex 0
/// @param[in] 	v1					Triangle vertex 1
/// @param[in] 	v2					Triangle vertex 2
/// @param[in] 	areaID				The area ID to assign to the rasterized spans
/// @param[in] 	heightfield			Heightfield to rasterize into
/// @param[in] 	heightfieldBBMin	The min extents of the heightfield bounding box
/// @param[in] 	heightfieldBBMax	The max extents of the heightfield bounding box
/// @param[in] 	cellSize			The x and z axis size of a voxel in the heightfield
/// @param[in] 	inverseCellSize		1 / cellSize
/// @param[in] 	inverseCellHeight	1 / cellHeight
/// @param[in] 	flagMergeThreshold	The threshold in which area flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield,
                         const float* heightfieldBBMin, const float* heightfieldBBMax,
                         const float cellSize, const float inverseCellSize, const float inverseCellHeight,
                         const int flagMergeThreshold, rcTempVector<RasterizedSpan>* rasterizedSpans)
{
	// Calculate the bounding box of the triangle.
	float triBBMin[3];
	rcVcopy(triBBMin, v0);
	rcVmin(triBBMin, v1);
	rcVmin(triBBMin, v2);

	float triBBMax[3];
	rcVcopy(triBBMax, v0);
	rcVmax(triBBMax, v1);
	rcVmax(triBBMax, v2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if (!overlapBounds(triBBMin, triBBMax, heightfieldBBMin, heightfieldBBMax))
	{
		return true;
	}

	const int w = heightfield.width;
	const int h = heightfield.height;
	const float by = heightfieldBBMax[1] - heightfieldBBMin[1];

	// Calculate the footprint of the triangle on the grid's z-axis
	int z0 = (int)((triBBMin[2] - heightfieldBBMin[2]) * inverseCellSize);
	int z1 = (int)((triBBMax[2] - heightfieldBBMin[2]) * inverseCellSize);

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	z0 = rcClamp(z0, -1, h - 1);
	z1 = rcClamp(z1, 0, h - 1);

	const int triangleX0 = (int)((triBBMin[0] - heightfieldBBMin[0]) * inverseCellSize);
	const int triangleX1 = (int)((triBBMax[0] - heightfieldBBMin[0]) * inverseCellSize);
	if (z0 == z1 && z0 >= 0 && triangleX0 == triangleX1 && triangleX0 >= 0 && triangleX0 < w)
	{
		const float cellX = heightfieldBBMin[0] + (float)triangleX0 * cellSize;
		const float cellZ = heightfieldBBMin[2] + (float)z0 * cellSize;
		if (triBBMin[0] >= cellX && triBBMax[0] <= cellX + cellSize
			&& triBBMin[2] >= cellZ && triBBMax[2] <= cellZ + cellSize)
		{
			return addRasterizedCellSpan(heightfield, triangleX0, z0, triBBMin[1], triBBMax[1],
				heightfieldBBMin[1], by, inverseCellHeight, areaID, flagMergeThreshold, rasterizedSpans);
		}
	}

	// Clip the triangle into all grid cells it touches.
	float buf[7 * 3 * 4];
	float* in = buf;
	float* inRow = buf + 7 * 3;
	float* p1 = inRow + 7 * 3;
	float* p2 = p1 + 7 * 3;

	rcVcopy(&in[0], v0);
	rcVcopy(&in[1 * 3], v1);
	rcVcopy(&in[2 * 3], v2);
	int nvRow;
	int nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cellZ = heightfieldBBMin[2] + (float)z * cellSize;
		float minX = 0.0f;
		float maxX = 0.0f;
		if (z == z1 && triBBMax[2] <= cellZ + cellSize)
		{
			// The residual polygon is already wholly inside the final row.
			// Consume it directly; there is no next row that needs a remainder.
			inRow = in;
			nvRow = nvIn;
			for (int vert = 0; vert < nvRow; ++vert)
			{
				includeRowX(inRow[vert * 3], vert, minX, maxX);
			}
		}
		else
		{
			dividePolyRow(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ + cellSize, minX, maxX);
			rcSwap(in, p1);
		}
		
		if (nvRow < 3)
		{
			continue;
		}
		if (z < 0)
		{
			continue;
		}
		
		int x0 = (int)((minX - heightfieldBBMin[0]) * inverseCellSize);
		int x1 = (int)((maxX - heightfieldBBMin[0]) * inverseCellSize);
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}
		x0 = rcClamp(x0, -1, w - 1);
		x1 = rcClamp(x1, 0, w - 1);

		int nv;
		int nv2 = nvRow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = heightfieldBBMin[0] + (float)x * cellSize;
			float spanMin = 0.0f;
			float spanMax = 0.0f;
			if (x == x1 && maxX <= cx + cellSize)
			{
				// The residual polygon is already wholly inside the final cell.
				// Reduce it directly; there is no next cell that needs a remainder.
				nv = nv2;
				if (nv >= 3)
				{
					spanMin = inRow[1];
					spanMax = inRow[1];
					for (int vert = 1; vert < nv; ++vert)
					{
						spanMin = rcMin(spanMin, inRow[vert * 3 + 1]);
						spanMax = rcMax(spanMax, inRow[vert * 3 + 1]);
					}
				}
			}
			else
			{
				dividePolyCell(inRow, nv2, &nv, spanMin, spanMax, p2, &nv2, cx + cellSize);
				rcSwap(inRow, p2);
			}
			
			if (nv < 3)
			{
				continue;
			}
			if (x < 0)
			{
				continue;
			}
			
			if (!addRasterizedCellSpan(heightfield, x, z, spanMin, spanMax, heightfieldBBMin[1], by,
				inverseCellHeight, areaID, flagMergeThreshold, rasterizedSpans))
			{
				return false;
			}
		}
	}

	return true;
}

bool rcRasterizeTriangle(rcContext* context,
                         const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	if (!rasterizeTri(v0, v1, v2, areaID, heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, NULL))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const int* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	const bool batchSpans = heightfield.pools == NULL && numTris >= 64;
	rcTempVector<RasterizedSpan> rasterizedSpans;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, batchSpans ? &rasterizedSpans : NULL))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}
	if (batchSpans && !addRasterizedSpans(heightfield, rasterizedSpans, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const unsigned short* tris, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	const bool batchSpans = heightfield.pools == NULL && numTris >= 64;
	rcTempVector<RasterizedSpan> rasterizedSpans;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, batchSpans ? &rasterizedSpans : NULL))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}
	if (batchSpans && !addRasterizedSpans(heightfield, rasterizedSpans, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const unsigned char* triAreaIDs, const int numTris,
                          rcHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	const bool batchSpans = heightfield.pools == NULL && numTris >= 64;
	rcTempVector<RasterizedSpan> rasterizedSpans;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[(triIndex * 3 + 0) * 3];
		const float* v1 = &verts[(triIndex * 3 + 1) * 3];
		const float* v2 = &verts[(triIndex * 3 + 2) * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold, batchSpans ? &rasterizedSpans : NULL))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}
	if (batchSpans && !addRasterizedSpans(heightfield, rasterizedSpans, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
		return false;
	}

	return true;
}
