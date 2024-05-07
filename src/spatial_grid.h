#pragma once

/**
 * thoughts on how i want to implement this
 * =================================================
 *
 * - every thread in the simulation loop will have
 *   its own hash grid to avoid race conditions
 * - similarly to how the boid array is handled,
 *   hash grids will be "double-buffered"
 * - there will be one grid from the previous
 *   iteration, that will only be read from, and
 *   one grid for the next iteration, that will
 *   only be written to (per thread)
 *
 * - the hash grid itself will map x/y/z coordinates
 *   to a list of elements that are within a cell
 * - since the maximum possible number of elements
 *   is known, all list items will be stored in a
 *   single stack which can be cleared during
 *   every iteration of the loop
 * - clearing the stack and reconstructing the
 *   buckets every iteration should be easier than
 *   updating the lists as it doesn't require
 *   searching for elements
 * - the lists of items belonging to the same cell
 *   will be constructed incrementally on the stack
 * - querying the hash grid with x/y/z coordinates
 *   will simply return the head of the associated
 *   list, potentially combining lists from all
 *   cells within a given range
 */
