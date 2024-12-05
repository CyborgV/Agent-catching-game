*Description : *This file implements the Map Abstract Data Type(ADT) used to represent 
 * a network of cities connected by roads. It supports basic operations 
 * for managing the map structure, such as adding cities, roads, and retrieving 
 * information about connections between cities. The map is designed to integrate 
 * seamlessly with agent-based simulations and supports adjacency matrix 
 * representation for efficient access.
 * 
 * Overview:
 * - The program uses modular design to encapsulate map-related state and operations.
 * - Supports key functionalities:
 *   - Adding and naming cities.
 *   - Inserting roads with specific lengths between cities.
 *   - Querying road information, including connections and distances.
 * - Includes utility functions for displaying the map and managing resources.
 * - Uses sorting (via `qsort`) to ensure consistent ordering of road outputs.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Map.h"

#define DEFAULT_CITY_NAME "unnamed"

// Struct representing a city in the map
typedef struct city
{
    char *name;
    bool hasInformant;
} City;

// Struct representing the map
struct map
{
    int nV;
    int nE;
    City *cities;
    int **edges;
};

static int compareByTo(const void *a, const void *b);

// Creates a new map with the specified number of cities
Map MapNew(int numCities)
{
    Map map = malloc(sizeof(struct map));
    if (map == NULL)
    {
        perror("Failed to allocate memory for Map");
        exit(EXIT_FAILURE);
    }

    map->nV = numCities;
    map->nE = 0;
    map->cities = malloc(numCities * sizeof(City));
    // Initialize all cities with default values
    for (int i = 0; i < numCities; i++)
    {
        map->cities[i].name = NULL;
        map->cities[i].hasInformant = false;
    }

    // Allocate and initialize the adjacency matrix for roads
    map->edges = calloc(sizeof(int *), numCities);
    for (int i = 0; i < numCities; i++)
    {
        map->edges[i] = calloc(sizeof(int), numCities);
    }

    return map;
}

// Frees all memory associated with the map
void MapFree(Map m)
{
    if (m == NULL)
        return;

    for (int i = 0; i < m->nV; i++)
    {
        free(m->edges[i]);
    }
    free(m->edges);
    // Free city names and city array
    for (int i = 0; i < m->nV; i++)
    {
        if (m->cities[i].name != NULL)
        {
            free(m->cities[i].name);
        }
    }
    free(m->cities);

    free(m);
}

// Returns the number of cities in the map
int MapNumCities(Map m)
{
    if (m == NULL)
        return 0;
    return m->nV;
}

// Returns the number of roads in the map
int MapNumRoads(Map m)
{
    if (m == NULL)
        return 0;
    return m->nE;
}

// Sets the name of a city
void MapSetName(Map m, int city, char *name)
{
    if (city < 0 || city > m->nV)
        return;
    City *mapCity = &m->cities[city];
    if (mapCity->name != NULL)
    {
        free(mapCity->name);
    }
    // Allocate memory and copy the new name
    mapCity->name = malloc(strlen(name) + 1);
    strcpy(mapCity->name, name);
}

// Gets the name of a city, or "unnamed" if no name is set
char *MapGetName(Map m, int city)
{
    if (m->cities[city].name == NULL)
    {
        return DEFAULT_CITY_NAME;
    }
    return m->cities[city].name;
}

// Inserts a road between two cities with the specified length
void MapInsertRoad(Map m, int city1, int city2, int length)
{
    if (MapContainsRoad(m, city1, city2))
        return;

    // Add road to adjacency matrix and increment the road count
    m->edges[city1][city2] = length;
    m->edges[city2][city1] = length;
    m->nE += 1;
}

// Checks if a road exists between two cities
int MapContainsRoad(Map m, int city1, int city2)
{
    return (m->edges[city1][city2]);
}

// Gets all roads from a city, storing them in the given array
// @return The number of roads found
int MapGetRoadsFrom(Map m, int city, struct road roads[])
{
    if (m == NULL)
        return 0;

    int numRoad = 0;
    // Check each possible destination city
    for (int i = 0; i < m->nV; i++)
    {
        if (m->edges[city][i] != 0)
        {
            roads[numRoad].from = city;
            roads[numRoad].to = i;
            roads[numRoad].length = m->edges[city][i];
            numRoad += 1;
        }
    }
    // Sort roads by destination city for consistent output
    qsort(roads, numRoad, sizeof(struct road), compareByTo);
    return numRoad;
}

// Comparison function for sorting roads by destination city
static int compareByTo(const void *a, const void *b)
{
    const struct road *roadA = (const struct road *)a;
    const struct road *roadB = (const struct road *)b;
    return roadA->to - roadB->to;
}
/**
 * !!! DO NOT EDIT THIS FUNCTION !!!
 * This function will work once the other functions are working
 */
void MapShow(Map m)
{
    printf("Number of cities: %d\n", MapNumCities(m));
    printf("Number of roads: %d\n", MapNumRoads(m));

    struct road *roads = malloc(MapNumRoads(m) * sizeof(struct road));
    if (roads == NULL)
    {
        fprintf(stderr, "error: out of memory\n");
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < MapNumCities(m); i++)
    {
        printf("[%d] %s has roads to:", i, MapGetName(m, i));
        int numRoads = MapGetRoadsFrom(m, i, roads);
        for (int j = 0; j < numRoads; j++)
        {
            if (j > 0)
            {
                printf(",");
            }
            printf(" [%d] %s (%d)", roads[j].to, MapGetName(m, roads[j].to),
                   roads[j].length);
        }
        printf("\n");
    }

    free(roads);
}