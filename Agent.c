#include "Agent.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Map.h"

#define PQ_DEFAULT_CAPACITY 64
#define INT_MAX 2147483647
#define NO_PREVIOUS -1
#define NO_THIEF_LOCATION -1
#define NO_STAMINA -1
#define NO_CITY -1

typedef struct pq *Pq;

// This struct stores information about an individual agent and can be
// used to store information that the agent needs to remember.
struct agent
{
    char *name;
    int startLocation;
    int location;
    int maxStamina; // max stamina
    int stamina;    // current stamina
    int strategy;
    Map map;
    // stage 1
    int *cityVisitCounts;
    // stage 2
    int *dfsVisited;
    int *dfsPre;
    // stage 3
    struct move *path;
    int pathLength;
    int pathIndex;
    bool hasTipOff;
    int thiefLocation;
};

// This struct represents a priority queue node used in Dijkstra's algorithm.
// It stores the current cost, city, and stamina of a path.
struct PQNode
{
    int cost;
    int city;
    int stamina;
};

// This struct implements a priority queue for Dijkstra's algorithm.
// The priority queue is implemented as a binary heap.
struct pq
{
    struct PQNode *items;
    int size;
    int capacity;
};

// Function prototypes
// stage 3
static struct move chooseLeastTurnPath(Agent agent, Map m);
// stage 0
static struct move chooseRandomMove(Agent agent, Map m);
static int filterRoads(Agent agent, struct road roads[], int numRoads,
                       struct road legalRoads[]);
// stage 1
static struct move chooseLeastCheapestMove(Agent agent, Map m);
// staeg 2
static struct move chooseDfsMove(Agent agent, Map m);
static void resetDfs(Agent agent, Map m);
static struct move dfsMove(struct move move, Agent agent, Map m,
                           struct road roads[], int numRoads,
                           bool *unvisitedAdj);
static struct move dfsBack(struct move move, Agent agent, Map m,
                           struct road roads[], int numRoads);

// stage 3
static void computeLeastTurnsPath(Agent agent, int thiefLocation);
static void initializeDataStructures(Agent agent, int **dist, int **prevCity,
                                     int **prevStamina, bool **visited);
static void performDijkstra(Agent agent, int targetCity, int **dist,
                            int **prevCity, int **prevStamina, bool **visited);
static void tryRestDijkstra(Agent agent, int city, int stamina, int cost,
                            int **dist, int **prevCity, int **prevStamina,
                            Pq pq);
static void tryMoveDijkstra(Agent agent, int city, int stamina, int cost,
                            int **dist, int **prevCity, int **prevStamina,
                            Pq pq);
static void reconstructPath(Agent agent, int targetCity, int **dist,
                            int **prevCity, int **prevStamina);
// Pq used in stage 3
static Pq PqNew(void);
static void PqFree(Pq pq);
static void PqInsert(Pq pq, struct PQNode node);
static struct PQNode PqExtract(Pq pq);
static void fixUp(Pq pq, int i);
static void fixDown(Pq pq, int i);
static void swap(struct PQNode *items, int i, int j);

// Creates a new agent
Agent AgentNew(int start, int stamina, int strategy, Map m, char *name)
{
    if (start >= MapNumCities(m))
    {
        fprintf(stderr, "error: starting city (%d) is invalid\n", start);
        exit(EXIT_FAILURE);
    }

    Agent agent = malloc(sizeof(struct agent));
    if (agent == NULL)
    {
        fprintf(stderr, "error: out of memory\n");
        exit(EXIT_FAILURE);
    }

    agent->startLocation = start;
    agent->location = start;
    agent->maxStamina = stamina;
    agent->stamina = stamina;
    agent->strategy = strategy;
    agent->map = m;
    agent->name = strdup(name);

    agent->cityVisitCounts = calloc(MapNumCities(m), sizeof(int));
    agent->cityVisitCounts[start] = 1;
    agent->dfsVisited = calloc(MapNumCities(m), sizeof(int));
    agent->dfsVisited[start] = 1;
    agent->dfsPre = calloc(MapNumCities(m), sizeof(int));
    for (int i = 0; i < MapNumCities(m); i++)
    {
        agent->dfsPre[i] = NO_PREVIOUS;
    }

    agent->path = NULL;
    agent->pathLength = 0;
    agent->pathIndex = 0;
    agent->hasTipOff = false;
    agent->thiefLocation = NO_THIEF_LOCATION;

    return agent;
}

// Frees all memory allocated to the agent
void AgentFree(Agent agent)
{
    free(agent->name);
    free(agent->cityVisitCounts);
    free(agent->dfsVisited);
    free(agent->dfsPre);
    if (agent->path != NULL)
    {
        free(agent->path);
    }
    free(agent);
}

////////////////////////////////////////////////////////////////////////
// Gets information about the agent

// Gets the name of the agent
char *AgentName(Agent agent)
{
    return agent->name;
}

// Gets the current location of the agent
int AgentLocation(Agent agent)
{
    return agent->location;
}

// Gets the amount of stamina the agent currently has
int AgentStamina(Agent agent)
{
    return agent->stamina;
}

////////////////////////////////////////////////////////////////////////
// Making moves

// Calculates the agent's next move
// NOTE: Does NOT actually carry out the move
struct move AgentGetNextMove(Agent agent, Map m)
{
    if (agent->hasTipOff)
    {
        return chooseLeastTurnPath(agent, m);
    }

    if (agent->strategy == STATIONARY)
    {
        return (struct move){agent->location, 0};
    }
    else if (agent->strategy == RANDOM)
    {
        return chooseRandomMove(agent, m);
    }
    else if (agent->strategy == CHEAPEST_LEAST_VISITED)
    {
        return chooseLeastCheapestMove(agent, m);
    }
    else if (agent->strategy == DFS)
    {
        return chooseDfsMove(agent, m);
    }
    else
    {
        printf("error: strategy not implemented yet\n");
        exit(EXIT_FAILURE);
    }
}

// Chooses the next move along the least-turns path if a tip-off has been
// received
// @return The next move along the least-turns path
static struct move chooseLeastTurnPath(Agent agent, Map m)
{
    if (agent->pathIndex < agent->pathLength)
    {
        struct move move = agent->path[agent->pathIndex];
        agent->pathIndex++;
        return move;
    }
    else
    {
        return (struct move){agent->location, 0};
    }
}

static struct move chooseRandomMove(Agent agent, Map m)
{
    struct road *roads = malloc(MapNumCities(m) * sizeof(struct road));
    struct road *legalRoads = malloc(MapNumCities(m) * sizeof(struct road));

    // Get all roads to adjacent cities
    int numRoads = MapGetRoadsFrom(m, agent->location, roads);

    // Filter out roads that the agent does not have enough stamina for
    int numLegalRoads = filterRoads(agent, roads, numRoads, legalRoads);

    struct move move;
    if (numLegalRoads > 0)
    {
        // nextMove is randomly chosen from the legal roads
        int k = rand() % numLegalRoads;
        move = (struct move){legalRoads[k].to, legalRoads[k].length};
    }
    else
    {
        // The agent must stay in the same location
        move = (struct move){agent->location, 0};
    }

    free(legalRoads);
    free(roads);
    return move;
}

// Takes an array with all the possible roads and puts the ones the agent
// has enough stamina for into the legalRoads array
// Returns the number of legal roads
static int filterRoads(Agent agent, struct road roads[], int numRoads,
                       struct road legalRoads[])
{
    int numLegalRoads = 0;
    for (int i = 0; i < numRoads; i++)
    {
        if (roads[i].length <= agent->stamina)
        {
            legalRoads[numLegalRoads++] = roads[i];
        }
    }
    return numLegalRoads;
}

// Chooses the cheapest and least-visited move among the legal moves
// @return The cheapest and least-visited move
static struct move chooseLeastCheapestMove(Agent agent, Map m)
{
    struct road *roads = malloc(MapNumCities(m) * sizeof(struct road));
    struct road *legalRoads = malloc(MapNumCities(m) * sizeof(struct road));
    int numRoads = MapGetRoadsFrom(m, agent->location, roads);
    int numLegalRoads = filterRoads(agent, roads, numRoads, legalRoads);
    struct move move = (struct move){agent->location, 0};

    if (numLegalRoads > 0)
    {
        int minVisitCount = INT_MAX;
        int minLength = INT_MAX;
        int minCityIndex = INT_MAX;
        int bestRoadIndex = -1;

        for (int i = 0; i < numLegalRoads; i++)
        {
            int toCity = legalRoads[i].to;
            int visitCount = agent->cityVisitCounts[toCity];
            int length = legalRoads[i].length;

            if (visitCount < minVisitCount ||
                (visitCount == minVisitCount && length < minLength) ||
                (visitCount == minVisitCount && length == minLength &&
                 toCity < minCityIndex))
            {
                minVisitCount = visitCount;
                minLength = length;
                minCityIndex = toCity;
                bestRoadIndex = i;
            }
        }

        if (bestRoadIndex != -1)
        {
            move = (struct move){legalRoads[bestRoadIndex].to,
                                 legalRoads[bestRoadIndex].length};
        }
    }

    free(legalRoads);
    free(roads);
    return move;
}

// Chooses the next move based on depth-first search (DFS) strategy
// @return The next move based on the DFS strategy
static struct move chooseDfsMove(Agent agent, Map m)
{
    struct road *roads = malloc(MapNumCities(m) * sizeof(struct road));
    struct road *legalRoads = malloc(MapNumCities(m) * sizeof(struct road));
    int numRoads = MapGetRoadsFrom(m, agent->location, roads);
    int numLegalRoads = filterRoads(agent, roads, numRoads, legalRoads);

    resetDfs(agent, m);

    struct move move = (struct move){agent->location, 0};

    if (numLegalRoads > 0)
    {
        bool unvisitedAdj = false;
        move = dfsMove(move, agent, m, roads, numRoads, &unvisitedAdj);
        if (agent->stamina < move.staminaCost)
        {
            move = (struct move){agent->location, 0};
        }
        else
        {
            agent->dfsVisited[move.to] = 1;
        }
        if (unvisitedAdj == false)
        {
            move = dfsBack(move, agent, m, roads, numRoads);
            if (agent->stamina < move.staminaCost)
            {
                move = (struct move){agent->location, 0};
            }
        }
    }

    free(legalRoads);
    free(roads);
    return move;
}

// Resets DFS data structures if all cities have been visited
static void resetDfs(Agent agent, Map m)
{
    int visited = true;
    for (int i = 0; i < MapNumCities(m); i++)
    {
        if (!agent->dfsVisited[i])
        {
            visited = false;
            break;
        }
    }
    if (visited)
    {
        for (int i = 0; i < MapNumCities(m); i++)
        {
            agent->dfsVisited[i] = 0;
            agent->dfsPre[i] = NO_PREVIOUS;
        }
    }
    agent->dfsVisited[agent->location] = 1;
}

// Finds the next move in the DFS traversal
// @return The next move in the DFS traversal
static struct move dfsMove(struct move move, Agent agent, Map m,
                           struct road roads[], int numRoads,
                           bool *unvisitedAdj)
{
    for (int i = 0; i < numRoads; i++)
    {
        int nextCity = roads[i].to;
        if (!agent->dfsVisited[nextCity])
        {
            move = (struct move){roads[i].to, roads[i].length};
            agent->dfsPre[nextCity] = agent->location;
            *unvisitedAdj = true;
            return move;
        }
    }
    return move;
}

// Finds the move to backtrack in the DFS traversal
// @return The backtracking move in the DFS traversal
static struct move dfsBack(struct move move, Agent agent, Map m,
                           struct road roads[], int numRoads)
{
    int pre = agent->dfsPre[agent->location];
    for (int i = 0; i < numRoads; i++)
    {
        if (roads[i].to == pre)
        {
            move = (struct move){roads[i].to, roads[i].length};
            return move;
        }
    }
    return move;
}

// Executes a given move by updating the agent's internal state
void AgentMakeNextMove(Agent agent, struct move move)
{
    if (move.to == agent->location)
    {
        agent->stamina = agent->maxStamina;
    }
    else
    {
        agent->stamina -= move.staminaCost;
    }
    agent->location = move.to;
    agent->cityVisitCounts[move.to]++;

    if (agent->hasTipOff)
    {
        if (agent->location == agent->thiefLocation ||
            agent->pathIndex >= agent->pathLength)
        {
            agent->hasTipOff = false;
            agent->thiefLocation = NO_THIEF_LOCATION;
            agent->pathLength = 0;
            agent->pathIndex = 0;
            if (agent->path != NULL)
            {
                free(agent->path);
                agent->path = NULL;
            }
        }
    }
}

// Tells the agent where the thief is
void AgentTipOff(Agent agent, int thiefLocation)
{
    agent->thiefLocation = thiefLocation;
    agent->hasTipOff = true;
    agent->pathIndex = 0;
    computeLeastTurnsPath(agent, thiefLocation);
}

// Computes the least-turns path to a target city using Dijkstra's algorithm
static void computeLeastTurnsPath(Agent agent, int thiefLocation)
{
    int numCities = MapNumCities(agent->map);
    int maxStamina = agent->maxStamina;

    int **dist = malloc(numCities * sizeof(int *));
    int **prevCity = malloc(numCities * sizeof(int *));
    int **prevStamina = malloc(numCities * sizeof(int *));
    bool **visited = malloc(numCities * sizeof(bool *));

    for (int i = 0; i < numCities; i++)
    {
        dist[i] = malloc((maxStamina + 1) * sizeof(int));
        prevCity[i] = malloc((maxStamina + 1) * sizeof(int));
        prevStamina[i] = malloc((maxStamina + 1) * sizeof(int));
        visited[i] = malloc((maxStamina + 1) * sizeof(bool));
    }

    initializeDataStructures(agent, dist, prevCity, prevStamina, visited);

    performDijkstra(agent, thiefLocation, dist, prevCity, prevStamina, visited);

    if (agent->path != NULL)
    {
        free(agent->path);
        agent->path = NULL;
    }
    reconstructPath(agent, thiefLocation, dist, prevCity, prevStamina);

    for (int i = 0; i < numCities; i++)
    {
        free(dist[i]);
    }
    free(dist);

    for (int i = 0; i < numCities; i++)
    {
        free(prevCity[i]);
        free(prevStamina[i]);
        free(visited[i]);
    }
    free(prevCity);
    free(prevStamina);
    free(visited);
}

// Initializes data structures used in Dijkstra's algorithm
static void initializeDataStructures(Agent agent, int **dist, int **prevCity,
                                     int **prevStamina, bool **visited)
{
    int numCities = MapNumCities(agent->map);
    int maxStamina = agent->maxStamina;

    for (int city = 0; city < numCities; city++)
    {
        for (int stamina = 0; stamina <= maxStamina; stamina++)
        {
            // Initialize distances to infinity
            dist[city][stamina] = INT_MAX;
            // No previous city yet
            prevCity[city][stamina] = NO_PREVIOUS;
            // No previous stamina yet
            prevStamina[city][stamina] = NO_PREVIOUS;
            // Mark all as unvisited
            visited[city][stamina] = false;
        }
    }

    int startCity = agent->location;
    int startStamina = agent->stamina;

    // Distance to start city is 0
    dist[startCity][startStamina] = 0;
    // No previous city for start
    prevCity[startCity][startStamina] = NO_PREVIOUS;
    // No previous stamina for start
    prevStamina[startCity][startStamina] = NO_PREVIOUS;
}

// Performs Dijkstra's algorithm to compute the least-turns path to the
// target city
static void performDijkstra(Agent agent, int targetCity, int **dist,
                            int **prevCity, int **prevStamina,
                            bool **visited)
{
    Pq pq = PqNew();

    int startCity = agent->location;
    int startStamina = agent->stamina;
    // Insert the starting city into the priority queue
    PqInsert(pq, (struct PQNode){0, startCity, startStamina});

    while (pq->size > 0)
    {
        struct PQNode current = PqExtract(pq);

        int cost = current.cost;
        int city = current.city;
        int stamina = current.stamina;
        // Skip if already visited
        if (visited[city][stamina])
            continue;
        // Mark as visited
        visited[city][stamina] = true;

        if (city == targetCity)
        {
            // Stop if we reached the target city
            break;
        }
        // Try resting to regain stamina
        tryRestDijkstra(agent, city, stamina, cost, dist, prevCity,
                        prevStamina, pq);
        // Try moving to adjacent cities
        tryMoveDijkstra(agent, city, stamina, cost, dist, prevCity,
                        prevStamina, pq);
    }

    PqFree(pq);
}

// Attempts to rest and updates the priority queue if the new cost is better
static void tryRestDijkstra(Agent agent, int city, int stamina, int cost,
                            int **dist, int **prevCity, int **prevStamina,
                            Pq pq)
{
    int maxStamina = agent->maxStamina;
    if (stamina < maxStamina)
    {
        // Fully regain stamina
        int newStamina = maxStamina;
        // Resting costs 1 turn
        int newCost = cost + 1;
        if (newCost < dist[city][newStamina])
        {
            // Update distance and previous state
            dist[city][newStamina] = newCost;
            prevCity[city][newStamina] = city;
            prevStamina[city][newStamina] = stamina;
            PqInsert(pq, (struct PQNode){newCost, city, newStamina});
        }
        else if (newCost == dist[city][newStamina] &&
                 stamina > prevStamina[city][newStamina])
        {
            // Update only if the stamina is better for equal cost
            prevCity[city][newStamina] = city;
            prevStamina[city][newStamina] = stamina;
        }
    }
}

// Attempts to move to adjacent cities and updates the priority
// queue if the new cost is better
static void tryMoveDijkstra(Agent agent, int city, int stamina, int cost,
                            int **dist, int **prevCity, int **prevStamina,
                            Pq pq)
{
    Map map = agent->map;
    int numCities = MapNumCities(map);
    struct road *roads = malloc(numCities * sizeof(struct road));
    int numRoads = MapGetRoadsFrom(map, city, roads);

    for (int i = 0; i < numRoads; i++)
    {
        int nextCity = roads[i].to;
        int roadLength = roads[i].length;
        if (roadLength <= stamina)
        {
            // Reduce stamina by road length
            int newStamina = stamina - roadLength;
            // Moving costs 1 turn
            int newCost = cost + 1;
            if (newCost < dist[nextCity][newStamina])
            {
                dist[nextCity][newStamina] = newCost;
                prevCity[nextCity][newStamina] = city;
                prevStamina[nextCity][newStamina] = stamina;
                PqInsert(pq, (struct PQNode){newCost, nextCity, newStamina});
            }
            else if (newCost == dist[nextCity][newStamina] &&
                     newStamina > prevStamina[nextCity][newStamina])
            {
                // Update only if the stamina is better for equal cost
                prevCity[nextCity][newStamina] = city;
                prevStamina[nextCity][newStamina] = stamina;
            }
        }
    }
    free(roads);
}

// Reconstructs the least-turns path to the target city
static void reconstructPath(Agent agent, int targetCity, int **dist,
                            int **prevCity, int **prevStamina)
{
    int minCost = INT_MAX;
    int endStamina = NO_STAMINA;
    for (int stamina = 0; stamina <= agent->maxStamina; stamina++)
    {
        if (dist[targetCity][stamina] < minCost)
        {
            minCost = dist[targetCity][stamina];
        }
        else if (dist[targetCity][stamina] == minCost &&
                 stamina > endStamina)
        {
        }
        endStamina = stamina;
    }

    if (minCost == INT_MAX)
    {
        agent->hasTipOff = false;
        return;
    }

    int pathLength = minCost;
    if (agent->path != NULL)
    {
        free(agent->path);
        agent->path = NULL;
    }
    struct move *path = malloc(sizeof(struct move) * pathLength);
    int idx = pathLength - 1;
    int city = targetCity;
    int stamina = endStamina;

    while (city != NO_CITY && idx >= 0)
    {
        int prevCityValue = prevCity[city][stamina];
        int prevStaminaValue = prevStamina[city][stamina];

        if (prevCityValue == NO_PREVIOUS)
            break;
        if (prevCityValue == city)
        {
            path[idx] = (struct move){city, 0};
        }
        else
        {
            int staminaCost = prevStaminaValue - stamina;
            path[idx] = (struct move){city, staminaCost};
        }
        city = prevCityValue;
        stamina = prevStaminaValue;
        idx--;
    }
    agent->path = path;
    agent->pathLength = pathLength;
    agent->pathIndex = 0;
}

////////////////////////////////////////////////////////////////////////
// Displaying state

// Prints information about the agent (for debugging purposes)
void AgentShow(Agent agent)
{
}

////////////////////////////////////////////////////////////////////////
// Pq used in stage 3

// Creates a new priority queue with default capacity
// This implementation was adapted from the course-provided week08 file:
// https://cgi.cse.unsw.edu.au/~cs2521/24T3/labs/week08/files/Pq.c
static Pq PqNew(void)
{
    Pq pq = malloc(sizeof(*pq));
    if (pq == NULL)
    {
        fprintf(stderr, "Couldn't allocate Pq!\n");
        exit(EXIT_FAILURE);
    }

    pq->items = malloc((PQ_DEFAULT_CAPACITY + 1) * sizeof(struct PQNode));
    if (pq->items == NULL)
    {
        fprintf(stderr, "Couldn't allocate Pq!\n");
        exit(EXIT_FAILURE);
    }

    pq->size = 0;
    pq->capacity = PQ_DEFAULT_CAPACITY;
    return pq;
}

static void PqFree(Pq pq)
{
    free(pq->items);
    free(pq);
}

// Inserts a node into the priority queue, expanding its capacity if needed
// This implementation was adapted from the course-provided week08 file:
// https://cgi.cse.unsw.edu.au/~cs2521/24T3/labs/week08/files/Pq.c
static void PqInsert(Pq pq, struct PQNode node)
{
    // If the Pq is full, expand it (i.e., double its capacity)
    if (pq->size == pq->capacity)
    {
        pq->capacity *= 2;
        pq->items =
            realloc(pq->items, (pq->capacity + 1) * sizeof(struct PQNode));
        if (pq->items == NULL)
        {
            fprintf(stderr, "Couldn't expand Pq!\n");
            exit(EXIT_FAILURE);
        }
    }

    // Add the new item to the end
    pq->size++;
    pq->items[pq->size] = node;
    fixUp(pq, pq->size);
}

static void fixUp(Pq pq, int i)
{
    while (i > 1 && pq->items[i].cost < pq->items[i / 2].cost)
    {
        swap(pq->items, i, i / 2);
        i = i / 2;
    }
}

// Extracts the minimum cost node from the priority queue
// This implementation was adapted from the course-provided week08 file:
// https://cgi.cse.unsw.edu.au/~cs2521/24T3/labs/week08/files/Pq.c
static struct PQNode PqExtract(Pq pq)
{
    assert(pq->size > 0);

    struct PQNode node = pq->items[1];
    pq->items[1] = pq->items[pq->size];
    pq->size--;
    fixDown(pq, 1);
    return node;
}

static void fixDown(Pq pq, int i)
{
    struct PQNode *nodes = pq->items;
    while (true)
    {
        int j = i;
        int l = 2 * i;
        int r = 2 * i + 1;
        if (l <= pq->size && nodes[l].cost < nodes[j].cost)
            j = l;
        if (r <= pq->size && nodes[r].cost < nodes[j].cost)
            j = r;
        if (j == i)
            break;
        swap(pq->items, i, j);
        i = j;
    }
}

static void swap(struct PQNode *items, int i, int j)
{
    struct PQNode tmp = items[i];
    items[i] = items[j];
    items[j] = tmp;
}