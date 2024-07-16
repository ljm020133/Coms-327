#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <endian.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <limits.h>
#include <sys/time.h>
#include <assert.h>
#include <stdbool.h>

#include "heap.h"

#define malloc(size) ({          \
  void *_tmp;                    \
  assert((_tmp = malloc(size))); \
  _tmp;                          \
})

typedef struct path {
  heap_node_t *hn;
  uint8_t pos[2];
  uint8_t from[2];
  int32_t cost;
} path_t;


typedef enum dim {
  dim_x,
  dim_y,
  num_dims
} dim_t;

typedef uint8_t pair_t[num_dims];

#define MAP_X              80
#define MAP_Y              21
#define MIN_TREES          10
#define MIN_BOULDERS       10
#define TREE_PROB          95
#define BOULDER_PROB       95
#define WORLD_SIZE         401

#define mappair(pair) (m->map[pair[dim_y]][pair[dim_x]])
#define mapxy(x, y) (m->map[y][x])
#define heightpair(pair) (m->height[pair[dim_y]][pair[dim_x]])
#define heightxy(x, y) (m->height[y][x])

typedef enum __attribute__ ((__packed__)) terrain_type {
  ter_debug,
  ter_boulder,
  ter_tree,
  ter_path,
  ter_mart,
  ter_center,
  ter_grass,
  ter_clearing,
  ter_mountain,
  ter_forest,
  ter_pc
} terrain_type_t;

typedef struct PC {
  int x, y;
} PC_t;


typedef struct map {
  terrain_type_t map[MAP_Y][MAP_X];
  int hikerMap[MAP_Y][MAP_X];
  int rivalMap[MAP_Y][MAP_X];
  uint8_t height[MAP_Y][MAP_X];
  uint8_t n, s, e, w;
  PC_t pc;
} map_t;


typedef struct world {
  map_t *world[399][399];
  int curX, curY;
} world_t;


typedef struct queue_node {
  int x, y;
  struct queue_node *next;
} queue_node_t;

static int32_t path_cmp(const void *key, const void *with) {
  return ((path_t *) key)->cost - ((path_t *) with)->cost;
}

static int32_t edge_penalty(uint8_t x, uint8_t y)
{
  return (x == 1 || y == 1 || x == MAP_X - 2 || y == MAP_Y - 2) ? 2 : 1;
}

static void dijkstra_path(map_t *m, pair_t from, pair_t to)
{
  static path_t path[MAP_Y][MAP_X], *p;
  static uint32_t initialized = 0;
  heap_t h;
  uint32_t x, y;

  if (!initialized) {
    for (y = 0; y < MAP_Y; y++) {
      for (x = 0; x < MAP_X; x++) {
        path[y][x].pos[dim_y] = y;
        path[y][x].pos[dim_x] = x;
      }
    }
    initialized = 1;
  }
  
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      path[y][x].cost = INT_MAX;
    }
  }

  path[from[dim_y]][from[dim_x]].cost = 0;

  heap_init(&h, path_cmp, NULL);

  for (y = 1; y < MAP_Y - 1; y++) {
    for (x = 1; x < MAP_X - 1; x++) {
      path[y][x].hn = heap_insert(&h, &path[y][x]);
    }
  }

  while ((p = heap_remove_min(&h))) {
    p->hn = NULL;

    if ((p->pos[dim_y] == to[dim_y]) && p->pos[dim_x] == to[dim_x]) {
      for (x = to[dim_x], y = to[dim_y];
           (x != from[dim_x]) || (y != from[dim_y]);
           p = &path[y][x], x = p->from[dim_x], y = p->from[dim_y]) {
        mapxy(x, y) = ter_path;
        heightxy(x, y) = 0;
      }
      heap_delete(&h);
      return;
    }

    if ((path[p->pos[dim_y] - 1][p->pos[dim_x]    ].hn) &&
        (path[p->pos[dim_y] - 1][p->pos[dim_x]    ].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[dim_x], p->pos[dim_y] - 1)))) {
      path[p->pos[dim_y] - 1][p->pos[dim_x]    ].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[dim_x], p->pos[dim_y] - 1));
      path[p->pos[dim_y] - 1][p->pos[dim_x]    ].from[dim_y] = p->pos[dim_y];
      path[p->pos[dim_y] - 1][p->pos[dim_x]    ].from[dim_x] = p->pos[dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[dim_y] - 1]
                                           [p->pos[dim_x]    ].hn);
    }
    if ((path[p->pos[dim_y]    ][p->pos[dim_x] - 1].hn) &&
        (path[p->pos[dim_y]    ][p->pos[dim_x] - 1].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[dim_x] - 1, p->pos[dim_y])))) {
      path[p->pos[dim_y]][p->pos[dim_x] - 1].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[dim_x] - 1, p->pos[dim_y]));
      path[p->pos[dim_y]    ][p->pos[dim_x] - 1].from[dim_y] = p->pos[dim_y];
      path[p->pos[dim_y]    ][p->pos[dim_x] - 1].from[dim_x] = p->pos[dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[dim_y]    ]
                                           [p->pos[dim_x] - 1].hn);
    }
    if ((path[p->pos[dim_y]    ][p->pos[dim_x] + 1].hn) &&
        (path[p->pos[dim_y]    ][p->pos[dim_x] + 1].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[dim_x] + 1, p->pos[dim_y])))) {
      path[p->pos[dim_y]][p->pos[dim_x] + 1].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[dim_x] + 1, p->pos[dim_y]));
      path[p->pos[dim_y]    ][p->pos[dim_x] + 1].from[dim_y] = p->pos[dim_y];
      path[p->pos[dim_y]    ][p->pos[dim_x] + 1].from[dim_x] = p->pos[dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[dim_y]    ]
                                           [p->pos[dim_x] + 1].hn);
    }
    if ((path[p->pos[dim_y] + 1][p->pos[dim_x]    ].hn) &&
        (path[p->pos[dim_y] + 1][p->pos[dim_x]    ].cost >
         ((p->cost + heightpair(p->pos)) *
          edge_penalty(p->pos[dim_x], p->pos[dim_y] + 1)))) {
      path[p->pos[dim_y] + 1][p->pos[dim_x]    ].cost =
        ((p->cost + heightpair(p->pos)) *
         edge_penalty(p->pos[dim_x], p->pos[dim_y] + 1));
      path[p->pos[dim_y] + 1][p->pos[dim_x]    ].from[dim_y] = p->pos[dim_y];
      path[p->pos[dim_y] + 1][p->pos[dim_x]    ].from[dim_x] = p->pos[dim_x];
      heap_decrease_key_no_replace(&h, path[p->pos[dim_y] + 1]
                                           [p->pos[dim_x]    ].hn);
    }
  }
}

int get_weight(map_t *m, int x, int y, char t) {
    int weight = 0;

    switch (t) {
        case 'h': // Hiker
            switch (m->map[y][x]) {
                case ter_boulder:
                    return INT_MAX;
                    break;
                case ter_tree:
                    return INT_MAX;
                    break;
                case ter_mart:
                    return INT_MAX;
                    break;
                case ter_center:
                    weight = INT_MAX;
                    break;
                case ter_path:
                    weight = 10;
                    break;
                case ter_grass:
                    weight = 15;
                    break;
                case ter_clearing:
                    weight = 10;
                    break;
                case ter_mountain:
                    weight = 15;
                    break;
                case ter_forest:
                    weight = 15;
                    break;
                case ter_debug:
                    // Debugging tile
                    weight = 0;
                    return weight;
                case ter_pc:
                    weight = INT_MAX;
                    break;
            }
            break;
        case 'r': // Rival
            switch (m->map[y][x]) {
                case ter_boulder:
                    weight = INT_MAX;
                    break;
                case ter_tree:
                    weight = INT_MAX;
                    break;
                case ter_mart:
                    weight = INT_MAX;
                    break;
                case ter_center:
                    weight = INT_MAX;
                    break;
                case ter_path:
                    weight = 10;
                    break;
                case ter_grass:
                    weight = 20;
                    break;
                case ter_clearing:
                    weight = 10;
                    break;
                case ter_mountain:
                    weight = INT_MAX;
                    break;
                case ter_forest:
                    weight = INT_MAX;
                    break;
                case ter_debug:
                    // Debugging tile
                    weight = 0;
                    return weight;
                case ter_pc:
                    weight = INT_MAX;
                    break;
            }
            break;
        case 'o':
            weight = 0;
            break;
        default:
            weight = 0; // Default weight if 't' is neither 'h', 'r', nor 'o'.
            break;
    }

    return weight;
}


void dijkstra_hiker(map_t *m)
{
	static path_t path[MAP_Y][MAP_X], *p;
  static uint32_t initialized = 0;
  heap_t h;
  uint32_t x, y;

  if (!initialized) {
    for (y = 0; y < MAP_Y; y++) {
      for (x = 0; x < MAP_X; x++) {
        path[y][x].pos[dim_y] = y;
        path[y][x].pos[dim_x] = x;
      }
    }
    initialized = 1;
  }
  
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      path[y][x].cost = INT_MAX;
      m->hikerMap[y][x] = INT_MAX;
    }
  } 

	path[m->pc.y][m->pc.x].cost = 0;

	heap_init(&h, path_cmp, NULL); // Initialize priority queue

	for (y = 0; y < MAP_Y; y++)
	{
		for (x = 0; x < MAP_X; x++)
		{
			if (m->map[y][x] != ter_boulder &&  m->map[y][x] != ter_tree && m->map[y][x] != ter_mart && m->map[y][x] != ter_center)
			{ 
				path[y][x].hn = heap_insert(&h, &path[y][x]);	
			}
			else
			{
				path[y][x].hn = NULL;
			}
		}
	}

	while ((p = heap_remove_min(&h)) && h.size > 0)
  {
	m->hikerMap[p->pos[dim_y]][p->pos[dim_x]] = p->cost;


		// Top - Left
    int check_cost = 0;
    if(get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] - 1, 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] - 1, 'h') + p->cost);
    }
		if(( path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].hn) &&
			(path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].cost > check_cost)){
				path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].cost = check_cost;
				path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x], p->pos[dim_y] - 1, 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x], p->pos[dim_y] - 1, 'h') + p->cost);
    }
		// Top
		if(( path[p->pos[dim_y] - 1][p->pos[dim_x]].hn) &&
			(path[p->pos[dim_y] - 1][p->pos[dim_x]].cost > check_cost)){
				path[p->pos[dim_y] - 1][p->pos[dim_x]].cost = check_cost;
				path[p->pos[dim_y] - 1][p->pos[dim_x]].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] - 1][p->pos[dim_x]].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] - 1][p->pos[dim_x]].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] - 1, 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] - 1, 'h') + p->cost);
    }
		// Top - Right
		if(( path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].hn) &&
			(path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].cost > check_cost)){
				path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].cost = check_cost;
				path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y], 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y], 'h') + p->cost);
    }
		// Left
		if(( path[p->pos[dim_y]][p->pos[dim_x] - 1].hn) &&
			(path[p->pos[dim_y]][p->pos[dim_x] - 1].cost > check_cost)){
				path[p->pos[dim_y]][p->pos[dim_x] - 1].cost = check_cost;
				path[p->pos[dim_y]][p->pos[dim_x] - 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y]][p->pos[dim_x] - 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y]][p->pos[dim_x] - 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y], 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y], 'h') + p->cost);
    }
		// Right
		if(( path[p->pos[dim_y]][p->pos[dim_x] + 1].hn) &&
			(path[p->pos[dim_y]][p->pos[dim_x] + 1].cost > check_cost)){
				path[p->pos[dim_y]][p->pos[dim_x] + 1].cost = check_cost;
				path[p->pos[dim_y]][p->pos[dim_x] + 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y]][p->pos[dim_x] + 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y]][p->pos[dim_x] + 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] + 1, 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] + 1, 'h') + p->cost);
    }
		// Bottom - Left
		if(( path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].hn) &&
			(path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].cost > check_cost)){
				path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].cost = check_cost;
				path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].hn);
		}


    check_cost = 0;
    if(get_weight(m, p->pos[dim_x], p->pos[dim_y] + 1, 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x], p->pos[dim_y] + 1, 'h') + p->cost);
    }
		// Bottom
		if(( path[p->pos[dim_y] + 1][p->pos[dim_x]].hn) &&
			(path[p->pos[dim_y] + 1][p->pos[dim_x]].cost > check_cost)){
				path[p->pos[dim_y] + 1][p->pos[dim_x]].cost = check_cost;
				path[p->pos[dim_y] + 1][p->pos[dim_x]].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] + 1][p->pos[dim_x]].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] + 1][p->pos[dim_x]].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] + 1, 'h') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] + 1, 'h') + p->cost);
    }
		// Bottom - Right
		if(( path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].hn) &&
			(path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].cost > check_cost)){
				path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].cost = check_cost;
				path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].hn);
		}
	}
	heap_delete(&h);
}


void dijkstra_rival(map_t *m)
{
	static path_t path[MAP_Y][MAP_X], *p;
  static uint32_t initialized = 0;
  heap_t h;
  uint32_t x, y;

  if (!initialized) {
    for (y = 0; y < MAP_Y; y++) {
      for (x = 0; x < MAP_X; x++) {
        path[y][x].pos[dim_y] = y;
        path[y][x].pos[dim_x] = x;
      }
    }
    initialized = 1;
  }
  
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      path[y][x].cost = INT_MAX;
      m->rivalMap[y][x] = INT_MAX;
    }
  } 

	path[m->pc.y][m->pc.x].cost = 0;

	heap_init(&h, path_cmp, NULL);

	for (y = 0; y < MAP_Y; y++)
	{
		for (x = 0; x < MAP_X; x++)
		{
			if (m->map[y][x] != ter_boulder && m->map[y][x] != ter_mountain && m->map[y][x] != ter_forest && m->map[y][x] != ter_tree && m->map[y][x] != ter_mart && m->map[y][x] != ter_center)
			{ 
				path[y][x].hn = heap_insert(&h, &path[y][x]);	
			}
			else
			{
				path[y][x].hn = NULL;
			}
		}
	}

	while ((p = heap_remove_min(&h)) && h.size > 0)
  {
	m->rivalMap[p->pos[dim_y]][p->pos[dim_x]] = p->cost;

		// Top - Left
    int check_cost = 0;
    if(get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] - 1, 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] - 1, 'r') + p->cost);
    }
		if(( path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].hn) &&
			(path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].cost > check_cost)){
				path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].cost = check_cost;
				path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] - 1][p->pos[dim_x] - 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x], p->pos[dim_y] - 1, 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x], p->pos[dim_y] - 1, 'r') + p->cost);
    }
		// Top
		if(( path[p->pos[dim_y] - 1][p->pos[dim_x]].hn) &&
			(path[p->pos[dim_y] - 1][p->pos[dim_x]].cost > check_cost)){
				path[p->pos[dim_y] - 1][p->pos[dim_x]].cost = check_cost;
				path[p->pos[dim_y] - 1][p->pos[dim_x]].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] - 1][p->pos[dim_x]].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] - 1][p->pos[dim_x]].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] - 1, 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] - 1, 'r') + p->cost);
    }
		// Top - Right
		if(( path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].hn) &&
			(path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].cost > check_cost)){
				path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].cost = check_cost;
				path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] - 1][p->pos[dim_x] + 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y], 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y], 'r') + p->cost);
    }
		// Left
		if(( path[p->pos[dim_y]][p->pos[dim_x] - 1].hn) &&
			(path[p->pos[dim_y]][p->pos[dim_x] - 1].cost > check_cost)){
				path[p->pos[dim_y]][p->pos[dim_x] - 1].cost = check_cost;
				path[p->pos[dim_y]][p->pos[dim_x] - 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y]][p->pos[dim_x] - 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y]][p->pos[dim_x] - 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y], 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y], 'r') + p->cost);
    }
		// Right
		if(( path[p->pos[dim_y]][p->pos[dim_x] + 1].hn) &&
			(path[p->pos[dim_y]][p->pos[dim_x] + 1].cost > check_cost)){
				path[p->pos[dim_y]][p->pos[dim_x] + 1].cost = check_cost;
				path[p->pos[dim_y]][p->pos[dim_x] + 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y]][p->pos[dim_x] + 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y]][p->pos[dim_x] + 1].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] + 1, 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] - 1, p->pos[dim_y] + 1, 'r') + p->cost);
    }
		// Bottom - Left
		if(( path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].hn) &&
			(path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].cost > check_cost)){
				path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].cost = check_cost;
				path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] + 1][p->pos[dim_x] - 1].hn);
		}


    check_cost = 0;
    if(get_weight(m, p->pos[dim_x], p->pos[dim_y] + 1, 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x], p->pos[dim_y] + 1, 'r') + p->cost);
    }
		// Bottom
		if(( path[p->pos[dim_y] + 1][p->pos[dim_x]].hn) &&
			(path[p->pos[dim_y] + 1][p->pos[dim_x]].cost > check_cost)){
				path[p->pos[dim_y] + 1][p->pos[dim_x]].cost = check_cost;
				path[p->pos[dim_y] + 1][p->pos[dim_x]].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] + 1][p->pos[dim_x]].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] + 1][p->pos[dim_x]].hn);
		}

    check_cost = 0;
    if(get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] + 1, 'r') == INT_MAX){
      check_cost = INT_MAX;
    } else{
      check_cost = (get_weight(m, p->pos[dim_x] + 1, p->pos[dim_y] + 1, 'r') + p->cost);
    }
		// Bottom - Right
		if(( path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].hn) &&
			(path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].cost > check_cost)){
				path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].cost = check_cost;
				path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].from[dim_y] = p->pos[dim_y];
				path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].from[dim_x] = p->pos[dim_x];
				heap_decrease_key_no_replace(&h, path[p->pos[dim_y] + 1][p->pos[dim_x] + 1].hn);
		}
	}
	heap_delete(&h);
}


static int build_paths(map_t *m)
{
  pair_t from, to;

  from[dim_x] = 1;
  to[dim_x] = MAP_X - 2;
  from[dim_y] = m->w;
  to[dim_y] = m->e;

  dijkstra_path(m, from, to);

  from[dim_y] = 1;
  to[dim_y] = MAP_Y - 2;
  from[dim_x] = m->n;
  to[dim_x] = m->s;

  dijkstra_path(m, from, to);

  return 0;
}

static int gaussian[5][5] = {
  {  1,  4,  7,  4,  1 },
  {  4, 16, 26, 16,  4 },
  {  7, 26, 41, 26,  7 },
  {  4, 16, 26, 16,  4 },
  {  1,  4,  7,  4,  1 }
};

static int smooth_height(map_t *m)
{
  int32_t i, x, y;
  int32_t s, t, p, q;
  queue_node_t *head, *tail, *tmp;
  /*  FILE *out;*/
  uint8_t height[MAP_Y][MAP_X];

  memset(&height, 0, sizeof (height));

  /* Seed with some values */
  for (i = 1; i < 255; i += 20) {
    do {
      x = rand() % MAP_X;
      y = rand() % MAP_Y;
    } while (height[y][x]);
    height[y][x] = i;
    if (i == 1) {
      head = tail = malloc(sizeof (*tail));
    } else {
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
    }
    tail->next = NULL;
    tail->x = x;
    tail->y = y;
  }

  /*
  out = fopen("seeded.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&height, sizeof (height), 1, out);
  fclose(out);
  */
  
  /* Diffuse the vaules to fill the space */
  while (head) {
    x = head->x;
    y = head->y;
    i = height[y][x];

    if (x - 1 >= 0 && y - 1 >= 0 && !height[y - 1][x - 1]) {
      height[y - 1][x - 1] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x - 1;
      tail->y = y - 1;
    }
    if (x - 1 >= 0 && !height[y][x - 1]) {
      height[y][x - 1] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x - 1;
      tail->y = y;
    }
    if (x - 1 >= 0 && y + 1 < MAP_Y && !height[y + 1][x - 1]) {
      height[y + 1][x - 1] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x - 1;
      tail->y = y + 1;
    }
    if (y - 1 >= 0 && !height[y - 1][x]) {
      height[y - 1][x] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x;
      tail->y = y - 1;
    }
    if (y + 1 < MAP_Y && !height[y + 1][x]) {
      height[y + 1][x] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x;
      tail->y = y + 1;
    }
    if (x + 1 < MAP_X && y - 1 >= 0 && !height[y - 1][x + 1]) {
      height[y - 1][x + 1] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x + 1;
      tail->y = y - 1;
    }
    if (x + 1 < MAP_X && !height[y][x + 1]) {
      height[y][x + 1] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x + 1;
      tail->y = y;
    }
    if (x + 1 < MAP_X && y + 1 < MAP_Y && !height[y + 1][x + 1]) {
      height[y + 1][x + 1] = i;
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
      tail->next = NULL;
      tail->x = x + 1;
      tail->y = y + 1;
    }

    tmp = head;
    head = head->next;
    free(tmp);
  }

  /* And smooth it a bit with a gaussian convolution */
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      for (s = t = p = 0; p < 5; p++) {
        for (q = 0; q < 5; q++) {
          if (y + (p - 2) >= 0 && y + (p - 2) < MAP_Y &&
              x + (q - 2) >= 0 && x + (q - 2) < MAP_X) {
            s += gaussian[p][q];
            t += height[y + (p - 2)][x + (q - 2)] * gaussian[p][q];
          }
        }
      }
      m->height[y][x] = t / s;
    }
  }
  /* Let's do it again, until it's smooth like Kenny G. */
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      for (s = t = p = 0; p < 5; p++) {
        for (q = 0; q < 5; q++) {
          if (y + (p - 2) >= 0 && y + (p - 2) < MAP_Y &&
              x + (q - 2) >= 0 && x + (q - 2) < MAP_X) {
            s += gaussian[p][q];
            t += height[y + (p - 2)][x + (q - 2)] * gaussian[p][q];
          }
        }
      }
      m->height[y][x] = t / s;
    }
  }

  /*
  out = fopen("diffused.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&height, sizeof (height), 1, out);
  fclose(out);

  out = fopen("smoothed.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&m->height, sizeof (m->height), 1, out);
  fclose(out);
  */

  return 0;
}

int randGen(int bound){
    return (rand() % bound);
}

static int spawn_chance(int x, int y){
  if(x == 199 && y == 199){
    return 100;
  }
  int man_distance = abs(x-199) + abs(y-199);
  int prob = (((-45*man_distance)/400)+50); 
  return prob;
}

static void find_building_location(map_t *m, pair_t p)
{
  do {
    p[dim_x] = rand() % (MAP_X - 5) + 3;
    p[dim_y] = rand() % (MAP_Y - 10) + 5;

    if ((((mapxy(p[dim_x] - 1, p[dim_y]    ) == ter_path)     &&
          (mapxy(p[dim_x] - 1, p[dim_y] + 1) == ter_path))    ||
         ((mapxy(p[dim_x] + 2, p[dim_y]    ) == ter_path)     &&
          (mapxy(p[dim_x] + 2, p[dim_y] + 1) == ter_path))    ||
         ((mapxy(p[dim_x]    , p[dim_y] - 1) == ter_path)     &&
          (mapxy(p[dim_x] + 1, p[dim_y] - 1) == ter_path))    ||
         ((mapxy(p[dim_x]    , p[dim_y] + 2) == ter_path)     &&
          (mapxy(p[dim_x] + 1, p[dim_y] + 2) == ter_path)))   &&
        (((mapxy(p[dim_x]    , p[dim_y]    ) != ter_mart)     &&
          (mapxy(p[dim_x]    , p[dim_y]    ) != ter_center)   &&
          (mapxy(p[dim_x] + 1, p[dim_y]    ) != ter_mart)     &&
          (mapxy(p[dim_x] + 1, p[dim_y]    ) != ter_center)   &&
          (mapxy(p[dim_x]    , p[dim_y] + 1) != ter_mart)     &&
          (mapxy(p[dim_x]    , p[dim_y] + 1) != ter_center)   &&
          (mapxy(p[dim_x] + 1, p[dim_y] + 1) != ter_mart)     &&
          (mapxy(p[dim_x] + 1, p[dim_y] + 1) != ter_center))) &&
        (((mapxy(p[dim_x]    , p[dim_y]    ) != ter_path)     &&
          (mapxy(p[dim_x] + 1, p[dim_y]    ) != ter_path)     &&
          (mapxy(p[dim_x]    , p[dim_y] + 1) != ter_path)     &&
          (mapxy(p[dim_x] + 1, p[dim_y] + 1) != ter_path)))) {
          break;
    }
  } while (1);
}

static int place_pokemart(world_t *w, map_t *m)
{
  int rand = randGen(100);
  int prob = spawn_chance(w->curX, w->curY);
  if(rand < prob){
    pair_t p;

    find_building_location(m, p);

    mapxy(p[dim_x]    , p[dim_y]    ) = ter_mart;
    mapxy(p[dim_x] + 1, p[dim_y]    ) = ter_mart;
    mapxy(p[dim_x]    , p[dim_y] + 1) = ter_mart;
    mapxy(p[dim_x] + 1, p[dim_y] + 1) = ter_mart;
    }
    return 0;
}

static int place_center(world_t *w, map_t *m)
{  
  int rand = randGen(100);
  int prob = spawn_chance(w->curX, w->curY);

  if(rand < prob){
    pair_t p;

    find_building_location(m, p);

    mapxy(p[dim_x]    , p[dim_y]    ) = ter_center;
    mapxy(p[dim_x] + 1, p[dim_y]    ) = ter_center;
    mapxy(p[dim_x]    , p[dim_y] + 1) = ter_center;
    mapxy(p[dim_x] + 1, p[dim_y] + 1) = ter_center;
    }
    return 0;
}

static int map_terrain(map_t *m, uint8_t n, uint8_t s, uint8_t e, uint8_t w)
{
  int32_t i, x, y;
  queue_node_t *head, *tail, *tmp;
  FILE *out;
  int num_grass, num_clearing, num_mountain, num_forest, num_total;
  terrain_type_t type;
  int added_current = 0;
  
  num_grass = rand() % 4 + 2;
  num_clearing = rand() % 4 + 2;
  num_mountain = rand() % 2 + 1;
  num_forest = rand() % 2 + 1;
  num_total = num_grass + num_clearing + num_mountain + num_forest;

  memset(&m->map, 0, sizeof (m->map));

  /* Seed with some values */
  for (i = 0; i < num_total; i++) {
    do {
      x = rand() % MAP_X;
      y = rand() % MAP_Y;
    } while (m->map[y][x]);
    if (i == 0) {
      type = ter_grass;
    } else if (i == num_grass) {
      type = ter_clearing;
    } else if (i == num_grass + num_clearing) {
      type = ter_mountain;
    } else if (i == num_grass + num_clearing + num_mountain) {
      type = ter_forest;
    }
    m->map[y][x] = type;
    if (i == 0) {
      head = tail = malloc(sizeof (*tail));
    } else {
      tail->next = malloc(sizeof (*tail));
      tail = tail->next;
    }
    tail->next = NULL;
    tail->x = x;
    tail->y = y;
  }

  out = fopen("seeded.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&m->map, sizeof (m->map), 1, out);
  fclose(out);

  /* Diffuse the vaules to fill the space */
  while (head) {
    x = head->x;
    y = head->y;
    i = m->map[y][x];
    
    if (x - 1 >= 0 && !m->map[y][x - 1]) {
      if ((rand() % 100) < 80) {
        m->map[y][x - 1] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x - 1;
        tail->y = y;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    if (y - 1 >= 0 && !m->map[y - 1][x]) {
      if ((rand() % 100) < 20) {
        m->map[y - 1][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y - 1;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    if (y + 1 < MAP_Y && !m->map[y + 1][x]) {
      if ((rand() % 100) < 20) {
        m->map[y + 1][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y + 1;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    if (x + 1 < MAP_X && !m->map[y][x + 1]) {
      if ((rand() % 100) < 80) {
        m->map[y][x + 1] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x + 1;
        tail->y = y;
      } else if (!added_current) {
        added_current = 1;
        m->map[y][x] = i;
        tail->next = malloc(sizeof (*tail));
        tail = tail->next;
        tail->next = NULL;
        tail->x = x;
        tail->y = y;
      }
    }

    added_current = 0;
    tmp = head;
    head = head->next;
    free(tmp);
  }

  out = fopen("diffused.pgm", "w");
  fprintf(out, "P5\n%u %u\n255\n", MAP_X, MAP_Y);
  fwrite(&m->map, sizeof (m->map), 1, out);
  fclose(out);

  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      if (y == 0 || y == MAP_Y - 1 ||
          x == 0 || x == MAP_X - 1) {
        mapxy(x, y) = ter_boulder;
      }
    }
  }

  m->n = n;
  m->s = s;
  m->e = e;
  m->w = w;

  mapxy(n,         0        ) = ter_path;
  mapxy(n,         1        ) = ter_path;
  mapxy(s,         MAP_Y - 1) = ter_path;
  mapxy(s,         MAP_Y - 2) = ter_path;
  mapxy(0,         w        ) = ter_path;
  mapxy(1,         w        ) = ter_path;
  mapxy(MAP_X - 1, e        ) = ter_path;
  mapxy(MAP_X - 2, e        ) = ter_path;

  return 0;
}

static int place_boulders(map_t *m)
{
  int i;
  int x, y;

  for (i = 0; i < MIN_BOULDERS || rand() % 100 < BOULDER_PROB; i++) {
    y = rand() % (MAP_Y - 2) + 1;
    x = rand() % (MAP_X - 2) + 1;
    if (m->map[y][x] != ter_forest) {
      m->map[y][x] = ter_boulder;
    }
  }

  return 0;
}

static int place_trees(map_t *m)
{
  int i;
  int x, y;
  
  for (i = 0; i < MIN_TREES || rand() % 100 < TREE_PROB; i++) {
    y = rand() % (MAP_Y - 2) + 1;
    x = rand() % (MAP_X - 2) + 1;
    if ((m->map[y][x] != ter_mountain) && (m->map[y][x] != ter_path)) {
      m->map[y][x] = ter_tree;
    }
  }

  return 0;
}

void place_pc(map_t *m){
  int rand_x = randGen(75) + 3;
  int y;

  for(y = 0; y < 21; y++){
    if(m->map[y][rand_x] == ter_path){
      m->map[y][rand_x] = ter_pc;
      break;
    }
  }

  
  PC_t pc;
  pc.x = rand_x;
  pc.y = y;
  m->pc = pc;
  m->map[m->pc.y][m->pc.x] = ter_pc;

}

static int new_map(world_t *w, map_t *m)
{
  smooth_height(m);
  uint8_t north, south, east, west;
  if (m->n != 0){
    north = m->n;
  } else{
    north = 1 + rand() % (MAP_X - 2);
  }
  if (m->s != 0){
    south = m->s;
  } else{
    south = 1 + rand() % (MAP_X - 2);
  }
  if (m->e != 0){
    east = m->e;
  } else{
    east = 1 + rand() % (MAP_Y - 2);
  }
  if (m->w != 0){
    west = m->w;
  } else{
    west = 1 + rand() % (MAP_Y - 2);
  }

  map_terrain(m, north, south, east, west);
  place_boulders(m);
  build_paths(m);
  place_trees(m);
  place_pokemart(w, m);
  place_center(w, m);
  place_pc(m);

  return 0;
}

static void print_map(map_t *m)
{
  int x, y;
  int default_reached = 0;
  
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      switch (m->map[y][x]) {
      case ter_pc:
        putchar('@');
        break;
      case ter_boulder:
      case ter_mountain:
        putchar('%');
        break;
      case ter_tree:
      case ter_forest:
        putchar('^');
        break;
      case ter_path:
        putchar('#');
        break;
      case ter_mart:
        putchar('M');
        break;
      case ter_center:
        putchar('C');
        break;
      case ter_grass:
        putchar(':');
        break;
      case ter_clearing:
        putchar('.');
        break;
      default:
        default_reached = 1;
        break;
      }
    }
    putchar('\n');
  }

  if (default_reached) {
    fprintf(stderr, "Default reached in %s\n", __FUNCTION__);
  }
}

static void print_hiker(map_t *m) {
    printf("\nHiker Distance Map:\n");
    for (int y = 0; y < MAP_Y; y++) {
        for (int x = 0; x < MAP_X; x++) {
            int distance = m->hikerMap[y][x];
            if (distance == INT_MAX) {
                printf("   ");
            } else {
                printf("%3d", distance % 100);
            }
        }
        printf("\n");
    }
}

static void print_rival(map_t *m) {
    printf("\nRival Distance Map:\n");
    for (int y = 0; y < MAP_Y; y++) {
        for (int x = 0; x < MAP_X; x++) {
            int distance = m->rivalMap[y][x];
            if (distance == INT_MAX) {
                printf("   ");
            } else {
                printf("%3d", distance % 100);
            }
        }
        printf("\n");
    }
}

bool move(world_t *wrld, uint8_t *n, uint8_t *s, uint8_t *e, uint8_t *w, int x, int y){
  if(x < 0 || x > 398 || y < 0 || y > 398){
    return false;
  }
  if(wrld->world[y][x] != 0){ // if the square is already discovered, return true and print the already printed map.
    return true;
  }

  if((y-1 >= 0 && y-1 <= 398) && wrld->world[y-1][x] != 0){ //check if north is discovered
    *n = ((wrld->world[y-1][x])->s);
  } else{
    *n = 0;
  }
  if((y+1 >= 0 && y+1 <= 398) && wrld->world[y+1][x] != 0){ //check if south is discovered
    *s = ((wrld->world[y+1][x])->n);
  } else{
    *s = 0;
  }
  if((x+1 >= 0 && x+1 <= 398) && wrld->world[y][x+1] != 0){ //check if east is discovered
    *e = ((wrld->world[y][x+1])->w);
  } else{
    *e = 0;
  }
  if((x-1 >= 0 && x-1 <= 398) && wrld->world[y][x-1] != 0){ //check if west is discovered
    *w = ((wrld->world[y][x-1])->e);
  } else{
    *w = 0;
  }
  return true;
}

void block_exits(map_t *m, int x, int y){
  int i;
  if(x == 0){
    for(i = 0; i < 21; i++){
      m->map[i][0] = ter_boulder;
    }
  }
  if(y == 0){
    for(i = 0; i < 80; i++){
      m->map[0][i] = ter_boulder;
    }
  }
  if(x == 398){
    for(i = 0; i < 21; i++){
      m->map[i][79] = ter_boulder;
    }
  }
  if(y == 398){
    for(i = 0; i < 80; i++){
      m->map[20][i] = ter_boulder;
    }
  }
}

int main(int argc, char *argv[])
{
  
  struct timeval tv;
  uint32_t seed;

  if (argc == 2) {
    seed = atoi(argv[1]);
  } else {
    gettimeofday(&tv, NULL);
    seed = (tv.tv_usec ^ (tv.tv_sec << 20)) & 0xffffffff;
  }

  printf("Using seed: %u\n", seed);
  srand(seed);
  world_t w;
  int i, j;
  for(i = 0; i < 399; i++){
    for(j = 0; j < 399; j++){
      w.world[i][j] = 0;
    }
  }
 
  map_t *d;

  
  d = malloc(sizeof(map_t));
  w.curX = 199;
  w.curY = 199;

  w.world[w.curY][w.curX] = d;

  d->n = 0;
  d->s = 0;
  d->w = 0;
  d->e = 0;

  new_map(&w, d);
  print_map(d);

  dijkstra_hiker(d);
  print_hiker(d);
  dijkstra_rival(d);
  print_rival(d);
  return 0;
}
