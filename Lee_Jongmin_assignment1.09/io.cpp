#include <unistd.h>
#include <ncurses.h>
#include <ctype.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>

#include "io.h"
#include "character.h"
#include "poke327.h"
#include "pokemon.h"
#include "db_parse.h"

typedef struct io_message {
  /* Will print " --more-- " at end of line when another message follows. *
   * Leave 10 extra spaces for that.                                      */
  char msg[71];
  struct io_message *next;
} io_message_t;

static io_message_t *io_head, *io_tail;

void io_init_terminal(void)
{
  initscr();
  raw();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  start_color();
  init_pair(COLOR_RED, COLOR_RED, COLOR_BLACK);
  init_pair(COLOR_GREEN, COLOR_GREEN, COLOR_BLACK);
  init_pair(COLOR_YELLOW, COLOR_YELLOW, COLOR_BLACK);
  init_pair(COLOR_BLUE, COLOR_BLUE, COLOR_BLACK);
  init_pair(COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLACK);
  init_pair(COLOR_CYAN, COLOR_CYAN, COLOR_BLACK);
  init_pair(COLOR_WHITE, COLOR_WHITE, COLOR_BLACK);
}

void io_reset_terminal(void)
{
  endwin();

  while (io_head) {
    io_tail = io_head;
    io_head = io_head->next;
    free(io_tail);
  }
  io_tail = NULL;
}

void io_queue_message(const char *format, ...)
{
  io_message_t *tmp;
  va_list ap;

  if (!(tmp = (io_message_t *) malloc(sizeof (*tmp)))) {
    perror("malloc");
    exit(1);
  }

  tmp->next = NULL;

  va_start(ap, format);

  vsnprintf(tmp->msg, sizeof (tmp->msg), format, ap);

  va_end(ap);

  if (!io_head) {
    io_head = io_tail = tmp;
  } else {
    io_tail->next = tmp;
    io_tail = tmp;
  }
}

static void io_print_message_queue(uint32_t y, uint32_t x)
{
  while (io_head) {
    io_tail = io_head;
    attron(COLOR_PAIR(COLOR_CYAN));
    mvprintw(y, x, "%-80s", io_head->msg);
    attroff(COLOR_PAIR(COLOR_CYAN));
    io_head = io_head->next;
    if (io_head) {
      attron(COLOR_PAIR(COLOR_CYAN));
      mvprintw(y, x + 70, "%10s", " --more-- ");
      attroff(COLOR_PAIR(COLOR_CYAN));
      refresh();
      getch();
    }
    free(io_tail);
  }
  io_tail = NULL;
}

/**************************************************************************
 * Compares trainer distances from the PC according to the rival distance *
 * map.  This gives the approximate distance that the PC must travel to   *
 * get to the trainer (doesn't account for crossing buildings).  This is  *
 * not the distance from the NPC to the PC unless the NPC is a rival.     *
 *                                                                        *
 * Not a bug.                                                             *
 **************************************************************************/
static int compare_trainer_distance(const void *v1, const void *v2)
{
  const Character *const *c1 = (const Character *const *) v1;
  const Character *const *c2 = (const Character *const *) v2;

  return (world.rival_dist[(*c1)->pos[dim_y]][(*c1)->pos[dim_x]] -
          world.rival_dist[(*c2)->pos[dim_y]][(*c2)->pos[dim_x]]);
}

static Character *io_nearest_visible_trainer()
{
  Character **c, *n;
  uint32_t x, y, count;

  c = (Character **) malloc(world.cur_map->num_trainers * sizeof (*c));

  /* Get a linear list of trainers */
  for (count = 0, y = 1; y < MAP_Y - 1; y++) {
    for (x = 1; x < MAP_X - 1; x++) {
      if (world.cur_map->cmap[y][x] && world.cur_map->cmap[y][x] !=
          &world.pc) {
        c[count++] = world.cur_map->cmap[y][x];
      }
    }
  }

  /* Sort it by distance from PC */
  qsort(c, count, sizeof (*c), compare_trainer_distance);

  n = c[0];

  free(c);

  return n;
}

void io_display()
{
  uint32_t y, x;
  Character *c;

  clear();
  for (y = 0; y < MAP_Y; y++) {
    for (x = 0; x < MAP_X; x++) {
      if (world.cur_map->cmap[y][x]) {
        mvaddch(y + 1, x, world.cur_map->cmap[y][x]->symbol);
      } else {
        switch (world.cur_map->map[y][x]) {
        case ter_boulder:
        case ter_mountain:
          attron(COLOR_PAIR(COLOR_MAGENTA));
          mvaddch(y + 1, x, '%');
          attroff(COLOR_PAIR(COLOR_MAGENTA));
          break;
        case ter_tree:
        case ter_forest:
          attron(COLOR_PAIR(COLOR_GREEN));
          mvaddch(y + 1, x, '^');
          attroff(COLOR_PAIR(COLOR_GREEN));
          break;
        case ter_path:
        case ter_exit:
          attron(COLOR_PAIR(COLOR_YELLOW));
          mvaddch(y + 1, x, '#');
          attroff(COLOR_PAIR(COLOR_YELLOW));
          break;
        case ter_mart:
          attron(COLOR_PAIR(COLOR_BLUE));
          mvaddch(y + 1, x, 'M');
          attroff(COLOR_PAIR(COLOR_BLUE));
          break;
        case ter_center:
          attron(COLOR_PAIR(COLOR_RED));
          mvaddch(y + 1, x, 'C');
          attroff(COLOR_PAIR(COLOR_RED));
          break;
        case ter_grass:
          attron(COLOR_PAIR(COLOR_GREEN));
          mvaddch(y + 1, x, ':');
          attroff(COLOR_PAIR(COLOR_GREEN));
          break;
        case ter_clearing:
          attron(COLOR_PAIR(COLOR_GREEN));
          mvaddch(y + 1, x, '.');
          attroff(COLOR_PAIR(COLOR_GREEN));
          break;
        default:
 /* Use zero as an error symbol, since it stands out somewhat, and it's *
  * not otherwise used.                                                 */
          attron(COLOR_PAIR(COLOR_CYAN));
          mvaddch(y + 1, x, '0');
          attroff(COLOR_PAIR(COLOR_CYAN)); 
       }
      }
    }
  }

  mvprintw(23, 1, "PC position is (%2d,%2d) on map %d%cx%d%c.",
           world.pc.pos[dim_x],
           world.pc.pos[dim_y],
           abs(world.cur_idx[dim_x] - (WORLD_SIZE / 2)),
           world.cur_idx[dim_x] - (WORLD_SIZE / 2) >= 0 ? 'E' : 'W',
           abs(world.cur_idx[dim_y] - (WORLD_SIZE / 2)),
           world.cur_idx[dim_y] - (WORLD_SIZE / 2) <= 0 ? 'N' : 'S');
  mvprintw(22, 1, "%d known %s.", world.cur_map->num_trainers,
           world.cur_map->num_trainers > 1 ? "trainers" : "trainer");
  mvprintw(22, 30, "Nearest visible trainer: ");
  if ((c = io_nearest_visible_trainer())) {
    attron(COLOR_PAIR(COLOR_RED));
    mvprintw(22, 55, "%c at %d %c by %d %c.",
             c->symbol,
             abs(c->pos[dim_y] - world.pc.pos[dim_y]),
             ((c->pos[dim_y] - world.pc.pos[dim_y]) <= 0 ?
              'N' : 'S'),
             abs(c->pos[dim_x] - world.pc.pos[dim_x]),
             ((c->pos[dim_x] - world.pc.pos[dim_x]) <= 0 ?
              'W' : 'E'));
    attroff(COLOR_PAIR(COLOR_RED));
  } else {
    attron(COLOR_PAIR(COLOR_BLUE));
    mvprintw(22, 55, "NONE.");
    attroff(COLOR_PAIR(COLOR_BLUE));
  }

  io_print_message_queue(0, 0);

  refresh();
}

uint32_t io_teleport_pc(pair_t dest)
{
  /* Just for fun. And debugging.  Mostly debugging. */

  do {
    dest[dim_x] = rand_range(1, MAP_X - 2);
    dest[dim_y] = rand_range(1, MAP_Y - 2);
  } while (world.cur_map->cmap[dest[dim_y]][dest[dim_x]]                  ||
           move_cost[char_pc][world.cur_map->map[dest[dim_y]]
                                                [dest[dim_x]]] == INT_MAX ||
           world.rival_dist[dest[dim_y]][dest[dim_x]] < 0);

  return 0;
}

static void io_scroll_trainer_list(char (*s)[40], uint32_t count)
{
  uint32_t offset;
  uint32_t i;

  offset = 0;

  while (1) {
    for (i = 0; i < 13; i++) {
      mvprintw(i + 6, 19, " %-40s ", s[i + offset]);
    }
    switch (getch()) {
    case KEY_UP:
      if (offset) {
        offset--;
      }
      break;
    case KEY_DOWN:
      if (offset < (count - 13)) {
        offset++;
      }
      break;
    case 27:
      return;
    }

  }
}

static void io_list_trainers_display(Npc **c,
                                     uint32_t count)
{
  uint32_t i;
  char (*s)[40]; /* pointer to array of 40 char */

  s = (char (*)[40]) malloc(count * sizeof (*s));

  mvprintw(3, 19, " %-40s ", "");
  /* Borrow the first element of our array for this string: */
  snprintf(s[0], 40, "You know of %d trainers:", count);
  mvprintw(4, 19, " %-40s ", s[0]);
  mvprintw(5, 19, " %-40s ", "");

  for (i = 0; i < count; i++) {
    snprintf(s[i], 40, "%16s %c: %2d %s by %2d %s",
             char_type_name[c[i]->ctype],
             c[i]->symbol,
             abs(c[i]->pos[dim_y] - world.pc.pos[dim_y]),
             ((c[i]->pos[dim_y] - world.pc.pos[dim_y]) <= 0 ?
              "North" : "South"),
             abs(c[i]->pos[dim_x] - world.pc.pos[dim_x]),
             ((c[i]->pos[dim_x] - world.pc.pos[dim_x]) <= 0 ?
              "West" : "East"));
    if (count <= 13) {
      /* Handle the non-scrolling case right here. *
       * Scrolling in another function.            */
      mvprintw(i + 6, 19, " %-40s ", s[i]);
    }
  }

  if (count <= 13) {
    mvprintw(count + 6, 19, " %-40s ", "");
    mvprintw(count + 7, 19, " %-40s ", "Hit escape to continue.");
    while (getch() != 27 /* escape */)
      ;
  } else {
    mvprintw(19, 19, " %-40s ", "");
    mvprintw(20, 19, " %-40s ",
             "Arrows to scroll, escape to continue.");
    io_scroll_trainer_list(s, count);
  }

  free(s);
}

static void io_list_trainers()
{
  Character **c;
  uint32_t x, y, count;

  c = (Character **) malloc(world.cur_map->num_trainers * sizeof (*c));

  /* Get a linear list of trainers */
  for (count = 0, y = 1; y < MAP_Y - 1; y++) {
    for (x = 1; x < MAP_X - 1; x++) {
      if (world.cur_map->cmap[y][x] && world.cur_map->cmap[y][x] !=
          &world.pc) {
        c[count++] = world.cur_map->cmap[y][x];
      }
    }
  }

  /* Sort it by distance from PC */
  qsort(c, count, sizeof (*c), compare_trainer_distance);

  /* Display it */
  io_list_trainers_display((Npc **)(c), count);
  free(c);

  /* And redraw the map */
  io_display();
}

void io_pokemart()
{
  mvprintw(0, 0, "Welcome to the Pokemart.  Could I interest you in some Pokeballs?");
  refresh();
  getch();
}

void io_pokemon_center()
{
  mvprintw(0, 0, "Welcome to the Pokemon Center.  How can Nurse Joy assist you?");
  refresh();
  getch();
}



uint32_t move_pc_dir(uint32_t input, pair_t dest)
{
  dest[dim_y] = world.pc.pos[dim_y];
  dest[dim_x] = world.pc.pos[dim_x];

  switch (input) {
  case 1:
  case 2:
  case 3:
    dest[dim_y]++;
    break;
  case 4:
  case 5:
  case 6:
    break;
  case 7:
  case 8:
  case 9:
    dest[dim_y]--;
    break;
  }
  switch (input) {
  case 1:
  case 4:
  case 7:
    dest[dim_x]--;
    break;
  case 2:
  case 5:
  case 8:
    break;
  case 3:
  case 6:
  case 9:
    dest[dim_x]++;
    break;
  case '>':
    if (world.cur_map->map[world.pc.pos[dim_y]][world.pc.pos[dim_x]] ==
        ter_mart) {
      io_pokemart();
    }
    if (world.cur_map->map[world.pc.pos[dim_y]][world.pc.pos[dim_x]] ==
        ter_center) {
      io_pokemon_center();
    }
    break;
  }

  if ((world.cur_map->map[dest[dim_y]][dest[dim_x]] == ter_exit) &&
      (input == 1 || input == 3 || input == 7 || input == 9)) {
    // Exiting diagonally leads to complicated entry into the new map
    // in order to avoid INT_MAX move costs in the destination.
    // Most easily solved by disallowing such entries here.
    return 1;
  }

  if (world.cur_map->cmap[dest[dim_y]][dest[dim_x]]) {
    if (dynamic_cast<Npc *>(world.cur_map->cmap[dest[dim_y]][dest[dim_x]]) &&
        ((Npc *) world.cur_map->cmap[dest[dim_y]][dest[dim_x]])->defeated) {
      // Some kind of greeting here would be nice
      return 1;
    } else if (dynamic_cast<Npc *>
               (world.cur_map->cmap[dest[dim_y]][dest[dim_x]])) {
      io_battle(world.cur_map->cmap[dest[dim_y]][dest[dim_x]]);
      // Not actually moving, so set dest back to PC position
      dest[dim_x] = world.pc.pos[dim_x];
      dest[dim_y] = world.pc.pos[dim_y];
    }
  }
  
  if (move_cost[char_pc][world.cur_map->map[dest[dim_y]][dest[dim_x]]] ==
      INT_MAX) {
    return 1;
  }

  return 0;
}

void io_teleport_world(pair_t dest)
{
  int x, y;
  
  world.cur_map->cmap[world.pc.pos[dim_y]][world.pc.pos[dim_x]] = NULL;

  mvprintw(0, 0, "Enter x [-200, 200]: ");
  refresh();
  echo();
  curs_set(1);
  mvscanw(0, 21, (char *) "%d", &x);
  mvprintw(0, 0, "Enter y [-200, 200]:          ");
  refresh();
  mvscanw(0, 21, (char *) "%d", &y);
  refresh();
  noecho();
  curs_set(0);

  if (x < -200) {
    x = -200;
  }
  if (x > 200) {
    x = 200;
  }
  if (y < -200) {
    y = -200;
  }
  if (y > 200) {
    y = 200;
  }
  
  x += 200;
  y += 200;

  world.cur_idx[dim_x] = x;
  world.cur_idx[dim_y] = y;

  new_map(1);
  io_teleport_pc(dest);
}

bool attempt_run(int t_speed, int w_speed, int attempts){
  int odds;
  odds = (((t_speed * 32) / ((w_speed/4) % 256)) + 30*attempts);
  int prob = rand() % 256;

  if(prob < odds){
    return true;
  } else{
    return false;
  }
}

bool is_alive(int index){
  if(world.pc.pokemon[index]){
    if(world.pc.pokemon[index]->cur_hp > 0){
      return true;
    } else{
      return false;
    }
  }

  return false;
}

void io_list_pokemon(){
  clear();
  mvprintw(0,0,"Pokemon:");

  int i;
  for(i = 0; i<6; i++){
    if(world.pc.pokemon[i]){
      if(is_alive(i)){
        mvprintw(i+3,0,"%d) %s: HP: %d",i+1, world.pc.pokemon[i]->get_species(), world.pc.pokemon[i]->cur_hp);
      }else{
        mvprintw(i+3,0,"%d) %s: Knocked Out!",i+1, world.pc.pokemon[i]->get_species());
      }
    }else{
      break;
    }
  }
}

int io_switch_pokemon_forced(){
  clear();
  io_list_pokemon();
  mvprintw(1,0, "Select which pokemon to switch to");
  refresh();
  bool deciding = true;
  char c;
  while(deciding){
    c = getch();
    switch(c){
      case '1':
        if(world.pc.pokemon[0]){
          if(world.pc.pokemon[0]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 0;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '2':
        if(world.pc.pokemon[1]){
          if(world.pc.pokemon[1]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 1;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '3':
        if(world.pc.pokemon[2]){
          if(world.pc.pokemon[2]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 2;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '4':
        if(world.pc.pokemon[3]){
          if(world.pc.pokemon[3]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 3;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '5':
        if(world.pc.pokemon[4]){
          if(world.pc.pokemon[4]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 4;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '6':
        if(world.pc.pokemon[5]){
          if(world.pc.pokemon[5]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 5;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
    }
  }
  return 0;
}

int io_switch_pokemon(int cur_index){
  clear();
  io_list_pokemon();
  mvprintw(1,0, "Select which pokemon to switch to");
  mvprintw(15,0, "Current pokemon: %s", world.pc.pokemon[cur_index]->get_species());
  mvprintw(16,0, "Level: %d    HP: %d/%d", world.pc.pokemon[cur_index]->get_level(), world.pc.pokemon[cur_index]->cur_hp, world.pc.pokemon[cur_index]->get_hp());
  mvprintw(21,0, "ESC to go back");
  refresh();
  bool deciding = true;
  char c;
  while(deciding){
    c = getch();
    switch(c){
      case '1':
        if(world.pc.pokemon[0] && cur_index != 0){
          if(world.pc.pokemon[0]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 0;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '2':
        if(world.pc.pokemon[1] && cur_index != 1){
          if(world.pc.pokemon[1]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 1;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '3':
        if(world.pc.pokemon[2] && cur_index != 2){
          if(world.pc.pokemon[2]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 2;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '4':
        if(world.pc.pokemon[3] && cur_index != 3){
          if(world.pc.pokemon[3]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 3;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '5':
        if(world.pc.pokemon[4] && cur_index != 4){
          if(world.pc.pokemon[4]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 4;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case '6':
        if(world.pc.pokemon[5] && cur_index != 5){
          if(world.pc.pokemon[5]->cur_hp < 1){
            mvprintw(19,0,"Cannot Switch to a knocked out pokemon");
          }else{
            deciding = false;
            return 5;
          }
        } else{
          mvprintw(20,0,"Please select a valid pokemon");
        }
        break;
      case 27:
        deciding = false;
        return -1;
        break;
    }
  }
  return 0;
}

int get_move_damage(double level, double power, double attack, double defense, double speed, double stab){
  double crit = 1;
  if(rand_range(0,255) < (speed/2)){
    crit = 1.5;
  }
  double random = rand_range(85,100) / 100.0;

  // mvprintw(15,70,"LVL: %3.3f",level);
  // mvprintw(16,70,"POW: %3.3f",power);
  // mvprintw(17,70,"ATK: %3.3f",attack);
  // mvprintw(18,70,"DEF: %3.3f",defense);
  // mvprintw(19,70,"SPD: %3.3f",speed);
  // mvprintw(20,70,"RNG: %d",random);
  // mvprintw(21,70,"CRT: %3.3f",crit);
  //refresh();
  return floor(((((((2.0*level)/5.0 +2.0) * power * (attack/defense)) / 50.0) + 2.0) * crit * random * stab * 1.0));
}

int do_move(int p_move, Pokemon *pc_poke, Pokemon *enemy){
  clear();
  mvprintw(0,0, "%s", pc_poke->get_species());
  mvprintw(1,0, "Level: %d    HP: %d/%d", pc_poke->get_level(), pc_poke->cur_hp, pc_poke->get_hp());
  mvprintw(0,40, "%s", enemy->get_species());
  mvprintw(1,40, "Level: %d    HP: %d/%d", enemy->get_level(), enemy->cur_hp, enemy->get_hp());
  mvprintw(21,0, "Press any Key to continue...");
  refresh();
  int i;
  int enemy_move_count = 0;
  for(i = 0; i<4; i++){
    if(enemy->get_move(i)[0] != '\0') {
      enemy_move_count++;
    }
  }
  int rand_move = rand_range(0, enemy_move_count-1);
  int enemy_move = enemy->get_move_id(rand_move);
  bool pc_first = false;

  bool pc_miss = true; //Default true for "missing" a non-attack move
  bool enemy_miss = true;

  if(p_move > 0){ //PLAYER DOES AN ACTUAL ATTACK
    int pc_pri = moves[p_move].priority;
    int enemy_pri = moves[enemy_move].priority;

    if(pc_pri == enemy_pri){
      int pc_speed = pc_poke->get_speed();
      int enemy_speed = enemy->get_speed();
      if(pc_speed == enemy_speed){
        if(rand()%2){
          pc_first = true;
        } else{
          pc_first = false;
        }
      } else if(pc_speed > enemy_speed){
        pc_first = true;
      } else{
        pc_first = false;
      }
    } else if(pc_pri > enemy_pri){
      pc_first = true;
    } else{
      pc_first = false;
    }
    if(rand() % 100 < moves[p_move].accuracy || moves[p_move].accuracy == -1){ //I WASNT SURE WHAT TO DO ABOUT NON ASSIGNED VALUES SO I JUST ASSUMED A HIT
      pc_miss = false;
    }

  }
  
  if(rand() % 100 < moves[enemy_move].accuracy || moves[enemy_move].accuracy == -1){
    enemy_miss = false;
  }

  if(pc_first){
    if(p_move != 0){
      mvprintw(2,0, "%s, USE %s", pc_poke->get_species(), moves[p_move].identifier);
      refresh();
      getch();
    }

    if(!pc_miss){
      int type = (pokemon_types[pc_poke->get_species_id()].type_id);
      double stab = 1.0;
      if (moves[p_move].type_id == type){
        stab = 1.5;
      } else{
        stab = 1.0;
      }
      int damage = get_move_damage(pc_poke->get_level() * 1.0, moves[p_move].power * 1.0, pc_poke->get_atk() * 1.0, enemy->get_def() * 1.0, pc_poke->get_speed() * 1.0, stab); //TODO PARSE IN STAB
      mvprintw(10,0, "%s Hit for %d!", moves[p_move].identifier, damage);
      refresh();
      getch();

      enemy->cur_hp -= damage;
      if(enemy->cur_hp <= 0){
        enemy->cur_hp = 0;
        mvprintw(11,40, "%s Was knocked out!", enemy->get_species());
        refresh();
        getch();
        return 1;
      }
    } else{
      if(p_move != 0){
        mvprintw(10,0, "%s Missed!", moves[p_move].identifier);
        refresh();
        getch();
      }
      
    }
    
    mvprintw(2,40, "%s, USES %s", enemy->get_species(), moves[enemy_move].identifier);
    refresh();
    getch();

    if(!enemy_miss){
      int damage_incoming = get_move_damage(enemy->get_level() * 1.0, moves[enemy_move].power * 1.0, enemy->get_atk() * 1.0, pc_poke->get_def() * 1.0, enemy->get_speed() * 1.0, 1.0); //TODO PARSE IN STAB
      mvprintw(10,40, "%s Hit for %d!", moves[enemy_move].identifier, damage_incoming);
      refresh();
      getch();

      pc_poke->cur_hp -= damage_incoming;
      if(pc_poke->cur_hp <= 0){
        pc_poke->cur_hp = 0;
        mvprintw(11,0, "%s Was knocked out!", pc_poke->get_species());
        refresh();
        getch();
        return 2;
      }

    } else{
      mvprintw(10,40, "%s Missed!", moves[enemy_move].identifier);
      refresh();
      getch();
    }

  } else{ //ENEMY FIRST
    mvprintw(2,40, "%s, USES %s", enemy->get_species(), moves[enemy_move].identifier);
    refresh();
    getch();

    if(!enemy_miss){
      int damage_incoming = get_move_damage(enemy->get_level() * 1.0, moves[enemy_move].power * 1.0, enemy->get_atk() * 1.0, pc_poke->get_def() * 1.0, enemy->get_speed() * 1.0, 1.0); //TODO PARSE IN STAB
      mvprintw(10,40, "%s Hit for %d!", moves[enemy_move].identifier, damage_incoming);
      refresh();
      getch();

      pc_poke->cur_hp -= damage_incoming;
      if(pc_poke->cur_hp <= 0){
        pc_poke->cur_hp = 0;
        mvprintw(11,0, "%s Was knocked out!", pc_poke->get_species());
        refresh();
        getch();
        return 2;
      }

    } else{
      mvprintw(10,40, "%s Missed!", moves[enemy_move].identifier);
      refresh();
      getch();
    }

    if(p_move != 0){
      mvprintw(2,0, "%s, USE %s", pc_poke->get_species(), moves[p_move].identifier);
      refresh();
      getch();
    }

    if(!pc_miss){
      int damage = get_move_damage(pc_poke->get_level() * 1.0, moves[p_move].power * 1.0, pc_poke->get_atk() * 1.0, enemy->get_def() * 1.0, pc_poke->get_speed() * 1.0, 1.0); //TODO PARSE IN STAB
      mvprintw(10,0, "%s Hit for %d!", moves[p_move].identifier, damage);
      refresh();
      getch();

      enemy->cur_hp -= damage;
      if(enemy->cur_hp <= 0){
        enemy->cur_hp = 0;
        mvprintw(11,40, "%s Was knocked out!", enemy->get_species());
        refresh();
        getch();
        return 1;
      }
    } else{
      if(p_move != 0){
        mvprintw(10,0, "%s Missed!", moves[p_move].identifier);
        refresh();
        getch();
      }
    }
  }

  return 0;
}

void attempt_capture(Pokemon *p){
  int i;
  for(i = 0; i<6; i++){
    if(!world.pc.pokemon[i]){
      break;
    }
  }
  if(i < 6){
    world.pc.pokemon[i] = p;
  }
}

int io_post_knockout_logic(){
  clear();
  int i;
  for(i = 0; i<6; i++){
    if(is_alive(i)){
      break;
    }
  }
  if(i == 6){
    mvprintw(0,0,"Your Last Pokemon has been knocked out :( you can revive them from your bag!, better luck next time");
    mvprintw(21,0, "Press any Key to continue...");
    refresh();
    getch();
    return -1; //NO AVAILABLE POKEMON TO USE, EXIT AND CONTINUE AS NORMAL
  } else{
    mvprintw(0,0,"Your Pokemon has been knocked out, Please pick a new one!");
    mvprintw(21,0, "Press any Key to continue...");
    refresh();
    getch();
    return io_switch_pokemon_forced();
  }

}

int io_fight(Pokemon* pc_poke, Pokemon *enemy, bool wild){
  char c;
  clear();
  mvprintw(0,0,"Choose your move!");
  mvprintw(15,0, "Current Pokemon: %s", pc_poke->get_species());
  mvprintw(16,0, "Level: %d    HP: %d/%d", pc_poke->get_level(), pc_poke->cur_hp, pc_poke->get_hp());
  mvprintw(15,40, "Current Opponent: %s", enemy->get_species());
  mvprintw(16,40, "Level: %d    HP: %d/%d", enemy->get_level(), enemy->cur_hp, enemy->get_hp());
  mvprintw(21,0, "ESC to go back");
  bool deciding = true;
  
  int i;
  int move_count = 0;

  for(i = 0; i<4; i++){
    if(pc_poke->get_move(i)[0] != '\0') {
      move_count++;
      mvprintw((i+1)*2,0, "%d) %s", i+1, pc_poke->get_move(i));
    }
  }
  int move_result = 0;
  refresh();
  while(deciding){
    refresh();
    c = getch();

    switch(c){
      case '1':
        move_result = do_move(pc_poke->get_move_id(0),pc_poke, enemy);
        if(move_result == 1 && wild){
          attempt_capture(enemy);
          clear();
          mvprintw(0,0,"%s Will be added to your bag if there is space!", enemy->get_species());
          mvprintw(21,0, "Press any Key to continue...");
          refresh();
          getch();
          return 100;
        } else if(move_result == 1){
            return 100;
        } else if(move_result == 2){
          int ko_res = 0;
          ko_res = io_post_knockout_logic();
          if(ko_res == -1){
            return 99;
          } else{
            return ko_res;
          }
        }
        deciding = false;
        return -1;
        break;
      case '2':
        if(move_count > 1){
          move_result = do_move(pc_poke->get_move_id(1),pc_poke, enemy);
          if(move_result == 1 && wild){
            attempt_capture(enemy);
            clear();
            mvprintw(0,0,"%s Will be added to your bag if there is space!", enemy->get_species());
            mvprintw(21,0, "Press any Key to continue...");
            refresh();
            getch();
            return 100;
          }  else if(move_result == 1){
            return 100;
          } else if(move_result == 2){
            int ko_res = 0;
            ko_res = io_post_knockout_logic();
            if(ko_res == -1){
              return 99;
            } else{
              return ko_res;
            }
          }
          deciding = false;
          return -1;
        } else{
          mvprintw(20,0, "Please choose a valid move");
        }
        break;
      case '3':
        if(move_count > 2){
          move_result = do_move(pc_poke->get_move_id(2),pc_poke, enemy);
          if(move_result == 1 && wild){
            attempt_capture(enemy);
            clear();
            mvprintw(0,0,"%s Will be added to your bag if there is space!", enemy->get_species());
            mvprintw(21,0, "Press any Key to continue...");
            refresh();
            getch();
            return 100;
          } else if(move_result == 1){
            return 100;
          } else if(move_result == 2){
            int ko_res = 0;
            ko_res = io_post_knockout_logic();
            if(ko_res == -1){
              return 99;
            } else{
              return ko_res;
            }
          }
          deciding = false;
          return -1;
        } else{
          mvprintw(20,0, "Please choose a valid move");
        }
        break;
      case '4':
        if(move_count > 3){
          move_result = do_move(pc_poke->get_move_id(3),pc_poke, enemy);
          if(move_result == 1 && wild){
            attempt_capture(enemy);
            clear();
            mvprintw(0,0,"%s Will be added to your bag if there is space!", enemy->get_species());
            mvprintw(21,0, "Press any Key to continue...");
            refresh();
            getch();
            return 100;
          } else if(move_result == 1){
              return 100;
          } else if(move_result == 2){
            int ko_res = 0;
            ko_res = io_post_knockout_logic();
            if(ko_res == -1){
              return 99;
            } else{
              return ko_res;
            }
          }
          deciding = false;
          return -1;
        } else{
          mvprintw(20,0, "Please choose a valid move");
        }
        break;
      case 27:
        deciding = false;
        return -1;
        break;
      default:
        mvprintw(20,0, "Please choose a valid move");
        break;
    }
  }
  
  return 0;
}

int io_enter_bag(bool in_wild_battle){
  clear();

  bool in_bag = true;
  char c;
  while(in_bag){
    mvprintw(0,0,"Welcome to your Bag!");
    mvprintw(1,0,"Please select which item you would like to use");

    mvprintw(3,0,"1) Revives: %d", world.pc.bag[revive]);
    mvprintw(4,0,"2) Potions: %d", world.pc.bag[potion]);
    mvprintw(5,0,"3) Pokeballs: %d (Capture if in battle)", world.pc.bag[pokeball]);
    mvprintw(21,0, "ESC to leave bag");
    refresh();

    c = getch();
    if(c == '1' && world.pc.bag[revive] > 0){
      clear();
      io_list_pokemon();
      mvprintw(1,0,"Please select which pokemon you would like to revive");
      mvprintw(21,0, "ESC to go back");
      refresh();
      bool deciding = true;
      int rev_target = 0;
      while(deciding){
        c = getch();
        switch(c){
          case '1':
            if(world.pc.pokemon[0]){
              if(world.pc.pokemon[0]->cur_hp > 0){
                mvprintw(19,0,"Cannot use revive on non-knocked pokemon");
              }else{
                deciding = false;
                rev_target = 0;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '2':
            if(world.pc.pokemon[1]){
              if(world.pc.pokemon[1]->cur_hp > 0){
                mvprintw(19,0,"Cannot use revive on non-knocked pokemon");
              }else{
                deciding = false;
                rev_target = 1;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '3':
            if(world.pc.pokemon[2]){
              if(world.pc.pokemon[2]->cur_hp > 0){
                mvprintw(19,0,"Cannot use revive on non-knocked pokemon");
              }else{
                deciding = false;
                rev_target = 2;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '4':
            if(world.pc.pokemon[3]){
              if(world.pc.pokemon[3]->cur_hp > 0){
                mvprintw(19,0,"Cannot use revive on non-knocked pokemon");
              }else{
                deciding = false;
                rev_target = 3;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '5':
            if(world.pc.pokemon[4]){
              if(world.pc.pokemon[4]->cur_hp > 0){
                mvprintw(19,0,"Cannot use revive on non-knocked pokemon");
              }else{
                deciding = false;
                rev_target = 4;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '6':
            if(world.pc.pokemon[5]){
              if(world.pc.pokemon[5]->cur_hp > 0){
                mvprintw(19,0,"Cannot use revive on non-knocked pokemon");
              }else{
                deciding = false;
                rev_target = 5;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case 27:
            deciding = false;
            rev_target = -1;
        }
      }
      if(rev_target != -1){
        int rev_hp = world.pc.pokemon[rev_target]->get_hp() / 2;
        world.pc.pokemon[rev_target]->cur_hp += rev_hp;
        world.pc.bag[revive]--;
        return 1;
      }
      clear();

    }else if(c == '2' && world.pc.bag[potion] > 0){
      clear();
      io_list_pokemon();
      mvprintw(1,0,"Please select which pokemon you would like to use the potion on");
      mvprintw(21,0, "ESC to go back");
      refresh();
      bool deciding = true;
      int pot_target = 0;
      while(deciding){
        c = getch();
        switch(c){
          case '1':
            if(world.pc.pokemon[0]){
              if(world.pc.pokemon[0]->cur_hp == world.pc.pokemon[0]->get_hp()){
                mvprintw(19,0,"Cannot use Potion on fully healed pokemon");
              }else{
                deciding = false;
                pot_target = 0;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '2':
            if(world.pc.pokemon[1]){
              if(world.pc.pokemon[1]->cur_hp == world.pc.pokemon[1]->get_hp()){
                mvprintw(19,0,"Cannot use Potion on fully healed pokemon");
              }else{
                deciding = false;
                pot_target = 1;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '3':
            if(world.pc.pokemon[2]){
              if(world.pc.pokemon[2]->cur_hp == world.pc.pokemon[2]->get_hp()){
                mvprintw(19,0,"Cannot use Potion on fully healed pokemon");
              }else{
                deciding = false;
                pot_target = 2;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '4':
            if(world.pc.pokemon[3]){
              if(world.pc.pokemon[3]->cur_hp == world.pc.pokemon[3]->get_hp()){
                mvprintw(19,0,"Cannot use Potion on fully healed pokemon");
              }else{
                deciding = false;
                pot_target = 3;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '5':
            if(world.pc.pokemon[4]){
              if(world.pc.pokemon[4]->cur_hp == world.pc.pokemon[4]->get_hp()){
                mvprintw(19,0,"Cannot use Potion on fully healed pokemon");
              }else{
                deciding = false;
                pot_target = 4;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case '6':
            if(world.pc.pokemon[5]){
              if(world.pc.pokemon[5]->cur_hp == world.pc.pokemon[5]->get_hp()){
                mvprintw(19,0,"Cannot use Potion on fully healed pokemon");
              }else{
                deciding = false;
                pot_target = 5;
              }
            } else{
              mvprintw(20,0,"Please select a valid pokemon");
            }
            break;
          case 27:
            deciding = false;
            pot_target = -1;
        }
      }
      if(pot_target != -1){
        if(world.pc.pokemon[pot_target]->cur_hp + 20 > world.pc.pokemon[pot_target]->get_hp()){
          world.pc.pokemon[pot_target]->cur_hp = world.pc.pokemon[pot_target]->get_hp();
        }else{
          world.pc.pokemon[pot_target]->cur_hp += 20;
        }
        world.pc.bag[potion]--;
        return 1;
      }
      clear();

    }else if(c == '3' && world.pc.bag[pokeball] > 0){
      if(in_wild_battle){
        if(world.pc.bag[pokeball] > 0){
          world.pc.bag[pokeball]--;
          return 2; //Attempt capture
        }else{
          clear();
          mvprintw(20,0, "No pokeballs availible :(");
        }
      } else{
        clear();
        mvprintw(20,0, "Use of pokeballs only availible in battle with wild pokemon!");
      }

    }else if(c == 27){
      in_bag = false; //redundant
      return 0;
    }else{
      clear();
      mvprintw(20,0,"Invalid command");
    }
  }

  return 0; //just looking
}



void io_encounter_pokemon()
{
  bool fighting = true;
  int attempts = 0;
  int cur_pokemon = -1;

  int i;
  for(i = 0; i<6; i++){
    if(is_alive(i)){
      cur_pokemon = i;
      break;
    }
  }
  if(cur_pokemon == -1){
    return; //NO AVAILABLE POKEMON TO USE, EXIT AND CONTINUE AS NORMAL
  }
  
  Pokemon *p;
  
  int md = (abs(world.cur_idx[dim_x] - (WORLD_SIZE / 2)) +
            abs(world.cur_idx[dim_x] - (WORLD_SIZE / 2)));
  int minl, maxl;
  
  if (md <= 200) {
    minl = 1;
    maxl = md / 2;
  } else {
    minl = (md - 200) / 2;
    maxl = 100;
  }
  if (minl < 1) {
    minl = 1;
  }
  if (minl > 100) {
    minl = 100;
  }
  if (maxl < 1) {
    maxl = 1;
  }
  if (maxl > 100) {
    maxl = 100;
  }

  p = new Pokemon(rand() % (maxl - minl + 1) + minl);

  //  std::cerr << *p << std::endl << std::endl;
  /*
  io_queue_message("%s%s%s: HP:%d ATK:%d DEF:%d SPATK:%d SPDEF:%d SPEED:%d %s",
                   p->is_shiny() ? "*" : "", p->get_species(),
                   p->is_shiny() ? "*" : "", p->get_hp(), p->get_atk(),
                   p->get_def(), p->get_spatk(), p->get_spdef(),
                   p->get_speed(), p->get_gender_string());
  io_queue_message("%s's moves: %s %s", p->get_species(),
                   p->get_move(0), p->get_move(1));
  */


  clear();
  while(fighting){
    mvprintw(0, 0, "A wild %s appeared!", p->get_species());
    mvprintw(15,0, "Current Pokemon: %s", world.pc.pokemon[cur_pokemon]->get_species());
    mvprintw(16,0, "Level: %d    HP: %d/%d", world.pc.pokemon[cur_pokemon]->get_level(), world.pc.pokemon[cur_pokemon]->cur_hp, world.pc.pokemon[cur_pokemon]->get_hp());
    mvprintw(15,40, "Opponent Pokemon: %s", p->get_species());
    mvprintw(16,40, "Level: %d    HP: %d/%d", p->get_level(), p->cur_hp, p->get_hp());
    mvprintw(3, 0, "1) Fight");
    mvprintw(4, 0, "2) Bag");
    mvprintw(5, 0, "3) Run");
    mvprintw(6, 0, "4) Switch Pokemon");
    refresh(); 
    
    char c = getch();

    if(c == '1'){ //fight
      clear();
      int fight_result = -1;
      fight_result = io_fight(world.pc.pokemon[cur_pokemon], p, true);
      if(fight_result >=0 && fight_result < 6){
        cur_pokemon = fight_result;
      } else if(fight_result > 6){
        fighting = false;
        return;
      }
      clear();
    }else if(c == '2'){ //bag
      clear();
      int decision;
      decision = io_enter_bag(true);
      if(decision == 2){
        attempt_capture(p);
        fighting = false;
      } else if(decision == 1){
        //TODO OPPONENT MOVE
      } else if(decision == 0){
        //DONT DO OPPONENT MOVE
      }
      clear();
      refresh();

    }else if(c == '3'){ //run
      if(attempt_run(world.pc.pokemon[cur_pokemon]->get_speed(), p->get_speed(), attempts) == true){
        fighting = false;
      } else{
        attempts++;
        clear();
        mvprintw(19, 0, "Escape Failed!");
        mvprintw(21,0, "Press any Key to continue...");
        refresh();
        getch();
        do_move(0, world.pc.pokemon[cur_pokemon], p);
        clear();
      }
    }else if(c == '4'){ //switch pokemon
      clear();
      int ret = -1;
      ret = io_switch_pokemon(cur_pokemon);
      if(ret == -1){
        //Dont do a move
      } else{ //switch the current pokemon out and do opponent move
        cur_pokemon = ret;
        mvprintw(19, 0, "Pokemon Swtiched!");
        mvprintw(21,0, "Press any Key to continue...");
        refresh();
        getch();
        do_move(0, world.pc.pokemon[cur_pokemon], p);
        clear();
      }
      clear();
      refresh();

    } else if(c == 'Q'){
        clear();
        io_display();
        refresh();
        return;
      } else{
      clear();
      mvprintw(20, 0, "Invalid Command");
      refresh();
    }
  }

  // Later on, don't delete if captured
  //delete p;
}

void gen_trainer_pokemon(Npc *npc){
  int num_pokes = 0;
  int r;
  do{
    num_pokes++;
    r = rand() % 100;
  }while(r < 60 && num_pokes < 6);

  int i;
  for(i = 0; i < num_pokes; i++){
    Pokemon *p;
    int md = (abs(world.cur_idx[dim_x] - (WORLD_SIZE / 2)) +
              abs(world.cur_idx[dim_x] - (WORLD_SIZE / 2)));
    int minl, maxl;
    
    if (md <= 200) {
      minl = 1;
      maxl = md / 2;
    } else {
      minl = (md - 200) / 2;
      maxl = 100;
    }
    if (minl < 1) {
      minl = 1;
    }
    if (minl > 100) {
      minl = 100;
    }
    if (maxl < 1) {
      maxl = 1;
    }
    if (maxl > 100) {
      maxl = 100;
    }

    p = new Pokemon(rand() % (maxl - minl + 1) + minl);
    npc->pokemon[i] = p;
  }
  
}
bool is_enemy_poke_alive(Pokemon *p){
  if(p->cur_hp > 0){
    return true;
  } else{
    return false;
  }
}

void display_opponent_pokes(Npc *enemy){
  mvprintw(0,50,"Opponents Pokemon:");

  int i;
  for(i = 0; i<6; i++){
    if(enemy->pokemon[i]){
      if(is_enemy_poke_alive(enemy->pokemon[i])){
        mvprintw(i+3,50,"%s: HP %d/%d", enemy->pokemon[i]->get_species(), enemy->pokemon[i]->cur_hp, enemy->pokemon[i]->get_hp());
      }else{
        mvprintw(i+3,50,"%s: Knocked Out!", enemy->pokemon[i]->get_species());
      }
    }else{
      break;
    }
  }
}

int get_next_enemy_poke(Character *enemy){
  int i;
  int alive = -1;
  for(i = 0; i<6; i++){
    if(enemy->pokemon[i]){
      if(is_enemy_poke_alive(enemy->pokemon[i])){
        alive = i;
        break;
      } 
    } else{
      break;
    }
  }
  return alive;
}

void io_battle(Character *enemy)
{
  bool fighting = true;
  int cur_pokemon = -1;
  

  int i;
  for(i = 0; i<6; i++){
    if(is_alive(i)){
      cur_pokemon = i;
      break;
    }
  }
  if(cur_pokemon == -1){
    return; //NO AVAILABLE POKEMON TO USE, EXIT AND CONTINUE AS NORMAL
  }

  Npc *npc;

  // io_display();
  // mvprintw(0, 0, "Aww, how'd you get so strong?  You and your pokemon must share a special bond!");
  // refresh();
  // getch();
  
  npc = dynamic_cast<Npc *>(enemy);

  if(!npc->pokemon[0]){
    gen_trainer_pokemon(npc);
  }
  int enemy_poke = -1;
  enemy_poke = get_next_enemy_poke(enemy);

  clear();
  while(fighting){
    mvprintw(0, 0, "You have been challenged by a Trainer to a dual!");
    mvprintw(15,0, "Current Pokemon: %s", world.pc.pokemon[cur_pokemon]->get_species());
    mvprintw(16,0, "Level: %d    HP: %d/%d", world.pc.pokemon[cur_pokemon]->get_level(), world.pc.pokemon[cur_pokemon]->cur_hp, world.pc.pokemon[cur_pokemon]->get_hp());
    mvprintw(3, 0, "1) Fight");
    mvprintw(4, 0, "2) Bag");
    mvprintw(5, 0, "3) Switch Pokemon");
    display_opponent_pokes(npc);
    refresh(); 
    
    char c = getch();

    if(c == '1'){ //fight
      int fight_result = -1;
      fight_result = io_fight(world.pc.pokemon[cur_pokemon], enemy->pokemon[enemy_poke], false);
      if(fight_result >=0 && fight_result < 6){
        cur_pokemon = fight_result;
      } if(fight_result == 100){
        enemy_poke = get_next_enemy_poke(enemy);
        if(enemy_poke == -1){
          npc->defeated = 1;
          if (npc->ctype == char_hiker || npc->ctype == char_rival) {
            npc->mtype = move_wander;
          }
          return;
        }
      }else if(fight_result > 6){
        fighting = false;
        return;
      }
      clear();
    }else if(c == '2'){ //bag
      clear();
      int decision;
      decision = io_enter_bag(false);
      if(decision == 1){
        //TODO OPPONENT MOVE
      } else if(decision == 0){
        //DONT DO OPPONENT MOVE
      }
      clear();
      refresh();

    }else if(c == '3'){ //switch pokemon
      clear();
      int ret = -1;
      ret = io_switch_pokemon(cur_pokemon);
      if(ret == -1){
        //Dont do a move
      } else{ //switch the current pokemon out and do opponent move
        cur_pokemon = ret;
        mvprintw(19, 0, "Pokemon Swtiched!");
        mvprintw(21,0, "Press any Key to continue...");
        refresh();
        getch();
        do_move(0, world.pc.pokemon[cur_pokemon], enemy->pokemon[enemy_poke]);
        clear();
      }
      clear();
      refresh();

    } else if(c == 'Q'){
        npc->defeated = 1; //For debugging stuff
        if (npc->ctype == char_hiker || npc->ctype == char_rival) {
          npc->mtype = move_wander;
        }
        clear();
        io_display();
        refresh();
        return;
      }else{
      clear();
      mvprintw(20, 0, "Invalid Command");
      refresh();
    }
  }
  
  


}

void io_handle_input(pair_t dest)
{
  uint32_t turn_not_consumed;
  int key;

  do {
    switch (key = getch()) {
    case '7':
    case 'y':
    case KEY_HOME:
      turn_not_consumed = move_pc_dir(7, dest);
      break;
    case '8':
    case 'k':
    case KEY_UP:
      turn_not_consumed = move_pc_dir(8, dest);
      break;
    case '9':
    case 'u':
    case KEY_PPAGE:
      turn_not_consumed = move_pc_dir(9, dest);
      break;
    case '6':
    case 'l':
    case KEY_RIGHT:
      turn_not_consumed = move_pc_dir(6, dest);
      break;
    case '3':
    case 'n':
    case KEY_NPAGE:
      turn_not_consumed = move_pc_dir(3, dest);
      break;
    case '2':
    case 'j':
    case KEY_DOWN:
      turn_not_consumed = move_pc_dir(2, dest);
      break;
    case '1':
    case 'b':
    case KEY_END:
      turn_not_consumed = move_pc_dir(1, dest);
      break;
    case '4':
    case 'h':
    case KEY_LEFT:
      turn_not_consumed = move_pc_dir(4, dest);
      break;
    case '5':
    case ' ':
    case '.':
    case KEY_B2:
      dest[dim_y] = world.pc.pos[dim_y];
      dest[dim_x] = world.pc.pos[dim_x];
      turn_not_consumed = 0;
      break;
    case '>':
      turn_not_consumed = move_pc_dir('>', dest);
      break;
    case 'B':
      io_enter_bag(false);
      clear();
      io_display();
      refresh();
      break;
    case 'Q':
      dest[dim_y] = world.pc.pos[dim_y];
      dest[dim_x] = world.pc.pos[dim_x];
      world.quit = 1;
      turn_not_consumed = 0;
      break;
      break;
    case 't':
      /* Teleport the PC to a random place in the map.              */
      io_teleport_pc(dest);
      turn_not_consumed = 0;
      break;
    case 'T':
      /* Teleport the PC to any map in the world.                   */
      io_teleport_world(dest);
      turn_not_consumed = 0;
      break;
    case 'm':
      io_list_trainers();
      turn_not_consumed = 1;
      break;
    case 'q':
      /* Demonstrate use of the message queue.  You can use this for *
       * printf()-style debugging (though gdb is probably a better   *
       * option.  Not that it matters, but using this command will   *
       * waste a turn.  Set turn_not_consumed to 1 and you should be *
       * able to figure out why I did it that way.                   */
      io_queue_message("This is the first message.");
      io_queue_message("Since there are multiple messages, "
                       "you will see \"more\" prompts.");
      io_queue_message("You can use any key to advance through messages.");
      io_queue_message("Normal gameplay will not resume until the queue "
                       "is empty.");
      io_queue_message("Long lines will be truncated, not wrapped.");
      io_queue_message("io_queue_message() is variadic and handles "
                       "all printf() conversion specifiers.");
      io_queue_message("Did you see %s?", "what I did there");
      io_queue_message("When the last message is displayed, there will "
                       "be no \"more\" prompt.");
      io_queue_message("Have fun!  And happy printing!");
      io_queue_message("Oh!  And use 'Q' to quit!");

      dest[dim_y] = world.pc.pos[dim_y];
      dest[dim_x] = world.pc.pos[dim_x];
      turn_not_consumed = 0;
      break;
    default:
      /* Also not in the spec.  It's not always easy to figure out what *
       * key code corresponds with a given keystroke.  Print out any    *
       * unhandled key here.  Not only does it give a visual error      *
       * indicator, but it also gives an integer value that can be used *
       * for that key in this (or other) switch statements.  Printed in *
       * octal, with the leading zero, because ncurses.h lists codes in *
       * octal, thus allowing us to do reverse lookups.  If a key has a *
       * name defined in the header, you can use the name here, else    *
       * you can directly use the octal value.                          */
      mvprintw(0, 0, "Unbound key: %#o ", key);
      turn_not_consumed = 1;
    }
    refresh();
  } while (turn_not_consumed);
}
