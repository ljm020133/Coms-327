#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ncurses.h>
#include <sys/stat.h>
#include <cstring>
#include <sstream>

class pokemon
{
    public:
        int id;
        std::string name;
        int species_id;
        int height;
        int weight;
        int base_xp;
        int order;
        int is_default;

    void clear()
    {
        id = -1;
        name = "";
        species_id = -1;
        height = -1;
        weight = -1;
        base_xp = -1;
        order = -1;
        is_default = -1;
    }

    void print()
    {
        std::cout << "id: " << id << std::endl;
        std::cout << "Name: " << name << std::endl;
        std::cout << "Species id: " << species_id << std::endl;
        std::cout << "Height: " << height << std::endl;
        std::cout << "Weight: " << weight << std::endl;
        std::cout << "Base XP: " << base_xp << std::endl;
        std::cout << "Order: " << order << std::endl;
        std::cout << "Is Default: " << is_default << std::endl;
        std::cout << std::endl;
    }
};

class moves
{
    public:
        int id;
        std::string name;
        int generation_id;
        int type_id;
        int power;
        int pp;
        int accuracy;
        int priority;
        int target_id;
        int damage_class_id;
        int effect_id;
        int effect_chance;
        int contest_type_id;
        int contest_effect_id;
        int super_contest_effect_id;

    void clear()
    {
        id = -1;
        name = "";
        generation_id = -1;
        type_id = -1;
        power = -1;
        pp = -1;
        accuracy = -1;
        priority = -1;
        target_id = -1;
        damage_class_id = -1;
        effect_id = -1;
        effect_chance = -1;
        contest_type_id = -1;
        contest_effect_id = -1;
        super_contest_effect_id = -1;
    }

    void print()
    {
        std::cout << "id: " << id << std::endl;
        std::cout << "Name: " << name << std::endl;
        std::cout << "Generation Id: " << generation_id << std::endl;
        std::cout << "Type Id: " << type_id << std::endl;
        std::cout << "Power: " << power << std::endl;
        std::cout << "PP: " << pp << std::endl;
        std::cout << "Accuracy: " << accuracy << std::endl;
        std::cout << "Priority: " << priority << std::endl;
        std::cout << "target_id: " << target_id << std::endl;
        std::cout << "damage_class_id: " << damage_class_id << std::endl;
        std::cout << "effect_id: " << effect_id << std::endl;
        std::cout << "effect_chance: " << effect_chance << std::endl;
        std::cout << "effect_chance: " << effect_chance << std::endl;
        std::cout << "contest_type_id: " << contest_type_id << std::endl;
        std::cout << "contest_effect_id: " << contest_effect_id << std::endl;
        std::cout << "super_contest_effect_id: " << super_contest_effect_id << std::endl;
        std::cout << std::endl;
    }
};

class pokemon_moves
{
    public:
        int pokemon_id;
        int version_group_id;
        int move_id;
        int pokemon_move_method_id;
        int level;
        int order;

    void clear()
    {
        pokemon_id = -1;
        version_group_id = -1;
        move_id = -1;
        pokemon_move_method_id = -1;
        level = -1;
        order = -1;
    }

    void print()
    {
        std::cout << "pokemon_id: " << pokemon_id << std::endl;
        std::cout << "version_group_id: " << version_group_id << std::endl;
        std::cout << "move_id: " << move_id << std::endl;
        std::cout << "pokemon_move_method_id: " << pokemon_move_method_id << std::endl;
        std::cout << "level: " << level << std::endl;
        std::cout << "order: " << order << std::endl;
        std::cout << std::endl;
    }
};

class pokemon_species
{
    public:
        int id;
        std::string name;
        int generation_id;
        int evolves_from_species_id;
        int evolution_chain_id;
        int color_id;
        int shape_id;
        int habit_id;
        int gender_rate;
        int capture_rate;
        int base_happiness;
        int is_baby;
        int hatch_counter;
        int has_gender_differences;
        int growth_rate_id;
        int forms_switchable;
        int is_legendary;
        int is_mythical;
        int order;
        int conquest_order;

    void clear()
    {
        id = -1;
        name = "";
        generation_id = -1;
        evolves_from_species_id = -1;
        evolution_chain_id = -1;
        color_id = -1;
        shape_id = -1;
        habit_id = -1;
        gender_rate = -1;
        capture_rate = -1;
        base_happiness = -1;
        is_baby = -1;
        hatch_counter = -1;
        has_gender_differences = -1;
        growth_rate_id = -1;
        forms_switchable = -1;
        is_legendary = -1;
        is_mythical = -1;
        order = -1;
        conquest_order = -1;
    }

    void print()
    {
        std::cout << "id: " << id << std::endl;
        std::cout << "Name: " << name << std::endl;
        std::cout << "Generation Id: " << generation_id << std::endl;
        std::cout << "evolves_from_species_id: " << evolves_from_species_id << std::endl;
        std::cout << "evolution_chain_id: " << evolution_chain_id << std::endl;
        std::cout << "color_id: " << color_id << std::endl;
        std::cout << "shape_id: " << shape_id << std::endl;
        std::cout << "habit_id: " << habit_id << std::endl;
        std::cout << "gender_rate: " << gender_rate << std::endl;
        std::cout << "capture_rate: " << capture_rate << std::endl;
        std::cout << "base_happiness: " << base_happiness << std::endl;
        std::cout << "is_baby: " << is_baby << std::endl;
        std::cout << "hatch_counter: " << hatch_counter << std::endl;
        std::cout << "has_gender_differences: " << has_gender_differences << std::endl;
        std::cout << "growth_rate_id: " << growth_rate_id << std::endl;
        std::cout << "forms_switchable: " << forms_switchable << std::endl;
        std::cout << "is_legendary: " << is_legendary << std::endl;
        std::cout << "is_mythical: " << is_mythical << std::endl;
        std::cout << "order: " << order << std::endl;
        std::cout << "conquest_order: " << conquest_order << std::endl;
        std::cout << std::endl;
    }
};

class experience
{
    public:
        int growth_rate_id;
        int level;
        int experience;

    void clear()
    {
        growth_rate_id = -1;
        level = -1;
        experience = -1;
    }

    void print()
    {
        std::cout << "growth_rate_id: " << growth_rate_id << std::endl;
        std::cout << "level: " << level << std::endl;
        std::cout << "experience: " << experience << std::endl;
        std::cout << std::endl;
    }
};

class type_names
{
    public:
        int type_id;
        int local_language_id;
        std::string name;

    void clear()
    {
        type_id = -1;
        local_language_id = -1;
        name = "";
    }

    void print()
    {
        std::cout << "type_id: " << type_id << std::endl;
        std::cout << "local_language_id: " << local_language_id << std::endl;
        std::cout << "name: " << name << std::endl;
        std::cout << std::endl;
    }
};

int main(int argc, char* argv[]){
    std::string path_a = "/share/cs327/";
    std::string home = getenv("HOME");
    std::string path_b = "";
    path_b = home + "/.poke327/";
    std::string csv_home = "pokedex/pokedex/data/csv/";
    std::ifstream f("");
    std::string path;
    std::string file_name;


    path_a += csv_home;
    path_b += csv_home;
    

    if(argc != 2){
        std::cout << "Invalid Args, Exiting..." << std::endl;
        return -1; 
    } else{
        path_a = path_a + argv[1] + ".csv";
        path_b = path_b + argv[1] + ".csv";
        file_name = argv[1];

        std::ifstream f(path_a);
        if(!f.good()){
            std::ifstream f(path_b);
            if(!f.good()){
                std::cout << "File not found, Exiting..." << std::endl;
                return -1;
            } else{
                path = path_b;
            }
        } else{
            path = path_a;
        }
    }
    f.open(path);

    if(file_name == "moves"){
        std::vector<moves> items;
        std::string line, word;
        moves p;
        int index;
        getline(f, line); //getting first line and dumping it immediately
        while(getline(f, line)){
            index = 0;
            p.clear();
            std::stringstream str(line);

            while(getline(str, word, ',')){
                switch(index){
                    case 0:
                        if(word != ""){
                            p.id = std::stoi(word);
                        }
                        break;
                    case 1:
                        if(word != ""){
                            p.name = word;
                        }
                        break;
                    case 2:
                        if(word != ""){
                            p.generation_id = std::stoi(word);
                        }
                        break;
                    case 3:
                        if(word != ""){
                            p.type_id = std::stoi(word);
                        }
                        break;
                    case 4:
                        if(word != ""){
                            p.power = std::stoi(word);
                        }
                        break;
                    case 5:
                        if(word != ""){
                            p.pp = std::stoi(word);
                        }
                        break;
                    case 6:
                        if(word != ""){
                            p.accuracy = std::stoi(word);
                        }
                        break;
                    case 7:
                        if(word != ""){
                            p.priority = std::stoi(word);
                        }
                        break;
                    case 8:
                        if(word != ""){
                            p.target_id = std::stoi(word);
                        }
                        break;
                    case 9:
                        if(word != ""){
                            p.damage_class_id = std::stoi(word);
                        }
                        break;
                    case 10:
                        if(word != ""){
                            p.effect_id = std::stoi(word);
                        }
                        break;
                    case 11:
                        if(word != ""){
                            p.effect_chance = std::stoi(word);
                        }
                        break;
                    case 12:
                        if(word != ""){
                            p.contest_type_id = std::stoi(word);
                        }
                        break;
                    case 13:
                        if(word != ""){
                            p.contest_effect_id = std::stoi(word);
                        }
                        break;
                    case 14:
                        if(word != ""){
                            p.super_contest_effect_id = std::stoi(word);
                        }
                        break;
                    default:
                        break;
                }
                index++;
            }
            items.push_back(p);
            p.print();
        }
    } else if(file_name == "pokemon"){
        std::vector<pokemon> items;
        std::string line, word;
        pokemon p;
        int index;
        getline(f, line); 
        while(getline(f, line)){
            index = 0;
            p.clear();
            std::stringstream str(line);

            while(getline(str, word, ',')){
                switch(index){
                    case 0:
                        if(word != ""){
                            p.id = std::stoi(word);
                        }
                        break;
                    case 1:
                        if(word != ""){
                            p.name = word;
                        }
                        break;
                    case 2:
                        if(word != ""){
                            p.species_id = std::stoi(word);
                        }
                        break;
                    case 3:
                        if(word != ""){
                            p.height = std::stoi(word);
                        }
                        break;
                    case 4:
                        if(word != ""){
                            p.weight = std::stoi(word);
                        }
                        break;
                    case 5:
                        if(word != ""){
                            p.base_xp = std::stoi(word);
                        }
                        break;
                    case 6:
                        if(word != ""){
                            p.order = std::stoi(word);
                        }
                        break;
                    case 7:
                        if(word != ""){
                            p.is_default = std::stoi(word);
                        }
                        break;
                    default:
                        break;
                }
                index++;
            }
            items.push_back(p);
            p.print();
        }
    } else if(file_name == "pokemon_moves"){
        std::vector<pokemon_moves> items;
        std::string line, word;
        pokemon_moves p;
        int index;
        getline(f, line); 
        while(getline(f, line)){
            index = 0;
            p.clear();
            std::stringstream str(line);

            while(getline(str, word, ',')){
                switch(index){
                    case 0:
                        if(word != ""){
                            p.pokemon_id = std::stoi(word);
                        }
                        break;
                    case 1:
                        if(word != ""){
                            p.version_group_id = std::stoi(word);
                        }
                        break;
                    case 2:
                        if(word != ""){
                            p.move_id = std::stoi(word);
                        }
                        break;
                    case 3:
                        if(word != ""){
                            p.pokemon_move_method_id = std::stoi(word);
                        }
                        break;
                    case 4:
                        if(word != ""){
                            p.level = std::stoi(word);
                        }
                        break;
                    case 5:
                        if(word != ""){
                            p.order = std::stoi(word);
                        }
                        break;
                    default:
                        break;
                }
                index++;
            }
            items.push_back(p);
            p.print();
        }
    } else if(file_name == "pokemon_species") {
        std::vector<pokemon_species> items;
        std::string line, word;
        pokemon_species p;
        int index;
        getline(f, line); 
        while(getline(f, line)){
            index = 0;
            p.clear();
            std::stringstream str(line);

            while(getline(str, word, ',')){
                switch(index){
                    case 0:
                        if(word != ""){
                            p.id = std::stoi(word);
                        }
                        break;
                    case 1:
                        if(word != ""){
                            p.name = word;
                        }
                        break;
                    case 2:
                        if(word != ""){
                            p.generation_id = std::stoi(word);
                        }
                        break;
                    case 3:
                        if(word != ""){
                            p.evolves_from_species_id = std::stoi(word);
                        }
                        break;
                    case 4:
                        if(word != ""){
                            p.evolution_chain_id = std::stoi(word);
                        }
                        break;
                    case 5:
                        if(word != ""){
                            p.color_id = std::stoi(word);
                        }
                        break;
                    case 6:
                        if(word != ""){
                            p.shape_id = std::stoi(word);
                        }
                        break;
                    case 7:
                        if(word != ""){
                            p.habit_id = std::stoi(word);
                        }
                        break;
                    case 8:
                        if(word != ""){
                            p.gender_rate = std::stoi(word);
                        }
                        break;
                    case 9:
                        if(word != ""){
                            p.capture_rate = std::stoi(word);
                        }
                        break;
                    case 10:
                        if(word != ""){
                            p.base_happiness = std::stoi(word);
                        }
                        break;
                    case 11:
                        if(word != ""){
                            p.is_baby = std::stoi(word);
                        }
                        break;
                    case 12:
                        if(word != ""){
                            p.hatch_counter = std::stoi(word);
                        }
                        break;
                    case 13:
                        if(word != ""){
                            p.has_gender_differences = std::stoi(word);
                        }
                        break;
                    case 14:
                        if(word != ""){
                            p.growth_rate_id = std::stoi(word);
                        }
                        break;
                    case 15:
                        if(word != ""){
                            p.forms_switchable = std::stoi(word);
                        }
                        break;
                    case 16:
                        if(word != ""){
                            p.is_legendary = std::stoi(word);
                        }
                        break;
                    case 17:
                        if(word != ""){
                            p.is_mythical = std::stoi(word);
                        }
                        break;
                    case 18:
                        if(word != ""){
                            p.order = std::stoi(word);
                        }
                        break;
                    case 19:
                        if(word != ""){
                            p.conquest_order = std::stoi(word);
                        }
                        break;
                    default:
                        break;
                }
                index++;
            }
            items.push_back(p);
            p.print();
        }
    } else if(file_name == "experience") {
        std::vector<experience> items;
        std::string line, word;
        experience p;
        int index;
        getline(f, line); 
        while(getline(f, line)){
            index = 0;
            p.clear();
            std::stringstream str(line);

            while(getline(str, word, ',')){
                switch(index){
                    case 0:
                        if(word != ""){
                            p.growth_rate_id = std::stoi(word);
                        }
                        break;
                    case 1:
                        if(word != ""){
                            p.level = std::stoi(word);
                        }
                        break;
                    case 2:
                        if(word != ""){
                            p.experience = std::stoi(word);
                        }
                        break;
                    default:
                        break;
                }
                index++;
            }
            items.push_back(p);
            p.print();
        }
    } else if(file_name == "type_names") {
        std::vector<type_names> items;
        std::string line, word;
        type_names p;
        int index;
        getline(f, line); 
        while(getline(f, line)){
            index = 0;
            p.clear();
            std::stringstream str(line);

            while(getline(str, word, ',')){
                switch(index){
                    case 0:
                        if(word != ""){
                            p.type_id = std::stoi(word);
                        }
                        break;
                    case 1:
                        if(word != ""){
                            p.local_language_id = std::stoi(word);
                        }
                        break;
                    case 2:
                        if(word != ""){
                            p.name = word;
                        }
                        break;
                    default:
                        break;
                }
                index++;
            }
            items.push_back(p);
            p.print();
        }
    } else{
        std::cout << "File Parsing Not Supported... Sorry :(" << std::endl;
    }


    return 0;


}