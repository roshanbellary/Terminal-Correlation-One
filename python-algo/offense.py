import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json
import copy
import heapq
import gamelib.navigation
class Offense(gamelib.AlgoCore):
    MP_Limiter = 12
    resource_changes = []
    enemy_health = []
    attacked = []
    MP_THRESHOLD = 4
    MP_BOOST = 2
    def __init__(self, algo_strategy):
        self.algo_strategy = algo_strategy
    def calculate_damage(self, game_state, point):
        damage = 0
        for attacker in game_state.get_attackers(point, 1):
            attacker.health
            damage += attacker.damage_i*(attacker.health)/(attacker.max_health)
        return damage
    def simulate_an_attack(self, game_state, SCOUT, MP, spawn_location):
        num_scouts = game_state.number_affordable(SCOUT)
        track_attacker_health = {}
        destroyed_attackers = []
        scout_health = num_scouts * gamelib.GameUnit(SCOUT, game_state.config).health
        curr_loc = spawn_location
        game_map_copy = copy.deepcopy(game_state.game_map)
        target_edge = game_state.get_target_edge(spawn_location)
        curr_path = game_state.find_path_to_edge(curr_loc, target_edge)
        curr_index = 0
        while True:
            if (curr_loc[1] < 13 and curr_index < len(curr_path)-1):
                curr_index += 1
                curr_loc = curr_path[curr_index]
            else:
                new_path = game_state.find_path_to_edge(curr_loc, target_edge)
                if len(new_path) > 1:
                    curr_loc = new_path[1]
                else:
                    num_scouts = 0
                    break    
            reached_edge = False 
            if curr_loc in game_state.game_map.get_edge_locations(
            game_state.game_map.TOP_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.TOP_RIGHT):
                reached_edge = True
            # Get all current attackers at this location
            current_attackers = game_state.get_attackers(curr_loc, 1)
            for attacker in current_attackers:
                if [attacker.x, attacker.y] not in destroyed_attackers:
                    scout_health -= attacker.damage_i
                    
            circle = game_state.game_map.get_locations_in_range([curr_loc[0], curr_loc[1]],gamelib.GameUnit(SCOUT,game_state.config).attackRange)
            get_affected_enemy_units = []
            for loc in circle:
                if game_state.contains_stationary_unit(loc):
                    unit =  game_state.contains_stationary_unit(loc)
                    if unit.player_index == 1 and [unit.x, unit.y] not in destroyed_attackers:
                        get_affected_enemy_units.append(game_state.contains_stationary_unit(loc))
            def sort_enemy_units(get_affected_enemy_units, current_position):
                # Define a sorting key function
                def sort_key(unit):
                    distance_to_current = game_state.game_map.distance_between_locations(
                        [unit.x, unit.y], current_position)
                    distance_to_edge = min(unit.x,game_state.game_map.ARENA_SIZE - unit.x)
                    return (distance_to_current, 
                            unit.health, 
                            -unit.y,  # Negative for descending order (highest y-value first)
                            distance_to_edge)
                
                # Sort the list using the key function
                get_affected_enemy_units.sort(key=sort_key)
                return get_affected_enemy_units
            get_affected_enemy_units = sort_enemy_units(get_affected_enemy_units, loc)
            num_scouts = round(scout_health / gamelib.GameUnit(SCOUT, game_state.config).health)
            scout_damage = num_scouts * gamelib.GameUnit(SCOUT, game_state.config).damage_f
            for unit in get_affected_enemy_units:
                if scout_damage <= 0:
                    break
                if tuple([unit.x, unit.y]) not in track_attacker_health:
                    if (unit.health > scout_damage):
                        track_attacker_health[tuple([unit.x, unit.y])] = unit.health - scout_damage
                        break
                    else:
                        game_state.game_map.remove_unit([unit.x, unit.y])
                        destroyed_attackers.append([unit.x, unit.y])
                        scout_damage -= round(unit.health/gamelib.GameUnit(SCOUT,game_state.config).damage_f) * gamelib.GameUnit(SCOUT,game_state.config).damage_f
                else:
                    curr_health = track_attacker_health[tuple([unit.x, unit.y])]
                    if (curr_health > scout_damage):
                        track_attacker_health[tuple([unit.x, unit.y])] = curr_health - scout_damage
                        break
                    else:
                        game_state.game_map.remove_unit([unit.x, unit.y])
                        destroyed_attackers.append([unit.x, unit.y])
                        scout_damage -= round(curr_health/gamelib.GameUnit(SCOUT,game_state.config).damage_f) * gamelib.GameUnit(SCOUT,game_state.config).damage_f
            if num_scouts <= 0:
                num_scouts = 0
                break
            if reached_edge:
                break
        game_state.game_map = game_map_copy
        return num_scouts
    """
        -Attacks should consider low health in case of desparation need to send all MP to send out scouts
            -Do when health < 3
        -Save MP while minimum damage path would still wipe out all scouts
            -Probably should only launch attacks when they can definitively take out 3 points or more from opponent health until we are desperate
        -When launching attack
            -If opponent has high SP then launch attack on second least well-defended sector to be careful of 
    """
    def send_out_troops(self, game_state, location, SCOUT, SUPPORT):
        game_state.attempt_spawn(SCOUT, location, game_state.number_affordable(SCOUT))
        if (location in game_state.game_map.get_edge_locations(
            game_state.game_map.BOTTOM_LEFT)):
            if (location != [0, 13]):
                game_state.attempt_spawn(SUPPORT, [location[0]-1, location[1]+1], 1)
                game_state.attempt_remove([location[0]-1, location[1]+1])
            else:
                game_state.attempt_spawn(SUPPORT, [location[0]+1, location[1]-1], 1)
                game_state.attempt_remove([location[0]+1, location[1]-1])
        else:
            if (location != [27, 13]):
                game_state.attempt_spawn(SUPPORT, [location[0]+1, location[1]+1], 1)
                game_state.attempt_remove([location[0]+1, location[1]+1])
            else:
                 game_state.attempt_spawn(SUPPORT, [location[0]-1, location[1]-1], 1)
                 game_state.attempt_remove([location[0]-1, location[1]-1])
        self.attacked.append(True)
    def send_the_cavalry(self, game_state, SCOUT, SUPPORT, MP, SP):
        """
        Find the least damage cost sectors and send troops to attack there
        """
        self.enemy_health.append(game_state.enemy_health)
        self.resource_changes.append(game_state.get_resource(SP, 1))
        friendly_edges = game_state.game_map.get_edge_locations(
            game_state.game_map.BOTTOM_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.BOTTOM_RIGHT)
        friendly_edges = [loc for loc in friendly_edges if not game_state.contains_stationary_unit(loc) ]
        damages = [self.simulate_an_attack(game_state, SCOUT, MP, loc) for loc in friendly_edges]
        spawn_location = friendly_edges[damages.index(max(damages))]
        num_scouts = max(damages)
        gamelib.debug_write(f" Mobile Points: {game_state.get_resource(MP)}, Num Scouts:{num_scouts}")

        if (game_state.my_health < 3):
            self.send_out_troops(game_state, spawn_location, SCOUT, SUPPORT)
            return
        # checks if the number of surviving is sizeable enough to put a dent in enemy health at least 30%
        # checks if we are not at MP limit for it to not grow enough in the future
        # if we are at MP limit or if we can make a sizeable enough dent then we go for an attack
        def find_length_of_forgone(arr):
            count = 0
            for i in range(len(arr)-1, 0, -1):
                if arr[i] == False:
                    count += 1
            return count
        
        if (num_scouts < max(game_state.enemy_health*0.3,self.MP_THRESHOLD)):
            gamelib.debug_write(f"num_scouts: {num_scouts}, enemy_health: {game_state.enemy_health},  forgone: {find_length_of_forgone(self.attacked)}")
            self.attacked.append(False)
        elif (len(self.enemy_health) > 2 and True in self.attacked):
            def find_latest_true(arr):
                for i in range(len(arr) - 1, 0, -1):
                    if arr[i]:
                        return i
                return None
            ind = find_latest_true(self.attacked)
            percent_change = (self.enemy_health[ind] - self.enemy_health[-1])/self.enemy_health[ind]
            if (ind == len(self.attacked)-1):
                if (percent_change < 0.2):
                    gamelib.debug_write(f"threshold update: {self.MP_THRESHOLD} and threshold_boost: {self.MP_BOOST}")
                    self.MP_THRESHOLD += self.MP_BOOST
                    self.attacked.append(False)
                else:
                    self.MP_THRESHOLD -= self.MP_BOOST
                    spawn_location = friendly_edges[damages.index(max(damages))]
                    gamelib.debug_write(f"spawn_location:{spawn_location}, affordable: {game_state.number_affordable(SCOUT)}")
                    self.send_out_troops(game_state, spawn_location, SCOUT, SUPPORT)
            else:
                spawn_location = friendly_edges[damages.index(max(damages))]
                gamelib.debug_write(f"spawn_location:{spawn_location}, affordable: {game_state.number_affordable(SCOUT)}")
                self.send_out_troops(game_state, spawn_location, SCOUT, SUPPORT)

        else:
            spawn_location = friendly_edges[damages.index(max(damages))]
            gamelib.debug_write(f"spawn_location:{spawn_location}, affordable: {game_state.number_affordable(SCOUT)}")
            self.send_out_troops(game_state, spawn_location, SCOUT, SUPPORT)

    
