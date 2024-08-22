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
class Resource_Tracker():
    sp_value = []
    sp_difference = []
    def add_sp_value(self, curr):
        self.sp_value.append(curr)
        self.sp_difference.append(0)
        if len(self.sp_value) > 1:
            self.sp_difference[len(self.sp_value)-2] = self.sp_value[-1] - self.sp_value[-2]
    def will_enemy_invest(self):
        sp_neg_vals = np.array([sp_val for idx, sp_val in enumerate(self.sp_value) if self.sp_difference[idx] < 0])
        if (len(sp_neg_vals) <= 1):
            return False
        
        avg = np.average(sp_neg_vals)
        stdev = np.std(sp_neg_vals)
        curr_sp_val = self.sp_value[-1]
        standard = (curr_sp_val - avg)/stdev
        cumulative_probability = 0.5 * (1 + np.math.erf(standard / np.sqrt(2)))
        return (random.random() < cumulative_probability)
        

class Offense(gamelib.AlgoCore):
    resource_changes = []
    enemy_health = []
    attacked = []
    MP_Limiter = 9
    MP_BOOST = 2
    SP = 0
    raised_limiter = False
    def __init__(self, algo_strategy):
        self.algo_strategy = algo_strategy
        self.tracker = Resource_Tracker()
    def go_to_target_edge(self, game_map, loc, target_edge):
        # gamelib.debug_write(loc)
        before_path = [loc.copy()]
        new_path = [loc.copy()]
        direction = []

        # Set initial direction based on target edge
        if target_edge == game_map.TOP_LEFT:
            direction = [[0, 1], [-1, 0]]  # Move up, then left
        else:
            direction = [[0, 1], [1, 0]]  # Move up, then right

        direction_set = 1

        while loc not in game_map.get_edge_locations(target_edge):
            # Update location by adding the current direction
            loc = [loc[0] + direction[direction_set][0], loc[1] + direction[direction_set][1]]

            # Ensure loc remains within the game bounds
            if not game_map.in_arena_bounds(loc) and loc not in game_map.get_edge_locations(target_edge):
                # Revert the location change if out of bounds
                loc = [loc[0] - direction[direction_set][0], loc[1] - direction[direction_set][1]]
                direction_set = (direction_set + 1) % 2  # Try the other direction
            elif loc != new_path[-1]:
                new_path.append(loc.copy())  # Append a copy of the current location

        # Ensure the final location is added to the path
        if new_path[-1] != loc:
            new_path.append(loc.copy())

        return before_path + new_path

    def simulate_an_attack(self, game_state, SCOUT, MP, spawn_location):
        num_scouts = game_state.number_affordable(SCOUT)
        track_attacker_health = {}
        destroyed_attackers = []
        scout_health = num_scouts * (gamelib.GameUnit(SCOUT, game_state.config).max_health + 3)
        curr_loc = spawn_location
        simulated_path = [curr_loc]
        game_map_copy = copy.deepcopy(game_state.game_map)
        target_edge = game_state.get_target_edge(spawn_location)
        points_target_edge = game_state.game_map.get_edge_locations(target_edge)
        curr_path = game_state.find_path_to_edge(curr_loc, target_edge)
        # gamelib.debug_write(f"curr_path:{curr_path}")
        curr_index = 0
        # gamelib.debug_write(f"Starting Location:{curr_loc}")
        # gamelib.debug_write(f"Target Edge: {points_target_edge}")
        gave_up = False
        set_arr = False
        dir_array = []
        dir_index = 1
        curr_index = 0
        recompute = True
        while True:
            if (curr_loc[1] < 13 and curr_index < len(curr_path) - 1):
                curr_index += 1
                curr_loc = curr_path[curr_index]
                simulated_path.append(curr_loc.copy())
            else:
                if not gave_up:
                    if recompute:
                        recompute = False
                        navigator = gamelib.navigation.ShortestPathFinder()  
                        navigator.initialize_map(game_state=game_state) 
                        curr_path = navigator.navigate_multiple_endpoints(start_point=curr_loc.copy(), end_points=points_target_edge, game_state=game_state)  # Call the method on the instance
                        curr_index = 1
                    if curr_index < len(curr_path):
                        curr_loc = curr_path[curr_index]
                        curr_index += 1
                    else:
                        gave_up = True
                else:
                    if not set_arr:
                        set_arr = True 
                        gamelib.debug_write("Given up on path computations?")
                        dir_array = self.go_to_target_edge(game_map_copy, curr_loc, target_edge)
                        curr_loc = dir_array[dir_index]
                    else:
                        dir_index += 1
                        curr_loc = dir_array[dir_index]
                if curr_loc == simulated_path[-1]:
                    continue
                simulated_path.append(curr_loc.copy())
            scout_damage = num_scouts * gamelib.GameUnit(SCOUT, game_state.config).damage_f
            if curr_loc in points_target_edge:
                break
            # Get all current attackers at this location
            current_attackers = game_state.get_attackers(curr_loc, 1)
            for attacker in current_attackers:
                if [attacker.x, attacker.y] not in destroyed_attackers:
                    scout_health -= attacker.damage_i

            circle = game_state.game_map.get_locations_in_range([curr_loc[0], curr_loc[1]],
                                                                gamelib.GameUnit(SCOUT, game_state.config).attackRange)
            get_affected_enemy_units = []
            for loc in circle:
                if game_state.contains_stationary_unit(loc):
                    unit = game_state.contains_stationary_unit(loc)
                    if unit.player_index == 1 and [unit.x, unit.y] not in destroyed_attackers:
                        get_affected_enemy_units.append(game_state.contains_stationary_unit(loc))

            def sort_enemy_units(get_affected_enemy_units, current_position):
                # Define a sorting key function
                def sort_key(unit):
                    distance_to_current = game_state.game_map.distance_between_locations(
                        [unit.x, unit.y], current_position)
                    distance_to_edge = min(unit.x, game_state.game_map.ARENA_SIZE - unit.x)
                    return (distance_to_current,
                            unit.health,
                            -unit.y,  # Negative for descending order (highest y-value first)
                            distance_to_edge)

                # Sort the list using the key function
                get_affected_enemy_units.sort(key=sort_key)
                return get_affected_enemy_units

            get_affected_enemy_units = sort_enemy_units(get_affected_enemy_units, loc)
            if (scout_health <= 0):
                # gamelib.debug_write("Gbye Scouts")
                num_scouts = 0
                break
            num_scouts = round(scout_health / gamelib.GameUnit(SCOUT, game_state.config).health)
            for unit in get_affected_enemy_units:
                if scout_damage <= 0:
                    break
                if tuple([unit.x, unit.y]) not in track_attacker_health:
                    if (unit.health > scout_damage):
                        track_attacker_health[tuple([unit.x, unit.y])] = unit.health - scout_damage
                        break
                    else:
                        recompute = True
                        game_state.game_map.remove_unit([unit.x, unit.y])
                        destroyed_attackers.append([unit.x, unit.y])
                        scout_damage -= round(
                            unit.health / gamelib.GameUnit(SCOUT, game_state.config).damage_f) * gamelib.GameUnit(SCOUT,
                                                                                                                  game_state.config).damage_f
                else:
                    curr_health = track_attacker_health[tuple([unit.x, unit.y])]
                    if (curr_health > scout_damage):
                        track_attacker_health[tuple([unit.x, unit.y])] = curr_health - scout_damage
                        break
                    else:
                        recompute = True
                        game_state.game_map.remove_unit([unit.x, unit.y])
                        destroyed_attackers.append([unit.x, unit.y])
                        scout_damage -= round(
                            curr_health / gamelib.GameUnit(SCOUT, game_state.config).damage_f) * gamelib.GameUnit(SCOUT,
                                                                                                                  game_state.config).damage_f
        # gamelib.debug_write(f"num_scouts: {num_scouts}")
        if simulated_path[-1] not in points_target_edge:
            num_scouts = 0
        # gamelib.debug_write(f"Simulated Path: {simulated_path}")
        # gamelib.debug_write(f"Scouts Surviving: {num_scouts}")
        game_state.game_map = game_map_copy
        return (num_scouts, -len(destroyed_attackers))

    """
        -Attacks should consider low health in case of desparation need to send all MP to send out scouts
            -Do when health < 3
        -Save MP while minimum damage path would still wipe out all scouts
            -Probably should only launch attacks when they can definitively take out 3 points or more from opponent health until we are desperate
        -When launching attack
            -If opponent has high SP then launch attack on second least well-defended sector to be careful of 
    """

    def send_out_troops(self, game_state, location, SCOUT, SUPPORT):
        self.raised_limiter = False
        # gamelib.debug_write(f"spawn_location:{location}, affordable: {game_state.number_affordable(SCOUT)}")
        game_state.attempt_spawn(SCOUT, location, game_state.number_affordable(SCOUT))
        val = False
        if (location in game_state.game_map.get_edge_locations(
                game_state.game_map.BOTTOM_LEFT)):
            if (location != [0, 13]):
                val = game_state.attempt_spawn(SUPPORT, [location[0] - 1, location[1] + 1], 1)
                game_state.attempt_remove([location[0] - 1, location[1] + 1])
            else:
                val = game_state.attempt_spawn(SUPPORT, [location[0] + 1, location[1] - 1], 1)
                game_state.attempt_remove([location[0] + 1, location[1] - 1])
        else:
            if (location != [27, 13]):
                val = game_state.attempt_spawn(SUPPORT, [location[0] + 1, location[1] + 1], 1)
                game_state.attempt_remove([location[0] + 1, location[1] + 1])
            else:
                val = game_state.attempt_spawn(SUPPORT, [location[0] - 1, location[1] - 1], 1)
                game_state.attempt_remove([location[0] - 1, location[1] - 1])
        self.attacked.append(True)
        if val != False:
            # gamelib.debug_write(f"SP cost:{game_state.get_resource(self.SP) - gamelib.GameUnit(SUPPORT,game_state.config).cost[0]} ")
            return game_state.get_resource(self.SP) - gamelib.GameUnit(SUPPORT, game_state.config).cost[0]
        else:
            return game_state.get_resource(self.SP)

    def send_the_cavalry(self, game_state, SCOUT, SUPPORT, MP, SP):
        """
        Find the least damage cost sectors and send troops to attack there
        """
        self.SP = SP
        self.enemy_health.append(game_state.enemy_health)
        self.tracker.add_sp_value(game_state.get_resource(SP, 1))
        friendly_edges = game_state.game_map.get_edge_locations(
            game_state.game_map.BOTTOM_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.BOTTOM_RIGHT)
        friendly_edges = [loc for loc in friendly_edges if not game_state.contains_stationary_unit(loc)]
        damages = [self.simulate_an_attack(game_state, SCOUT, MP, loc) for loc in friendly_edges]
        spawn_location = friendly_edges[damages.index(max(damages))]
        num_scouts, _ = max(damages)
        # gamelib.debug_write(f" Mobile Points: {game_state.get_resource(MP)}, Num Scouts:{num_scouts}")

        if (game_state.my_health < 3):
            return self.send_out_troops(game_state, spawn_location, SCOUT, SUPPORT)

        # checks if the number of surviving is sizeable enough to put a dent in enemy health at least 30%
        # checks if we are not at MP limit for it to not grow enough in the future
        # if we are at MP limit or if we can make a sizeable enough dent then we go for an attack
        gamelib.debug_write(f"expected damage: {num_scouts}")
        if (self.tracker.will_enemy_invest() and num_scouts < self.MP_Limiter):
            gamelib.debug_write(f"simulated num_scouts: {num_scouts}, enemy_health: {game_state.enemy_health}")
            self.attacked.append(False)
        elif (len(self.enemy_health) > 2 and True in self.attacked):
            def find_latest_true(arr):
                for i in range(len(arr) - 1, 0, -1):
                    if arr[i]:
                        return i
                return None

            ind = find_latest_true(self.attacked)
            percent_change = (self.enemy_health[ind] - self.enemy_health[-1])/self.enemy_health[ind]
            if (percent_change < 0.15 and not self.raised_limiter):
                self.MP_Limiter += self.MP_BOOST
                self.attacked.append(False)
                self.raised_limiter = True
            else:
                spawn_location = friendly_edges[damages.index(max(damages))]
                return self.send_out_troops(game_state, spawn_location, SCOUT, SUPPORT)

        else:
            spawn_location = friendly_edges[damages.index(max(damages))]
            return self.send_out_troops(game_state, spawn_location, SCOUT, SUPPORT)
        return game_state.get_resource(self.SP)

