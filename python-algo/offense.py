import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json
import heapq
import gamelib.navigation
class Offense(gamelib.AlgoCore):
    MP_Limiter = 12
    resource_changes = []
    enemy_health = []
    attacked = []
    MP_THRESHOLD = 6
    MP_BOOST = 5
    def __init__(self, algo_strategy):
        self.algo_strategy = algo_strategy
    def is_point_in_rectangle(self, point, rect_points):
        def cross_product(p1, p2, p3):
            """Calculate the cross product of vectors p1p2 and p1p3."""
            return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])

        def is_same_side(p1, p2, a, b):
            """Check if points p1 and p2 are on the same side of the line ab."""
            cp1 = cross_product(a, b, p1)
            cp2 = cross_product(a, b, p2)
            return cp1 * cp2 >= 0

        # Check if the point is on the same side of each of the rectangle's edges
        p1, p2, p3, p4 = rect_points
        return (is_same_side(point, p1, p2, p3) and
                is_same_side(point, p2, p3, p4) and
                is_same_side(point, p3, p4, p1) and
                is_same_side(point, p4, p1, p2))

    def check_sector_one_left(self, point):
        boundary = [[-0.5, 13.5], [4,9], [8.5, 13.5], [4,18]]
        return self.is_point_in_rectangle(point, boundary)
    def check_sector_one_right(self, point):
        boundary = [[18.5, 13.5], [23, 18], [27.5, 13.5], [23, 9]]
        return self.is_point_in_rectangle(point, boundary)

    def check_sector_two(self, point):
        boundary = [[7.5, 13.5], [13.5, 19], [19.5, 13.5], [13.5, 8]]
        return self.is_point_in_rectangle(point, boundary)

    def check_sector_three_left(self, point):
        boundary = [[4,18], [9, 23], [13, 19], [8, 14]]
        return self.is_point_in_rectangle(point, boundary)
    def check_sector_three_right(self, point):
        boundary = [[14, 19], [18,23],[23, 18], [19, 14]]
        return self.is_point_in_rectangle(point, boundary)
    def calculate_sector_average_damage(self, game_state):
        sector_boundaries = {
            "sector_one_left": self.check_sector_one_left,
            "sector_one_right": self.check_sector_one_right,
            "sector_two": self.check_sector_two,
            "sector_three_left": self.check_sector_three_left,
            "sector_three_right": self.check_sector_three_right
        }

        sector_points = {sector: [] for sector in sector_boundaries}
        sector_damage_totals = {sector: 0 for sector in sector_boundaries}
        sector_point_counts = {sector: 0 for sector in sector_boundaries}

        # Iterate over all points in the game map
        for point in game_state.game_map:
            # Check which sector the point belongs to
            for sector, check_function in sector_boundaries.items():
                if check_function(point):
                    sector_points[sector].append(point)
                    break

        # Calculate the damage in each sector
        for sector, points in sector_points.items():
            for point in points:
                for attacker in game_state.get_attackers(point, 1):
                 sector_damage_totals[sector] += attacker.damage_i *(attacker.health)/attacker.max_health
                sector_point_counts[sector] += 1

        # Calculate average damage for each sector
        sector_average_damage = {sector: (sector_damage_totals[sector] / sector_point_counts[sector]) if sector_point_counts[sector] > 0 else 0
                                for sector in sector_boundaries}
        sector_average_damage["sector_three_left"] +=  sector_average_damage["sector_two"]
        sector_average_damage["sector_three_right"] += sector_average_damage["sector_two"]
        sector_average_damage["sector_two"] = 1e6 #can't send units to sector two anways
        return sector_average_damage
    def return_point_list(self, start, end, incrementer):
        res = [start]
        while True:
            if (start[0] == end[0] and start[1] == end[1]):
                break
            start[0] += incrementer[0]
            start[1] += incrementer[1]
            res.append(start)
        return res
    def find_opposing_friendly_edges(self, game_state):
        sector_one_right = self.return_point_list([9,4], [13,0], [1,-1])
        sector_one_left = self.return_point_list([14,0], [18,4], [1, 1])
        sector_three_right = self.return_point_list([4,9], [9,4], [1, -1])
        sector_three_left = self.return_point_list([18,4], [23,9], [1, 1])
        sector_dict = {

        }
        sector_dict["sector_one_right"] = sector_one_right
        sector_dict["sector_one_left"] = sector_one_left
        sector_dict["sector_three_right"] = sector_three_right
        sector_dict["sector_three_left"] = sector_three_left
        return sector_dict
    def calculate_damage(self, game_state, point):
        damage = 0
        for attacker in game_state.get_attackers(point, 1):
            attacker.health
            damage += attacker.damage_i*(attacker.health)/(attacker.max_health)
        return damage
    def find_optimized_path(self, game_state, point):
        path = game_state.find_path_to_edge(point)
        for idx, node in enumerate(path):
            if (node[1] == 13):
                path = path[:idx + 1]
                break

        curr = path[-1]

        if point[0] > 13:
            direction = [[0,1], [-1, 0]]
            direction_set = 1
            while curr[1]-curr[0] <= 14:
                path.append(curr)
                curr[0] += direction[direction_set][0]
                curr[1] += direction[direction_set][1]
                direction_set = (direction_set + 1) % 2
        else:
            direction = [[0,1], [1, 0]]
            direction_set = 1
            while curr[1]+curr[0] <= 41:
                path.append(curr)
                curr[0] += direction[direction_set][0]
                curr[1] += direction[direction_set][1]
                direction_set = (direction_set + 1) % 2
        return path
    def simulate_an_attack(self, game_state, SCOUT, MP, spawn_location):
        num_scouts = game_state.number_affordable(SCOUT)
        track_attacker_health = {}
        destroyed_attackers = []
        scout_health = num_scouts * gamelib.GameUnit(SCOUT, game_state.config).health

        # Iterate over the optimized path that the scouts follow
        for loc in self.find_optimized_path(game_state, spawn_location):
            if num_scouts == 0:
                break
            
            # Get all current attackers at this location
            current_attackers = game_state.get_attackers(loc, 1)
            for attacker in current_attackers:
                if [attacker.x, attacker.y] not in destroyed_attackers:
                    scout_health -= attacker.damage_i
            circle = game_state.game_map.get_locations_in_range([loc[0], loc[1]],gamelib.GameUnit(SCOUT,game_state.config).attackRange)
            get_affected_enemy_units = []
            for loc in circle:
                if game_state.contains_stationary_unit(loc):
                    unit =  game_state.contains_stationary_unit(loc)
                    if unit.player_index == 1 and [unit.x, unit.y] not in destroyed_attackers:
                        get_affected_enemy_units.append(game_state.contains_stationary_unit(loc))
            def sort_enemy_units(game_state, get_affected_enemy_units, current_position):
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
            get_affected_enemy_units = sort_enemy_units(game_state, get_affected_enemy_units, loc)
            scout_damage = num_scouts * gamelib.GameUnit(SCOUT, game_state.config).damage_f
            num_scouts = round(scout_health / gamelib.GameUnit(SCOUT, game_state.config).health)
            for unit in get_affected_enemy_units:
                if scout_damage <= 0:
                    break
                if tuple([unit.x, unit.y]) not in track_attacker_health:
                    if (unit.health > scout_damage):
                        track_attacker_health[tuple([unit.x, unit.y])] = unit.health - scout_damage
                        break
                    else:
                        destroyed_attackers.append([unit.x, unit.y])
                        scout_damage -= round(unit.health/gamelib.GameUnit(SCOUT,game_state.config).damage_f) * gamelib.GameUnit(SCOUT,game_state.config).damage_f
                else:
                    curr_health = track_attacker_health[tuple([unit.x, unit.y])]
                    if (curr_health > scout_damage):
                        track_attacker_health[tuple([unit.x, unit.y])] = curr_health - scout_damage
                        break
                    else:
                        destroyed_attackers.append([unit.x, unit.y])
                        scout_damage -= round(curr_health/gamelib.GameUnit(SCOUT,game_state.config).damage_f) * gamelib.GameUnit(SCOUT,game_state.config).damage_f
        return num_scouts
    """
        -Attacks should consider low health in case of desparation need to send all MP to send out scouts
            -Do when health < 3
        -Save MP while minimum damage path would still wipe out all scouts
            -Probably should only launch attacks when they can definitively take out 3 points or more from opponent health until we are desperate
        -When launching attack
            -If opponent has high SP then launch attack on second least well-defended sector to be careful of 
    """
    def send_the_cavalry(self, game_state, SCOUT, MP, SP):
        """
        Find the least damage cost sectors and send troops to attack there
        """
        self.enemy_health.append(game_state.enemy_health)
        self.resource_changes.append(game_state.get_resource(SP, 1))
        friendly_edges = game_state.game_map.get_edge_locations(
            game_state.game_map.BOTTOM_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.BOTTOM_RIGHT)
        friendly_edges = [loc for loc in friendly_edges if not game_state.contains_stationary_unit(loc) ]
        damages = []
        for loc in friendly_edges:
            scouts_remaining = self.simulate_an_attack(game_state, SCOUT, MP, loc)
            damages.append(scouts_remaining)
        spawn_location = friendly_edges[damages.index(max(damages))]
        num_scouts = max(damages)
        gamelib.debug_write(f" Mobile Points: {game_state.get_resource(MP)}")

        if (game_state.my_health < 3):
            game_state.attempt_spawn(SCOUT, spawn_location, game_state.number_affordable(SCOUT))
            self.attacked.append(True)
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
        
        if (num_scouts < max(game_state.enemy_health*0.3,self.MP_THRESHOLD) and find_length_of_forgone(self.attacked) < 5):
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
            if (ind == len(self.attacked)-1) and (percent_change < 0.2):
                self.MP_THRESHOLD += self.MP_BOOST
            else:
                # spawn_location = friendly_edges[damages.index(max(damages))]
                # gamelib.debug_write(f"spawn_location:{spawn_location}, affordable: {game_state.number_affordable(SCOUT)}")
                # game_state.attempt_spawn(SCOUT, spawn_location, game_state.number_affordable(SCOUT))
                # self.attacked.append(True)
                #checks if attack was unsuccessful
                # avg_damage_dict = self.calculate_sector_average_damage(game_state)
                # min_damage_sector = min(avg_damage_dict, key=avg_damage_dict.get)
                # avg_damage_dict = {k: v for k, v in avg_damage_dict.items() if k != min_damage_sector}
                # min_damage_sector = min(avg_damage_dict, key=avg_damage_dict.get)
                
                # sector_edge_dict = self.find_opposing_friendly_edges(game_state)
                # friendly_edge_set = sector_edge_dict[min_damage_sector]
                # available_locations = [loc for loc in friendly_edge_set if not game_state.contains_stationary_unit(loc)]
                # if not available_locations:
                #     return
                # damages = [self.simulate_an_attack(game_state, SCOUT, MP, loc) for loc in available_locations]
                # # Attempt to spawn SCOUT units at location with most surviving scouts
                # spawn_location = available_locations[damages.index(max(damages))]
                # gamelib.debug_write(f"spawn_location:{spawn_location}, affordable: {game_state.number_affordable(SCOUT)}")
                # game_state.attempt_spawn(SCOUT, spawn_location, game_state.number_affordable(SCOUT))
                # self.attacked.append(True)
                spawn_location = friendly_edges[damages.index(max(damages))]
                gamelib.debug_write(f"spawn_location:{spawn_location}, affordable: {game_state.number_affordable(SCOUT)}")
                game_state.attempt_spawn(SCOUT, spawn_location, game_state.number_affordable(SCOUT))
                self.attacked.append(True)
        else:
            spawn_location = friendly_edges[damages.index(max(damages))]
            gamelib.debug_write(f"spawn_location:{spawn_location}, affordable: {game_state.number_affordable(SCOUT)}")
            game_state.attempt_spawn(SCOUT, spawn_location, game_state.number_affordable(SCOUT))
            self.attacked.append(True)

    
