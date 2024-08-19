import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json

class Offense(gamelib.AlgoCore):
    MP_Limiter = 12
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
        sector_three_right = self.return_point_list([14,0], [18,4], [1, 1])
        sector_three_left = self.return_point_list([4,9], [9,4], [1, -1])
        sector_dict = {

        }
        sector_dict["sector_one_right"] = sector_one_right
        sector_dict["sector_one_left"] = sector_one_left
        sector_dict["sector_three_right"] = sector_three_right
        sector_dict["sector_three_left"] = sector_three_left
        return sector_dict
    def send_the_cavalry(self, game_state, SCOUT, MP):
        """
        Find the least damage cost sectors and send troops to attack there
        """
        #len(game_state.get_attackers(path_location, 0))
        if game_state.get_resource(MP) > self.MP_Limiter:
            avg_damage_dict = self.calculate_sector_average_damage(game_state)
            min_damage_sector = min(avg_damage_dict, key=avg_damage_dict.get)
            sector_edge_dict = self.find_opposing_friendly_edges(game_state)
            friendly_edge_set = sector_edge_dict[min_damage_sector]
            unit_numbers = game_state.get_resource(MP) 
            available_locations = [loc for loc in friendly_edge_set if not game_state.contains_stationary_unit(loc)]
            # Get the cost of a SCOUT in MP
            scout_cost = game_state.type_cost(SCOUT)[MP]
            
            # Calculate the number of SCOUT units we can afford
            num_scouts = (int)(unit_numbers / scout_cost)
            # Attempt to spawn SCOUT units at random location from friendly_edge_set
            if not available_locations:
                return
            spawn_location = random.choice(available_locations)
            game_state.attempt_spawn(SCOUT, spawn_location, num_scouts)
    

    def interfere_with_interceptors(self, game_state, INTERCEPTOR):
        """
        Send out interceptors at random locations to defend our base from enemy moving units.
        """
        # We can spawn moving units on our edges so a list of all our edge locations
        friendly_edges = game_state.game_map.get_edge_locations(
            game_state.game_map.BOTTOM_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.BOTTOM_RIGHT)

        # Remove locations that are blocked by our own structures 
        # since we can't deploy units there.
        deploy_locations = self.filter_blocked_locations(friendly_edges, game_state)
        scored_locations = self.scored_on_locations
        defense_locations = [loc for loc in deploy_locations if loc in scored_locations]
        if not defense_locations:
            defense_locations = deploy_locations
        

        # While we have remaining MP to spend lets send out interceptors randomly.
        while game_state.get_resource(MP) >= game_state.type_cost(INTERCEPTOR)[MP] and len(deploy_locations) > 0:
            # Choose a random deploy location.
            deploy_index = random.randint(0, len(deploy_locations) - 1)
            deploy_location = deploy_locations[deploy_index]

            game_state.attempt_spawn(INTERCEPTOR, deploy_location)
            """
            We don't have to remove the location since multiple mobile 
            units can occupy the same space.
            """

    
