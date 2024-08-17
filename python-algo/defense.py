import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json


class Defense(gamelib.AlgoCore):

    def __init__(self, algo_strategy):
        self.algo_strategy = algo_strategy

    def turret_opt(self, game_state):
        while True:
            enemy_edges = game_state.game_map.get_edge_locations(
                game_state.game_map.TOP_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.TOP_RIGHT)
            path_costs = []
            for loc in enemy_edges:
                path_costs += self.path_damage(game_state, loc)

            turret_costs = np.empty((game_state.game_map.HALF_ARENA, game_state.game_map.HALF_ARENA))
            path_counts = np.empty((game_state.game_map.HALF_ARENA, game_state.game_map.HALF_ARENA))
            for loc in range(len(enemy_edges)):
                path = game_state.find_path_to_edge(enemy_edges[loc])
                for i in path:
                    if i[1] < game_state.game_map.HALF_ARENA:
                        turret_costs[i[0]][i[1]] += path_costs[loc]
                        path_counts[i[0]][i[1]] += 1

            cost_prop = gamelib.GameUnit(self.algo_strategy.TURRET, game_state.config).cost / game_state.SP
            min_cost_idx = np.argmin(turret_costs)
            min_cost_idx = np.unravel_index(min_cost_idx, turret_costs.shape)
            if cost_prop < path_counts[min_cost_idx] / np.min(turret_costs):
                game_state.attempt_spawn(self.algo_strategy.TURRET, [min_cost_idx])
                print('hi')
            else:
                return

    def path_damage(self, game_state, location):
        path = game_state.find_path_to_edge(location)
        damage = 0
        for path_location in path:
            damage += len(game_state.get_attackers(path_location, 0)) * gamelib.GameUnit(self.algo_strategy.TURRET,
                                                                                         game_state.config).damage_i
        return damage
