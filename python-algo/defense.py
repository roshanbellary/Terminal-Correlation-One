import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json


class AlgoStrategy(gamelib.AlgoCore):

    def turret_opt(self, game_state):
        costs = self.damage_avg_cost(game_state)
        start_cost = np.average(costs)


    def damage_avg_cost(self, game_state):
        enemy_edges = game_state.game_map.get_edge_locations(
            game_state.game_map.TOP_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.TOP_RIGHT)
        costs = []
        for loc in enemy_edges:
            costs += self.path_damage(game_state, loc)
        return costs

    def path_damage(self, game_state, location):
        path = game_state.find_path_to_edge(location)
        damage = 0
        for path_location in path:
            damage += len(game_state.get_attackers(path_location, 0)) * gamelib.GameUnit(TURRET,
                                                                                         game_state.config).damage_i
        return damage
