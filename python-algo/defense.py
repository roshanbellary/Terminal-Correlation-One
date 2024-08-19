import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json


def calc_point_costs(game_state, path_costs):
    enemy_edges = game_state.game_map.get_edge_locations(
        game_state.game_map.TOP_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.TOP_RIGHT)
    point_costs = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.ARENA_SIZE))
    path_counts = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.ARENA_SIZE))
    for loc in range(len(enemy_edges)):
        path = game_state.find_path_to_edge(enemy_edges[loc])
        if path is not None:
            for i in path:
                if i[1] < game_state.game_map.HALF_ARENA:
                    point_costs[i] += path_costs[loc]
                    path_counts[i] += 1
    return point_costs / path_counts, path_counts


class Defense(gamelib.AlgoCore):

    def __init__(self, algo_strategy):
        self.algo_strategy = algo_strategy

    def turret_opt(self, game_state, TURRET):
        while True:
            if game_state.SP > 0:
                path_costs, turret_hits = self.calc_path_damages(game_state)

                point_costs, path_counts = calc_point_costs(game_state, path_costs)
                turret_hits = turret_hits / path_counts

                turret_costs = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                turret_counts = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        if game_state.can_spawn(TURRET, [x, y]):
                            circle = game_state.game_map.get_locations_in_range([x, y],
                                                                                gamelib.GameUnit(TURRET,
                                                                                                 game_state.config).attackRange)
                            for loc in circle:
                                turret_costs[x][y] += point_costs[loc]
                                turret_counts[x][y] += 1
                turret_costs = turret_costs / turret_counts

                upgraded_turret_costs = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                upgraded_turret_counts = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        if game_state.can_spawn(TURRET, [x, y]):
                            # Change to upgraded turret range
                            circle = game_state.game_map.get_locations_in_range([x, y], 5)
                            for loc in circle:
                                upgraded_turret_costs[x][y] += point_costs[loc]
                                upgraded_turret_counts[x][y] += 1
                upgraded_turret_costs = upgraded_turret_costs / upgraded_turret_counts

                curr_turret_cost_ratios = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        if game_state.game_map.in_arena_bounds([x, y]):
                            for unit in game_state.game_map[[x, y]]:
                                if unit.damage_i + unit.damage_f > 0 and unit.player_index == 1:
                                    damage_chg = turret_hits[x][y] / turret_costs[x][y]
                                    if gamelib.GameUnit(unit.unit_type, game_state.config).upgraded:
                                        damage_chg = turret_hits[x][y] / upgraded_turret_costs[x][y]
                                    point_chg = 0.75 * gamelib.GameUnit(unit.unit_type, game_state.config).health / \
                                                gamelib.GameUnit(unit.unit_type, game_state.config).max_health * \
                                                gamelib.GameUnit(unit.unit_type, game_state.config).cost / game_state.SP
                                    curr_turret_cost_ratios[x][y] = damage_chg / point_chg

                new_turret_cost_ratios = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        new_turret_cost_ratios[x][y] = (gamelib.GameUnit(TURRET, game_state.config).damage_i /
                                                        turret_costs[x][y]) \
                                                       / (game_state.type_cost(TURRET)[0] / game_state.SP)

                upgraded_turret_cost_ratios = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        cost = game_state.type_cost(TURRET, True)
                        for unit in game_state.game_map[[x, y]]:
                            if unit.damage_i + unit.damage_f > 0 and unit.player_index == 1:
                                cost -= game_state.type_cost(TURRET)
                        # Change to upgraded turret damage
                        upgraded_turret_cost_ratios[x][y] = (14 / upgraded_turret_costs[x][y]) / (cost / game_state.SP)

                if np.min(new_turret_cost_ratios) > 1:
                    pos = np.unravel_index(np.argmin(new_turret_cost_ratios), new_turret_cost_ratios.shape)
                    game_state.attempt_spawn(TURRET, [pos])
                elif np.min(upgraded_turret_costs) > 1:
                    pos = np.unravel_index(np.argmin(upgraded_turret_cost_ratios), upgraded_turret_cost_ratios.shape)
                    game_state.attempt_spawn(TURRET, [pos])
                    game_state.attempt_upgrade([pos])
                elif np.min(curr_turret_cost_ratios) < 1:
                    pos = np.unravel_index(np.argmin(curr_turret_cost_ratios), curr_turret_cost_ratios.shape)
                    game_state.attempt_spawn(TURRET, [pos])
                else:
                    return
            else:
                return

    def calc_path_damages(self, game_state):
        enemy_edges = game_state.game_map.get_edge_locations(
            game_state.game_map.TOP_LEFT) + game_state.game_map.get_edge_locations(game_state.game_map.TOP_RIGHT)
        path_costs = []
        turret_hits = np.empty((game_state.game_map.ARENA_SIZE, game_state.game_map.ARENA_SIZE))

        for location in enemy_edges:
            path = game_state.find_path_to_edge(location)
            path_costs.append(0)
            if path is not None:
                for path_location in path:
                    attackers = game_state.get_attackers(path_location, 0)
                    for i in attackers:
                        for unit in game_state.game_map[i]:
                            # Does this account for whether the turret is upgraded?
                            path_costs[-1] += gamelib.GameUnit(unit.unit_type, game_state.config).damage_i * \
                                              gamelib.GameUnit(unit.unit_type, game_state.config).health / \
                                              gamelib.GameUnit(unit.unit_type, game_state.config).max_health
                            turret_hits[i] += gamelib.GameUnit(unit.unit_type, game_state.config).damage_i

        return path_costs, turret_hits
