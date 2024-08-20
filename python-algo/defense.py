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
    point_costs = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
    path_counts = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
    for loc in range(len(enemy_edges)):
        path = game_state.find_path_to_edge(enemy_edges[loc])
        if path is not None:
            for i in path:
                if i[1] < game_state.game_map.HALF_ARENA:
                    point_costs[i[0]][i[1]] += path_costs[loc]
                    path_counts[i[0]][i[1]] += 1
    point_costs = np.divide(point_costs, path_counts, out=np.zeros_like(point_costs, dtype=float),
                            where=path_counts != 0)

    return point_costs, path_counts


class Defense(gamelib.AlgoCore):

    def __init__(self, algo_strategy):
        self.algo_strategy = algo_strategy

    def turret_opt(self, game_state, TURRET):
        while True:
            gamelib.debug_write('again')
            if game_state.get_resource(0) > 0:
                path_costs, turret_hits = self.calc_path_damages(game_state)

                point_costs, path_counts = calc_point_costs(game_state, path_costs)
                turret_hits = np.divide(turret_hits, path_counts, out=np.zeros_like(turret_hits, dtype=float),
                                        where=path_counts != 0)

                turret_costs = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                turret_counts = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        if game_state.game_map.in_arena_bounds([x, y]):
                            circle = game_state.game_map.get_locations_in_range([x, y],
                                                                                gamelib.GameUnit(TURRET,
                                                                                                 game_state.config).attackRange)
                            for loc in circle:
                                if loc[1] < game_state.game_map.HALF_ARENA:
                                    turret_costs[x][y] += point_costs[loc[0]][loc[1]]
                                    turret_counts[x][y] += 1
                turret_costs = np.divide(turret_costs, turret_counts, out=np.zeros_like(turret_costs, dtype=float),
                                         where=turret_counts != 0)

                upgraded_turret_costs = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                upgraded_turret_counts = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        if game_state.game_map.in_arena_bounds([x, y]):
                            # Change to upgraded turret range
                            circle = game_state.game_map.get_locations_in_range([x, y], 5)
                            for loc in circle:
                                if loc[1] < game_state.game_map.HALF_ARENA:
                                    upgraded_turret_costs[x][y] += point_costs[loc[0]][loc[1]]
                                    upgraded_turret_counts[x][y] += 1
                upgraded_turret_costs = np.divide(upgraded_turret_costs, upgraded_turret_counts,
                                                  out=np.zeros_like(upgraded_turret_costs, dtype=float),
                                                  where=upgraded_turret_counts != 0)

                curr_turret_cost_ratios = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
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
                                                gamelib.GameUnit(unit.unit_type,
                                                                 game_state.config).cost / game_state.get_resource(0)
                                    curr_turret_cost_ratios[x][y] = damage_chg / point_chg

                new_turret_cost_ratios = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        if turret_costs[x][y] != 0:
                            new_turret_cost_ratios[x][y] = (gamelib.GameUnit(TURRET, game_state.config).damage_i /
                                                            turret_costs[x][y]) \
                                                           / (game_state.type_cost(TURRET)[0] / game_state.get_resource(
                                0))

                upgraded_turret_cost_ratios = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))
                for y in range(game_state.game_map.HALF_ARENA):
                    for x in range(game_state.game_map.ARENA_SIZE):
                        cost = game_state.type_cost(TURRET, True)[0]
                        if game_state.game_map.in_arena_bounds([x, y]):
                            for unit in game_state.game_map[[x, y]]:
                                if unit.damage_i + unit.damage_f > 0 and unit.player_index == 1:
                                    cost -= game_state.type_cost(TURRET)
                            # Change to upgraded turret damage
                            upgraded_turret_cost_ratios[x][y] = (14 / upgraded_turret_costs[x][y]) / (
                                    cost / game_state.get_resource(0))

                gamelib.debug_write('ratios')
                gamelib.debug_write(np.max(upgraded_turret_cost_ratios))
                gamelib.debug_write(np.max(new_turret_cost_ratios))
                gamelib.debug_write(np.min(curr_turret_cost_ratios))
                if np.max(upgraded_turret_cost_ratios) > 0.1:
                    pos = np.unravel_index(np.argmin(upgraded_turret_cost_ratios), upgraded_turret_cost_ratios.shape)
                    game_state.attempt_spawn(TURRET, [pos])
                    game_state.attempt_upgrade([pos])
                elif np.max(new_turret_cost_ratios) > 0.1:
                    pos = np.unravel_index(np.argmin(new_turret_cost_ratios), new_turret_cost_ratios.shape)
                    game_state.attempt_spawn(TURRET, [pos])
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
        turret_hits = np.zeros((game_state.game_map.ARENA_SIZE, game_state.game_map.HALF_ARENA))

        for location in enemy_edges:
            path = game_state.find_path_to_edge(location)
            path_costs.append(0)
            if path is not None:
                for path_location in path:
                    attackers = game_state.get_attackers(path_location, 1)
                    for unit in attackers:
                        # Does this account for whether the turret is upgraded?
                        path_costs[-1] += gamelib.GameUnit(unit.unit_type, game_state.config).damage_i * \
                                          gamelib.GameUnit(unit.unit_type, game_state.config).health / \
                                          gamelib.GameUnit(unit.unit_type, game_state.config).max_health
                        turret_hits[unit.x][unit.y] += gamelib.GameUnit(unit.unit_type, game_state.config).damage_i

        # gamelib.debug_write(turret_hits)
        return path_costs, turret_hits
