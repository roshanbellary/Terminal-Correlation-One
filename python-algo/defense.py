import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json
import copy


class Defense(gamelib.AlgoCore):

    def __init__(self):
        self.curr_game_state = None

    def turret_opt(self, game_state, TURRET, SP):
        self.curr_game_state = copy.deepcopy(game_state)
        arena_size = self.curr_game_state.game_map.ARENA_SIZE
        half_arena = self.curr_game_state.game_map.HALF_ARENA
        while True:
            if self.curr_game_state.get_resource(SP) > 0:
                gamelib.debug_write('again')
                path_costs, turret_hits = self.calc_path_damages()

                point_costs, path_counts = self.calc_point_costs(path_costs)

                turret_costs = np.zeros((arena_size, half_arena))
                turret_counts = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if self.curr_game_state.game_map.in_arena_bounds([x, y]):
                            circle = self.curr_game_state.game_map.get_locations_in_range([x, y],
                                                                                          gamelib.GameUnit(TURRET,
                                                                                                           self.curr_game_state.config).attackRange)
                            for loc in circle:
                                if loc[1] < half_arena:
                                    turret_costs[x][y] += point_costs[loc[0]][loc[1]]
                                    turret_counts[x][y] += 1
                turret_costs = np.divide(turret_costs, turret_counts, out=np.zeros_like(turret_costs, dtype=float),
                                         where=turret_counts != 0)

                upgraded_turret_costs = np.zeros((arena_size, half_arena))
                upgraded_turret_counts = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if self.curr_game_state.game_map.in_arena_bounds([x, y]):
                            # Change to upgraded turret range
                            circle = self.curr_game_state.game_map.get_locations_in_range([x, y], 5)
                            for loc in circle:
                                if loc[1] < half_arena:
                                    upgraded_turret_costs[x][y] += point_costs[loc[0]][loc[1]]
                                    upgraded_turret_counts[x][y] += 1
                upgraded_turret_costs = np.divide(upgraded_turret_costs, upgraded_turret_counts,
                                                  out=np.zeros_like(upgraded_turret_costs, dtype=float),
                                                  where=upgraded_turret_counts != 0)

                curr_turret_cost_ratios = np.full((arena_size, half_arena), np.inf)
                curr_turret_costs = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if self.curr_game_state.game_map.in_arena_bounds([x, y]):
                            for unit in self.curr_game_state.game_map[[x, y]]:
                                if unit.damage_i + unit.damage_f > 0 and unit.player_index == 0:
                                    if unit.upgraded:
                                        # gamelib.debug_write([x, y])
                                        # gamelib.debug_write(turret_hits[x][y])
                                        # gamelib.debug_write(upgraded_turret_counts[x][y])
                                        # gamelib.debug_write(upgraded_turret_costs[x][y])
                                        damage_chg = (turret_hits[x][y] / upgraded_turret_counts[x][y]) / \
                                                     upgraded_turret_costs[x][y]
                                    else:
                                        # gamelib.debug_write([x, y])
                                        # gamelib.debug_write(turret_hits[x][y])
                                        # gamelib.debug_write(turret_counts[x][y])
                                        # gamelib.debug_write(turret_costs[x][y])
                                        damage_chg = (turret_hits[x][y] / turret_counts[x][y]) / turret_costs[x][y]
                                    point_chg = 0.75 * unit.health / unit.max_health * self.curr_game_state.type_cost(
                                        unit.unit_type, upgrade=unit.upgraded)[0] / self.curr_game_state.get_resource(
                                        SP)
                                    curr_turret_cost_ratios[x][y] = damage_chg / point_chg
                                    curr_turret_costs[x][y] = 0.75 * unit.health / unit.max_health * \
                                                              self.curr_game_state.type_cost(
                                                                  unit.unit_type, upgrade=unit.upgraded)[0]

                new_turret_cost_ratios = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if turret_costs[x][y] != 0:
                            new_turret_cost_ratios[x][y] = (gamelib.GameUnit(TURRET,
                                                                             self.curr_game_state.config).damage_i /
                                                            turret_costs[x][y]) \
                                                           / (self.curr_game_state.type_cost(TURRET)[
                                                                  0] / self.curr_game_state.get_resource(SP))

                upgraded_turret_cost_ratios = np.zeros((arena_size, half_arena))
                upgraded_turret_costs = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if self.curr_game_state.game_map.in_arena_bounds([x, y]):
                            cost = self.curr_game_state.type_cost(TURRET, upgrade=True)[0]
                            for unit in self.curr_game_state.game_map[[x, y]]:
                                if unit.damage_i + unit.damage_f > 0 and unit.player_index == 0:
                                    # cost -= self.curr_game_state.type_cost(TURRET)[0]
                                    cost -= 3
                            # Change to upgraded turret damage
                            if upgraded_turret_costs[x][y] != 0:
                                upgraded_turret_cost_ratios[x][y] = (14 / upgraded_turret_costs[x][y]) / (
                                        cost / self.curr_game_state.get_resource(SP))
                                upgraded_turret_costs[x][y] = cost

                gamelib.debug_write('ratios')
                gamelib.debug_write(np.max(upgraded_turret_cost_ratios))
                gamelib.debug_write(np.max(new_turret_cost_ratios))
                gamelib.debug_write(np.min(curr_turret_cost_ratios))
                if np.max(upgraded_turret_cost_ratios) > 1:
                    pos = np.unravel_index(np.argmax(upgraded_turret_cost_ratios), upgraded_turret_cost_ratios.shape)
                    gamelib.debug_write(np.array(pos).reshape(1, -1).tolist())
                    game_state.attempt_spawn(TURRET, pos)
                    game_state.attempt_upgrade(pos)
                    self.curr_game_state.game_map.add_unit(TURRET, pos[0])
                    self.curr_game_state.game_map[pos][0].upgrade()
                    # self.curr_game_state.__set_resource(SP, 0 - upgraded_turret_costs[pos[0]])
                elif np.max(new_turret_cost_ratios) > 1:
                    pos = np.unravel_index(np.argmax(new_turret_cost_ratios), new_turret_cost_ratios.shape)
                    gamelib.debug_write(np.array(pos).reshape(1, -1).tolist())
                    game_state.attempt_spawn(TURRET, np.array(pos).reshape(1, -1).tolist())
                    self.curr_game_state.game_map.add_unit(TURRET, np.array(pos).reshape(1, -1).tolist()[0])
                    # self.curr_game_state.__set_resource(SP, 0 - self.curr_game_state.type_cost(TURRET))
                elif np.min(curr_turret_cost_ratios) < 0.1:
                    pos = np.unravel_index(np.argmin(curr_turret_cost_ratios), curr_turret_cost_ratios.shape)
                    game_state.attempt_remove(np.array(pos).reshape(1, -1).tolist())
                    self.curr_game_state.game_map.remove_unit(np.array(pos).reshape(1, -1).tolist()[0])
                    # self.curr_game_state.__set_resource(SP, 0 - curr_turret_costs[pos[0]][pos[1]])
                    gamelib.debug_write(self.curr_game_state.get_resource(SP))
                else:
                    return
            else:
                return

    def calc_path_damages(self):
        enemy_edges = self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_LEFT) + \
                      self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_RIGHT)
        path_costs = []
        turret_hits = np.zeros((self.curr_game_state.game_map.ARENA_SIZE, self.curr_game_state.game_map.HALF_ARENA))

        for location in enemy_edges:
            path = self.curr_game_state.find_path_to_edge(location)
            path_costs.append(0)
            if path is not None:
                for path_location in path:
                    for unit in self.curr_game_state.get_attackers(path_location, 1):
                        # Does this account for whether the turret is upgraded?
                        path_costs[-1] += unit.damage_i * unit.health / unit.max_health
                        if unit.x == 15 and unit.y == 2:
                            print('found it')
                        turret_hits[unit.x][unit.y] += unit.damage_i
        return path_costs, turret_hits

    def calc_point_costs(self, path_costs):
        enemy_edges = self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_LEFT) + \
                      self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_RIGHT)
        point_costs = np.zeros((self.curr_game_state.game_map.ARENA_SIZE, self.curr_game_state.game_map.HALF_ARENA))
        path_counts = np.zeros((self.curr_game_state.game_map.ARENA_SIZE, self.curr_game_state.game_map.HALF_ARENA))
        for loc in range(len(enemy_edges)):
            path = self.curr_game_state.find_path_to_edge(enemy_edges[loc])
            if path is not None:
                for i in path:
                    if i[1] < self.curr_game_state.game_map.HALF_ARENA:
                        point_costs[i[0]][i[1]] += path_costs[loc]
                        path_counts[i[0]][i[1]] += 1
        point_costs = np.divide(point_costs, path_counts, out=np.zeros_like(point_costs, dtype=float),
                                where=path_counts != 0)

        return point_costs, path_counts
