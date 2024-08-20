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
        self.curr_SP = 0

    def turret_opt(self, game_state, TURRET, SP):
        self.curr_game_state = copy.deepcopy(game_state)
        self.curr_SP = self.curr_game_state.get_resource(SP)
        arena_size = self.curr_game_state.game_map.ARENA_SIZE
        half_arena = self.curr_game_state.game_map.HALF_ARENA

        # UPGRADE_TURRET_RANGE = 0
        # for unit in self.curr_game_state.config["unitInformation"]:
        #     if unit.get('attackRange', 0) >= UPGRADE_TURRET_RANGE:
        #         UPGRADE_TURRET_RANGE = unit.get('attackRange', 0)
        # gamelib.debug_write(UPGRADE_TURRET_RANGE)

        self.curr_game_state.game_map.add_unit(TURRET, [11, 16])
        upgraded_unit = self.curr_game_state.game_map[11, 16][0]
        upgraded_unit.upgrade()
        UPGRADE_TURRET_RANGE = upgraded_unit.attackRange
        UPGRADE_TURRET_DAMAGE = upgraded_unit.damage_i
        UPGRADE_TURRET_MAX_HEALTH = upgraded_unit.max_health
        UPGRADE_TURRET_COST = self.curr_game_state.type_cost(TURRET, upgrade=True)[SP] - self.curr_game_state.type_cost(TURRET)[SP]
        self.curr_game_state.game_map.remove_unit([11, 16])
        while True:
            if self.curr_SP > 0:
                gamelib.debug_write('again')
                path_costs, turret_hits = self.calc_path_damages()

                point_costs, path_counts = self.calc_point_costs(path_costs)

                gamelib.debug_write(point_costs[13][0])
                gamelib.debug_write(turret_hits[13][0])
                gamelib.debug_write(path_counts[13][0])

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
                            circle = self.curr_game_state.game_map.get_locations_in_range([x, y], UPGRADE_TURRET_RANGE)
                            for loc in circle:
                                if loc[1] < half_arena:
                                    upgraded_turret_costs[x][y] += point_costs[loc[0]][loc[1]]
                                    upgraded_turret_counts[x][y] += 1
                upgraded_turret_costs = np.divide(upgraded_turret_costs, upgraded_turret_counts,
                                                  out=np.zeros_like(upgraded_turret_costs, dtype=float),
                                                  where=upgraded_turret_counts != 0)

                curr_turret_cost_ratios = np.full((arena_size, half_arena), np.inf)
                curr_turret_points = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if self.curr_game_state.game_map.in_arena_bounds([x, y]):
                            for unit in self.curr_game_state.game_map[[x, y]]:
                                if unit.damage_i > 0 and unit.player_index == 0:

                                    damage_chg = float('inf')
                                    if turret_costs[x][y] != 0:
                                        damage_chg = (turret_hits[x][y] / turret_counts[x][y]) / turret_costs[x][y]
                                        point_chg = 0.75 * unit.health / unit.max_health * \
                                                    self.curr_game_state.type_cost(
                                                        unit.unit_type, upgrade=unit.upgraded)[SP] / self.curr_SP
                                        curr_turret_points[x][y] = 0.75 * unit.health / unit.max_health * \
                                                                   self.curr_game_state.type_cost(
                                                                       unit.unit_type, upgrade=unit.upgraded)[0]
                                        curr_turret_cost_ratios[x][y] = damage_chg / point_chg

                                    if unit.upgraded:
                                        if upgraded_turret_costs[x][y] != 0:
                                            damage_chg = (turret_hits[x][y] / upgraded_turret_counts[x][y]) / \
                                                         upgraded_turret_costs[x][y]
                                            point_chg = 0.75 * unit.health / UPGRADE_TURRET_MAX_HEALTH * \
                                                        self.curr_game_state.type_cost(
                                                            unit.unit_type, upgrade=unit.upgraded)[SP] / self.curr_SP
                                            curr_turret_points[x][y] = 0.75 * unit.health / UPGRADE_TURRET_MAX_HEALTH * \
                                                                       self.curr_game_state.type_cost(
                                                                           unit.unit_type, upgrade=unit.upgraded)[0]
                                            curr_turret_cost_ratios[x][y] = damage_chg / point_chg

                new_turret_cost_ratios = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if turret_costs[x][y] != 0:
                            new_turret_cost_ratios[x][y] = (gamelib.GameUnit(TURRET,
                                                                             self.curr_game_state.config).damage_i /
                                                            turret_costs[x][y]) \
                                                           / (self.curr_game_state.type_cost(TURRET)[
                                                                  0] / self.curr_SP)

                upgraded_turret_cost_ratios = np.zeros((arena_size, half_arena))
                upgraded_turret_points = np.zeros((arena_size, half_arena))
                for y in range(half_arena):
                    for x in range(arena_size):
                        if self.curr_game_state.game_map.in_arena_bounds([x, y]):
                            cost = self.curr_game_state.type_cost(TURRET, upgrade=True)[0]
                            for unit in self.curr_game_state.game_map[[x, y]]:
                                if unit.damage_i > 0 and unit.player_index == 0:
                                    cost -= self.curr_game_state.type_cost(TURRET)[0]
                                if unit.upgraded:
                                    cost = 0
                                    # cost -= 3
                            # Change to upgraded turret damage
                            if upgraded_turret_costs[x][y] != 0 and cost != 0:
                                upgraded_turret_cost_ratios[x][y] = (14 / upgraded_turret_costs[x][y]) / (
                                        cost / self.curr_SP)
                                upgraded_turret_points[x][y] = cost

                gamelib.debug_write('ratios')
                gamelib.debug_write(np.max(upgraded_turret_cost_ratios))
                gamelib.debug_write(np.max(new_turret_cost_ratios))
                gamelib.debug_write(np.min(curr_turret_cost_ratios))
                upgraded_max_turret_pos = np.array(np.unravel_index(np.argmax(upgraded_turret_cost_ratios),
                                                                    upgraded_turret_cost_ratios.shape)).reshape(1,
                                                                                                                -1).tolist()
                new_max_turret_pos = np.array(np.unravel_index(np.argmax(upgraded_turret_cost_ratios),
                                                               upgraded_turret_cost_ratios.shape)).reshape(1,
                                                                                                           -1).tolist()
                curr_max_turret_pos = np.array(
                    np.unravel_index(np.argmin(curr_turret_cost_ratios), curr_turret_cost_ratios.shape)).reshape(1,
                                                                                                                 -1).tolist()
                if np.max(upgraded_turret_cost_ratios) > 1 and (
                        not self.curr_game_state.contains_stationary_unit(upgraded_max_turret_pos[0]) or not self.curr_game_state.game_map[upgraded_max_turret_pos[0]][
                    0].upgraded):
                    gamelib.debug_write('upgrade')
                    gamelib.debug_write(upgraded_max_turret_pos)
                    if not (game_state.attempt_spawn(TURRET, upgraded_max_turret_pos) and
                            game_state.attempt_upgrade(upgraded_max_turret_pos)):
                        return
                    self.curr_game_state.game_map.add_unit(TURRET, upgraded_max_turret_pos[0])
                    self.curr_game_state.game_map[upgraded_max_turret_pos[0]][0].upgrade()
                    self.curr_SP -= upgraded_turret_points[upgraded_max_turret_pos[0][0]][upgraded_max_turret_pos[0][1]]
                elif np.max(new_turret_cost_ratios) > 1 and self.curr_game_state.can_spawn(TURRET,
                                                                                           new_max_turret_pos[0]):
                    gamelib.debug_write('add new')
                    gamelib.debug_write(new_max_turret_pos)
                    game_state.attempt_spawn(TURRET, new_max_turret_pos)
                    self.curr_game_state.game_map.add_unit(TURRET, new_max_turret_pos[0])
                    self.curr_SP -= self.curr_game_state.type_cost(TURRET)
                elif np.min(curr_turret_cost_ratios) < 0.1:
                    gamelib.debug_write('remove')
                    gamelib.debug_write(curr_max_turret_pos)
                    game_state.attempt_remove(curr_max_turret_pos)
                    self.curr_game_state.game_map.remove_unit(curr_max_turret_pos[0])
                    self.curr_SP += curr_turret_points[curr_max_turret_pos[0][0]][curr_max_turret_pos[0][1]]
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
            cost = 0
            if path is not None:
                for path_location in path:
                    for unit in self.curr_game_state.get_attackers(path_location, 0):
                        # Does this account for whether the turret is upgraded?
                        cost += unit.damage_i * unit.health / unit.max_health
                        # if path_location == [13, 4]:
                        #     gamelib.debug_write([unit.x, unit.y])
                        # if unit.x == 13 and unit.y == 0:
                        #     print('found it')
                        turret_hits[unit.x][unit.y] += unit.damage_i
            path_costs.append(cost)

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
        # for i in self.curr_game_state.game_map.get_locations_in_range([13, 0], 4.5):
        #     if point_costs[i[0]][i[1]] != 0:
        #         gamelib.debug_write(i)
        #         gamelib.debug_write(point_costs[i[0]][i[1]])

        point_costs = np.divide(point_costs, path_counts, out=np.zeros_like(point_costs, dtype=float),
                                where=path_counts != 0)

        return point_costs, path_counts
