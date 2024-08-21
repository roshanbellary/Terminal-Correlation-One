import numpy as np
import gamelib
import random
import math
import warnings
from sys import maxsize
import json
import copy
import time


def go_to_target_edge(game_map, loc, target_edge):
    before_path = [loc.copy()]
    new_path = [loc.copy()]
    direction = []
    if target_edge == game_map.TOP_LEFT:
        direction = [[0, 1], [-1, 0]] # Move up, then left
        direction_set = 0
    else:
        direction = [[0, 1], [1, 0]] # Move up, then right
        direction_set = 1
    while loc not in game_map.get_edge_locations(target_edge):
        # Update location by adding the current direction
        loc = [loc[0] + direction[direction_set][0], loc[1] + direction[direction_set][1]]
        # Ensure loc remains within the game bounds
        if not game_map.in_arena_bounds(loc) and loc not in game_map.get_edge_locations(target_edge):
            # Revert the location change if out of bounds
            loc = [loc[0] - direction[direction_set][0], loc[1] - direction[direction_set][1]]
            direction_set = (direction_set + 1) % 2 # Try the other direction
        elif loc != new_path[-1]:
            new_path.append(loc.copy()) # Append a copy of the current location
            # Ensure the final location is added to the path
        if new_path[-1] != loc:
            new_path.append(loc.copy())
    gamelib.debug_write(new_path)
    return new_path


class Defense(gamelib.AlgoCore):

    def __init__(self):
        self.curr_game_state = None
        self.curr_SP = None

    def turret_opt(self, game_state, curr_sp, scored_on_locations, elapsed_time, TURRET, WALL, SP):
        start_time = time.time()
        self.curr_game_state = game_state
        # self.curr_game_state = copy.deepcopy(game_state)
        # self.curr_SP = self.curr_game_state.get_resource(SP)
        self.curr_SP = curr_sp
        gamelib.debug_write(self.curr_SP)
        arena_size = self.curr_game_state.game_map.ARENA_SIZE
        half_arena = self.curr_game_state.game_map.HALF_ARENA

        self.curr_game_state.game_map.add_unit(TURRET, [11, 16])
        upgraded_unit = self.curr_game_state.game_map[11, 16][0]
        upgraded_unit.upgrade()
        UPGRADE_TURRET_RANGE = upgraded_unit.attackRange
        UPGRADE_TURRET_DAMAGE = upgraded_unit.damage_i
        UPGRADE_TURRET_MAX_HEALTH = upgraded_unit.max_health
        self.curr_game_state.game_map.remove_unit([11, 16])

        while True:
            if self.curr_SP > 0 and time.time() - start_time < 8 - elapsed_time:
                gamelib.debug_write('again')
                path_costs, turret_hits, cost_ratio_multipliers = self.calc_path_damages(scored_on_locations)

                point_costs, path_counts = self.calc_point_costs(path_costs)

                valid_positions = np.array([[self.curr_game_state.game_map.in_arena_bounds([x, y])
                                             for x in range(arena_size)]
                                            for y in range(half_arena - 1)])

                try:
                    turret_costs = np.zeros((arena_size, half_arena))
                    turret_counts = np.zeros((arena_size, half_arena))
                    for y in range(half_arena - 1):
                        for x in range(arena_size):
                            try:
                                if valid_positions[y, x]:
                                    circle = self.curr_game_state.game_map.get_locations_in_range([x, y],
                                                                                                  gamelib.GameUnit(
                                                                                                      TURRET,
                                                                                                      self.curr_game_state.config).attackRange)
                                    for loc in circle:
                                        if loc[1] < half_arena:
                                            turret_costs[x][y] += point_costs[loc[0]][loc[1]]
                                            turret_counts[x][y] += 1
                            except Exception as e:
                                gamelib.debug_write(f"Error processing turret at ({x}, {y}): {e}")

                    try:
                        turret_costs = np.divide(turret_costs, turret_counts,
                                                 out=np.zeros_like(turret_costs, dtype=float), where=turret_counts != 0)
                    except Exception as e:
                        gamelib.debug_write(f"Error dividing turret costs: {e}")

                    upgraded_turret_costs = np.zeros((arena_size, half_arena))
                    upgraded_turret_counts = np.zeros((arena_size, half_arena))
                    for y in range(half_arena - 1):
                        for x in range(arena_size):
                            try:
                                if valid_positions[y, x]:
                                    circle = self.curr_game_state.game_map.get_locations_in_range([x, y],
                                                                                                  UPGRADE_TURRET_RANGE)
                                    for loc in circle:
                                        if loc[1] < half_arena:
                                            upgraded_turret_costs[x][y] += point_costs[loc[0]][loc[1]]
                                            upgraded_turret_counts[x][y] += 1
                            except Exception as e:
                                gamelib.debug_write(f"Error processing upgraded turret at ({x}, {y}): {e}")

                    try:
                        upgraded_turret_costs = np.divide(upgraded_turret_costs, upgraded_turret_counts,
                                                          out=np.zeros_like(upgraded_turret_costs, dtype=float),
                                                          where=upgraded_turret_counts != 0)
                    except Exception as e:
                        gamelib.debug_write(f"Error dividing upgraded turret costs: {e}")

                    # curr_turret_cost_ratios = np.full((arena_size, half_arena), 100000000)
                    # curr_turret_points = np.zeros((arena_size, half_arena))
                    # for y in range(half_arena):
                    #     for x in range(arena_size):
                    #         try:
                    #             if self.curr_game_state.game_map.in_arena_bounds([x, y]):
                    #                 for unit in self.curr_game_state.game_map[[x, y]]:
                    #                     if unit.damage_i > 0 and unit.player_index == 0:
                    #                         if turret_costs[x][y] != 0:
                    #                             damage_chg = (turret_hits[x][y] / turret_counts[x][y]) / \
                    #                                          turret_costs[x][y]
                    #                             point_chg = 0.75 * unit.health / unit.max_health * \
                    #                                         self.curr_game_state.type_cost(unit.unit_type,
                    #                                                                        upgrade=unit.upgraded)[
                    #                                             SP] / self.curr_SP
                    #                             curr_turret_points[x][y] = 0.75 * unit.health / unit.max_health * \
                    #                                                        self.curr_game_state.type_cost(
                    #                                                            unit.unit_type, upgrade=unit.upgraded)[0]
                    #                             curr_turret_cost_ratios[x][y] = damage_chg / point_chg
                    #
                    #                         if unit.upgraded and upgraded_turret_costs[x][y] != 0:
                    #                             damage_chg = (turret_hits[x][y] / upgraded_turret_counts[x][y]) / \
                    #                                          upgraded_turret_costs[x][y]
                    #                             point_chg = 0.75 * unit.health / UPGRADE_TURRET_MAX_HEALTH * \
                    #                                         self.curr_game_state.type_cost(unit.unit_type,
                    #                                                                        upgrade=unit.upgraded)[
                    #                                             SP] / self.curr_SP
                    #                             curr_turret_points[x][
                    #                                 y] = 0.75 * unit.health / UPGRADE_TURRET_MAX_HEALTH * \
                    #                                      self.curr_game_state.type_cost(unit.unit_type,
                    #                                                                     upgrade=unit.upgraded)[0]
                    #                             curr_turret_cost_ratios[x][y] = damage_chg / point_chg
                    #         except Exception as e:
                    #             gamelib.debug_write(f"Error processing turret cost ratios at ({x}, {y}): {e}")

                    # cost_ratio_multipliers = self.get_cost_ratios_multipliers(scored_on_locations)
                    try:
                        new_turret_cost_ratios = np.zeros((arena_size, half_arena))
                        for y in range(half_arena - 1):
                            for x in range(arena_size):
                                if valid_positions[y, x]:
                                    if turret_costs[x][y] != 0:
                                        if self.can_spawn(TURRET, [x, y]):
                                            new_turret_cost_ratios[x][y] = (gamelib.GameUnit(TURRET,
                                                                                             self.curr_game_state.config).damage_i /
                                                                            turret_costs[x][y]) / \
                                                                           (self.curr_game_state.type_cost(TURRET)[
                                                                                SP] / self.curr_SP)
                                    else:
                                        new_turret_cost_ratios[x][y] = 0
                        new_turret_cost_ratios = cost_ratio_multipliers * new_turret_cost_ratios
                    except Exception as e:
                        gamelib.debug_write(f"Error calculating new turret cost ratios: {e}")

                    try:
                        upgraded_turret_cost_ratios = np.zeros((arena_size, half_arena))
                        upgraded_turret_points = np.zeros((arena_size, half_arena))
                        for y in range(half_arena - 1):
                            for x in range(arena_size):
                                if valid_positions[y, x]:
                                    cost = self.curr_game_state.type_cost(TURRET, upgrade=True)[SP] + \
                                           self.curr_game_state.type_cost(TURRET)[SP]
                                    for unit in self.curr_game_state.game_map[[x, y]]:
                                        if unit.damage_i > 0 and unit.player_index == 0:
                                            cost -= self.curr_game_state.type_cost(TURRET)[0]
                                        if unit.upgraded:
                                            cost = 0
                                    if upgraded_turret_costs[x][y] != 0:
                                        if cost != 0:
                                            upgraded_turret_cost_ratios[x][y] = (UPGRADE_TURRET_DAMAGE /
                                                                                 upgraded_turret_costs[x][y]) / \
                                                                                (cost / self.curr_SP) * 2
                                            upgraded_turret_points[x][y] = cost
                                    else:
                                        upgraded_turret_cost_ratios[x][y] = 0
                        upgraded_turret_cost_ratios = cost_ratio_multipliers * upgraded_turret_cost_ratios
                    except Exception as e:
                        gamelib.debug_write(f"Error calculating upgraded turret cost ratios: {e}")

                    wall_cost_ratios = np.zeros((arena_size, half_arena))
                    for y in range(half_arena - 1):
                        for x in range(arena_size):
                            if turret_hits[x][y] != 0 and y > 10:
                                if self.can_spawn_wall(WALL, [x, y + 1], SP):
                                    wall_cost_ratios[x][y + 1] = pow(np.abs(half_arena - np.abs(half_arena - x)) / half_arena, 2) * \
                                                                 pow(y / half_arena, 2) * \
                                                                 gamelib.GameUnit(WALL,
                                                                                  self.curr_game_state.config).max_health * \
                                                                 self.curr_game_state.game_map[x][y][0].health / \
                                                                 self.curr_game_state.game_map[x][y].max_health / (
                                                                         4 / self.curr_SP)

                except Exception as e:
                    gamelib.debug_write(f"Error in main turret cost calculation: {e}")

                upgraded_max_turret_pos = np.array(np.unravel_index(np.argmax(upgraded_turret_cost_ratios),
                                                                    upgraded_turret_cost_ratios.shape)).reshape(1,
                                                                                                                -1).tolist()
                while np.count_nonzero(upgraded_turret_cost_ratios) > 0 and np.max(
                        upgraded_turret_cost_ratios) > 0.05 and \
                        not self.can_spawn_upgraded_turret(upgraded_max_turret_pos[0], TURRET, SP):
                    upgraded_turret_cost_ratios[upgraded_max_turret_pos[0][0]][upgraded_max_turret_pos[0][1]] = 0
                    upgraded_max_turret_pos = np.array(np.unravel_index(np.argmax(upgraded_turret_cost_ratios),
                                                                        upgraded_turret_cost_ratios.shape)).reshape(1,
                                                                                                                    -1).tolist()

                new_max_turret_pos = np.array(np.unravel_index(np.argmax(new_turret_cost_ratios),
                                                               new_turret_cost_ratios.shape)).reshape(1, -1).tolist()
                while np.count_nonzero(new_turret_cost_ratios) > 0 and np.max(
                        new_turret_cost_ratios) > 0.05 and not self.can_spawn(TURRET, new_max_turret_pos[0]):
                    new_turret_cost_ratios[new_max_turret_pos[0][0]][new_max_turret_pos[0][1]] = 0
                    new_max_turret_pos = np.array(np.unravel_index(np.argmax(new_turret_cost_ratios),
                                                                   new_turret_cost_ratios.shape)).reshape(1,
                                                                                                          -1).tolist()

                max_wall_pos = np.array(np.unravel_index(np.argmax(wall_cost_ratios),
                                                         wall_cost_ratios.shape)).reshape(1, -1).tolist()
                while np.count_nonzero(wall_cost_ratios) > 0 and np.max(
                        wall_cost_ratios) > 0.05 and not self.can_spawn_wall(WALL, max_wall_pos[0], SP):
                    wall_cost_ratios[max_wall_pos[0][0]][max_wall_pos[0][1]] = 0
                    max_wall_pos = np.array(np.unravel_index(np.argmax(wall_cost_ratios),
                                                             wall_cost_ratios.shape)).reshape(1, -1).tolist()

                #
                # curr_max_turret_pos = np.array(
                #     np.unravel_index(np.argmin(curr_turret_cost_ratios), curr_turret_cost_ratios.shape)).reshape(1,
                #                                                                                                  -1).tolist()

                gamelib.debug_write('ratios')
                gamelib.debug_write(np.max(upgraded_turret_cost_ratios))
                gamelib.debug_write(np.max(new_turret_cost_ratios))
                # gamelib.debug_write(np.min(curr_turret_cost_ratios))
                gamelib.debug_write(np.max(wall_cost_ratios))

                max_cost = np.max([np.max(new_turret_cost_ratios),
                                   np.max(upgraded_turret_cost_ratios), np.max(wall_cost_ratios)])

                if np.max(new_turret_cost_ratios) > 0.05 and np.max(
                        new_turret_cost_ratios) == max_cost and self.can_spawn(TURRET, new_max_turret_pos[0]):
                    gamelib.debug_write('add new')
                    gamelib.debug_write(new_max_turret_pos)
                    game_state.attempt_spawn(TURRET, new_max_turret_pos)
                    # self.curr_game_state.attempt_spawn(TURRET, new_max_turret_pos)
                    self.curr_SP -= self.curr_game_state.type_cost(TURRET)[SP]
                elif np.max(upgraded_turret_cost_ratios) > 0.05 and np.max(
                        upgraded_turret_cost_ratios) == max_cost and self.can_spawn_upgraded_turret(
                    upgraded_max_turret_pos[0], TURRET, SP):
                    gamelib.debug_write('upgrade')
                    gamelib.debug_write(upgraded_max_turret_pos)
                    game_state.attempt_spawn(TURRET, upgraded_max_turret_pos)
                    game_state.attempt_upgrade(upgraded_max_turret_pos)
                    # self.curr_game_state.attempt_spawn(TURRET, upgraded_max_turret_pos)
                    # self.curr_game_state.attempt_upgrade(upgraded_max_turret_pos)
                    self.curr_SP -= upgraded_turret_points[upgraded_max_turret_pos[0][0]][upgraded_max_turret_pos[0][1]]
                elif np.max(wall_cost_ratios) > 0.05 and True and self.can_spawn_wall(WALL, max_wall_pos[0], SP):
                    gamelib.debug_write('wall')
                    gamelib.debug_write(max_wall_pos)
                    if self.curr_game_state.attempt_spawn(WALL, max_wall_pos):
                        self.curr_game_state.attempt_upgrade(max_wall_pos)
                        self.curr_SP -= self.curr_game_state.type_cost(WALL, upgrade=True)[SP]
                    else:
                        self.curr_SP -= self.curr_game_state.type_cost(WALL)[SP]
                    # gamelib.debug_write(self.curr_game_state.game_map[max_wall_pos[0]][0].health)
                # elif np.min(curr_turret_cost_ratios) < 0.1:
                #     gamelib.debug_write('remove')
                #     gamelib.debug_write(curr_max_turret_pos)
                #     game_state.attempt_remove(curr_max_turret_pos)
                #     self.curr_game_state.game_map.remove_unit(curr_max_turret_pos[0])
                #     self.curr_SP += curr_turret_points[curr_max_turret_pos[0][0]][curr_max_turret_pos[0][1]]
                else:
                    gamelib.debug_write('Defense Time: ' + str(time.time() - start_time))
                    return
            else:
                gamelib.debug_write('Defense Time: ' + str(time.time() - start_time))
                return

    def can_spawn_upgraded_turret(self, loc, TURRET, SP):
        can_spawn = self.curr_game_state.can_spawn(TURRET, loc) and \
                    self.curr_SP > self.curr_game_state.type_cost(TURRET, upgrade=True)[SP] + \
                    self.curr_game_state.type_cost(TURRET)[SP]
        can_upgrade = self.curr_game_state.contains_stationary_unit(loc) and (
                not self.curr_game_state.game_map[loc][0].upgraded and
                self.curr_SP > self.curr_game_state.type_cost(TURRET, upgrade=True)[SP])
        return can_spawn or can_upgrade

    def can_spawn(self, unit_type, location):
        affordable = self.curr_SP >= self.curr_game_state.type_cost(unit_type)[0]
        blocked = self.curr_game_state.contains_stationary_unit(location) or (
                len(self.curr_game_state.game_map[location[0], location[1]]) > 0)
        correct_territory = location[1] < self.curr_game_state.HALF_ARENA
        return affordable and correct_territory and not blocked

    def can_spawn_wall(self, WALL, location, SP):
        can_upgrade = self.curr_game_state.contains_stationary_unit(location) and (
                not self.curr_game_state.game_map[location][0].upgraded and
                self.curr_SP > self.curr_game_state.type_cost(WALL, upgrade=True)[SP])
        return self.can_spawn(WALL, location) or can_upgrade

    def calc_path_damages(self, scored_locations):
        multipliers = np.full((self.curr_game_state.game_map.ARENA_SIZE, self.curr_game_state.game_map.HALF_ARENA), 1)
        for x in range(self.curr_game_state.game_map.ARENA_SIZE):
            for y in range(self.curr_game_state.game_map.HALF_ARENA):
                if y > self.curr_game_state.game_map.HALF_ARENA / 2:
                    multipliers[x][y] *= 10
                # multipliers[x][y] += y * np.abs(self.curr_game_state.game_map.HALF_ARENA - x)
                # multipliers[x][y] *= y

        enemy_edges = self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_LEFT) + \
                      self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_RIGHT)
        friendly_edges = self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.BOTTOM_LEFT) + \
                         self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.BOTTOM_RIGHT)
        path_costs = []
        turret_hits = np.zeros((self.curr_game_state.game_map.ARENA_SIZE, self.curr_game_state.game_map.HALF_ARENA))

        for location in enemy_edges:
            path = self.curr_game_state.find_path_to_edge(location)
            # path = go_to_target_edge(self.curr_game_state.game_map, location,
            #                               self.curr_game_state.game_map.TOP_RIGHT if location[0] < self.curr_game_state.game_map.HALF_ARENA else self.curr_game_state.game_map.TOP_LEFT)
            cost = 0
            if path is not None and path[-1] in friendly_edges:
                if len(scored_locations) > 0:
                    mult = pow(10 * np.sum(np.all(np.array(scored_locations) == path[-1], axis=1)), 2)
                for path_location in path:
                    for unit in self.curr_game_state.get_attackers(path_location, 0):
                        loc_cost = unit.damage_i * pow(unit.health / unit.max_health, 2)
                        cost += loc_cost
                        turret_hits[unit.x][unit.y] += unit.damage_i
                        if len(scored_locations) > 0 and path_location[1] < self.curr_game_state.game_map.HALF_ARENA:
                            multipliers[path_location[0]][path_location[1]] += mult
            path_costs.append(cost)

        multipliers = multipliers / np.max(multipliers)
        multipliers = multipliers * self.curr_game_state.get_resources(player_index=1)[1]

        return path_costs, turret_hits, multipliers

    def calc_point_costs(self, path_costs):
        enemy_edges = self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_LEFT) + \
                      self.curr_game_state.game_map.get_edge_locations(self.curr_game_state.game_map.TOP_RIGHT)
        point_costs = np.zeros((self.curr_game_state.game_map.ARENA_SIZE, self.curr_game_state.game_map.HALF_ARENA))
        path_counts = np.zeros((self.curr_game_state.game_map.ARENA_SIZE, self.curr_game_state.game_map.HALF_ARENA))
        for loc in range(len(enemy_edges)):
            path = self.curr_game_state.find_path_to_edge(enemy_edges[loc])
            if path is not None:
                # for i in path:
                #     if i[1] < self.curr_game_state.game_map.HALF_ARENA:
                #         point_costs[i[0]][i[1]] += path_costs[loc]
                #         path_counts[i[0]][i[1]] += 1
                valid_indices = [(i[0], i[1]) for i in path if i[1] < self.curr_game_state.game_map.HALF_ARENA]
                valid_indices = np.array(valid_indices)
                if len(valid_indices) > 0:
                    point_costs[valid_indices[:, 0], valid_indices[:, 1]] += path_costs[loc]
                    path_counts[valid_indices[:, 0], valid_indices[:, 1]] += 1

        point_costs = np.divide(point_costs, path_counts, out=np.zeros_like(point_costs, dtype=float),
                                where=path_counts != 0)

        return point_costs, path_counts

