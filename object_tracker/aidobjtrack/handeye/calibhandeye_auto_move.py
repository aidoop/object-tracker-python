from enum import Enum


class HandEyeAutoMove:
    STAGE_GONEXT = 1
    STAGE_CAPTURE = 2

    def __init__(self):
        self.initialized = False
        self.automove_on = False

        self.trans_move_x = 0.05
        self.trans_move_y = 0.05
        self.trans_move_z = 0.05
        self.rot_x = 10.0
        self.rot_y = 10.0
        self.rot_z = 10.0
        self.total_move = 70

        self.stage = self.STAGE_GONEXT

        self.auto_move_rotation_poses = [
            [0.0, 0.0, 0.0, self.rot_x, 0.0, 0.0],
            [0.0, 0.0, 0.0, self.rot_x * (-2.0), 0.0, 0.0],
            [0.0, 0.0, 0.0, self.rot_x, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, self.rot_y, 0.0],
            [0.0, 0.0, 0.0, 0.0, self.rot_y * (-2.0), 0.0],
            [0.0, 0.0, 0.0, 0.0, self.rot_y, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, self.rot_z],
            [0.0, 0.0, 0.0, 0.0, 0.0, self.rot_z * (-2.0)],
            [0.0, 0.0, 0.0, 0.0, 0.0, self.rot_z],
        ]

        self.auto_move_translation_poses = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [self.trans_move_x, 0.0, 0.0, 0.0, 0.0, 0.0],
            [self.trans_move_x * (-1.0), self.trans_move_y, 0.0, 0.0, 0.0, 0.0],
            [
                self.trans_move_x * (-1.0),
                self.trans_move_y * (-1.0),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            [self.trans_move_x, self.trans_move_y * (-1.0), 0.0, 0.0, 0.0, 0.0],
            [0.0, self.trans_move_y, self.trans_move_z * (-1.0), 0.0, 0.0, 0.0],
            [0.0, 0.0, self.trans_move_z, 0.0, 0.0, 0.0],
        ]

        # handeye position sequence table
        self.auto_move_rel_poses = []
        self.curr_list_index = -1

    def initialize(
        self,
        trans_move_x=0.05,
        trans_move_y=0.05,
        trans_move_z=0.05,
        rot_x=10.0,
        rot_y=10.0,
        rot_z=10.0,
        total_move=70,
    ):
        self.trans_move_x = trans_move_x
        self.trans_move_y = trans_move_y
        self.trans_move_z = trans_move_z
        self.rot_x = rot_x
        self.rot_y = rot_z
        self.rot_y = rot_z
        self.total_move = total_move

        for trans_pose in self.auto_move_translation_poses:
            self.auto_move_rel_poses += [trans_pose] + self.auto_move_rotation_poses

        self.initialized = True

    def get_move_list(self):
        return self.auto_move_rel_poses

    def get_next(self):
        self.curr_list_index += 1
        if self.curr_list_index >= min(len(self.auto_move_rel_poses), self.total_move):
            self.reset()
            return []

        next_move = self.auto_move_rel_poses[self.curr_list_index]
        return next_move

    def start(self):
        self.automove_on = True

    def stop(self):
        self.automove_on = False

    def reset(self):
        self.automove_on = False
        self.curr_list_index = -1

    def isStarted(self):
        return self.automove_on

    def isInitialized(self):
        return self.initialized

    def get_stage(self):
        return self.stage

    def set_stage(self, stage):
        self.stage = stage
