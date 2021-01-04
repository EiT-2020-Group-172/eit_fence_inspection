import os
import cv2
import sys
import random
import numpy as np 
import matplotlib.pyplot as plt 

from scipy import ndimage
from sklearn.cluster import AgglomerativeClustering

class BreachDetection:
    def __init__(self):
        self.original_img = None
        self.binary_img   = None
        self.init_pointer = True
        self.img_height   = 0
        self.img_width    = 0

        # Next move on line
        self.pointer        = [0,0]
        self.new_pointers   = []
        self.new_moves      = []
        self.peaks          = []
        self.end_points     = []
        self.way_to_fence   = []
        self.line_dis_thres = 1

        # Variables for  line follower
        self.line_thres = 2
        self.step_size  = [1,2,3]
        self.move_dir   = [1,2,3,0]

        # Valid moves [up_right down_right down_left up_left]
        self.valid_moves = [[3,1],[0,2],[1,3],[2,0]] 

        # Variables for peak search
        self.peak_thres      = 5
        self.peak_dist_thres = 8
        self.peak_step_size  = [4,5,6,7]
        self.peak_move_dir   = [[0,1,0], [1,2,1],[2,3,2],[3,0,3]]
        self.peak_search_dir = [[3,0,1],[0,1,2],[1,2,3],[2,3,0]]
        self.peak_indeices   = [[0,1],[-1,1],[0,1]]

        # Init fence search
        self.init_fence_follower = True

    def peak_search(self, pointer, pre_move):
        print("Number of branches: " + str(self.number_of_branches))

        pointer = self.new_pointers[0]
        self.new_pointers.pop(0)
        pre_move = self.new_moves[0]
        self.new_moves.pop(0)

        while True:
            # Return if image dimensions exceeds
            if self.image_dimenions_exceeds(self.img_height, self.img_width, 300, 20, pointer):
                break

            # Check if pointer has reached a texel joint by checking if neighboors has a
            # pixel intensity above a predefined threshold
            peak_found   = True
            new_branches = []
            new_moves = []

            for (move_dir, search_dir, index) in zip(self.peak_move_direction[pre_move], self.peak_search_direction[pre_move], self.prek_indicies):
                found, brach_pointer = self.fence_connection_search(pointer, move_dir, search_dir, self.prek_thres, index[0], index[1], self.peak_step_size)
                if not found:
                    peak_found = False
                    break
                else:
                    new_branches.append(brach_pointer)
                    new_moves.append(search_dir)
            
            # By now the texel joints has been detected.
            if peak_found:
                new_peak_found = True
                for peak in self.peaks:
                    delta_x = pointer[0] - peak[0]
                    delta_y = pointer[1] - peak[1]
                    dist = np.sqrt(delta_x*delta_x+delta_y*delta_y)
                    if dist < self.peak_dist_thres:
                        new_peak_found = False
                        break
                if new_peak_found:
                    peak_x = sum([pixel[0] for pixel in new_branches]) / len(new_branches)
                    peak_y = sum([pixel[1] for pixel in new_branches]) / len(new_branches)
                    pointer = [peak_x, peak_y]
                    self.peaks.append(pointer)
                    for (new_branch, new_move) in zip(new_branches, new_moves)
                        self.number_of_branches += 1
                        self.new_pointers.append(new_branch)
                        self.new_moves.append(new_move)
                break

            # Search in pre move direction. If black pixel values are found, move one pixel 
            # value to the side and try again. This continues until end point is reached.
            end_reached = True
            found, pointer = self.fence_connection_search(pointer, self.line_move_direction[pre_move], pre_move, self.line_thres, -1, 2, self.step_size)
            if found:
                end_reached = False
            
            if end_reached:
                self.end_points.append(pointer)
                break
        self.number_of_branches -= 1
    
    def next_moves(self, pointer, step_size):
        # Update possible directions to move
        up_left    = [pointer[0] - step_size, pointer[1] - step_size]
        down_right = [pointer[0] + step_size, pointer[1] - step_size]
        down_left  = [pointer[0] - step_size, pointer[1] + step_size]
        up_right   = [pointer[0] + step_size, pointer[1] + step_size]

        # Place directions in config vector
        config = [up_right, down_right, down_left, up_left]

        return config

    def fence_connection_search(self, pointer, move_dir, search_dir, search_dir, thres, start_index, end_index, step_size):
        best_move  = self.next_moves(pointer, 0)
        best_thres = 0

        for index in  range(start_index, end_index):
            for step in step_size:
                moves = self.new_moves(pointer, index)
                moves = self.new_moves(moves[move_dir], step)
                x = moves[search_dir][0]
                x = moves[search_dir][1]
                if abs(self.binary_img[y,x]) >= best_thres:
                    best_thres = self.binary_img[y,x]
                    best_move = moves[search_dir]

        if best_thres >= thres:
            return True, best_move
        
        # No connection found 
        return False, pointer

    def move_pointer_to_fence(self, pointer, pre_move):
        while self.binary_img[pointer[1], pointer[0]] < self.line_thres:
            config = self.next_moves(pointer, 1)
            pointer = config[pre_move]
            self.way_to_fence.append(pointer)

        for move in self.valid_moves[pre_move]:
            found, pointer = self.fence_connection_search(pointer, pre_move, move, self.line_thres, 0, 1, self.step_size)
            if found:
                return pointer, move

        return pointer, pre_move

if __name__ == "__main__":
    branching = BreachDetection()
        