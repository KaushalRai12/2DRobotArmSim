# Project Name: 2D Robot Arm Simulation with Inverse Kinematics
# Author: Uijin Cho
# Github: https://github.com/jinc77/2DRobotArmSim
# Contact: jinc3930@gmail.com
# License: MIT License
# Description: Simulate 2D Robot Arm movement with linear algebra & inverse kinematics concepts

# License Details:
# This project is licensed under the MIT License – see the LICENSE file for details.

import pygame
import math
import numpy as np

# Initialize Pygame
pygame.init()

# Set up the screen
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption('2D Robot Arm Simulation')

# Define some colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Set up the clock for frame rate control
clock = pygame.time.Clock()

# Define the arm parameters with correct lengths
arm_lengths = [150, 100, 50]  # A-B = 150, B-C = 100, C-D = 50
current_angles = [0, 0, 0]    # Initial angles for the joints
target_angles = [0, 0, 0]     # Target angles
base_position = (400, 400)    # Point A position (will be transformed to appear as 0,0)

# Movement control parameters
learning_rate = 0.3
max_iterations = 50
NUM_POINTS = 21

# Path tracking variables
current_path = []
path_index = 0
is_moving = False

# Add these near the top with other global variables
completed_points = set()  # To track which points we've completed
point_completion_time = 0  # To track when we started waiting at a point
POINT_WAIT_TIME = 5000  # 5 seconds in milliseconds

# Add these with other global variables
previous_angles = [0, 0, 0]  # To track previous angles for delta calculations

def transform_to_screen(x, y):
    """Transform from mathematical coordinates to screen coordinates"""
    return (x + 400, 400 - y)  # Center is (400,400) on screen

def transform_from_screen(x, y):
    """Transform from screen coordinates to mathematical coordinates"""
    return (x - 400, -(y - 400))  # Convert screen coordinates to mathematical

def forward_kinematics(angles, lengths):
    """Calculate joint positions starting from (0,0)"""
    x, y = (0, 0)  # Start at mathematical origin
    joint_positions = [(x, y)]
    
    for i in range(len(angles)):
        x += lengths[i] * math.cos(math.radians(angles[i]))
        y += lengths[i] * math.sin(math.radians(angles[i]))
        joint_positions.append((x, y))
    
    # Transform all points to screen coordinates
    screen_positions = [transform_to_screen(x, y) for x, y in joint_positions]
    return screen_positions

def calculate_angle_differences(current, previous):
    """Calculate absolute angle differences"""
    return [abs(c - p) for c, p in zip(current, previous)]

def calculate_relative_angles(angles):
    """Calculate relative angles between segments"""
    # First angle is relative to Y-axis
    ab_angle = angles[0]
    # Second angle is relative to first segment
    bc_angle = angles[1] - angles[0]
    # Third angle is relative to second segment
    cd_angle = angles[2] - angles[1]
    return ab_angle, bc_angle, cd_angle

def draw_arm(screen, angles, lengths):
    global previous_angles
    joint_positions = forward_kinematics(angles, lengths)
    
    # Calculate relative angles and differences
    ab_angle, bc_angle, cd_angle = calculate_relative_angles(angles)
    angle_diffs = calculate_angle_differences(angles, previous_angles)
    max_angle_diff = max(angle_diffs)
    
    # Draw the path and points if exists
    if current_path:
        # Draw line
        pygame.draw.line(screen, (100, 100, 255), 
                        current_path[0], current_path[-1], 2)
        # Draw dots
        for i, point in enumerate(current_path):
            if i in completed_points:
                color = (0, 255, 0)  # Green for completed points
            elif i == path_index and is_moving:
                color = (255, 0, 0)  # Red for current point
            else:
                color = (0, 0, 255)  # Blue for future points
            pygame.draw.circle(screen, color, point, 4)
    
    # Draw arm segments
    for i in range(len(joint_positions) - 1):
        pygame.draw.line(screen, BLACK, joint_positions[i], joint_positions[i + 1], 5)
        pygame.draw.circle(screen, RED, joint_positions[i], 8)
    
    # Draw end effector (point D)
    pygame.draw.circle(screen, RED, joint_positions[-1], 8)
    
    # Display metrics
    font = pygame.font.SysFont(None, 24)
    y_offset = 20
    
    # 1. Display dot numbers
    if current_path:
        for i, point in enumerate(current_path):
            num_text = font.render(str(i+1), True, BLACK)
            screen.blit(num_text, (point[0] + 10, point[1] - 10))
    
    # Display positions and angles
    metrics = [
        f"B pos: ({int(joint_positions[1][0]-400)}, {int(400-joint_positions[1][1])})",
        f"A-B ∡: {int(ab_angle)}°",
        f"Δ A-B ∡: {int(angle_diffs[0])}°",
        f"C pos: ({int(joint_positions[2][0]-400)}, {int(400-joint_positions[2][1])})",
        f"B-C ∡: {int(bc_angle)}°",
        f"Δ B-C ∡: {int(angle_diffs[1])}°",
        f"D pos: ({int(joint_positions[3][0]-400)}, {int(400-joint_positions[3][1])})",
        f"C-D ∡: {int(cd_angle)}°",
        f"Δ C-D ∡: {int(angle_diffs[2])}°",
        f"Max Δ ∡: {int(max_angle_diff)}°"
    ]
    
    for i, metric in enumerate(metrics):
        text = font.render(metric, True, BLACK)
        screen.blit(text, (20, y_offset + i * 30))
    
    # Update previous angles for next frame
    previous_angles = angles.copy()

def calculate_path(start, end, num_points):
    """Calculate equally spaced points along the line"""
    path = []
    for i in range(num_points):
        t = i / (num_points - 1)
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        path.append((int(x), int(y)))
    return path

def inverse_kinematics(target_pos, current_angles, lengths, max_iter, learning_rate):
    """
    Calculate inverse kinematics using gradient descent
    target_pos: (x, y) target position in mathematical coordinates
    current_angles: list of current joint angles
    lengths: list of arm segment lengths
    max_iter: maximum iterations for optimization
    learning_rate: step size for gradient descent
    """
    angles = current_angles.copy()
    
    for _ in range(max_iter):
        # Calculate current end effector position
        x, y = 0, 0
        for i in range(len(angles)):
            x += lengths[i] * math.cos(math.radians(angles[i]))
            y += lengths[i] * math.sin(math.radians(angles[i]))
        
        # Calculate error
        error_x = target_pos[0] - x
        error_y = target_pos[1] - y
        error = math.sqrt(error_x**2 + error_y**2)
        
        if error < 1:  # If close enough to target
            break
        
        # Calculate gradients for each angle
        for i in range(len(angles)):
            # Calculate partial derivatives
            dx, dy = 0, 0
            for j in range(i, len(angles)):
                dx -= lengths[j] * math.sin(math.radians(angles[j]))
                dy += lengths[j] * math.cos(math.radians(angles[j]))
            
            # Update angle using gradient descent
            gradient = (error_x * dx + error_y * dy)
            angles[i] += learning_rate * gradient
            
            # Normalize angles to -180 to 180 degrees
            angles[i] = ((angles[i] + 180) % 360) - 180
    
    return angles

# Main loop
running = True
while running:
    screen.fill(WHITE)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if not is_moving:
                # Reset completed points
                completed_points.clear()
                point_completion_time = 0
                # Get current end effector position
                current_pos = forward_kinematics(current_angles, arm_lengths)[-1]
                clicked_pos = pygame.mouse.get_pos()
                
                # Calculate path with 21 points
                current_path = calculate_path(current_pos, clicked_pos, NUM_POINTS)
                path_index = 0
                is_moving = True
                target_position = current_path[0]

    if is_moving:
        target_position = current_path[path_index]
        current_pos = forward_kinematics(current_angles, arm_lengths)[-1]
        
        # Calculate distance to current target point
        distance = math.sqrt((current_pos[0] - target_position[0])**2 + 
                           (current_pos[1] - target_position[1])**2)
        
        if distance < 1:  # When reached current point
            current_time = pygame.time.get_ticks()
            
            # If we just arrived at this point
            if path_index not in completed_points and point_completion_time == 0:
                point_completion_time = current_time
            
            # Check if we've waited long enough
            if current_time - point_completion_time >= POINT_WAIT_TIME:
                completed_points.add(path_index)
                point_completion_time = 0  # Reset timer
                path_index += 1
                
                if path_index >= len(current_path):
                    is_moving = False
                    # Keep the last point green
                    completed_points.add(len(current_path) - 1)
                else:
                    target_position = current_path[path_index]
        else:
            # Reset timer if we're not at the target point
            point_completion_time = 0
        
        # Convert target position to mathematical coordinates for IK
        target_math = transform_from_screen(*target_position)
        target_angles = inverse_kinematics(target_math, list(current_angles), 
                                         arm_lengths, max_iterations, learning_rate)
        
        # Update angles directly
        current_angles = target_angles

    # Draw the arm
    draw_arm(screen, current_angles, arm_lengths)
    
    # Update previous_angles if not moving
    if not is_moving:
        previous_angles = current_angles.copy()
    
    pygame.display.flip()
    clock.tick(30)

pygame.quit()

