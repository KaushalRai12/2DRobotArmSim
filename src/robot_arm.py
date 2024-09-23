import pygame
import math
import numpy as np

# Initialize Pygame
pygame.init()

# Set up the screen
screen = pygame.display.set_mode((800, 800))
pygame.display.set_caption('Optimized 2D Robot Arm Simulation')

# Define some colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)

# Set up the clock for frame rate control
clock = pygame.time.Clock()

# Define the arm parameters (3 segments)
arm_lengths = [150, 100, 75]  # Lengths of the arm segments
current_angles = [0, 0, 0]    # Initial angles for the joints (current state)
target_angles = [0, 0, 0]     # Target angles for smoother interpolation
base_position = (400, 400)     # Base of the robot arm (center of screen)

# Inverse kinematics parameters
target_position = [400, 400]   # Mouse-controlled target
max_iterations = 30            # Increased maximum iterations for faster IK convergence

# Movement parameters
learning_rate = 0.2            # Increased learning rate for faster movement
smooth_factor = 0.2            # Increased smooth factor for quicker angle transitions

# Function to calculate forward kinematics
def forward_kinematics(angles, lengths):
    x, y = base_position
    joint_positions = [(x, y)]
    
    for i in range(len(angles)):
        x += lengths[i] * math.cos(math.radians(angles[i]))
        y += lengths[i] * math.sin(math.radians(angles[i]))
        joint_positions.append((x, y))
        
    return joint_positions

# Function to draw the robotic arm
def draw_arm(screen, angles, lengths):
    joint_positions = forward_kinematics(angles, lengths)
    
    # Draw base
    pygame.draw.circle(screen, BLUE, base_position, 10)
    
    # Draw arm segments
    for i in range(len(joint_positions) - 1):
        pygame.draw.line(screen, BLACK, joint_positions[i], joint_positions[i + 1], 5)
        pygame.draw.circle(screen, RED, (int(joint_positions[i + 1][0]), int(joint_positions[i + 1][1])), 10)
    
    # Draw target position
    pygame.draw.circle(screen, BLUE, (int(target_position[0]), int(target_position[1])), 10)

    # Display end-effector position and angles
    end_effector = joint_positions[-1]
    font = pygame.font.SysFont(None, 24)
    text = font.render(f'End Effector: ({int(end_effector[0])}, {int(end_effector[1])})', True, BLACK)
    screen.blit(text, (20, 20))
    
    for i, angle in enumerate(angles):
        text_angle = font.render(f'Joint {i + 1} Angle: {int(angle)}°', True, BLACK)
        screen.blit(text_angle, (20, 50 + i * 30))

# Inverse kinematics using Jacobian Transpose
def inverse_kinematics(target, angles, lengths, max_iterations=10, learning_rate=0.1):
    for _ in range(max_iterations):
        # Calculate current end-effector position using forward kinematics
        joint_positions = forward_kinematics(angles, lengths)
        end_effector = joint_positions[-1]
        
        # Calculate error (distance to target)
        error = np.array([target[0] - end_effector[0], target[1] - end_effector[1]])
        if np.linalg.norm(error) < 1:  # Stop if the error is small enough
            break

        # Calculate partial derivatives (Jacobian transpose method)
        for i in range(len(angles) - 1, -1, -1):  # Iterate backward from end-effector
            joint_position = joint_positions[i]
            to_end_effector = np.array([end_effector[0] - joint_position[0], end_effector[1] - joint_position[1]])
            to_target = np.array([target[0] - joint_position[0], target[1] - joint_position[1]])
            
            # Compute the cross product (perpendicular vector)
            cross_product = np.cross(np.append(to_end_effector, 0), np.append(to_target, 0))[2]
            angles[i] += learning_rate * cross_product

        # Normalize angles to keep them between 0 and 360 degrees
        angles = [angle % 360 for angle in angles]
    
    return angles

# Main loop
running = True
while running:
    screen.fill(WHITE)

    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Mouse control for target position
    if pygame.mouse.get_pressed()[0]:  # Left mouse button pressed
        target_position = list(pygame.mouse.get_pos())

    # Apply inverse kinematics to calculate target angles with increased iterations
    target_angles = inverse_kinematics(target_position, target_angles, arm_lengths, max_iterations=max_iterations, learning_rate=learning_rate)

    # Faster interpolation between current and target angles for quicker movement
    for i in range(len(current_angles)):
        current_angles[i] += smooth_factor * (target_angles[i] - current_angles[i])

    # Draw the robotic arm with optimized angles
    draw_arm(screen, current_angles, arm_lengths)

    # Update the display
    pygame.display.flip()

    # Frame rate
    clock.tick(60)  # 60 FPS for smooth rendering

# Quit Pygame
pygame.quit()

