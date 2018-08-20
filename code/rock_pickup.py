import numpy as np
import math

# This function is based on 
# https://github.com/jojobilly/Rover-Simulation

def yaw_to_target(Rover):
    if (len(Rover.rock_target_pos) != 2):
        return Rover.rock_target_yaw
    
    direction = np.array(Rover.rock_target_pos) - np.array(Rover.pos)
    norm = math.sqrt(direction[0] * direction[0] + direction[1] * direction[1])
    if (norm > 0):
        direction /= norm
    
    if (direction[0] > 0):
        target_yaw = math.atan(direction[1]/direction[0])
    elif (direction[0] < 0):
        target_yaw = math.atan(direction[1]/direction[0]) + np.pi
    elif direction[1] > 0:
        target_yaw = np.pi / 2
    else:
        target_yaw = -np.pi / 2
        
    if target_yaw < 0:
        target_yaw += np.pi * 2
    
    return target_yaw * 180/np.pi

def rock_pickup(Rover):
    # There could be no rocks visible from this position. But if we saw it 
    # before we need to continue rotating to the target angle.
    if (len(Rover.rock_ang) == 0):
        Rover.no_rock_counter += 1
        
        # In case we have not seen the rock for too long - give up trying. 
        if Rover.no_rock_counter > 500:
            Rover.rock_pickup_flag = False
            Rover.no_rock_counter = 0
            
        # We have reached the target angle. Stop rotating and approach slowly.
        if np.abs(Rover.yaw - yaw_to_target(Rover)) < 10:
            Rover.steer = 0
            Rover.throttle = .5
        else:
            Rover.brake = 0
            # The rock might be not visible now. But keep rotating to the
            # previously selected direction 
            if np.abs(Rover.steer) < 3:
                Rover.steer = -4
        return Rover
    else: 
        # We see the rock, so set a flag which would result in calling this 
        # function on the next iteration.
        Rover.rock_pickup_flag = True
        Rover.no_rock_counter = 0
        Rover.rock_target_yaw = (int)(Rover.yaw + np.mean(Rover.rock_ang * 180/np.pi) + 360) % 360
        Rover.rock_target_pos = (np.array(Rover.pos) + abs(Rover.rock_dist.min()) * 
            np.array((
                math.cos(Rover.rock_target_yaw / 180 * np.pi), 
                math.sin(Rover.rock_target_yaw / 180 * np.pi)))).tolist()


    rock_min_ang = abs(Rover.rock_ang.min())  
    rock_min_dist = abs(Rover.rock_dist.min())
    

    # If the rock is close then decrease speed
    if rock_min_dist > Rover.rock_stop_forward:  
        if Rover.vel > .8:
            Rover.throttle = 0
            Rover.steer = np.clip(np.mean(Rover.rock_ang * 180/np.pi), -15, 15)
        if Rover.vel < .8:
            Rover.throttle = .5
            Rover.brake = 0
            Rover.steer = np.clip(np.mean(Rover.rock_ang * 180/np.pi), -15, 15)

    # The rock is very close, stop
    if rock_min_dist < Rover.rock_stop_forward:  
        Rover.throttle = 0
        if rock_min_ang >.7 and rock_min_dist < 60:
            if Rover.vel > 0: 
                Rover.brake = .1
     
    if Rover.vel == 0:
        Rover.throttle = 0
        Rover.brake = 0
        if (Rover.rock_ang.min()) > 0:
            Rover.steer = 4
        if (Rover.rock_ang.min()) < 0:
            Rover.steer = -4
            
    if abs(Rover.rock_ang.min()) < .2:  #When Rover is lined up with rock increase speed pickup rock
        Rover.steer = np.clip(np.mean(Rover.rock_ang * 180/np.pi), -15, 15)
        Rover.throttle = .5
        Rover.brake = 0

    #Apply brake when Rover is near sample
    if Rover.near_sample:  
        Rover.throttle = 0
        Rover.brake = 1
        if Rover.near_sample and Rover.vel == 0:  #When stopped in front of rock send pick up command
            Rover.send_pickup = 'True'
            Rover.reverse = 'True'
            Rover.rock_pickup_flag = False
        else:
            Rover.send_pickup = 'False'
            Rover.brake = 0
    return Rover
