import numpy as np
import time

#Initializing global variable stuck_cycles to limit the number of cycles the Rover can be stuck for
#before initiating the unstuck manuver.
stuck_cycles = 0

def steer_and_throttle(Rover):
    # For the rover to steer and throttle as set below.
    Rover.throttle = Rover.throttle_set
    Rover.brake = 0
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    # The offset is to make the mean shift more towards the right of the rover(negative)
    #to crawl along the right wall.

def only_steer_no_throttle(Rover):
    # For the rover to coast (no throttle) and steer only when max vel has been reached.
    Rover.throttle = 0
    Rover.brake = 0
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    # The offset is to make the mean shift more towards the right of the rover(negative)
    #to crawl along the right wall.

def no_steer_only_throttle(Rover):
    # For when the rover is in reverse.
    Rover.throttle = Rover.throttle_set
    Rover.brake = 0
    Rover.steer = 0

def brake(Rover):
    #For braking.
    Rover.throttle = 0
    Rover.brake = Rover.brake_set
    Rover.steer = 0

def turn(Rover,angle):
    #For turning to find a better path.
    Rover.throttle = 0
    # Release the brake to allow turning
    Rover.brake = 0
    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    Rover.steer = angle # Could be more clever here about which way to turn

def unstuck(Rover):
    global stuck_cycles
    # Incrementing stuck_cycle counts.
    stuck_cycles = stuck_cycles + 1
    # If it has been stuck for over 50 cycles.
    # Which is enough to check that the rover is actually stuck
    #and not just starting from rest or from a reverse.
    if stuck_cycles > 50:
        initial = time.clock()
        #Reverse throttle for 1 sec.
        while time.clock() < initial + 0.5:
            turn(Rover,15)
    # If stuck cycles not yet reached.
    # This is for when the rover could not be stuck but starting from rest.
    else:
        steer_and_throttle(Rover)

def decision_step(Rover):
    global stuck_cycles
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max

                if Rover.vel < Rover.max_vel:
                    # Setting condition for when the Rover gets stuck.
                    if -0.1 < Rover.vel < 0.1 and Rover.throttle == 0.2:
                        unstuck(Rover)

                    # For setting the steer to zero when the rover is still in reverse.
                    elif Rover.vel <= -0.1:
                        # Restting stuck_cycle counter.
                        stuck_cycles = 0
                        no_steer_only_throttle(Rover)

                    # If Rover.vel is between 0.1 and 1 which is Rover.max_vel.
                    else:
                        stuck_cycles = 0
                        steer_and_throttle(Rover)

                else: # Else coast when Rover.vel >= Rover.max_vel
                    only_steer_no_throttle(Rover)

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    brake(Rover)
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                brake(Rover)

            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    # Turning to find a better path.
                    turn(Rover,15)

                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value, rel the brake and steer to mean angle.
                    steer_and_throttle(Rover)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0  
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    return Rover

